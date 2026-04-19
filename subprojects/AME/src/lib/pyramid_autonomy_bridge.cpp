#include "ame/pyramid_autonomy_bridge.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace ame {

namespace model = pyramid::data_model;
namespace common = pyramid::data_model::common;

namespace {

template <typename T>
std::vector<T> readStore(const std::unordered_map<std::string, T>& store,
                         const PyramidAutonomyBridge::Query& query) {
  std::vector<T> result;
  if (query.id.empty()) {
    result.reserve(store.size());
    for (const auto& [id, value] : store) {
      result.push_back(value);
    }
    return result;
  }

  for (const auto& id : query.id) {
    const auto it = store.find(id);
    if (it != store.end()) {
      result.push_back(it->second);
    }
  }
  return result;
}

template <typename T>
void eraseReadItems(std::unordered_map<std::string, T>& store,
                    const PyramidAutonomyBridge::Query& query) {
  if (!query.one_shot.value_or(false)) {
    return;
  }
  if (query.id.empty()) {
    store.clear();
    return;
  }
  for (const auto& id : query.id) {
    store.erase(id);
  }
}

}  // namespace

PyramidAutonomyBridge::PyramidAutonomyBridge(
    WorldModel& world_model,
    const ActionRegistry& action_registry,
    const Planner& planner,
    const PlanCompiler& plan_compiler)
    : world_model_(world_model),
      action_registry_(action_registry),
      planner_(planner),
      plan_compiler_(plan_compiler) {}

std::vector<PyramidAutonomyBridge::Capabilities>
PyramidAutonomyBridge::handleReadCapabilities(const Query& request) {
  Capabilities capabilities;
  capabilities.id = "ame.current_stack";
  capabilities.backend_id = "ame.current_stack";
  capabilities.supports_plan_only = true;
  capabilities.supports_plan_and_execute = false;
  capabilities.supports_execute_approved_plan = false;
  capabilities.supports_replanning = true;
  capabilities.supports_typed_component_requirement_placement = true;
  capabilities.supports_state_update_ingress = true;
  if (!request.id.empty() && !queryIncludes(request, capabilities.id)) {
    return {};
  }
  return {capabilities};
}

PyramidAutonomyBridge::Identifier PyramidAutonomyBridge::handleCreateRequirement(
    const PlanningExecutionRequirement& request) {
  auto requirement = request;
  auto requirement_id = requirement.base.id;
  if (requirement_id.empty()) {
    requirement_id = nextId("ame-req");
    requirement.base.id = requirement_id;
  }

  requirement.status.status = common::Progress::InProgress;
  requirements_[requirement_id] = requirement;

  for (const auto& agent : requirement.available_agents) {
    world_model_.registerAgent(agent.agent_id, agent.agent_type);
    if (auto* wm_agent = world_model_.getAgent(agent.agent_id)) {
      wm_agent->available = agent.available;
    }
  }

  std::vector<std::string> goals;
  for (const auto& goal : requirement.goal) {
    if (goal.expression.has_value()) {
      goals.push_back(goal.expression.value());
    }
  }
  world_model_.setGoal(goals);

  auto plan = makePlan(requirement, requirement_id);
  plans_[plan.id] = plan;

  auto run = makeRun(requirement, plan);
  runs_[run.id] = run;

  for (const auto& step : plan.step) {
    auto placement = makePlacement(requirement_id, plan, step);
    placements_[placement.id] = placement;
  }

  return requirement_id;
}

std::vector<PyramidAutonomyBridge::PlanningExecutionRequirement>
PyramidAutonomyBridge::handleReadRequirement(const Query& request) {
  return readStore(requirements_, request);
}

PyramidAutonomyBridge::Ack PyramidAutonomyBridge::handleUpdateRequirement(
    const PlanningExecutionRequirement& request) {
  if (request.base.id.empty()) {
    return model::kAckFail;
  }
  auto it = requirements_.find(request.base.id);
  if (it == requirements_.end()) {
    return model::kAckFail;
  }
  it->second = request;
  return model::kAckOk;
}

PyramidAutonomyBridge::Ack PyramidAutonomyBridge::handleDeleteRequirement(
    const Identifier& request) {
  auto erased = requirements_.erase(request);
  for (auto& [id, run] : runs_) {
    if (run.planning_execution_requirement_id == request) {
      run.state = model::PlanningExecutionState::Cancelled;
      run.achievement.status = common::Progress::Cancelled;
    }
  }
  return erased > 0 ? model::kAckOk : model::kAckFail;
}

PyramidAutonomyBridge::Identifier PyramidAutonomyBridge::handleCreateState(
    const StateUpdate& request) {
  auto update = request;
  if (update.id.empty()) {
    update.id = nextId("ame-state");
  }
  applyStateUpdate(update);
  return update.id;
}

PyramidAutonomyBridge::Ack PyramidAutonomyBridge::handleUpdateState(
    const StateUpdate& request) {
  applyStateUpdate(request);
  return model::kAckOk;
}

PyramidAutonomyBridge::Ack PyramidAutonomyBridge::handleDeleteState(
    const Identifier& request) {
  return request.empty() ? model::kAckFail : model::kAckOk;
}

std::vector<PyramidAutonomyBridge::Plan>
PyramidAutonomyBridge::handleReadPlan(const Query& request) {
  auto result = readStore(plans_, request);
  eraseReadItems(plans_, request);
  return result;
}

std::vector<PyramidAutonomyBridge::ExecutionRun>
PyramidAutonomyBridge::handleReadRun(const Query& request) {
  auto result = readStore(runs_, request);
  eraseReadItems(runs_, request);
  return result;
}

std::vector<PyramidAutonomyBridge::RequirementPlacement>
PyramidAutonomyBridge::handleReadPlacement(const Query& request) {
  auto result = readStore(placements_, request);
  eraseReadItems(placements_, request);
  return result;
}

FactAuthority PyramidAutonomyBridge::toWorldModelAuthority(
    model::FactAuthorityLevel authority) {
  switch (authority) {
    case model::FactAuthorityLevel::Believed:
      return FactAuthority::BELIEVED;
    case model::FactAuthorityLevel::Confirmed:
      return FactAuthority::CONFIRMED;
    case model::FactAuthorityLevel::Unspecified:
      return FactAuthority::CONFIRMED;
  }
  return FactAuthority::CONFIRMED;
}

std::string PyramidAutonomyBridge::actionName(const std::string& signature) {
  const auto paren = signature.find('(');
  return paren == std::string::npos ? signature : signature.substr(0, paren);
}

bool PyramidAutonomyBridge::queryIncludes(const Query& query,
                                          const std::string& id) {
  return std::find(query.id.begin(), query.id.end(), id) != query.id.end();
}

std::string PyramidAutonomyBridge::nextId(const std::string& prefix) {
  return prefix + "-" + std::to_string(++next_id_);
}

PyramidAutonomyBridge::Plan PyramidAutonomyBridge::makePlan(
    const PlanningExecutionRequirement&,
    const std::string& requirement_id) {
  auto result = planner_.solve(world_model_);

  Plan plan;
  plan.id = nextId("ame-plan");
  plan.planning_execution_requirement_id = requirement_id;
  plan.backend_id = "ame.current_stack";
  plan.world_version = world_model_.version();
  plan.replan_count = 0;
  plan.plan_success = result.success;
  plan.solve_time_ms = result.solve_time_ms;

  for (size_t i = 0; i < result.steps.size(); ++i) {
    const auto& step_result = result.steps[i];
    const auto& action = world_model_.groundActions()[step_result.action_index];

    model::PlanStep step;
    step.id = plan.id + "/step/" + std::to_string(i + 1);
    step.sequence_number = static_cast<uint32_t>(i + 1);
    step.action_name = actionName(action.signature);
    step.signature = action.signature;

    model::PlannedComponentInteraction interaction;
    interaction.operation =
        model::RequirementPlacementOperation::CreateRequirement;
    if (step.action_name == "search") {
      interaction.target_component = "tactical_objects";
      interaction.target_service = "Object_Of_Interest_Service";
      interaction.target_type =
          "pyramid.data_model.tactical.ObjectInterestRequirement";
    } else {
      interaction.target_component = "ame.unmapped";
      interaction.target_service = step.action_name;
      interaction.target_type = "ame.unmapped";
    }
    step.interaction.push_back(interaction);
    plan.step.push_back(step);
  }

  if (result.success) {
    plan.compiled_bt_xml =
        plan_compiler_.compileSequential(result.steps, world_model_,
                                         action_registry_);
    plan.predicted_quality = 100.0;
  }
  return plan;
}

PyramidAutonomyBridge::ExecutionRun PyramidAutonomyBridge::makeRun(
    const PlanningExecutionRequirement& requirement,
    const Plan& plan) const {
  ExecutionRun run;
  run.id = plan.id + "/run";
  run.planning_execution_requirement_id = plan.planning_execution_requirement_id;
  run.plan_id = plan.id;
  run.replan_count = plan.replan_count;

  if (!plan.plan_success) {
    run.state = model::PlanningExecutionState::Failed;
    run.achievement.status = common::Progress::Failed;
    return run;
  }

  if (requirement.mode == model::PlanningExecutionMode::PlanAndExecute) {
    run.state = model::PlanningExecutionState::Accepted;
    run.achievement.status = common::Progress::NotStarted;
  } else {
    run.state = model::PlanningExecutionState::Achieved;
    run.achievement.status = common::Progress::Completed;
    run.achievement.quality = 100.0;
  }
  return run;
}

PyramidAutonomyBridge::RequirementPlacement PyramidAutonomyBridge::makePlacement(
    const std::string& requirement_id,
    const Plan& plan,
    const model::PlanStep& step) const {
  RequirementPlacement placement;
  placement.id = plan.id + "/placement/" + std::to_string(step.sequence_number);
  placement.planning_execution_requirement_id = requirement_id;
  placement.plan_id = plan.id;
  placement.plan_step_id = step.id;
  if (!step.interaction.empty()) {
    const auto& interaction = step.interaction.front();
    placement.target_component = interaction.target_component;
    placement.target_service = interaction.target_service;
    placement.target_type = interaction.target_type;
    placement.operation = interaction.operation;
  }
  placement.progress = plan.plan_success ? common::Progress::NotStarted
                                         : common::Progress::Failed;
  return placement;
}

void PyramidAutonomyBridge::applyStateUpdate(const StateUpdate& update) {
  for (const auto& fact_update : update.fact_update) {
    world_model_.setFact(fact_update.key,
                         fact_update.value,
                         fact_update.source,
                         toWorldModelAuthority(fact_update.authority));
  }
}

}  // namespace ame
