#include "ame/pyramid_autonomy_bridge.h"

#include <algorithm>
#include <memory>
#include <sstream>
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
    const PlanCompiler& plan_compiler,
    std::shared_ptr<IExecutionSink> execution_sink)
    : world_model_(world_model),
      action_registry_(action_registry),
      planner_(planner),
      plan_compiler_(plan_compiler),
      execution_sink_(std::move(execution_sink)) {
  if (!execution_sink_) {
    execution_sink_ = std::make_shared<RequirementBindingExecutionSink>(
        ExecutionBindingPolicy::PreferTypedPlacement,
        defaultExecutionBindings());
  }
  execution_sink_->reset("ame-pyramid-bridge");
}

PyramidAutonomyBridge::PyramidAutonomyBridge(
    WorldModel& world_model,
    const ActionRegistry& action_registry,
    const PyramidAutonomyBridgeOptions& options,
    const Planner& planner,
    const PlanCompiler& plan_compiler)
    : world_model_(world_model),
      action_registry_(action_registry),
      planner_(planner),
      plan_compiler_(plan_compiler),
      execution_sink_(options.execution_sink) {
  if (!execution_sink_) {
    auto bindings = options.include_default_execution_bindings
                        ? defaultExecutionBindings()
                        : std::vector<ExecutionBinding>{};
    bindings.insert(bindings.end(),
                    options.execution_bindings.begin(),
                    options.execution_bindings.end());
    execution_sink_ = std::make_shared<RequirementBindingExecutionSink>(
        options.execution_policy, std::move(bindings));
  }
  execution_sink_->reset("ame-pyramid-bridge");
}

std::vector<ExecutionBinding> PyramidAutonomyBridge::defaultExecutionBindings() {
  ExecutionBinding move;
  move.action_name = "move";
  move.target_component = "mobility";
  move.target_service = "Move_Service";
  move.target_type = "pyramid.data_model.mobility.MoveRequirement";

  ExecutionBinding search;
  search.action_name = "search";
  search.target_component = "tactical_objects";
  search.target_service = "Object_Of_Interest_Service";
  search.target_type =
      "pyramid.data_model.tactical.ObjectInterestRequirement";

  return {move, search};
}

bool PyramidAutonomyBridge::registerExecutionBinding(
    const ExecutionBinding& binding) {
  auto binding_sink =
      std::dynamic_pointer_cast<RequirementBindingExecutionSink>(
          execution_sink_);
  if (!binding_sink) {
    return false;
  }
  binding_sink->addBinding(binding);
  return true;
}

std::vector<PyramidAutonomyBridge::Capabilities>
PyramidAutonomyBridge::handleReadCapabilities(const Query& request) {
  Capabilities capabilities;
  capabilities.id = "ame.current_stack";
  capabilities.backend_id = "ame.current_stack";
  capabilities.supports_plan_only = true;
  capabilities.supports_plan_and_execute = true;
  capabilities.supports_execute_approved_plan = true;
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

  Plan plan;
  const auto execute_approved =
      requirement.mode == model::PlanningExecutionMode::ExecuteApprovedPlan &&
      !requirement.approved_plan_id.empty();
  if (execute_approved) {
    const auto plan_it = plans_.find(requirement.approved_plan_id);
    if (plan_it != plans_.end()) {
      plan = plan_it->second;
    } else {
      plan.id = requirement.approved_plan_id;
      plan.planning_execution_requirement_id = requirement_id;
      plan.backend_id = "ame.current_stack";
      plan.world_version = world_model_.version();
      plan.plan_success = false;
    }
  } else {
    plan = makePlan(requirement, requirement_id);
    plans_[plan.id] = plan;
  }

  auto run = makeRun(requirement, plan);
  runs_[run.id] = run;

  const auto execute =
      plan.plan_success &&
      (requirement.mode == model::PlanningExecutionMode::PlanAndExecute ||
       requirement.mode == model::PlanningExecutionMode::ExecuteApprovedPlan);
  bool execution_rejected = false;
  for (const auto& step : plan.step) {
    if (execute) {
      const auto command = makeCommand(requirement_id, step);
      const auto submission = execution_sink_->submit(command);
      if (submission.placement.has_value()) {
        placement_contexts_[submission.placement->command_id] =
            {requirement_id, plan.id, step.id};
        placements_[submission.placement->placement_id] =
            makePlacement(submission.placement.value());
        continue;
      }

      auto fallback = makePlacement(requirement_id, plan, step);
      fallback.progress =
          submission.accepted ? common::Progress::InProgress
                              : common::Progress::Failed;
      execution_rejected = execution_rejected || !submission.accepted;
      fallback.target_component =
          submission.command_egress_visible ? "ame.command_fallback"
                                            : fallback.target_component;
      placements_[fallback.id] = fallback;
      continue;
    }

    auto placement = makePlacement(requirement_id, plan, step);
    placements_[placement.id] = placement;
  }
  syncPlacementRecordsFromSink();
  if (execution_rejected) {
    for (auto& [run_id, stored_run] : runs_) {
      if (stored_run.planning_execution_requirement_id == requirement_id) {
        stored_run.state = model::PlanningExecutionState::Failed;
        stored_run.achievement.status = common::Progress::Failed;
      }
    }
    requirements_[requirement_id].status.status = common::Progress::Failed;
    return requirement_id;
  }
  refreshExecutionProgress(requirement_id);

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
  refreshExecutionProgress(request.base.id);
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
  syncPlacementRecordsFromSink();
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

std::vector<std::string> PyramidAutonomyBridge::actionParameters(
    const std::string& signature) {
  std::vector<std::string> params;
  const auto open = signature.find('(');
  const auto close = signature.rfind(')');
  if (open == std::string::npos || close == std::string::npos ||
      close <= open + 1) {
    return params;
  }

  std::string token;
  for (size_t i = open + 1; i < close; ++i) {
    if (signature[i] == ',') {
      params.push_back(token);
      token.clear();
      continue;
    }
    if (signature[i] != ' ') {
      token.push_back(signature[i]);
    }
  }
  if (!token.empty()) {
    params.push_back(token);
  }
  return params;
}

bool PyramidAutonomyBridge::queryIncludes(const Query& query,
                                          const std::string& id) {
  return std::find(query.id.begin(), query.id.end(), id) != query.id.end();
}

common::Progress PyramidAutonomyBridge::progressFromPlacementState(
    RequirementPlacementState state) {
  switch (state) {
    case RequirementPlacementState::Pending:
      return common::Progress::NotStarted;
    case RequirementPlacementState::Running:
      return common::Progress::InProgress;
    case RequirementPlacementState::Completed:
      return common::Progress::Completed;
    case RequirementPlacementState::Failed:
    case RequirementPlacementState::Unsupported:
      return common::Progress::Failed;
    case RequirementPlacementState::Cancelled:
      return common::Progress::Cancelled;
    case RequirementPlacementState::Unspecified:
      return common::Progress::Unspecified;
  }
  return common::Progress::Unspecified;
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
    if (step.action_name == "move") {
      interaction.target_component = "mobility";
      interaction.target_service = "Move_Service";
      interaction.target_type = "pyramid.data_model.mobility.MoveRequirement";
    } else if (step.action_name == "search") {
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
  run.id = plan.id + "/run/" + requirement.base.id;
  run.planning_execution_requirement_id = requirement.base.id;
  run.plan_id = plan.id;
  run.replan_count = plan.replan_count;

  if (!plan.plan_success) {
    run.state = model::PlanningExecutionState::Failed;
    run.achievement.status = common::Progress::Failed;
    return run;
  }

  if (requirement.mode == model::PlanningExecutionMode::PlanAndExecute ||
      requirement.mode ==
          model::PlanningExecutionMode::ExecuteApprovedPlan) {
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

ActionCommand PyramidAutonomyBridge::makeCommand(
    const std::string& requirement_id,
    const model::PlanStep& step) const {
  ActionCommand command;
  command.command_id = requirement_id + "/command/" +
                       std::to_string(step.sequence_number);
  command.action_name = step.action_name;
  command.signature = step.signature;
  command.service_name = step.action_name;
  command.operation = step.action_name;

  const auto params = actionParameters(step.signature);
  for (size_t i = 0; i < params.size(); ++i) {
    command.request_fields["param" + std::to_string(i)] = params[i];
  }
  return command;
}

PyramidAutonomyBridge::RequirementPlacement PyramidAutonomyBridge::makePlacement(
    const RequirementPlacementRecord& record) const {
  RequirementPlacement placement;
  placement.id = record.placement_id;
  const auto context_it = placement_contexts_.find(record.command_id);
  if (context_it != placement_contexts_.end()) {
    placement.planning_execution_requirement_id =
        context_it->second.requirement_id;
    placement.plan_id = context_it->second.plan_id;
    placement.plan_step_id = context_it->second.plan_step_id;
  }
  placement.target_component = record.target_component;
  placement.target_service = record.target_service;
  placement.target_type = record.target_type;
  placement.operation = model::RequirementPlacementOperation::CreateRequirement;
  placement.target_requirement_id = record.target_requirement_id;
  placement.progress = progressFromPlacementState(record.state);
  return placement;
}

void PyramidAutonomyBridge::syncPlacementRecordsFromSink() {
  for (const auto& record : execution_sink_->readPlacements()) {
    if (placement_contexts_.find(record.command_id) ==
        placement_contexts_.end()) {
      continue;
    }
    placements_[record.placement_id] = makePlacement(record);
  }
}

void PyramidAutonomyBridge::applyStateUpdate(const StateUpdate& update) {
  for (const auto& fact_update : update.fact_update) {
    world_model_.setFact(fact_update.key,
                         fact_update.value,
                         fact_update.source,
                         toWorldModelAuthority(fact_update.authority));
  }
  std::vector<std::string> requirement_ids;
  requirement_ids.reserve(requirements_.size());
  for (const auto& [id, requirement] : requirements_) {
    requirement_ids.push_back(id);
  }
  for (const auto& id : requirement_ids) {
    refreshExecutionProgress(id);
  }
}

bool PyramidAutonomyBridge::requirementGoalsSatisfied(
    const PlanningExecutionRequirement& requirement) const {
  bool has_expression_goal = false;
  for (const auto& goal : requirement.goal) {
    if (!goal.expression.has_value()) {
      continue;
    }
    has_expression_goal = true;
    if (!world_model_.getFact(goal.expression.value())) {
      return false;
    }
  }
  return has_expression_goal;
}

std::vector<PyramidAutonomyBridge::RequirementPlacement>
PyramidAutonomyBridge::outstandingPlacements(
    const std::string& requirement_id,
    const std::string& plan_id) const {
  std::vector<RequirementPlacement> result;
  for (const auto& [id, placement] : placements_) {
    if (placement.planning_execution_requirement_id == requirement_id &&
        placement.plan_id == plan_id &&
        placement.progress != common::Progress::Completed &&
        placement.progress != common::Progress::Cancelled &&
        placement.progress != common::Progress::Failed) {
      result.push_back(placement);
    }
  }
  return result;
}

void PyramidAutonomyBridge::refreshExecutionProgress(
    const std::string& requirement_id) {
  auto requirement_it = requirements_.find(requirement_id);
  if (requirement_it == requirements_.end()) {
    return;
  }
  auto& requirement = requirement_it->second;
  if (requirement.mode != model::PlanningExecutionMode::PlanAndExecute &&
      requirement.mode != model::PlanningExecutionMode::ExecuteApprovedPlan) {
    return;
  }

  const auto goals_satisfied = requirementGoalsSatisfied(requirement);
  for (auto& [id, run] : runs_) {
    if (run.planning_execution_requirement_id != requirement_id ||
        run.state == model::PlanningExecutionState::Cancelled ||
        run.state == model::PlanningExecutionState::Failed) {
      continue;
    }

    if (goals_satisfied) {
      for (const auto& [command_id, context] : placement_contexts_) {
        if (context.requirement_id == requirement_id &&
            context.plan_id == run.plan_id) {
          CommandResult result;
          result.command_id = command_id;
          result.status = CommandStatus::SUCCEEDED;
          result.source = "ame:pyramid-state-feedback";
          execution_sink_->pushResult(result);
        }
      }
      syncPlacementRecordsFromSink();

      run.state = model::PlanningExecutionState::Achieved;
      run.achievement.status = common::Progress::Completed;
      run.achievement.quality = 100.0;
      run.outstanding_placement.clear();
      requirement.status.status = common::Progress::Completed;
      requirement.status.quality = 100.0;

      for (auto& [placement_id, placement] : placements_) {
        if (placement.planning_execution_requirement_id == requirement_id &&
            placement.plan_id == run.plan_id) {
          placement.progress = common::Progress::Completed;
        }
      }
      continue;
    }

    run.state = model::PlanningExecutionState::WaitingForComponents;
    run.achievement.status = common::Progress::InProgress;
    requirement.status.status = common::Progress::InProgress;
    run.outstanding_placement =
        outstandingPlacements(requirement_id, run.plan_id);
  }
}

}  // namespace ame
