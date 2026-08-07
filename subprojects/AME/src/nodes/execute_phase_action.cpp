#include <ame/action_registry.h>
#include <ame/bt_nodes/execute_phase_action.h>
#include <ame/plan_audit_log.h>
#include <ame/plan_compiler.h>
#include <ame/planner.h>
#include <ame/world_model.h>

#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <sstream>
#include <stdexcept>

namespace ame {

ExecutePhaseAction::ExecutePhaseAction(const std::string& name,
                                       const BT::NodeConfiguration& config)
    : PlannedActionNode(name, config) {}

BT::PortsList ExecutePhaseAction::providedPorts() {
  return withBasePorts({
      BT::InputPort<std::string>(
          "phase_goals",
          "Semicolon-separated goal fluents, e.g. \"(searched sector_a);(classified sector_a)\""),
      BT::InputPort<std::string>("phase_name", "", "Human-readable phase label for audit"),
  });
}

BT::NodeStatus ExecutePhaseAction::onActionStart() {
  auto encoded_goals = getInput<std::string>("phase_goals");
  if (!encoded_goals) {
    return BT::NodeStatus::FAILURE;
  }

  auto goals = parseGoals(encoded_goals.value());
  if (goals.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  std::string phase_name;
  auto phase_name_input = getInput<std::string>("phase_name");
  if (phase_name_input) {
    phase_name = phase_name_input.value();
  }

  WorldModel* wm = nullptr;
  try {
    wm = config().blackboard->get<WorldModel*>("world_model");
  } catch (const std::exception&) {
    return BT::NodeStatus::FAILURE;
  }
  if (!wm) {
    return BT::NodeStatus::FAILURE;
  }

  return planDirect(goals, wm, phase_name);
}

BT::NodeStatus ExecutePhaseAction::planDirect(const std::vector<std::string>& goals,
                                              WorldModel* wm,
                                              const std::string& phase_name) {
  Planner* planner = nullptr;
  PlanCompiler* compiler = nullptr;
  ActionRegistry* registry = nullptr;
  try {
    planner = config().blackboard->get<Planner*>("planner");
    compiler = config().blackboard->get<PlanCompiler*>("plan_compiler");
    registry = config().blackboard->get<ActionRegistry*>("action_registry");
  } catch (const std::exception&) {
    return BT::NodeStatus::FAILURE;
  }

  if (!planner || !compiler || !registry) {
    return BT::NodeStatus::FAILURE;
  }

  WorldModel local_wm(*wm);
  local_wm.setAuditCallback({});

  std::vector<unsigned> goal_ids;
  goal_ids.reserve(goals.size());
  try {
    for (const auto& goal : goals) {
      goal_ids.push_back(local_wm.fluentIndex(goal));
    }
  } catch (const std::exception&) {
    recordAuditEpisode(phase_name, goals, local_wm, "BRFS",
                       0.0, false, 0, 0, 0.0f, {}, "");
    return BT::NodeStatus::FAILURE;
  }

  auto result = planner->solve(local_wm, goal_ids);
  if (!result.success || result.steps.empty()) {
    recordAuditEpisode(phase_name, goals, local_wm, "BRFS",
                       result.solve_time_ms, false,
                       result.expanded, result.generated,
                       result.cost, {}, "");
    return BT::NodeStatus::FAILURE;
  }

  std::string bt_xml = compiler->compile(result.steps, local_wm, *registry, goal_ids);

  // Collect plan action signatures for audit.
  std::vector<std::string> plan_actions;
  for (const auto& step : result.steps) {
    plan_actions.push_back(local_wm.groundActions()[step.action_index].signature);
  }

  recordAuditEpisode(phase_name, goals, local_wm, "BRFS",
                     result.solve_time_ms, true,
                     result.expanded, result.generated,
                     result.cost, plan_actions, bt_xml);

  // Build the sub-tree using the same factory and blackboard as the parent.
  BT::BehaviorTreeFactory* factory_ptr = nullptr;
  try {
    factory_ptr = config().blackboard->get<BT::BehaviorTreeFactory*>("bt_factory");
  } catch (const std::exception&) {
    return BT::NodeStatus::FAILURE;
  }
  if (!factory_ptr) {
    return BT::NodeStatus::FAILURE;
  }

  try {
    auto bb = config().blackboard;
    // Propagate this episode's ID as parent for any nested ExecutePhaseAction nodes.
    if (episode_id_ != 0) {
      bb->set("parent_episode_id", episode_id_);
    }
    sub_tree_ =
        std::make_unique<BT::Tree>(factory_ptr->createTreeFromText(bt_xml, bb));
  } catch (const std::exception&) {
    return BT::NodeStatus::FAILURE;
  }

  return onActionRunning();
}

void ExecutePhaseAction::recordAuditEpisode(
    const std::string& phase_name,
    const std::vector<std::string>& goals,
    const WorldModel& wm,
    const std::string& solver,
    double solve_time_ms,
    bool success,
    unsigned expanded,
    unsigned generated,
    float cost,
    const std::vector<std::string>& plan_actions,
    const std::string& bt_xml) {
  PlanAuditLog* audit = nullptr;
  try {
    audit = config().blackboard->get<PlanAuditLog*>("plan_audit_log");
  } catch (const std::exception&) {}

  if (!audit) {
    return;
  }

  PlanAuditLog::Episode ep;
  ep.phase_name = phase_name;
  ep.ts_us = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
  ep.solver = solver;
  ep.solve_time_ms = solve_time_ms;
  ep.success = success;
  ep.expanded = expanded;
  ep.generated = generated;
  ep.cost = cost;
  ep.plan_actions = plan_actions;
  ep.bt_xml = bt_xml;
  ep.goal_fluents = goals;

  // Capture init facts from world model.
  for (unsigned i = 0; i < wm.numFluents(); ++i) {
    if (wm.getFact(i)) {
      ep.init_facts.push_back(wm.fluentName(i));
    }
  }

  // Link to parent episode if we're inside a nested hierarchy.
  try {
    ep.parent_episode_id =
        config().blackboard->get<uint64_t>("parent_episode_id");
  } catch (const std::exception&) {
    ep.parent_episode_id = 0;
  }

  try {
    ep.session_id = config().blackboard->get<std::string>("session_id");
  } catch (const std::exception&) {
    ep.session_id.clear();
  }

  episode_id_ = audit->recordEpisode(std::move(ep));
}

BT::NodeStatus ExecutePhaseAction::onActionRunning() {
  if (!sub_tree_) {
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus status = sub_tree_->tickOnce();
  if (status == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }

  sub_tree_.reset();
  return status;
}

void ExecutePhaseAction::onActionHalted() {
  if (sub_tree_) {
    sub_tree_->haltTree();
    sub_tree_.reset();
  }
}

std::vector<std::string> ExecutePhaseAction::parseGoals(const std::string& encoded) {
  std::vector<std::string> goals;
  std::istringstream stream(encoded);
  std::string token;
  while (std::getline(stream, token, ';')) {
    size_t start = token.find_first_not_of(" \t");
    size_t end = token.find_last_not_of(" \t");
    if (start != std::string::npos) {
      goals.push_back(token.substr(start, end - start + 1));
    }
  }
  return goals;
}

} // namespace ame
