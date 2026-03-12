#include <ame/action_registry.h>
#include <ame/bt_nodes/execute_phase_action.h>
#include <ame/plan_compiler.h>
#include <ame/planner.h>
#include <ame/world_model.h>

#include <behaviortree_cpp/bt_factory.h>
#include <sstream>
#include <stdexcept>

namespace ame {

ExecutePhaseAction::ExecutePhaseAction(const std::string& name,
                                       const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList ExecutePhaseAction::providedPorts() {
  return {
      BT::InputPort<std::string>(
          "phase_goals",
          "Semicolon-separated goal fluents, e.g. \"(searched sector_a);(classified sector_a)\""),
      BT::InputPort<std::string>("phase_name", "", "Human-readable phase label for audit"),
  };
}

BT::NodeStatus ExecutePhaseAction::onStart() {
  auto encoded_goals = getInput<std::string>("phase_goals");
  if (!encoded_goals) {
    return BT::NodeStatus::FAILURE;
  }

  auto goals = parseGoals(encoded_goals.value());
  if (goals.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  WorldModel* wm = nullptr;
  Planner* planner = nullptr;
  PlanCompiler* compiler = nullptr;
  ActionRegistry* registry = nullptr;
  try {
    wm = config().blackboard->get<WorldModel*>("world_model");
    planner = config().blackboard->get<Planner*>("planner");
    compiler = config().blackboard->get<PlanCompiler*>("plan_compiler");
    registry = config().blackboard->get<ActionRegistry*>("action_registry");
  } catch (const std::exception&) {
    return BT::NodeStatus::FAILURE;
  }

  if (!wm || !planner || !compiler || !registry) {
    return BT::NodeStatus::FAILURE;
  }

  // Set the sub-goals on a temporary WorldModel copy so we don't disturb
  // the current goal of the outer mission. The sub-goals guide sub-planning;
  // the original WorldModel state (facts) is shared read-only.
  wm->setGoal(goals);

  auto result = planner->solve(*wm);
  if (!result.success || result.steps.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  std::string bt_xml = compiler->compile(result.steps, *wm, *registry);

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
    sub_tree_ =
        std::make_unique<BT::Tree>(factory_ptr->createTreeFromText(bt_xml, config().blackboard));
  } catch (const std::exception&) {
    return BT::NodeStatus::FAILURE;
  }

  return onRunning();
}

BT::NodeStatus ExecutePhaseAction::onRunning() {
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

void ExecutePhaseAction::onHalted() {
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
