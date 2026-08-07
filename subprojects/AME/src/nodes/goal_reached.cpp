#include "ame/bt_nodes/goal_reached.h"

#include "ame/bt_nodes/planned_action_node.h"
#include "ame/world_model.h"
#include "ame/world_state_access.h"

#include <sstream>

namespace ame {

GoalReached::GoalReached(const std::string& name,
                         const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList GoalReached::providedPorts() {
  return {
      BT::InputPort<std::string>("goals", "Grounded goal facts"),
      BT::InputPort<std::string>("ame_required_authority", "any",
                                 "Authority goal facts must carry: "
                                 "'any' (default) or 'confirmed'"),
  };
}

BT::NodeStatus GoalReached::tick() {
  auto encoded = getInput<std::string>("goals");
  if (!encoded) {
    return BT::NodeStatus::FAILURE;
  }

  IWorldStateAccess* access = nullptr;
  try {
    access = config().blackboard->get<IWorldStateAccess*>("world_state");
  } catch (const std::exception&) {}

  LocalWorldStateAccess local(nullptr);
  if (access == nullptr) {
    try {
      local = LocalWorldStateAccess(
          config().blackboard->get<WorldModel*>("world_model"));
      access = &local;
    } catch (const std::exception&) {
      return BT::NodeStatus::FAILURE;
    }
  }

  const bool require_confirmed =
      PlannedActionNode::requiresConfirmedPreconditions(*this);

  std::istringstream stream(encoded.value());
  std::string fact;
  while (std::getline(stream, fact, ';')) {
    const auto first = fact.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      continue;
    }
    const auto last = fact.find_last_not_of(" \t\r\n");
    if (!PlannedActionNode::factSatisfies(
            *access, fact.substr(first, last - first + 1), require_confirmed)) {
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace ame
