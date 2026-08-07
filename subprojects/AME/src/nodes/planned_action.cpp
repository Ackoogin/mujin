#include "ame/bt_nodes/planned_action.h"

#include "ame/bt_nodes/planned_action_node.h"
#include "ame/world_model.h"
#include "ame/world_state_access.h"

#include <sstream>

namespace ame {

PlannedAction::PlannedAction(const std::string& name,
                             const BT::NodeConfiguration& config)
    : BT::DecoratorNode(name, config) {}

BT::PortsList PlannedAction::providedPorts() {
  return PlannedActionNode::providedPorts();
}

BT::NodeStatus PlannedAction::tick() {
  if (!child_node_) {
    return BT::NodeStatus::FAILURE;
  }

  if (!started_) {
    getInput("ame_reactive", reactive_);
    if (!preconditionsMet()) {
      return BT::NodeStatus::FAILURE;
    }
    started_ = true;
  } else if (reactive_ && !preconditionsMet()) {
    resetChild();
    started_ = false;
    return BT::NodeStatus::FAILURE;
  }

  const auto status = child_node_->executeTick();
  if (status == BT::NodeStatus::RUNNING) {
    return status;
  }

  started_ = false;
  if (status == BT::NodeStatus::SUCCESS && !commitEffects()) {
    resetChild();
    return BT::NodeStatus::FAILURE;
  }
  resetChild();
  return status;
}

void PlannedAction::halt() {
  started_ = false;
  BT::DecoratorNode::halt();
}

std::vector<std::string> PlannedAction::factList(
    const std::string& port_name) const {
  std::string encoded;
  getInput(port_name, encoded);
  std::vector<std::string> facts;
  std::istringstream stream(encoded);
  std::string fact;
  while (std::getline(stream, fact, ';')) {
    const auto first = fact.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      continue;
    }
    const auto last = fact.find_last_not_of(" \t\r\n");
    facts.push_back(fact.substr(first, last - first + 1));
  }
  return facts;
}

IWorldStateAccess* PlannedAction::worldStateAccess() {
  try {
    auto* access = config().blackboard->get<IWorldStateAccess*>("world_state");
    if (access != nullptr) {
      return access;
    }
  } catch (const std::exception&) {}
  try {
    auto* world_model = config().blackboard->get<WorldModel*>("world_model");
    if (world_model != nullptr) {
      local_world_state_access_ = LocalWorldStateAccess(world_model);
      return &local_world_state_access_;
    }
  } catch (const std::exception&) {}
  return nullptr;
}

bool PlannedAction::preconditionsMet() {
  const auto facts = factList("ame_preconditions");
  if (facts.empty()) {
    return true;
  }
  auto* world_state = worldStateAccess();
  if (world_state == nullptr) {
    return false;
  }
  for (const auto& fact : facts) {
    if (!world_state->getFact(fact)) {
      return false;
    }
  }
  return true;
}

bool PlannedAction::commitEffects() {
  const auto add_effects = factList("ame_add_effects");
  const auto del_effects = factList("ame_del_effects");
  if (add_effects.empty() && del_effects.empty()) {
    return true;
  }
  auto* world_state = worldStateAccess();
  if (world_state == nullptr) {
    return false;
  }
  const std::string source = "PlannedAction:" + name();
  for (const auto& fact : add_effects) {
    if (!world_state->setFact(fact, true, source)) {
      return false;
    }
  }
  for (const auto& fact : del_effects) {
    if (!world_state->setFact(fact, false, source)) {
      return false;
    }
  }
  return true;
}

}  // namespace ame
