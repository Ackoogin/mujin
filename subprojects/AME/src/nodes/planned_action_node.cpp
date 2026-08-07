#include "ame/bt_nodes/planned_action_node.h"

#include "ame/world_model.h"
#include "ame/world_state_access.h"

#include <sstream>

namespace ame {

namespace {

std::vector<std::string> parseFacts(const std::string& encoded) {
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

}  // namespace

PlannedActionNode::PlannedActionNode(const std::string& name,
                                     const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList PlannedActionNode::providedPorts() {
  return {
      BT::InputPort<std::string>("ame_preconditions", "", "Grounded preconditions"),
      BT::InputPort<std::string>("ame_add_effects", "", "Grounded add effects"),
      BT::InputPort<std::string>("ame_del_effects", "", "Grounded delete effects"),
      BT::InputPort<bool>("ame_reactive", false, "Recheck preconditions while running"),
  };
}

BT::PortsList PlannedActionNode::withBasePorts(BT::PortsList ports) {
  for (auto& [name, info] : providedPorts()) {
    ports.insert_or_assign(name, std::move(info));
  }
  return ports;
}

BT::NodeStatus PlannedActionNode::onStart() {
  getInput("ame_reactive", reactive_);
  if (!preconditionsMet()) {
    return BT::NodeStatus::FAILURE;
  }
  return finishAction(onActionStart());
}

BT::NodeStatus PlannedActionNode::onRunning() {
  if (reactive_ && !preconditionsMet()) {
    onActionHalted();
    return BT::NodeStatus::FAILURE;
  }
  return finishAction(onActionRunning());
}

void PlannedActionNode::onHalted() {
  onActionHalted();
}

BT::NodeStatus PlannedActionNode::onActionRunning() {
  return BT::NodeStatus::FAILURE;
}

void PlannedActionNode::onActionHalted() {}

bool PlannedActionNode::commitEffects() {
  auto* world_state = worldStateAccess();
  const auto add_effects = factList("ame_add_effects");
  const auto del_effects = factList("ame_del_effects");
  if ((add_effects.empty() && del_effects.empty())) {
    return true;
  }
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

IWorldStateAccess* PlannedActionNode::worldStateAccess() {
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

bool PlannedActionNode::preconditionsMet() {
  const auto preconditions = factList("ame_preconditions");
  if (preconditions.empty()) {
    return true;
  }
  auto* world_state = worldStateAccess();
  if (world_state == nullptr) {
    return false;
  }
  for (const auto& fact : preconditions) {
    if (!world_state->getFact(fact)) {
      return false;
    }
  }
  return true;
}

BT::NodeStatus PlannedActionNode::finishAction(BT::NodeStatus status) {
  if (status == BT::NodeStatus::SUCCESS && !commitEffects()) {
    return BT::NodeStatus::FAILURE;
  }
  return status;
}

std::vector<std::string> PlannedActionNode::factList(
    const std::string& port_name) const {
  std::string encoded;
  getInput(port_name, encoded);
  return parseFacts(encoded);
}

}  // namespace ame
