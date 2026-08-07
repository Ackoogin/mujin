#include "ame/bt_nodes/simulated_action.h"

namespace ame {

SimulatedAction::SimulatedAction(const std::string& name,
                                 const BT::NodeConfiguration& config)
    : PlannedActionNode(name, config) {}

BT::PortsList SimulatedAction::providedPorts() {
  return withBasePorts({
      BT::InputPort<unsigned>("ticks", 1u, "Ticks before completion"),
      BT::InputPort<bool>("success", true, "Whether the action succeeds"),
  });
}

BT::NodeStatus SimulatedAction::onActionStart() {
  configured_ticks_ = 1;
  completed_ticks_ = 0;
  succeeds_ = true;
  getInput("ticks", configured_ticks_);
  getInput("success", succeeds_);
  if (configured_ticks_ == 0) {
    configured_ticks_ = 1;
  }
  return advance();
}

BT::NodeStatus SimulatedAction::onActionRunning() {
  return advance();
}

void SimulatedAction::onActionHalted() {
  completed_ticks_ = 0;
}

BT::NodeStatus SimulatedAction::advance() {
  ++completed_ticks_;
  if (completed_ticks_ < configured_ticks_) {
    return BT::NodeStatus::RUNNING;
  }
  return succeeds_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace ame
