#pragma once

#include "ame/bt_nodes/planned_action_node.h"

namespace ame {

/// \brief Stand-in planned action for simulation and unregistered actions.
class SimulatedAction : public PlannedActionNode {
public:
  SimulatedAction(const std::string& name,
                  const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

private:
  BT::NodeStatus advance();

  unsigned configured_ticks_ = 1;
  unsigned completed_ticks_ = 0;
  bool succeeds_ = true;
};

}  // namespace ame
