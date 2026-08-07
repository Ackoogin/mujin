#pragma once

#include "ame/bt_nodes/planned_action_node.h"

#include <cstdint>
#include <string>

namespace ame {

class IExecutionSink;

/// \brief Generic BT leaf that dispatches a planned PDDL action through an
///        IExecutionSink.
///
/// The PlanCompiler emits one node per grounded action, tagged with the action
/// verb (e.g. <navigate-to-waypoint param0="uav1" param1="wp"/>). Every such
/// verb is registered against this single class via factory().registerNodeType,
/// so the registration name carries the verb. Parameters arrive as positional
/// ports param0..param7 (matching ActionRegistry::resolve output).
///
/// Blackboard key: "action_sink" (IExecutionSink*), injected by the
/// ExecutorComponent in loadAndExecute().
///
/// Planned-action lifecycle:
///   onActionStart() -- build ActionCommand from the verb and parameters, then submit it.
///   onActionRunning() -- poll sink->isPending()/resultFor(); RUNNING while pending,
///                  SUCCESS/FAILURE on terminal result. The sink is expected to
///                  perform any per-tick work (e.g. re-issuing position targets,
///                  arrival detection) inside isPending().
///   onActionHalted() -- cancel the pending command.
class AmeDispatchNode : public PlannedActionNode {
public:
  AmeDispatchNode(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus onActionStart() override;
  BT::NodeStatus onActionRunning() override;
  void onActionHalted() override;

private:
  std::string command_id_;
  IExecutionSink* sink_ = nullptr;
};

}  // namespace ame
