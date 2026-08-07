#pragma once

#include "ame/world_state_access.h"

#include <behaviortree_cpp/action_node.h>

#include <string>
#include <vector>

namespace ame {

/// \brief Base class for actions produced by PlanCompiler.
///
/// The compiler supplies the grounded preconditions and effects through the
/// base ports. The node checks its preconditions before it starts and, when
/// configured as reactive, before every later tick. Effects are committed only
/// after the concrete action reports SUCCESS.
///
/// The default commitEffects() records declared effects as BELIEVED facts. A
/// deployed action should override commitEffects() when it needs to record only
/// the state that its external system or sensors actually confirmed. Confirmed
/// state enforcement is deliberately outside this base class.
///
/// The ame_required_authority port decides what a precondition must carry
/// before the action will start. The default, "any", accepts a fact whether it
/// was predicted by a plan effect or observed. Setting it to "confirmed"
/// accepts only observed facts, which is how a deployment, or a tool driving
/// one, asks for execution to proceed on evidence rather than on prediction.
///
/// Derived synchronous actions implement onActionStart() and return SUCCESS or
/// FAILURE without ever returning RUNNING.
class PlannedActionNode : public BT::StatefulActionNode {
public:
  PlannedActionNode(const std::string& name,
                    const BT::NodeConfiguration& config);

  /// \brief Ports shared by every planned action.
  static BT::PortsList providedPorts();

  /// \brief Merge planned-action ports into ports declared by a derived node.
  static BT::PortsList withBasePorts(BT::PortsList ports);

  /// \brief Whether a node's ame_required_authority port demands confirmed
  /// state. Shared with the PlannedAction decorator, which applies the same
  /// contract to a registered subtree.
  static bool requiresConfirmedPreconditions(const BT::TreeNode& node);

  /// \brief Whether one precondition holds, given the authority requirement.
  static bool factSatisfies(IWorldStateAccess& world_state,
                            const std::string& fact,
                            bool require_confirmed);

  BT::NodeStatus onStart() final;
  BT::NodeStatus onRunning() final;
  void onHalted() final;

protected:
  /// \brief Start the concrete action.
  virtual BT::NodeStatus onActionStart() = 0;

  /// \brief Continue a concrete action that returned RUNNING.
  virtual BT::NodeStatus onActionRunning();

  /// \brief Stop concrete action work after the node is halted.
  virtual void onActionHalted();

  /// \brief Record effects after the concrete action reports SUCCESS.
  ///
  /// Override this in a deployed action to record the state that the action
  /// actually confirmed. Return false if the resulting state could not be
  /// recorded.
  virtual bool commitEffects();

  /// \brief Resolve the blackboard world-state access, including the legacy
  /// WorldModel fallback.
  IWorldStateAccess* worldStateAccess();

private:
  bool preconditionsMet();
  BT::NodeStatus finishAction(BT::NodeStatus status);
  std::vector<std::string> factList(const std::string& port_name) const;

  bool reactive_ = false;
  LocalWorldStateAccess local_world_state_access_{nullptr};
};

}  // namespace ame
