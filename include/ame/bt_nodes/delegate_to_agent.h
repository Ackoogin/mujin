#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <string>
#include <vector>
#include <memory>

namespace ame {

class WorldModel;
class Planner;
class PlanCompiler;
class ActionRegistry;
class PlanAuditLog;

/// \brief Multi-agent delegation BT node.
///
/// Delegates a set of goals to a specific agent, planning and executing
/// actions scoped to that agent. Supports the PYRAMID leader-delegation
/// pattern where a leader plans at the goal level and delegates sub-goals.
///
/// Blackboard keys (required):
///   "world_model"     — WorldModel*
///   "bt_factory"      — BT::BehaviorTreeFactory*
///   "planner"         — Planner*
///   "plan_compiler"   — PlanCompiler*
///   "action_registry" — ActionRegistry*
///
/// Blackboard keys (optional):
///   "plan_audit_log"     — PlanAuditLog*
///   "parent_episode_id"  — uint64_t
///
/// Ports:
///   agent_id    (input) — ID of the agent to delegate to, e.g. "uav1"
///   agent_goals (input) — semicolon-separated goal fluent strings
///
/// The node marks the agent as unavailable during execution and restores
/// availability when the delegation completes (success or failure).
///
/// Returns SUCCESS when all delegated goals are achieved, FAILURE if
/// planning fails or agent is not found, RUNNING while executing.
class DelegateToAgent : public BT::StatefulActionNode {
public:
  DelegateToAgent(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  static std::vector<std::string> parseGoals(const std::string& encoded);

  std::unique_ptr<BT::Tree> sub_tree_;
  std::string agent_id_;
  uint64_t episode_id_ = 0;
};

} // namespace ame
