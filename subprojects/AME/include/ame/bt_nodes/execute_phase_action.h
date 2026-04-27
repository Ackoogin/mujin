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
class PlannerComponent;

/// \brief Hierarchical planning BT node.
///
/// Triggers a sub-planning episode for a decomposed mission phase, compiles
/// the result into a BT subtree, and ticks the subtree to completion.
///
/// Blackboard keys (required):
///   "world_model"    -- WorldModel*
///   "bt_factory"     -- BT::BehaviorTreeFactory*
///
/// Blackboard keys (planning -- one of two paths):
///   Path A (direct):
///     "planner"        -- Planner*
///     "plan_compiler"  -- PlanCompiler*
///     "action_registry"-- ActionRegistry*
///   Path B (component):
///     "planner_component" -- PlannerComponent*
///
/// Blackboard keys (optional):
///   "plan_audit_log"     -- PlanAuditLog*   (for recording sub-planning episodes)
///   "parent_episode_id"  -- uint64_t        (causal link to parent phase, 0 = top-level)
///
/// Ports:
///   phase_goals  (input) -- semicolon-separated goal fluent strings,
///                          e.g. "(searched sector_a);(classified sector_a)"
///   phase_name   (input) -- human-readable label for audit / logging
///
/// Lifecycle:
///   onStart()   -- solve sub-plan, compile to BT XML, instantiate subtree
///   onRunning() -- tick the compiled subtree once per BT cycle
///   onHalted()  -- halt and destroy the subtree
///
/// Returns SUCCESS when all sub-goals are achieved, FAILURE if planning fails,
/// RUNNING while the subtree is executing.
class ExecutePhaseAction : public BT::StatefulActionNode {
public:
  ExecutePhaseAction(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  /// Parse semicolon-delimited goal string into individual fluent strings.
  static std::vector<std::string> parseGoals(const std::string& encoded);

  /// Plan via direct Planner + PlanCompiler.
  BT::NodeStatus planDirect(const std::vector<std::string>& goals,
                            WorldModel* wm,
                            const std::string& phase_name);

  /// Plan via PlannerComponent.
  BT::NodeStatus planViaComponent(const std::vector<std::string>& goals,
                                  PlannerComponent* component,
                                  const std::string& phase_name);

  /// Record a sub-planning episode to the audit log.
  void recordAuditEpisode(const std::string& phase_name,
                          const std::vector<std::string>& goals,
                          const WorldModel& wm,
                          const std::string& solver,
                          double solve_time_ms,
                          bool success,
                          unsigned expanded,
                          unsigned generated,
                          float cost,
                          const std::vector<std::string>& plan_actions,
                          const std::string& bt_xml);

  std::unique_ptr<BT::Tree> sub_tree_;
  uint64_t episode_id_ = 0;  // ID assigned to this phase's audit episode
};

} // namespace ame
