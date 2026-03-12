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

// ExecutePhaseAction: hierarchical planning BT node.
//
// Triggers a sub-planning episode for a decomposed mission phase, compiles
// the result into a BT subtree, and ticks the subtree to completion.
//
// Blackboard keys:
//   "world_model"    — WorldModel*
//   "planner"        — Planner*
//   "plan_compiler"  — PlanCompiler*
//   "action_registry"— ActionRegistry*
//
// Ports:
//   phase_goals  (input) — semicolon-separated goal fluent strings,
//                          e.g. "(searched sector_a);(classified sector_a)"
//   phase_name   (input) — human-readable label for audit / logging
//
// Lifecycle:
//   onStart()   — solve sub-plan, compile to BT XML, instantiate subtree
//   onRunning() — tick the compiled subtree once per BT cycle
//   onHalted()  — halt and destroy the subtree
//
// Returns SUCCESS when all sub-goals are achieved, FAILURE if planning fails,
// RUNNING while the subtree is executing.
class ExecutePhaseAction : public BT::StatefulActionNode {
public:
  ExecutePhaseAction(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // Parse semicolon-delimited goal string into individual fluent strings.
  static std::vector<std::string> parseGoals(const std::string& encoded);

  std::unique_ptr<BT::Tree> sub_tree_;
};

} // namespace ame
