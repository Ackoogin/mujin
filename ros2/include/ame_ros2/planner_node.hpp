#pragma once

#include <ame/planner_component.h>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

#include <ame_ros2/action/plan.hpp>
#include <ame_ros2/srv/query_state.hpp>

#include <memory>
#include <thread>

namespace ame_ros2 {

/// \brief ROS2 lifecycle node running LAPKT BRFS as an action server.
///
/// Action server (active after on_activate):
///   /ame/plan     (ame_ros2/action/Plan)
///
/// Parameters:
///   domain.pddl_file       (string, "") — must match WorldModelNode
///   domain.problem_file    (string, "") — must match WorldModelNode
///   world_model_node       (string, "world_model_node") — namespace for QueryState client
///   plan_audit.enabled     (bool, true)
///   plan_audit.path        (string, "plan_audit.jsonl")
///   compiler.parallel      (bool, false) — use causal-graph compiler
///   action_registry.<name> (string) — maps PDDL action name to BT node type
class PlannerNode : public rclcpp_lifecycle::LifecycleNode {
public:
  using PlanAction = ame_ros2::action::Plan;
  using GoalHandlePlan = rclcpp_action::ServerGoalHandle<PlanAction>;

  explicit PlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~PlannerNode();

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev) override;

  /// \brief Injects canonical WorldModel for in-process mode.
  /// When set, snapshotWorldModel() copies state from this pointer instead of
  /// calling the QueryState service.
  void setInProcessWorldModel(ame::WorldModel* wm) {
    inprocess_wm_ = wm;
    component_.setInProcessWorldModel(wm);
  }

  /// \brief Exposes ActionRegistry for action node type registration.
  ame::ActionRegistry& actionRegistry() { return component_.actionRegistry(); }

  /// \brief Expose Planner for in-process hierarchical planning.
  ame::Planner& planner() { return component_.planner(); }

  /// \brief Expose PlanCompiler for in-process hierarchical planning.
  ame::PlanCompiler& compiler() { return component_.compiler(); }

  /// \brief Expose PlanAuditLog for in-process audit trail (nullptr if disabled).
  ame::PlanAuditLog* planAuditLog() { return component_.planAuditLog(); }

private:
  ame::PlannerComponent component_;

  // In-process mode: direct WorldModel pointer (null = use service)
  ame::WorldModel* inprocess_wm_ = nullptr;

  // Action server
  rclcpp_action::Server<PlanAction>::SharedPtr action_server_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_bt_xml_;

  // Client to WorldModelNode QueryState service (distributed mode)
  rclcpp::Client<ame_ros2::srv::QueryState>::SharedPtr client_query_state_;

  // Action server callbacks
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const PlanAction::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandlePlan> goal_handle);

  void handleAccepted(std::shared_ptr<GoalHandlePlan> goal_handle);

  /// \brief Runs on a dedicated thread and blocks during BRFS solve.
  void executePlan(std::shared_ptr<GoalHandlePlan> goal_handle);

  ame::WorldStateSnapshot queryWorldState() const;
};

} // namespace ame_ros2
