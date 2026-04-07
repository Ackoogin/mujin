#pragma once

#include <ame/action_registry.h>
#include <ame/executor_component.h>
#include <ame/plan_audit_log.h>
#include <ame/plan_compiler.h>
#include <ame/planner.h>
#include <ame/planner_component.h>
#include <ame/pyramid_service.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <ame_ros2/srv/get_fact.hpp>
#include <ame_ros2/srv/set_fact.hpp>

namespace ame_ros2 {

/// \brief ROS2 lifecycle node that owns the BehaviorTree.CPP runtime.
///
/// BT ticking is driven by a ROS2 timer (tick_rate_hz parameter).
/// BT world model access uses either direct WorldModel* (in-process mode)
/// or ROS2 service calls to WorldModelNode (distributed mode).
///
/// For multi-agent deployments, set agent_id to namespace all topics:
///   /{agent_id}/executor/bt_xml     — subscription for BT XML
///   /{agent_id}/executor/bt_events  — JSON lines from AmeBTLogger
///   /{agent_id}/executor/status     — "IDLE"/"RUNNING"/"SUCCESS"/"FAILURE"
///
/// Publishers (active after on_activate):
///   [prefix]/executor/bt_events  (std_msgs/String) — JSON lines from AmeBTLogger
///   [prefix]/executor/status     (std_msgs/String) — "IDLE"/"RUNNING"/"SUCCESS"/"FAILURE"
///
/// Parameters:
///   agent_id             (string, "")     — agent ID for topic namespacing (empty = no prefix)
///   tick_rate_hz         (double, 50.0)
///   bt_log.enabled       (bool, true)
///   bt_log.path          (string, "bt_events.jsonl")
///   bt_log.tree_id       (string, "MissionPlan")
///   world_model_node     (string, "world_model_node")
class ExecutorNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit ExecutorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev) override;

  /// \brief Loads BT XML and begins ticking.
  /// Called externally or via the /executor/bt_xml subscription.
  void loadAndExecute(const std::string& bt_xml);

  /// \brief Injects WorldModel pointer for in-process mode.
  void setInProcessWorldModel(ame::WorldModel* wm) {
    inprocess_wm_ = wm;
    component_.setInProcessWorldModel(wm);
  }

  /// \brief Injects PYRAMID service for InvokeService BT nodes.
  void setPyramidService(ame::IPyramidService* svc) { pyramid_service_ = svc; }

  /// \brief Injects planner for in-process hierarchical planning (ExecutePhaseAction).
  void setPlanner(ame::Planner* p) { planner_ = p; }

  /// \brief Injects plan compiler for in-process hierarchical planning.
  void setPlanCompiler(ame::PlanCompiler* c) { plan_compiler_ = c; }

  /// \brief Injects action registry for in-process hierarchical planning.
  void setActionRegistry(ame::ActionRegistry* r) { action_registry_ = r; }

  /// \brief Injects plan audit log for hierarchical phase recording.
  void setPlanAuditLog(ame::PlanAuditLog* l) { plan_audit_log_ = l; }

  /// \brief Injects PlannerComponent for distributed hierarchical planning.
  void setPlannerComponent(ame::PlannerComponent* pc) { planner_component_ = pc; }

  /// \brief Exposes factory for custom action node type registration.
  BT::BehaviorTreeFactory& factory() { return component_.factory(); }

  /// \brief Returns the cached final status after execution completes.
  /// While running, delegates to the component; after halt, returns the
  /// terminal status captured before the component was reset.
  BT::NodeStatus lastStatus() const { return final_status_; }

  /// \brief Returns the agent ID for this executor (empty if not agent-scoped).
  const std::string& agentId() const { return agent_id_; }

private:
  ame::ExecutorComponent component_;

  ame::WorldModel* inprocess_wm_ = nullptr;
  ame::IPyramidService* pyramid_service_ = nullptr;
  ame::Planner* planner_ = nullptr;
  ame::PlanCompiler* plan_compiler_ = nullptr;
  ame::ActionRegistry* action_registry_ = nullptr;
  ame::PlanAuditLog* plan_audit_log_ = nullptr;
  ame::PlannerComponent* planner_component_ = nullptr;
  std::string agent_id_;  ///< Agent ID for topic namespacing

  // Tick timer
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_bt_xml_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_bt_events_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_status_;

  // Service clients for distributed mode
  rclcpp::Client<ame_ros2::srv::GetFact>::SharedPtr client_get_fact_;
  rclcpp::Client<ame_ros2::srv::SetFact>::SharedPtr client_set_fact_;

  bool core_nodes_registered_ = false;
  BT::NodeStatus final_status_ = BT::NodeStatus::IDLE;

  void tickOnce();
  void registerCoreNodes();
  void publishStatus(const std::string& status_str);
};

} // namespace ame_ros2
