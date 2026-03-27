#pragma once

#include <ame/executor_component.h>

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
/// Publishers (active after on_activate):
///   /executor/bt_events  (std_msgs/String) — JSON lines from AmeBTLogger
///   /executor/status     (std_msgs/String) — "IDLE"/"RUNNING"/"SUCCESS"/"FAILURE"
///
/// Parameters:
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

  /// \brief Exposes factory for custom action node type registration.
  BT::BehaviorTreeFactory& factory() { return component_.factory(); }

  BT::NodeStatus lastStatus() const { return component_.lastStatus(); }

private:
  ame::ExecutorComponent component_;

  ame::WorldModel* inprocess_wm_ = nullptr;

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

  void tickOnce();
  void registerCoreNodes();
  void publishStatus(const std::string& status_str);
};

} // namespace ame_ros2
