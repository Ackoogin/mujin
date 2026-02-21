#pragma once

#include "mujin/world_model.h"
#include "mujin/bt_logger.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

#include "mujin_ros2/srv/get_fact.hpp"
#include "mujin_ros2/srv/set_fact.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

namespace mujin_ros2 {

/// ROS2 lifecycle node that owns the BehaviorTree.CPP runtime.
///
/// BT ticking is driven by a ROS2 timer (tick_rate_hz parameter).
/// BT world model access uses either direct WorldModel* (in-process mode)
/// or ROS2 service calls to WorldModelNode (distributed mode).
///
/// Publishers (active after on_activate):
///   /executor/bt_events  (std_msgs/String) — JSON lines from MujinBTLogger
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

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev) override;

    /// Load BT XML and begin ticking.
    /// Called externally (e.g. from combined_main after Plan action completes).
    void loadAndExecute(const std::string& bt_xml);

    /// For in-process mode: inject WorldModel pointer so BT nodes skip service calls.
    void setInProcessWorldModel(mujin::WorldModel* wm) { inprocess_wm_ = wm; }

    /// Expose factory so callers can register custom action node types.
    BT::BehaviorTreeFactory& factory() { return factory_; }

    BT::NodeStatus lastStatus() const { return last_status_; }

private:
    BT::BehaviorTreeFactory factory_;
    std::unique_ptr<BT::Tree> tree_;
    std::unique_ptr<mujin::MujinBTLogger> bt_logger_;

    mujin::WorldModel* inprocess_wm_ = nullptr;

    // Tick timer
    rclcpp::TimerBase::SharedPtr tick_timer_;

    // Publishers
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_bt_events_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_status_;

    // Service clients for distributed mode
    rclcpp::Client<mujin_ros2::srv::GetFact>::SharedPtr client_get_fact_;
    rclcpp::Client<mujin_ros2::srv::SetFact>::SharedPtr client_set_fact_;

    bool executing_ = false;
    BT::NodeStatus last_status_ = BT::NodeStatus::IDLE;

    void tickOnce();
    void registerCoreNodes();
    void publishStatus(const std::string& status_str);
};

} // namespace mujin_ros2
