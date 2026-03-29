#pragma once

#include <ame/agent_dispatcher.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

#include <map>
#include <memory>
#include <string>

namespace ame_ros2 {

/// \brief ROS2 lifecycle node that wraps AgentDispatcher for multi-agent coordination.
///
/// Provides transport layer for dispatching BT XML to remote agent executors
/// via topic publication and status subscription.
///
/// For each agent, publishes to:
///   /{agent_id}/executor/bt_xml     — BT XML to execute
///
/// Subscribes to:
///   /{agent_id}/executor/status     — "IDLE"/"RUNNING"/"SUCCESS"/"FAILURE"
///
/// Service (active after on_activate):
///   ~/dispatch_goals    (ame_ros2/srv/DispatchGoals) — trigger goal dispatch
///
/// Parameters:
///   world_model_node    (string, "world_model_node") — for in-process WM access
class AgentDispatcherNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit AgentDispatcherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev) override;

    /// \brief Inject in-process dependencies for single-process deployment.
    void setInProcessDependencies(ame::WorldModel* wm,
                                   ame::Planner* planner,
                                   ame::PlanCompiler* compiler,
                                   ame::ActionRegistry* registry);

    /// \brief Access the underlying component.
    ame::AgentDispatcher& dispatcher() { return component_; }

private:
    ame::AgentDispatcher component_;

    // In-process mode pointers
    ame::WorldModel* inprocess_wm_ = nullptr;
    ame::Planner* inprocess_planner_ = nullptr;
    ame::PlanCompiler* inprocess_compiler_ = nullptr;
    ame::ActionRegistry* inprocess_registry_ = nullptr;

    // Publishers for each agent (created dynamically)
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> agent_bt_pubs_;

    // Subscriptions for agent status
    std::map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> agent_status_subs_;
    std::map<std::string, std::string> agent_statuses_;

    // Transport callbacks
    bool sendBTToAgent(const std::string& agent_id, const std::string& bt_xml);
    std::string queryAgentStatus(const std::string& agent_id);

    // Ensure publisher exists for agent
    void ensureAgentPublisher(const std::string& agent_id);
    void ensureAgentStatusSubscription(const std::string& agent_id);
};

}  // namespace ame_ros2
