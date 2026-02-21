#include "mujin_ros2/lifecycle_manager.hpp"

#include <lifecycle_msgs/msg/transition.hpp>

namespace mujin_ros2 {

MujinLifecycleManager::MujinLifecycleManager(const rclcpp::NodeOptions& options)
    : rclcpp::Node("lifecycle_manager", options)
{
    declare_parameter("managed_nodes", std::vector<std::string>{
        "world_model_node", "planner_node", "executor_node"});
    declare_parameter("transition_timeout_ms", 5000);

    managed_nodes_ = get_parameter("managed_nodes").as_string_array();
    const int timeout_ms = get_parameter("transition_timeout_ms").as_int();
    transition_timeout_ = std::chrono::milliseconds(timeout_ms);

    RCLCPP_INFO(get_logger(), "LifecycleManager ready (%zu nodes)",
                managed_nodes_.size());
}

bool MujinLifecycleManager::startup() {
    // Configure all nodes in order (WorldModel first — others depend on its services)
    for (const auto& node : managed_nodes_) {
        RCLCPP_INFO(get_logger(), "Configuring %s ...", node.c_str());
        if (!changeNodeState(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
            RCLCPP_ERROR(get_logger(), "Failed to configure %s", node.c_str());
            return false;
        }
    }
    // Activate all nodes in order
    for (const auto& node : managed_nodes_) {
        RCLCPP_INFO(get_logger(), "Activating %s ...", node.c_str());
        if (!changeNodeState(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
            RCLCPP_ERROR(get_logger(), "Failed to activate %s", node.c_str());
            return false;
        }
    }
    RCLCPP_INFO(get_logger(), "All nodes active");
    return true;
}

bool MujinLifecycleManager::shutdown() {
    // Reverse order
    for (auto it = managed_nodes_.rbegin(); it != managed_nodes_.rend(); ++it) {
        const auto& node = *it;
        RCLCPP_INFO(get_logger(), "Deactivating %s ...", node.c_str());
        changeNodeState(node, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        RCLCPP_INFO(get_logger(), "Cleaning up %s ...", node.c_str());
        changeNodeState(node, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    }
    return true;
}

bool MujinLifecycleManager::changeNodeState(const std::string& node_name,
                                             uint8_t transition_id)
{
    const auto service_name = "/" + node_name + "/change_state";
    auto client = create_client<lifecycle_msgs::srv::ChangeState>(service_name);

    if (!client->wait_for_service(transition_timeout_)) {
        RCLCPP_ERROR(get_logger(), "Service %s not available", service_name.c_str());
        return false;
    }

    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->transition.id = transition_id;

    auto future = client->async_send_request(req);
    if (future.wait_for(transition_timeout_) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "Transition timed out for %s", node_name.c_str());
        return false;
    }

    return future.get()->success;
}

uint8_t MujinLifecycleManager::getNodeState(const std::string& node_name) {
    const auto service_name = "/" + node_name + "/get_state";
    auto client = create_client<lifecycle_msgs::srv::GetState>(service_name);

    if (!client->wait_for_service(transition_timeout_)) return 0;

    auto req    = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = client->async_send_request(req);
    if (future.wait_for(transition_timeout_) != std::future_status::ready) return 0;

    return future.get()->current_state.id;
}

} // namespace mujin_ros2
