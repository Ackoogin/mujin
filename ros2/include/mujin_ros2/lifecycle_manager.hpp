#pragma once

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

#include <string>
#include <vector>

namespace mujin_ros2 {

/// Manages the lifecycle of WorldModelNode, PlannerNode, and ExecutorNode.
///
/// Sends ChangeState service calls in the correct dependency order:
///   startup:  WorldModel → Planner → Executor  (configure then activate each)
///   shutdown: Executor → Planner → WorldModel  (deactivate then cleanup each)
///
/// Parameters:
///   managed_nodes (string[], ["world_model_node","planner_node","executor_node"])
///   transition_timeout_ms (int, 5000)
class MujinLifecycleManager : public rclcpp::Node {
public:
    explicit MujinLifecycleManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /// Bring all managed nodes through configure → activate.
    bool startup();

    /// Bring all managed nodes through deactivate → cleanup.
    bool shutdown();

private:
    std::vector<std::string> managed_nodes_;
    std::chrono::milliseconds transition_timeout_;

    bool changeNodeState(const std::string& node_name, uint8_t transition_id);
    uint8_t getNodeState(const std::string& node_name);
};

} // namespace mujin_ros2
