#include <ame_ros2/agent_dispatcher_node.hpp>

namespace ame_ros2 {

AgentDispatcherNode::AgentDispatcherNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("agent_dispatcher_node", options) {}

AgentDispatcherNode::CallbackReturn
AgentDispatcherNode::on_configure(const rclcpp_lifecycle::State&) {
    declare_parameter("agent_ids", std::string(""));

    component_.setParam("agent_ids",
                        get_parameter("agent_ids").as_string().c_str());

    if (component_.configure() != PCL_OK) {
        RCLCPP_ERROR(get_logger(), "Failed to configure AgentDispatcher");
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "AgentDispatcherNode configured");
    return CallbackReturn::SUCCESS;
}

AgentDispatcherNode::CallbackReturn
AgentDispatcherNode::on_activate(const rclcpp_lifecycle::State&) {
    if (component_.activate() != PCL_OK) {
        RCLCPP_ERROR(get_logger(), "Failed to activate AgentDispatcher");
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "AgentDispatcherNode activated");
    return CallbackReturn::SUCCESS;
}

AgentDispatcherNode::CallbackReturn
AgentDispatcherNode::on_deactivate(const rclcpp_lifecycle::State&) {
    component_.deactivate();
    return CallbackReturn::SUCCESS;
}

AgentDispatcherNode::CallbackReturn
AgentDispatcherNode::on_cleanup(const rclcpp_lifecycle::State&) {
    component_.cleanup();
    return CallbackReturn::SUCCESS;
}

AgentDispatcherNode::CallbackReturn
AgentDispatcherNode::on_shutdown(const rclcpp_lifecycle::State&) {
    component_.shutdown();
    return CallbackReturn::SUCCESS;
}

void AgentDispatcherNode::setInProcessDependencies(
    ame::WorldModel* wm,
    ame::Planner* planner,
    ame::PlanCompiler* compiler,
    ame::ActionRegistry* registry) {
    component_.setWorldModel(wm);
    component_.setPlanner(planner);
    component_.setPlanCompiler(compiler);
    component_.setActionRegistry(registry);
}

}  // namespace ame_ros2
