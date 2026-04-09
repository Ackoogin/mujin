#include <ame_ros2/planner_node.hpp>

namespace ame_ros2 {

PlannerNode::PlannerNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("planner_node", options)
{}

PlannerNode::CallbackReturn
PlannerNode::on_configure(const rclcpp_lifecycle::State&) {
  declare_parameter("domain.pddl_file",    std::string(""));
  declare_parameter("domain.problem_file", std::string(""));
  declare_parameter("plan_audit.enabled",  true);
  declare_parameter("plan_audit.path",     std::string("plan_audit.jsonl"));
  declare_parameter("compiler.parallel",   false);

  component_.setParam("domain.pddl_file",
                      get_parameter("domain.pddl_file").as_string().c_str());
  component_.setParam("domain.problem_file",
                      get_parameter("domain.problem_file").as_string().c_str());
  component_.setParam("plan_audit.enabled",
                      get_parameter("plan_audit.enabled").as_bool());
  component_.setParam("plan_audit.path",
                      get_parameter("plan_audit.path").as_string().c_str());
  component_.setParam("compiler.parallel",
                      get_parameter("compiler.parallel").as_bool());

  component_.setInProcessWorldModel(inprocess_wm_);

  if (component_.configure() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to configure PlannerComponent");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "PlannerNode configured");
  return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn
PlannerNode::on_activate(const rclcpp_lifecycle::State&) {
  if (component_.activate() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to activate PlannerComponent");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(), "PlannerNode activated");
  return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn
PlannerNode::on_deactivate(const rclcpp_lifecycle::State&) {
  component_.deactivate();
  RCLCPP_INFO(get_logger(), "PlannerNode deactivated");
  return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn
PlannerNode::on_cleanup(const rclcpp_lifecycle::State&) {
  component_.cleanup();
  return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn
PlannerNode::on_shutdown(const rclcpp_lifecycle::State&) {
  component_.shutdown();
  return CallbackReturn::SUCCESS;
}

}  // namespace ame_ros2
