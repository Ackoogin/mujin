#include <ame_ros2/world_model_node.hpp>

#include <ame/world_model_component.h>

namespace ame_ros2 {

WorldModelNode::WorldModelNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("world_model_node", options)
  , component_(std::make_unique<ame::WorldModelComponent>())
{}

WorldModelNode::~WorldModelNode() = default;

ame::WorldModel& WorldModelNode::worldModel()             { return component_->worldModel(); }
const ame::WorldModel& WorldModelNode::worldModel() const { return component_->worldModel(); }
ame::WorldModelComponent& WorldModelNode::component()     { return *component_; }

WorldModelNode::CallbackReturn
WorldModelNode::on_configure(const rclcpp_lifecycle::State&) {
  declare_parameter("domain.pddl_file",               std::string(""));
  declare_parameter("domain.problem_file",             std::string(""));
  declare_parameter("audit_log.enabled",               true);
  declare_parameter("audit_log.path",                  std::string("wm_audit.jsonl"));
  declare_parameter("publish_rate_hz",                 10.0);
  declare_parameter("perception.enabled",              true);
  declare_parameter("perception.confidence_threshold", 0.5);

  component_->setParam("domain.pddl_file",
                      get_parameter("domain.pddl_file").as_string().c_str());
  component_->setParam("domain.problem_file",
                      get_parameter("domain.problem_file").as_string().c_str());
  component_->setParam("audit_log.enabled",
                      get_parameter("audit_log.enabled").as_bool());
  component_->setParam("audit_log.path",
                      get_parameter("audit_log.path").as_string().c_str());
  component_->setParam("publish_rate_hz",
                      get_parameter("publish_rate_hz").as_double());
  component_->setParam("perception.enabled",
                      get_parameter("perception.enabled").as_bool());
  component_->setParam("perception.confidence_threshold",
                      get_parameter("perception.confidence_threshold").as_double());

  if (component_->configure() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to configure WorldModelComponent");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(),
              "WorldModelNode configured: %u fluents, %u ground actions",
              component_->worldModel().numFluents(),
              component_->worldModel().numGroundActions());
  return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_activate(const rclcpp_lifecycle::State&) {
  if (component_->activate() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to activate WorldModelComponent");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(), "WorldModelNode activated");
  return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_deactivate(const rclcpp_lifecycle::State&) {
  component_->deactivate();
  RCLCPP_INFO(get_logger(), "WorldModelNode deactivated");
  return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_cleanup(const rclcpp_lifecycle::State&) {
  component_->cleanup();
  RCLCPP_INFO(get_logger(), "WorldModelNode cleaned up");
  return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_shutdown(const rclcpp_lifecycle::State&) {
  component_->shutdown();
  return CallbackReturn::SUCCESS;
}

}  // namespace ame_ros2
