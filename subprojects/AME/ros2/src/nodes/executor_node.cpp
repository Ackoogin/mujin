#include <ame_ros2/executor_node.hpp>
#include <ame_ros2/ros_wm_bridge.hpp>

#include <ame/bt_nodes/check_world_predicate.h>
#include <ame/bt_nodes/delegate_to_agent.h>
#include <ame/bt_nodes/execute_phase_action.h>
#include <ame/bt_nodes/invoke_service.h>
#include <ame/bt_nodes/set_world_predicate.h>

namespace ame_ros2 {

ExecutorNode::ExecutorNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("executor_node", options)
{}

ExecutorNode::CallbackReturn
ExecutorNode::on_configure(const rclcpp_lifecycle::State&) {
  declare_parameter("agent_id",          std::string(""));
  declare_parameter("tick_rate_hz",      50.0);
  declare_parameter("bt_log.enabled",    true);
  declare_parameter("bt_log.path",       std::string("bt_events.jsonl"));
  declare_parameter("bt_log.tree_id",    std::string("MissionPlan"));

  agent_id_ = get_parameter("agent_id").as_string();

  component_.setParam("agent_id",       agent_id_.c_str());
  component_.setParam("tick_rate_hz",   get_parameter("tick_rate_hz").as_double());
  component_.setParam("bt_log.enabled", get_parameter("bt_log.enabled").as_bool());
  component_.setParam("bt_log.path",    get_parameter("bt_log.path").as_string().c_str());
  component_.setParam("bt_log.tree_id", get_parameter("bt_log.tree_id").as_string().c_str());

  registerCoreNodes();

  // Wire blackboard initializer with in-process dependencies
  component_.setBlackboardInitializer([this](const BT::Blackboard::Ptr& bb) {
    if (!agent_id_.empty()) {
      bb->set("current_agent_id", agent_id_);
    }
    if (pyramid_service_) {
      bb->set<ame::IPyramidService*>("pyramid_service", pyramid_service_);
    }
    bb->set<BT::BehaviorTreeFactory*>("bt_factory", &component_.factory());
    if (inprocess_wm_) {
      bb->set<ame::WorldModel*>("world_model", inprocess_wm_);
    }
    if (planner_) {
      bb->set<ame::Planner*>("planner", planner_);
    }
    if (plan_compiler_) {
      bb->set<ame::PlanCompiler*>("plan_compiler", plan_compiler_);
    }
    if (action_registry_) {
      bb->set<ame::ActionRegistry*>("action_registry", action_registry_);
    }
    if (plan_audit_log_) {
      bb->set<ame::PlanAuditLog*>("plan_audit_log", plan_audit_log_);
    }
    if (planner_component_) {
      bb->set<ame::PlannerComponent*>("planner_component", planner_component_);
    }
  });

  if (component_.configure() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to configure ExecutorComponent");
    return CallbackReturn::FAILURE;
  }

  if (!agent_id_.empty()) {
    RCLCPP_INFO(get_logger(), "ExecutorNode configured for agent '%s'", agent_id_.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "ExecutorNode configured");
  }
  return CallbackReturn::SUCCESS;
}

ExecutorNode::CallbackReturn
ExecutorNode::on_activate(const rclcpp_lifecycle::State&) {
  if (component_.activate() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to activate ExecutorComponent");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(), "ExecutorNode activated");
  return CallbackReturn::SUCCESS;
}

ExecutorNode::CallbackReturn
ExecutorNode::on_deactivate(const rclcpp_lifecycle::State&) {
  component_.deactivate();
  return CallbackReturn::SUCCESS;
}

ExecutorNode::CallbackReturn
ExecutorNode::on_cleanup(const rclcpp_lifecycle::State&) {
  component_.cleanup();
  return CallbackReturn::SUCCESS;
}

ExecutorNode::CallbackReturn
ExecutorNode::on_shutdown(const rclcpp_lifecycle::State&) {
  component_.shutdown();
  return CallbackReturn::SUCCESS;
}

void ExecutorNode::registerCoreNodes() {
  if (core_nodes_registered_) return;

  if (!inprocess_wm_) {
    component_.factory().registerNodeType<RosCheckWorldPredicate>("CheckWorldPredicate");
    component_.factory().registerNodeType<RosSetWorldPredicate>("SetWorldPredicate");
  }
  component_.factory().registerNodeType<ame::InvokeService>("InvokeService");
  component_.factory().registerNodeType<ame::ExecutePhaseAction>("ExecutePhaseAction");
  component_.factory().registerNodeType<ame::DelegateToAgent>("DelegateToAgent");

  core_nodes_registered_ = true;
}

}  // namespace ame_ros2
