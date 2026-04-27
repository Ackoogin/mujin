#include <ame_ros2/executor_node.hpp>
#include <ame_ros2/ros_wm_bridge.hpp>

#include <ame/executor_component.h>
#include <ame/planner_component.h>
#include <ame/bt_nodes/check_world_predicate.h>
#include <ame/bt_nodes/delegate_to_agent.h>
#include <ame/bt_nodes/execute_phase_action.h>
#include <ame/bt_nodes/invoke_service.h>
#include <ame/bt_nodes/set_world_predicate.h>

#include <behaviortree_cpp/basic_types.h>

namespace ame_ros2 {

ExecutorNode::ExecutorNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("executor_node", options)
  , component_(std::make_unique<ame::ExecutorComponent>())
{}

ExecutorNode::~ExecutorNode() = default;

void ExecutorNode::setInProcessWorldModel(ame::WorldModel* wm) {
  inprocess_wm_ = wm;
  component_->setInProcessWorldModel(wm);
}

void ExecutorNode::setPyramidService(ame::IPyramidService* svc) { pyramid_service_ = svc; }
void ExecutorNode::setPlanner(ame::Planner* p)                   { planner_ = p; }
void ExecutorNode::setPlanCompiler(ame::PlanCompiler* c)         { plan_compiler_ = c; }
void ExecutorNode::setActionRegistry(ame::ActionRegistry* r)     { action_registry_ = r; }
void ExecutorNode::setPlanAuditLog(ame::PlanAuditLog* l)         { plan_audit_log_ = l; }
void ExecutorNode::setPlannerComponent(ame::PlannerComponent* pc){ planner_component_ = pc; }

BT::BehaviorTreeFactory& ExecutorNode::factory()  { return component_->factory(); }
const std::string& ExecutorNode::agentId() const   { return agent_id_; }
ame::ExecutorComponent& ExecutorNode::component()  { return *component_; }

ExecutorNode::CallbackReturn
ExecutorNode::on_configure(const rclcpp_lifecycle::State&) {
  declare_parameter("agent_id",          std::string(""));
  declare_parameter("tick_rate_hz",      50.0);
  declare_parameter("bt_log.enabled",    true);
  declare_parameter("bt_log.path",       std::string("bt_events.jsonl"));
  declare_parameter("bt_log.tree_id",    std::string("MissionPlan"));

  agent_id_ = get_parameter("agent_id").as_string();

  component_->setParam("agent_id",       agent_id_.c_str());
  component_->setParam("tick_rate_hz",   get_parameter("tick_rate_hz").as_double());
  component_->setParam("bt_log.enabled", get_parameter("bt_log.enabled").as_bool());
  component_->setParam("bt_log.path",    get_parameter("bt_log.path").as_string().c_str());
  component_->setParam("bt_log.tree_id", get_parameter("bt_log.tree_id").as_string().c_str());

  registerCoreNodes();

  // Wire blackboard initializer with in-process dependencies
  component_->setBlackboardInitializer([this](const BT::Blackboard::Ptr& bb) {
    if (!agent_id_.empty()) {
      bb->set("current_agent_id", agent_id_);
    }
    if (pyramid_service_) {
      bb->set<ame::IPyramidService*>("pyramid_service", pyramid_service_);
    }
    bb->set<BT::BehaviorTreeFactory*>("bt_factory", &component_->factory());
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

  if (component_->configure() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to configure ExecutorComponent");
    return CallbackReturn::FAILURE;
  }

  // -- ROS2 service servers -------------------------------------------------
  svc_load_bt_ = create_service<ame_ros2::srv::LoadBt>(
      "~/load_bt",
      [this](const ame_ros2::srv::LoadBt::Request::SharedPtr req,
             ame_ros2::srv::LoadBt::Response::SharedPtr resp) {
        RCLCPP_INFO(get_logger(), "load_bt: %zu bytes of BT XML", req->bt_xml.size());
        try {
          std::lock_guard<std::mutex> lock(exec_mutex_);
          component_->loadAndExecute(req->bt_xml);
          resp->success = true;
          RCLCPP_INFO(get_logger(), "load_bt: executing");
        } catch (const std::exception& e) {
          resp->success   = false;
          resp->error_msg = e.what();
          RCLCPP_ERROR(get_logger(), "load_bt: FAILED -- %s", e.what());
        }
      });

  svc_stop_execution_ = create_service<ame_ros2::srv::StopExecution>(
      "~/stop_execution",
      [this](const ame_ros2::srv::StopExecution::Request::SharedPtr,
             ame_ros2::srv::StopExecution::Response::SharedPtr resp) {
        RCLCPP_INFO(get_logger(), "stop_execution");
        std::lock_guard<std::mutex> lock(exec_mutex_);
        component_->haltExecution();
        resp->success = true;
      });

  // -- ROS2 publishers ------------------------------------------------------
  // Use a large queue for events so fast BT ticking doesn't drop messages.
  pub_bt_events_ = create_publisher<std_msgs::msg::String>(
      "~/bt_events", rclcpp::QoS(200));
  pub_execution_status_ = create_publisher<std_msgs::msg::String>(
      "~/execution_status", rclcpp::QoS(10).transient_local());

  // Publish initial IDLE status
  {
    std_msgs::msg::String msg;
    msg.data = "IDLE";
    pub_execution_status_->publish(msg);
  }

  // Wire ExecutorComponent event sink -> ROS2 bt_events topic + status topic
  component_->setEventSink([this](const std::string& json_line) {
    if (pub_bt_events_) {
      std_msgs::msg::String msg;
      msg.data = json_line;
      pub_bt_events_->publish(msg);
    }
    if (pub_execution_status_) {
      const auto s = component_->lastStatus();
      std_msgs::msg::String status_msg;
      switch (s) {
        case BT::NodeStatus::RUNNING: status_msg.data = "RUNNING"; break;
        case BT::NodeStatus::SUCCESS: status_msg.data = "SUCCESS"; break;
        case BT::NodeStatus::FAILURE: status_msg.data = "FAILURE"; break;
        default:                      status_msg.data = "IDLE";    break;
      }
      pub_execution_status_->publish(status_msg);
    }
  });

  if (!agent_id_.empty()) {
    RCLCPP_INFO(get_logger(), "ExecutorNode configured for agent '%s'", agent_id_.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "ExecutorNode configured");
  }
  return CallbackReturn::SUCCESS;
}

ExecutorNode::CallbackReturn
ExecutorNode::on_activate(const rclcpp_lifecycle::State&) {
  if (component_->activate() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to activate ExecutorComponent");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(), "ExecutorNode activated");
  return CallbackReturn::SUCCESS;
}

ExecutorNode::CallbackReturn
ExecutorNode::on_deactivate(const rclcpp_lifecycle::State&) {
  component_->deactivate();
  return CallbackReturn::SUCCESS;
}

ExecutorNode::CallbackReturn
ExecutorNode::on_cleanup(const rclcpp_lifecycle::State&) {
  component_->setEventSink({});
  svc_load_bt_.reset();
  svc_stop_execution_.reset();
  pub_bt_events_.reset();
  pub_execution_status_.reset();
  component_->cleanup();
  return CallbackReturn::SUCCESS;
}

ExecutorNode::CallbackReturn
ExecutorNode::on_shutdown(const rclcpp_lifecycle::State&) {
  component_->shutdown();
  return CallbackReturn::SUCCESS;
}

void ExecutorNode::registerCoreNodes() {
  if (core_nodes_registered_) return;

  if (!inprocess_wm_) {
    component_->factory().registerNodeType<RosCheckWorldPredicate>("CheckWorldPredicate");
    component_->factory().registerNodeType<RosSetWorldPredicate>("SetWorldPredicate");
  }
  component_->factory().registerNodeType<ame::InvokeService>("InvokeService");
  component_->factory().registerNodeType<ame::ExecutePhaseAction>("ExecutePhaseAction");
  component_->factory().registerNodeType<ame::DelegateToAgent>("DelegateToAgent");

  core_nodes_registered_ = true;
}

}  // namespace ame_ros2
