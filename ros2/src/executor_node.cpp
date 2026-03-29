#include <ame_ros2/executor_node.hpp>
#include <ame_ros2/ros_wm_bridge.hpp>

#include <ame/bt_nodes/check_world_predicate.h>
#include <ame/bt_nodes/set_world_predicate.h>

#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <stdexcept>

namespace ame_ros2 {

ExecutorNode::ExecutorNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("executor_node", options)
{}

ExecutorNode::CallbackReturn
ExecutorNode::on_configure(const rclcpp_lifecycle::State&) {
  declare_parameter("agent_id", std::string(""));
  declare_parameter("tick_rate_hz", 50.0);
  declare_parameter("bt_log.enabled", true);
  declare_parameter("bt_log.path", std::string("bt_events.jsonl"));
  declare_parameter("bt_log.tree_id", std::string("MissionPlan"));
  declare_parameter("world_model_node", std::string("world_model_node"));

  agent_id_ = get_parameter("agent_id").as_string();

  const auto wm_ns = get_parameter("world_model_node").as_string();
  client_get_fact_ = create_client<ame_ros2::srv::GetFact>("/" + wm_ns + "/get_fact");
  client_set_fact_ = create_client<ame_ros2::srv::SetFact>("/" + wm_ns + "/set_fact");

  component_.setParam("bt_log.enabled", get_parameter("bt_log.enabled").as_bool());
  component_.setParam("bt_log.path", get_parameter("bt_log.path").as_string().c_str());
  component_.setParam("bt_log.tree_id", get_parameter("bt_log.tree_id").as_string().c_str());
  component_.setTickRateHz(get_parameter("tick_rate_hz").as_double());

  registerCoreNodes();
  if (inprocess_wm_) {
    component_.setBlackboardInitializer([this](const BT::Blackboard::Ptr& blackboard) {
      if (!agent_id_.empty()) {
        blackboard->set("current_agent_id", agent_id_);
      }
    });
  } else {
    component_.setBlackboardInitializer([this](const BT::Blackboard::Ptr& blackboard) {
      blackboard->set("get_fact_client", client_get_fact_.get());
      blackboard->set("set_fact_client", client_set_fact_.get());
      if (!agent_id_.empty()) {
        blackboard->set("current_agent_id", agent_id_);
      }
    });
  }

  if (component_.configure() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to configure executor component");
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
    RCLCPP_ERROR(get_logger(), "Failed to activate executor component");
    return CallbackReturn::FAILURE;
  }

  // Build topic prefix based on agent_id
  std::string topic_prefix = agent_id_.empty() ? "" : ("/" + agent_id_);

  auto qos = rclcpp::QoS(50).reliable();
  pub_bt_events_ = create_publisher<std_msgs::msg::String>(
      topic_prefix + "/executor/bt_events", qos);
  pub_status_ = create_publisher<std_msgs::msg::String>(
      topic_prefix + "/executor/status", rclcpp::QoS(1).reliable().transient_local());
  pub_bt_events_->on_activate();
  pub_status_->on_activate();
  sub_bt_xml_ = create_subscription<std_msgs::msg::String>(
      topic_prefix + "/executor/bt_xml",
      rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::String& msg) {
        try {
          loadAndExecute(msg.data);
        } catch (const std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Failed to load BT XML: %s", e.what());
          publishStatus("FAILURE");
        }
      });

  component_.setEventSink([this](const std::string& json_line) {
    if (!pub_bt_events_ || !pub_bt_events_->is_activated()) {
      return;
    }
    auto event_msg = std::make_unique<std_msgs::msg::String>();
    event_msg->data = json_line;
    pub_bt_events_->publish(std::move(event_msg));
  });

  publishStatus("IDLE");
  if (!agent_id_.empty()) {
    RCLCPP_INFO(get_logger(), "ExecutorNode activated for agent '%s'", agent_id_.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "ExecutorNode activated");
  }
  return CallbackReturn::SUCCESS;
}

ExecutorNode::CallbackReturn
ExecutorNode::on_deactivate(const rclcpp_lifecycle::State&) {
  tick_timer_.reset();
  sub_bt_xml_.reset();
  pub_bt_events_.reset();
  pub_status_.reset();
  component_.deactivate();
  return CallbackReturn::SUCCESS;
}

ExecutorNode::CallbackReturn
ExecutorNode::on_cleanup(const rclcpp_lifecycle::State&) {
  client_get_fact_.reset();
  client_set_fact_.reset();
  component_.cleanup();
  return CallbackReturn::SUCCESS;
}

ExecutorNode::CallbackReturn
ExecutorNode::on_shutdown(const rclcpp_lifecycle::State&) {
  component_.shutdown();
  return CallbackReturn::SUCCESS;
}

void ExecutorNode::registerCoreNodes() {
  if (core_nodes_registered_) {
    return;
  }

  if (inprocess_wm_) {
    component_.factory().registerNodeType<ame::CheckWorldPredicate>("CheckWorldPredicate");
    component_.factory().registerNodeType<ame::SetWorldPredicate>("SetWorldPredicate");
  } else {
    component_.factory().registerNodeType<RosCheckWorldPredicate>("CheckWorldPredicate");
    component_.factory().registerNodeType<RosSetWorldPredicate>("SetWorldPredicate");
  }

  core_nodes_registered_ = true;
}

void ExecutorNode::loadAndExecute(const std::string& bt_xml) {
  tick_timer_.reset();
  component_.loadAndExecute(bt_xml);
  publishStatus("RUNNING");

  double rate_hz = component_.tickRateHz();
  if (rate_hz <= 0.0) {
    rate_hz = 50.0;
  }
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz));

  tick_timer_ = create_wall_timer(period, [this]() { tickOnce(); });

  RCLCPP_INFO(get_logger(), "BT loaded and ticking at %.1f Hz", rate_hz);
}

void ExecutorNode::tickOnce() {
  component_.tickOnce();

  if (!component_.isExecuting()) {
    tick_timer_.reset();
    const std::string status_str =
      (component_.lastStatus() == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE";
    publishStatus(status_str);
    RCLCPP_INFO(get_logger(), "BT execution finished: %s", status_str.c_str());
  }
}

void ExecutorNode::publishStatus(const std::string& status_str) {
  if (!pub_status_ || !pub_status_->is_activated()) {
    return;
  }
  auto status_msg = std::make_unique<std_msgs::msg::String>();
  status_msg->data = status_str;
  pub_status_->publish(std::move(status_msg));
}

} // namespace ame_ros2
