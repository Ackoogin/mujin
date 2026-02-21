#include "mujin_ros2/executor_node.hpp"
#include "mujin_ros2/ros_wm_bridge.hpp"

#include "mujin/bt_nodes/check_world_predicate.h"
#include "mujin/bt_nodes/set_world_predicate.h"

#include <behaviortree_cpp/loggers/bt_observer.h>
#include <std_msgs/msg/string.hpp>

#include <chrono>

namespace mujin_ros2 {

ExecutorNode::ExecutorNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("executor_node", options)
{}

ExecutorNode::CallbackReturn
ExecutorNode::on_configure(const rclcpp_lifecycle::State&) {
    declare_parameter("tick_rate_hz", 50.0);
    declare_parameter("bt_log.enabled", true);
    declare_parameter("bt_log.path", std::string("bt_events.jsonl"));
    declare_parameter("bt_log.tree_id", std::string("MissionPlan"));
    declare_parameter("world_model_node", std::string("world_model_node"));

    const auto wm_ns = get_parameter("world_model_node").as_string();
    client_get_fact_ = create_client<mujin_ros2::srv::GetFact>("/" + wm_ns + "/get_fact");
    client_set_fact_ = create_client<mujin_ros2::srv::SetFact>("/" + wm_ns + "/set_fact");

    // Register BT node types based on mode
    registerCoreNodes();

    RCLCPP_INFO(get_logger(), "ExecutorNode configured");
    return CallbackReturn::SUCCESS;
}

ExecutorNode::CallbackReturn
ExecutorNode::on_activate(const rclcpp_lifecycle::State&) {
    auto qos = rclcpp::QoS(50).reliable();
    pub_bt_events_ = create_publisher<std_msgs::msg::String>("/executor/bt_events", qos);
    pub_status_    = create_publisher<std_msgs::msg::String>("/executor/status",
                                                              rclcpp::QoS(1).reliable().transient_local());

    publishStatus("IDLE");
    RCLCPP_INFO(get_logger(), "ExecutorNode activated");
    return CallbackReturn::SUCCESS;
}

ExecutorNode::CallbackReturn
ExecutorNode::on_deactivate(const rclcpp_lifecycle::State&) {
    tick_timer_.reset();
    executing_ = false;
    pub_bt_events_.reset();
    pub_status_.reset();
    return CallbackReturn::SUCCESS;
}

ExecutorNode::CallbackReturn
ExecutorNode::on_cleanup(const rclcpp_lifecycle::State&) {
    tree_.reset();
    bt_logger_.reset();
    client_get_fact_.reset();
    client_set_fact_.reset();
    return CallbackReturn::SUCCESS;
}

ExecutorNode::CallbackReturn
ExecutorNode::on_shutdown(const rclcpp_lifecycle::State&) {
    if (bt_logger_) bt_logger_->flush();
    return CallbackReturn::SUCCESS;
}

void ExecutorNode::registerCoreNodes() {
    if (inprocess_wm_) {
        // In-process: use direct WorldModel pointer variants (no service overhead)
        factory_.registerNodeType<mujin::CheckWorldPredicate>("CheckWorldPredicate");
        factory_.registerNodeType<mujin::SetWorldPredicate>("SetWorldPredicate");
    } else {
        // Distributed: use service-backed variants
        factory_.registerNodeType<RosCheckWorldPredicate>("CheckWorldPredicate");
        factory_.registerNodeType<RosSetWorldPredicate>("SetWorldPredicate");
    }
}

void ExecutorNode::loadAndExecute(const std::string& bt_xml) {
    // Stop any running tree
    tick_timer_.reset();
    tree_.reset();
    bt_logger_.reset();
    executing_ = false;

    // Create tree from XML
    tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(bt_xml));

    // Set blackboard entries for BT node access
    if (inprocess_wm_) {
        tree_->rootBlackboard()->set("world_model", inprocess_wm_);
    } else {
        tree_->rootBlackboard()->set("get_fact_client", client_get_fact_.get());
        tree_->rootBlackboard()->set("set_fact_client", client_set_fact_.get());
    }

    // Attach MujinBTLogger (Layer 2)
    const auto tree_id = get_parameter("bt_log.tree_id").as_string();
    bt_logger_ = std::make_unique<mujin::MujinBTLogger>(
        *tree_, tree_id, inprocess_wm_);

    if (get_parameter("bt_log.enabled").as_bool()) {
        bt_logger_->addFileSink(get_parameter("bt_log.path").as_string());
    }

    // Publish BT events as ROS2 topic
    if (pub_bt_events_ && pub_bt_events_->is_activated()) {
        bt_logger_->addCallbackSink([this](const std::string& json_line) {
            auto msg  = std::make_unique<std_msgs::msg::String>();
            msg->data = json_line;
            pub_bt_events_->publish(std::move(msg));
        });
    }

    // Start tick timer
    executing_   = true;
    last_status_ = BT::NodeStatus::RUNNING;
    publishStatus("RUNNING");

    double rate_hz = get_parameter("tick_rate_hz").as_double();
    if (rate_hz <= 0.0) rate_hz = 50.0;
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz));

    tick_timer_ = create_wall_timer(period, [this]() { tickOnce(); });

    RCLCPP_INFO(get_logger(), "BT loaded and ticking at %.1f Hz", rate_hz);
}

void ExecutorNode::tickOnce() {
    if (!tree_ || !executing_) return;

    last_status_ = tree_->tickOnce();

    if (last_status_ != BT::NodeStatus::RUNNING) {
        tick_timer_.reset();
        executing_ = false;
        if (bt_logger_) bt_logger_->flush();

        const std::string status_str =
            (last_status_ == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE";
        publishStatus(status_str);
        RCLCPP_INFO(get_logger(), "BT execution finished: %s", status_str.c_str());
    }
}

void ExecutorNode::publishStatus(const std::string& status_str) {
    if (!pub_status_ || !pub_status_->is_activated()) return;
    auto msg  = std::make_unique<std_msgs::msg::String>();
    msg->data = status_str;
    pub_status_->publish(std::move(msg));
}

} // namespace mujin_ros2
