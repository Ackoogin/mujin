#include "mujin_ros2/ros_wm_bridge.hpp"

#include <chrono>

namespace mujin_ros2 {

// ---------------------------------------------------------------------------
// RosCheckWorldPredicate
// ---------------------------------------------------------------------------

RosCheckWorldPredicate::RosCheckWorldPredicate(const std::string& name,
                                               const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{}

BT::PortsList RosCheckWorldPredicate::providedPorts() {
    return {
        BT::InputPort<std::string>("predicate"),
        BT::InputPort<bool>("expected", true, "expected truth value"),
    };
}

BT::NodeStatus RosCheckWorldPredicate::tick() {
    auto pred = getInput<std::string>("predicate");
    if (!pred) return BT::NodeStatus::FAILURE;

    bool expected = true;
    getInput("expected", expected);

    auto* client = config().blackboard->get<
        rclcpp::Client<mujin_ros2::srv::GetFact>*>("get_fact_client");
    if (!client) return BT::NodeStatus::FAILURE;

    auto req  = std::make_shared<mujin_ros2::srv::GetFact::Request>();
    req->key  = pred.value();

    auto future = client->async_send_request(req);
    if (future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
        return BT::NodeStatus::FAILURE;
    }

    auto res = future.get();
    return (res->found && res->value == expected)
        ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// RosSetWorldPredicate
// ---------------------------------------------------------------------------

RosSetWorldPredicate::RosSetWorldPredicate(const std::string& name,
                                           const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{}

BT::PortsList RosSetWorldPredicate::providedPorts() {
    return {
        BT::InputPort<std::string>("predicate"),
        BT::InputPort<bool>("value", true, "value to set"),
    };
}

BT::NodeStatus RosSetWorldPredicate::tick() {
    auto pred = getInput<std::string>("predicate");
    if (!pred) return BT::NodeStatus::FAILURE;

    bool value = true;
    getInput("value", value);

    auto* client = config().blackboard->get<
        rclcpp::Client<mujin_ros2::srv::SetFact>*>("set_fact_client");
    if (!client) return BT::NodeStatus::FAILURE;

    auto req     = std::make_shared<mujin_ros2::srv::SetFact::Request>();
    req->key     = pred.value();
    req->value   = value;
    req->source  = std::string("SetWorldPredicate:") + name();

    auto future = client->async_send_request(req);
    if (future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
        return BT::NodeStatus::FAILURE;
    }

    auto res = future.get();
    return res->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace mujin_ros2
