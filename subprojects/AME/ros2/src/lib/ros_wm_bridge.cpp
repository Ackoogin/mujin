#include "ame_ros2/ros_wm_bridge.hpp"

#include <chrono>
#include <utility>

namespace ame_ros2 {

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
    rclcpp::Client<ame_ros2::srv::GetFact>*>("get_fact_client");
  if (!client) return BT::NodeStatus::FAILURE;

  auto req  = std::make_shared<ame_ros2::srv::GetFact::Request>();
  req->key  = pred.value();

  auto future = client->async_send_request(req);
  if (future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
    return BT::NodeStatus::FAILURE;
  }

  auto res = future.get();
  return (res->found && res->value == expected)
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

RosWorldStateAccess::RosWorldStateAccess(
    rclcpp::Client<ame_ros2::srv::GetFact>::SharedPtr get_fact_client,
    rclcpp::Client<ame_ros2::srv::SetFact>::SharedPtr set_fact_client)
    : get_fact_client_(std::move(get_fact_client)),
      set_fact_client_(std::move(set_fact_client)) {}

bool RosWorldStateAccess::getFact(const std::string& key) {
  if (!get_fact_client_) {
    return false;
  }
  auto request = std::make_shared<ame_ros2::srv::GetFact::Request>();
  request->key = key;
  auto future = get_fact_client_->async_send_request(request);
  if (future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
    return false;
  }
  const auto response = future.get();
  return response->found && response->value;
}

ame::FactAuthority RosWorldStateAccess::factAuthority(const std::string& key) {
  if (!get_fact_client_) {
    return ame::FactAuthority::BELIEVED;
  }
  auto request = std::make_shared<ame_ros2::srv::GetFact::Request>();
  request->key = key;
  auto future = get_fact_client_->async_send_request(request);
  if (future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
    return ame::FactAuthority::BELIEVED;
  }
  const auto response = future.get();
  if (!response->found ||
      response->authority != ame_ros2::srv::GetFact::Response::AUTHORITY_CONFIRMED) {
    return ame::FactAuthority::BELIEVED;
  }
  return ame::FactAuthority::CONFIRMED;
}

bool RosWorldStateAccess::setFact(const std::string& key,
                                  bool value,
                                  const std::string& source) {
  if (!set_fact_client_) {
    return false;
  }
  auto request = std::make_shared<ame_ros2::srv::SetFact::Request>();
  request->key = key;
  request->value = value;
  request->source = source;
  auto future = set_fact_client_->async_send_request(request);
  if (future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
    return false;
  }
  return future.get()->success;
}

} // namespace ame_ros2
