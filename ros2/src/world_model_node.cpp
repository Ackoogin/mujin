#include <ame_ros2/world_model_node.hpp>

#include <rclcpp/qos.hpp>

#include <chrono>

namespace ame_ros2 {

WorldModelNode::WorldModelNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("world_model_node", options)
{}

WorldModelNode::CallbackReturn
WorldModelNode::on_configure(const rclcpp_lifecycle::State&) {
  declare_parameter("domain.pddl_file", "");
  declare_parameter("domain.problem_file", "");
  declare_parameter("audit_log.enabled", true);
  declare_parameter("audit_log.path", std::string("wm_audit.jsonl"));
  declare_parameter("publish_rate_hz", 10.0);

  component_.setParam("domain.pddl_file", get_parameter("domain.pddl_file").as_string().c_str());
  component_.setParam(
      "domain.problem_file", get_parameter("domain.problem_file").as_string().c_str());
  component_.setParam("audit_log.enabled", get_parameter("audit_log.enabled").as_bool());
  component_.setParam("audit_log.path", get_parameter("audit_log.path").as_string().c_str());
  component_.setParam("publish_rate_hz", get_parameter("publish_rate_hz").as_double());

  if (component_.configure() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to configure world model component");
    return CallbackReturn::FAILURE;
  }

  // Create services during configure so other nodes can query state before activation.
  using GetFact = ame_ros2::srv::GetFact;
  using SetFact = ame_ros2::srv::SetFact;
  using QueryState = ame_ros2::srv::QueryState;

  srv_get_fact_ = create_service<GetFact>(
      "~/get_fact",
      [this](std::shared_ptr<GetFact::Request> req,
             std::shared_ptr<GetFact::Response> res) { handleGetFact(req, res); });

  srv_set_fact_ = create_service<SetFact>(
      "~/set_fact",
      [this](std::shared_ptr<SetFact::Request> req,
             std::shared_ptr<SetFact::Response> res) { handleSetFact(req, res); });

  srv_query_state_ = create_service<QueryState>(
      "~/query_state",
      [this](std::shared_ptr<QueryState::Request> req,
             std::shared_ptr<QueryState::Response> res) { handleQueryState(req, res); });

  RCLCPP_INFO(
      get_logger(),
      "WorldModelNode configured: %u fluents, %u ground actions",
      component_.worldModel().numFluents(),
      component_.worldModel().numGroundActions());
  return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_activate(const rclcpp_lifecycle::State&) {
  if (component_.activate() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to activate world model component");
    return CallbackReturn::FAILURE;
  }

  auto qos = rclcpp::QoS(10).reliable().transient_local();
  pub_world_state_ = create_publisher<ame_ros2::msg::WorldState>("/world_state", qos);
  pub_world_state_->on_activate();

  double rate_hz = get_parameter("publish_rate_hz").as_double();
  if (rate_hz <= 0.0) {
    rate_hz = 10.0;
  }
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz));

  publish_timer_ = create_wall_timer(period, [this]() {
    if (component_.consumeStateDirty()) {
      publishWorldState();
    }
  });

  publishWorldState();

  RCLCPP_INFO(get_logger(), "WorldModelNode activated (publish %.1f Hz)", rate_hz);
  return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_deactivate(const rclcpp_lifecycle::State&) {
  publish_timer_.reset();
  pub_world_state_.reset();
  component_.deactivate();
  RCLCPP_INFO(get_logger(), "WorldModelNode deactivated");
  return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_cleanup(const rclcpp_lifecycle::State&) {
  srv_get_fact_.reset();
  srv_set_fact_.reset();
  srv_query_state_.reset();
  component_.cleanup();
  RCLCPP_INFO(get_logger(), "WorldModelNode cleaned up");
  return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_shutdown(const rclcpp_lifecycle::State&) {
  component_.shutdown();
  return CallbackReturn::SUCCESS;
}

void WorldModelNode::handleGetFact(
    std::shared_ptr<ame_ros2::srv::GetFact::Request> req,
    std::shared_ptr<ame_ros2::srv::GetFact::Response> res) {
  const auto result = component_.getFact(req->key);
  res->value = result.value;
  res->found = result.found;
  res->wm_version = result.wm_version;
}

void WorldModelNode::handleSetFact(
    std::shared_ptr<ame_ros2::srv::SetFact::Request> req,
    std::shared_ptr<ame_ros2::srv::SetFact::Response> res) {
  const auto result = component_.setFact(req->key, req->value, req->source);
  res->success = result.success;
  res->wm_version = result.wm_version;
}

void WorldModelNode::handleQueryState(
    std::shared_ptr<ame_ros2::srv::QueryState::Request> req,
    std::shared_ptr<ame_ros2::srv::QueryState::Response> res) {
  const auto snapshot = component_.queryState(req->keys);
  res->wm_version = snapshot.wm_version;
  res->success = snapshot.success;

  for (const auto& fact : snapshot.facts) {
    ame_ros2::msg::WorldFact msg;
    msg.key = fact.key;
    msg.value = fact.value;
    msg.source = fact.source;
    msg.wm_version = fact.wm_version;
    res->facts.push_back(msg);
  }
}

void WorldModelNode::publishWorldState() {
  if (!pub_world_state_ || !pub_world_state_->is_activated()) {
    return;
  }

  const auto snapshot = component_.queryState({});
  ame_ros2::msg::WorldState msg;
  msg.header.stamp = now();
  msg.header.frame_id = "";
  msg.wm_version = snapshot.wm_version;

  for (const auto& fact : snapshot.facts) {
    ame_ros2::msg::WorldFact fact_msg;
    fact_msg.key = fact.key;
    fact_msg.value = fact.value;
    fact_msg.source = fact.source;
    fact_msg.wm_version = fact.wm_version;
    msg.facts.push_back(fact_msg);
  }

  for (const auto& goal : snapshot.goal_fluents) {
    msg.goal_fluents.push_back(goal);
  }

  pub_world_state_->publish(msg);
}

} // namespace ame_ros2
