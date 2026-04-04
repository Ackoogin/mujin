#include <ame_ros2/world_model_node.hpp>

#include <ame/world_model.h>
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
  declare_parameter("perception.enabled", true);
  declare_parameter("perception.confidence_threshold", 0.5);

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

  using LoadDomain = ame_ros2::srv::LoadDomain;
  srv_load_domain_ = create_service<LoadDomain>(
    "~/load_domain",
    [this](std::shared_ptr<LoadDomain::Request> req,
           std::shared_ptr<LoadDomain::Response> res) { handleLoadDomain(req, res); });

  // Perception integration
  perception_confidence_threshold_ = get_parameter("perception.confidence_threshold").as_double();
  if (get_parameter("perception.enabled").as_bool()) {
    sub_detections_ = create_subscription<ame_ros2::msg::Detection>(
      "/detections", rclcpp::SensorDataQoS(),
      [this](const ame_ros2::msg::Detection::SharedPtr msg) { handleDetection(msg); });
    RCLCPP_INFO(get_logger(), "Perception integration enabled (confidence >= %.2f)",
                perception_confidence_threshold_);
  }

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
    size_t applied = component_.worldModel().applyQueuedMutations();
    if (applied > 0 || component_.consumeStateDirty()) {
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
  sub_detections_.reset();
  component_.deactivate();
  RCLCPP_INFO(get_logger(), "WorldModelNode deactivated");
  return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn
WorldModelNode::on_cleanup(const rclcpp_lifecycle::State&) {
  srv_get_fact_.reset();
  srv_set_fact_.reset();
  srv_query_state_.reset();
  srv_load_domain_.reset();
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
  RCLCPP_INFO(get_logger(), "get_fact called (key='%s')", req->key.c_str());
  const auto result = component_.getFact(req->key);
  res->value = result.value;
  res->found = result.found;
  res->wm_version = result.wm_version;
}

void WorldModelNode::handleSetFact(
  std::shared_ptr<ame_ros2::srv::SetFact::Request> req,
  std::shared_ptr<ame_ros2::srv::SetFact::Response> res) {
  RCLCPP_INFO(get_logger(), "set_fact called (key='%s' value=%s source='%s')",
              req->key.c_str(), req->value ? "true" : "false", req->source.c_str());
  const auto result = component_.setFact(req->key, req->value, req->source);
  res->success = result.success;
  res->wm_version = result.wm_version;
}

void WorldModelNode::handleQueryState(
  std::shared_ptr<ame_ros2::srv::QueryState::Request> req,
  std::shared_ptr<ame_ros2::srv::QueryState::Response> res) {
  RCLCPP_INFO(get_logger(), "query_state called (keys=%zu)", req->keys.size());
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

void WorldModelNode::handleLoadDomain(
  std::shared_ptr<ame_ros2::srv::LoadDomain::Request> req,
  std::shared_ptr<ame_ros2::srv::LoadDomain::Response> res) {
  RCLCPP_INFO(get_logger(), "load_domain called (domain_id='%s', %zu bytes domain, %zu bytes problem)",
              req->domain_id.c_str(), req->domain_pddl.size(), req->problem_pddl.size());

  auto result = component_.loadDomainFromStrings(req->domain_pddl, req->problem_pddl);
  res->success = result.success;
  res->error_msg = result.error_msg;
  res->num_fluents = result.num_fluents;
  res->num_ground_actions = result.num_ground_actions;

  if (result.success) {
    RCLCPP_INFO(get_logger(), "Domain '%s' loaded into WorldModel: %u fluents, %u ground actions",
                req->domain_id.c_str(), result.num_fluents, result.num_ground_actions);
  } else {
    RCLCPP_ERROR(get_logger(), "Domain load failed: %s", result.error_msg.c_str());
  }
}

void WorldModelNode::handleDetection(const ame_ros2::msg::Detection::SharedPtr msg) {
  // Filter by confidence threshold
  if (msg->confidence < perception_confidence_threshold_) {
    RCLCPP_DEBUG(get_logger(), "Ignoring detection '%s' (confidence %.2f < %.2f)",
                 msg->entity_id.c_str(), msg->confidence, perception_confidence_threshold_);
    return;
  }

  // Map detection properties to world model facts with CONFIRMED authority
  auto& wm = component_.worldModel();
  const std::string source = "perception:" + msg->sensor_source;

  for (size_t i = 0; i < msg->property_keys.size() && i < msg->property_values.size(); ++i) {
    // Build PDDL-style fact key: (predicate entity_id arg...)
    std::string fact_key = "(" + msg->property_keys[i] + " " + msg->entity_id;
    if (!msg->property_values[i].empty()) {
      fact_key += " " + msg->property_values[i];
    }
    fact_key += ")";

    try {
      // Check for authority conflict before setting
      unsigned fluent_id = wm.fluentIndex(fact_key);
      bool perceived_value = true;  // Detection implies predicate is true

      if (wm.hasAuthorityConflict(fluent_id, perceived_value)) {
        RCLCPP_WARN(get_logger(),
          "Authority conflict: perception says '%s' is %s but plan predicted otherwise",
          fact_key.c_str(), perceived_value ? "true" : "false");
      }

      wm.enqueueMutation(fluent_id, true, source, ame::FactAuthority::CONFIRMED);
      RCLCPP_DEBUG(get_logger(), "Perception enqueued CONFIRMED fact: %s", fact_key.c_str());
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Failed to set perception fact '%s': %s",
                  fact_key.c_str(), e.what());
    }
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
