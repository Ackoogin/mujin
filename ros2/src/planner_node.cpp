#include <mujin_ros2/planner_node.hpp>

#include <chrono>
#include <stdexcept>
#include <thread>

namespace mujin_ros2 {

PlannerNode::PlannerNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("planner_node", options)
{}

PlannerNode::~PlannerNode() = default;

PlannerNode::CallbackReturn
PlannerNode::on_configure(const rclcpp_lifecycle::State&) {
  declare_parameter("domain.pddl_file", "");
  declare_parameter("domain.problem_file", "");
  declare_parameter("world_model_node", std::string("world_model_node"));
  declare_parameter("plan_audit.enabled", true);
  declare_parameter("plan_audit.path", std::string("plan_audit.jsonl"));
  declare_parameter("compiler.parallel", false);

  const auto domain_file = get_parameter("domain.pddl_file").as_string();
  const auto problem_file = get_parameter("domain.problem_file").as_string();
  if (domain_file.empty() || problem_file.empty()) {
    RCLCPP_WARN(
        get_logger(),
        "domain.pddl_file / domain.problem_file not set. Planning will only work "
        "in in-process mode.");
  }

  const auto wm_ns = get_parameter("world_model_node").as_string();
  client_query_state_ =
      create_client<mujin_ros2::srv::QueryState>("/" + wm_ns + "/query_state");

  component_.setParam("domain.pddl_file", domain_file.c_str());
  component_.setParam("domain.problem_file", problem_file.c_str());
  component_.setParam("plan_audit.enabled", get_parameter("plan_audit.enabled").as_bool());
  component_.setParam("plan_audit.path", get_parameter("plan_audit.path").as_string().c_str());
  component_.setParam("compiler.parallel", get_parameter("compiler.parallel").as_bool());
  component_.setInProcessWorldModel(inprocess_wm_);
  component_.setQueryStateCallback([this]() { return queryWorldState(); });

  if (component_.configure() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to configure planner component");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "PlannerNode configured");
  return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn
PlannerNode::on_activate(const rclcpp_lifecycle::State&) {
  if (component_.activate() != PCL_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to activate planner component");
    return CallbackReturn::FAILURE;
  }

  action_server_ = rclcpp_action::create_server<PlanAction>(
      this,
      "/mujin/plan",
      [this](const auto& uuid, auto goal) { return handleGoal(uuid, goal); },
      [this](auto gh) { return handleCancel(gh); },
      [this](auto gh) { handleAccepted(gh); });
  pub_bt_xml_ =
      create_publisher<std_msgs::msg::String>("/executor/bt_xml", rclcpp::QoS(10).reliable());
  pub_bt_xml_->on_activate();

  RCLCPP_INFO(get_logger(), "PlannerNode activated - action server at /mujin/plan");
  return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn
PlannerNode::on_deactivate(const rclcpp_lifecycle::State&) {
  action_server_.reset();
  pub_bt_xml_.reset();
  component_.deactivate();
  RCLCPP_INFO(get_logger(), "PlannerNode deactivated");
  return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn
PlannerNode::on_cleanup(const rclcpp_lifecycle::State&) {
  client_query_state_.reset();
  component_.cleanup();
  return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn
PlannerNode::on_shutdown(const rclcpp_lifecycle::State&) {
  component_.shutdown();
  return CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse PlannerNode::handleGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const PlanAction::Goal> goal) {
  if (goal->goal_fluents.empty()) {
    RCLCPP_WARN(get_logger(), "Plan goal has no goal_fluents - rejecting");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(
      get_logger(), "Accepting plan goal (%zu goal fluents)", goal->goal_fluents.size());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlannerNode::handleCancel(std::shared_ptr<GoalHandlePlan>) {
  RCLCPP_INFO(get_logger(), "Plan cancellation requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerNode::handleAccepted(std::shared_ptr<GoalHandlePlan> goal_handle) {
  // Run planning on a dedicated thread so the ROS2 executor is not blocked.
  std::thread([this, goal_handle]() { executePlan(goal_handle); }).detach();
}

void PlannerNode::executePlan(std::shared_ptr<GoalHandlePlan> goal_handle) {
  const auto& goal = goal_handle->get_goal();
  auto result = std::make_shared<PlanAction::Result>();

  RCLCPP_INFO(get_logger(), "Planning started (%zu goal fluents)", goal->goal_fluents.size());

  auto feedback = std::make_shared<PlanAction::Feedback>();
  feedback->status_msg = "Solving...";
  feedback->nodes_expanded = 0;
  feedback->nodes_generated = 0;
  feedback->elapsed_ms = 0.0;
  goal_handle->publish_feedback(feedback);

  mujin::PlannerExecutionResult execution_result;
  try {
    execution_result = component_.solveGoal(goal->goal_fluents);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Planning failed: %s", e.what());
    result->success = false;
    result->error_msg = e.what();
    goal_handle->abort(result);
    return;
  }

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->error_msg = "Cancelled";
    goal_handle->canceled(result);
    return;
  }

  result->success = execution_result.success;
  result->solve_time_ms = execution_result.solve_time_ms;
  result->expanded = execution_result.expanded;
  result->generated = execution_result.generated;

  if (!execution_result.success) {
    result->error_msg = execution_result.error_msg;
    RCLCPP_WARN(get_logger(), "Planning failed (no plan found)");
    goal_handle->abort(result);
    return;
  }

  result->plan_actions = execution_result.plan_actions;
  result->action_indices = execution_result.action_indices;
  result->bt_xml = execution_result.bt_xml;

  if (pub_bt_xml_ && pub_bt_xml_->is_activated()) {
    std_msgs::msg::String bt_xml_msg;
    bt_xml_msg.data = execution_result.bt_xml;
    pub_bt_xml_->publish(bt_xml_msg);
  }

  RCLCPP_INFO(
      get_logger(),
      "Plan found: %zu steps, %.1f ms",
      execution_result.plan_actions.size(),
      execution_result.solve_time_ms);
  goal_handle->succeed(result);
}

mujin::WorldStateSnapshot PlannerNode::queryWorldState() const {
  mujin::WorldStateSnapshot snapshot;

  if (inprocess_wm_ != nullptr) {
    snapshot.wm_version = inprocess_wm_->version();
    for (unsigned i = 0; i < inprocess_wm_->numFluents(); ++i) {
      if (!inprocess_wm_->getFact(i)) {
        continue;
      }
      mujin::WorldFactValue fact;
      fact.key = inprocess_wm_->fluentName(i);
      fact.value = true;
      fact.wm_version = inprocess_wm_->version();
      snapshot.facts.push_back(fact);
    }
    return snapshot;
  }

  if (!client_query_state_->wait_for_service(std::chrono::seconds(5))) {
    throw std::runtime_error("QueryState service not available");
  }

  auto req = std::make_shared<mujin_ros2::srv::QueryState::Request>();
  auto future = client_query_state_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    throw std::runtime_error("QueryState service timed out");
  }

  auto res = future.get();
  snapshot.success = res->success;
  snapshot.wm_version = res->wm_version;
  for (const auto& msg_fact : res->facts) {
    mujin::WorldFactValue fact;
    fact.key = msg_fact.key;
    fact.value = msg_fact.value;
    fact.source = msg_fact.source;
    fact.wm_version = msg_fact.wm_version;
    snapshot.facts.push_back(fact);
  }
  return snapshot;
}

} // namespace mujin_ros2
