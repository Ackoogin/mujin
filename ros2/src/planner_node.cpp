#include "mujin_ros2/planner_node.hpp"
#include "mujin/pddl_parser.h"

#include <chrono>
#include <stdexcept>

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

    domain_file_  = get_parameter("domain.pddl_file").as_string();
    problem_file_ = get_parameter("domain.problem_file").as_string();

    if (domain_file_.empty() || problem_file_.empty()) {
        RCLCPP_WARN(get_logger(),
            "domain.pddl_file / domain.problem_file not set. "
            "Planning will only work in in-process mode.");
    }

    if (get_parameter("plan_audit.enabled").as_bool()) {
        plan_audit_ = mujin::PlanAuditLog(get_parameter("plan_audit.path").as_string());
    }

    // QueryState client for distributed mode
    const auto wm_ns = get_parameter("world_model_node").as_string();
    client_query_state_ = create_client<mujin_ros2::srv::QueryState>(
        "/" + wm_ns + "/query_state");

    RCLCPP_INFO(get_logger(), "PlannerNode configured");
    return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn
PlannerNode::on_activate(const rclcpp_lifecycle::State&) {
    action_server_ = rclcpp_action::create_server<PlanAction>(
        this,
        "/mujin/plan",
        [this](const auto& uuid, auto goal) { return handleGoal(uuid, goal); },
        [this](auto gh)                      { return handleCancel(gh); },
        [this](auto gh)                      { handleAccepted(gh); });

    RCLCPP_INFO(get_logger(), "PlannerNode activated — action server at /mujin/plan");
    return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn
PlannerNode::on_deactivate(const rclcpp_lifecycle::State&) {
    action_server_.reset();
    RCLCPP_INFO(get_logger(), "PlannerNode deactivated");
    return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn
PlannerNode::on_cleanup(const rclcpp_lifecycle::State&) {
    client_query_state_.reset();
    plan_audit_.flush();
    return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn
PlannerNode::on_shutdown(const rclcpp_lifecycle::State&) {
    plan_audit_.flush();
    return CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse PlannerNode::handleGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const PlanAction::Goal> goal)
{
    if (goal->goal_fluents.empty()) {
        RCLCPP_WARN(get_logger(), "Plan goal has no goal_fluents — rejecting");
        return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "Accepting plan goal (%zu goal fluents)",
                goal->goal_fluents.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlannerNode::handleCancel(
    std::shared_ptr<GoalHandlePlan>)
{
    RCLCPP_INFO(get_logger(), "Plan cancellation requested");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerNode::handleAccepted(std::shared_ptr<GoalHandlePlan> goal_handle) {
    // Run planning on a dedicated thread so the ROS2 executor is not blocked
    std::thread([this, goal_handle]() {
        executePlan(goal_handle);
    }).detach();
}

void PlannerNode::executePlan(std::shared_ptr<GoalHandlePlan> goal_handle) {
    const auto& goal = goal_handle->get_goal();
    auto result = std::make_shared<PlanAction::Result>();

    RCLCPP_INFO(get_logger(), "Planning started (%zu goal fluents)",
                goal->goal_fluents.size());

    // Build a local WorldModel snapshot for this planning episode
    std::unique_ptr<mujin::WorldModel> local_wm;
    try {
        local_wm = snapshotWorldModel(goal->goal_fluents);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to snapshot WorldModel: %s", e.what());
        result->success   = false;
        result->error_msg = e.what();
        goal_handle->abort(result);
        return;
    }

    // Stream initial feedback
    auto feedback = std::make_shared<PlanAction::Feedback>();
    feedback->status_msg     = "Solving...";
    feedback->nodes_expanded = 0;
    feedback->nodes_generated = 0;
    feedback->elapsed_ms     = 0.0;
    goal_handle->publish_feedback(feedback);

    // Run BRFS
    const auto plan_result = planner_.solve(*local_wm);

    if (goal_handle->is_canceling()) {
        result->success   = false;
        result->error_msg = "Cancelled";
        goal_handle->canceled(result);
        return;
    }

    result->success      = plan_result.success;
    result->solve_time_ms = plan_result.solve_time_ms;
    result->expanded     = plan_result.expanded;
    result->generated    = plan_result.generated;

    if (!plan_result.success) {
        result->error_msg = "No plan found";
        RCLCPP_WARN(get_logger(), "Planning failed (no plan found)");
        goal_handle->abort(result);
        return;
    }

    // Build action signatures and indices
    for (const auto& step : plan_result.steps) {
        const auto& ga = local_wm->groundActions()[step.action_index];
        result->plan_actions.push_back(ga.signature);
        result->action_indices.push_back(step.action_index);
    }

    // Compile to BT XML
    const bool use_parallel = get_parameter("compiler.parallel").as_bool();
    if (use_parallel) {
        result->bt_xml = compiler_.compile(plan_result.steps, *local_wm, registry_);
    } else {
        result->bt_xml = compiler_.compileSequential(plan_result.steps, *local_wm, registry_);
    }

    // Record plan audit trail (Layer 5)
    {
        mujin::PlanAuditLog::Episode ep;
        ep.ts_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        for (unsigned i = 0; i < local_wm->numFluents(); ++i) {
            if (local_wm->getFact(i)) ep.init_facts.push_back(local_wm->fluentName(i));
        }
        for (auto gid : local_wm->goalFluentIds()) {
            ep.goal_fluents.push_back(local_wm->fluentName(gid));
        }
        ep.solver        = "BRFS";
        ep.solve_time_ms = plan_result.solve_time_ms;
        ep.success       = plan_result.success;
        ep.expanded      = plan_result.expanded;
        ep.generated     = plan_result.generated;
        ep.cost          = plan_result.cost;
        for (const auto& sig : result->plan_actions) ep.plan_actions.push_back(sig);
        ep.bt_xml = result->bt_xml;
        plan_audit_.recordEpisode(ep);
        plan_audit_.flush();
    }

    RCLCPP_INFO(get_logger(), "Plan found: %zu steps, %.1f ms",
                plan_result.steps.size(), plan_result.solve_time_ms);
    goal_handle->succeed(result);
}

std::unique_ptr<mujin::WorldModel> PlannerNode::snapshotWorldModel(
    const std::vector<std::string>& goal_fluents)
{
    auto wm = std::make_unique<mujin::WorldModel>();

    // Load domain structure (types, objects, predicates, actions) from PDDL
    if (!domain_file_.empty() && !problem_file_.empty()) {
        mujin::PddlParser::parse(domain_file_, problem_file_, *wm);
        // Clear all facts set by problem init — we will override from live state
        for (unsigned i = 0; i < wm->numFluents(); ++i) {
            wm->setFact(i, false, "planner_snapshot_reset");
        }
    } else if (inprocess_wm_) {
        // In-process: rebuild domain from canonical WM's registered facts
        // (We cannot copy WorldModel directly, so we use PDDL parsing fallback.
        // For in-process mode without PDDL files, the caller must set domain files.)
        RCLCPP_WARN(get_logger(),
            "In-process mode without PDDL files: snapshot may be incomplete");
    }

    // Apply current live state
    if (inprocess_wm_) {
        // In-process: read directly
        for (unsigned i = 0; i < inprocess_wm_->numFluents(); ++i) {
            try {
                wm->setFact(inprocess_wm_->fluentName(i),
                            inprocess_wm_->getFact(i), "snapshot");
            } catch (...) {}
        }
    } else {
        // Distributed: call QueryState service
        if (!client_query_state_->wait_for_service(std::chrono::seconds(5))) {
            throw std::runtime_error("QueryState service not available");
        }
        auto req = std::make_shared<mujin_ros2::srv::QueryState::Request>();
        // empty keys = return all true fluents
        auto future = client_query_state_->async_send_request(req);
        if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            throw std::runtime_error("QueryState service timed out");
        }
        auto res = future.get();
        for (const auto& fact : res->facts) {
            try {
                wm->setFact(fact.key, fact.value, "snapshot");
            } catch (...) {}
        }
    }

    // Set goal
    wm->setGoal(goal_fluents);

    return wm;
}

} // namespace mujin_ros2
