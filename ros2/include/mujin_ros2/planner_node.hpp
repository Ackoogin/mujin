#pragma once

#include "mujin/world_model.h"
#include "mujin/planner.h"
#include "mujin/plan_compiler.h"
#include "mujin/action_registry.h"
#include "mujin/plan_audit_log.h"
#include "mujin/pddl_parser.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "mujin_ros2/action/plan.hpp"
#include "mujin_ros2/srv/query_state.hpp"

#include <memory>
#include <thread>

namespace mujin_ros2 {

/// ROS2 lifecycle node that runs the LAPKT BRFS planner as an action server.
///
/// Action server (active after on_activate):
///   /mujin/plan     (mujin_ros2/action/Plan)
///
/// Parameters:
///   domain.pddl_file       (string, "") — must match WorldModelNode
///   domain.problem_file    (string, "") — must match WorldModelNode
///   world_model_node       (string, "world_model_node") — namespace for QueryState client
///   plan_audit.enabled     (bool, true)
///   plan_audit.path        (string, "plan_audit.jsonl")
///   compiler.parallel      (bool, false) — use causal-graph compiler
///   action_registry.<name> (string) — maps PDDL action name to BT node type
class PlannerNode : public rclcpp_lifecycle::LifecycleNode {
public:
    using PlanAction     = mujin_ros2::action::Plan;
    using GoalHandlePlan = rclcpp_action::ServerGoalHandle<PlanAction>;

    explicit PlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~PlannerNode();

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev) override;

    /// For in-process mode: inject the canonical WorldModel directly.
    /// When set, snapshotWorldModel() copies state from this pointer instead of
    /// calling the QueryState service.
    void setInProcessWorldModel(mujin::WorldModel* wm) { inprocess_wm_ = wm; }

    /// Expose ActionRegistry so combined_main can register action node types.
    mujin::ActionRegistry& actionRegistry() { return registry_; }

private:
    mujin::Planner       planner_;
    mujin::PlanCompiler  compiler_;
    mujin::ActionRegistry registry_;
    mujin::PlanAuditLog  plan_audit_;

    // In-process mode: direct WorldModel pointer (null = use service)
    mujin::WorldModel* inprocess_wm_ = nullptr;

    // PDDL file paths (used to reconstruct local WM structure for planning)
    std::string domain_file_;
    std::string problem_file_;

    // Action server
    rclcpp_action::Server<PlanAction>::SharedPtr action_server_;

    // Client to WorldModelNode QueryState service (distributed mode)
    rclcpp::Client<mujin_ros2::srv::QueryState>::SharedPtr client_query_state_;

    // Action server callbacks
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const PlanAction::Goal> goal);

    rclcpp_action::CancelResponse handleCancel(
        std::shared_ptr<GoalHandlePlan> goal_handle);

    void handleAccepted(std::shared_ptr<GoalHandlePlan> goal_handle);

    // Runs on a dedicated thread — blocks during BRFS solve
    void executePlan(std::shared_ptr<GoalHandlePlan> goal_handle);

    // Build a local WorldModel snapshot for planning
    std::unique_ptr<mujin::WorldModel> snapshotWorldModel(
        const std::vector<std::string>& goal_fluents);
};

} // namespace mujin_ros2
