#pragma once

#include "mujin/world_model.h"
#include "mujin/wm_audit_log.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "mujin_ros2/msg/world_state.hpp"
#include "mujin_ros2/srv/get_fact.hpp"
#include "mujin_ros2/srv/set_fact.hpp"
#include "mujin_ros2/srv/query_state.hpp"

#include <atomic>
#include <mutex>
#include <optional>

namespace mujin_ros2 {

/// ROS2 lifecycle node that owns a mujin::WorldModel and exposes it via services.
///
/// Services (available after on_configure):
///   ~/get_fact      (mujin_ros2/srv/GetFact)
///   ~/set_fact      (mujin_ros2/srv/SetFact)
///   ~/query_state   (mujin_ros2/srv/QueryState)
///
/// Publisher (active after on_activate):
///   /world_state    (mujin_ros2/msg/WorldState) — QoS: reliable, transient_local
///
/// Parameters:
///   domain.pddl_file    (string, "")    — path to PDDL domain file
///   domain.problem_file (string, "")    — path to PDDL problem file
///   audit_log.enabled   (bool, true)
///   audit_log.path      (string, "wm_audit.jsonl")
///   publish_rate_hz     (double, 10.0)
class WorldModelNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit WorldModelNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev) override;

    /// Direct access for in-process (combined_main) mode.
    mujin::WorldModel& worldModel() { return wm_; }
    const mujin::WorldModel& worldModel() const { return wm_; }

private:
    mujin::WorldModel  wm_;
    std::optional<mujin::WmAuditLog> wm_audit_;

    // Services
    rclcpp::Service<mujin_ros2::srv::GetFact>::SharedPtr    srv_get_fact_;
    rclcpp::Service<mujin_ros2::srv::SetFact>::SharedPtr    srv_set_fact_;
    rclcpp::Service<mujin_ros2::srv::QueryState>::SharedPtr srv_query_state_;

    // Publisher + debounce timer
    rclcpp_lifecycle::LifecyclePublisher<mujin_ros2::msg::WorldState>::SharedPtr pub_world_state_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::atomic<bool> state_dirty_{false};

    // Service handlers
    void handleGetFact(
        std::shared_ptr<mujin_ros2::srv::GetFact::Request>  req,
        std::shared_ptr<mujin_ros2::srv::GetFact::Response> res);

    void handleSetFact(
        std::shared_ptr<mujin_ros2::srv::SetFact::Request>  req,
        std::shared_ptr<mujin_ros2::srv::SetFact::Response> res);

    void handleQueryState(
        std::shared_ptr<mujin_ros2::srv::QueryState::Request>  req,
        std::shared_ptr<mujin_ros2::srv::QueryState::Response> res);

    void publishWorldState();
    void loadDomainFromParams();
};

} // namespace mujin_ros2
