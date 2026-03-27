#pragma once

#include <ame/world_model_component.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <ame_ros2/msg/world_state.hpp>
#include <ame_ros2/srv/get_fact.hpp>
#include <ame_ros2/srv/query_state.hpp>
#include <ame_ros2/srv/set_fact.hpp>

namespace ame_ros2 {

/// \brief ROS2 lifecycle node owning `ame::WorldModel` and exposing services.
///
/// Services (available after on_configure):
///   ~/get_fact      (ame_ros2/srv/GetFact)
///   ~/set_fact      (ame_ros2/srv/SetFact)
///   ~/query_state   (ame_ros2/srv/QueryState)
///
/// Publisher (active after on_activate):
///   /world_state    (ame_ros2/msg/WorldState) — QoS: reliable, transient_local
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

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev) override;

  /// \brief Direct world-model access for in-process mode.
  ame::WorldModel& worldModel() { return component_.worldModel(); }
  const ame::WorldModel& worldModel() const { return component_.worldModel(); }

private:
  ame::WorldModelComponent component_;

  // Services
  rclcpp::Service<ame_ros2::srv::GetFact>::SharedPtr srv_get_fact_;
  rclcpp::Service<ame_ros2::srv::SetFact>::SharedPtr srv_set_fact_;
  rclcpp::Service<ame_ros2::srv::QueryState>::SharedPtr srv_query_state_;

  // Publisher + debounce timer
  rclcpp_lifecycle::LifecyclePublisher<ame_ros2::msg::WorldState>::SharedPtr
    pub_world_state_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Service handlers
  void handleGetFact(
    std::shared_ptr<ame_ros2::srv::GetFact::Request> req,
    std::shared_ptr<ame_ros2::srv::GetFact::Response> res);

  void handleSetFact(
    std::shared_ptr<ame_ros2::srv::SetFact::Request> req,
    std::shared_ptr<ame_ros2::srv::SetFact::Response> res);

  void handleQueryState(
    std::shared_ptr<ame_ros2::srv::QueryState::Request> req,
    std::shared_ptr<ame_ros2::srv::QueryState::Response> res);

  void publishWorldState();
};

} // namespace ame_ros2
