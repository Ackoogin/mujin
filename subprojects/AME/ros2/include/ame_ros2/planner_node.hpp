#pragma once

#include <ame/planner_component.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace ame_ros2 {

/// \brief Thin ROS2 lifecycle wrapper for ame::PlannerComponent.
///
/// All planning logic, service handling, and port wiring live in the PCL
/// component.  This node only bridges ROS2 lifecycle transitions and the
/// ROS2 parameter system.
///
/// The component creates these PCL ports during on_configure():
///   pub  "bt_xml"      (ame/BTXML)      — compiled BT XML after plan
///   svc  "load_domain" (ame/LoadDomain) — load PDDL from strings
///   svc  "plan"        (ame/Plan)       — synchronous plan request
///
/// Parameters forwarded to the PCL component before configure:
///   domain.pddl_file    (string, "")
///   domain.problem_file (string, "")
///   plan_audit.enabled  (bool,   true)
///   plan_audit.path     (string, "plan_audit.jsonl")
///   compiler.parallel   (bool,   false)
class PlannerNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit PlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev) override;

  /// \brief Injects canonical WorldModel for in-process mode.
  void setInProcessWorldModel(ame::WorldModel* wm) {
    inprocess_wm_ = wm;
    component_.setInProcessWorldModel(wm);
  }

  ame::ActionRegistry& actionRegistry() { return component_.actionRegistry(); }
  ame::Planner& planner()               { return component_.planner(); }
  ame::PlanCompiler& compiler()         { return component_.compiler(); }
  ame::PlanAuditLog* planAuditLog()     { return component_.planAuditLog(); }
  ame::PlannerComponent& plannerComponent() { return component_; }

private:
  ame::PlannerComponent component_;
  ame::WorldModel* inprocess_wm_ = nullptr;
};

}  // namespace ame_ros2
