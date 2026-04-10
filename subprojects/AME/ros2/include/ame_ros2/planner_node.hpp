#pragma once

#include <memory>
#include <mutex>

#include <rclcpp/node_options.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <ame_ros2/action/plan.hpp>
#include <ame_ros2/srv/load_domain.hpp>

// Forward-declare AME types — full headers are not needed by ROS2 consumers
namespace ame {
  class PlannerComponent;
  class WorldModel;
  class ActionRegistry;
  class Planner;
  class PlanCompiler;
  class PlanAuditLog;
}

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
  ~PlannerNode();

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev) override;

  /// \brief Injects canonical WorldModel for in-process mode.
  void setInProcessWorldModel(ame::WorldModel* wm);

  /// \brief Inject a shared mutex that protects the in-process WorldModel.
  /// Must be called before on_configure if in-process mode is used.
  void setWorldModelMutex(std::mutex* wm_mutex) { wm_mutex_ = wm_mutex; }

  /// \brief Component and sub-object accessors for in-process mode.
  /// Callers must #include the relevant <ame/...> headers to use these references.
  ame::ActionRegistry& actionRegistry();
  ame::Planner& planner();
  ame::PlanCompiler& compiler();
  ame::PlanAuditLog* planAuditLog();
  ame::PlannerComponent& plannerComponent();

private:
  std::unique_ptr<ame::PlannerComponent> component_;
  ame::WorldModel* inprocess_wm_ = nullptr;
  std::mutex* wm_mutex_ = nullptr;  ///< Borrowed from WorldModelComponent (may be null).

  rclcpp_action::Server<ame_ros2::action::Plan>::SharedPtr plan_action_srv_;
  rclcpp::Service<ame_ros2::srv::LoadDomain>::SharedPtr    svc_load_domain_;
};

}  // namespace ame_ros2
