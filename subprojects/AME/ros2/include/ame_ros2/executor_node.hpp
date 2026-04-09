#pragma once

#include <ame/action_registry.h>
#include <ame/executor_component.h>
#include <ame/plan_audit_log.h>
#include <ame/plan_compiler.h>
#include <ame/planner.h>
#include <ame/planner_component.h>
#include <ame/pyramid_service.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <behaviortree_cpp/bt_factory.h>

namespace ame_ros2 {

/// \brief Thin ROS2 lifecycle wrapper for ame::ExecutorComponent.
///
/// All BT ticking, pub/sub (bt_xml, bt_events, status), and port wiring live
/// in the PCL component.  This node only bridges ROS2 lifecycle transitions
/// and the ROS2 parameter system.
///
/// The component creates these PCL ports during on_configure():
///   sub  "{agent_id}/executor/bt_xml"    (ame/BTXML)
///   pub  "{agent_id}/executor/bt_events" (ame/BTEvent)
///   pub  "{agent_id}/executor/status"    (ame/Status)
///
/// Parameters forwarded to the PCL component before configure:
///   agent_id          (string, "")
///   tick_rate_hz      (double, 50.0)
///   bt_log.enabled    (bool,   true)
///   bt_log.path       (string, "bt_events.jsonl")
///   bt_log.tree_id    (string, "MissionPlan")
class ExecutorNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit ExecutorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev) override;

  /// \brief Injects WorldModel pointer for in-process mode.
  void setInProcessWorldModel(ame::WorldModel* wm) {
    inprocess_wm_ = wm;
    component_.setInProcessWorldModel(wm);
  }

  /// \brief Injects PYRAMID service for InvokeService BT nodes.
  void setPyramidService(ame::IPyramidService* svc) { pyramid_service_ = svc; }

  /// \brief Injects planner for in-process hierarchical planning.
  void setPlanner(ame::Planner* p) { planner_ = p; }

  /// \brief Injects plan compiler for in-process hierarchical planning.
  void setPlanCompiler(ame::PlanCompiler* c) { plan_compiler_ = c; }

  /// \brief Injects action registry for in-process hierarchical planning.
  void setActionRegistry(ame::ActionRegistry* r) { action_registry_ = r; }

  /// \brief Injects plan audit log for hierarchical phase recording.
  void setPlanAuditLog(ame::PlanAuditLog* l) { plan_audit_log_ = l; }

  /// \brief Injects PlannerComponent for distributed hierarchical planning.
  void setPlannerComponent(ame::PlannerComponent* pc) { planner_component_ = pc; }

  /// \brief Exposes factory for custom action node type registration.
  BT::BehaviorTreeFactory& factory() { return component_.factory(); }

  /// \brief Returns the agent ID for this executor.
  const std::string& agentId() const { return agent_id_; }

  /// \brief Direct component access.
  ame::ExecutorComponent& component() { return component_; }

private:
  ame::ExecutorComponent component_;

  ame::WorldModel*       inprocess_wm_      = nullptr;
  ame::IPyramidService*  pyramid_service_   = nullptr;
  ame::Planner*          planner_           = nullptr;
  ame::PlanCompiler*     plan_compiler_     = nullptr;
  ame::ActionRegistry*   action_registry_   = nullptr;
  ame::PlanAuditLog*     plan_audit_log_    = nullptr;
  ame::PlannerComponent* planner_component_ = nullptr;
  std::string            agent_id_;

  bool core_nodes_registered_ = false;
  void registerCoreNodes();
};

}  // namespace ame_ros2
