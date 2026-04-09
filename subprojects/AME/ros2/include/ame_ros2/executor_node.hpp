#pragma once

#include <memory>
#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/node_options.hpp>

// Forward-declare AME and BT types — full headers are not needed by ROS2 consumers
namespace ame {
  class ExecutorComponent;
  class WorldModel;
  class IPyramidService;
  class Planner;
  class PlanCompiler;
  class ActionRegistry;
  class PlanAuditLog;
  class PlannerComponent;
}
namespace BT { class BehaviorTreeFactory; }

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
  ~ExecutorNode();

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev) override;

  /// \brief Injects WorldModel pointer for in-process mode.
  void setInProcessWorldModel(ame::WorldModel* wm);

  /// \brief Injects PYRAMID service for InvokeService BT nodes.
  void setPyramidService(ame::IPyramidService* svc);

  /// \brief Injects planner for in-process hierarchical planning.
  void setPlanner(ame::Planner* p);

  /// \brief Injects plan compiler for in-process hierarchical planning.
  void setPlanCompiler(ame::PlanCompiler* c);

  /// \brief Injects action registry for in-process hierarchical planning.
  void setActionRegistry(ame::ActionRegistry* r);

  /// \brief Injects plan audit log for hierarchical phase recording.
  void setPlanAuditLog(ame::PlanAuditLog* l);

  /// \brief Injects PlannerComponent for distributed hierarchical planning.
  void setPlannerComponent(ame::PlannerComponent* pc);

  /// \brief Exposes factory for custom action node type registration.
  /// Callers must #include <behaviortree_cpp/bt_factory.h> to use the returned reference.
  BT::BehaviorTreeFactory& factory();

  /// \brief Returns the agent ID for this executor.
  const std::string& agentId() const;

  /// \brief Direct component access.
  /// Callers must #include <ame/executor_component.h> to use the returned reference.
  ame::ExecutorComponent& component();

private:
  std::unique_ptr<ame::ExecutorComponent> component_;

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
