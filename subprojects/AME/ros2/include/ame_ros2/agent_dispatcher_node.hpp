#pragma once

#include <memory>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/node_options.hpp>

// Forward-declare AME types — full headers are not needed by ROS2 consumers
namespace ame {
  class AgentDispatcher;
  class WorldModel;
  class Planner;
  class PlanCompiler;
  class ActionRegistry;
}

namespace ame_ros2 {

/// \brief Thin ROS2 lifecycle wrapper for ame::AgentDispatcher.
///
/// All multi-agent dispatch logic, pub/sub wiring, and port creation live
/// in the PCL component.  This node only bridges ROS2 lifecycle transitions
/// and the ROS2 parameter system.
///
/// The component creates these PCL ports during on_configure():
///   pub  "{id}/executor/bt_xml"  (ame/BTXML)         — one per agent in roster
///   sub  "{id}/executor/status"  (ame/Status)        — one per agent in roster
///   svc  "dispatch_goals"        (ame/DispatchGoals)
///
/// Parameters forwarded to PCL component before configure:
///   agent_ids  (string, "") — comma-separated agent IDs (e.g. "uav1,uav2")
class AgentDispatcherNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit AgentDispatcherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~AgentDispatcherNode();

    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev) override;

    /// \brief Inject in-process dependencies for single-process deployment.
    void setInProcessDependencies(ame::WorldModel* wm,
                                   ame::Planner* planner,
                                   ame::PlanCompiler* compiler,
                                   ame::ActionRegistry* registry);

    /// \brief Direct component access.
    /// Callers must #include <ame/agent_dispatcher.h> to use the returned reference.
    ame::AgentDispatcher& dispatcher();

private:
    std::unique_ptr<ame::AgentDispatcher> component_;
};

}  // namespace ame_ros2
