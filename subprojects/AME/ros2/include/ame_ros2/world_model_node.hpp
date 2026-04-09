#pragma once

#include <memory>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/node_options.hpp>

// Forward-declare AME types — full headers are not needed by ROS2 consumers
namespace ame {
  class WorldModelComponent;
  class WorldModel;
}

namespace ame_ros2 {

/// \brief Thin ROS2 lifecycle wrapper for ame::WorldModelComponent.
///
/// All business logic and communication (pub/sub/services) live in the
/// PCL component.  This node only bridges ROS2 lifecycle transitions and
/// the ROS2 parameter system to the component.
///
/// Communication is provided by PCL ports on the component:
///   pub  "world_state"  — periodic state snapshot (ame/WorldState)
///   sub  "detections"   — perception ingress (ame/Detection)
///   svc  "get_fact"     — GetFact request/response
///   svc  "set_fact"     — SetFact request/response
///   svc  "query_state"  — QueryState request/response
///   svc  "load_domain"  — LoadDomain request/response
///
/// Parameters (forwarded to PCL component before configure):
///   domain.pddl_file                (string, "")
///   domain.problem_file             (string, "")
///   audit_log.enabled               (bool,   true)
///   audit_log.path                  (string, "wm_audit.jsonl")
///   publish_rate_hz                 (double, 10.0)
///   perception.enabled              (bool,   true)
///   perception.confidence_threshold (double, 0.5)
class WorldModelNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit WorldModelNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~WorldModelNode();

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& prev) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& prev) override;

  /// \brief Direct world-model access for in-process mode.
  /// Callers must #include <ame/world_model.h> to use the returned reference.
  ame::WorldModel& worldModel();
  const ame::WorldModel& worldModel() const;

  /// \brief Direct component access for in-process mode.
  /// Callers must #include <ame/world_model_component.h> to use the returned reference.
  ame::WorldModelComponent& component();

private:
  std::unique_ptr<ame::WorldModelComponent> component_;
};

}  // namespace ame_ros2
