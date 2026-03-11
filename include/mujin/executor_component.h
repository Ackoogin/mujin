#pragma once

#include "mujin/bt_logger.h"
#include "mujin/world_model.h"
#include "pcl/component.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <functional>
#include <memory>
#include <string>

namespace mujin {

/// \brief PCL-backed BT execution component with ROS-agnostic runtime logic.
class ExecutorComponent : public pcl::Component {
public:
  using BlackboardInitializer = std::function<void(const BT::Blackboard::Ptr&)>;
  using EventSink = std::function<void(const std::string&)>;

  ExecutorComponent();

  /// \brief Inject direct world model access for in-process execution.
  void setInProcessWorldModel(WorldModel* wm);

  /// \brief Customize blackboard initialization for transport-specific handles.
  void setBlackboardInitializer(BlackboardInitializer initializer);

  /// \brief Forward BT event lines to an arbitrary sink.
  void setEventSink(EventSink sink);

  /// \brief Expose the factory for action node registration.
  BT::BehaviorTreeFactory& factory() { return factory_; }

  /// \brief Expose the factory for action node registration.
  const BT::BehaviorTreeFactory& factory() const { return factory_; }

  /// \brief Load a BT and prepare it for ticking.
  void loadAndExecute(const std::string& bt_xml);

  /// \brief Tick the current tree once if execution is active.
  void tickOnce();

  /// \brief Current execution state.
  bool isExecuting() const { return executing_; }

  /// \brief Last observed BT status.
  BT::NodeStatus lastStatus() const { return last_status_; }

protected:
  pcl_status_t on_configure() override;
  pcl_status_t on_activate() override;
  pcl_status_t on_deactivate() override;
  pcl_status_t on_cleanup() override;
  pcl_status_t on_shutdown() override;

private:
  void resetExecutionState();

  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::unique_ptr<MujinBTLogger> bt_logger_;
  WorldModel* inprocess_wm_ = nullptr;
  BlackboardInitializer blackboard_initializer_;
  EventSink event_sink_;
  bool executing_ = false;
  BT::NodeStatus last_status_ = BT::NodeStatus::IDLE;
};

}  // namespace mujin
