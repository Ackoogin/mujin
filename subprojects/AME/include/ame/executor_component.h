#pragma once

#include <ame/bt_logger.h>
#include <ame/world_model.h>
#include <pcl/component.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include <functional>
#include <memory>
#include <string>

namespace ame {

/// \brief PCL-backed BT execution component.
///
/// Ports created during on_configure():
///   sub  "{prefix}/executor/bt_xml"    (ame/BTXML)   -- BT XML to load and execute
///   pub  "{prefix}/executor/bt_events" (ame/BTEvent) -- JSON event lines from AmeBTLogger
///   pub  "{prefix}/executor/status"    (ame/Status)  -- "IDLE"/"RUNNING"/"SUCCESS"/"FAILURE"
///
/// on_tick() drives BT ticking at tick_rate_hz (default 50 Hz).
///
/// Parameters:
///   agent_id          (string, "")   -- prefix for topic names (empty = no prefix)
///   tick_rate_hz      (double, 50.0)
///   bt_log.enabled    (bool,   true)
///   bt_log.path       (string, "bt_events.jsonl")
///   bt_log.tree_id    (string, "MissionPlan")
class ExecutorComponent : public pcl::Component {
public:
  using BlackboardInitializer = std::function<void(const BT::Blackboard::Ptr&)>;

  /// \brief Callback for receiving BT event lines.
  /// Retained for testing and backward-compatibility; PCL subscribers
  /// are the primary channel in production.
  using EventSink = std::function<void(const std::string&)>;

  ExecutorComponent();

  /// \brief Inject direct world model access for in-process execution.
  void setInProcessWorldModel(WorldModel* wm);

  /// \brief Customize blackboard initialization for transport-specific handles.
  void setBlackboardInitializer(BlackboardInitializer initializer);

  /// \brief Register an event callback (useful for testing; PCL port is the production path).
  void setEventSink(EventSink sink);

  /// \brief Expose the factory for action node registration.
  BT::BehaviorTreeFactory& factory() { return factory_; }
  const BT::BehaviorTreeFactory& factory() const { return factory_; }

  /// \brief Load a BT and prepare it for ticking.
  void loadAndExecute(const std::string& bt_xml);

  /// \brief Tick the current tree once if execution is active.
  void tickOnce();

  /// \brief Stop execution explicitly: halt the tree and reset state.
  void haltExecution();

  bool isExecuting() const { return executing_; }
  BT::NodeStatus lastStatus() const { return last_status_; }

protected:
  pcl_status_t on_configure() override;
  pcl_status_t on_activate() override;
  pcl_status_t on_deactivate() override;
  pcl_status_t on_cleanup() override;
  pcl_status_t on_shutdown() override;
  pcl_status_t on_tick(double dt) override;

private:
  void resetExecutionState();

  /// \brief Emit one BT event line via both EventSink and PCL port.
  void emitEvent(const std::string& json_line);

  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::unique_ptr<AmeBTLogger> bt_logger_;
  WorldModel* inprocess_wm_ = nullptr;
  BlackboardInitializer blackboard_initializer_;
  EventSink event_sink_;
  bool executing_ = false;
  BT::NodeStatus last_status_ = BT::NodeStatus::IDLE;

  // PCL ports (valid after on_configure)
  pcl_port_t* pub_bt_events_ = nullptr;
  pcl_port_t* pub_status_    = nullptr;

  // Response buffer for status publish
  std::string status_buf_;

  // -- Static PCL callbacks ------------------------------------------------

  static void onBTXmlCb(pcl_container_t*, const pcl_msg_t* msg, void* ud);
};

}  // namespace ame
