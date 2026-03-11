#include <mujin/executor_component.h>

#include <stdexcept>

namespace mujin {

ExecutorComponent::ExecutorComponent()
    : pcl::Component("executor_component") {}

void ExecutorComponent::setInProcessWorldModel(WorldModel* wm) {
  inprocess_wm_ = wm;
}

void ExecutorComponent::setBlackboardInitializer(BlackboardInitializer initializer) {
  blackboard_initializer_ = std::move(initializer);
}

void ExecutorComponent::setEventSink(EventSink sink) {
  event_sink_ = std::move(sink);
}

void ExecutorComponent::loadAndExecute(const std::string& bt_xml) {
  if (state() != PCL_STATE_ACTIVE) {
    throw std::runtime_error("Executor component must be active before loading a BT");
  }

  resetExecutionState();
  tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(bt_xml));

  const auto blackboard = tree_->rootBlackboard();
  if (inprocess_wm_ != nullptr) {
    blackboard->set("world_model", inprocess_wm_);
  }
  if (blackboard_initializer_) {
    blackboard_initializer_(blackboard);
  }

  bt_logger_ = std::make_unique<MujinBTLogger>(
      *tree_,
      paramStr("bt_log.tree_id", "MissionPlan"),
      inprocess_wm_);

  if (paramBool("bt_log.enabled", true)) {
    bt_logger_->addFileSink(paramStr("bt_log.path", "bt_events.jsonl"));
  }
  if (event_sink_) {
    bt_logger_->addCallbackSink(event_sink_);
  }

  executing_ = true;
  last_status_ = BT::NodeStatus::RUNNING;
}

void ExecutorComponent::tickOnce() {
  if (!tree_ || !executing_) {
    return;
  }

  last_status_ = tree_->tickOnce();
  if (last_status_ != BT::NodeStatus::RUNNING) {
    executing_ = false;
    if (bt_logger_) {
      bt_logger_->flush();
    }
  }
}

pcl_status_t ExecutorComponent::on_configure() {
  return PCL_OK;
}

pcl_status_t ExecutorComponent::on_activate() {
  resetExecutionState();
  return PCL_OK;
}

pcl_status_t ExecutorComponent::on_deactivate() {
  resetExecutionState();
  return PCL_OK;
}

pcl_status_t ExecutorComponent::on_cleanup() {
  resetExecutionState();
  return PCL_OK;
}

pcl_status_t ExecutorComponent::on_shutdown() {
  if (bt_logger_) {
    bt_logger_->flush();
  }
  return PCL_OK;
}

void ExecutorComponent::resetExecutionState() {
  executing_ = false;
  last_status_ = BT::NodeStatus::IDLE;
  tree_.reset();
  bt_logger_.reset();
}

}  // namespace mujin
