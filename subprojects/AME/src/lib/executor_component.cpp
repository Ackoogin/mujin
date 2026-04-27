#include <ame/executor_component.h>
#include <ame/pcl_msg_json.h>

#include <ame/bt_nodes/check_world_predicate.h>
#include <ame/bt_nodes/set_world_predicate.h>

#include <stdexcept>

namespace ame {

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

void ExecutorComponent::emitEvent(const std::string& json_line) {
  if (event_sink_) {
    event_sink_(json_line);
  }
  if (pub_bt_events_) {
    pcl_msg_t msg;
    msg.data      = json_line.c_str();
    msg.size      = static_cast<uint32_t>(json_line.size());
    msg.type_name = "ame/BTEvent";
    pcl_port_publish(pub_bt_events_, &msg);
  }
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

  bt_logger_ = std::make_unique<AmeBTLogger>(
      *tree_,
      paramStr("bt_log.tree_id", "MissionPlan"),
      inprocess_wm_);

  if (paramBool("bt_log.enabled", true)) {
    bt_logger_->addFileSink(paramStr("bt_log.path", "bt_events.jsonl"));
  }

  // Route events through emitEvent (calls both EventSink and PCL port)
  bt_logger_->addCallbackSink([this](const std::string& line) { emitEvent(line); });

  executing_   = true;
  last_status_ = BT::NodeStatus::RUNNING;
}

void ExecutorComponent::tickOnce() {
  if (!tree_ || !executing_) return;
  last_status_ = tree_->tickOnce();
  if (last_status_ != BT::NodeStatus::RUNNING && bt_logger_) {
    bt_logger_->flush();
  }
}

void ExecutorComponent::haltExecution() {
  if (tree_) tree_->haltTree();
  executing_   = false;
  last_status_ = BT::NodeStatus::IDLE;
  if (bt_logger_) bt_logger_->flush();
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

pcl_status_t ExecutorComponent::on_configure() {
  factory_.registerNodeType<CheckWorldPredicate>("CheckWorldPredicate");
  factory_.registerNodeType<SetWorldPredicate>("SetWorldPredicate");

  // Build topic prefix from agent_id parameter
  const auto agent_id = paramStr("agent_id", "");
  const std::string prefix = agent_id.empty() ? "" : (agent_id + "/");

  pub_bt_events_ = addPublisher((prefix + "executor/bt_events").c_str(),
                                "ame/BTEvent");
  pub_status_    = addPublisher((prefix + "executor/status").c_str(),
                                "ame/Status");
  addSubscriber((prefix + "executor/bt_xml").c_str(), "ame/BTXML",
                onBTXmlCb, this);

  const double rate_hz = paramF64("tick_rate_hz", 50.0);
  setTickRateHz(rate_hz > 0.0 ? rate_hz : 50.0);

  return PCL_OK;
}

pcl_status_t ExecutorComponent::on_activate() {
  resetExecutionState();
  // Publish initial IDLE status
  status_buf_ = "IDLE";
  if (pub_status_) {
    pcl_msg_t msg;
    ame_make_pcl_msg(status_buf_, "ame/Status", msg);
    pcl_port_publish(pub_status_, &msg);
  }
  return PCL_OK;
}

pcl_status_t ExecutorComponent::on_deactivate() {
  resetExecutionState();
  return PCL_OK;
}

pcl_status_t ExecutorComponent::on_cleanup() {
  resetExecutionState();
  pub_bt_events_ = nullptr;
  pub_status_    = nullptr;
  return PCL_OK;
}

pcl_status_t ExecutorComponent::on_shutdown() {
  if (bt_logger_) bt_logger_->flush();
  return PCL_OK;
}

pcl_status_t ExecutorComponent::on_tick(double /*dt*/) {
  if (!executing_) return PCL_OK;

  tickOnce();

  if (last_status_ == BT::NodeStatus::SUCCESS ||
      last_status_ == BT::NodeStatus::FAILURE) {
    const bool success = (last_status_ == BT::NodeStatus::SUCCESS);
    // Stop ticking but preserve last_status_ for callers to observe.
    // Do NOT call haltExecution() here -- that would reset last_status_ to IDLE.
    executing_ = false;
    if (tree_) tree_->haltTree();
    if (bt_logger_) bt_logger_->flush();

    if (pub_status_) {
      status_buf_ = success ? "SUCCESS" : "FAILURE";
      pcl_msg_t msg;
      ame_make_pcl_msg(status_buf_, "ame/Status", msg);
      pcl_port_publish(pub_status_, &msg);
    }
  }
  return PCL_OK;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void ExecutorComponent::resetExecutionState() {
  executing_   = false;
  last_status_ = BT::NodeStatus::IDLE;
  tree_.reset();
  bt_logger_.reset();
}

// ---------------------------------------------------------------------------
// Static PCL callbacks
// ---------------------------------------------------------------------------

void ExecutorComponent::onBTXmlCb(pcl_container_t*,
                                   const pcl_msg_t* msg,
                                   void* ud) {
  auto* self   = static_cast<ExecutorComponent*>(ud);
  auto  bt_xml = ame_msg_to_string(msg);
  if (bt_xml.empty()) return;
  try {
    self->loadAndExecute(bt_xml);
    // Publish RUNNING status
    if (self->pub_status_) {
      self->status_buf_ = "RUNNING";
      pcl_msg_t smsg;
      ame_make_pcl_msg(self->status_buf_, "ame/Status", smsg);
      pcl_port_publish(self->pub_status_, &smsg);
    }
  } catch (const std::exception& e) {
    self->logError("Failed to load BT XML: %s", e.what());
    if (self->pub_status_) {
      self->status_buf_ = "FAILURE";
      pcl_msg_t smsg;
      ame_make_pcl_msg(self->status_buf_, "ame/Status", smsg);
      pcl_port_publish(self->pub_status_, &smsg);
    }
  }
}

}  // namespace ame
