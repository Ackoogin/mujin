#include <ame/executor_component.h>
#include <ame/pcl_msg_json.h>

#include <ame/action_registry.h>
#include <ame/bt_nodes/ame_dispatch_node.h>
#include <ame/bt_nodes/check_world_predicate.h>
#include <ame/bt_nodes/goal_reached.h>
#include <ame/bt_nodes/planned_action.h>
#include <ame/bt_nodes/simulated_action.h>

#include <stdexcept>
#include <unordered_set>

namespace ame {

ExecutorComponent::ExecutorComponent()
    : pcl::Component("executor_component") {}

void ExecutorComponent::setInProcessWorldModel(WorldModel* wm) {
  inprocess_wm_ = wm;
  if (wm != nullptr) {
    local_world_state_access_ = std::make_unique<LocalWorldStateAccess>(wm);
  } else {
    local_world_state_access_.reset();
  }
}

void ExecutorComponent::setActionSink(IExecutionSink* sink) {
  action_sink_ = sink;
}

void ExecutorComponent::setActionRegistry(const ActionRegistry* registry) {
  action_registry_ = registry;
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

#if defined(AME_NEURO)
  plan_step_meta_.clear();
  completed_step_uids_.clear();
  step_status_subscribers_.clear();
#endif

  resetExecutionState();
  registerDispatchNodesFromRegistry();
  tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(bt_xml));

  const auto blackboard = tree_->rootBlackboard();
  if (inprocess_wm_ != nullptr) {
    blackboard->set("world_model", inprocess_wm_);
  }
  if (local_world_state_access_ != nullptr) {
    blackboard->set<IWorldStateAccess*>("world_state",
                                        local_world_state_access_.get());
  }
  if (action_sink_ != nullptr) {
    blackboard->set("action_sink", action_sink_);
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

void ExecutorComponent::loadAndExecute(const CompiledPlan& compiled_plan) {
  loadAndExecute(compiled_plan.xml);
#if defined(AME_NEURO)
  plan_step_meta_ = compiled_plan.steps;

  std::unordered_set<std::string> action_unit_names;
  action_unit_names.reserve(plan_step_meta_.size());
  for (const auto& meta : plan_step_meta_) {
    action_unit_names.insert(meta.action_unit_name);
  }

  const std::function<void(BT::TreeNode*)> visitor =
      [this, &action_unit_names](BT::TreeNode* node) {
    if (action_unit_names.count(node->name()) == 0) {
      return;
    }
    auto subscriber = node->subscribeToStatusChange(
        [this](BT::TimePoint,
               const BT::TreeNode& node,
               BT::NodeStatus,
               BT::NodeStatus status) {
      if (status == BT::NodeStatus::SUCCESS) {
        completed_step_uids_.insert(node.UID());
      }
    });
    step_status_subscribers_.push_back(std::move(subscriber));
  };
  tree_->applyVisitor(visitor);
#endif
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
  const auto& builders = factory_.builders();
  if (builders.find("CheckWorldPredicate") == builders.end()) {
    factory_.registerNodeType<CheckWorldPredicate>("CheckWorldPredicate");
  }
  factory_.registerNodeType<GoalReached>("GoalReached");
  factory_.registerNodeType<PlannedAction>("PlannedAction");
  factory_.registerNodeType<SimulatedAction>("SimulatedAction");

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

#if defined(AME_NEURO)
    // On failure, consult the repair hook before publishing FAILURE.
    // If the hook returns non-empty BT XML, restart execution with the repair plan.
    // Returning empty falls through to the baseline FAILURE path.
    if (!success && repair_hook_ && inprocess_wm_) {
      const unsigned failed_step = computeFailedStepFromMetadata();
      try {
        std::string repair_xml = repair_hook_(failed_step, *inprocess_wm_);
        if (!repair_xml.empty()) {
          // Save FAILURE status before loadAndExecute resets it; restore on throw
          // so callers always observe FAILURE when repair loading fails.
          const auto pre_repair_status = last_status_;
          try {
            loadAndExecute(repair_xml);
          } catch (...) {
            last_status_ = pre_repair_status;
            throw; // re-throw → outer catch → baseline FAILURE path
          }
          if (pub_status_) {
            status_buf_ = "RUNNING";
            pcl_msg_t msg;
            ame_make_pcl_msg(status_buf_, "ame/Status", msg);
            pcl_port_publish(pub_status_, &msg);
          }
          return PCL_OK; // continue ticking the repair plan
        }
      } catch (...) {
        // Hook threw or repair BT XML invalid; fall through to baseline FAILURE.
      }
    }
#endif

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
#if defined(AME_NEURO)
  completed_step_uids_.clear();
  step_status_subscribers_.clear();
#endif
  tree_.reset();
  bt_logger_.reset();
}

void ExecutorComponent::registerDispatchNodesFromRegistry() {
  if (!action_registry_) {
    return;
  }

  const auto& builders = factory_.builders();
  for (const auto& verb : action_registry_->registeredNames()) {
    if (builders.find(verb) != builders.end()) {
      continue;
    }
    factory_.registerNodeType<AmeDispatchNode>(verb);
  }
}

#if defined(AME_NEURO)
unsigned ExecutorComponent::computeFailedStepFromMetadata() const {
  if (!tree_ || plan_step_meta_.empty()) {
    return 0;
  }
  return static_cast<unsigned>(completed_step_uids_.size());
}
#endif

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
