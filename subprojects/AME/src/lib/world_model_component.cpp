#include <ame/pcl_msg_json.h>
#include <ame/pddl_parser.h>
#include <ame/world_model_component.h>

#include <exception>
#include <stdexcept>

namespace ame {

WorldModelComponent::WorldModelComponent()
    : pcl::Component("world_model_component") {}

void WorldModelComponent::setWmPublishCallback(
    std::function<void(const WorldStateSnapshot&)> cb) {
  wm_publish_cb_ = std::move(cb);
}

GetFactResult WorldModelComponent::getFact(const std::string& key) const {
  std::lock_guard<std::mutex> lock(wm_mutex_);
  GetFactResult result;
  result.wm_version = wm_.version();
  try {
    result.value = wm_.getFact(key);
    result.found = true;
  } catch (...) {
    result.found = false;
    result.value = false;
  }
  return result;
}

SetFactResult WorldModelComponent::setFact(const std::string& key,
                                           bool value,
                                           const std::string& source) {
  std::lock_guard<std::mutex> lock(wm_mutex_);
  SetFactResult result;
  try {
    wm_.setFact(key, value, source);
    result.success = true;
  } catch (...) {
    result.success = false;
  }
  result.wm_version = wm_.version();
  return result;
}

WorldStateSnapshot WorldModelComponent::queryState(
    const std::vector<std::string>& keys) const {
  std::lock_guard<std::mutex> lock(wm_mutex_);
  return buildSnapshot(keys);
}

bool WorldModelComponent::consumeStateDirty() {
  return state_dirty_.exchange(false);
}

void WorldModelComponent::applyDetection(const Detection& det) {
  if (det.confidence < static_cast<float>(perception_confidence_threshold_)) {
    logDebug("Ignoring detection '%s' (confidence %.2f < %.2f)",
             det.entity_id.c_str(), det.confidence,
             perception_confidence_threshold_);
    return;
  }

  const std::string source = "perception:" + det.sensor_source;
  for (size_t i = 0;
       i < det.property_keys.size() && i < det.property_values.size(); ++i) {
    std::string fact_key = "(" + det.property_keys[i] + " " + det.entity_id;
    if (!det.property_values[i].empty()) {
      fact_key += " " + det.property_values[i];
    }
    fact_key += ")";
    try {
      unsigned fluent_id = wm_.fluentIndex(fact_key);
      if (wm_.hasAuthorityConflict(fluent_id, true)) {
        logWarn("Authority conflict: perception says '%s' is true but plan predicted otherwise",
                fact_key.c_str());
      }
      wm_.enqueueMutation(fluent_id, true, source, FactAuthority::CONFIRMED);
    } catch (const std::exception& e) {
      logWarn("Failed to apply perception fact '%s': %s", fact_key.c_str(), e.what());
    }
  }
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

pcl_status_t WorldModelComponent::on_configure() {
  wm_ = WorldModel();
  audit_log_.reset();
  state_dirty_.store(false);
  pub_world_state_ = nullptr;

  try {
    loadDomainFromParams();
  } catch (const std::exception& e) {
    logError("Failed to load domain: %s", e.what());
    return PCL_ERR_CALLBACK;
  }

  if (paramBool("audit_log.enabled", true)) {
    audit_log_.emplace(paramStr("audit_log.path", "wm_audit.jsonl"));
  }

  rewireAuditCallback();
  state_dirty_.store(true);

  // Perception params
  perception_confidence_threshold_ =
      paramF64("perception.confidence_threshold", 0.5);

  // Ports
  pub_world_state_ = addPublisher("world_state", "ame/WorldState");

  if (paramBool("perception.enabled", true)) {
    addSubscriber("detections", "ame/Detection", onDetectionCb, this);
  }

  addService("get_fact",    "ame/GetFact",    handleGetFactCb,    this);
  addService("set_fact",    "ame/SetFact",    handleSetFactCb,    this);
  addService("query_state", "ame/QueryState", handleQueryStateCb, this);
  addService("load_domain", "ame/LoadDomain", handleLoadDomainCb, this);

  const double rate_hz = paramF64("publish_rate_hz", 10.0);
  setTickRateHz(rate_hz > 0.0 ? rate_hz : 10.0);

  logInfo("Configured with %u fluents and %u ground actions",
          wm_.numFluents(), wm_.numGroundActions());
  return PCL_OK;
}

pcl_status_t WorldModelComponent::on_activate() {
  state_dirty_.store(true);
  return PCL_OK;
}

pcl_status_t WorldModelComponent::on_deactivate() {
  return PCL_OK;
}

pcl_status_t WorldModelComponent::on_cleanup() {
  if (audit_log_) {
    audit_log_->flush();
  }
  audit_log_.reset();
  wm_ = WorldModel();
  state_dirty_.store(false);
  pub_world_state_ = nullptr;
  return PCL_OK;
}

pcl_status_t WorldModelComponent::on_shutdown() {
  if (audit_log_) {
    audit_log_->flush();
  }
  return PCL_OK;
}

pcl_status_t WorldModelComponent::on_tick(double /*dt*/) {
  WorldStateSnapshot snap;
  bool should_publish = false;
  {
    std::lock_guard<std::mutex> lock(wm_mutex_);
    const size_t applied = wm_.applyQueuedMutations();
    if (applied > 0 || consumeStateDirty()) {
      if (pub_world_state_ || wm_publish_cb_) {
        snap = buildSnapshot({});
        should_publish = true;
      }
    }
  }
  if (should_publish) {
    if (pub_world_state_) {
      std::string json = ame_pack_world_state(snap);
      pcl_msg_t msg;
      ame_make_pcl_msg(json, "ame/WorldState", msg);
      pcl_port_publish(pub_world_state_, &msg);
    }
    if (wm_publish_cb_) {
      wm_publish_cb_(snap);
    }
  }
  return PCL_OK;
}

// ---------------------------------------------------------------------------
// loadDomainFromStrings
// ---------------------------------------------------------------------------

WorldModelComponent::LoadDomainResult WorldModelComponent::loadDomainFromStrings(
    const std::string& domain_pddl,
    const std::string& problem_pddl) {
  // Capture current facts under lock, then parse without holding it.
  std::vector<std::pair<std::string, std::string>> preserved_facts;
  {
    std::lock_guard<std::mutex> lock(wm_mutex_);
    for (unsigned i = 0; i < wm_.numFluents(); ++i) {
      if (wm_.getFact(i)) {
        const auto& meta = wm_.getFactMetadata(i);
        preserved_facts.emplace_back(wm_.fluentName(i), meta.source);
      }
    }
  }

  LoadDomainResult result;
  try {
    WorldModel new_wm;
    PddlParser::parseFromString(domain_pddl, problem_pddl, new_wm);

    for (const auto& [key, source] : preserved_facts) {
      try {
        new_wm.setFact(key, true, source.empty() ? "domain_reload" : source);
      } catch (...) {
      }
    }

    std::lock_guard<std::mutex> lock(wm_mutex_);
    wm_ = std::move(new_wm);
    rewireAuditCallback();
    state_dirty_.store(true);

    result.success = true;
    result.num_fluents = wm_.numFluents();
    result.num_ground_actions = wm_.numGroundActions();
  } catch (const std::exception& e) {
    result.error_msg = e.what();
  }
  return result;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void WorldModelComponent::loadDomainFromParams() {
  const auto domain_file  = paramStr("domain.pddl_file",  "");
  const auto problem_file = paramStr("domain.problem_file", "");
  if (!domain_file.empty() && !problem_file.empty()) {
    PddlParser::parse(domain_file, problem_file, wm_);
    return;
  }
  logWarn("No PDDL files specified. World model starts empty.");
}

void WorldModelComponent::rewireAuditCallback() {
  wm_.setAuditCallback([this](uint64_t ver, uint64_t ts_us,
                              const std::string& fact, bool value,
                              const std::string& source) {
    if (audit_log_) {
      audit_log_->onFactChange(ver, ts_us, fact, value, source);
    }
    state_dirty_.store(true);
  });
}

WorldStateSnapshot WorldModelComponent::buildSnapshot(
    const std::vector<std::string>& keys) const {
  WorldStateSnapshot snapshot;
  snapshot.wm_version = wm_.version();

  if (keys.empty()) {
    for (unsigned i = 0; i < wm_.numFluents(); ++i) {
      if (!wm_.getFact(i)) continue;
      WorldFactValue fact;
      fact.key        = wm_.fluentName(i);
      fact.value      = true;
      fact.wm_version = wm_.version();
      snapshot.facts.push_back(fact);
    }
  } else {
    for (const auto& key : keys) {
      WorldFactValue fact;
      fact.key        = key;
      fact.wm_version = wm_.version();
      try {
        fact.value = wm_.getFact(key);
      } catch (...) {
        fact.value = false;
      }
      snapshot.facts.push_back(fact);
    }
  }

  for (auto goal_id : wm_.goalFluentIds()) {
    snapshot.goal_fluents.push_back(wm_.fluentName(goal_id));
  }
  return snapshot;
}

// ---------------------------------------------------------------------------
// Static PCL callbacks
// ---------------------------------------------------------------------------

void WorldModelComponent::onDetectionCb(pcl_container_t*,
                                         const pcl_msg_t* msg,
                                         void* ud) {
  auto* self = static_cast<WorldModelComponent*>(ud);
  auto det   = ame_unpack_detection(msg);
  self->applyDetection(det);
}

pcl_status_t WorldModelComponent::handleGetFactCb(pcl_container_t*,
                                                    const pcl_msg_t* req,
                                                    pcl_msg_t* resp,
                                                    pcl_svc_context_t*,
                                                    void* ud) {
  auto* self  = static_cast<WorldModelComponent*>(ud);
  auto  key   = ame_unpack_get_fact_request_key(req);
  auto  result = self->getFact(key);
  self->resp_buf_get_fact_ = ame_pack_get_fact_response(result);
  ame_make_pcl_msg(self->resp_buf_get_fact_, "ame/GetFact_Response", *resp);
  return PCL_OK;
}

pcl_status_t WorldModelComponent::handleSetFactCb(pcl_container_t*,
                                                    const pcl_msg_t* req,
                                                    pcl_msg_t* resp,
                                                    pcl_svc_context_t*,
                                                    void* ud) {
  auto* self   = static_cast<WorldModelComponent*>(ud);
  auto  sreq   = ame_unpack_set_fact_request(req);
  auto  result = self->setFact(sreq.key, sreq.value, sreq.source);
  self->resp_buf_set_fact_ = ame_pack_set_fact_response(result);
  ame_make_pcl_msg(self->resp_buf_set_fact_, "ame/SetFact_Response", *resp);
  return PCL_OK;
}

pcl_status_t WorldModelComponent::handleQueryStateCb(pcl_container_t*,
                                                      const pcl_msg_t* req,
                                                      pcl_msg_t* resp,
                                                      pcl_svc_context_t*,
                                                      void* ud) {
  auto* self = static_cast<WorldModelComponent*>(ud);
  auto  keys = ame_unpack_query_state_request_keys(req);
  auto  snap = self->queryState(keys);
  self->resp_buf_query_state_ = ame_pack_query_state_response(snap);
  ame_make_pcl_msg(self->resp_buf_query_state_, "ame/QueryState_Response", *resp);
  return PCL_OK;
}

pcl_status_t WorldModelComponent::handleLoadDomainCb(pcl_container_t*,
                                                      const pcl_msg_t* req,
                                                      pcl_msg_t* resp,
                                                      pcl_svc_context_t*,
                                                      void* ud) {
  auto* self   = static_cast<WorldModelComponent*>(ud);
  auto  lreq   = ame_unpack_load_domain_request(req);
  auto  result = self->loadDomainFromStrings(lreq.domain_pddl, lreq.problem_pddl);

  LoadDomainResponse lresp;
  lresp.success          = result.success;
  lresp.error_msg        = result.error_msg;
  lresp.num_fluents      = result.num_fluents;
  lresp.num_ground_actions = result.num_ground_actions;
  self->resp_buf_load_domain_ = ame_pack_load_domain_response(lresp);
  ame_make_pcl_msg(self->resp_buf_load_domain_, "ame/LoadDomain_Response", *resp);
  return PCL_OK;
}

}  // namespace ame
