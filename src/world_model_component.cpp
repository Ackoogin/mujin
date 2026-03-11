#include <mujin/pddl_parser.h>
#include <mujin/world_model_component.h>

#include <exception>
#include <stdexcept>

namespace mujin {

WorldModelComponent::WorldModelComponent()
    : pcl::Component("world_model_component") {}

GetFactResult WorldModelComponent::getFact(const std::string& key) const {
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
  return buildSnapshot(keys);
}

bool WorldModelComponent::consumeStateDirty() {
  return state_dirty_.exchange(false);
}

pcl_status_t WorldModelComponent::on_configure() {
  wm_ = WorldModel();
  audit_log_.reset();
  state_dirty_.store(false);

  try {
    loadDomainFromParams();
  } catch (const std::exception& e) {
    logError("Failed to load domain: %s", e.what());
    return PCL_ERR_CALLBACK;
  }

  if (paramBool("audit_log.enabled", true)) {
    audit_log_.emplace(paramStr("audit_log.path", "wm_audit.jsonl"));
  }

  wm_.setAuditCallback([this](uint64_t ver,
                              uint64_t ts_us,
                              const std::string& fact,
                              bool value,
                              const std::string& source) {
    if (audit_log_) {
      audit_log_->onFactChange(ver, ts_us, fact, value, source);
    }
    state_dirty_.store(true);
  });

  state_dirty_.store(true);
  logInfo("Configured with %u fluents and %u ground actions",
          wm_.numFluents(),
          wm_.numGroundActions());
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
  return PCL_OK;
}

pcl_status_t WorldModelComponent::on_shutdown() {
  if (audit_log_) {
    audit_log_->flush();
  }
  return PCL_OK;
}

void WorldModelComponent::loadDomainFromParams() {
  const auto domain_file = paramStr("domain.pddl_file", "");
  const auto problem_file = paramStr("domain.problem_file", "");

  if (!domain_file.empty() && !problem_file.empty()) {
    PddlParser::parse(domain_file, problem_file, wm_);
    return;
  }

  logWarn("No PDDL files specified. World model starts empty.");
}

WorldStateSnapshot WorldModelComponent::buildSnapshot(
    const std::vector<std::string>& keys) const {
  WorldStateSnapshot snapshot;
  snapshot.wm_version = wm_.version();

  if (keys.empty()) {
    for (unsigned i = 0; i < wm_.numFluents(); ++i) {
      if (!wm_.getFact(i)) {
        continue;
      }

      WorldFactValue fact;
      fact.key = wm_.fluentName(i);
      fact.value = true;
      fact.wm_version = wm_.version();
      snapshot.facts.push_back(fact);
    }
  } else {
    for (const auto& key : keys) {
      WorldFactValue fact;
      fact.key = key;
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

}  // namespace mujin
