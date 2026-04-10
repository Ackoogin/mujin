#pragma once

#include <ame/world_model.h>
#include <ame/wm_audit_log.h>
#include <pcl/component.hpp>

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace ame {

struct WorldFactValue {
  std::string key;
  bool value = false;
  std::string source;
  uint64_t wm_version = 0;
};

struct GetFactResult {
  bool found = false;
  bool value = false;
  uint64_t wm_version = 0;
};

struct SetFactResult {
  bool success = false;
  uint64_t wm_version = 0;
};

struct WorldStateSnapshot {
  bool success = true;
  uint64_t wm_version = 0;
  std::vector<WorldFactValue> facts;
  std::vector<std::string> goal_fluents;
};

// Forward declaration (defined in pcl_msg_json.h).
struct Detection;

/// \brief PCL-backed world model component.
///
/// Ports created during on_configure():
///   pub  "world_state"  (ame/WorldState)   — periodic state snapshot
///   sub  "detections"   (ame/Detection)    — perception ingress
///   svc  "get_fact"     (ame/GetFact)
///   svc  "set_fact"     (ame/SetFact)
///   svc  "query_state"  (ame/QueryState)
///   svc  "load_domain"  (ame/LoadDomain)
///
/// on_tick() publishes world_state at publish_rate_hz (default 10 Hz)
/// when the state is dirty.
///
/// Parameters:
///   domain.pddl_file                (string, "")
///   domain.problem_file             (string, "")
///   audit_log.enabled               (bool,   true)
///   audit_log.path                  (string, "wm_audit.jsonl")
///   publish_rate_hz                 (double, 10.0)
///   perception.enabled              (bool,   true)
///   perception.confidence_threshold (double, 0.5)
class WorldModelComponent : public pcl::Component {
public:
  WorldModelComponent();

  /// \brief Access the canonical world model for in-process integrations.
  WorldModel& worldModel() { return wm_; }
  const WorldModel& worldModel() const { return wm_; }

  /// \brief Mutex protecting wm_ for cross-thread access (e.g. ROS2 ↔ PCL).
  std::mutex& worldModelMutex() { return wm_mutex_; }

  /// \brief Register a callback invoked on every world-state publish (from the PCL thread).
  /// Used by WorldModelNode to bridge PCL state publishes into ROS2 topics.
  void setWmPublishCallback(std::function<void(const WorldStateSnapshot&)> cb);

  /// \brief Query a single fact by key.
  GetFactResult getFact(const std::string& key) const;

  /// \brief Set a single fact by key.
  SetFactResult setFact(const std::string& key, bool value, const std::string& source);

  /// \brief Build a snapshot of either all true facts or the requested keys.
  WorldStateSnapshot queryState(const std::vector<std::string>& keys) const;

  struct LoadDomainResult {
    bool success = false;
    std::string error_msg;
    unsigned num_fluents = 0;
    unsigned num_ground_actions = 0;
  };
  LoadDomainResult loadDomainFromStrings(const std::string& domain_pddl,
                                         const std::string& problem_pddl);

  /// \brief Returns true once after the state changes.
  bool consumeStateDirty();

  /// \brief Apply a detection to the world model (thread-safe via mutation queue).
  void applyDetection(const Detection& det);

protected:
  pcl_status_t on_configure() override;
  pcl_status_t on_activate() override;
  pcl_status_t on_deactivate() override;
  pcl_status_t on_cleanup() override;
  pcl_status_t on_shutdown() override;
  pcl_status_t on_tick(double dt) override;

private:
  void loadDomainFromParams();
  WorldStateSnapshot buildSnapshot(const std::vector<std::string>& keys) const;
  void rewireAuditCallback();

  // PCL ports (valid after on_configure)
  pcl_port_t* pub_world_state_ = nullptr;

  mutable std::mutex wm_mutex_;  ///< Guards wm_ for concurrent PCL ↔ ROS2 access.
  WorldModel wm_;
  std::optional<WmAuditLog> audit_log_;
  std::atomic<bool> state_dirty_{false};
  double perception_confidence_threshold_ = 0.5;
  std::function<void(const WorldStateSnapshot&)> wm_publish_cb_;

  // -- Static PCL callbacks ------------------------------------------------

  static void onDetectionCb(pcl_container_t*, const pcl_msg_t* msg, void* ud);

  static pcl_status_t handleGetFactCb(pcl_container_t*,
                                       const pcl_msg_t* req,
                                       pcl_msg_t* resp,
                                       pcl_svc_context_t*,
                                       void* ud);

  static pcl_status_t handleSetFactCb(pcl_container_t*,
                                       const pcl_msg_t* req,
                                       pcl_msg_t* resp,
                                       pcl_svc_context_t*,
                                       void* ud);

  static pcl_status_t handleQueryStateCb(pcl_container_t*,
                                          const pcl_msg_t* req,
                                          pcl_msg_t* resp,
                                          pcl_svc_context_t*,
                                          void* ud);

  static pcl_status_t handleLoadDomainCb(pcl_container_t*,
                                          const pcl_msg_t* req,
                                          pcl_msg_t* resp,
                                          pcl_svc_context_t*,
                                          void* ud);

  // Per-callback string buffers (valid for the duration of one service call;
  // service callbacks are serialised on the executor thread so a single buffer
  // per service is sufficient).
  std::string resp_buf_get_fact_;
  std::string resp_buf_set_fact_;
  std::string resp_buf_query_state_;
  std::string resp_buf_load_domain_;
};

}  // namespace ame
