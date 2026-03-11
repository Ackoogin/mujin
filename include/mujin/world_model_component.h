#pragma once

#include <mujin/world_model.h>
#include <mujin/wm_audit_log.h>
#include <pcl/component.hpp>

#include <atomic>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace mujin {

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

/// \brief PCL-backed world model component with ROS-agnostic business logic.
class WorldModelComponent : public pcl::Component {
public:
  WorldModelComponent();

  /// \brief Access the canonical world model for in-process integrations.
  WorldModel& worldModel() { return wm_; }

  /// \brief Access the canonical world model for in-process integrations.
  const WorldModel& worldModel() const { return wm_; }

  /// \brief Query a single fact by key.
  GetFactResult getFact(const std::string& key) const;

  /// \brief Set a single fact by key.
  SetFactResult setFact(const std::string& key, bool value, const std::string& source);

  /// \brief Build a snapshot of either all true facts or the requested keys.
  WorldStateSnapshot queryState(const std::vector<std::string>& keys) const;

  /// \brief Returns true once after the state changes.
  bool consumeStateDirty();

protected:
  pcl_status_t on_configure() override;
  pcl_status_t on_activate() override;
  pcl_status_t on_deactivate() override;
  pcl_status_t on_cleanup() override;
  pcl_status_t on_shutdown() override;

private:
  void loadDomainFromParams();
  WorldStateSnapshot buildSnapshot(const std::vector<std::string>& keys) const;

  WorldModel wm_;
  std::optional<WmAuditLog> audit_log_;
  std::atomic<bool> state_dirty_{false};
};

}  // namespace mujin
