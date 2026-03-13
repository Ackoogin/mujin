#pragma once

#include <TacticalObjectsTypes.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace tactical_objects {

/// \brief Sparse component table keyed by UUIDKey.
/// Only entities that actually carry a given component consume storage.
template <typename T>
class ComponentTable {
public:
  /// \brief Check whether the entity has this component.
  bool has(const UUIDKey& key) const {
    return data_.count(key) > 0;
  }

  /// \brief Read-only access. Returns nullptr if absent.
  const T* get(const UUIDKey& key) const {
    auto it = data_.find(key);
    return it != data_.end() ? &it->second : nullptr;
  }

  /// \brief Mutable access. Returns nullptr if absent.
  T* getMutable(const UUIDKey& key) {
    auto it = data_.find(key);
    return it != data_.end() ? &it->second : nullptr;
  }

  /// \brief Insert or overwrite a component value.
  void set(const UUIDKey& key, T value) {
    data_[key] = std::move(value);
  }

  /// \brief Remove the component for an entity.
  void remove(const UUIDKey& key) {
    data_.erase(key);
  }

  /// \brief Number of entities with this component.
  size_t size() const { return data_.size(); }

  /// \brief Iterate all entries.
  template <typename Fn>
  void forEach(Fn fn) const {
    for (auto& kv : data_) {
      fn(kv.first, kv.second);
    }
  }

private:
  std::unordered_map<UUIDKey, T> data_;
};

struct IdentityComponent {
  std::string name;
  std::vector<SourceRef> source_refs;
};

struct KinematicsComponent {
  Position position;
  Velocity velocity;
  double timestamp = 0.0;
};

struct MilClassComponent {
  MilClassProfile profile;
};

struct CorrelationComponent {
  std::vector<UUIDKey> contributing_observations;
  std::vector<UUIDKey> contributing_tracks;
  std::vector<SourceRef> source_refs;
  double confidence = 0.0;
  Provenance provenance = Provenance::Direct;
};

struct QualityComponent {
  double confidence = 0.0;
  double completeness = 0.0;
  double freshness_timestamp = 0.0;
};

struct GeometryComponent {
  ZoneGeometry geometry;
};

struct LifecycleComponent {
  LifecycleStatus status = LifecycleStatus::Active;
  double last_update_timestamp = 0.0;
  uint64_t version = 0;
};

struct BehaviorComponent {
  std::string behavior_pattern;
  std::string intent_hypothesis;
  std::string operational_state;
};

struct ZoneComponent {
  ZoneType zone_type = ZoneType::AOI;
  double active_from = 0.0;
  double active_until = 0.0;
  int priority = 0;
  std::string owner;
  std::string semantics;
};

} // namespace tactical_objects
