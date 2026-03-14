#pragma once

#include <TacticalObjectsTypes.h>
#include <store/ObjectComponents.h>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace tactical_objects {

struct EntityRecord {
  UUIDKey id;
  ObjectType type = ObjectType::Platform;
  uint64_t version = 0;
};

/// \brief ECS-style sparse-set object store with UUID identity and versioning.
class ObjectStore {
public:
  /// \brief Create a new entity, returning its UUID key.
  UUIDKey createObject(ObjectType type);

  /// \brief Look up an entity record by UUID. Returns nullptr if absent.
  const EntityRecord* getRecord(const UUIDKey& id) const;

  /// \brief Remove an entity and all its components. Returns true if found.
  bool deleteObject(const UUIDKey& id);

  /// \brief Number of live entities.
  size_t objectCount() const;

  /// \brief Increment an entity's version counter.
  void bumpVersion(const UUIDKey& id);

  // --- Dirty mask tracking (Step 3) ---

  /// \brief Get the accumulated dirty field-mask for an entity since last clear.
  uint16_t getDirtyMask(const UUIDKey& id) const;

  /// \brief OR additional dirty bits into an entity's mask.
  void setDirtyBits(const UUIDKey& id, uint16_t bits);

  /// \brief Clear the dirty mask for one entity.
  void clearDirtyMask(const UUIDKey& id);

  /// \brief Clear dirty masks for all entities.
  void clearAllDirtyMasks();

  /// \brief Iterate all entity records.
  template <typename Fn>
  void forEachObject(Fn fn) const {
    for (auto& rec : dense_) {
      fn(rec);
    }
  }

  ComponentTable<IdentityComponent>& identities() { return identities_; }
  const ComponentTable<IdentityComponent>& identities() const { return identities_; }

  ComponentTable<KinematicsComponent>& kinematics() { return kinematics_; }
  const ComponentTable<KinematicsComponent>& kinematics() const { return kinematics_; }

  ComponentTable<MilClassComponent>& milclass() { return milclass_; }
  const ComponentTable<MilClassComponent>& milclass() const { return milclass_; }

  ComponentTable<CorrelationComponent>& correlation() { return correlation_; }
  const ComponentTable<CorrelationComponent>& correlation() const { return correlation_; }

  ComponentTable<QualityComponent>& quality() { return quality_; }
  const ComponentTable<QualityComponent>& quality() const { return quality_; }

  ComponentTable<GeometryComponent>& geometries() { return geometries_; }
  const ComponentTable<GeometryComponent>& geometries() const { return geometries_; }

  ComponentTable<LifecycleComponent>& lifecycle() { return lifecycle_; }
  const ComponentTable<LifecycleComponent>& lifecycle() const { return lifecycle_; }

  ComponentTable<BehaviorComponent>& behaviors() { return behaviors_; }
  const ComponentTable<BehaviorComponent>& behaviors() const { return behaviors_; }

  ComponentTable<ZoneComponent>& zones() { return zones_; }
  const ComponentTable<ZoneComponent>& zones() const { return zones_; }

private:
  std::vector<EntityRecord> dense_;
  std::unordered_map<UUIDKey, size_t> sparse_;

  ComponentTable<IdentityComponent> identities_;
  ComponentTable<KinematicsComponent> kinematics_;
  ComponentTable<MilClassComponent> milclass_;
  ComponentTable<CorrelationComponent> correlation_;
  ComponentTable<QualityComponent> quality_;
  ComponentTable<GeometryComponent> geometries_;
  ComponentTable<LifecycleComponent> lifecycle_;
  ComponentTable<BehaviorComponent> behaviors_;
  ComponentTable<ZoneComponent> zones_;

  void removeFromAllComponents(const UUIDKey& id);

  std::unordered_map<UUIDKey, uint16_t> dirty_masks_;
};

} // namespace tactical_objects
