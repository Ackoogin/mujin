#pragma once

#include <TacticalObjectsTypes.h>
#include <StreamingCodec.h>
#include <store/ObjectStore.h>
#include <spatial/SpatialIndex.h>
#include <milclass/MilClassEngine.h>
#include <correlation/CorrelationEngine.h>
#include <zone/ZoneEngine.h>
#include <query/QueryEngine.h>
#include <interest/InterestManager.h>
#include <logging/Logger.h>
#include <tl/optional.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace tactical_objects {

/// \brief Opaque handle returned by registerStreamSubscriber.
using SubscriptionHandle = uint64_t;

/// \brief Callback invoked for each (interest, entity) pair that has changed.
using EntityUpdateCallback = std::function<void(const UUIDKey& interest_id,
                                                 const UUIDKey& entity_id)>;

/// \brief One tick's worth of entity changes for a single subscriber/interest.
struct StreamFrame {
  uint64_t tick_id   = 0;
  double   timestamp = 0.0;
  std::vector<EntityUpdateFrame> updates;
  std::vector<UUIDKey>           deletes;
};

/// \brief Facade integrating all domain engines into a single runtime API.
class TacticalObjectsRuntime {
public:
  TacticalObjectsRuntime();

  // --- Object CRUD ---

  /// \brief Create a new tracked object manually.
  UUIDKey createObject(const ObjectDefinition& def);

  /// \brief Update fields on an existing object.
  bool updateObject(const UUIDKey& id, const ObjectUpdate& update);

  /// \brief Delete an object.
  bool deleteObject(const UUIDKey& id);

  /// \brief Retrieve an entity record, or nullptr if absent.
  const EntityRecord* getRecord(const UUIDKey& id) const;

  // --- Classification ---

  void setMilClass(const UUIDKey& id, const MilClassProfile& profile);
  tl::optional<MilClassProfile> getMilClass(const UUIDKey& id) const;
  std::string deriveSymbolKey(const UUIDKey& id) const;

  // --- Zones ---

  UUIDKey createZone(const ZoneDefinition& def);
  bool removeZone(const UUIDKey& id);
  tl::optional<ZoneDefinition> getZone(const UUIDKey& id) const;
  bool isInsideZone(const Position& pos, const UUIDKey& zone_id) const;
  ZoneRelationship evaluateZoneTransition(const UUIDKey& obj, const UUIDKey& zone,
                                          const Position& pos, double time = 0.0);

  // --- Correlation ---

  CorrelationResult processObservation(const Observation& obs);
  CorrelationResult processObservationBatch(const ObservationBatch& batch);

  // --- Query ---

  QueryResponse query(const QueryRequest& req) const;

  // --- Behavior & state ---

  void setBehavior(const UUIDKey& id, const std::string& behavior_pattern,
                   const std::string& operational_state = "");
  tl::optional<BehaviorComponent> getBehavior(const UUIDKey& id) const;

  // --- Diagnostics ---

  /// \brief Log a diagnostic entry. Used for missing-information events.
  void logMissingInfo(const UUIDKey& id, const std::string& field);

  // --- Interest management ---

  InterestManager& interestManager() { return interest_manager_; }
  const InterestManager& interestManager() const { return interest_manager_; }

  // --- Streaming subscriptions (Step 1) ---

  /// \brief Register a callback to be invoked for every entity matching
  ///        \p interest_id that becomes dirty during a tick.
  ///        Returns a handle that can be passed to removeStreamSubscriber().
  SubscriptionHandle registerStreamSubscriber(const UUIDKey& interest_id,
                                               EntityUpdateCallback cb);

  /// \brief Deregister a previously registered subscriber.
  void removeStreamSubscriber(SubscriptionHandle handle);

  // --- Streaming frame assembly (Steps 3 + 5) ---

  /// \brief Build a StreamFrame for a specific (interest, subscriber) pair,
  ///        applying version-gated delta encoding.
  StreamFrame assembleStreamFrame(const UUIDKey& interest_id,
                                   SubscriptionHandle subscriber,
                                   double timestamp);

  /// \brief Flush all dirty entities: evaluate interests, invoke callbacks,
  ///        advance subscriber versions, clear dirty sets.
  ///        Called by TacticalObjectsComponent::on_tick().
  void flushDirtyEntities(double timestamp);

  // --- Engine access ---

  std::shared_ptr<ObjectStore> store() { return store_; }
  std::shared_ptr<SpatialIndex> spatialIndex() { return spatial_; }
  std::shared_ptr<MilClassEngine> milclassEngine() { return milclass_; }
  std::shared_ptr<CorrelationEngine> correlationEngine() { return correlation_; }
  std::shared_ptr<ZoneEngine> zoneEngine() { return zone_; }
  std::shared_ptr<QueryEngine> queryEngine() { return query_; }

private:
  std::shared_ptr<ObjectStore>       store_;
  std::shared_ptr<SpatialIndex>      spatial_;
  std::shared_ptr<MilClassEngine>    milclass_;
  std::shared_ptr<CorrelationEngine> correlation_;
  std::shared_ptr<ZoneEngine>        zone_;
  std::shared_ptr<QueryEngine>       query_;
  InterestManager                    interest_manager_;
  pyramid::core::logging::Logger     logger_;

  // Dirty entity tracking
  std::unordered_set<UUIDKey> dirty_entities_;
  std::unordered_set<UUIDKey> deleted_entities_;

  // Subscribers: interest_id -> list of (handle, callback)
  std::unordered_map<UUIDKey,
    std::vector<std::pair<SubscriptionHandle, EntityUpdateCallback>>> subscribers_;
  // Reverse map: handle -> interest_id  (for removal)
  std::unordered_map<SubscriptionHandle, UUIDKey> handle_to_interest_;
  uint64_t next_handle_ = 1;

  // Per-subscriber, per-entity last-seen version  (Step 3)
  // subscriber_versions_[handle][entity_id] = last_published_version
  std::unordered_map<SubscriptionHandle,
    std::unordered_map<UUIDKey, uint64_t>> subscriber_versions_;
};

} // namespace tactical_objects
