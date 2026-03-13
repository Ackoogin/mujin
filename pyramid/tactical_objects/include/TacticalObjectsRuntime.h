#pragma once

#include <TacticalObjectsTypes.h>
#include <store/ObjectStore.h>
#include <spatial/SpatialIndex.h>
#include <milclass/MilClassEngine.h>
#include <correlation/CorrelationEngine.h>
#include <zone/ZoneEngine.h>
#include <query/QueryEngine.h>
#include <logging/Logger.h>
#include <tl/optional.hpp>

#include <memory>
#include <string>
#include <vector>

namespace tactical_objects {

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

  // --- Engine access ---

  std::shared_ptr<ObjectStore> store() { return store_; }
  std::shared_ptr<SpatialIndex> spatialIndex() { return spatial_; }
  std::shared_ptr<MilClassEngine> milclassEngine() { return milclass_; }
  std::shared_ptr<CorrelationEngine> correlationEngine() { return correlation_; }
  std::shared_ptr<ZoneEngine> zoneEngine() { return zone_; }
  std::shared_ptr<QueryEngine> queryEngine() { return query_; }

private:
  std::shared_ptr<ObjectStore> store_;
  std::shared_ptr<SpatialIndex> spatial_;
  std::shared_ptr<MilClassEngine> milclass_;
  std::shared_ptr<CorrelationEngine> correlation_;
  std::shared_ptr<ZoneEngine> zone_;
  std::shared_ptr<QueryEngine> query_;
  pyramid::core::logging::Logger logger_;
};

} // namespace tactical_objects
