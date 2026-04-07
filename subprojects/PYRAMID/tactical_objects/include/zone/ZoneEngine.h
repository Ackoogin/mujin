#pragma once

#include <TacticalObjectsTypes.h>
#include <store/ObjectStore.h>
#include <spatial/SpatialIndex.h>
#include <tl/optional.hpp>

#include <memory>
#include <unordered_map>

namespace tactical_objects {

/// \brief Zone CRUD, geometry checks, and boundary transition detection.
class ZoneEngine {
public:
  ZoneEngine(std::shared_ptr<ObjectStore> store,
             std::shared_ptr<SpatialIndex> spatial);

  /// \brief Create or update a zone. Returns the zone UUID key.
  UUIDKey upsertZone(const ZoneDefinition& def);

  /// \brief Create or update a zone with a known ID.
  void upsertZone(const UUIDKey& id, const ZoneDefinition& def);

  /// \brief Retrieve a zone definition. Empty if absent.
  tl::optional<ZoneDefinition> getZone(const UUIDKey& id) const;

  /// \brief Remove a zone.
  bool removeZone(const UUIDKey& id);

  /// \brief Point-in-zone test.
  bool isInside(const Position& point, const UUIDKey& zone_id) const;

  /// \brief Evaluate the zone transition for an entity, comparing current vs prior state.
  ZoneRelationship evaluateTransition(const UUIDKey& object_id,
                                      const UUIDKey& zone_id,
                                      const Position& current_pos,
                                      double current_time = 0.0);

  /// \brief Compute a bounding box for a zone geometry.
  static BoundingBox computeBoundingBox(const ZoneGeometry& geom);

  /// \brief Haversine distance in meters between two positions.
  static double haversineMeters(const Position& a, const Position& b);

private:
  std::shared_ptr<ObjectStore> store_;
  std::shared_ptr<SpatialIndex> spatial_;

  struct PriorState {
    ZoneRelationship relationship = ZoneRelationship::Outside;
  };

  struct PairKey {
    UUIDKey object_id;
    UUIDKey zone_id;
    bool operator==(const PairKey& o) const {
      return object_id == o.object_id && zone_id == o.zone_id;
    }
  };

  struct PairKeyHash {
    size_t operator()(const PairKey& k) const {
      auto h1 = std::hash<UUIDKey>{}(k.object_id);
      auto h2 = std::hash<UUIDKey>{}(k.zone_id);
      return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
  };

  std::unordered_map<PairKey, PriorState, PairKeyHash> prior_states_;

  static bool pointInPolygon(const Position& point, const std::vector<Position>& vertices);
};

} // namespace tactical_objects
