#pragma once

#include <uuid/UUID.h>
#include <uuid/UUIDHelper.h>
#include <tl/optional.hpp>

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace tactical_objects {

enum class ObjectType {
  Platform,
  Person,
  Equipment,
  Unit,
  Formation,
  Installation,
  Feature,
  Route,
  Point,
  Area,
  Zone
};

// Ordinals match pyramid.data_model.common.StandardIdentity proto values.
enum class Affiliation {
  Unspecified   = 0,  // no standard equivalent
  Unknown       = 1,  // STANDARD_IDENTITY_UNKNOWN
  Friendly      = 2,  // STANDARD_IDENTITY_FRIENDLY
  Hostile       = 3,  // STANDARD_IDENTITY_HOSTILE
  Suspect       = 4,  // STANDARD_IDENTITY_SUSPECT
  Neutral       = 5,  // STANDARD_IDENTITY_NEUTRAL
  Pending       = 6,  // STANDARD_IDENTITY_PENDING
  Joker         = 7,  // STANDARD_IDENTITY_JOKER
  Faker         = 8,  // STANDARD_IDENTITY_FAKER
  AssumedFriend = 9   // STANDARD_IDENTITY_ASSUMED_FRIENDLY
};

// Ordinals match pyramid.data_model.common.BattleDimension proto values.
// Space and SOF are domain extensions with no standard equivalent.
enum class BattleDimension {
  Unspecified  = 0,  // no standard equivalent
  Ground       = 1,  // BATTLE_DIMENSION_GROUND
  Subsurface   = 2,  // BATTLE_DIMENSION_SUBSURFACE
  SeaSurface   = 3,  // BATTLE_DIMENSION_SEA_SURFACE
  Air          = 4,  // BATTLE_DIMENSION_AIR
  Unknown      = 5,  // BATTLE_DIMENSION_UNKNOWN
  Space        = 6,  // domain extension
  SOF          = 7   // domain extension
};

enum class MilStatus {
  Present,
  Anticipated,
  Planned,
  Known
};

enum class Echelon {
  Team,
  Squad,
  Section,
  Platoon,
  Company,
  Battalion,
  Regiment,
  Brigade,
  Division,
  Corps,
  Army,
  ArmyGroup
};

enum class Mobility {
  None,
  Wheeled,
  Tracked,
  Towed,
  Rail,
  OverSnow,
  Sled,
  Barge,
  Amphibious
};

enum class ZoneType {
  AOI,
  PatrolArea,
  RestrictedArea,
  NoGoArea,
  KillBox,
  PhaseLine,
  Boundary,
  RouteCorridor,
  SensorCoverageArea
};

enum class RelationshipType {
  Hierarchical,
  Organizational,
  Tactical,
  Proximity,
  EquipmentOnPlatform,
  MemberOfUnit,
  UnitInFormation,
  Tracking,
  Escorting,
  Engaging,
  Protecting,
  Following,
  Supporting
};

enum class Provenance {
  Correlated,
  Direct
};

enum class LifecycleStatus {
  Active,
  Stale,
  Retired
};

enum class ZoneRelationship {
  Inside,
  Outside,
  Entering,
  Leaving,
  Intersecting
};

struct UUIDKey {
  pyramid::core::uuid::UUID uuid;

  UUIDKey() : uuid(pyramid::core::uuid::UUIDHelper::getNullUUID()) {}
  explicit UUIDKey(const pyramid::core::uuid::UUID& u) : uuid(u) {}

  bool operator==(const UUIDKey& other) const { return uuid == other.uuid; }
  bool operator!=(const UUIDKey& other) const { return uuid != other.uuid; }
  bool operator<(const UUIDKey& other) const { return uuid < other.uuid; }

  bool isNull() const { return uuid == pyramid::core::uuid::UUIDHelper::getNullUUID(); }
};

struct Position {
  double lat = 0.0;  // radians
  double lon = 0.0;  // radians
  double alt = 0.0;  // metres
};

struct Velocity {
  double north = 0.0;
  double east = 0.0;
  double down = 0.0;
};

struct SourceRef {
  std::string source_system;
  std::string source_entity_id;
  std::string source_track_id;
  double first_seen = 0.0;
  double last_seen = 0.0;
};

struct BoundingBox {
  double min_lat = 0.0;  // radians
  double max_lat = 0.0;  // radians
  double min_lon = 0.0;  // radians
  double max_lon = 0.0;  // radians

  bool contains(double lat, double lon) const {
    return lat >= min_lat && lat <= max_lat &&
           lon >= min_lon && lon <= max_lon;
  }
};

struct MilClassProfile {
  BattleDimension battle_dim = BattleDimension::Ground;
  Affiliation affiliation = Affiliation::Unknown;
  std::string role;
  MilStatus status = MilStatus::Present;
  Echelon echelon = Echelon::Team;
  Mobility mobility = Mobility::None;
  bool hq = false;
  bool task_force = false;
  bool feint_dummy = false;
  bool installation = false;
  std::string source_sidc;

  bool operator==(const MilClassProfile& o) const {
    return battle_dim == o.battle_dim &&
           affiliation == o.affiliation &&
           role == o.role &&
           status == o.status &&
           echelon == o.echelon &&
           mobility == o.mobility &&
           hq == o.hq &&
           task_force == o.task_force &&
           feint_dummy == o.feint_dummy &&
           installation == o.installation &&
           source_sidc == o.source_sidc;
  }
  bool operator!=(const MilClassProfile& o) const { return !(*this == o); }
};

struct ZoneGeometry {
  ZoneType geometry_type = ZoneType::AOI;
  std::vector<Position> vertices;
  Position center;
  double radius_m = 0.0;
  BoundingBox cached_bbox;
};

struct Observation {
  pyramid::core::uuid::UUID observation_id;
  double received_at = 0.0;
  double observed_at = 0.0;
  SourceRef source_ref;
  ObjectType object_hint_type = ObjectType::Platform;
  Position position;
  Velocity velocity;
  std::string classification_hint;
  Affiliation affiliation_hint = Affiliation::Unknown;
  double confidence = 0.0;
  double uncertainty_radius_m = 0.0;
  std::string source_sidc;
  std::vector<UUIDKey> lineage_parent_ids;
};

struct ObservationBatch {
  std::vector<Observation> observations;
};

struct ObjectDefinition {
  ObjectType type = ObjectType::Platform;
  Position position;
  Velocity velocity;
  Affiliation affiliation = Affiliation::Unknown;
  MilClassProfile mil_class;
  std::string identity_name;
  std::vector<SourceRef> source_refs;
};

struct ObjectUpdate {
  tl::optional<Position> position;
  tl::optional<Velocity> velocity;
  tl::optional<Affiliation> affiliation;
  tl::optional<MilClassProfile> mil_class;
  tl::optional<std::string> identity_name;
  tl::optional<std::string> operational_state;
  tl::optional<std::string> behavior_pattern;
};

struct ZoneDefinition {
  ZoneType zone_type = ZoneType::AOI;
  ZoneGeometry geometry;
  double active_from = 0.0;
  double active_until = 0.0;
  int priority = 0;
  std::string owner;
  std::string semantics;
};

} // namespace tactical_objects

namespace std {
template <>
struct hash<tactical_objects::UUIDKey> {
  size_t operator()(const tactical_objects::UUIDKey& k) const {
    size_t h = 0;
    for (auto b : k.uuid.bytes) {
      h ^= std::hash<uint8_t>{}(b) + 0x9e3779b9 + (h << 6) + (h >> 2);
    }
    return h;
  }
};
} // namespace std
