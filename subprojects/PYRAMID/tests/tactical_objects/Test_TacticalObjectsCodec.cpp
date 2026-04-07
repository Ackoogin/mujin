#include <gtest/gtest.h>
#include <TacticalObjectsCodec.h>
#include <uuid/UUIDHelper.h>
#include <nlohmann/json.hpp>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

///< REQ_TACTICAL_OBJECTS_041: UUID serializes and deserializes to canonical string.
TEST(TacticalObjectsCodec, UUIDRoundTrip) {
  auto uuid = UUIDHelper::generateV4();
  UUIDKey key{uuid};
  auto j = TacticalObjectsCodec::encodeUUID(key);
  auto decoded = TacticalObjectsCodec::decodeUUID(j);
  ASSERT_EQ(key, decoded);
}

///< REQ_TACTICAL_OBJECTS_041: ObservationBatch round-trips through JSON.
TEST(TacticalObjectsCodec, ObservationBatchRoundTrip) {
  ObservationBatch batch;
  for (int i = 0; i < 3; ++i) {
    Observation obs;
    obs.observation_id = UUIDHelper::generateV4();
    obs.received_at = 1000.0 + i;
    obs.observed_at = 999.0 + i;
    obs.position = Position{51.0 + i * 0.1, -0.1 + i * 0.1, 100.0};
    obs.velocity = Velocity{10.0, 20.0, 0.0};
    obs.confidence = 0.7 + i * 0.05;
    obs.affiliation_hint = Affiliation::Hostile;
    obs.object_hint_type = ObjectType::Platform;
    obs.source_ref.source_system = "radar-" + std::to_string(i);
    obs.source_ref.source_entity_id = "T" + std::to_string(i);
    obs.source_ref.first_seen = 900.0;
    obs.source_ref.last_seen = 1000.0;
    obs.source_sidc = "SFGPUCAA";
    batch.observations.push_back(obs);
  }

  auto j = TacticalObjectsCodec::encodeObservationBatch(batch);
  auto decoded = TacticalObjectsCodec::decodeObservationBatch(j);

  ASSERT_EQ(decoded.observations.size(), 3u);
  for (size_t i = 0; i < 3; ++i) {
    auto& orig = batch.observations[i];
    auto& dec = decoded.observations[i];
    ASSERT_EQ(UUIDKey{orig.observation_id}, UUIDKey{dec.observation_id});
    ASSERT_DOUBLE_EQ(orig.position.lat, dec.position.lat);
    ASSERT_DOUBLE_EQ(orig.position.lon, dec.position.lon);
    ASSERT_DOUBLE_EQ(orig.confidence, dec.confidence);
    ASSERT_EQ(orig.affiliation_hint, dec.affiliation_hint);
    ASSERT_EQ(orig.source_ref.source_system, dec.source_ref.source_system);
  }
}

///< REQ_TACTICAL_OBJECTS_041: ObjectDefinition round-trips through JSON.
TEST(TacticalObjectsCodec, ObjectDefinitionRoundTrip) {
  ObjectDefinition def;
  def.type = ObjectType::Unit;
  def.position = Position{52.5, 1.3, 0};
  def.velocity = Velocity{5.0, -3.0, 0.0};
  def.affiliation = Affiliation::Friendly;
  def.mil_class.battle_dim = BattleDimension::Ground;
  def.mil_class.affiliation = Affiliation::Friendly;
  def.mil_class.role = "infantry";
  def.mil_class.hq = true;
  def.identity_name = "Alpha Company";
  SourceRef sr;
  sr.source_system = "c2-system";
  sr.source_entity_id = "UNIT-001";
  def.source_refs.push_back(sr);

  auto j = TacticalObjectsCodec::encodeObjectDefinition(def);
  auto decoded = TacticalObjectsCodec::decodeObjectDefinition(j);

  ASSERT_EQ(decoded.type, ObjectType::Unit);
  ASSERT_DOUBLE_EQ(decoded.position.lat, 52.5);
  ASSERT_EQ(decoded.affiliation, Affiliation::Friendly);
  ASSERT_EQ(decoded.mil_class.role, "infantry");
  ASSERT_TRUE(decoded.mil_class.hq);
  ASSERT_EQ(decoded.identity_name, "Alpha Company");
  ASSERT_EQ(decoded.source_refs.size(), 1u);
  ASSERT_EQ(decoded.source_refs[0].source_system, "c2-system");
}

///< REQ_TACTICAL_OBJECTS_041: QueryRequest/Response round-trips through JSON.
TEST(TacticalObjectsCodec, QueryRoundTrip) {
  QueryRequest req;
  req.by_type = ObjectType::Platform;
  req.by_affiliation = Affiliation::Hostile;
  req.by_region = BoundingBox{50.0, 52.0, -1.0, 1.0};

  auto j = TacticalObjectsCodec::encodeQueryRequest(req);
  auto decoded_req = TacticalObjectsCodec::decodeQueryRequest(j);

  ASSERT_TRUE(decoded_req.by_type.has_value());
  ASSERT_EQ(*decoded_req.by_type, ObjectType::Platform);
  ASSERT_TRUE(decoded_req.by_affiliation.has_value());
  ASSERT_EQ(*decoded_req.by_affiliation, Affiliation::Hostile);
  ASSERT_TRUE(decoded_req.by_region.has_value());
  ASSERT_DOUBLE_EQ(decoded_req.by_region->min_lat, 50.0);

  QueryResponse resp;
  QueryResultEntry e1;
  e1.id = UUIDKey{UUIDHelper::generateV4()};
  e1.record.type = ObjectType::Platform;
  e1.record.version = 3;
  resp.entries.push_back(e1);
  QueryResultEntry e2;
  e2.id = UUIDKey{UUIDHelper::generateV4()};
  e2.record.type = ObjectType::Person;
  e2.record.version = 1;
  resp.entries.push_back(e2);
  resp.total = 2;

  auto jr = TacticalObjectsCodec::encodeQueryResponse(resp);
  auto decoded_resp = TacticalObjectsCodec::decodeQueryResponse(jr);

  ASSERT_EQ(decoded_resp.entries.size(), 2u);
  ASSERT_EQ(decoded_resp.total, 2u);
  ASSERT_EQ(decoded_resp.entries[0].id, e1.id);
  ASSERT_EQ(decoded_resp.entries[1].record.type, ObjectType::Person);
}

///< REQ_TACTICAL_OBJECTS_041: ObjectUpdate encodes and decodes with all optional fields.
TEST(TacticalObjectsCodec, EncodeDecodeObjectUpdate) {
  ObjectUpdate upd;
  upd.position = Position{51.5, -0.1, 100.0};
  Velocity v; v.north = 10.0; v.east = -5.0; v.down = 0.5;
  upd.velocity = v;
  upd.affiliation = Affiliation::Hostile;
  MilClassProfile p; p.battle_dim = BattleDimension::Air; p.affiliation = Affiliation::Hostile;
  upd.mil_class = p;
  upd.identity_name = std::string("Unit-42");
  upd.operational_state = std::string("active");
  upd.behavior_pattern = std::string("patrol");

  auto j = TacticalObjectsCodec::encodeObjectUpdate(upd);
  auto decoded = TacticalObjectsCodec::decodeObjectUpdate(j);

  ASSERT_TRUE(decoded.position.has_value());
  ASSERT_DOUBLE_EQ(decoded.position->lat, 51.5);
  ASSERT_TRUE(decoded.velocity.has_value());
  ASSERT_DOUBLE_EQ(decoded.velocity->north, 10.0);
  ASSERT_DOUBLE_EQ(decoded.velocity->east, -5.0);
  ASSERT_DOUBLE_EQ(decoded.velocity->down, 0.5);
  ASSERT_TRUE(decoded.affiliation.has_value());
  ASSERT_EQ(*decoded.affiliation, Affiliation::Hostile);
  ASSERT_TRUE(decoded.mil_class.has_value());
  ASSERT_EQ(decoded.mil_class->battle_dim, BattleDimension::Air);
  ASSERT_TRUE(decoded.identity_name.has_value());
  ASSERT_EQ(*decoded.identity_name, "Unit-42");
  ASSERT_TRUE(decoded.operational_state.has_value());
  ASSERT_EQ(*decoded.operational_state, "active");
  ASSERT_TRUE(decoded.behavior_pattern.has_value());
  ASSERT_EQ(*decoded.behavior_pattern, "patrol");
}

///< REQ_TACTICAL_OBJECTS_041: All Affiliation enum values round-trip through ObjectDefinition.
TEST(TacticalObjectsCodec, AllAffiliationValuesRoundTrip) {
  // Exercise affiliationToString/stringToAffiliation for all enum values via
  // encodeObjectDefinition / decodeObjectDefinition which use these private helpers.
  const Affiliation affiliations[] = {
    Affiliation::Friendly, Affiliation::Hostile, Affiliation::Neutral, Affiliation::Unknown,
    Affiliation::AssumedFriend, Affiliation::Suspect, Affiliation::Joker,
    Affiliation::Faker, Affiliation::Pending
  };
  for (auto aff : affiliations) {
    ObjectDefinition def;
    def.affiliation = aff;
    def.mil_class.affiliation = aff;
    auto j = TacticalObjectsCodec::encodeObjectDefinition(def);
    auto decoded = TacticalObjectsCodec::decodeObjectDefinition(j);
    EXPECT_EQ(decoded.affiliation, aff);
    EXPECT_EQ(decoded.mil_class.affiliation, aff);
  }
  // default case: out-of-range value encodes through affiliation field
  ObjectDefinition def_bad;
  def_bad.affiliation = static_cast<Affiliation>(999);
  auto j_bad = TacticalObjectsCodec::encodeObjectDefinition(def_bad);
  auto decoded_bad = TacticalObjectsCodec::decodeObjectDefinition(j_bad);
  EXPECT_EQ(decoded_bad.affiliation, Affiliation::Unknown); // default maps back to Unknown
}

///< REQ_TACTICAL_OBJECTS_041: All ZoneType enum values round-trip through ZoneDefinition.
TEST(TacticalObjectsCodec, AllZoneTypeValuesRoundTrip) {
  const ZoneType zone_types[] = {
    ZoneType::AOI, ZoneType::PatrolArea, ZoneType::RestrictedArea,
    ZoneType::NoGoArea, ZoneType::KillBox, ZoneType::PhaseLine,
    ZoneType::Boundary, ZoneType::RouteCorridor, ZoneType::SensorCoverageArea
  };
  for (auto zt : zone_types) {
    ZoneDefinition def;
    def.zone_type = zt;
    auto j = TacticalObjectsCodec::encodeZoneDefinition(def);
    auto decoded = TacticalObjectsCodec::decodeZoneDefinition(j);
    EXPECT_EQ(decoded.zone_type, zt);
  }
  // default case for zoneTypeToString
  ZoneDefinition def_bad;
  def_bad.zone_type = static_cast<ZoneType>(999);
  auto j_bad = TacticalObjectsCodec::encodeZoneDefinition(def_bad);
  auto decoded_bad = TacticalObjectsCodec::decodeZoneDefinition(j_bad);
  EXPECT_EQ(decoded_bad.zone_type, ZoneType::AOI);
}

///< REQ_TACTICAL_OBJECTS_041: All BattleDimension enum values round-trip through ObjectDefinition.
TEST(TacticalObjectsCodec, AllBattleDimValuesRoundTrip) {
  const BattleDimension dims[] = {
    BattleDimension::Ground, BattleDimension::Air, BattleDimension::SeaSurface,
    BattleDimension::Subsurface, BattleDimension::Space, BattleDimension::SOF
  };
  for (auto bd : dims) {
    ObjectDefinition def;
    def.mil_class.battle_dim = bd;
    auto j = TacticalObjectsCodec::encodeObjectDefinition(def);
    auto decoded = TacticalObjectsCodec::decodeObjectDefinition(j);
    EXPECT_EQ(decoded.mil_class.battle_dim, bd);
  }
  // default case for battleDimToString
  ObjectDefinition def_bad;
  def_bad.mil_class.battle_dim = static_cast<BattleDimension>(999);
  auto j_bad = TacticalObjectsCodec::encodeObjectDefinition(def_bad);
  auto decoded_bad = TacticalObjectsCodec::decodeObjectDefinition(j_bad);
  EXPECT_EQ(decoded_bad.mil_class.battle_dim, BattleDimension::Ground);
}

///< REQ_TACTICAL_OBJECTS_041: objectTypeFromString and affiliationFromString wrappers.
TEST(TacticalObjectsCodec, ObjectTypeAndAffiliationFromString) {
  EXPECT_EQ(TacticalObjectsCodec::objectTypeFromString("Person"), ObjectType::Person);
  EXPECT_EQ(TacticalObjectsCodec::objectTypeFromString("Equipment"), ObjectType::Equipment);
  EXPECT_EQ(TacticalObjectsCodec::objectTypeFromString("Unit"), ObjectType::Unit);
  EXPECT_EQ(TacticalObjectsCodec::affiliationFromString("Hostile"), Affiliation::Hostile);
  EXPECT_EQ(TacticalObjectsCodec::affiliationFromString("Friendly"), Affiliation::Friendly);
  EXPECT_EQ(TacticalObjectsCodec::affiliationFromString("Neutral"), Affiliation::Neutral);
}

///< REQ_TACTICAL_OBJECTS_041: objectTypeToString default case (public function).
TEST(TacticalObjectsCodec, ObjectTypeDefaultCase) {
  EXPECT_EQ(TacticalObjectsCodec::objectTypeToString(ObjectType::Platform), "Platform");
  // Out-of-range cast reaches the default branch of the public function
  EXPECT_EQ(TacticalObjectsCodec::objectTypeToString(static_cast<ObjectType>(999)), "Platform");
}

///< REQ_TACTICAL_OBJECTS_041: decodeUUID returns null key for invalid UUID string.
TEST(TacticalObjectsCodec, DecodeUUIDBadString) {
  nlohmann::json j = "not-a-uuid";
  auto result = TacticalObjectsCodec::decodeUUID(j);
  ASSERT_TRUE(result.isNull());
}

///< REQ_TACTICAL_OBJECTS_041: ZoneDefinition round-trips through JSON.
TEST(TacticalObjectsCodec, ZoneDefinitionRoundTrip) {
  ZoneDefinition def;
  def.zone_type = ZoneType::PatrolArea;
  def.active_from = 1000.0;
  def.active_until = 5000.0;
  def.priority = 3;
  def.owner = "squadron-1";
  def.semantics = "patrol";
  def.geometry.center = Position{51.5, -0.1, 0};
  def.geometry.radius_m = 5000.0;
  def.geometry.vertices = {
    Position{51.4, -0.2, 0}, Position{51.6, -0.2, 0},
    Position{51.6, 0.0, 0}, Position{51.4, 0.0, 0},
    Position{51.4, -0.2, 0}
  };

  auto j = TacticalObjectsCodec::encodeZoneDefinition(def);
  auto decoded = TacticalObjectsCodec::decodeZoneDefinition(j);

  ASSERT_EQ(decoded.zone_type, ZoneType::PatrolArea);
  ASSERT_DOUBLE_EQ(decoded.active_from, 1000.0);
  ASSERT_DOUBLE_EQ(decoded.active_until, 5000.0);
  ASSERT_EQ(decoded.priority, 3);
  ASSERT_EQ(decoded.owner, "squadron-1");
  ASSERT_EQ(decoded.geometry.vertices.size(), 5u);
  ASSERT_DOUBLE_EQ(decoded.geometry.center.lat, 51.5);
  ASSERT_DOUBLE_EQ(decoded.geometry.radius_m, 5000.0);
}

///< Coverage: TacticalObjectsCodec::battleDimensionFromString (lines 19-20).
TEST(TacticalObjectsCodec, BattleDimensionFromString) {
  EXPECT_EQ(TacticalObjectsCodec::battleDimensionFromString("Air"),        BattleDimension::Air);
  EXPECT_EQ(TacticalObjectsCodec::battleDimensionFromString("Ground"),     BattleDimension::Ground);
  EXPECT_EQ(TacticalObjectsCodec::battleDimensionFromString("SeaSurface"), BattleDimension::SeaSurface);
  EXPECT_EQ(TacticalObjectsCodec::battleDimensionFromString("Subsurface"), BattleDimension::Subsurface);
  EXPECT_EQ(TacticalObjectsCodec::battleDimensionFromString("Space"),      BattleDimension::Space);
  EXPECT_EQ(TacticalObjectsCodec::battleDimensionFromString("SOF"),        BattleDimension::SOF);
}
