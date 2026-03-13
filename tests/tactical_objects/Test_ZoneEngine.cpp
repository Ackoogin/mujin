#include <gtest/gtest.h>
#include <zone/ZoneEngine.h>
#include <store/ObjectStore.h>
#include <spatial/SpatialIndex.h>

using namespace tactical_objects;

class ZoneEngineTest : public ::testing::Test {
protected:
  std::shared_ptr<ObjectStore> store = std::make_shared<ObjectStore>();
  std::shared_ptr<SpatialIndex> spatial = std::make_shared<SpatialIndex>(1.0);
  ZoneEngine engine{store, spatial};

  UUIDKey makeSquareZone(double lat, double lon, double half_deg) {
    ZoneDefinition def;
    def.zone_type = ZoneType::NoGoArea;
    def.geometry.geometry_type = ZoneType::NoGoArea;
    def.geometry.vertices = {
      Position{lat - half_deg, lon - half_deg, 0},
      Position{lat - half_deg, lon + half_deg, 0},
      Position{lat + half_deg, lon + half_deg, 0},
      Position{lat + half_deg, lon - half_deg, 0}
    };
    return engine.upsertZone(def);
  }
};

///< REQ_TACTICAL_OBJECTS_017: ZoneEngine supports polygon zones.
TEST_F(ZoneEngineTest, CreatePolygonZone) {
  auto id = makeSquareZone(51.5, -0.1, 0.5);
  auto got = engine.getZone(id);
  ASSERT_TRUE(got.has_value());
  ASSERT_EQ(got->zone_type, ZoneType::NoGoArea);
}

///< REQ_TACTICAL_OBJECTS_017: ZoneEngine supports circle zones.
TEST_F(ZoneEngineTest, CreateCircleZone) {
  ZoneDefinition def;
  def.zone_type = ZoneType::AOI;
  def.geometry.geometry_type = ZoneType::AOI;
  def.geometry.center = Position{51.5, -0.1, 0};
  def.geometry.radius_m = 5000.0;
  auto id = engine.upsertZone(def);
  auto got = engine.getZone(id);
  ASSERT_TRUE(got.has_value());
}

///< REQ_TACTICAL_OBJECTS_018: Bounding box cached for non-point geometries.
TEST_F(ZoneEngineTest, BoundingBoxContainsAllVertices) {
  ZoneDefinition def;
  def.zone_type = ZoneType::KillBox;
  def.geometry.geometry_type = ZoneType::KillBox;
  def.geometry.vertices = {
    Position{10.0, 20.0, 0},
    Position{12.0, 20.0, 0},
    Position{12.0, 22.0, 0},
    Position{10.0, 22.0, 0}
  };
  auto id = engine.upsertZone(def);
  auto got = engine.getZone(id);
  ASSERT_TRUE(got.has_value());
  auto& bb = got->geometry.cached_bbox;
  for (auto& v : def.geometry.vertices) {
    ASSERT_TRUE(bb.contains(v.lat, v.lon));
  }
}

///< REQ_TACTICAL_OBJECTS_019: Point-in-polygon check.
TEST_F(ZoneEngineTest, PointInsidePolygon) {
  auto id = makeSquareZone(50.0, 0.0, 1.0);
  ASSERT_TRUE(engine.isInside(Position{50.0, 0.0, 0}, id));
  ASSERT_FALSE(engine.isInside(Position{55.0, 10.0, 0}, id));
}

///< REQ_TACTICAL_OBJECTS_019: Point-in-circle check.
TEST_F(ZoneEngineTest, PointInsideCircle) {
  ZoneDefinition def;
  def.zone_type = ZoneType::AOI;
  def.geometry.geometry_type = ZoneType::AOI;
  def.geometry.center = Position{51.5, -0.1, 0};
  def.geometry.radius_m = 1000.0;
  auto id = engine.upsertZone(def);

  ASSERT_TRUE(engine.isInside(Position{51.5, -0.1, 0}, id));
  ASSERT_FALSE(engine.isInside(Position{52.0, 1.0, 0}, id));
}

///< REQ_TACTICAL_OBJECTS_020: Enter and leave transitions detected.
TEST_F(ZoneEngineTest, BoundaryTransitionDetection) {
  auto zone_id = makeSquareZone(50.0, 0.0, 1.0);
  auto obj_id = UUIDKey{pyramid::core::uuid::UUIDHelper::generateV4()};

  auto r1 = engine.evaluateTransition(obj_id, zone_id, Position{55.0, 10.0, 0});
  ASSERT_EQ(r1, ZoneRelationship::Outside);

  auto r2 = engine.evaluateTransition(obj_id, zone_id, Position{50.0, 0.0, 0});
  ASSERT_EQ(r2, ZoneRelationship::Entering);

  auto r3 = engine.evaluateTransition(obj_id, zone_id, Position{55.0, 10.0, 0});
  ASSERT_EQ(r3, ZoneRelationship::Leaving);
}

///< REQ_TACTICAL_OBJECTS_021: Expired zones excluded from results.
TEST_F(ZoneEngineTest, ExpiredZoneExcluded) {
  ZoneDefinition def;
  def.zone_type = ZoneType::NoGoArea;
  def.geometry.geometry_type = ZoneType::NoGoArea;
  def.geometry.vertices = {
    Position{49.0, -1.0, 0},
    Position{49.0, 1.0, 0},
    Position{51.0, 1.0, 0},
    Position{51.0, -1.0, 0}
  };
  def.active_from = 100.0;
  def.active_until = 200.0;
  auto zone_id = engine.upsertZone(def);

  auto obj_id = UUIDKey{pyramid::core::uuid::UUIDHelper::generateV4()};
  auto r = engine.evaluateTransition(obj_id, zone_id, Position{50.0, 0.0, 0}, 300.0);
  ASSERT_EQ(r, ZoneRelationship::Outside);
}

///< REQ_TACTICAL_OBJECTS_055: Zone semantic type is preserved and queryable.
TEST_F(ZoneEngineTest, ZoneSemanticTypeRoundTrip) {
  ZoneDefinition def;
  def.zone_type = ZoneType::SensorCoverageArea;
  def.geometry.center = Position{51.0, 0.0, 0};
  def.geometry.radius_m = 500.0;
  def.semantics = "radar-coverage";
  auto id = engine.upsertZone(def);
  auto got = engine.getZone(id);
  ASSERT_TRUE(got.has_value());
  ASSERT_EQ(got->zone_type, ZoneType::SensorCoverageArea);
  ASSERT_EQ(got->semantics, "radar-coverage");
}

///< REQ_TACTICAL_OBJECTS_019: isInside returns false for zone with no geometry.
TEST_F(ZoneEngineTest, IsInsideWithNoGeometryReturnsFalse) {
  // Create a zone with neither vertices nor radius — fallback path returns false
  ZoneDefinition def;
  def.zone_type = ZoneType::AOI;
  def.geometry.radius_m = 0.0;
  def.geometry.vertices.clear();
  auto id = engine.upsertZone(def);
  ASSERT_FALSE(engine.isInside(Position{51.0, 0.0, 0}, id));
}

///< REQ_TACTICAL_OBJECTS_021: evaluateTransition returns Outside before active_from.
TEST_F(ZoneEngineTest, EvaluateTransitionBeforeActivationTime) {
  ZoneDefinition def;
  def.zone_type = ZoneType::PatrolArea;
  def.geometry.center = Position{50.0, 0.0, 0};
  def.geometry.radius_m = 10000.0;
  def.active_from = 500.0;
  def.active_until = 1000.0;
  auto zone_id = engine.upsertZone(def);

  auto obj_id = UUIDKey{pyramid::core::uuid::UUIDHelper::generateV4()};
  // Object is spatially inside the zone but queried before active_from
  auto r = engine.evaluateTransition(obj_id, zone_id, Position{50.0, 0.0, 0}, 200.0);
  ASSERT_EQ(r, ZoneRelationship::Outside);
}

///< REQ_TACTICAL_OBJECTS_056: Circle and proximity checks use geodesic reasoning.
TEST_F(ZoneEngineTest, GeodesicDistanceUsedForCircleChecks) {
  ZoneDefinition def;
  def.zone_type = ZoneType::AOI;
  def.geometry.center = Position{70.0, 25.0, 0};
  def.geometry.radius_m = 10000.0;
  auto id = engine.upsertZone(def);

  ASSERT_TRUE(engine.isInside(Position{70.0, 25.0, 0}, id));
  ASSERT_FALSE(engine.isInside(Position{70.5, 25.0, 0}, id));
}
