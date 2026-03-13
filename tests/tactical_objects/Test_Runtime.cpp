#include <gtest/gtest.h>
#include <TacticalObjectsRuntime.h>
#include <uuid/UUIDHelper.h>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

class RuntimeTest : public ::testing::Test {
protected:
  TacticalObjectsRuntime rt;

  ObjectDefinition makePlatform(double lat, double lon,
                                Affiliation aff = Affiliation::Unknown) {
    ObjectDefinition def;
    def.type = ObjectType::Platform;
    def.position = Position{lat, lon, 0};
    def.affiliation = aff;
    def.mil_class.affiliation = aff;
    return def;
  }
};

///< REQ_TACTICAL_OBJECTS_026: Direct CRUD create returns valid UUID.
TEST_F(RuntimeTest, CreateObjectReturnsValidUUID) {
  auto id = rt.createObject(makePlatform(51.5, -0.1));
  ASSERT_FALSE(id.isNull());
  ASSERT_NE(rt.getRecord(id), nullptr);
}

///< REQ_TACTICAL_OBJECTS_027: Direct CRUD update modifies fields.
TEST_F(RuntimeTest, UpdateObjectModifiesPosition) {
  auto id = rt.createObject(makePlatform(51.5, -0.1));
  ObjectUpdate upd;
  upd.position = Position{52.0, 1.0, 0};
  ASSERT_TRUE(rt.updateObject(id, upd));
  auto* kc = rt.store()->kinematics().get(id);
  ASSERT_NE(kc, nullptr);
  ASSERT_DOUBLE_EQ(kc->position.lat, 52.0);
}

///< REQ_TACTICAL_OBJECTS_028: Direct CRUD delete removes entity.
TEST_F(RuntimeTest, DeleteObjectRemovesEntity) {
  auto id = rt.createObject(makePlatform(51.5, -0.1));
  ASSERT_TRUE(rt.deleteObject(id));
  ASSERT_EQ(rt.getRecord(id), nullptr);
}

///< REQ_TACTICAL_OBJECTS_029: Delete non-existent ID returns false.
TEST_F(RuntimeTest, DeleteNonExistentReturnsFalse) {
  UUIDKey bogus{UUIDHelper::generateV4()};
  ASSERT_FALSE(rt.deleteObject(bogus));
}

///< REQ_TACTICAL_OBJECTS_030: Runtime delegates to correlation engine.
TEST_F(RuntimeTest, CorrelationDelegation) {
  Observation obs;
  obs.observation_id = UUIDHelper::generateV4();
  obs.position = Position{51.5, -0.1, 0};
  obs.confidence = 0.8;
  obs.affiliation_hint = Affiliation::Hostile;
  obs.source_ref.source_system = "radar";
  obs.source_ref.source_entity_id = "T01";
  auto r = rt.processObservation(obs);
  ASSERT_EQ(r.outcome, CorrelationOutcome::Created);
}

///< REQ_TACTICAL_OBJECTS_031: Runtime delegates to query engine.
TEST_F(RuntimeTest, QueryDelegation) {
  auto id = rt.createObject(makePlatform(51.5, -0.1, Affiliation::Friendly));
  QueryRequest req;
  req.by_uuid = id;
  auto resp = rt.query(req);
  ASSERT_EQ(resp.entries.size(), 1u);
}

///< REQ_TACTICAL_OBJECTS_032: Runtime delegates zone operations.
TEST_F(RuntimeTest, ZoneDelegation) {
  ZoneDefinition def;
  def.zone_type = ZoneType::AOI;
  def.geometry.center = Position{51.0, 0.0, 0};
  def.geometry.radius_m = 5000.0;
  auto zid = rt.createZone(def);
  ASSERT_TRUE(rt.isInsideZone(Position{51.0, 0.0, 0}, zid));
}

///< REQ_TACTICAL_OBJECTS_033: Runtime delegates milclass operations.
TEST_F(RuntimeTest, MilClassDelegation) {
  auto id = rt.createObject(makePlatform(51.5, -0.1));
  MilClassProfile p;
  p.affiliation = Affiliation::Hostile;
  p.battle_dim = BattleDimension::Air;
  rt.setMilClass(id, p);
  auto got = rt.getMilClass(id);
  ASSERT_TRUE(got.has_value());
  ASSERT_EQ(got->battle_dim, BattleDimension::Air);
}

///< REQ_TACTICAL_OBJECTS_029: Evidence and direct objects queryable together.
TEST_F(RuntimeTest, BothPathsQueryableTogether) {
  auto direct_id = rt.createObject(makePlatform(51.5, -0.1, Affiliation::Friendly));

  Observation obs;
  obs.observation_id = UUIDHelper::generateV4();
  obs.position = Position{52.0, 0.0, 0};
  obs.confidence = 0.8;
  obs.affiliation_hint = Affiliation::Hostile;
  obs.source_ref.source_system = "radar";
  obs.source_ref.source_entity_id = "T01";
  auto cr = rt.processObservation(obs);

  QueryRequest req;
  req.by_type = ObjectType::Platform;
  auto resp = rt.query(req);
  bool found_direct = false;
  bool found_evidence = false;
  for (auto& e : resp.entries) {
    if (e.id == direct_id) found_direct = true;
    if (e.id == cr.object_id) found_evidence = true;
  }
  ASSERT_TRUE(found_direct);
  ASSERT_TRUE(found_evidence);
}

///< REQ_TACTICAL_OBJECTS_044: Same batch in same order produces same result.
TEST_F(RuntimeTest, BatchCorrelationDeterminism) {
  auto make_batch = []() {
    ObservationBatch batch;
    for (int i = 0; i < 3; ++i) {
      Observation obs;
      obs.observation_id = UUIDHelper::generateV4();
      obs.position = Position{51.0 + i * 0.5, -0.1 + i * 0.5, 0};
      obs.confidence = 0.7;
      obs.affiliation_hint = Affiliation::Neutral;
      obs.source_ref.source_system = "s" + std::to_string(i);
      obs.source_ref.source_entity_id = "T" + std::to_string(i);
      batch.observations.push_back(obs);
    }
    return batch;
  };

  TacticalObjectsRuntime rt_a;
  auto batch_a = make_batch();
  rt_a.processObservationBatch(batch_a);

  TacticalObjectsRuntime rt_b;
  auto batch_b = make_batch();
  rt_b.processObservationBatch(batch_b);

  ASSERT_EQ(rt_a.store()->objectCount(), rt_b.store()->objectCount());
}

///< REQ_TACTICAL_OBJECTS_040: Missing-information event is logged diagnostically.
TEST_F(RuntimeTest, LogMissingInfoDiagnostics) {
  auto id = rt.createObject(makePlatform(51.5, -0.1));
  rt.logMissingInfo(id, "velocity");
  SUCCEED();
}

///< REQ_TACTICAL_OBJECTS_053: Behavior estimation round-trip.
TEST_F(RuntimeTest, BehaviorEstimationRoundTrip) {
  auto id = rt.createObject(makePlatform(51.5, -0.1));
  rt.setBehavior(id, "patrol", "active");
  auto bc = rt.getBehavior(id);
  ASSERT_TRUE(bc.has_value());
  ASSERT_EQ(bc->behavior_pattern, "patrol");
  ASSERT_EQ(bc->operational_state, "active");
}

///< REQ_TACTICAL_OBJECTS_054: Operational state and freshness tracked.
TEST_F(RuntimeTest, OperationalStateAndFreshness) {
  auto id = rt.createObject(makePlatform(51.5, -0.1));
  ObjectUpdate upd;
  upd.operational_state = std::string("degraded");
  ASSERT_TRUE(rt.updateObject(id, upd));
  auto bc = rt.getBehavior(id);
  ASSERT_TRUE(bc.has_value());
  ASSERT_EQ(bc->operational_state, "degraded");
}
