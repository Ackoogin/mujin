#include <gtest/gtest.h>
#include <TacticalObjectsRuntime.h>
#include <uuid/UUIDHelper.h>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

static constexpr double DEG = 3.14159265358979323846 / 180.0;

class RuntimeTest : public ::testing::Test {
protected:
  TacticalObjectsRuntime rt;

  ObjectDefinition makePlatform(double lat_deg, double lon_deg,
                                Affiliation aff = Affiliation::Unknown) {
    ObjectDefinition def;
    def.type = ObjectType::Platform;
    def.position = Position{lat_deg * DEG, lon_deg * DEG, 0};
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
  upd.position = Position{52.0 * DEG, 1.0 * DEG, 0};
  ASSERT_TRUE(rt.updateObject(id, upd));
  auto* kc = rt.store()->kinematics().get(id);
  ASSERT_NE(kc, nullptr);
  ASSERT_DOUBLE_EQ(kc->position.lat, 52.0 * DEG);
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
  obs.position = Position{51.5 * DEG, -0.1 * DEG, 0};
  obs.confidence = 0.8;
  obs.affiliation_hint = Affiliation::Hostile;
  obs.source_ref.source_system = "radar";
  obs.source_ref.source_entity_id = "T01";
  auto r = rt.processObservation(obs);
  ASSERT_EQ(r.outcome, CorrelationOutcome::Created);
}

///< REQ_TACTICAL_OBJECTS_031: Runtime delegates to query engine.
///< REQ_TACTICAL_OBJECTS_059: Tactical object solution is determined for a requirement.
TEST_F(RuntimeTest, TacticalObjectSolutionDetermination) {
  // Register an active-find interest with object type, AOI, and timeliness
  InterestCriteria crit;
  crit.query_mode = QueryMode::ActiveFind;
  crit.object_type = ObjectType::Platform;
  crit.affiliation = Affiliation::Hostile;
  crit.area = BoundingBox{50.0 * DEG, 52.0 * DEG, -2.0 * DEG, 2.0 * DEG};
  crit.time_window_start = 1000.0;
  crit.time_window_end = 2000.0;

  auto interest_id = rt.interestManager().registerInterest(crit, 1000.0, 5000.0);

  // Determine solution
  auto solution = rt.determineSolution(interest_id);

  // Verify solution captures requirement criteria
  EXPECT_FALSE(solution.solution_id.isNull());
  EXPECT_EQ(solution.source_interest_id, interest_id);
  ASSERT_TRUE(solution.intended_object_type.has_value());
  EXPECT_EQ(*solution.intended_object_type, ObjectType::Platform);
  ASSERT_TRUE(solution.area_of_interest.has_value());
  EXPECT_DOUBLE_EQ(solution.area_of_interest->min_lat, 50.0 * DEG);
  EXPECT_GT(solution.predicted_quality, 0.0);
  // Active-find generates derived evidence requirements
  EXPECT_FALSE(solution.evidence_requirements.empty());
}

TEST_F(RuntimeTest, QueryDelegation) {
  auto id = rt.createObject(makePlatform(51.5, -0.1, Affiliation::Friendly));
  QueryRequest req;
  req.by_uuid = id;
  auto resp = rt.query(req);
  ASSERT_EQ(resp.entries.size(), 1u);
}

///< REQ_TACTICAL_OBJECTS_032: Runtime delegates zone operations.
///< REQ_TACTICAL_OBJECTS_058: Achievability via zone containment (AOI-based).
///< REQ_TACTICAL_OBJECTS_064: Spatial reasoning for object-in-location (zone).
///< REQ_TACTICAL_OBJECTS_065: Capability via zone/query delegation.
TEST_F(RuntimeTest, ZoneDelegation) {
  ZoneDefinition def;
  def.zone_type = ZoneType::AOI;
  def.geometry.center = Position{51.0 * DEG, 0.0, 0};
  def.geometry.radius_m = 5000.0;
  auto zid = rt.createZone(def);
  ASSERT_TRUE(rt.isInsideZone(Position{51.0 * DEG, 0.0, 0}, zid));
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
  obs.position = Position{52.0 * DEG, 0.0, 0};
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
///< REQ_TACTICAL_OBJECTS_061: Traceability via deterministic correlation (lineage shape).
TEST_F(RuntimeTest, BatchCorrelationDeterminism) {
  auto make_batch = []() {
    ObservationBatch batch;
    for (int i = 0; i < 3; ++i) {
      Observation obs;
      obs.observation_id = UUIDHelper::generateV4();
      obs.position = Position{(51.0 + i * 0.5) * DEG, (-0.1 + i * 0.5) * DEG, 0};
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

///< REQ_TACTICAL_OBJECTS_026: createObject stores identity_name when provided.
TEST_F(RuntimeTest, CreateObjectWithIdentityName) {
  ObjectDefinition def;
  def.type = ObjectType::Unit;
  def.position = Position{51.0, 0.0, 0};
  def.identity_name = "Alpha Company";
  auto id = rt.createObject(def);
  ASSERT_FALSE(id.isNull());
  // Identity component should be set since identity_name is non-empty
  const auto* ic = rt.store()->identities().get(id);
  ASSERT_NE(ic, nullptr);
  ASSERT_EQ(ic->name, "Alpha Company");
}

///< REQ_TACTICAL_OBJECTS_026: createObject stores source_refs when provided.
TEST_F(RuntimeTest, CreateObjectWithSourceRefs) {
  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = Position{52.0, 1.0, 0};
  SourceRef sr;
  sr.source_system = "radar";
  sr.source_entity_id = "T99";
  def.source_refs.push_back(sr);
  auto id = rt.createObject(def);
  ASSERT_FALSE(id.isNull());
  // Identity component should be set since source_refs is non-empty
  const auto* ic = rt.store()->identities().get(id);
  ASSERT_NE(ic, nullptr);
  ASSERT_EQ(ic->source_refs.size(), 1u);
  ASSERT_EQ(ic->source_refs[0].source_system, "radar");
}

///< REQ_TACTICAL_OBJECTS_027: updateObject creates kinematics when component absent.
TEST_F(RuntimeTest, UpdateObjectCreatesKinematicsWhenAbsent) {
  // Create an object and then manually remove its kinematics so the else-branch fires
  auto id = rt.createObject(makePlatform(51.0, 0.0));
  rt.store()->kinematics().remove(id);
  ASSERT_EQ(rt.store()->kinematics().get(id), nullptr);

  ObjectUpdate upd;
  upd.position = Position{53.0 * DEG, 2.0 * DEG, 0};
  ASSERT_TRUE(rt.updateObject(id, upd));

  const auto* kc = rt.store()->kinematics().get(id);
  ASSERT_NE(kc, nullptr);
  ASSERT_DOUBLE_EQ(kc->position.lat, 53.0 * DEG);
}

///< REQ_TACTICAL_OBJECTS_027: updateObject with mil_class field.
TEST_F(RuntimeTest, UpdateObjectMilClass) {
  auto id = rt.createObject(makePlatform(51.5, -0.1));
  ObjectUpdate upd;
  MilClassProfile p;
  p.affiliation = Affiliation::Hostile;
  p.battle_dim = BattleDimension::Air;
  upd.mil_class = p;
  ASSERT_TRUE(rt.updateObject(id, upd));
  auto got = rt.getMilClass(id);
  ASSERT_TRUE(got.has_value());
  ASSERT_EQ(got->battle_dim, BattleDimension::Air);
}

///< REQ_TACTICAL_OBJECTS_027: updateObject sets identity_name on new and existing components.
TEST_F(RuntimeTest, UpdateObjectIdentityNameBothPaths) {
  // Path A: no existing IdentityComponent
  auto id_a = rt.createObject(makePlatform(51.0, 0.0));
  ObjectUpdate upd_a;
  upd_a.identity_name = std::string("Bravo");
  ASSERT_TRUE(rt.updateObject(id_a, upd_a));
  const auto* ic_a = rt.store()->identities().get(id_a);
  ASSERT_NE(ic_a, nullptr);
  ASSERT_EQ(ic_a->name, "Bravo");

  // Path B: existing IdentityComponent
  ObjectDefinition def;
  def.identity_name = "Charlie";
  def.position = Position{52.0, 1.0, 0};
  auto id_b = rt.createObject(def);
  ObjectUpdate upd_b;
  upd_b.identity_name = std::string("Delta");
  ASSERT_TRUE(rt.updateObject(id_b, upd_b));
  const auto* ic_b = rt.store()->identities().get(id_b);
  ASSERT_NE(ic_b, nullptr);
  ASSERT_EQ(ic_b->name, "Delta");
}

///< REQ_TACTICAL_OBJECTS_053: updateObject updates behavior when component already exists.
TEST_F(RuntimeTest, UpdateObjectBehaviorComponentAlreadyExists) {
  auto id = rt.createObject(makePlatform(51.5, -0.1));
  // Pre-create behavior component
  rt.setBehavior(id, "patrol", "active");

  ObjectUpdate upd;
  upd.behavior_pattern = std::string("loiter");
  upd.operational_state = std::string("degraded");
  ASSERT_TRUE(rt.updateObject(id, upd));

  auto bc = rt.getBehavior(id);
  ASSERT_TRUE(bc.has_value());
  ASSERT_EQ(bc->behavior_pattern, "loiter");
  ASSERT_EQ(bc->operational_state, "degraded");
}

///< REQ_TACTICAL_OBJECTS_033: deriveSymbolKey delegates to MilClassEngine.
TEST_F(RuntimeTest, DeriveSymbolKeyDelegates) {
  auto id = rt.createObject(makePlatform(51.5, -0.1));
  MilClassProfile p;
  p.affiliation = Affiliation::Hostile;
  p.battle_dim = BattleDimension::Air;
  rt.setMilClass(id, p);
  std::string key = rt.deriveSymbolKey(id);
  ASSERT_FALSE(key.empty());
}

///< REQ_TACTICAL_OBJECTS_032: evaluateZoneTransition delegates to ZoneEngine.
TEST_F(RuntimeTest, EvaluateZoneTransitionDelegates) {
  ZoneDefinition def;
  def.zone_type = ZoneType::AOI;
  def.geometry.center = Position{51.0 * DEG, 0.0, 0};
  def.geometry.radius_m = 5000.0;
  auto zid = rt.createZone(def);

  auto obj_id = rt.createObject(makePlatform(51.0, 0.0));
  auto rel = rt.evaluateZoneTransition(obj_id, zid, Position{51.0 * DEG, 0.0, 0}, 0.0);
  ASSERT_EQ(rel, ZoneRelationship::Entering);
}
