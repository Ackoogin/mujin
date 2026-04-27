#include <gtest/gtest.h>
#include <correlation/CorrelationEngine.h>
#include <uuid/UUIDHelper.h>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

static constexpr double DEG = 3.14159265358979323846 / 180.0;

class CorrelationEngineTest : public ::testing::Test {
protected:
  std::shared_ptr<ObjectStore> store = std::make_shared<ObjectStore>();
  std::shared_ptr<SpatialIndex> spatial = std::make_shared<SpatialIndex>(DEG);
  std::shared_ptr<MilClassEngine> milclass = std::make_shared<MilClassEngine>(store);

  Observation makeObs(double lat, double lon, Affiliation aff = Affiliation::Unknown,
                      double conf = 0.5, const std::string& src_sys = "sensor-1",
                      const std::string& src_eid = "track-A") {
    Observation obs;
    obs.observation_id = UUIDHelper::generateV4();
    obs.observed_at = 1000.0;
    obs.received_at = 1001.0;
    obs.position.lat = lat;
    obs.position.lon = lon;
    obs.velocity = Velocity{0, 0, 0};
    obs.confidence = conf;
    obs.affiliation_hint = aff;
    obs.source_ref.source_system = src_sys;
    obs.source_ref.source_entity_id = src_eid;
    obs.source_ref.last_seen = obs.observed_at;
    return obs;
  }
};

///< REQ_TACTICAL_OBJECTS_005: First observation creates a new entity.
TEST_F(CorrelationEngineTest, FirstObservationCreatesEntity) {
  CorrelationEngine engine(store, spatial, milclass);
  auto obs = makeObs(51.5 * DEG, -0.1 * DEG, Affiliation::Hostile, 0.8);
  auto result = engine.processObservation(obs);
  ASSERT_EQ(result.outcome, CorrelationOutcome::Created);
  ASSERT_FALSE(result.object_id.isNull());
  ASSERT_GE(store->objectCount(), 1u);
}

///< REQ_TACTICAL_OBJECTS_006: Correlated observation merges into existing entity.
TEST_F(CorrelationEngineTest, CorrelatedObservationMerges) {
  CorrelationConfig config;
  config.gate_radius_rad = 0.5 * DEG;
  config.merge_threshold = 0.3;
  CorrelationEngine engine(store, spatial, milclass, config);

  auto obs1 = makeObs(51.5 * DEG, -0.1 * DEG, Affiliation::Hostile, 0.8, "radar", "T001");
  auto r1 = engine.processObservation(obs1);

  auto obs2 = makeObs(51.5001 * DEG, -0.1001 * DEG, Affiliation::Hostile, 0.7, "radar", "T001");
  obs2.observed_at = 1010.0;
  auto r2 = engine.processObservation(obs2);

  ASSERT_EQ(r2.outcome, CorrelationOutcome::Merged);
  ASSERT_EQ(r2.object_id, r1.object_id);
}

///< REQ_TACTICAL_OBJECTS_007: Distant observation creates a separate entity.
TEST_F(CorrelationEngineTest, DistantObservationNotCorrelated) {
  CorrelationEngine engine(store, spatial, milclass);

  auto obs1 = makeObs(10.0 * DEG, 20.0 * DEG, Affiliation::Neutral);
  auto r1 = engine.processObservation(obs1);

  auto obs2 = makeObs(60.0 * DEG, 90.0 * DEG, Affiliation::Neutral);
  auto r2 = engine.processObservation(obs2);

  ASSERT_EQ(r2.outcome, CorrelationOutcome::Created);
  ASSERT_NE(r2.object_id, r1.object_id);
}

///< REQ_TACTICAL_OBJECTS_008: Repeated incompatibility triggers a split.
TEST_F(CorrelationEngineTest, IncompatibleAffiliationSplits) {
  CorrelationConfig config;
  config.gate_radius_rad = 0.5 * DEG;
  config.merge_threshold = 0.3;
  config.split_incompatibility_count = 2;
  CorrelationEngine engine(store, spatial, milclass, config);

  auto obs1 = makeObs(51.5 * DEG, -0.1 * DEG, Affiliation::Friendly, 0.9, "radar", "T01");
  auto r1 = engine.processObservation(obs1);
  ASSERT_EQ(r1.outcome, CorrelationOutcome::Created);

  auto obs2 = makeObs(51.5001 * DEG, -0.1 * DEG, Affiliation::Hostile, 0.8, "radar", "T01");
  engine.processObservation(obs2);
  auto obs3 = makeObs(51.5002 * DEG, -0.1 * DEG, Affiliation::Hostile, 0.8, "radar", "T01");
  auto r3 = engine.processObservation(obs3);

  ASSERT_EQ(r3.outcome, CorrelationOutcome::Split);
  ASSERT_NE(r3.object_id, r1.object_id);
}

///< REQ_TACTICAL_OBJECTS_009: Same input produces same score (deterministic).
TEST_F(CorrelationEngineTest, DeterministicScore) {
  CorrelationConfig config;
  config.gate_radius_rad = 0.5 * DEG;
  config.merge_threshold = 0.3;

  auto run = [&]() {
    auto s = std::make_shared<ObjectStore>();
    auto sp = std::make_shared<SpatialIndex>(DEG);
    auto mc = std::make_shared<MilClassEngine>(s);
    CorrelationEngine eng(s, sp, mc, config);
    auto obs1 = makeObs(51.5 * DEG, -0.1 * DEG, Affiliation::Hostile, 0.8, "radar", "T01");
    eng.processObservation(obs1);
    auto obs2 = makeObs(51.5001 * DEG, -0.1001 * DEG, Affiliation::Hostile, 0.7, "radar", "T01");
    return eng.processObservation(obs2);
  };
  auto r1 = run();
  auto r2 = run();
  ASSERT_EQ(r1.outcome, r2.outcome);
}

///< REQ_TACTICAL_OBJECTS_008: Only local candidates are scored (spatial prefilter).
TEST_F(CorrelationEngineTest, SpatialPrefilterLimitsScoring) {
  CorrelationConfig config;
  config.gate_radius_rad = 0.5 * DEG;
  config.merge_threshold = 0.3;
  CorrelationEngine engine(store, spatial, milclass, config);

  for (int i = 0; i < 20; ++i) {
    auto obs = makeObs(static_cast<double>(i) * 5.0 * DEG, static_cast<double>(i) * 10.0 * DEG,
                       Affiliation::Neutral, 0.5, "src-" + std::to_string(i), "T" + std::to_string(i));
    engine.processObservation(obs);
  }
  ASSERT_GE(store->objectCount(), 20u);
  auto local_obs = makeObs(0.001 * DEG, 0.001 * DEG, Affiliation::Neutral, 0.5, "local", "TL");
  auto r = engine.processObservation(local_obs);
  ASSERT_EQ(r.outcome, CorrelationOutcome::Merged);
}

///< REQ_TACTICAL_OBJECTS_006: Correlated object retains all contributing source refs.
TEST_F(CorrelationEngineTest, SourceRefsRetained) {
  CorrelationConfig config;
  config.gate_radius_rad = 0.5 * DEG;
  config.merge_threshold = 0.3;
  CorrelationEngine engine(store, spatial, milclass, config);

  auto obs1 = makeObs(51.5 * DEG, -0.1 * DEG, Affiliation::Hostile, 0.8, "radar", "T01");
  auto r1 = engine.processObservation(obs1);
  auto obs2 = makeObs(51.5001 * DEG, -0.1001 * DEG, Affiliation::Hostile, 0.7, "ais", "T02");
  engine.processObservation(obs2);
  auto obs3 = makeObs(51.5002 * DEG, -0.1002 * DEG, Affiliation::Hostile, 0.75, "elint", "T03");
  engine.processObservation(obs3);

  const auto* cc = store->correlation().get(r1.object_id);
  ASSERT_NE(cc, nullptr);
  ASSERT_EQ(cc->source_refs.size(), 3u);
}

///< REQ_TACTICAL_OBJECTS_013: Aggregate confidence from weighted sources.
TEST_F(CorrelationEngineTest, ConfidenceAggregation) {
  CorrelationConfig config;
  config.gate_radius_rad = 0.5 * DEG;
  config.merge_threshold = 0.3;
  CorrelationEngine engine(store, spatial, milclass, config);

  auto obs1 = makeObs(51.5 * DEG, -0.1 * DEG, Affiliation::Hostile, 0.9, "radar", "T01");
  auto r1 = engine.processObservation(obs1);
  auto obs2 = makeObs(51.5001 * DEG, -0.1001 * DEG, Affiliation::Hostile, 0.3, "ais", "T01");
  engine.processObservation(obs2);

  const auto* cc = store->correlation().get(r1.object_id);
  ASSERT_NE(cc, nullptr);
  ASSERT_GT(cc->confidence, 0.3);
  ASSERT_LT(cc->confidence, 0.9);
}

///< REQ_TACTICAL_OBJECTS_009: Contributing observations tracked in CorrelationComponent.
TEST_F(CorrelationEngineTest, ContributingObservationsTracked) {
  CorrelationConfig config;
  config.gate_radius_rad = 0.5 * DEG;
  config.merge_threshold = 0.3;
  CorrelationEngine engine(store, spatial, milclass, config);

  auto obs1 = makeObs(51.5 * DEG, -0.1 * DEG, Affiliation::Hostile, 0.9, "radar", "T01");
  auto r1 = engine.processObservation(obs1);

  auto obs2 = makeObs(51.5001 * DEG, -0.1001 * DEG, Affiliation::Hostile, 0.8, "radar", "T01");
  engine.processObservation(obs2);

  const auto* fc = store->correlation().get(r1.object_id);
  ASSERT_NE(fc, nullptr);
  ASSERT_GE(fc->contributing_observations.size(), 2u);
}

// ---------------------------------------------------------------------------
// SIDC battle-dimension parsing (lines 90-91, 93-96 in CorrelationEngine.cpp)
// ---------------------------------------------------------------------------

///< Coverage: SIDC position 2 == 'P' -> BattleDimension::Space
TEST_F(CorrelationEngineTest, SIDCBattleDimension_Space) {
  CorrelationEngine engine(store, spatial, milclass);
  auto obs = makeObs(10.0 * DEG, 10.0 * DEG);
  obs.source_sidc = "SFP";  // position 2 = 'P'
  auto result = engine.processObservation(obs);
  ASSERT_FALSE(result.object_id.isNull());
  const auto* mc = store->milclass().get(result.object_id);
  ASSERT_NE(mc, nullptr);
  EXPECT_EQ(mc->profile.battle_dim, BattleDimension::Space);
}

///< Coverage: SIDC position 2 == 'A' -> BattleDimension::Air
TEST_F(CorrelationEngineTest, SIDCBattleDimension_Air) {
  CorrelationEngine engine(store, spatial, milclass);
  auto obs = makeObs(11.0 * DEG, 10.0 * DEG);
  obs.source_sidc = "SFA";
  auto result = engine.processObservation(obs);
  ASSERT_FALSE(result.object_id.isNull());
  const auto* mc = store->milclass().get(result.object_id);
  ASSERT_NE(mc, nullptr);
  EXPECT_EQ(mc->profile.battle_dim, BattleDimension::Air);
}

///< Coverage: SIDC position 2 == 'S' -> BattleDimension::SeaSurface
TEST_F(CorrelationEngineTest, SIDCBattleDimension_SeaSurface) {
  CorrelationEngine engine(store, spatial, milclass);
  auto obs = makeObs(12.0 * DEG, 10.0 * DEG);
  obs.source_sidc = "SFS";
  auto result = engine.processObservation(obs);
  ASSERT_FALSE(result.object_id.isNull());
  const auto* mc = store->milclass().get(result.object_id);
  ASSERT_NE(mc, nullptr);
  EXPECT_EQ(mc->profile.battle_dim, BattleDimension::SeaSurface);
}

///< Coverage: SIDC position 2 == 'U' -> BattleDimension::Subsurface
TEST_F(CorrelationEngineTest, SIDCBattleDimension_Subsurface) {
  CorrelationEngine engine(store, spatial, milclass);
  auto obs = makeObs(13.0 * DEG, 10.0 * DEG);
  obs.source_sidc = "SFU";
  auto result = engine.processObservation(obs);
  ASSERT_FALSE(result.object_id.isNull());
  const auto* mc = store->milclass().get(result.object_id);
  ASSERT_NE(mc, nullptr);
  EXPECT_EQ(mc->profile.battle_dim, BattleDimension::Subsurface);
}

///< Coverage: SIDC position 2 == 'F' -> BattleDimension::SOF
TEST_F(CorrelationEngineTest, SIDCBattleDimension_SOF) {
  CorrelationEngine engine(store, spatial, milclass);
  auto obs = makeObs(14.0 * DEG, 10.0 * DEG);
  obs.source_sidc = "SFF";
  auto result = engine.processObservation(obs);
  ASSERT_FALSE(result.object_id.isNull());
  const auto* mc = store->milclass().get(result.object_id);
  ASSERT_NE(mc, nullptr);
  EXPECT_EQ(mc->profile.battle_dim, BattleDimension::SOF);
}

///< Coverage: SIDC position 2 is unrecognized character -> default: break (line 96)
TEST_F(CorrelationEngineTest, SIDCBattleDimension_UnknownChar) {
  CorrelationEngine engine(store, spatial, milclass);
  auto obs = makeObs(15.0 * DEG, 10.0 * DEG);
  obs.source_sidc = "SFX";  // position 2 = 'X' -> hits default: break
  auto result = engine.processObservation(obs);
  ASSERT_FALSE(result.object_id.isNull());
  // No specific battle_dim assertion: default value remains
}
