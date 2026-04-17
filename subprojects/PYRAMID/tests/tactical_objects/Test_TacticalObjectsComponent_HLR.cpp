/// \file Test_TacticalObjectsComponent_HLR.cpp
/// \brief PCL component tests with explicit HLR (TOBJ/PYR-RESP) traceability.
/// All assertions are made exclusively through data ports (services and
/// subscribers) — no direct access to component internals or runtime.
#include <gtest/gtest.h>
#include <TacticalObjectsComponent.h>
#include <TacticalObjectsCodec.h>
#include <pcl/executor.hpp>
#include <pcl/pcl_executor.h>
#include <uuid/UUIDHelper.h>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

static constexpr double DEG = 3.14159265358979323846 / 180.0;

// ---------------------------------------------------------------------------
// Port helpers
// ---------------------------------------------------------------------------

static pcl_msg_t makeMsg(const std::string& str) {
  pcl_msg_t m = {};
  m.data = str.data();
  m.size = static_cast<uint32_t>(str.size());
  m.type_name = "application/json";
  return m;
}

static void dispatchObservation(pcl::Executor& exec, const std::string& payload) {
  pcl_msg_t msg = makeMsg(payload);
  exec.dispatchIncoming("observation_ingress", &msg);
}

static void invokeService(pcl_executor_t* e, const char* svc,
                          const std::string& req_str,
                          char* resp_buf, size_t resp_len, pcl_msg_t* resp) {
  pcl_msg_t req = makeMsg(req_str);
  resp->data = resp_buf;
  resp->size = static_cast<uint32_t>(resp_len);
  ASSERT_EQ(pcl_executor_invoke_service(e, svc, &req, resp), PCL_OK);
}

/// Helper: invoke create_object and return the object_id string.
static std::string createObjectViaPort(pcl_executor_t* e,
                                       const ObjectDefinition& def,
                                       char* buf, size_t buf_len) {
  auto j = TacticalObjectsCodec::encodeObjectDefinition(def);
  pcl_msg_t resp = {};
  invokeService(e, "create_object", j.dump(), buf, buf_len, &resp);
  return nlohmann::json::parse(
    std::string(static_cast<const char*>(resp.data), resp.size))
    .value("object_id", "");
}

/// Helper: invoke query and return parsed JSON response.
static nlohmann::json queryViaPort(pcl_executor_t* e,
                                   const QueryRequest& req,
                                   char* buf, size_t buf_len) {
  auto j_req = TacticalObjectsCodec::encodeQueryRequest(req);
  pcl_msg_t resp = {};
  invokeService(e, "query", j_req.dump(), buf, buf_len, &resp);
  return nlohmann::json::parse(
    std::string(static_cast<const char*>(resp.data), resp.size));
}

/// Helper: invoke get_object and return parsed JSON response.
static nlohmann::json getObjectViaPort(pcl_executor_t* e,
                                       const std::string& id_str,
                                       char* buf, size_t buf_len) {
  nlohmann::json req;
  req["object_id"] = id_str;
  pcl_msg_t resp = {};
  invokeService(e, "get_object", req.dump(), buf, buf_len, &resp);
  return nlohmann::json::parse(
    std::string(static_cast<const char*>(resp.data), resp.size));
}

// ---------------------------------------------------------------------------
// HLR Tests — data-port only
// ---------------------------------------------------------------------------

///< TOBJ.004: External ID association. TOBJ.020: Evidence lineage.
///< TOBJ.022: Entity integration. PYR-RESP-0735: Object confidence. PYR-RESP-0737: Relationships.
///< Two observations from the same source merge; source_refs and lineage are
///< verified entirely through the get_object service port.
TEST(TacticalObjectsComponentHLR, ExternalIdAndLineageFromCorrelation) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  ObservationBatch batch1;
  Observation obs1;
  obs1.observation_id = UUIDHelper::generateV4();
  obs1.position = Position{51.5 * DEG, -0.1 * DEG, 0};
  obs1.confidence = 0.9;
  obs1.affiliation_hint = Affiliation::Hostile;
  obs1.source_ref.source_system = "radar";
  obs1.source_ref.source_entity_id = "T-001";
  batch1.observations.push_back(obs1);
  dispatchObservation(exec, TacticalObjectsCodec::encodeObservationBatch(batch1).dump());

  ObservationBatch batch2;
  Observation obs2;
  obs2.observation_id = UUIDHelper::generateV4();
  obs2.position = Position{51.5001 * DEG, -0.0999 * DEG, 0};
  obs2.confidence = 0.85;
  obs2.affiliation_hint = Affiliation::Hostile;
  obs2.source_ref.source_system = "radar";
  obs2.source_ref.source_entity_id = "T-001";
  batch2.observations.push_back(obs2);
  dispatchObservation(exec, TacticalObjectsCodec::encodeObservationBatch(batch2).dump());

  // Retrieve all objects via the query service port.
  char qbuf[8192];
  auto jq = queryViaPort(exec.handle(), QueryRequest{}, qbuf, sizeof(qbuf));
  ASSERT_GE(jq["entries"].size(), 1u);

  // Find the correlated entity via the get_object service port.
  bool found_correlated = false;
  char gbuf[4096];
  for (auto& entry : jq["entries"]) {
    auto gdata = getObjectViaPort(exec.handle(),
                                  entry["id"].get<std::string>(),
                                  gbuf, sizeof(gbuf));
    if (gdata.value("has_correlation", false)) {
      EXPECT_GE(gdata.value("source_refs_count", 0), 1);
      EXPECT_GE(gdata.value("contributing_observations_count", 0), 1);
      found_correlated = true;
      break;
    }
  }
  ASSERT_TRUE(found_correlated);
}

///< TOBJ.009: Region query. TOBJ.036: Zone relationship queries. TOBJ.040: Spatial indexing.
///< PYR-RESP-0734: Determine potential objects at locations.
TEST(TacticalObjectsComponentHLR, RegionQueryViaService) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  char cbuf[1024];

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = Position{51.5 * DEG, 0.0, 0};
  def.affiliation = Affiliation::Friendly;
  std::string id1_str = createObjectViaPort(exec.handle(), def, cbuf, sizeof(cbuf));
  ASSERT_FALSE(id1_str.empty());

  def.position = Position{52.5 * DEG, 1.0 * DEG, 0};
  std::string id2_str = createObjectViaPort(exec.handle(), def, cbuf, sizeof(cbuf));
  ASSERT_FALSE(id2_str.empty());

  QueryRequest qreq;
  qreq.by_region = BoundingBox{51.0 * DEG, 52.0 * DEG, -0.5 * DEG, 0.5 * DEG};
  char qbuf[4096];
  auto jr = queryViaPort(exec.handle(), qreq, qbuf, sizeof(qbuf));
  ASSERT_GE(jr["entries"].size(), 1u);

  bool found_in_region = false;
  for (auto& e : jr["entries"]) {
    if (e["id"].get<std::string>() == id1_str) {
      found_in_region = true;
      break;
    }
  }
  ASSERT_TRUE(found_in_region);
}

///< TOBJ.010: Temporal query. TOBJ.017: State freshness.
///< PYR-RESP-0734: Query with freshness filter.
///< An observation with observed_at=100.0 sets freshness; query with
///< max_age=200 / current_time=150 (age=50) must return the entity.
TEST(TacticalObjectsComponentHLR, TemporalQueryViaService) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  // Ingest via subscriber — sets freshness_timestamp = observed_at.
  ObservationBatch batch;
  Observation obs;
  obs.observation_id = UUIDHelper::generateV4();
  obs.position = Position{51.5 * DEG, 0.0, 0};
  obs.observed_at = 100.0;
  obs.confidence = 0.8;
  batch.observations.push_back(obs);
  dispatchObservation(exec, TacticalObjectsCodec::encodeObservationBatch(batch).dump());

  QueryRequest qreq;
  qreq.max_age_seconds = 200.0;
  qreq.current_time = 150.0;  // age = 50 < 200 → entity must match
  char qbuf[4096];
  auto jr = queryViaPort(exec.handle(), qreq, qbuf, sizeof(qbuf));
  ASSERT_GE(jr["entries"].size(), 1u);
}

///< TOBJ.015: Behavior estimation. TOBJ.016: Operational state.
///< PYR-RESP-0738: Estimate object behaviour.
TEST(TacticalObjectsComponentHLR, BehaviorAndOperationalStateViaUpdate) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = Position{51.5 * DEG, 0.0, 0};
  char cbuf[1024];
  std::string id_str = createObjectViaPort(exec.handle(), def, cbuf, sizeof(cbuf));
  ASSERT_FALSE(id_str.empty());

  nlohmann::json j_upd;
  j_upd["object_id"] = id_str;
  j_upd["behavior_pattern"] = "loitering";
  j_upd["operational_state"] = "airborne";
  char ubuf[256];
  pcl_msg_t uresp = {};
  invokeService(exec.handle(), "update_object", j_upd.dump(), ubuf, sizeof(ubuf), &uresp);

  // Verify behavior through the get_object service port.
  char gbuf[2048];
  auto gdata = getObjectViaPort(exec.handle(), id_str, gbuf, sizeof(gbuf));
  ASSERT_TRUE(gdata.value("has_behavior", false));
  EXPECT_EQ(gdata["behavior"].value("behavior_pattern", ""), "loitering");
  EXPECT_EQ(gdata["behavior"].value("operational_state", ""), "airborne");
}

///< TOBJ.019: Confidence tracking. PYR-RESP-0735: Determine object information confidence.
TEST(TacticalObjectsComponentHLR, ConfidenceFromCorrelation) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  ObservationBatch batch;
  Observation obs;
  obs.observation_id = UUIDHelper::generateV4();
  obs.position = Position{51.5 * DEG, -0.1 * DEG, 0};
  obs.confidence = 0.75;
  obs.affiliation_hint = Affiliation::Neutral;
  obs.source_ref.source_system = "sensor";
  obs.source_ref.source_entity_id = "O1";
  batch.observations.push_back(obs);
  dispatchObservation(exec, TacticalObjectsCodec::encodeObservationBatch(batch).dump());

  // Retrieve all objects via the query service port.
  char qbuf[4096];
  auto jq = queryViaPort(exec.handle(), QueryRequest{}, qbuf, sizeof(qbuf));
  ASSERT_GE(jq["entries"].size(), 1u);

  // Find the correlated entity with a confidence value via get_object.
  bool found_confidence = false;
  char gbuf[2048];
  for (auto& entry : jq["entries"]) {
    auto gdata = getObjectViaPort(exec.handle(),
                                  entry["id"].get<std::string>(),
                                  gbuf, sizeof(gbuf));
    if (gdata.contains("confidence")) {
      EXPECT_DOUBLE_EQ(gdata.value("confidence", 0.0), 0.75);
      found_confidence = true;
      break;
    }
  }
  ASSERT_TRUE(found_confidence);
}

///< TOBJ.028: Status. TOBJ.029: Echelon. TOBJ.030: Indicator flags. TOBJ.031: Mobility.
///< PYR-RESP-0741: Capture classification details.
TEST(TacticalObjectsComponentHLR, FullMilitaryClassificationViaCreate) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  ObjectDefinition def;
  def.type = ObjectType::Unit;
  def.position = Position{51.5 * DEG, 0.0, 0};
  def.affiliation = Affiliation::Friendly;
  def.mil_class.battle_dim = BattleDimension::Ground;
  def.mil_class.affiliation = Affiliation::Friendly;
  def.mil_class.role = "armor";
  def.mil_class.hq = true;
  def.mil_class.task_force = true;

  char cbuf[1024];
  std::string id_str = createObjectViaPort(exec.handle(), def, cbuf, sizeof(cbuf));
  ASSERT_FALSE(id_str.empty());

  // Verify classification through the get_object service port.
  char gbuf[2048];
  auto gdata = getObjectViaPort(exec.handle(), id_str, gbuf, sizeof(gbuf));
  ASSERT_TRUE(gdata.value("found", false));
  ASSERT_TRUE(gdata.contains("mil_class"));
  EXPECT_EQ(gdata["mil_class"].value("battle_dim", ""), "Ground");
  EXPECT_EQ(gdata["mil_class"].value("affiliation", ""), "Friendly");
  EXPECT_EQ(gdata["mil_class"].value("role", ""), "armor");
  EXPECT_TRUE(gdata["mil_class"].value("hq", false));
  EXPECT_TRUE(gdata["mil_class"].value("task_force", false));
}

///< TOBJ.032: Source symbol code preservation.
///< PYR-RESP-0741: Capture SIDC when provided.
TEST(TacticalObjectsComponentHLR, SourceSidcPreserved) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  ObservationBatch batch;
  Observation obs;
  obs.observation_id = UUIDHelper::generateV4();
  obs.position = Position{51.5 * DEG, -0.1 * DEG, 0};
  obs.confidence = 0.8;
  obs.source_sidc = "SFGPUCIZ----";
  obs.source_ref.source_system = "tracks";
  obs.source_ref.source_entity_id = "S1";
  batch.observations.push_back(obs);
  dispatchObservation(exec, TacticalObjectsCodec::encodeObservationBatch(batch).dump());

  // Retrieve all objects via the query service port.
  char qbuf[4096];
  auto jq = queryViaPort(exec.handle(), QueryRequest{}, qbuf, sizeof(qbuf));
  ASSERT_GE(jq["entries"].size(), 1u);

  // Find the entity carrying the source SIDC via get_object.
  bool found_sidc = false;
  char gbuf[2048];
  for (auto& entry : jq["entries"]) {
    auto gdata = getObjectViaPort(exec.handle(),
                                  entry["id"].get<std::string>(),
                                  gbuf, sizeof(gbuf));
    if (gdata.contains("mil_class") &&
        !gdata["mil_class"].value("source_sidc", "").empty()) {
      EXPECT_EQ(gdata["mil_class"].value("source_sidc", ""), "SFGPUCIZ----");
      found_sidc = true;
      break;
    }
  }
  ASSERT_TRUE(found_sidc);
}

///< TOBJ.039: Sparse component storage.
///< Entity with minimal fields does not populate unused components;
///< verified through the get_object service port presence flags.
TEST(TacticalObjectsComponentHLR, SparseObjectCreation) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  ObjectDefinition def;
  def.type = ObjectType::Point;
  char cbuf[1024];
  std::string id_str = createObjectViaPort(exec.handle(), def, cbuf, sizeof(cbuf));
  ASSERT_FALSE(id_str.empty());

  char gbuf[2048];
  auto gdata = getObjectViaPort(exec.handle(), id_str, gbuf, sizeof(gbuf));
  ASSERT_TRUE(gdata.value("found", false));
  EXPECT_FALSE(gdata.value("has_identity", true));
  EXPECT_FALSE(gdata.value("has_correlation", true));
  EXPECT_FALSE(gdata.value("has_behavior", true));
}

///< TOBJ.001: All object types supported.
TEST(TacticalObjectsComponentHLR, AllObjectTypesViaCreate) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  std::vector<ObjectType> types = {
    ObjectType::Platform, ObjectType::Person, ObjectType::Equipment,
    ObjectType::Unit, ObjectType::Formation, ObjectType::Installation,
    ObjectType::Feature, ObjectType::Route, ObjectType::Point,
    ObjectType::Area, ObjectType::Zone
  };

  char cbuf[1024];
  for (ObjectType t : types) {
    ObjectDefinition def;
    def.type = t;
    def.position = Position{51.0 * DEG, 0.0, 0};
    std::string id_str = createObjectViaPort(exec.handle(), def, cbuf, sizeof(cbuf));
    ASSERT_FALSE(id_str.empty());
  }

  char qbuf[8192];
  auto jq = queryViaPort(exec.handle(), QueryRequest{}, qbuf, sizeof(qbuf));
  ASSERT_GE(jq["entries"].size(), static_cast<size_t>(types.size()));
}

///< TOBJ.008: Query by external source reference.
///< PYR-RESP-0734: Matching_Objects by source identifier.
TEST(TacticalObjectsComponentHLR, QueryBySourceRef) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  ObservationBatch batch;
  Observation obs;
  obs.observation_id = UUIDHelper::generateV4();
  obs.position = Position{51.5 * DEG, -0.1 * DEG, 0};
  obs.confidence = 0.8;
  obs.source_ref.source_system = "tracker-a";
  obs.source_ref.source_entity_id = "ext-42";
  batch.observations.push_back(obs);
  dispatchObservation(exec, TacticalObjectsCodec::encodeObservationBatch(batch).dump());

  QueryRequest qreq;
  qreq.by_source_system = "tracker-a";
  qreq.by_source_entity_id = "ext-42";
  char qbuf[4096];
  auto jr = queryViaPort(exec.handle(), qreq, qbuf, sizeof(qbuf));
  ASSERT_GE(jr["entries"].size(), 1u);
}
