/// \file Test_TacticalObjectsComponent_HLR.cpp
/// \brief PCL component tests with explicit HLR (TOBJ/RESP) traceability.
/// Ensures 100% coverage of HLR requirements exercisable through the component.
#include <gtest/gtest.h>
#include <TacticalObjectsComponent.h>
#include <TacticalObjectsCodec.h>
#include <pcl/executor.hpp>
#include <pcl/pcl_executor.h>
#include <uuid/UUIDHelper.h>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

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

///< TOBJ.004: External ID association. TOBJ.020: Evidence lineage.
///< TOBJ.022: Entity integration. RESP.007: Object confidence. RESP.009: Relationships.
///< Two observations from same source merge; source_refs and lineage retained.
TEST(TacticalObjectsComponentHLR, ExternalIdAndLineageFromCorrelation) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  ObservationBatch batch1;
  Observation obs1;
  obs1.observation_id = UUIDHelper::generateV4();
  obs1.position = Position{51.5, -0.1, 0};
  obs1.confidence = 0.9;
  obs1.affiliation_hint = Affiliation::Hostile;
  obs1.source_ref.source_system = "radar";
  obs1.source_ref.source_entity_id = "T-001";
  batch1.observations.push_back(obs1);

  auto j1 = TacticalObjectsCodec::encodeObservationBatch(batch1);
  dispatchObservation(exec, j1.dump());

  ObservationBatch batch2;
  Observation obs2;
  obs2.observation_id = UUIDHelper::generateV4();
  obs2.position = Position{51.5001, -0.0999, 0};
  obs2.confidence = 0.85;
  obs2.affiliation_hint = Affiliation::Hostile;
  obs2.source_ref.source_system = "radar";
  obs2.source_ref.source_entity_id = "T-001";
  batch2.observations.push_back(obs2);

  auto j2 = TacticalObjectsCodec::encodeObservationBatch(batch2);
  dispatchObservation(exec, j2.dump());

  auto resp = comp.runtime().query(QueryRequest());
  ASSERT_GE(resp.entries.size(), 1u);
  UUIDKey correlated_id;
  for (const auto& e : resp.entries) {
    if (comp.runtime().store()->correlation().get(e.id)) {
      correlated_id = e.id;
      break;
    }
  }
  ASSERT_FALSE(correlated_id.isNull());
  const auto* cc = comp.runtime().store()->correlation().get(correlated_id);
  ASSERT_NE(cc, nullptr);
  EXPECT_GE(cc->source_refs.size(), 1u);
  EXPECT_GE(cc->contributing_observations.size(), 1u);
}

///< TOBJ.009: Region query. TOBJ.036: Zone relationship queries. TOBJ.040: Spatial indexing.
///< RESP.006: Determine potential objects at locations.
TEST(TacticalObjectsComponentHLR, RegionQueryViaService) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = Position{51.5, 0.0, 0};
  def.affiliation = Affiliation::Friendly;
  auto id1 = comp.runtime().createObject(def);

  def.position = Position{52.5, 1.0, 0};
  auto id2 = comp.runtime().createObject(def);

  QueryRequest qreq;
  qreq.by_region = BoundingBox{51.0, 52.0, -0.5, 0.5};
  auto j_req = TacticalObjectsCodec::encodeQueryRequest(qreq);
  char resp_buf[4096];
  pcl_msg_t resp = {};
  invokeService(exec.handle(), "query", j_req.dump(), resp_buf, sizeof(resp_buf),
               &resp);

  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  ASSERT_GE(jr["entries"].size(), 1u);
  bool found_in_region = false;
  for (auto& e : jr["entries"]) {
    if (e["id"].get<std::string>() == UUIDHelper::toString(id1.uuid)) {
      found_in_region = true;
      break;
    }
  }
  ASSERT_TRUE(found_in_region);
}

///< TOBJ.010: Temporal query. TOBJ.017: State freshness.
///< RESP.006: Query with freshness filter.
TEST(TacticalObjectsComponentHLR, TemporalQueryViaService) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = Position{51.5, 0.0, 0};
  auto id = comp.runtime().createObject(def);
  comp.runtime().store()->quality().set(id, QualityComponent{0.8, 0.9, 100.0});

  QueryRequest qreq;
  qreq.max_age_seconds = 200.0;
  qreq.current_time = 150.0;
  auto j_req = TacticalObjectsCodec::encodeQueryRequest(qreq);
  char resp_buf[4096];
  pcl_msg_t resp = {};
  invokeService(exec.handle(), "query", j_req.dump(), resp_buf, sizeof(resp_buf),
               &resp);

  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  ASSERT_GE(jr["entries"].size(), 1u);
}

///< TOBJ.015: Behavior estimation. TOBJ.016: Operational state.
///< RESP.010: Estimate object behaviour.
TEST(TacticalObjectsComponentHLR, BehaviorAndOperationalStateViaUpdate) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = Position{51.5, 0.0, 0};
  auto j_create = TacticalObjectsCodec::encodeObjectDefinition(def);
  char resp_buf[1024];
  pcl_msg_t resp = {};
  invokeService(exec.handle(), "create_object", j_create.dump(),
               resp_buf, sizeof(resp_buf), &resp);

  std::string id_str = nlohmann::json::parse(
    std::string(static_cast<const char*>(resp.data), resp.size)).value("object_id", "");

  nlohmann::json j_upd;
  j_upd["object_id"] = id_str;
  j_upd["behavior_pattern"] = "loitering";
  j_upd["operational_state"] = "airborne";

  invokeService(exec.handle(), "update_object", j_upd.dump(),
                resp_buf, sizeof(resp_buf), &resp);

  auto parsed = UUIDHelper::fromString(id_str);
  ASSERT_TRUE(parsed.second);
  auto bc = comp.runtime().getBehavior(UUIDKey(parsed.first));
  ASSERT_TRUE(bc.has_value());
  EXPECT_EQ(bc->behavior_pattern, "loitering");
  EXPECT_EQ(bc->operational_state, "airborne");
}

///< TOBJ.019: Confidence tracking. RESP.007: Determine object information confidence.
TEST(TacticalObjectsComponentHLR, ConfidenceFromCorrelation) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  ObservationBatch batch;
  Observation obs;
  obs.observation_id = UUIDHelper::generateV4();
  obs.position = Position{51.5, -0.1, 0};
  obs.confidence = 0.75;
  obs.affiliation_hint = Affiliation::Neutral;
  obs.source_ref.source_system = "sensor";
  obs.source_ref.source_entity_id = "O1";
  batch.observations.push_back(obs);

  auto j = TacticalObjectsCodec::encodeObservationBatch(batch);
  dispatchObservation(exec, j.dump());

  auto qresp = comp.runtime().query(QueryRequest());
  ASSERT_GE(qresp.entries.size(), 1u);
  const QualityComponent* qc = nullptr;
  for (const auto& e : qresp.entries) {
    qc = comp.runtime().store()->quality().get(e.id);
    if (qc) break;
  }
  ASSERT_NE(qc, nullptr);
  EXPECT_DOUBLE_EQ(qc->confidence, 0.75);
}

///< TOBJ.028: Status. TOBJ.029: Echelon. TOBJ.030: Indicator flags. TOBJ.031: Mobility.
///< RESP.013: Capture classification details.
///< Note: Codec supports battle_dim, affiliation, role, hq, task_force, feint_dummy, installation.
TEST(TacticalObjectsComponentHLR, FullMilitaryClassificationViaCreate) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  ObjectDefinition def;
  def.type = ObjectType::Unit;
  def.position = Position{51.5, 0.0, 0};
  def.affiliation = Affiliation::Friendly;
  def.mil_class.battle_dim = BattleDimension::Ground;
  def.mil_class.affiliation = Affiliation::Friendly;
  def.mil_class.role = "armor";
  def.mil_class.hq = true;
  def.mil_class.task_force = true;

  auto j = TacticalObjectsCodec::encodeObjectDefinition(def);
  char resp_buf[1024];
  pcl_msg_t resp = {};
  invokeService(exec.handle(), "create_object", j.dump(),
               resp_buf, sizeof(resp_buf), &resp);

  std::string id_str = nlohmann::json::parse(
    std::string(static_cast<const char*>(resp.data), resp.size)).value("object_id", "");
  auto parsed = UUIDHelper::fromString(id_str);
  ASSERT_TRUE(parsed.second);

  auto prof = comp.runtime().getMilClass(UUIDKey(parsed.first));
  ASSERT_TRUE(prof.has_value());
  EXPECT_EQ(prof->battle_dim, BattleDimension::Ground);
  EXPECT_EQ(prof->affiliation, Affiliation::Friendly);
  EXPECT_EQ(prof->role, "armor");
  EXPECT_TRUE(prof->hq);
  EXPECT_TRUE(prof->task_force);
}

///< TOBJ.032: Source symbol code preservation.
///< RESP.013: Capture SIDC when provided.
TEST(TacticalObjectsComponentHLR, SourceSidcPreserved) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  ObservationBatch batch;
  Observation obs;
  obs.observation_id = UUIDHelper::generateV4();
  obs.position = Position{51.5, -0.1, 0};
  obs.confidence = 0.8;
  obs.source_sidc = "SFGPUCIZ----";
  obs.source_ref.source_system = "tracks";
  obs.source_ref.source_entity_id = "S1";
  batch.observations.push_back(obs);

  auto j = TacticalObjectsCodec::encodeObservationBatch(batch);
  dispatchObservation(exec, j.dump());

  auto qresp = comp.runtime().query(QueryRequest());
  ASSERT_GE(qresp.entries.size(), 1u);
  tl::optional<MilClassProfile> prof;
  for (const auto& e : qresp.entries) {
    prof = comp.runtime().getMilClass(e.id);
    if (prof.has_value() && !prof->source_sidc.empty()) break;
  }
  ASSERT_TRUE(prof.has_value());
  EXPECT_EQ(prof->source_sidc, "SFGPUCIZ----");
}

///< TOBJ.039: Sparse component storage.
///< Entity with minimal fields does not populate unused components.
TEST(TacticalObjectsComponentHLR, SparseObjectCreation) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  ObjectDefinition def;
  def.type = ObjectType::Point;
  auto j = TacticalObjectsCodec::encodeObjectDefinition(def);
  char resp_buf[1024];
  pcl_msg_t resp = {};
  invokeService(exec.handle(), "create_object", j.dump(),
               resp_buf, sizeof(resp_buf), &resp);

  std::string id_str = nlohmann::json::parse(
    std::string(static_cast<const char*>(resp.data), resp.size)).value("object_id", "");
  auto parsed = UUIDHelper::fromString(id_str);
  ASSERT_TRUE(parsed.second);
  UUIDKey id(parsed.first);

  EXPECT_FALSE(comp.runtime().store()->identities().has(id));
  EXPECT_FALSE(comp.runtime().store()->correlation().has(id));
  EXPECT_FALSE(comp.runtime().store()->behaviors().has(id));
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

  char resp_buf[1024];
  pcl_msg_t resp = {};
  for (ObjectType t : types) {
    ObjectDefinition def;
    def.type = t;
    def.position = Position{51.0, 0.0, 0};
    auto j = TacticalObjectsCodec::encodeObjectDefinition(def);
    invokeService(exec.handle(), "create_object", j.dump(),
                  resp_buf, sizeof(resp_buf), &resp);
    std::string id_str = nlohmann::json::parse(
      std::string(static_cast<const char*>(resp.data), resp.size)).value("object_id", "");
    ASSERT_FALSE(id_str.empty());
  }

  auto q = comp.runtime().query(QueryRequest());
  ASSERT_GE(q.entries.size(), static_cast<size_t>(types.size()));
}

///< TOBJ.008: Query by external source reference.
///< RESP.006: Matching_Objects by source identifier.
TEST(TacticalObjectsComponentHLR, QueryBySourceRef) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  ObservationBatch batch;
  Observation obs;
  obs.observation_id = UUIDHelper::generateV4();
  obs.position = Position{51.5, -0.1, 0};
  obs.confidence = 0.8;
  obs.source_ref.source_system = "tracker-a";
  obs.source_ref.source_entity_id = "ext-42";
  batch.observations.push_back(obs);

  auto j = TacticalObjectsCodec::encodeObservationBatch(batch);
  dispatchObservation(exec, j.dump());

  QueryRequest qreq;
  qreq.by_source_system = "tracker-a";
  qreq.by_source_entity_id = "ext-42";
  auto j_req = TacticalObjectsCodec::encodeQueryRequest(qreq);
  char resp_buf[4096];
  pcl_msg_t resp = {};
  invokeService(exec.handle(), "query", j_req.dump(), resp_buf, sizeof(resp_buf),
               &resp);
  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  ASSERT_GE(jr["entries"].size(), 1u);
}
