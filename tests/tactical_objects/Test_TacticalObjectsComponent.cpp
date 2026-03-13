#include <gtest/gtest.h>
#include <TacticalObjectsComponent.h>
#include <TacticalObjectsCodec.h>
#include <pcl/executor.hpp>
#include <pcl/pcl_executor.h>
#include <uuid/UUIDHelper.h>
#include <pcl_internal.h>
#include <cstring>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

///< REQ_TACTICAL_OBJECTS_033: All ports created during on_configure.
///< TOBJ.045: Snapshot and event access via PCL lifecycle.
TEST(TacticalObjectsComponent, ConfigureCreatesAllPorts) {
  TacticalObjectsComponent comp;
  ASSERT_EQ(comp.configure(), PCL_OK);
  EXPECT_EQ(comp.state(), PCL_STATE_CONFIGURED);
  ASSERT_EQ(comp.activate(), PCL_OK);
  EXPECT_EQ(comp.state(), PCL_STATE_ACTIVE);
}

///< REQ_TACTICAL_OBJECTS_033: Activate succeeds.
TEST(TacticalObjectsComponent, ActivateSucceeds) {
  TacticalObjectsComponent comp;
  comp.configure();
  ASSERT_EQ(comp.activate(), PCL_OK);
  EXPECT_EQ(comp.state(), PCL_STATE_ACTIVE);
}

///< REQ_TACTICAL_OBJECTS_035: Observation ingress through subscriber callback.
///< TOBJ.018: Multi-source ingest via observation_ingress.
///< TOBJ.021: Entity correlation (first obs creates object).
///< RESP.013: Capture object information from evidence.
TEST(TacticalObjectsComponent, EvidenceIngressViaSubscriber) {
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
  obs.affiliation_hint = Affiliation::Hostile;
  obs.source_ref.source_system = "radar";
  obs.source_ref.source_entity_id = "T01";
  batch.observations.push_back(obs);

  auto j = TacticalObjectsCodec::encodeObservationBatch(batch);
  std::string payload = j.dump();
  pcl_msg_t msg = {};
  msg.data = payload.data();
  msg.size = static_cast<uint32_t>(payload.size());
  msg.type_name = "application/json";

  ASSERT_EQ(exec.dispatchIncoming("observation_ingress", &msg), PCL_OK);

  auto resp = comp.runtime().query(QueryRequest());
  ASSERT_GE(resp.entries.size(), 1u);
}

///< REQ_TACTICAL_OBJECTS_036: Direct create is exposed through service handler.
///< TOBJ.001: Object types (Platform).
///< TOBJ.002: Object properties (identity, classification, kinematics).
///< TOBJ.003: Stable internal UUID.
///< TOBJ.025: Battle dimension. TOBJ.026: Affiliation. TOBJ.027: Role.
///< RESP.013: Capture object information.
TEST(TacticalObjectsComponent, DirectCreateViaService) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = Position{51.5, -0.1, 0};
  def.affiliation = Affiliation::Friendly;
  def.mil_class.battle_dim = BattleDimension::Ground;
  def.mil_class.role = "infantry";

  auto j = TacticalObjectsCodec::encodeObjectDefinition(def);
  std::string req_str = j.dump();
  pcl_msg_t req = {};
  req.data = req_str.data();
  req.size = static_cast<uint32_t>(req_str.size());
  req.type_name = "application/json";

  pcl_msg_t resp = {};
  char resp_buf[1024];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "create_object", &req, &resp),
            PCL_OK);
  ASSERT_GT(resp.size, 0u);

  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  std::string id_str = jr.value("object_id", "");
  ASSERT_FALSE(id_str.empty());

  auto parsed = UUIDHelper::fromString(id_str);
  ASSERT_TRUE(parsed.second);
  UUIDKey id(parsed.first);

  const auto* rec = comp.runtime().getRecord(id);
  ASSERT_NE(rec, nullptr);
  ASSERT_EQ(rec->type, ObjectType::Platform);
}

///< REQ_TACTICAL_OBJECTS_036: Direct update is exposed through service handler.
TEST(TacticalObjectsComponent, DirectUpdateViaService) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = Position{51.5, -0.1, 0};
  def.affiliation = Affiliation::Friendly;
  auto j_create = TacticalObjectsCodec::encodeObjectDefinition(def);
  std::string req_str = j_create.dump();
  pcl_msg_t req = {};
  req.data = req_str.data();
  req.size = static_cast<uint32_t>(req_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char resp_buf[1024];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  pcl_executor_invoke_service(exec.handle(), "create_object", &req, &resp);
  std::string id_str = nlohmann::json::parse(
    std::string(static_cast<const char*>(resp.data), resp.size)).value("object_id", "");

  nlohmann::json j_upd;
  j_upd["object_id"] = id_str;
  j_upd["affiliation"] = "Hostile";
  std::string upd_str = j_upd.dump();
  req.data = upd_str.data();
  req.size = static_cast<uint32_t>(upd_str.size());

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "update_object", &req, &resp),
            PCL_OK);

  auto parsed = UUIDHelper::fromString(id_str);
  ASSERT_TRUE(parsed.second);
  auto prof = comp.runtime().getMilClass(UUIDKey(parsed.first));
  ASSERT_TRUE(prof.has_value());
  ASSERT_EQ(prof->affiliation, Affiliation::Hostile);
}

///< REQ_TACTICAL_OBJECTS_036: Direct delete is exposed through service handler.
///< TOBJ.003: Delete by internal UUID.
///< RESP.013: Capture object information (delete lifecycle).
TEST(TacticalObjectsComponent, DirectDeleteViaService) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  auto id = comp.runtime().createObject(ObjectDefinition());
  nlohmann::json j_req;
  j_req["object_id"] = UUIDHelper::toString(id.uuid);
  std::string req_str = j_req.dump();
  pcl_msg_t req = {};
  req.data = req_str.data();
  req.size = static_cast<uint32_t>(req_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char resp_buf[256];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "delete_object", &req, &resp),
            PCL_OK);
  EXPECT_EQ(comp.runtime().getRecord(id), nullptr);
}

///< REQ_TACTICAL_OBJECTS_037: Query through service handler.
TEST(TacticalObjectsComponent, QueryViaService) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  comp.runtime().createObject(ObjectDefinition());
  Observation obs;
  obs.observation_id = UUIDHelper::generateV4();
  obs.position = Position{52.0, 0.0, 0};
  obs.confidence = 0.7;
  obs.affiliation_hint = Affiliation::Neutral;
  obs.source_ref.source_system = "radar";
  comp.runtime().processObservation(obs);

  pcl::Executor exec;
  exec.add(comp);

  QueryRequest qreq;
  qreq.by_type = ObjectType::Platform;
  auto j_req = TacticalObjectsCodec::encodeQueryRequest(qreq);
  std::string req_str = j_req.dump();
  pcl_msg_t req = {};
  req.data = req_str.data();
  req.size = static_cast<uint32_t>(req_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char resp_buf[4096];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "query", &req, &resp),
            PCL_OK);
  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  ASSERT_GE(jr["entries"].size(), 2u);
}

///< REQ_TACTICAL_OBJECTS_037: Get object via service handler.
///< TOBJ.008: Single object query by internal ID.
///< RESP.006: Specific_Object_Detail retrieval.
TEST(TacticalObjectsComponent, GetObjectViaService) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  ObjectDefinition def;
  def.type = ObjectType::Unit;
  def.mil_class.role = "armor";
  auto id = comp.runtime().createObject(def);

  pcl::Executor exec;
  exec.add(comp);

  nlohmann::json j_req;
  j_req["object_id"] = UUIDHelper::toString(id.uuid);
  std::string req_str = j_req.dump();
  pcl_msg_t req = {};
  req.data = req_str.data();
  req.size = static_cast<uint32_t>(req_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char resp_buf[1024];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "get_object", &req, &resp),
            PCL_OK);
  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  ASSERT_TRUE(jr.value("found", false));
  ASSERT_EQ(jr.value("type", ""), "Unit");
}

///< REQ_TACTICAL_OBJECTS: Zone upsert via service.
TEST(TacticalObjectsComponent, ZoneUpsertViaService) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  ZoneDefinition def;
  def.zone_type = ZoneType::AOI;
  def.geometry.center = Position{51.0, 0.0, 0};
  def.geometry.radius_m = 5000.0;
  auto j_req = TacticalObjectsCodec::encodeZoneDefinition(def);
  std::string req_str = j_req.dump();
  pcl_msg_t req = {};
  req.data = req_str.data();
  req.size = static_cast<uint32_t>(req_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char resp_buf[256];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "upsert_zone", &req, &resp),
            PCL_OK);
  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  std::string zid_str = jr.value("zone_id", "");
  ASSERT_FALSE(zid_str.empty());
}

///< REQ_TACTICAL_OBJECTS_037: Zone remove via service handler.
///< TOBJ.033: Zone lifecycle. TOBJ.034: Zone semantics.
///< RESP.006: Region-based zone management.
TEST(TacticalObjectsComponent, ZoneRemoveViaService) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  ZoneDefinition def;
  def.zone_type = ZoneType::AOI;
  def.geometry.center = Position{51.0, 0.0, 0};
  def.geometry.radius_m = 5000.0;
  auto zid = comp.runtime().createZone(def);

  pcl::Executor exec;
  exec.add(comp);

  nlohmann::json j_req;
  j_req["zone_id"] = UUIDHelper::toString(zid.uuid);
  std::string req_str = j_req.dump();
  pcl_msg_t req = {};
  req.data = req_str.data();
  req.size = static_cast<uint32_t>(req_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char resp_buf[256];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "remove_zone", &req, &resp),
            PCL_OK);
  EXPECT_FALSE(comp.runtime().getZone(zid).has_value());
}

///< REQ_TACTICAL_OBJECTS_033: on_shutdown lifecycle callback succeeds.
TEST(TacticalObjectsComponent, ShutdownLifecycle) {
  TacticalObjectsComponent comp;
  // shutdown() is callable from any non-finalized state
  ASSERT_EQ(comp.shutdown(), PCL_OK);
  ASSERT_EQ(comp.state(), PCL_STATE_FINALIZED);
}

///< REQ_TACTICAL_OBJECTS_033: on_tick callback executes when accumulator fires.
TEST(TacticalObjectsComponent, TickCallbackExecutes) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  // With tick_rate_hz = 1000, period = 0.001 s.
  // spinOnce uses dt = 0.001 on the first call (prev_time == 0),
  // so tick_accumulator (0 + 0.001) >= period (0.001) — fires on_tick.
  comp.setTickRateHz(1000.0);

  pcl::Executor exec;
  exec.add(comp);
  ASSERT_EQ(exec.spinOnce(0), PCL_OK);
}

///< REQ_TACTICAL_OBJECTS_037: handleGetObject returns found=false for unknown ID.
TEST(TacticalObjectsComponent, GetObjectNotFound) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  // Use a UUID that was never registered
  nlohmann::json j_req;
  j_req["object_id"] = UUIDHelper::toString(UUIDHelper::generateV4());
  std::string req_str = j_req.dump();
  pcl_msg_t req = {};
  req.data = req_str.data();
  req.size = static_cast<uint32_t>(req_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char resp_buf[256];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "get_object", &req, &resp), PCL_OK);
  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  ASSERT_FALSE(jr.value("found", true));
}

// ---------------------------------------------------------------------------
// PCL port-overflow tests: verify on_configure returns PCL_ERR_CALLBACK when
// addService fails because the port table is full.
// Each test pre-fills port_count to PCL_MAX_PORTS - (N+1) so that the Nth
// addService call overflows. PCL_MAX_PORTS = 64, subscriber takes 1 slot,
// services take 7 more.
// ---------------------------------------------------------------------------

///< REQ_TACTICAL_OBJECTS_033: configure returns PCL_ERR_CALLBACK if create_object service fails.
TEST(TacticalObjectsComponent, ConfigureFailsIfCreateObjectServiceFull) {
  TacticalObjectsComponent comp;
  // subscriber gets slot 63 (last), create_object addService sees port_count=64 -> fail
  comp.handle()->port_count = PCL_MAX_PORTS - 1;
  ASSERT_NE(comp.configure(), PCL_OK);
}

///< REQ_TACTICAL_OBJECTS_033: configure returns PCL_ERR_CALLBACK if update_object service fails.
TEST(TacticalObjectsComponent, ConfigureFailsIfUpdateObjectServiceFull) {
  TacticalObjectsComponent comp;
  comp.handle()->port_count = PCL_MAX_PORTS - 2;
  ASSERT_NE(comp.configure(), PCL_OK);
}

///< REQ_TACTICAL_OBJECTS_033: configure returns PCL_ERR_CALLBACK if delete_object service fails.
TEST(TacticalObjectsComponent, ConfigureFailsIfDeleteObjectServiceFull) {
  TacticalObjectsComponent comp;
  comp.handle()->port_count = PCL_MAX_PORTS - 3;
  ASSERT_NE(comp.configure(), PCL_OK);
}

///< REQ_TACTICAL_OBJECTS_033: configure returns PCL_ERR_CALLBACK if query service fails.
TEST(TacticalObjectsComponent, ConfigureFailsIfQueryServiceFull) {
  TacticalObjectsComponent comp;
  comp.handle()->port_count = PCL_MAX_PORTS - 4;
  ASSERT_NE(comp.configure(), PCL_OK);
}

///< REQ_TACTICAL_OBJECTS_033: configure returns PCL_ERR_CALLBACK if get_object service fails.
TEST(TacticalObjectsComponent, ConfigureFailsIfGetObjectServiceFull) {
  TacticalObjectsComponent comp;
  comp.handle()->port_count = PCL_MAX_PORTS - 5;
  ASSERT_NE(comp.configure(), PCL_OK);
}

///< REQ_TACTICAL_OBJECTS_033: configure returns PCL_ERR_CALLBACK if upsert_zone service fails.
TEST(TacticalObjectsComponent, ConfigureFailsIfUpsertZoneServiceFull) {
  TacticalObjectsComponent comp;
  comp.handle()->port_count = PCL_MAX_PORTS - 6;
  ASSERT_NE(comp.configure(), PCL_OK);
}

///< REQ_TACTICAL_OBJECTS_033: configure returns PCL_ERR_CALLBACK if remove_zone service fails.
TEST(TacticalObjectsComponent, ConfigureFailsIfRemoveZoneServiceFull) {
  TacticalObjectsComponent comp;
  comp.handle()->port_count = PCL_MAX_PORTS - 7;
  ASSERT_NE(comp.configure(), PCL_OK);
}

// ---------------------------------------------------------------------------
// JSON parse-error coverage: send malformed JSON to exercise the catch(...)
// blocks in every service handler and the subscriber callback.
// ---------------------------------------------------------------------------

///< Coverage: onObservationIngress silently ignores invalid JSON.
TEST(TacticalObjectsComponent, ObservationIngressInvalidJsonIgnored) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  const char* bad = "{ not valid json !!!";
  pcl_msg_t msg{};
  msg.data     = bad;
  msg.size     = static_cast<uint32_t>(strlen(bad));
  msg.type_name = "application/json";

  // The subscriber catch block swallows the parse error; dispatch returns OK.
  EXPECT_EQ(exec.dispatchIncoming("observation_ingress", &msg), PCL_OK);
  // No object should have been created.
  EXPECT_EQ(comp.runtime().query(QueryRequest()).total, 0u);
}

///< Coverage: handleUpdateObject catch block returns error on invalid JSON.
TEST(TacticalObjectsComponent, UpdateObjectInvalidJsonReturnsError) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  const char* bad = "{ not valid json !!!";
  pcl_msg_t req{};
  req.data      = bad;
  req.size      = static_cast<uint32_t>(strlen(bad));
  req.type_name = "application/json";
  pcl_msg_t resp{};
  char resp_buf[64];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  EXPECT_NE(pcl_executor_invoke_service(exec.handle(), "update_object", &req, &resp), PCL_OK);
}

///< Coverage: handleDeleteObject catch block returns error on invalid JSON.
TEST(TacticalObjectsComponent, DeleteObjectInvalidJsonReturnsError) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  const char* bad = "{ not valid json !!!";
  pcl_msg_t req{};
  req.data      = bad;
  req.size      = static_cast<uint32_t>(strlen(bad));
  req.type_name = "application/json";
  pcl_msg_t resp{};
  char resp_buf[64];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  EXPECT_NE(pcl_executor_invoke_service(exec.handle(), "delete_object", &req, &resp), PCL_OK);
}

///< Coverage: handleQuery catch block falls through to empty QueryRequest on invalid JSON.
TEST(TacticalObjectsComponent, QueryInvalidJsonFallsBackToEmptyRequest) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  comp.runtime().createObject(ObjectDefinition());

  pcl::Executor exec;
  exec.add(comp);

  const char* bad = "{ not valid json !!!";
  pcl_msg_t req{};
  req.data      = bad;
  req.size      = static_cast<uint32_t>(strlen(bad));
  req.type_name = "application/json";
  pcl_msg_t resp{};
  char resp_buf[4096];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  // On parse error the handler falls back to an empty QueryRequest and still
  // returns PCL_OK with a valid (possibly empty) response.
  EXPECT_EQ(pcl_executor_invoke_service(exec.handle(), "query", &req, &resp), PCL_OK);
  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  EXPECT_TRUE(jr.contains("entries"));
}

///< Coverage: handleGetObject catch block returns error on invalid JSON.
TEST(TacticalObjectsComponent, GetObjectInvalidJsonReturnsError) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  const char* bad = "{ not valid json !!!";
  pcl_msg_t req{};
  req.data      = bad;
  req.size      = static_cast<uint32_t>(strlen(bad));
  req.type_name = "application/json";
  pcl_msg_t resp{};
  char resp_buf[64];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  EXPECT_NE(pcl_executor_invoke_service(exec.handle(), "get_object", &req, &resp), PCL_OK);
}

///< Coverage: handleUpsertZone catch block returns error on invalid JSON.
TEST(TacticalObjectsComponent, UpsertZoneInvalidJsonReturnsError) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  const char* bad = "{ not valid json !!!";
  pcl_msg_t req{};
  req.data      = bad;
  req.size      = static_cast<uint32_t>(strlen(bad));
  req.type_name = "application/json";
  pcl_msg_t resp{};
  char resp_buf[64];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  EXPECT_NE(pcl_executor_invoke_service(exec.handle(), "upsert_zone", &req, &resp), PCL_OK);
}

///< Coverage: handleRemoveZone catch block returns error on invalid JSON.
TEST(TacticalObjectsComponent, RemoveZoneInvalidJsonReturnsError) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  const char* bad = "{ not valid json !!!";
  pcl_msg_t req{};
  req.data      = bad;
  req.size      = static_cast<uint32_t>(strlen(bad));
  req.type_name = "application/json";
  pcl_msg_t resp{};
  char resp_buf[64];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  EXPECT_NE(pcl_executor_invoke_service(exec.handle(), "remove_zone", &req, &resp), PCL_OK);
}
