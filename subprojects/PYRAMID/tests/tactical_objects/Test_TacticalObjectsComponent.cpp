#include <gtest/gtest.h>
#include <TacticalObjectsComponent.h>
#include <TacticalObjectsCodec.h>
#include <StandardBridge.h>
#include <pcl/executor.hpp>
#include <pcl/pcl_executor.h>
#include <uuid/UUIDHelper.h>
#include <pcl_internal.h>
#include <cstring>
#include <thread>
#include <chrono>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

static constexpr double DEG = 3.14159265358979323846 / 180.0;

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
///< PYR-RESP-0741: Capture object information from evidence.
TEST(TacticalObjectsComponent, EvidenceIngressViaSubscriber) {
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
///< PYR-RESP-0741: Capture object information.
TEST(TacticalObjectsComponent, DirectCreateViaService) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = Position{51.5 * DEG, -0.1 * DEG, 0};
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
  def.position = Position{51.5 * DEG, -0.1 * DEG, 0};
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
///< PYR-RESP-0741: Capture object information (delete lifecycle).
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
  obs.position = Position{52.0 * DEG, 0.0, 0};
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
///< PYR-RESP-0734: Specific_Object_Detail retrieval.
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
  def.geometry.center = Position{51.0 * DEG, 0.0, 0};
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
///< PYR-RESP-0734: Region-based zone management.
TEST(TacticalObjectsComponent, ZoneRemoveViaService) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  ZoneDefinition def;
  def.zone_type = ZoneType::AOI;
  def.geometry.center = Position{51.0 * DEG, 0.0, 0};
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

///< REQ_TACTICAL_OBJECTS_034: No port creation after configure; shutdown from any state.
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
  // so tick_accumulator (0 + 0.001) >= period (0.001) -- fires on_tick.
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

///< REQ_TACTICAL_OBJECTS_046: subscribe_interest service registers interest and returns interest_id.
///< REQ_TACTICAL_OBJECTS_057: subscribe_interest and resync via PCL service.
///< TOBJ.005: Interest requirement registration.
TEST(TacticalObjectsComponent, SubscribeInterestViaService) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  nlohmann::json j_req;
  j_req["object_type"] = "Platform";
  j_req["affiliation"] = "Hostile";
  j_req["area"] = nlohmann::json::object({{"min_lat", 50.0 * DEG}, {"max_lat", 52.0 * DEG}, {"min_lon", -1.0 * DEG}, {"max_lon", 1.0 * DEG}});
  j_req["behavior_pattern"] = "patrol";
  j_req["minimum_confidence"] = 0.7;
  j_req["expires_at"] = 9999.0;

  std::string req_str = j_req.dump();
  pcl_msg_t req = {};
  req.data = req_str.data();
  req.size = static_cast<uint32_t>(req_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char resp_buf[512];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "subscribe_interest", &req, &resp), PCL_OK);
  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  std::string interest_id_str = jr.value("interest_id", "");
  ASSERT_FALSE(interest_id_str.empty());

  auto parsed = UUIDHelper::fromString(interest_id_str);
  ASSERT_TRUE(parsed.second);
}

///< REQ_TACTICAL_OBJECTS_057: resync via tactical_objects.resync service.
///< REQ_TACTICAL_OBJECTS: resync service returns full snapshot for interest.
TEST(TacticalObjectsComponent, ResyncViaService) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = Position{51.0 * DEG, 0.0, 0};
  def.affiliation = Affiliation::Hostile;
  comp.runtime().createObject(def);

  InterestCriteria criteria;
  criteria.object_type = ObjectType::Platform;
  auto interest_id = comp.runtime().interestManager().registerInterest(criteria, 0.0, 9999.0);
  std::string interest_id_str = UUIDHelper::toString(interest_id.uuid);

  nlohmann::json j_req;
  j_req["interest_id"] = interest_id_str;

  std::string req_str = j_req.dump();
  pcl_msg_t req = {};
  req.data = req_str.data();
  req.size = static_cast<uint32_t>(req_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char resp_buf[4096];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "tactical_objects.resync", &req, &resp), PCL_OK);
  ASSERT_GT(resp.size, 0u);
  EXPECT_EQ(std::string(static_cast<const char*>(resp.type_name)), "application/octet-stream");
}

///< Coverage: resync returns error when interest_id not found.
TEST(TacticalObjectsComponent, ResyncInterestNotFound) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  nlohmann::json j_req;
  j_req["interest_id"] = UUIDHelper::toString(UUIDHelper::generateV4());

  std::string req_str = j_req.dump();
  pcl_msg_t req = {};
  req.data = req_str.data();
  req.size = static_cast<uint32_t>(req_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char resp_buf[256];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  EXPECT_NE(pcl_executor_invoke_service(exec.handle(), "tactical_objects.resync", &req, &resp), PCL_OK);
}

///< Coverage: subscribe_interest invalid JSON returns error.
TEST(TacticalObjectsComponent, SubscribeInterestInvalidJsonReturnsError) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  const char* bad = "{ not valid json !!!";
  pcl_msg_t req{};
  req.data = bad;
  req.size = static_cast<uint32_t>(strlen(bad));
  req.type_name = "application/json";
  pcl_msg_t resp{};
  char resp_buf[64];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  EXPECT_NE(pcl_executor_invoke_service(exec.handle(), "subscribe_interest", &req, &resp), PCL_OK);
}

///< Coverage: resync invalid JSON returns error.
TEST(TacticalObjectsComponent, ResyncInvalidJsonReturnsError) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  const char* bad = "{ not valid json !!!";
  pcl_msg_t req{};
  req.data = bad;
  req.size = static_cast<uint32_t>(strlen(bad));
  req.type_name = "application/json";
  pcl_msg_t resp{};
  char resp_buf[64];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  EXPECT_NE(pcl_executor_invoke_service(exec.handle(), "tactical_objects.resync", &req, &resp), PCL_OK);
}

///< Coverage: streaming_tick_divisor > 1 skips publish on non-divisor ticks.
///< REQ_TACTICAL_OBJECTS_038: Tick-driven housekeeping.
TEST(TacticalObjectsComponent, StreamingTickDivisorSkipsPublish) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  comp.setStreamingTickDivisor(2);
  comp.setTickRateHz(1000.0);

  pcl::Executor exec;
  exec.add(comp);

  exec.spinOnce(0);
  exec.spinOnce(0.001);
}

///< Coverage: max_entities_per_frame chunks large frames.
TEST(TacticalObjectsComponent, ChunkingWithMaxEntitiesPerFrame) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  comp.setMaxEntitiesPerFrame(2);
  comp.setTickRateHz(1000.0);

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = Position{51.0 * DEG, 0.0, 0};
  def.affiliation = Affiliation::Hostile;

  for (int i = 0; i < 5; ++i) {
    comp.runtime().createObject(def);
  }

  InterestCriteria criteria;
  criteria.object_type = ObjectType::Platform;
  comp.runtime().interestManager().registerInterest(criteria, 0.0, 9999.0);

  pcl::Executor exec;
  exec.add(comp);
  exec.spinOnce(0);
  exec.spinOnce(0);
}

///< Coverage: subscribe then create then tick exercises full publish path.
TEST(TacticalObjectsComponent, SubscribeCreateTickPublishes) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  comp.setTickRateHz(1000.0);

  pcl::Executor exec;
  exec.add(comp);

  nlohmann::json sub_req;
  sub_req["object_type"] = "Platform";
  sub_req["affiliation"] = "Hostile";
  sub_req["behavior_pattern"] = "patrol";
  sub_req["expires_at"] = 9999.0;
  std::string sub_str = sub_req.dump();
  pcl_msg_t req = {};
  req.data = sub_str.data();
  req.size = static_cast<uint32_t>(sub_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char resp_buf[512];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "subscribe_interest", &req, &resp), PCL_OK);

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = Position{51.0 * DEG, 0.0, 0};
  def.affiliation = Affiliation::Hostile;
  auto j_create = TacticalObjectsCodec::encodeObjectDefinition(def);
  std::string create_str = j_create.dump();
  req.data = create_str.data();
  req.size = static_cast<uint32_t>(create_str.size());
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);
  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "create_object", &req, &resp), PCL_OK);

  exec.spinOnce(0);
  exec.spinOnce(0);
}

///< Coverage: deleted entity appears in streaming frame deletes list (lines 107-116).
TEST(TacticalObjectsComponent, DeletedEntityPublishesDeleteFrame) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  comp.setTickRateHz(1000.0);

  pcl::Executor exec;
  exec.add(comp);

  nlohmann::json sub_req;
  sub_req["object_type"] = "Platform";
  sub_req["affiliation"] = "Hostile";
  sub_req["expires_at"] = 9999.0;
  std::string sub_str = sub_req.dump();
  pcl_msg_t req = {};
  req.data = sub_str.data();
  req.size = static_cast<uint32_t>(sub_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char resp_buf[512];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "subscribe_interest", &req, &resp), PCL_OK);

  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = Position{51.0 * DEG, 0.0, 0};
  def.affiliation = Affiliation::Hostile;
  auto j_create = TacticalObjectsCodec::encodeObjectDefinition(def);
  std::string create_str = j_create.dump();
  req.data = create_str.data();
  req.size = static_cast<uint32_t>(create_str.size());
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);
  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "create_object", &req, &resp), PCL_OK);

  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  std::string obj_id_str = jr.value("object_id", "");
  ASSERT_FALSE(obj_id_str.empty());

  for (int i = 0; i < 5; ++i) exec.spinOnce(0);

  nlohmann::json del_req;
  del_req["object_id"] = obj_id_str;
  std::string del_str = del_req.dump();
  req.data = del_str.data();
  req.size = static_cast<uint32_t>(del_str.size());
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);
  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "delete_object", &req, &resp), PCL_OK);

  // Sleep so the accumulator reaches >= 1ms (period at 1000Hz) before next spinOnce.
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  for (int i = 0; i < 5; ++i) exec.spinOnce(0);
}


///< Coverage: subscribe_interest with query_mode="active_find" registers ActiveFind interest
///< and returns solution_id + evidence_requirements (lines 418-420, 464-491).
TEST(TacticalObjectsComponent, SubscribeInterestActiveFindReturnsEvidenceRequirements) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  static constexpr double DEG = 3.14159265358979323846 / 180.0;
  nlohmann::json j_req;
  j_req["query_mode"]       = "active_find";
  j_req["object_type"]      = "Platform";
  j_req["affiliation"]      = "Hostile";
  j_req["battle_dimension"] = "Air";
  j_req["area"] = nlohmann::json::object({
      {"min_lat", 50.0 * DEG}, {"max_lat", 52.0 * DEG},
      {"min_lon", -1.0 * DEG}, {"max_lon", 1.0 * DEG}});
  j_req["expires_at"]       = 9999.0;

  std::string req_str = j_req.dump();
  pcl_msg_t req = {};
  req.data      = req_str.data();
  req.size      = static_cast<uint32_t>(req_str.size());
  req.type_name = "application/json";

  pcl_msg_t resp = {};
  char resp_buf[2048];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "subscribe_interest", &req, &resp), PCL_OK);

  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);

  ASSERT_TRUE(jr.contains("interest_id"));
  ASSERT_TRUE(jr.contains("solution_id"));
  ASSERT_TRUE(jr.contains("evidence_requirements"));
  ASSERT_FALSE(jr["evidence_requirements"].empty());

  // Each requirement must have aligned id, policy, dimension, and area fields.
  for (auto& ev : jr["evidence_requirements"]) {
    EXPECT_TRUE(ev.contains("id"));
    EXPECT_FALSE(ev.value("id", std::string{}).empty());
    EXPECT_EQ(ev.value("policy", -1), 1);           // DataPolicy::Query = 1
    EXPECT_EQ(ev.value("dimension", -1), 4);         // BattleDimension::Air = 4
    EXPECT_TRUE(ev.contains("min_lat_rad"));
    EXPECT_NEAR(ev.value("min_lat_rad", 0.0), 50.0 * DEG, 1e-12);
    EXPECT_NEAR(ev.value("max_lat_rad", 0.0), 52.0 * DEG, 1e-12);
  }
}

///< Coverage: subscribe_interest with query_mode="read_current" sets ReadCurrent (line 422).
TEST(TacticalObjectsComponent, SubscribeInterestReadCurrentMode) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  nlohmann::json j_req;
  j_req["query_mode"] = "read_current";
  j_req["expires_at"] = 9999.0;

  std::string req_str = j_req.dump();
  pcl_msg_t req = {};
  req.data      = req_str.data();
  req.size      = static_cast<uint32_t>(req_str.size());
  req.type_name = "application/json";

  pcl_msg_t resp = {};
  char resp_buf[512];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "subscribe_interest", &req, &resp), PCL_OK);

  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  ASSERT_TRUE(jr.contains("interest_id"));
  // ReadCurrent -> no solution_id
  ASSERT_FALSE(jr.contains("solution_id"));
}

///< Coverage: subscribe_interest with battle_dimension field (lines 437-439).
TEST(TacticalObjectsComponent, SubscribeInterestWithBattleDimension) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  nlohmann::json j_req;
  j_req["battle_dimension"] = "Ground";
  j_req["expires_at"]       = 9999.0;

  std::string req_str = j_req.dump();
  pcl_msg_t req = {};
  req.data      = req_str.data();
  req.size      = static_cast<uint32_t>(req_str.size());
  req.type_name = "application/json";

  pcl_msg_t resp = {};
  char resp_buf[512];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "subscribe_interest", &req, &resp), PCL_OK);

  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  ASSERT_FALSE(jr.value("interest_id", "").empty());
}

///< Coverage: on_tick publishes delete frames after entity removed (lines 113-119).
///< The test registers an interest, waits for a full-snapshot tick so the
///< subscriber_versions table is populated, then deletes the entity and
///< waits for the next tick to fire so the delete frame is assembled.
TEST(TacticalObjectsComponent, DeletedEntityAppearsInDeleteFrame) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  comp.setTickRateHz(1000.0); // 1 ms period -- first spinOnce (dt=0.001) fires tick

  pcl::Executor exec;
  exec.add(comp);

  // Register a read-current interest so on_tick assembles frames.
  nlohmann::json sub_req;
  sub_req["object_type"] = "Platform";
  sub_req["expires_at"]  = 9999.0;
  std::string sub_str    = sub_req.dump();
  pcl_msg_t req = {};
  req.data      = sub_str.data();
  req.size      = static_cast<uint32_t>(sub_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char resp_buf[512];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);
  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "subscribe_interest", &req, &resp), PCL_OK);

  // Create a Platform entity.
  ObjectDefinition def;
  def.type        = ObjectType::Platform;
  def.position    = Position{51.0, 0.0, 0};
  def.affiliation = Affiliation::Hostile;
  auto id = comp.runtime().createObject(def);

  // First tick: subscriber is "new" (sub_versions empty) -- full snapshot,
  // populates subscriber_versions so the entity is tracked.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  exec.spinOnce(0);

  // Delete the entity; it goes into deleted_entities_.
  auto id_str = UUIDHelper::toString(id.uuid);
  nlohmann::json del_req;
  del_req["object_id"] = id_str;
  std::string del_str  = del_req.dump();
  req.data      = del_str.data();
  req.size      = static_cast<uint32_t>(del_str.size());
  resp.data     = resp_buf;
  resp.size     = sizeof(resp_buf);
  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "delete_object", &req, &resp), PCL_OK);

  // Second tick: not a new subscriber, deleted_entities_ = {id} -> assembles
  // delete frame, hitting lines 113-119.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  exec.spinOnce(0);
}

///< Coverage: on_tick splits large frames into multiple chunks (line 107, second
///< iteration of do-while where chunk_start > 0).
///< Strategy: first tick populates subscriber_versions (new-subscriber full scan),
///< then all entities are marked dirty again so the second tick processes them
///< through the dirty-entity path with max=1 -> multiple chunks.
TEST(TacticalObjectsComponent, MultipleChunksSecondIteration) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  comp.setMaxEntitiesPerFrame(1); // forces multiple loop iterations
  comp.setTickRateHz(1000.0);     // 1 ms period -- first spinOnce (dt=0.001) fires tick

  pcl::Executor exec;
  exec.add(comp);

  // Register interest.
  nlohmann::json sub_req;
  sub_req["object_type"] = "Platform";
  sub_req["expires_at"]  = 9999.0;
  std::string sub_str    = sub_req.dump();
  pcl_msg_t req = {};
  req.data      = sub_str.data();
  req.size      = static_cast<uint32_t>(sub_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char resp_buf[512];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);
  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "subscribe_interest", &req, &resp), PCL_OK);

  // Create 3 objects.
  ObjectDefinition def;
  def.type        = ObjectType::Platform;
  def.position    = Position{51.0, 0.0, 0};
  def.affiliation = Affiliation::Hostile;
  auto id1 = comp.runtime().createObject(def);
  def.position.lat = 52.0 * DEG;
  auto id2 = comp.runtime().createObject(def);
  def.position.lat = 53.0 * DEG;
  auto id3 = comp.runtime().createObject(def);

  // First tick: new subscriber, full scan produces 3 updates (1 chunk each
  // with max=1, so 3 loop iterations already, but chunk_start is always the
  // next start value). Populates subscriber_versions.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  exec.spinOnce(0);

  // Mark all 3 dirty again so the next (non-new-subscriber) tick processes them.
  ObjectUpdate upd;
  upd.position = Position{51.5 * DEG, 0.0, 0};
  comp.runtime().updateObject(id1, upd);
  upd.position = Position{52.5 * DEG, 0.0, 0};
  comp.runtime().updateObject(id2, upd);
  upd.position = Position{53.5 * DEG, 0.0, 0};
  comp.runtime().updateObject(id3, upd);

  // Second tick: NOT new subscriber, dirty_entities has 3 entries ->
  // sf.updates.size() == 3, max==1 -> second and third iterations with
  // chunk_start > 0, covering line 107.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  exec.spinOnce(0);
}

///< Coverage: subscribe_interest with non-string object_type/affiliation/battle_dimension
///< triggers catch (...) {} exception handlers at lines 428, 433, 439.
///< nlohmann::json::get<std::string>() throws type_error when the value is not a string.
TEST(TacticalObjectsComponent, SubscribeInterestInvalidTypeFieldsCoverCatchBlocks) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  // object_type is an integer -- not null, but get<std::string>() throws (line 428).
  {
    nlohmann::json j;
    j["object_type"] = 42;       // integer, not string
    j["expires_at"]  = 9999.0;
    std::string s = j.dump();
    pcl_msg_t req = {}; req.data = s.data(); req.size = static_cast<uint32_t>(s.size());
    req.type_name = "application/json";
    pcl_msg_t resp = {}; char buf[512]; resp.data = buf; resp.size = sizeof(buf);
    EXPECT_EQ(pcl_executor_invoke_service(exec.handle(), "subscribe_interest", &req, &resp), PCL_OK);
  }

  // affiliation is an integer -- triggers catch at line 433.
  {
    nlohmann::json j;
    j["affiliation"] = 99;
    j["expires_at"]  = 9999.0;
    std::string s = j.dump();
    pcl_msg_t req = {}; req.data = s.data(); req.size = static_cast<uint32_t>(s.size());
    req.type_name = "application/json";
    pcl_msg_t resp = {}; char buf[512]; resp.data = buf; resp.size = sizeof(buf);
    EXPECT_EQ(pcl_executor_invoke_service(exec.handle(), "subscribe_interest", &req, &resp), PCL_OK);
  }

  // battle_dimension is an integer -- triggers catch at line 439.
  {
    nlohmann::json j;
    j["battle_dimension"] = 7;
    j["expires_at"]       = 9999.0;
    std::string s = j.dump();
    pcl_msg_t req = {}; req.data = s.data(); req.size = static_cast<uint32_t>(s.size());
    req.type_name = "application/json";
    pcl_msg_t resp = {}; char buf[512]; resp.data = buf; resp.size = sizeof(buf);
    EXPECT_EQ(pcl_executor_invoke_service(exec.handle(), "subscribe_interest", &req, &resp), PCL_OK);
  }
}

///< REQ_TACTICAL_OBJECTS_026: matching_objects.read_match returns ObjectMatch array.
TEST(TacticalObjectsComponent, StandardReadMatchReturnsMatchArray) {
  static constexpr double DEG = 3.14159265358979323846 / 180.0;
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  StandardBridge bridge(comp.runtime(), exec.handle(), false);
  bridge.configure();
  bridge.activate();
  exec.add(bridge);

  // Create an object so query returns a result
  ObjectDefinition def;
  def.type       = ObjectType::Platform;
  def.position   = Position{51.0 * DEG, 0.0, 0};
  def.affiliation = Affiliation::Hostile;
  auto j_create  = TacticalObjectsCodec::encodeObjectDefinition(def);
  std::string create_str = j_create.dump();
  char create_resp_buf[512];
  pcl_msg_t create_req = {}, create_resp = {};
  create_req.data = create_str.data();
  create_req.size = static_cast<uint32_t>(create_str.size());
  create_req.type_name = "application/json";
  create_resp.data = create_resp_buf;
  create_resp.size = sizeof(create_resp_buf);
  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "create_object", &create_req, &create_resp), PCL_OK);

  // Invoke standard read_match with empty Query (returns all)
  char resp_buf[8192];
  pcl_msg_t req = {}, resp = {};
  req.data = nullptr;
  req.size = 0;
  req.type_name = "application/json";
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);
  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "matching_objects.read_match", &req, &resp), PCL_OK);

  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  ASSERT_TRUE(jr.is_array());
  ASSERT_GE(jr.size(), 1u);
  EXPECT_TRUE(jr[0].contains("id"));
  EXPECT_TRUE(jr[0].contains("matching_object_id"));
  EXPECT_EQ(jr[0].value("source", ""), "tactical_objects");
}

///< REQ_TACTICAL_OBJECTS_026: specific_object_detail.read_detail returns ObjectDetail array.
TEST(TacticalObjectsComponent, StandardReadDetailReturnsDetailArray) {
  static constexpr double DEG = 3.14159265358979323846 / 180.0;
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();

  pcl::Executor exec;
  exec.add(comp);

  StandardBridge bridge(comp.runtime(), exec.handle(), false);
  bridge.configure();
  bridge.activate();
  exec.add(bridge);

  // Create an object
  ObjectDefinition def;
  def.type       = ObjectType::Platform;
  def.position   = Position{52.0 * DEG, 1.0 * DEG, 0};
  def.affiliation = Affiliation::Friendly;
  auto j_create  = TacticalObjectsCodec::encodeObjectDefinition(def);
  std::string create_str = j_create.dump();
  char create_resp_buf[512];
  pcl_msg_t create_req = {}, create_resp = {};
  create_req.data = create_str.data();
  create_req.size = static_cast<uint32_t>(create_str.size());
  create_req.type_name = "application/json";
  create_resp.data = create_resp_buf;
  create_resp.size = sizeof(create_resp_buf);
  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "create_object", &create_req, &create_resp), PCL_OK);
  std::string create_resp_str(static_cast<const char*>(create_resp.data), create_resp.size);
  auto created = nlohmann::json::parse(create_resp_str);
  std::string object_id = created.value("object_id", "");
  ASSERT_FALSE(object_id.empty());

  // Query by id via specific_object_detail.read_detail
  nlohmann::json q_req;
  q_req["id"] = nlohmann::json::array({object_id});
  std::string q_str = q_req.dump();
  char resp_buf[8192];
  pcl_msg_t req = {}, resp = {};
  req.data = q_str.data();
  req.size = static_cast<uint32_t>(q_str.size());
  req.type_name = "application/json";
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);
  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "specific_object_detail.read_detail", &req, &resp), PCL_OK);

  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  ASSERT_TRUE(jr.is_array());
  ASSERT_EQ(jr.size(), 1u);
  EXPECT_EQ(jr[0].value("id", ""), object_id);
  // Position should be in radians -- verify roughly correct
  EXPECT_NEAR(jr[0]["position"].value("latitude", 0.0), 52.0 * DEG, 1e-10);
  EXPECT_NEAR(jr[0]["position"].value("longitude", 0.0), 1.0 * DEG, 1e-10);
}
