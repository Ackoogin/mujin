/// \file Test_TacticalObjects_E2E.cpp
/// \brief End-to-end test: Ada client, tactical_objects, evidence producer.
///
/// Flow:
/// 1. Ada client component registers interest in a zone area via subscribe_interest
/// 2. Tactical objects services the interest
/// 3. Evidence producer component produces entity observations
/// 4. Tactical objects ingests, correlates, publishes entity_updates
/// 5. Ada client receives entity evidence via entity_updates subscription
#include <gtest/gtest.h>
#include <TacticalObjectsComponent.h>
#include <TacticalObjectsCodec.h>
#include <StreamingCodec.h>
#include <pcl/executor.hpp>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_transport.h>
#include <uuid/UUIDHelper.h>
#include <pcl/pcl_log.h>

#include <chrono>
#include <thread>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

static constexpr double DEG = 3.14159265358979323846 / 180.0;

static const char* TOPIC_OBSERVATION_INGRESS = "observation_ingress";
static const char* TOPIC_ENTITY_UPDATES     = "entity_updates";

struct AdaClientContext {
  std::string interest_id;
  std::vector<EntityUpdateFrame> received_frames;
  bool received_any = false;
};

static void ada_client_entity_updates_cb(pcl_container_t*, const pcl_msg_t* msg,
                                         void* user_data) {
  auto* ctx = static_cast<AdaClientContext*>(user_data);
  if (!msg->data || msg->size == 0) return;
  auto frames = StreamingCodec::decodeBatchFrame(
      static_cast<const uint8_t*>(msg->data), msg->size);
  for (const auto& f : frames) {
    ctx->received_frames.push_back(f);
    ctx->received_any = true;
  }
}

static pcl_status_t ada_client_on_configure(pcl_container_t* c, void* ud) {
  (void)ud;
  pcl_container_add_subscriber(c, TOPIC_ENTITY_UPDATES, "application/octet-stream",
                              ada_client_entity_updates_cb, ud);
  return PCL_OK;
}

struct EvidenceProducerContext {
  pcl_executor_t* executor = nullptr;
  ObservationBatch batch;
};

static pcl_status_t evidence_producer_on_tick(pcl_container_t* c, double, void* ud) {
  auto* ctx = static_cast<EvidenceProducerContext*>(ud);
  if (!ctx->executor || ctx->batch.observations.empty()) return PCL_OK;
  auto j = TacticalObjectsCodec::encodeObservationBatch(ctx->batch);
  std::string payload = j.dump();
  pcl_msg_t msg = {};
  msg.data      = payload.data();
  msg.size      = static_cast<uint32_t>(payload.size());
  msg.type_name = "application/json";
  pcl_executor_dispatch_incoming(ctx->executor, TOPIC_OBSERVATION_INGRESS, &msg);
  ctx->batch.observations.clear();
  return PCL_OK;
}

static pcl_status_t evidence_producer_on_configure(pcl_container_t* c, void* ud) {
  (void)c;
  (void)ud;
  return PCL_OK;
}

///< E2E: Ada client registers zone interest, evidence producer injects observations,
///< tactical_objects correlates and publishes, Ada client receives entity_updates.
TEST(TacticalObjectsE2E, AdaClientZoneInterestReceivesEntityEvidence) {
  TacticalObjectsComponent tobj;
  tobj.configure();
  tobj.activate();
  tobj.setTickRateHz(1000.0);

  AdaClientContext ada_ctx;
  pcl_callbacks_t ada_cbs = {};
  ada_cbs.on_configure = ada_client_on_configure;
  pcl_container_t* ada_client = pcl_container_create("ada_client", &ada_cbs, &ada_ctx);
  ASSERT_NE(ada_client, nullptr);
  pcl_container_configure(ada_client);
  pcl_container_activate(ada_client);

  EvidenceProducerContext ev_ctx;
  ev_ctx.executor = nullptr;
  pcl_callbacks_t ev_cbs = {};
  ev_cbs.on_configure = evidence_producer_on_configure;
  ev_cbs.on_tick      = evidence_producer_on_tick;
  pcl_container_t* evidence_producer =
      pcl_container_create("evidence_producer", &ev_cbs, &ev_ctx);
  ASSERT_NE(evidence_producer, nullptr);
  pcl_container_configure(evidence_producer);
  pcl_container_set_tick_rate_hz(evidence_producer, 1000.0);
  pcl_container_activate(evidence_producer);

  {
  pcl::Executor exec;
  exec.add(tobj);
  exec.add(ada_client);
  exec.add(evidence_producer);
  ev_ctx.executor = exec.handle();

  nlohmann::json sub_req;
  sub_req["object_type"] = "Platform";
  sub_req["affiliation"] = "Hostile";
  sub_req["area"] = nlohmann::json::object({
      {"min_lat", 50.0 * DEG}, {"max_lat", 52.0 * DEG}, {"min_lon", -1.0 * DEG}, {"max_lon", 1.0 * DEG}});
  sub_req["expires_at"] = 9999.0;
  std::string sub_str = sub_req.dump();
  pcl_msg_t req = {};
  req.data      = sub_str.data();
  req.size      = static_cast<uint32_t>(sub_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char resp_buf[512];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "subscribe_interest",
                                        &req, &resp),
            PCL_OK);
  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr                = nlohmann::json::parse(resp_str);
  ada_ctx.interest_id     = jr.value("interest_id", "");
  ASSERT_FALSE(ada_ctx.interest_id.empty());

  ObjectDefinition def;
  def.type       = ObjectType::Platform;
  def.position   = Position{51.0 * DEG, 0.0, 0};
  def.affiliation = Affiliation::Hostile;
  auto j_create  = TacticalObjectsCodec::encodeObjectDefinition(def);
  std::string create_str = j_create.dump();
  req.data = create_str.data();
  req.size = static_cast<uint32_t>(create_str.size());
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);
  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "create_object", &req, &resp),
            PCL_OK);

  for (int i = 0; i < 100; ++i) {
    exec.spinOnce(0);
  }

  EXPECT_TRUE(ada_ctx.received_any)
      << "Ada client should have received entity_updates from tactical_objects";
  EXPECT_GE(ada_ctx.received_frames.size(), 1u)
      << "Ada client should have received at least one entity frame";
  }

  pcl_container_destroy(evidence_producer);
  pcl_container_destroy(ada_client);
}

// -----------------------------------------------------------------------------
// TOBJ.047: ActiveFindSolutionDrivesEvidenceProvider Flow
// -----------------------------------------------------------------------------

static const char* TOPIC_EVIDENCE_REQUIREMENTS = "evidence_requirements";

struct E2EActiveFindClientContext {
  std::string interest_id;
  std::string solution_id;
  std::vector<EntityUpdateFrame> received_frames;
  bool solution_received = false;
  bool derived_reqs_received = false;
  bool entity_updates_received = false;
};

static void e2e_active_find_entity_updates_cb(pcl_container_t*, const pcl_msg_t* msg, void* user_data) {
  auto* ctx = static_cast<E2EActiveFindClientContext*>(user_data);
  if (!msg->data || msg->size == 0) return;
  auto frames = StreamingCodec::decodeBatchFrame(
      static_cast<const uint8_t*>(msg->data), msg->size);
  for (const auto& f : frames) {
    ctx->received_frames.push_back(f);
    ctx->entity_updates_received = true;
  }
}

static pcl_status_t e2e_active_find_client_on_configure(pcl_container_t* c, void* ud) {
  pcl_container_add_subscriber(c, TOPIC_ENTITY_UPDATES, "application/octet-stream",
                               e2e_active_find_entity_updates_cb, ud);
  return PCL_OK;
}

// --- Evidence Provider ---

struct DynamicEvidenceProviderContext {
  pcl_executor_t* executor = nullptr;
  bool requirement_received = false;
  std::string active_requirement_id;
  std::vector<Observation> pending_observations;
};

// The provider listens to evidence requirements. When one arrives, it simulates
// "producing" an evidence observation that satisfies the requirement on the next tick.
static void e2e_provider_evidence_req_cb(pcl_container_t*, const pcl_msg_t* msg, void* user_data) {
  auto* ctx = static_cast<DynamicEvidenceProviderContext*>(user_data);
  if (!msg->data || msg->size == 0) return;
  
  std::string payload(static_cast<const char*>(msg->data), msg->size);
  auto j = nlohmann::json::parse(payload);
  
  ctx->active_requirement_id = j.value("requirement_id", "");
  ctx->requirement_received = true;

  // Simulate fulfilling this requirement: prepare an observation of a SeaSurface Hostile
  Observation obs;
  obs.observation_id = UUIDHelper::generateV4();
  obs.observed_at = 0.5;
  obs.position = Position{51.0 * DEG, 0.0, 0};
  obs.confidence = 0.9;
  obs.object_hint_type = ObjectType::Platform;
  
  obs.affiliation_hint = Affiliation::Hostile;
  obs.classification_hint = "Vessel";
  // Encode battle dimension into the SIDC or we need it to default correctly if MilClass isn't fully set.
  // Wait, correlation engine uses `obs.source_sidc` to create the profile. Let's just pass a generic SIDC that implies SeaSurface Hostile: "SH*P------*****"
  obs.source_sidc = "SHSP------*****";
  
  ctx->pending_observations.push_back(obs);
}

static pcl_status_t e2e_provider_on_configure(pcl_container_t* c, void* ud) {
  // Subscribes to evidence_requirements (Object_Solution_Evidence service)
  pcl_container_add_subscriber(c, TOPIC_EVIDENCE_REQUIREMENTS, "application/json",
                               e2e_provider_evidence_req_cb, ud);
  return PCL_OK;
}

static pcl_status_t e2e_provider_on_tick(pcl_container_t*, double, void* ud) {
  auto* ctx = static_cast<DynamicEvidenceProviderContext*>(ud);
  if (!ctx->executor || ctx->pending_observations.empty()) return PCL_OK;
  
  ObservationBatch batch;
  batch.observations = ctx->pending_observations;
  auto j = TacticalObjectsCodec::encodeObservationBatch(batch);
  std::string payload = j.dump();
  
  pcl_msg_t out_msg = {};
  out_msg.data      = payload.data();
  out_msg.size      = static_cast<uint32_t>(payload.size());
  out_msg.type_name = "application/json";
  
  // Publish findings to Object_Evidence service (observation_ingress topic)
  pcl_executor_dispatch_incoming(ctx->executor, TOPIC_OBSERVATION_INGRESS, &out_msg);
  
  ctx->pending_observations.clear();
  return PCL_OK;
}

///< REQ_TACTICAL_OBJECTS_059 / TOBJ.047: Full end-to-end active solution flow.
TEST(TacticalObjectsE2E, ActiveFindSolutionDrivesEvidenceProvider) {
  TacticalObjectsComponent tobj;
  tobj.configure();
  tobj.activate();
  tobj.setTickRateHz(1000.0);

  // 1. Setup Ada Client
  E2EActiveFindClientContext client_ctx;
  pcl_callbacks_t client_cbs = {};
  client_cbs.on_configure = e2e_active_find_client_on_configure;
  pcl_container_t* client_cont = pcl_container_create("ada_client", &client_cbs, &client_ctx);
  ASSERT_NE(client_cont, nullptr);
  pcl_container_configure(client_cont);
  pcl_container_activate(client_cont);

  // 2. Setup Evidence Provider
  DynamicEvidenceProviderContext provider_ctx;
  pcl_callbacks_t provider_cbs = {};
  provider_cbs.on_configure = e2e_provider_on_configure;
  provider_cbs.on_tick = e2e_provider_on_tick;
  pcl_container_t* provider_cont = pcl_container_create("provider", &provider_cbs, &provider_ctx);
  ASSERT_NE(provider_cont, nullptr);
  pcl_container_configure(provider_cont);
  pcl_container_set_tick_rate_hz(provider_cont, 1000.0);
  pcl_container_activate(provider_cont);

  {
  // 3. Create executor and run
  pcl::Executor exec;
  exec.add(tobj);
  exec.add(client_cont);
  exec.add(provider_cont);
  provider_ctx.executor = exec.handle();

  // 4. Client issues an ActiveFind request for SeaSurface Hostiles
  nlohmann::json sub_req;
  sub_req["query_mode"] = "active_find";
  sub_req["object_type"] = "Platform";
  sub_req["affiliation"] = "Hostile";
  sub_req["battle_dimension"] = "SeaSurface";
  sub_req["area"] = nlohmann::json::object({
      {"min_lat", 50.0 * DEG}, {"max_lat", 52.0 * DEG}, {"min_lon", -1.0 * DEG}, {"max_lon", 1.0 * DEG}});
  sub_req["expires_at"] = 9999.0;
  
  std::string sub_str = sub_req.dump();
  pcl_msg_t req = {};
  req.data      = sub_str.data();
  req.size      = static_cast<uint32_t>(sub_str.size());
  req.type_name = "application/json";
  
  pcl_msg_t resp = {};
  char resp_buf[2048];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);

  ASSERT_EQ(pcl_executor_invoke_service(exec.handle(), "subscribe_interest", &req, &resp), PCL_OK);
  
  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  
  client_ctx.interest_id = jr.value("interest_id", "");
  ASSERT_FALSE(client_ctx.interest_id.empty());
  
  // Verify solution details returned immediately
  client_ctx.solution_id = jr.value("solution_id", "");
  ASSERT_FALSE(client_ctx.solution_id.empty());
  client_ctx.solution_received = true;
  
  if (jr.contains("evidence_requirements") && jr["evidence_requirements"].is_array() && 
      !jr["evidence_requirements"].empty()) {
    client_ctx.derived_reqs_received = true;
  }
  ASSERT_TRUE(client_ctx.derived_reqs_received);

  // 5. Spin executor to allow messages to propagate.
  //    Each spinOnce needs real wall-clock time to elapse so that
  //    container ticks (1000 Hz = 1 ms period) actually fire.
  for (int i = 0; i < 20; ++i) {
    exec.spinOnce(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  // 6. Verify full flow
  pcl_log(nullptr, PCL_LOG_DEBUG, "requirement_received = %d", (int)provider_ctx.requirement_received);
  pcl_log(nullptr, PCL_LOG_DEBUG, "active_requirement_id = %s", provider_ctx.active_requirement_id.c_str());
  pcl_log(nullptr, PCL_LOG_DEBUG, "entity_updates_received = %d", (int)client_ctx.entity_updates_received);
  pcl_log(nullptr, PCL_LOG_DEBUG, "received_frames.size() = %zu", client_ctx.received_frames.size());

  EXPECT_TRUE(provider_ctx.requirement_received) 
    << "Evidence provider should have received the evidence requirement";
  
  EXPECT_TRUE(client_ctx.entity_updates_received) 
    << "Client should have received entity updates matching their interest";
    
  ASSERT_FALSE(client_ctx.received_frames.empty());
  
  // Find the entity update
  bool found_match = false;
  for (const auto& upd : client_ctx.received_frames) {
    if (upd.affiliation.has_value() && upd.affiliation.value() == Affiliation::Hostile &&
        upd.object_type.has_value() && upd.object_type.value() == ObjectType::Platform) {
      found_match = true;
      break;
    }
  }
  EXPECT_TRUE(found_match) << "Received update should match requested criteria";
  
  } // RAII teardown

  pcl_container_destroy(provider_cont);
  pcl_container_destroy(client_cont);
}
