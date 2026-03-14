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

using namespace tactical_objects;
using namespace pyramid::core::uuid;

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
TEST(TacticalObjectsE2E, DISABLED_AdaClientZoneInterestReceivesEntityEvidence) {
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
      {"min_lat", 50.0}, {"max_lat", 52.0}, {"min_lon", -1.0}, {"max_lon", 1.0}});
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
  def.position   = Position{51.0, 0.0, 0};
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
