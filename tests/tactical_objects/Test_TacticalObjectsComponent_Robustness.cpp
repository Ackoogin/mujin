/// \file Test_TacticalObjectsComponent_Robustness.cpp
/// \brief Robustness and stress tests for TacticalObjectsComponent against PCL.
#include <gtest/gtest.h>
#include <TacticalObjectsComponent.h>
#include <TacticalObjectsCodec.h>
#include <pcl/executor.hpp>
#include <pcl/pcl_executor.h>
#include <uuid/UUIDHelper.h>

#include <chrono>
#include <thread>
#include <vector>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

static void invokeCreateAndAssert(pcl_executor_t* e, int i) {
  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = Position{51.0 + i * 0.01, -0.1 + i * 0.01, 0};
  def.affiliation = Affiliation::Neutral;
  auto j = TacticalObjectsCodec::encodeObjectDefinition(def);
  std::string req_str = j.dump();
  pcl_msg_t req = {};
  req.data = req_str.data();
  req.size = static_cast<uint32_t>(req_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  ASSERT_EQ(pcl_executor_invoke_service(e, "create_object", &req, &resp),
            PCL_OK);
  ASSERT_GT(resp.size, 0u);
}

///< Stress: Rapid create_object service calls.
///< TOBJ.038: Thousands of entities. TOBJ.041: Incremental updates.
///< PYR-RESP-0734, PYR-RESP-0741: Scale validation.
TEST(TacticalObjectsComponentRobustness, RapidServiceCalls) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  for (int i = 0; i < 200; ++i) {
    invokeCreateAndAssert(exec.handle(), i);
  }
  auto q = comp.runtime().query(QueryRequest());
  ASSERT_EQ(q.entries.size(), 200u);
}

///< Stress: Burst of observation ingress via dispatch.
///< TOBJ.018: Multi-source ingest. TOBJ.042: Bulk ingest.
///< PYR-RESP-0741: Capture object information at scale.
TEST(TacticalObjectsComponentRobustness, BurstObservationIngress) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  for (int i = 0; i < 100; ++i) {
    ObservationBatch batch;
    Observation obs;
    obs.observation_id = UUIDHelper::generateV4();
    obs.position = Position{51.0 + i * 0.1, -0.1, 0};
    obs.confidence = 0.7;
    obs.affiliation_hint = Affiliation::Neutral;
    obs.source_ref.source_system = "burst";
    obs.source_ref.source_entity_id = "T" + std::to_string(i);
    batch.observations.push_back(obs);

    auto j = TacticalObjectsCodec::encodeObservationBatch(batch);
    std::string payload = j.dump();
    pcl_msg_t msg = {};
    msg.data = payload.data();
    msg.size = static_cast<uint32_t>(payload.size());
    msg.type_name = "application/json";

    ASSERT_EQ(exec.dispatchIncoming("observation_ingress", &msg), PCL_OK);
  }

  auto q = comp.runtime().query(QueryRequest());
  ASSERT_GE(q.entries.size(), 50u);
}

///< Robustness: Malformed JSON on create_object returns error.
TEST(TacticalObjectsComponentRobustness, MalformedJsonRejected) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  std::string bad_json = "{invalid json";
  pcl_msg_t req = {};
  req.data = bad_json.data();
  req.size = static_cast<uint32_t>(bad_json.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char buf[256];
  resp.data = buf;
  resp.size = sizeof(buf);

  auto rc = pcl_executor_invoke_service(exec.handle(), "create_object", &req, &resp);
  ASSERT_TRUE(rc == PCL_ERR_INVALID || rc == PCL_ERR_CALLBACK);
}

///< Robustness: Empty request body on create_object.
TEST(TacticalObjectsComponentRobustness, EmptyRequestHandled) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  pcl_msg_t req = {};
  req.data = nullptr;
  req.size = 0;
  req.type_name = "application/json";
  pcl_msg_t resp = {};

  auto rc = pcl_executor_invoke_service(exec.handle(), "create_object", &req, &resp);
  ASSERT_NE(rc, PCL_OK);
}

///< Robustness: Unknown service returns NOT_FOUND.
TEST(TacticalObjectsComponentRobustness, UnknownServiceReturnsNotFound) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  std::string body = "{}";
  pcl_msg_t req = {};
  req.data = body.data();
  req.size = static_cast<uint32_t>(body.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};

  auto rc = pcl_executor_invoke_service(exec.handle(), "nonexistent_service", &req, &resp);
  ASSERT_EQ(rc, PCL_ERR_NOT_FOUND);
}

///< Robustness: Multiple configure-activate-deactivate-cleanup cycles.
TEST(TacticalObjectsComponentRobustness, RepeatedLifecycleCycles) {
  TacticalObjectsComponent comp;

  for (int cycle = 0; cycle < 5; ++cycle) {
    ASSERT_EQ(comp.configure(), PCL_OK);
    ASSERT_EQ(comp.activate(), PCL_OK);
    ASSERT_EQ(comp.state(), PCL_STATE_ACTIVE);

    auto id = comp.runtime().createObject(ObjectDefinition());
    ASSERT_FALSE(id.isNull());

    ASSERT_EQ(comp.deactivate(), PCL_OK);
    ASSERT_EQ(comp.cleanup(), PCL_OK);
    ASSERT_EQ(comp.state(), PCL_STATE_UNCONFIGURED);
  }
}

///< Stress: Mixed evidence and direct CRUD under executor.
///< TOBJ.018: Multi-source ingest. TOBJ.007: Criteria query over combined sources.
///< PYR-RESP-0734: Determine potential objects from evidence and direct.
TEST(TacticalObjectsComponentRobustness, MixedEvidenceAndDirectUnderExecutor) {
  TacticalObjectsComponent comp;
  comp.configure();
  comp.activate();
  pcl::Executor exec;
  exec.add(comp);

  for (int i = 0; i < 30; ++i) {
    invokeCreateAndAssert(exec.handle(), i);
  }

  for (int i = 0; i < 20; ++i) {
    ObservationBatch batch;
    Observation obs;
    obs.observation_id = UUIDHelper::generateV4();
    obs.position = Position{52.0 + i * 0.05, 1.0, 0};
    obs.confidence = 0.6;
    obs.affiliation_hint = Affiliation::Hostile;
    obs.source_ref.source_system = "sensor";
    obs.source_ref.source_entity_id = "O" + std::to_string(i);
    batch.observations.push_back(obs);
    auto j = TacticalObjectsCodec::encodeObservationBatch(batch);
    std::string payload = j.dump();
    pcl_msg_t msg = {};
    msg.data = payload.data();
    msg.size = static_cast<uint32_t>(payload.size());
    msg.type_name = "application/json";
    exec.dispatchIncoming("observation_ingress", &msg);
  }

  QueryRequest qreq;
  qreq.by_type = ObjectType::Platform;
  auto q = comp.runtime().query(qreq);
  ASSERT_GE(q.entries.size(), 45u);
}
