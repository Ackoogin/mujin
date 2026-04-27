/// \file test_pcl_apos_transport.cpp
/// \brief Tests for the APOS Local Virtual Channel PCL transport.
#include <gtest/gtest.h>

#include <string>

extern "C" {
#include "apos.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_log.h"
#include "pcl/pcl_transport_apos.h"
}

#include "pcl_transport_conformance.hpp"

namespace {

void silence_logs() {
  pcl_log_set_handler([](pcl_log_level_t, const char*, const char*, void*) {},
                      nullptr);
}

void restore_logs() {
  pcl_log_set_handler(nullptr, nullptr);
  pcl_log_set_level(PCL_LOG_INFO);
}

struct AposPair {
  pcl_executor_t*      exec_a = nullptr;
  pcl_executor_t*      exec_b = nullptr;
  pcl_apos_transport_t* apos_a = nullptr;
  pcl_apos_transport_t* apos_b = nullptr;

  AposPair() {
    exec_a = pcl_executor_create();
    exec_b = pcl_executor_create();

    apos_a = pcl_apos_transport_create(1001u, 502, 501, exec_a);
    apos_b = pcl_apos_transport_create(1002u, 501, 502, exec_b);

    pcl_apos_transport_set_peer_id(apos_a, "node_b");
    pcl_apos_transport_set_peer_id(apos_b, "node_a");

    pcl_executor_set_transport(exec_a, pcl_apos_transport_get_transport(apos_a));
    pcl_executor_set_transport(exec_b, pcl_apos_transport_get_transport(apos_b));
    pcl_executor_register_transport(exec_a, "node_b",
        pcl_apos_transport_get_transport(apos_a));
    pcl_executor_register_transport(exec_b, "node_a",
        pcl_apos_transport_get_transport(apos_b));
  }

  ~AposPair() {
    if (exec_a) {
      pcl_executor_set_transport(exec_a, nullptr);
      pcl_executor_register_transport(exec_a, "node_b", nullptr);
    }
    if (exec_b) {
      pcl_executor_set_transport(exec_b, nullptr);
      pcl_executor_register_transport(exec_b, "node_a", nullptr);
    }
    if (apos_a) pcl_apos_transport_destroy(apos_a);
    if (apos_b) pcl_apos_transport_destroy(apos_b);
    if (exec_a) pcl_executor_destroy(exec_a);
    if (exec_b) pcl_executor_destroy(exec_b);
  }

  pcl_conformance::TransportPair conformancePair_AtoB() const {
    pcl_conformance::TransportPair p;
    p.sender_exec = exec_a;
    p.receiver_exec = exec_b;
    p.sender_vtable = pcl_apos_transport_get_transport(apos_a);
    p.receiver_vtable = pcl_apos_transport_get_transport(apos_b);
    p.sender_peer_id = "node_a";
    p.receiver_peer_id = "node_b";
    return p;
  }
};

}  // namespace

TEST(AposStub, FifoAndWaitOnMultiChannel) {
  setupAPOS(42u);

  unsigned char first[] = { 'a', 'b', 'c' };
  unsigned char second[] = { 'd', 'e' };
  RS_Status rs = RS_ERROR;
  sendMessageNonBlocking(7001, first, 3, &rs);
  ASSERT_EQ(rs, RS_SUCCESS);
  sendMessageNonBlocking(7001, second, 2, &rs);
  ASSERT_EQ(rs, RS_SUCCESS);

  int channels[] = { 7000, 7001 };
  int ready[] = { -1, -1 };
  CTimeout zero = { 0u, 0u };
  TM_Status tm = TM_ERROR;
  waitOnMultiChannel(channels, 2, ready, &zero, &tm);
  ASSERT_EQ(tm, TM_SUCCESS);
  EXPECT_EQ(ready[0], 7001);

  unsigned char out[8] = {};
  int size = 0;
  receiveMessageNonBlocking(7001, out, sizeof(out), &size, &rs);
  ASSERT_EQ(rs, RS_SUCCESS);
  EXPECT_EQ(size, 3);
  EXPECT_EQ(std::string(reinterpret_cast<char*>(out), size), "abc");

  receiveMessageNonBlocking(7001, out, sizeof(out), &size, &rs);
  ASSERT_EQ(rs, RS_SUCCESS);
  EXPECT_EQ(size, 2);
  EXPECT_EQ(std::string(reinterpret_cast<char*>(out), size), "de");
}

TEST(PclTransportApos, VtableShape) {
  silence_logs();
  AposPair p;
  pcl_conformance::expectVtableShape(
      pcl_apos_transport_get_transport(p.apos_a),
      true);
  restore_logs();
}

TEST(PclTransportApos_Conformance, PublishRoundTrip) {
  silence_logs();
  AposPair p;
  pcl_conformance::expectPublishRoundTrip(p.conformancePair_AtoB(),
                                          "telemetry/heartbeat",
                                          "Heartbeat",
                                          "tick-tock");
  restore_logs();
}

TEST(PclTransportApos_Conformance, ServiceRoundTrip) {
  silence_logs();
  AposPair p;
  pcl_conformance::expectServiceRoundTrip(p.conformancePair_AtoB(),
                                          "math/echo",
                                          "ping",
                                          "pong");
  restore_logs();
}
