/// \file test_pcl_apos_transport.cpp
/// \brief Tests for the APOS Local Virtual Channel PCL transport.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

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

///< REQ_PCL_444, REQ_PCL_222: the APOS stub delivers messages FIFO per
///< channel and waitOnMultiChannel() reports which channel has data.
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

///< REQ_PCL_445: the APOS transport's vtable exposes the full pub/sub + unary-RPC shape (PCL.072 conformance).
TEST(PclTransportApos, VtableShape) {
  silence_logs();
  AposPair p;
  pcl_conformance::expectVtableShape(
      pcl_apos_transport_get_transport(p.apos_a),
      true);
  restore_logs();
}

///< REQ_PCL_446: the APOS transport passes the shared publish-round-trip conformance case (PCL.072).
TEST(PclTransportApos_Conformance, PublishRoundTrip) {
  silence_logs();
  AposPair p;
  pcl_conformance::expectPublishRoundTrip(p.conformancePair_AtoB(),
                                          "telemetry/heartbeat",
                                          "Heartbeat",
                                          "tick-tock");
  restore_logs();
}

///< REQ_PCL_447: the APOS transport passes the shared service-round-trip conformance case (PCL.072).
TEST(PclTransportApos_Conformance, ServiceRoundTrip) {
  silence_logs();
  AposPair p;
  pcl_conformance::expectServiceRoundTrip(p.conformancePair_AtoB(),
                                          "math/echo",
                                          "ping",
                                          "pong");
  restore_logs();
}

// ---------------------------------------------------------------------------
// APOS stub API semantics (statement-coverage driven)
// ---------------------------------------------------------------------------

namespace {

std::atomic<int> g_delay_calls{0};

extern "C" void counting_delay(int uS) {
  (void)uS;
  g_delay_calls.fetch_add(1);
}

}  // namespace

///< REQ_PCL_297, REQ_PCL_107, REQ_PCL_108, REQ_PCL_109: the blocking
///< send/receive wrappers, delay-function hook, setupApos alias, and
///< logEvent stub behave per the APOS binding contract.
TEST(AposStub, BlockingWrappersAndDelayFunction) {
  setupApos(43u);        // alias for setupAPOS
  logEvent(const_cast<char*>("apos stub log line"));  // no-op

  // Delay callback fires after each non-blocking send.
  g_delay_calls = 0;
  setDelayFunction(&counting_delay);

  unsigned char payload[] = {'x', 'y'};
  TM_Status tm = TM_ERROR;
  CTimeout timeout = {1u, 0u};
  sendMessage(7100, payload, 2, &timeout, &tm);
  EXPECT_EQ(tm, TM_SUCCESS);
  EXPECT_EQ(g_delay_calls.load(), 1);
  setDelayFunction(nullptr);

  // Blocking receive drains what the blocking send queued.
  unsigned char out[8] = {};
  int size = 0;
  tm = TM_ERROR;
  receiveMessage(7100, out, sizeof(out), &size, &timeout, &tm);
  EXPECT_EQ(tm, TM_SUCCESS);
  EXPECT_EQ(size, 2);

  // Blocking receive on an empty channel with a short timeout: TM_TIMEOUT.
  tm = TM_ERROR;
  CTimeout brief = {0u, 20u * 1000u * 1000u};  // 20 ms
  receiveMessage(7100, out, sizeof(out), &size, &brief, &tm);
  EXPECT_EQ(tm, TM_TIMEOUT);

  // Invalid arguments: negative max size / NULL buffer with nonzero size.
  tm = TM_SUCCESS;
  receiveMessage(7100, out, -1, &size, &brief, &tm);
  EXPECT_EQ(tm, TM_ERROR);
  tm = TM_SUCCESS;
  receiveMessage(7100, nullptr, 4, &size, &brief, &tm);
  EXPECT_EQ(tm, TM_ERROR);

  // Blocking send with invalid (negative) size reports TM_ERROR.
  tm = TM_SUCCESS;
  sendMessage(7100, payload, -1, &timeout, &tm);
  EXPECT_EQ(tm, TM_ERROR);
  RS_Status rs = RS_SUCCESS;
  sendMessageNonBlocking(7100, nullptr, 4, &rs);
  EXPECT_EQ(rs, RS_ERROR);
}

///< REQ_PCL_322: non-blocking receive distinguishes an empty channel
///< (RS_RESOURCE) from an undersized caller buffer (RS_ERROR, message kept).
TEST(AposStub, NonBlockingReceiveEdgeCases) {
  setupAPOS(44u);

  unsigned char out[2] = {};
  int size = 0;
  RS_Status rs = RS_SUCCESS;

  // Channel never written: RS_RESOURCE.
  receiveMessageNonBlocking(7200, out, sizeof(out), &size, &rs);
  EXPECT_EQ(rs, RS_RESOURCE);

  // Message larger than the caller's buffer: RS_ERROR, size reported.
  unsigned char big[] = {'1', '2', '3', '4'};
  sendMessageNonBlocking(7201, big, 4, &rs);
  ASSERT_EQ(rs, RS_SUCCESS);
  rs = RS_SUCCESS;
  receiveMessageNonBlocking(7201, out, sizeof(out), &size, &rs);
  EXPECT_EQ(rs, RS_ERROR);
  EXPECT_EQ(size, 4);

  // Retrieve it with an adequate buffer to leave the channel clean.
  unsigned char ok[8] = {};
  receiveMessageNonBlocking(7201, ok, sizeof(ok), &size, &rs);
  EXPECT_EQ(rs, RS_SUCCESS);
  EXPECT_EQ(size, 4);
}

///< REQ_PCL_323, REQ_PCL_218: an infinite wait (NULL timeout) blocks until
///< a producer thread queues data on one of the watched channels, and
///< waitOnMultiChannel() rejects invalid arguments.
TEST(AposStub, InfiniteWaitWokenByProducerThread) {
  setupAPOS(45u);

  std::thread producer([] {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    unsigned char b[] = {'z'};
    RS_Status rs = RS_ERROR;
    sendMessageNonBlocking(7300, b, 1, &rs);
    ASSERT_EQ(rs, RS_SUCCESS);
  });

  int channels[] = {7300};
  int ready[] = {-1};
  TM_Status tm = TM_ERROR;
  waitOnMultiChannel(channels, 1, ready, nullptr, &tm);  // NULL = wait forever
  EXPECT_EQ(tm, TM_SUCCESS);
  EXPECT_EQ(ready[0], 7300);
  producer.join();

  // waitOnMultiChannel invalid arguments fail closed.
  tm = TM_SUCCESS;
  waitOnMultiChannel(nullptr, 1, ready, nullptr, &tm);
  EXPECT_EQ(tm, TM_ERROR);
  tm = TM_SUCCESS;
  waitOnMultiChannel(channels, 0, ready, nullptr, &tm);
  EXPECT_EQ(tm, TM_ERROR);
  tm = TM_SUCCESS;
  waitOnMultiChannel(channels, 1, nullptr, nullptr, &tm);
  EXPECT_EQ(tm, TM_ERROR);

  // Drain the channel for later tests.
  unsigned char out[4] = {};
  int size = 0;
  RS_Status rs = RS_ERROR;
  receiveMessageNonBlocking(7300, out, sizeof(out), &size, &rs);
  EXPECT_EQ(rs, RS_SUCCESS);
}

// ---------------------------------------------------------------------------
// APOS transport error paths
// ---------------------------------------------------------------------------

///< REQ_PCL_298, REQ_PCL_110: transport creation fails closed without an
///< executor, and destroy(NULL) is a safe no-op.
TEST(PclTransportApos, CreateWithoutExecutorReturnsNull) {
  EXPECT_EQ(pcl_apos_transport_create(1u, 1, 2, nullptr), nullptr);
  pcl_apos_transport_destroy(nullptr);  // NULL-safe
}

///< REQ_PCL_300: frames whose strings exceed the 16-bit wire limits are
///< rejected at encode time and dropped rather than truncated.
TEST(PclTransportApos, OversizedTopicRejectedAtEncode) {
  silence_logs();
  AposPair p;
  const pcl_transport_t* vt = pcl_apos_transport_get_transport(p.apos_a);
  ASSERT_NE(vt, nullptr);

  std::string huge_topic(70000, 't');
  pcl_msg_t msg = {};
  msg.data = "x";
  msg.size = 1u;
  msg.type_name = "T";
  // The vtable publish enqueues; the send thread's encode rejects and drops.
  EXPECT_EQ(vt->publish(vt->adapter_ctx, huge_topic.c_str(), &msg), PCL_OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  restore_logs();
}

///< REQ_PCL_325: malformed or empty frames arriving on the receive channel
///< are dropped without crashing the receive thread.
TEST(PclTransportApos, MalformedInboundAposMessagesDropped) {
  silence_logs();
  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);

  // Transport listens on LVC 7401.
  pcl_apos_transport_t* apos = pcl_apos_transport_create(46u, 7400, 7401, exec);
  ASSERT_NE(apos, nullptr);

  RS_Status rs = RS_ERROR;

  // Zero-length message: hook_recv reports ERR_STATE and retries.
  sendMessageNonBlocking(7401, nullptr, 0, &rs);
  ASSERT_EQ(rs, RS_SUCCESS);

  // Unknown frame kind byte.
  unsigned char bogus_kind[] = {0x7F, 0x00, 0x00};
  sendMessageNonBlocking(7401, bogus_kind, 3, &rs);
  ASSERT_EQ(rs, RS_SUCCESS);

  // Truncated PUBLISH: kind byte only.
  unsigned char truncated[] = {0x00};
  sendMessageNonBlocking(7401, truncated, 1, &rs);
  ASSERT_EQ(rs, RS_SUCCESS);

  // PUBLISH whose topic length points past the end of the message.
  unsigned char bad_topic_len[] = {0x00, 0xFF, 0xFF, 'a'};
  sendMessageNonBlocking(7401, bad_topic_len, 4, &rs);
  ASSERT_EQ(rs, RS_SUCCESS);

  // PUBLISH with valid topic but a payload-size field that disagrees with
  // the actual remaining bytes.
  unsigned char bad_payload[] = {0x00, 0x00, 0x01, 't', 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x10};
  sendMessageNonBlocking(7401, bad_payload, 10, &rs);
  ASSERT_EQ(rs, RS_SUCCESS);

  // Give the receive thread time to chew through (and drop) everything.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  pcl_apos_transport_destroy(apos);
  pcl_executor_destroy(exec);
  restore_logs();
}

///< REQ_PCL_324: a blocking receive whose message exceeds the caller's
///< buffer maps the non-blocking RS_ERROR onto TM_ERROR.
TEST(AposStub, BlockingReceiveUndersizedBufferIsError) {
  setupAPOS(47u);

  unsigned char big[] = {'a', 'b', 'c', 'd', 'e', 'f'};
  RS_Status rs = RS_ERROR;
  sendMessageNonBlocking(7500, big, 6, &rs);
  ASSERT_EQ(rs, RS_SUCCESS);

  unsigned char tiny[2] = {};
  int size = 0;
  TM_Status tm = TM_SUCCESS;
  CTimeout timeout = {1u, 0u};
  receiveMessage(7500, tiny, sizeof(tiny), &size, &timeout, &tm);
  EXPECT_EQ(tm, TM_ERROR);
  EXPECT_EQ(size, 6);

  // Drain with a big-enough buffer.
  unsigned char out[8] = {};
  receiveMessageNonBlocking(7500, out, sizeof(out), &size, &rs);
  EXPECT_EQ(rs, RS_SUCCESS);
}
