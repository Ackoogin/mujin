/// \file test_pcl_udp_transport.cpp
/// \brief Tests for PCL UDP datagram transport -- pub/sub round-trip,
///        peer identity filtering, ephemeral port binding, null safety.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  pragma comment(lib, "Ws2_32.lib")
#else
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <unistd.h>
#endif

extern "C" {
#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_transport.h"
#include "pcl/pcl_transport_udp.h"
#include "pcl/pcl_log.h"
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void silence_logs() {
  pcl_log_set_handler([](pcl_log_level_t, const char*, const char*, void*) {},
                      nullptr);
}

static void restore_logs() {
  pcl_log_set_handler(nullptr, nullptr);
  pcl_log_set_level(PCL_LOG_INFO);
}

/// Pick a free UDP loopback port by binding a temporary socket.
static uint16_t pick_free_udp_port() {
#ifdef _WIN32
  WSADATA wsa;
  WSAStartup(MAKEWORD(2, 2), &wsa);
  SOCKET tmp = socket(AF_INET, SOCK_DGRAM, 0);
  if (tmp == INVALID_SOCKET) return 0;
#else
  int tmp = socket(AF_INET, SOCK_DGRAM, 0);
  if (tmp < 0) return 0;
#endif
  sockaddr_in addr{};
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port        = 0;
  if (bind(tmp, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
#ifdef _WIN32
    closesocket(tmp);
#else
    close(tmp);
#endif
    return 0;
  }
#ifdef _WIN32
  int len = static_cast<int>(sizeof(addr));
#else
  socklen_t len = sizeof(addr);
#endif
  getsockname(tmp, reinterpret_cast<sockaddr*>(&addr), &len);
  uint16_t port = ntohs(addr.sin_port);
#ifdef _WIN32
  closesocket(tmp);
#else
  close(tmp);
#endif
  return port;
}

// ---------------------------------------------------------------------------
// Creation / destruction
// ---------------------------------------------------------------------------

TEST(PclUdpTransport, CreateAndDestroy) {
  silence_logs();
  auto* e = pcl_executor_create();
  ASSERT_NE(e, nullptr);

  auto* t = pcl_udp_transport_create(0, "127.0.0.1", 9999, e);
  ASSERT_NE(t, nullptr);

  // Ephemeral port must be non-zero after creation.
  EXPECT_NE(pcl_udp_transport_get_local_port(t), 0);

  const pcl_transport_t* vt = pcl_udp_transport_get_transport(t);
  ASSERT_NE(vt, nullptr);
  EXPECT_NE(vt->publish,   nullptr);
  EXPECT_NE(vt->subscribe, nullptr);
  EXPECT_NE(vt->shutdown,  nullptr);

  pcl_udp_transport_destroy(t);
  pcl_executor_destroy(e);
  restore_logs();
}

TEST(PclUdpTransport, NullArgsReturnNull) {
  silence_logs();
  auto* e = pcl_executor_create();

  EXPECT_EQ(pcl_udp_transport_create(0, nullptr,     9999, e),       nullptr);
  EXPECT_EQ(pcl_udp_transport_create(0, "127.0.0.1", 9999, nullptr), nullptr);
  EXPECT_EQ(pcl_udp_transport_get_local_port(nullptr), 0);
  EXPECT_EQ(pcl_udp_transport_get_transport(nullptr),  nullptr);
  EXPECT_EQ(pcl_udp_transport_set_peer_id(nullptr, "x"),
            PCL_ERR_INVALID);

  pcl_executor_destroy(e);
  restore_logs();
}

TEST(PclUdpTransport, SetPeerIdRoundTrip) {
  silence_logs();
  auto* e = pcl_executor_create();
  auto* t = pcl_udp_transport_create(0, "127.0.0.1", 9999, e);
  ASSERT_NE(t, nullptr);

  EXPECT_EQ(pcl_udp_transport_set_peer_id(t, "telemetry_peer"), PCL_OK);
  EXPECT_EQ(pcl_udp_transport_set_peer_id(t, ""),               PCL_ERR_INVALID);
  EXPECT_EQ(pcl_udp_transport_set_peer_id(t, nullptr),          PCL_ERR_INVALID);

  pcl_udp_transport_destroy(t);
  pcl_executor_destroy(e);
  restore_logs();
}

TEST(PclUdpTransport, DestroyNullIsNoOp) {
  pcl_udp_transport_destroy(nullptr);  // must not crash
  SUCCEED();
}

// ---------------------------------------------------------------------------
// Publish round-trip: sender -> receiver on loopback
// ---------------------------------------------------------------------------

TEST(PclUdpTransport, PublishDeliveredToSubscriber) {
  silence_logs();

  const uint16_t recv_port = pick_free_udp_port();
  const uint16_t send_port = pick_free_udp_port();
  ASSERT_NE(recv_port, 0);
  ASSERT_NE(send_port, 0);
  ASSERT_NE(recv_port, send_port);

  auto* recv_exec = pcl_executor_create();
  auto* send_exec = pcl_executor_create();

  // Receiver binds recv_port and points "back" at the sender so symmetry holds;
  // we only exercise one direction in this test.
  auto* recv_udp = pcl_udp_transport_create(recv_port, "127.0.0.1", send_port,
                                             recv_exec);
  auto* send_udp = pcl_udp_transport_create(send_port, "127.0.0.1", recv_port,
                                             send_exec);
  ASSERT_NE(recv_udp, nullptr);
  ASSERT_NE(send_udp, nullptr);

  pcl_udp_transport_set_peer_id(recv_udp, "sender");
  pcl_udp_transport_set_peer_id(send_udp, "receiver");

  // Register the sender transport under the peer name the subscriber allows.
  pcl_executor_register_transport(recv_exec, "sender",
      pcl_udp_transport_get_transport(recv_udp));

  struct SubState {
    std::atomic<bool> received{false};
    std::string       payload;
  } state;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    const char* peers[] = {"sender"};
    pcl_port_t* port = pcl_container_add_subscriber(
        c, "telemetry/heartbeat", "HeartbeatMsg",
        [](pcl_container_t*, const pcl_msg_t* msg, void* ud2) {
          auto* s = static_cast<SubState*>(ud2);
          s->payload.assign(static_cast<const char*>(msg->data), msg->size);
          s->received = true;
        }, ud);
    pcl_port_set_route(port, PCL_ROUTE_REMOTE, peers, 1);
    return PCL_OK;
  };

  auto* sub_c = pcl_container_create("recv_sub", &cbs, &state);
  pcl_container_configure(sub_c);
  pcl_container_activate(sub_c);
  pcl_executor_add(recv_exec, sub_c);

  // Publish via the send transport's vtable.
  const char* payload = "tick";
  pcl_msg_t msg = {};
  msg.data      = payload;
  msg.size      = static_cast<uint32_t>(strlen(payload));
  msg.type_name = "HeartbeatMsg";

  const pcl_transport_t* vt = pcl_udp_transport_get_transport(send_udp);
  // UDP is best-effort -- send a few times to paper over occasional loss.
  for (int i = 0; i < 5 && !state.received; ++i) {
    vt->publish(vt->adapter_ctx, "telemetry/heartbeat", &msg);
    for (int j = 0; j < 20 && !state.received; ++j) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      pcl_executor_spin_once(recv_exec, 0);
    }
  }

  EXPECT_TRUE(state.received) << "subscriber never received the UDP datagram";
  EXPECT_EQ(state.payload, "tick");
  EXPECT_GT(pcl_udp_transport_received_datagrams(recv_udp), 0u);
  EXPECT_EQ(pcl_udp_transport_dropped_datagrams(recv_udp), 0u);

  pcl_executor_remove(recv_exec, sub_c);
  pcl_container_destroy(sub_c);
  pcl_udp_transport_destroy(send_udp);
  pcl_udp_transport_destroy(recv_udp);
  pcl_executor_destroy(send_exec);
  pcl_executor_destroy(recv_exec);
  restore_logs();
}

// ---------------------------------------------------------------------------
// Peer identity filtering: subscriber allow-list drops foreign peers
// ---------------------------------------------------------------------------

TEST(PclUdpTransport, SubscriberPeerFilterDropsForeignIngress) {
  silence_logs();

  const uint16_t recv_port = pick_free_udp_port();
  const uint16_t send_port = pick_free_udp_port();
  ASSERT_NE(recv_port, 0);
  ASSERT_NE(send_port, 0);
  ASSERT_NE(recv_port, send_port);

  auto* recv_exec = pcl_executor_create();
  auto* send_exec = pcl_executor_create();

  auto* recv_udp = pcl_udp_transport_create(recv_port, "127.0.0.1", send_port,
                                             recv_exec);
  auto* send_udp = pcl_udp_transport_create(send_port, "127.0.0.1", recv_port,
                                             send_exec);
  ASSERT_NE(recv_udp, nullptr);
  ASSERT_NE(send_udp, nullptr);

  // The sender arrives tagged as "intruder" but the subscriber only allows
  // traffic from "trusted" -- the message must be dropped.
  pcl_udp_transport_set_peer_id(recv_udp, "intruder");

  pcl_executor_register_transport(recv_exec, "intruder",
      pcl_udp_transport_get_transport(recv_udp));

  struct SubState {
    std::atomic<bool> received{false};
  } state;

  pcl_callbacks_t cbs = {};
  cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    const char* peers[] = {"trusted"};
    pcl_port_t* port = pcl_container_add_subscriber(
        c, "sensitive/topic", "Msg",
        [](pcl_container_t*, const pcl_msg_t*, void* ud2) {
          static_cast<SubState*>(ud2)->received = true;
        }, ud);
    pcl_port_set_route(port, PCL_ROUTE_REMOTE, peers, 1);
    return PCL_OK;
  };

  auto* sub_c = pcl_container_create("filt_sub", &cbs, &state);
  pcl_container_configure(sub_c);
  pcl_container_activate(sub_c);
  pcl_executor_add(recv_exec, sub_c);

  pcl_msg_t msg = {};
  const char* p  = "nope";
  msg.data       = p;
  msg.size       = 4;
  msg.type_name  = "Msg";

  const pcl_transport_t* vt = pcl_udp_transport_get_transport(send_udp);
  for (int i = 0; i < 5; ++i) {
    vt->publish(vt->adapter_ctx, "sensitive/topic", &msg);
  }
  for (int j = 0; j < 30; ++j) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    pcl_executor_spin_once(recv_exec, 0);
  }

  EXPECT_FALSE(state.received)
      << "peer filter failed: subscriber accepted traffic from foreign peer";

  pcl_executor_remove(recv_exec, sub_c);
  pcl_container_destroy(sub_c);
  pcl_udp_transport_destroy(send_udp);
  pcl_udp_transport_destroy(recv_udp);
  pcl_executor_destroy(send_exec);
  pcl_executor_destroy(recv_exec);
  restore_logs();
}

// ---------------------------------------------------------------------------
// Size cap: oversized payloads are rejected (not silently truncated)
// ---------------------------------------------------------------------------

TEST(PclUdpTransport, OversizedPublishReturnsNomem) {
  silence_logs();
  auto* e = pcl_executor_create();
  auto* t = pcl_udp_transport_create(0, "127.0.0.1", 9999, e);
  ASSERT_NE(t, nullptr);

  std::vector<char> big(2000, 'x');
  pcl_msg_t msg = {};
  msg.data      = big.data();
  msg.size      = static_cast<uint32_t>(big.size());
  msg.type_name = "Huge";

  const pcl_transport_t* vt = pcl_udp_transport_get_transport(t);
  EXPECT_EQ(vt->publish(vt->adapter_ctx, "big/topic", &msg), PCL_ERR_NOMEM);

  pcl_udp_transport_destroy(t);
  pcl_executor_destroy(e);
  restore_logs();
}

// ---------------------------------------------------------------------------
// No invoke_async: UDP transport deliberately does not support RPC
// ---------------------------------------------------------------------------

TEST(PclUdpTransport, NoServiceRpcSupport) {
  silence_logs();
  auto* e = pcl_executor_create();
  auto* t = pcl_udp_transport_create(0, "127.0.0.1", 9999, e);
  ASSERT_NE(t, nullptr);

  const pcl_transport_t* vt = pcl_udp_transport_get_transport(t);
  EXPECT_EQ(vt->invoke_async, nullptr);
  EXPECT_EQ(vt->respond,      nullptr);
  EXPECT_EQ(vt->serve,        nullptr);
  EXPECT_EQ(vt->invoke_stream, nullptr);

  pcl_udp_transport_destroy(t);
  pcl_executor_destroy(e);
  restore_logs();
}

///< REQ_PCL_301: the UDP vtable subscribe is a no-op returning PCL_OK and
///< shutdown stops the receive thread.
TEST(PclUdpTransport, SubscribeAndShutdownVtable) {
  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);
  pcl_udp_transport_t* udp =
      pcl_udp_transport_create(0, "127.0.0.1", 18790, exec);
  ASSERT_NE(udp, nullptr);

  const pcl_transport_t* vt = pcl_udp_transport_get_transport(udp);
  ASSERT_NE(vt, nullptr);
  ASSERT_NE(vt->subscribe, nullptr);
  EXPECT_EQ(vt->subscribe(vt->adapter_ctx, "t", "T"), PCL_OK);

  ASSERT_NE(vt->shutdown, nullptr);
  vt->shutdown(vt->adapter_ctx);
  vt->shutdown(nullptr);  // NULL-safe

  const char payload[] = "after-shutdown";
  pcl_msg_t msg = {};
  msg.data = payload;
  msg.size = static_cast<uint32_t>(sizeof(payload) - 1u);
  msg.type_name = "T";
  EXPECT_EQ(vt->publish(vt->adapter_ctx, "t", &msg), PCL_ERR_STATE);

  pcl_udp_transport_destroy(udp);
  pcl_executor_destroy(exec);
}

///< REQ_PCL_302: creation fails closed (NULL) when the remote host cannot
///< be resolved.
TEST(PclUdpTransport, UnresolvableRemoteHostReturnsNull) {
  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);
  EXPECT_EQ(pcl_udp_transport_create(0, "no.such.host.invalid", 18791, exec),
            nullptr);
  pcl_executor_destroy(exec);
}

///< REQ_PCL_303, REQ_PCL_504: malformed and unsupported datagrams are dropped
///< without wedging the receiver, but every datagram is counted. PCL.084.
TEST(PclUdpTransport, MalformedDatagramsIncreaseReceivedCounter) {
  silence_logs();
  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);

  pcl_udp_transport_t* udp =
      pcl_udp_transport_create(0, "127.0.0.1", 18792, exec);
  ASSERT_NE(udp, nullptr);
  const uint16_t port = pcl_udp_transport_get_local_port(udp);
  ASSERT_NE(port, 0);

#ifdef _WIN32
  SOCKET raw = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_NE(raw, INVALID_SOCKET);
#else
  int raw = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(raw, 0);
#endif
  sockaddr_in to{};
  to.sin_family      = AF_INET;
  to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  to.sin_port        = htons(port);

  auto send_raw = [&](const uint8_t* data, size_t n) {
    sendto(raw, reinterpret_cast<const char*>(data), (int)n, 0,
           reinterpret_cast<sockaddr*>(&to), sizeof(to));
  };

  // Truncated fixed header.
  const uint8_t truncated_header[] = {0x00};
  send_raw(truncated_header, sizeof(truncated_header));

  // Complete fixed header with an unknown message type byte.
  const uint8_t unknown_type[] = {
      0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_raw(unknown_type, sizeof(unknown_type));

  // PUBLISH with a topic length that exceeds the datagram.
  const uint8_t truncated_topic[] = {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 'a', 'b'};
  send_raw(truncated_topic, sizeof(truncated_topic));

  // PUBLISH with a valid topic but truncated type-name length field.
  // [type=0][sequence=0][topic_len=1]['t'][type_len=0x0040]
  const uint8_t truncated_type[] = {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 't', 0x00, 0x40};
  send_raw(truncated_type, sizeof(truncated_type));

  // PUBLISH with valid topic and type but data_len larger than the datagram.
  // [type=0][sequence=0][topic_len=1]['t'][type_len=1]['T'][data_len=0xFFFF]
  const uint8_t truncated_data[] = {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 't',
      0x00, 0x01, 'T', 0x00, 0x00, 0xFF, 0xFF};
  send_raw(truncated_data, sizeof(truncated_data));

  for (int i = 0;
       i < 100 && pcl_udp_transport_received_datagrams(udp) < 5u;
       ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(pcl_udp_transport_received_datagrams(udp), 5u);
  EXPECT_EQ(pcl_udp_transport_dropped_datagrams(udp), 0u);
  EXPECT_EQ(pcl_udp_transport_received_datagrams(nullptr), 0u);
  EXPECT_EQ(pcl_udp_transport_dropped_datagrams(nullptr), 0u);

  // The transport is still alive and well-formed traffic still works: the
  // destroy path below would deadlock or crash on a wedged receive thread.
#ifdef _WIN32
  closesocket(raw);
#else
  close(raw);
#endif
  pcl_udp_transport_destroy(udp);
  pcl_executor_destroy(exec);
  restore_logs();
}

///< REQ_PCL_505: UDP forward sequence gaps are counted per source while
///< duplicate, reordered, and stale values do not increase the count. PCL.084.
TEST(PclUdpTransport, SequenceGapsAreCountedPerSource) {
  silence_logs();
  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);
  pcl_udp_transport_t* udp =
      pcl_udp_transport_create(0, "127.0.0.1", 18796, exec);
  ASSERT_NE(udp, nullptr);
  const uint16_t port = pcl_udp_transport_get_local_port(udp);
  ASSERT_NE(port, 0);

#ifdef _WIN32
  SOCKET raw_a = socket(AF_INET, SOCK_DGRAM, 0);
  SOCKET raw_b = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_NE(raw_a, INVALID_SOCKET);
  ASSERT_NE(raw_b, INVALID_SOCKET);
#else
  int raw_a = socket(AF_INET, SOCK_DGRAM, 0);
  int raw_b = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(raw_a, 0);
  ASSERT_GE(raw_b, 0);
#endif
  sockaddr_in to{};
  to.sin_family = AF_INET;
  to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  to.sin_port = htons(port);

  auto packet = [](uint32_t sequence) {
    std::vector<uint8_t> bytes;
    auto append_u16 = [&](uint16_t value) {
      bytes.push_back(static_cast<uint8_t>(value >> 8));
      bytes.push_back(static_cast<uint8_t>(value));
    };
    auto append_u32 = [&](uint32_t value) {
      bytes.push_back(static_cast<uint8_t>(value >> 24));
      bytes.push_back(static_cast<uint8_t>(value >> 16));
      bytes.push_back(static_cast<uint8_t>(value >> 8));
      bytes.push_back(static_cast<uint8_t>(value));
    };
    bytes.push_back(0x00);
    append_u32(sequence);
    append_u16(1u);
    bytes.push_back('t');
    append_u16(1u);
    bytes.push_back('T');
    append_u32(1u);
    bytes.push_back('x');
    return bytes;
  };
  auto send_sequence = [&](auto raw, uint32_t sequence) {
    const std::vector<uint8_t> bytes = packet(sequence);
    return sendto(raw, reinterpret_cast<const char*>(bytes.data()),
                  static_cast<int>(bytes.size()), 0,
                  reinterpret_cast<sockaddr*>(&to), sizeof(to));
  };

  EXPECT_GT(send_sequence(raw_a, 10u), 0);
  EXPECT_GT(send_sequence(raw_a, 13u), 0);  // two inferred drops
  EXPECT_GT(send_sequence(raw_a, 12u), 0);  // stale/reordered
  EXPECT_GT(send_sequence(raw_a, 13u), 0);  // duplicate
  EXPECT_GT(send_sequence(raw_a, 15u), 0);  // one inferred drop
  EXPECT_GT(send_sequence(raw_b, 100u), 0); // independent baseline
  EXPECT_GT(send_sequence(raw_b, 102u), 0); // one inferred drop

  for (int i = 0;
       i < 100 && pcl_udp_transport_received_datagrams(udp) < 7u;
       ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(pcl_udp_transport_received_datagrams(udp), 7u);
  EXPECT_EQ(pcl_udp_transport_dropped_datagrams(udp), 4u);

#ifdef _WIN32
  closesocket(raw_a);
  closesocket(raw_b);
#else
  close(raw_a);
  close(raw_b);
#endif
  pcl_udp_transport_destroy(udp);
  pcl_executor_destroy(exec);
  restore_logs();
}

///< REQ_PCL_304: destroying a UDP transport installed as the executor's
///< default transport clears the default slot.
TEST(PclUdpTransport, DestroyClearsDefaultTransportWhenSelf) {
  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);
  pcl_udp_transport_t* udp =
      pcl_udp_transport_create(0, "127.0.0.1", 18793, exec);
  ASSERT_NE(udp, nullptr);

  ASSERT_EQ(pcl_executor_set_transport(exec,
                                       pcl_udp_transport_get_transport(udp)),
            PCL_OK);
  pcl_udp_transport_destroy(udp);
  EXPECT_EQ(pcl_executor_get_transport(exec), nullptr);
  pcl_executor_destroy(exec);
}
