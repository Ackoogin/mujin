/// \file test_pcl_udp_transport.cpp
/// \brief Tests for PCL UDP datagram transport -- pub/sub round-trip,
///        peer identity filtering, ephemeral port binding, null safety.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

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
