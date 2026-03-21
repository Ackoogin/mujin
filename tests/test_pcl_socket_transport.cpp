/// \file test_pcl_socket_transport.cpp
/// \brief Tests for PCL TCP socket transport — server/client, wire protocol,
///        gateway container, non-blocking send, and async remote service invocation.
///
/// Covers LLRs REQ_PCL_115–REQ_PCL_130 (tracing to HLRs PCL.031–PCL.036, PCL.045).
///
/// Each test stands up a loopback server+client pair on an ephemeral port,
/// exercises the transport layer, then tears down cleanly.

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
#include "pcl/pcl_transport_socket.h"
#include "pcl/pcl_log.h"
}

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Silence logs during tests so stderr is clean.
static void silence_logs() {
  pcl_log_set_handler([](pcl_log_level_t, const char*, const char*, void*) {},
                      nullptr);
}

static void restore_logs() {
  pcl_log_set_handler(nullptr, nullptr);
  pcl_log_set_level(PCL_LOG_INFO);
}

/// Helper struct for a server+client loopback pair.
/// server_create blocks until a client connects, so we launch server creation
/// in a background thread and connect the client from the main thread.
struct LoopbackPair {
  pcl_executor_t*          server_exec  = nullptr;
  pcl_executor_t*          client_exec  = nullptr;
  pcl_socket_transport_t*  server_transport = nullptr;
  pcl_socket_transport_t*  client_transport = nullptr;
  uint16_t                 port = 0;

  /// Build a loopback pair.  Returns true on success.
  bool create() {
    server_exec = pcl_executor_create();
    client_exec = pcl_executor_create();
    if (!server_exec || !client_exec) return false;

    // We need an ephemeral port.  Bind a temporary socket to pick a port,
    // then close it and pass the port to both sides.
    // Alternatively, create_server(0) picks an ephemeral port, but we
    // can't read it back from the opaque handle.  Instead we'll use a
    // small known port range — or just use a fixed high port and retry.
    //
    // Simplest approach: start server on port 0 in a thread.  But we need
    // the port before the client can connect.  The server blocks on accept,
    // so we need to know the port it chose.
    //
    // Workaround: bind a temporary socket to port 0, read the port,
    // close it, then immediately reuse that port for the server.
#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    // Pick an ephemeral port by binding temporarily.
    {
#ifdef _WIN32
      SOCKET tmp = socket(AF_INET, SOCK_STREAM, 0);
      if (tmp == INVALID_SOCKET) return false;
#else
      int tmp = socket(AF_INET, SOCK_STREAM, 0);
      if (tmp < 0) return false;
#endif
      struct sockaddr_in addr = {};
      addr.sin_family      = AF_INET;
      addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
      addr.sin_port        = 0;

      if (bind(tmp, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
#ifdef _WIN32
        closesocket(tmp);
#else
        close(tmp);
#endif
        return false;
      }

#ifdef _WIN32
      int len = (int)sizeof(addr);
#else
      socklen_t len = sizeof(addr);
#endif
      getsockname(tmp, (struct sockaddr*)&addr, &len);
      port = ntohs(addr.sin_port);
#ifdef _WIN32
      closesocket(tmp);
#else
      close(tmp);
#endif
    }

    // Launch server in a background thread (blocks on accept).
    std::thread server_thread([this]() {
      server_transport = pcl_socket_transport_create_server(port, server_exec);
    });

    // Small delay to let server start listening before we connect.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Connect client.
    client_transport = pcl_socket_transport_create_client("127.0.0.1", port,
                                                          client_exec);

    server_thread.join();

    return server_transport != nullptr && client_transport != nullptr;
  }

  void destroy() {
    if (client_transport) {
      pcl_socket_transport_destroy(client_transport);
      client_transport = nullptr;
    }
    if (server_transport) {
      pcl_socket_transport_destroy(server_transport);
      server_transport = nullptr;
    }
    if (client_exec) {
      pcl_executor_destroy(client_exec);
      client_exec = nullptr;
    }
    if (server_exec) {
      pcl_executor_destroy(server_exec);
      server_exec = nullptr;
    }
  }
};

// ═══════════════════════════════════════════════════════════════════════════
// REQ_PCL_115–116 — Server Mode (PCL.031)
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_115: Server creation returns valid transport. PCL.031.
TEST(PclSocketTransport, ServerCreationAndDestroy) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create()) << "Failed to create loopback pair";

  // Server transport should be non-NULL and provide a valid transport vtable.
  EXPECT_NE(pair.server_transport, nullptr);
  const pcl_transport_t* t = pcl_socket_transport_get_transport(
      pair.server_transport);
  EXPECT_NE(t, nullptr);
  EXPECT_NE(t->publish, nullptr);
  EXPECT_NE(t->subscribe, nullptr);
  EXPECT_NE(t->shutdown, nullptr);

  pair.destroy();
  restore_logs();
}

///< REQ_PCL_116: Server null executor returns NULL. PCL.031, PCL.045.
TEST(PclSocketTransport, ServerNullExecutorReturnsNull) {
  silence_logs();
  auto* t = pcl_socket_transport_create_server(0, nullptr);
  EXPECT_EQ(t, nullptr);
  restore_logs();
}

// ═══════════════════════════════════════════════════════════════════════════
// REQ_PCL_117–119 — Client Mode (PCL.032)
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_117: Client creation returns valid transport. PCL.032.
TEST(PclSocketTransport, ClientCreationAndDestroy) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create()) << "Failed to create loopback pair";

  EXPECT_NE(pair.client_transport, nullptr);
  const pcl_transport_t* t = pcl_socket_transport_get_transport(
      pair.client_transport);
  EXPECT_NE(t, nullptr);

  pair.destroy();
  restore_logs();
}

///< REQ_PCL_118: Client null args return NULL. PCL.032, PCL.045.
TEST(PclSocketTransport, ClientNullArgsReturnNull) {
  silence_logs();
  auto* e = pcl_executor_create();

  EXPECT_EQ(pcl_socket_transport_create_client(nullptr, 12345, e), nullptr);
  EXPECT_EQ(pcl_socket_transport_create_client("127.0.0.1", 12345, nullptr),
            nullptr);

  pcl_executor_destroy(e);
  restore_logs();
}

///< REQ_PCL_119: Client connect to nonexistent server fails. PCL.032.
TEST(PclSocketTransport, ClientConnectToNonexistentServerFails) {
  silence_logs();
  auto* e = pcl_executor_create();

  // Port 1 is almost certainly not listening on loopback.
  auto* t = pcl_socket_transport_create_client("127.0.0.1", 1, e);
  EXPECT_EQ(t, nullptr);

  pcl_executor_destroy(e);
  restore_logs();
}

// ═══════════════════════════════════════════════════════════════════════════
// REQ_PCL_120–121 — Wire Protocol (PCL.033)
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_120: Publish server to client delivered. PCL.033.
TEST(PclSocketTransport, PublishServerToClientDelivered) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  // Wire the server transport to its executor.
  pcl_executor_set_transport(pair.server_exec,
      pcl_socket_transport_get_transport(pair.server_transport));

  // Set up a client-side subscriber.
  struct SubState {
    std::atomic<bool> received{false};
    std::string payload;
  } sub_state;

  pcl_callbacks_t sub_cbs = {};
  sub_cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_subscriber(c, "test/topic", "TestMsg",
        [](pcl_container_t*, const pcl_msg_t* msg, void* ud2) {
          auto* s = static_cast<SubState*>(ud2);
          s->payload.assign(static_cast<const char*>(msg->data), msg->size);
          s->received = true;
        }, ud);
    return PCL_OK;
  };

  auto* sub_c = pcl_container_create("client_sub", &sub_cbs, &sub_state);
  pcl_container_configure(sub_c);
  pcl_container_activate(sub_c);
  pcl_executor_add(pair.client_exec, sub_c);

  // Publish from server side via the transport vtable.
  const char* payload = "hello from server";
  pcl_msg_t msg = {};
  msg.data = payload;
  msg.size = static_cast<uint32_t>(strlen(payload));
  msg.type_name = "TestMsg";

  const pcl_transport_t* transport =
      pcl_socket_transport_get_transport(pair.server_transport);
  pcl_status_t rc = transport->publish(transport->adapter_ctx, "test/topic",
                                        &msg);
  EXPECT_EQ(rc, PCL_OK);

  // Spin the client executor to drain the ingress queue.
  // The recv_thread posts incoming messages, so give it time.
  for (int i = 0; i < 50 && !sub_state.received; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    pcl_executor_spin_once(pair.client_exec, 0);
  }

  EXPECT_TRUE(sub_state.received) << "Client did not receive the published message";
  EXPECT_EQ(sub_state.payload, "hello from server");

  pcl_executor_remove(pair.client_exec, sub_c);
  pcl_container_destroy(sub_c);
  pair.destroy();
  restore_logs();
}

// ═══════════════════════════════════════════════════════════════════════════
// REQ_PCL_122–124 — Gateway Container (PCL.034)
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_122: Gateway container exists and is configurable. PCL.034.
TEST(PclSocketTransport, GatewayContainerExists) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  pcl_container_t* gw = pcl_socket_transport_gateway_container(
      pair.server_transport);
  EXPECT_NE(gw, nullptr);
  EXPECT_STREQ(pcl_container_name(gw), "__pcl_socket_gateway");

  // Gateway should be configurable.
  EXPECT_EQ(pcl_container_configure(gw), PCL_OK);
  EXPECT_EQ(pcl_container_state(gw), PCL_STATE_CONFIGURED);
  EXPECT_EQ(pcl_container_activate(gw), PCL_OK);
  EXPECT_EQ(pcl_container_state(gw), PCL_STATE_ACTIVE);

  pair.destroy();
  restore_logs();
}

///< REQ_PCL_123: Gateway null transport returns NULL. PCL.034, PCL.045.
TEST(PclSocketTransport, GatewayContainerNullReturnsNull) {
  EXPECT_EQ(pcl_socket_transport_gateway_container(nullptr), nullptr);
}

///< REQ_PCL_124: Client has no gateway. PCL.034.
TEST(PclSocketTransport, ClientHasNoGateway) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  // Client transport should return NULL for gateway.
  pcl_container_t* gw = pcl_socket_transport_gateway_container(
      pair.client_transport);
  EXPECT_EQ(gw, nullptr);

  pair.destroy();
  restore_logs();
}

// ═══════════════════════════════════════════════════════════════════════════
// REQ_PCL_125 — Non-Blocking Send (PCL.035)
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_125: Publish is non-blocking. PCL.035.
TEST(PclSocketTransport, PublishIsNonBlocking) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  pcl_executor_set_transport(pair.server_exec,
      pcl_socket_transport_get_transport(pair.server_transport));

  const pcl_transport_t* transport =
      pcl_socket_transport_get_transport(pair.server_transport);

  // Publish many messages rapidly; none should block.
  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 100; ++i) {
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "msg-%d", i);
    pcl_msg_t msg = {};
    msg.data = buf;
    msg.size = static_cast<uint32_t>(len);
    msg.type_name = "BurstMsg";

    pcl_status_t rc = transport->publish(transport->adapter_ctx, "burst/topic",
                                          &msg);
    EXPECT_EQ(rc, PCL_OK);
  }
  auto elapsed = std::chrono::steady_clock::now() - start;

  // 100 enqueues should complete in well under 1 second (no blocking I/O).
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(),
             1000);

  pair.destroy();
  restore_logs();
}

// ═══════════════════════════════════════════════════════════════════════════
// REQ_PCL_126–128 — Async Remote Service Invocation (PCL.036)
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_126: Async remote service round trip. PCL.036.
TEST(PclSocketTransport, AsyncRemoteServiceRoundTrip) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  // Wire transports.
  pcl_executor_set_transport(pair.server_exec,
      pcl_socket_transport_get_transport(pair.server_transport));
  pcl_executor_set_transport(pair.client_exec,
      pcl_socket_transport_get_transport(pair.client_transport));

  // Register a service on the server side.
  struct SvcState {
    bool invoked = false;
  } svc_state;

  pcl_callbacks_t svc_cbs = {};
  svc_cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_service(c, "echo_service", "EchoReq",
        [](pcl_container_t*, const pcl_msg_t* req,
           pcl_msg_t* resp, void* ud2) -> pcl_status_t {
          auto* s = static_cast<SvcState*>(ud2);
          s->invoked = true;
          // Echo the request back as response.
          resp->data = req->data;
          resp->size = req->size;
          return PCL_OK;
        }, ud);
    return PCL_OK;
  };

  auto* svc_c = pcl_container_create("echo_server", &svc_cbs, &svc_state);
  pcl_container_configure(svc_c);
  pcl_container_activate(svc_c);
  pcl_executor_add(pair.server_exec, svc_c);

  // Set up and add the gateway container.
  pcl_container_t* gw = pcl_socket_transport_gateway_container(
      pair.server_transport);
  ASSERT_NE(gw, nullptr);
  pcl_container_configure(gw);
  pcl_container_activate(gw);
  pcl_executor_add(pair.server_exec, gw);

  // Client-side: invoke the remote service.
  struct RespState {
    std::atomic<bool> received{false};
    std::string payload;
  } resp_state;

  const char* req_data = "ping";
  pcl_msg_t req = {};
  req.data = req_data;
  req.size = static_cast<uint32_t>(strlen(req_data));
  req.type_name = "EchoReq";

  pcl_status_t rc = pcl_socket_transport_invoke_remote_async(
      pair.client_transport, "echo_service", &req,
      [](const pcl_msg_t* resp, void* ud) {
        auto* s = static_cast<RespState*>(ud);
        if (resp && resp->data && resp->size > 0) {
          s->payload.assign(static_cast<const char*>(resp->data), resp->size);
        }
        s->received = true;
      },
      &resp_state);
  EXPECT_EQ(rc, PCL_OK);

  // Spin both executors until the response arrives or timeout.
  for (int i = 0; i < 100 && !resp_state.received; ++i) {
    pcl_executor_spin_once(pair.server_exec, 0);
    pcl_executor_spin_once(pair.client_exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_TRUE(svc_state.invoked) << "Server service handler was not invoked";
  EXPECT_TRUE(resp_state.received) << "Client did not receive the response";
  EXPECT_EQ(resp_state.payload, "ping");

  pcl_executor_remove(pair.server_exec, gw);
  pcl_executor_remove(pair.server_exec, svc_c);
  pcl_container_destroy(svc_c);
  pair.destroy();
  restore_logs();
}

///< REQ_PCL_127: Invoke remote async null arguments. PCL.036, PCL.045.
TEST(PclSocketTransport, InvokeRemoteAsyncNullArgs) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  pcl_msg_t req = {};
  req.data = "x";
  req.size = 1;
  req.type_name = "T";

  auto noop_cb = [](const pcl_msg_t*, void*) {};

  EXPECT_EQ(pcl_socket_transport_invoke_remote_async(
      nullptr, "svc", &req, noop_cb, nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_socket_transport_invoke_remote_async(
      pair.client_transport, nullptr, &req, noop_cb, nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_socket_transport_invoke_remote_async(
      pair.client_transport, "svc", nullptr, noop_cb, nullptr), PCL_ERR_INVALID);
  EXPECT_EQ(pcl_socket_transport_invoke_remote_async(
      pair.client_transport, "svc", &req, nullptr, nullptr), PCL_ERR_INVALID);

  pair.destroy();
  restore_logs();
}

///< REQ_PCL_128: Invoke remote async on server returns invalid. PCL.036.
TEST(PclSocketTransport, InvokeRemoteAsyncOnServerReturnsInvalid) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  pcl_msg_t req = {};
  req.data = "x";
  req.size = 1;
  req.type_name = "T";

  auto noop_cb = [](const pcl_msg_t*, void*) {};

  EXPECT_EQ(pcl_socket_transport_invoke_remote_async(
      pair.server_transport, "svc", &req, noop_cb, nullptr), PCL_ERR_INVALID);

  pair.destroy();
  restore_logs();
}

// ═══════════════════════════════════════════════════════════════════════════
// REQ_PCL_129–130 — Accessor null safety (PCL.045)
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_129: Get transport null returns NULL. PCL.045.
TEST(PclSocketTransport, GetTransportNullReturnsNull) {
  EXPECT_EQ(pcl_socket_transport_get_transport(nullptr), nullptr);
}

///< REQ_PCL_130: Destroy null is no-op. PCL.045.
TEST(PclSocketTransport, DestroyNullIsNoOp) {
  pcl_socket_transport_destroy(nullptr); // must not crash
}

// ═══════════════════════════════════════════════════════════════════════════
// REQ_PCL_121 — Wire protocol: client→server (PCL.033)
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_121: Publish client to server delivered. PCL.033.
TEST(PclSocketTransport, PublishClientToServerDelivered) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  // Wire the client transport to its executor.
  pcl_executor_set_transport(pair.client_exec,
      pcl_socket_transport_get_transport(pair.client_transport));

  // Set up a server-side subscriber.
  struct SubState {
    std::atomic<bool> received{false};
    std::string payload;
  } sub_state;

  pcl_callbacks_t sub_cbs = {};
  sub_cbs.on_configure = [](pcl_container_t* c, void* ud) -> pcl_status_t {
    pcl_container_add_subscriber(c, "uplink/data", "UplinkMsg",
        [](pcl_container_t*, const pcl_msg_t* msg, void* ud2) {
          auto* s = static_cast<SubState*>(ud2);
          s->payload.assign(static_cast<const char*>(msg->data), msg->size);
          s->received = true;
        }, ud);
    return PCL_OK;
  };

  auto* sub_c = pcl_container_create("server_sub", &sub_cbs, &sub_state);
  pcl_container_configure(sub_c);
  pcl_container_activate(sub_c);
  pcl_executor_add(pair.server_exec, sub_c);

  // Publish from client side.
  const char* payload = "uplink data";
  pcl_msg_t msg = {};
  msg.data = payload;
  msg.size = static_cast<uint32_t>(strlen(payload));
  msg.type_name = "UplinkMsg";

  const pcl_transport_t* transport =
      pcl_socket_transport_get_transport(pair.client_transport);
  pcl_status_t rc = transport->publish(transport->adapter_ctx, "uplink/data",
                                        &msg);
  EXPECT_EQ(rc, PCL_OK);

  // Spin the server executor to drain ingress.
  for (int i = 0; i < 50 && !sub_state.received; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    pcl_executor_spin_once(pair.server_exec, 0);
  }

  EXPECT_TRUE(sub_state.received) << "Server did not receive the published message";
  EXPECT_EQ(sub_state.payload, "uplink data");

  pcl_executor_remove(pair.server_exec, sub_c);
  pcl_container_destroy(sub_c);
  pair.destroy();
  restore_logs();
}

// ═══════════════════════════════════════════════════════════════════════════
// Additional coverage tests — vtable functions, edge cases, cleanup paths
// ═══════════════════════════════════════════════════════════════════════════

///< REQ_PCL_158: Subscribe vtable is callable. PCL.031.
TEST(PclSocketTransport, SubscribeVtableCallSucceeds) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  const pcl_transport_t* t =
      pcl_socket_transport_get_transport(pair.server_transport);
  ASSERT_NE(t, nullptr);
  ASSERT_NE(t->subscribe, nullptr);

  // The subscribe vtable function is a no-op stub that always returns OK.
  pcl_status_t rc = t->subscribe(t->adapter_ctx, "some/topic", "SomeType");
  EXPECT_EQ(rc, PCL_OK);

  pair.destroy();
  restore_logs();
}

///< REQ_PCL_159: Shutdown vtable sets stop flag. PCL.031.
TEST(PclSocketTransport, ShutdownVtableCallSucceeds) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  const pcl_transport_t* t =
      pcl_socket_transport_get_transport(pair.server_transport);
  ASSERT_NE(t, nullptr);
  ASSERT_NE(t->shutdown, nullptr);

  // Calling shutdown via vtable sets recv_stop flag.
  t->shutdown(t->adapter_ctx);

  pair.destroy();
  restore_logs();
}

///< REQ_PCL_160: Destroy with pending async calls frees resources. PCL.036.
TEST(PclSocketTransport, DestroyWithPendingAsyncCallNoLeak) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  pcl_executor_set_transport(pair.client_exec,
      pcl_socket_transport_get_transport(pair.client_transport));

  // Invoke a remote service but DO NOT spin either executor — the response
  // will never arrive, leaving a pending record.
  auto noop_cb = [](const pcl_msg_t*, void*) {};
  pcl_msg_t req = {};
  const char* data = "orphan";
  req.data = data;
  req.size = static_cast<uint32_t>(strlen(data));
  req.type_name = "OrphanReq";

  pcl_status_t rc = pcl_socket_transport_invoke_remote_async(
      pair.client_transport, "no_such_service", &req, noop_cb, nullptr);
  EXPECT_EQ(rc, PCL_OK);

  // Destroy while the pending record is still queued — exercises the
  // pending-list drain in pcl_socket_transport_destroy.
  pair.destroy();
  restore_logs();
}

///< REQ_PCL_161: Destroy with unsent frames frees resources. PCL.035.
TEST(PclSocketTransport, DestroyWithUnsentFramesNoLeak) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  pcl_executor_set_transport(pair.server_exec,
      pcl_socket_transport_get_transport(pair.server_transport));

  const pcl_transport_t* transport =
      pcl_socket_transport_get_transport(pair.server_transport);

  // Burst-publish many messages so the send queue builds up, then immediately
  // destroy the transport without draining — exercises the frame-drain loop.
  for (int i = 0; i < 200; ++i) {
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "burst-%d", i);
    pcl_msg_t msg = {};
    msg.data = buf;
    msg.size = static_cast<uint32_t>(len);
    msg.type_name = "BurstMsg";
    transport->publish(transport->adapter_ctx, "drain/topic", &msg);
  }

  // Immediately destroy — unsent frames must be freed.
  pair.destroy();
  restore_logs();
}

///< REQ_PCL_162: Oversized payload returns NOMEM. PCL.036.
TEST(PclSocketTransport, InvokeRemoteAsyncOversizedPayloadReturnsNomem) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  // payload_size = 1 + 4 + 2 + svc_len + 4 + req_len.
  // svc_len is uint16_t, so we need svc_len <= 65535.
  // With req_len=1: payload = 11 + svc_len.  Need > 65536, so svc_len >= 65526.
  // Use exactly 65526 chars: payload = 11 + 65526 = 65537 > 65536.
  std::string huge_name(65526, 'A');

  auto noop_cb = [](const pcl_msg_t*, void*) {};
  pcl_msg_t req = {};
  req.data = "x";
  req.size = 1;
  req.type_name = "T";

  pcl_status_t rc = pcl_socket_transport_invoke_remote_async(
      pair.client_transport, huge_name.c_str(), &req, noop_cb, nullptr);
  EXPECT_EQ(rc, PCL_ERR_NOMEM);

  pair.destroy();
  restore_logs();
}

// Note: bind-failure test removed — SO_REUSEADDR on Windows allows re-binding
// to an in-use port, making it unreliable as a unit test trigger.

///< REQ_PCL_163: Gateway service dispatch with no match. PCL.034, PCL.036.
TEST(PclSocketTransport, GatewayServiceDispatchNoMatch) {
  silence_logs();
  LoopbackPair pair;
  ASSERT_TRUE(pair.create());

  // Wire transports.
  pcl_executor_set_transport(pair.server_exec,
      pcl_socket_transport_get_transport(pair.server_transport));
  pcl_executor_set_transport(pair.client_exec,
      pcl_socket_transport_get_transport(pair.client_transport));

  // Set up gateway on server — but do NOT register any service handler.
  pcl_container_t* gw = pcl_socket_transport_gateway_container(
      pair.server_transport);
  ASSERT_NE(gw, nullptr);
  pcl_container_configure(gw);
  pcl_container_activate(gw);
  pcl_executor_add(pair.server_exec, gw);

  // Client invokes a service that doesn't exist on the server.
  std::atomic<bool> resp_received{false};
  auto resp_cb = [](const pcl_msg_t*, void* ud) {
    static_cast<std::atomic<bool>*>(ud)->store(true);
  };

  pcl_msg_t req = {};
  const char* data = "test";
  req.data = data;
  req.size = static_cast<uint32_t>(strlen(data));
  req.type_name = "NoSuchReq";

  pcl_socket_transport_invoke_remote_async(
      pair.client_transport, "nonexistent_service", &req,
      resp_cb, &resp_received);

  // Spin both executors — the gateway should process the request and
  // send back an empty response (since invoke_service fails).
  for (int i = 0; i < 50 && !resp_received; ++i) {
    pcl_executor_spin_once(pair.server_exec, 0);
    pcl_executor_spin_once(pair.client_exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_TRUE(resp_received) << "Response callback was not invoked";

  pcl_executor_remove(pair.server_exec, gw);
  pair.destroy();
  restore_logs();
}
