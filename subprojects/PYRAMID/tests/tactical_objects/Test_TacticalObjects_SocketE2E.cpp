/// \file Test_TacticalObjects_SocketE2E.cpp
/// \brief Socket transport E2E test: C++ client <-> C++ server over TCP.
///
/// Validates the full socket transport round-trip without requiring Ada:
/// 1. Server thread: TacticalObjectsComponent with socket server transport
/// 2. Main thread (client): subscribes to entity_updates, invokes
///    subscribe_interest and create_object via remote service calls,
///    receives entity update frames.
#include <gtest/gtest.h>
#include <TacticalObjectsComponent.h>
#include <TacticalObjectsCodec.h>
#include <StreamingCodec.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_transport.h>
#include <pcl/pcl_transport_socket.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace tactical_objects;

namespace {

static const char* TOPIC_ENTITY_UPDATES = "entity_updates";

struct ServerState {
  std::atomic<uint16_t> port{0};
  std::atomic<bool> ready{false};
  std::atomic<bool> done{false};
  std::atomic<bool> failed{false};
};

struct ClientRecvContext {
  std::mutex mu;
  std::vector<EntityUpdateFrame> received_frames;
  std::atomic<bool> received_any{false};
};

static void client_entity_updates_cb(pcl_container_t*, const pcl_msg_t* msg,
                                     void* user_data) {
  auto* ctx = static_cast<ClientRecvContext*>(user_data);
  if (!msg || !msg->data || msg->size == 0) return;
  auto frames = StreamingCodec::decodeBatchFrame(
      static_cast<const uint8_t*>(msg->data), msg->size);
  std::lock_guard<std::mutex> lock(ctx->mu);
  for (const auto& f : frames) {
    ctx->received_frames.push_back(f);
  }
  if (!frames.empty()) {
    ctx->received_any.store(true);
  }
}

static pcl_status_t client_on_configure(pcl_container_t* c, void* ud) {
  pcl_container_add_subscriber(c, TOPIC_ENTITY_UPDATES,
                               "application/octet-stream",
                               client_entity_updates_cb, ud);
  return PCL_OK;
}

/// Server thread function -- no GTest assertions (not safe off main thread).
/// Sets srv.failed on error instead.
void server_thread_fn(ServerState& srv) {
  pcl_executor_t* exec = pcl_executor_create();
  if (!exec) { srv.failed.store(true); srv.ready.store(true); return; }

  // Pick a test port (derived from PID to reduce collisions in CI)
  uint16_t test_port = static_cast<uint16_t>(19000 + (getpid() % 1000));
  srv.port.store(test_port);
  srv.ready.store(true);  // signal port is known; server will now block on accept

  // Blocks until client connects
  pcl_socket_transport_t* transport =
      pcl_socket_transport_create_server(test_port, exec);
  if (!transport) {
    std::fprintf(stderr, "[server] create_server failed on port %d\n", test_port);
    srv.failed.store(true);
    pcl_executor_destroy(exec);
    return;
  }

  pcl_executor_set_transport(exec,
                             pcl_socket_transport_get_transport(transport));

  // Create TacticalObjectsComponent
  TacticalObjectsComponent tobj;
  tobj.configure();
  tobj.activate();
  tobj.setTickRateHz(100.0);
  pcl_executor_add(exec, tobj.handle());

  // Gateway container (dispatches SERVICE_REQ -> service handlers)
  pcl_container_t* gateway =
      pcl_socket_transport_gateway_container(transport);
  pcl_container_configure(gateway);
  pcl_container_activate(gateway);
  pcl_executor_add(exec, gateway);

  // Create a test entity so there's data to stream
  {
    ObjectDefinition def;
    def.type = ObjectType::Platform;
    def.position = Position{51.0, 0.0, 0};
    def.affiliation = Affiliation::Hostile;
    auto j = TacticalObjectsCodec::encodeObjectDefinition(def);
    std::string s = j.dump();
    pcl_msg_t req = {};
    req.data = s.data();
    req.size = static_cast<uint32_t>(s.size());
    req.type_name = "application/json";
    char rbuf[512];
    pcl_msg_t resp = {};
    resp.data = rbuf;
    resp.size = sizeof(rbuf);
    pcl_executor_invoke_service(exec, "create_object", &req, &resp);
  }

  // Spin until done or timeout
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!srv.done.load() &&
         std::chrono::steady_clock::now() < deadline) {
    pcl_executor_spin_once(exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  // Destroy transport first (closes socket, unblocks recv thread)
  pcl_socket_transport_destroy(transport);
  pcl_executor_destroy(exec);
  std::fprintf(stderr, "[server] Thread exiting\n");
}

}  // namespace

TEST(TacticalObjectsSocketE2E, CppClientReceivesEntityUpdates) {
  ServerState srv;

  // Start server thread (will block on accept until client connects)
  std::thread server(server_thread_fn, std::ref(srv));

  // Wait for server to signal port
  {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!srv.ready.load() &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  ASSERT_TRUE(srv.ready.load()) << "Server thread did not become ready";
  ASSERT_FALSE(srv.failed.load()) << "Server thread reported early failure";

  // Small delay to ensure server is in accept()
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  uint16_t port = srv.port.load();
  std::fprintf(stderr, "[client] Connecting to 127.0.0.1:%d\n", port);

  // --- Client side (on main thread) ---
  pcl_executor_t* client_exec = pcl_executor_create();
  ASSERT_NE(client_exec, nullptr);

  pcl_socket_transport_t* client_transport =
      pcl_socket_transport_create_client("127.0.0.1", port, client_exec);
  ASSERT_NE(client_transport, nullptr)
      << "Failed to connect to server on port " << port;

  pcl_executor_set_transport(
      client_exec, pcl_socket_transport_get_transport(client_transport));

  // Subscriber container
  ClientRecvContext recv_ctx;
  pcl_callbacks_t client_cbs = {};
  client_cbs.on_configure = client_on_configure;
  pcl_container_t* client_container =
      pcl_container_create("socket_test_client", &client_cbs, &recv_ctx);
  ASSERT_NE(client_container, nullptr);
  pcl_container_configure(client_container);
  pcl_container_activate(client_container);
  pcl_executor_add(client_exec, client_container);

  // Remote service call: subscribe_interest (async -- non-blocking enqueue)
  struct SvcRespCtx {
    std::string   body;
    std::atomic<bool> ready{false};
  } svc_resp;

  {
    nlohmann::json sub_req;
    sub_req["object_type"] = "Platform";
    sub_req["affiliation"] = "Hostile";
    sub_req["area"] = nlohmann::json::object(
        {{"min_lat", 50.0}, {"max_lat", 52.0},
         {"min_lon", -1.0}, {"max_lon", 1.0}});
    sub_req["expires_at"] = 9999.0;
    std::string sub_str = sub_req.dump();
    pcl_msg_t req = {};
    req.data = sub_str.data();
    req.size = static_cast<uint32_t>(sub_str.size());
    req.type_name = "application/json";

    struct SvcCb {
      static void fn(const pcl_msg_t* resp, void* ud) {
        auto* ctx = static_cast<SvcRespCtx*>(ud);
        if (resp && resp->data && resp->size > 0) {
          ctx->body.assign(static_cast<const char*>(resp->data), resp->size);
        }
        ctx->ready.store(true);
      }
    };

    pcl_status_t rc = pcl_socket_transport_invoke_remote_async(
        client_transport, "subscribe_interest", &req,
        SvcCb::fn, &svc_resp);
    ASSERT_EQ(rc, PCL_OK) << "invoke_remote_async enqueue failed";
  }

  // Spin client until service response and at least one entity update arrive
  {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      pcl_executor_spin_once(client_exec, 0);
      if (svc_resp.ready.load() && recv_ctx.received_any.load()) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  ASSERT_TRUE(svc_resp.ready.load()) << "subscribe_interest response timed out";
  {
    auto jr = nlohmann::json::parse(svc_resp.body);
    std::string interest_id = jr.value("interest_id", "");
    ASSERT_FALSE(interest_id.empty())
        << "subscribe_interest returned no interest_id";
    std::fprintf(stderr, "[client] Got interest_id: %s\n",
                 interest_id.c_str());
  }

  // Capture results before teardown
  bool got_any;
  size_t frame_count;
  uint8_t first_msg_type = 0;
  {
    std::lock_guard<std::mutex> lock(recv_ctx.mu);
    got_any = recv_ctx.received_any.load();
    frame_count = recv_ctx.received_frames.size();
    if (!recv_ctx.received_frames.empty()) {
      first_msg_type = recv_ctx.received_frames[0].message_type;
    }
    std::fprintf(stderr, "[client] Received %zu entity frames total\n",
                 frame_count);
  }

  // Signal server to stop, then destroy client transport (closes socket,
  // which unblocks the server's recv thread so it can exit cleanly).
  srv.done.store(true);
  pcl_socket_transport_destroy(client_transport);
  // Destroy executor before container: pcl_executor_destroy iterates its
  // containers array and writes c->executor = NULL.  If the container is
  // freed first that write is a use-after-free / access violation.
  pcl_executor_destroy(client_exec);
  pcl_container_destroy(client_container);

  server.join();

  // Assertions (after all threads joined to avoid races)
  ASSERT_FALSE(srv.failed.load()) << "Server thread reported failure";
  EXPECT_TRUE(got_any)
      << "Client should have received entity_updates over socket transport";
  EXPECT_GE(frame_count, 1u)
      << "Client should have received at least one entity frame";
  if (frame_count > 0) {
    EXPECT_EQ(first_msg_type, STREAM_MSG_ENTITY_UPDATE);
  }
}
