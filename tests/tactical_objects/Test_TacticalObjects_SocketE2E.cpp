/// \file Test_TacticalObjects_SocketE2E.cpp
/// \brief Socket transport E2E test: C++ client ↔ C++ server over TCP.
///
/// Validates the full socket transport round-trip without requiring Ada:
/// 1. Server thread: TacticalObjectsComponent with socket server transport
/// 2. Client thread: subscribes to entity_updates, invokes subscribe_interest
///    and create_object via remote service calls, receives entity update frames.
#include <gtest/gtest.h>
#include <TacticalObjectsComponent.h>
#include <TacticalObjectsCodec.h>
#include <StreamingCodec.h>
#include <pcl/executor.hpp>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_transport.h>
#include <pcl/pcl_transport_socket.h>
#include <uuid/UUIDHelper.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace tactical_objects;
using namespace pyramid::core::uuid;

namespace {

static const char* TOPIC_ENTITY_UPDATES = "entity_updates";

/// Port file mechanism: server writes ephemeral port to this path.
struct ServerContext {
  std::atomic<uint16_t> port{0};
  std::atomic<bool> ready{false};
  std::atomic<bool> done{false};
};

struct ClientRecvContext {
  std::mutex mu;
  std::vector<EntityUpdateFrame> received_frames;
  std::atomic<bool> received_any{false};
};

static void client_entity_updates_cb(pcl_container_t*, const pcl_msg_t* msg,
                                     void* user_data) {
  auto* ctx = static_cast<ClientRecvContext*>(user_data);
  if (!msg->data || msg->size == 0) return;
  auto frames = StreamingCodec::decodeBatchFrame(
      static_cast<const uint8_t*>(msg->data), msg->size);
  std::lock_guard<std::mutex> lock(ctx->mu);
  for (const auto& f : frames) {
    ctx->received_frames.push_back(f);
    ctx->received_any.store(true);
  }
}

static pcl_status_t client_on_configure(pcl_container_t* c, void* ud) {
  pcl_container_add_subscriber(c, TOPIC_ENTITY_UPDATES,
                               "application/octet-stream",
                               client_entity_updates_cb, ud);
  return PCL_OK;
}

/// Server thread: creates socket server transport, configures TacticalObjects,
/// creates a test entity, spins until told to stop.
void server_thread_fn(ServerContext& srv) {
  // Create executor
  pcl_executor_t* exec = pcl_executor_create();
  ASSERT_NE(exec, nullptr);

  // Create TacticalObjectsComponent and configure
  TacticalObjectsComponent tobj;
  tobj.configure();
  tobj.activate();
  tobj.setTickRateHz(1000.0);

  pcl_executor_add(exec, tobj.handle());

  // Create socket server transport on ephemeral port.
  // accept() blocks until client connects, so we must signal port first.
  // Since create_server binds+listens then blocks on accept, and port is
  // assigned after bind, we need a two-phase approach: but the current API
  // doesn't support that. Instead we use a known port range.
  //
  // Actually, looking at the implementation: bind → getsockname → listen → accept.
  // The port is discovered before accept blocks. But create_server returns only
  // after accept. So we run it in this thread and have the client connect.
  //
  // The trick: the client must know the port before connecting. We use a fixed
  // test port derived from the thread ID to reduce collision probability.
  uint16_t test_port = 19000 + static_cast<uint16_t>(
      std::hash<std::thread::id>{}(std::this_thread::get_id()) % 1000);
  srv.port.store(test_port);
  srv.ready.store(true);

  // This blocks until client connects
  pcl_socket_transport_t* transport =
      pcl_socket_transport_create_server(test_port, exec);
  if (!transport) {
    srv.done.store(true);
    pcl_executor_destroy(exec);
    FAIL() << "Failed to create socket server transport on port " << test_port;
    return;
  }

  pcl_executor_set_transport(exec, pcl_socket_transport_get_transport(transport));

  // Add gateway container (dispatches SERVICE_REQ to service handlers)
  pcl_container_t* gateway = pcl_socket_transport_gateway_container(transport);
  pcl_container_configure(gateway);
  pcl_container_activate(gateway);
  pcl_executor_add(exec, gateway);

  // Create a test entity so there's something to stream
  ObjectDefinition def;
  def.type = ObjectType::Platform;
  def.position = Position{51.0, 0.0, 0};
  def.affiliation = Affiliation::Hostile;
  auto j_create = TacticalObjectsCodec::encodeObjectDefinition(def);
  std::string create_str = j_create.dump();
  pcl_msg_t req = {};
  req.data = create_str.data();
  req.size = static_cast<uint32_t>(create_str.size());
  req.type_name = "application/json";
  pcl_msg_t resp = {};
  char resp_buf[512];
  resp.data = resp_buf;
  resp.size = sizeof(resp_buf);
  pcl_executor_invoke_service(exec, "create_object", &req, &resp);

  // Spin until client signals done or timeout
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!srv.done.load() &&
         std::chrono::steady_clock::now() < deadline) {
    pcl_executor_spin_once(exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  pcl_socket_transport_destroy(transport);
  pcl_executor_destroy(exec);
}

}  // namespace

TEST(TacticalObjectsSocketE2E, CppClientReceivesEntityUpdates) {
  ServerContext srv;

  // Start server in background thread
  std::thread server(server_thread_fn, std::ref(srv));

  // Wait for server to be ready (port assigned, listening)
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!srv.ready.load() &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(srv.ready.load()) << "Server did not become ready in time";

  // Brief delay to ensure server is in accept()
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  uint16_t port = srv.port.load();

  // --- Client side ---
  pcl_executor_t* client_exec = pcl_executor_create();
  ASSERT_NE(client_exec, nullptr);

  // Connect to server
  pcl_socket_transport_t* client_transport =
      pcl_socket_transport_create_client("127.0.0.1", port, client_exec);
  ASSERT_NE(client_transport, nullptr)
      << "Failed to connect to server on port " << port;

  pcl_executor_set_transport(
      client_exec, pcl_socket_transport_get_transport(client_transport));

  // Create client container that subscribes to entity_updates
  ClientRecvContext recv_ctx;
  pcl_callbacks_t client_cbs = {};
  client_cbs.on_configure = client_on_configure;
  pcl_container_t* client_container =
      pcl_container_create("socket_test_client", &client_cbs, &recv_ctx);
  ASSERT_NE(client_container, nullptr);
  pcl_container_configure(client_container);
  pcl_container_activate(client_container);
  pcl_executor_add(client_exec, client_container);

  // Invoke remote service: subscribe_interest
  nlohmann::json sub_req;
  sub_req["object_type"] = "Platform";
  sub_req["affiliation"] = "Hostile";
  sub_req["area"] = nlohmann::json::object(
      {{"min_lat", 50.0}, {"max_lat", 52.0}, {"min_lon", -1.0}, {"max_lon", 1.0}});
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

  pcl_status_t rc = pcl_socket_transport_invoke_remote(
      client_transport, "subscribe_interest", &req, &resp);
  ASSERT_EQ(rc, PCL_OK) << "subscribe_interest remote call failed";

  std::string resp_str(static_cast<const char*>(resp.data), resp.size);
  auto jr = nlohmann::json::parse(resp_str);
  std::string interest_id = jr.value("interest_id", "");
  ASSERT_FALSE(interest_id.empty()) << "subscribe_interest returned no interest_id";

  // Spin client to receive entity updates
  auto recv_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!recv_ctx.received_any.load() &&
         std::chrono::steady_clock::now() < recv_deadline) {
    pcl_executor_spin_once(client_exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // Signal server to stop
  srv.done.store(true);

  // Verify client received entity updates
  {
    std::lock_guard<std::mutex> lock(recv_ctx.mu);
    EXPECT_TRUE(recv_ctx.received_any.load())
        << "Client should have received entity_updates over socket transport";
    EXPECT_GE(recv_ctx.received_frames.size(), 1u)
        << "Client should have received at least one entity frame";
  }

  // Cleanup
  pcl_socket_transport_destroy(client_transport);
  pcl_container_destroy(client_container);
  pcl_executor_destroy(client_exec);

  server.join();
}
