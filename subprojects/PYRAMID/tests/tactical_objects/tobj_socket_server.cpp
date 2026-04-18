/// \file tobj_socket_server.cpp
/// \brief Standalone C++ server for cross-process socket E2E testing.
///
/// Starts a TacticalObjectsComponent with socket server transport,
/// creates a test entity, writes the port to a file, and spins until
/// a timeout expires or a signal is received.
///
/// Usage: tobj_socket_server [--port PORT] [--port-file PATH] [--timeout SECS]
#include <TacticalObjectsComponent.h>
#include <TacticalObjectsCodec.h>
#include <StandardBridge.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_transport_socket.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

using namespace tactical_objects;

static std::atomic<bool> g_shutdown{false};

static void signal_handler(int) { g_shutdown.store(true); }
static double deg_to_rad(double degrees) { return degrees * 0.017453292519943295; }

int main(int argc, char* argv[]) {
  uint16_t port = 19123;
  std::string port_file;
  int timeout_secs = 15;
  bool create_entity = true;
  bool use_bridge = true;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (std::strcmp(argv[i], "--port-file") == 0 && i + 1 < argc) {
      port_file = argv[++i];
    } else if (std::strcmp(argv[i], "--timeout") == 0 && i + 1 < argc) {
      timeout_secs = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--no-entity") == 0) {
      create_entity = false;
    } else if (std::strcmp(argv[i], "--no-bridge") == 0) {
      use_bridge = false;
    }
  }

  std::signal(SIGTERM, signal_handler);
  std::signal(SIGINT, signal_handler);

  // Create executor
  pcl_executor_t* exec = pcl_executor_create();
  if (!exec) {
    std::fprintf(stderr, "[server] Failed to create executor\n");
    return 1;
  }

  // Create and configure TacticalObjectsComponent
  TacticalObjectsComponent tobj;
  tobj.configure();
  tobj.activate();
  tobj.setTickRateHz(100.0);
  pcl_executor_add(exec, tobj.handle());

  // Create and configure StandardBridge (translates standard proto ↔ internal)
  // When --no-bridge is set, the bridge runs as a separate process.
  std::unique_ptr<StandardBridge> bridge;
  if (use_bridge) {
    bridge = std::make_unique<StandardBridge>(tobj.runtime(), exec);
    bridge->configure();
    bridge->activate();
    pcl_executor_add(exec, bridge->handle());
  }

  // Write port file before accept (so client knows the port to connect to)
  if (!port_file.empty()) {
    std::ofstream pf(port_file);
    pf << port << std::endl;
    pf.close();
    std::fprintf(stderr, "[server] Wrote port %d to %s\n", port,
                 port_file.c_str());
  }

  std::fprintf(stderr, "[server] Waiting for client connection on port %d...\n",
               port);

  // This blocks until a client connects
  pcl_socket_transport_t* transport =
      pcl_socket_transport_create_server(port, exec);
  if (!transport) {
    std::fprintf(stderr, "[server] Failed to create socket server transport\n");
    pcl_executor_destroy(exec);
    return 1;
  }

  pcl_executor_set_transport(exec,
                             pcl_socket_transport_get_transport(transport));

  // Add gateway container
  pcl_container_t* gateway =
      pcl_socket_transport_gateway_container(transport);
  pcl_container_configure(gateway);
  pcl_container_activate(gateway);
  pcl_executor_add(exec, gateway);

  std::fprintf(stderr, "[server] Client connected.\n");

  if (create_entity) {
    std::fprintf(stderr, "[server] Creating test entity...\n");
    ObjectDefinition def;
    def.type = ObjectType::Platform;
    def.position = Position{deg_to_rad(51.0), 0.0, 0};
    def.affiliation = Affiliation::Hostile;
    auto j = TacticalObjectsCodec::encodeObjectDefinition(def);
    std::string create_str = j.dump();
    pcl_msg_t req = {};
    req.data = create_str.data();
    req.size = static_cast<uint32_t>(create_str.size());
    req.type_name = "application/json";
    pcl_msg_t resp = {};
    char resp_buf[512];
    resp.data = resp_buf;
    resp.size = sizeof(resp_buf);
    pcl_executor_invoke_service(exec, "create_object", &req, &resp);
  } else {
    std::fprintf(stderr, "[server] Skipping entity creation (--no-entity)\n");
  }

  std::fprintf(stderr, "[server] Spinning for up to %d seconds...\n",
               timeout_secs);

  // Spin until timeout or signal
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::seconds(timeout_secs);
  while (!g_shutdown.load() &&
         std::chrono::steady_clock::now() < deadline) {
    pcl_executor_spin_once(exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  std::fprintf(stderr, "[server] Shutting down.\n");

  pcl_socket_transport_destroy(transport);
  pcl_executor_destroy(exec);
  return 0;
}
