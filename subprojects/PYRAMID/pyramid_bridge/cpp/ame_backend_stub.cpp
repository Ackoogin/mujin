/// \file ame_backend_stub.cpp
/// \brief Standalone C++ stub that acts as an AME Autonomy Backend on a
///        shared-memory PCL bus.
///
/// This stub exists because the real AME backend migration to the PYRAMID
/// proto service shape is not yet complete.  It provides enough of the
/// autonomy_backend.provided interface to let the Pyramid Bridge exercise
/// world-fact delivery end-to-end.
///
/// The stub:
///   - Joins a named shared-memory bus as participant "ame_backend".
///   - Registers state.create_state and state.update_state service handlers.
///   - Decodes each incoming State_Update (JSON codec) and prints every
///     World_Fact_Update to stderr.
///   - Tracks total facts received; exits with 0 when at least one fact
///     was received within the run window.
///
/// Usage:
///   ame_backend_stub [--bus NAME] [--timeout SECS]
///     --bus NAME      shared-memory bus name (default: pyramid_bridge)
///     --timeout SECS  run duration in seconds, 0 = run until SIGINT (default 30)

#include "pyramid_data_model_autonomy_codec.hpp"
#include "pyramid_services_autonomy_backend_provided.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport_shared_memory.h>

#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

using json = nlohmann::json;
namespace autonomy_codec = pyramid::data_model::autonomy;

static std::atomic<bool> g_shutdown{false};
static void signal_handler(int) { g_shutdown.store(true); }

// ---------------------------------------------------------------------------
// Global counters (written on executor thread, read in main after spin ends)
// ---------------------------------------------------------------------------

static std::atomic<int> g_facts_received{0};
static std::atomic<int> g_state_updates_received{0};

// ---------------------------------------------------------------------------
// State_Update handler — decodes JSON and logs each world fact
// ---------------------------------------------------------------------------

static void handle_state_update(const pyramid::data_model::autonomy::StateUpdate& update) {
  g_state_updates_received.fetch_add(1);
  std::fprintf(stderr, "[ame_stub] state.update_state received"
               " source=%s facts=%zu\n",
               update.source.c_str(),
               update.fact_update.size());

  for (const auto& fact : update.fact_update) {
    std::fprintf(stderr, "[ame_stub]   fact key=%s value=%s authority=%s\n",
                 fact.key.c_str(),
                 fact.value ? "true" : "false",
                 autonomy_codec::toString(fact.authority).c_str());
    g_facts_received.fetch_add(1);
  }
  std::fflush(stderr);
}

// ---------------------------------------------------------------------------
// Service handler for state.create_state and state.update_state
// ---------------------------------------------------------------------------

static std::string g_resp_buf;

static pcl_status_t state_service_handler(pcl_container_t*,
                                          const pcl_msg_t* request,
                                          pcl_msg_t*       response,
                                          pcl_svc_context_t*,
                                          void* /*user_data*/) {
  if (!request || !request->data || request->size == 0) {
    return PCL_ERR_INVALID;
  }

  try {
    const std::string payload(static_cast<const char*>(request->data),
                              request->size);
    const auto update =
        autonomy_codec::fromJson(payload,
                                 static_cast<pyramid::data_model::autonomy::StateUpdate*>(nullptr));
    handle_state_update(update);
  } catch (const std::exception& ex) {
    std::fprintf(stderr, "[ame_stub] JSON decode error: %s\n", ex.what());
  }

  // Respond with a simple ACK identifier.
  g_resp_buf = R"({"success":true})";
  response->data      = const_cast<char*>(g_resp_buf.data());
  response->size      = static_cast<uint32_t>(g_resp_buf.size());
  response->type_name = "application/json";
  return PCL_OK;
}

// ---------------------------------------------------------------------------
// Container on_configure: register state service ports
// ---------------------------------------------------------------------------

static pcl_status_t on_configure(pcl_container_t* c, void* /*user_data*/) {
  // Register both create_state and update_state so either call from the bridge
  // is handled.
  pcl_container_add_service(c, "state.create_state", "application/json",
                            state_service_handler, nullptr);
  pcl_container_add_service(c, "state.update_state", "application/json",
                            state_service_handler, nullptr);
  std::fprintf(stderr, "[ame_stub] registered state.create_state and "
               "state.update_state services\n");
  return PCL_OK;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char* argv[]) {
  const char* bus_name = "pyramid_bridge";
  int timeout_secs = 30;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--bus") == 0 && i + 1 < argc) {
      bus_name = argv[++i];
    } else if (std::strcmp(argv[i], "--timeout") == 0 && i + 1 < argc) {
      timeout_secs = std::atoi(argv[++i]);
    }
  }

  std::signal(SIGINT,  signal_handler);
  std::signal(SIGTERM, signal_handler);

  std::fprintf(stderr, "[ame_stub] joining shared-memory bus '%s' as 'ame_backend'\n",
               bus_name);

  pcl_executor_t* exec = pcl_executor_create();
  if (!exec) {
    std::fprintf(stderr, "[ame_stub] FAIL: could not create executor\n");
    return 1;
  }

  pcl_shared_memory_transport_t* transport =
      pcl_shared_memory_transport_create(bus_name, "ame_backend", exec);
  if (!transport) {
    std::fprintf(stderr,
                 "[ame_stub] FAIL: could not join shared-memory bus '%s'\n",
                 bus_name);
    pcl_executor_destroy(exec);
    return 1;
  }

  pcl_executor_set_transport(exec,
                             pcl_shared_memory_transport_get_transport(transport));

  // Gateway container dispatches inbound service requests from the bus.
  pcl_container_t* gateway =
      pcl_shared_memory_transport_gateway_container(transport);
  pcl_container_configure(gateway);
  pcl_container_activate(gateway);
  pcl_executor_add(exec, gateway);

  // Service container exposes state.create_state / state.update_state.
  pcl_callbacks_t callbacks{};
  callbacks.on_configure = on_configure;
  pcl_container_t* svc_container =
      pcl_container_create("ame_state_service", &callbacks, nullptr);
  if (!svc_container) {
    std::fprintf(stderr, "[ame_stub] FAIL: could not create service container\n");
    pcl_shared_memory_transport_destroy(transport);
    pcl_executor_destroy(exec);
    return 1;
  }

  pcl_container_configure(svc_container);
  pcl_container_activate(svc_container);
  pcl_executor_add(exec, svc_container);

  std::fprintf(stderr, "[ame_stub] ready — waiting for world facts (timeout=%ds)\n",
               timeout_secs);

  const auto start    = std::chrono::steady_clock::now();
  const auto deadline = start + std::chrono::seconds(timeout_secs > 0 ? timeout_secs : 86400);

  while (!g_shutdown.load() && std::chrono::steady_clock::now() < deadline) {
    pcl_executor_spin_once(exec, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const int facts = g_facts_received.load();
  const int updates = g_state_updates_received.load();
  std::fprintf(stderr,
               "[ame_stub] shutdown — state_updates=%d world_facts=%d\n",
               updates, facts);

  pcl_container_destroy(svc_container);
  pcl_shared_memory_transport_destroy(transport);
  pcl_executor_destroy(exec);

  return (facts > 0) ? 0 : 1;
}
