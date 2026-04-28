/// \file pyramid_bridge_evidence_client.cpp
/// \brief Test client that publishes standard.object_evidence to Tactical Objects.

#include "pyramid_services_tactical_objects_consumed.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport_shared_memory.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

namespace {

namespace consumed = pyramid::components::tactical_objects::services::consumed;

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
std::atomic<bool> g_shutdown{false};

void signal_handler(int) {
  g_shutdown.store(true);
}

struct ClientState {
  std::string content_type = consumed::kJsonContentType;
  pcl_port_t* publisher = nullptr;
  int published_count = 0;
};

pcl_status_t on_configure(pcl_container_t* container, void* user_data) {
  auto* state = static_cast<ClientState*>(user_data);
  state->publisher = pcl_container_add_publisher(
      container, consumed::kTopicObjectEvidence, state->content_type.c_str());
  return state->publisher ? PCL_OK : PCL_ERR_CALLBACK;
}

pyramid::domain_model::ObjectDetail make_evidence() {
  pyramid::domain_model::ObjectDetail evidence;
  evidence.id = "bridge-e2e-evidence";
  evidence.entity_source = "demo-radar";
  evidence.identity = pyramid::domain_model::StandardIdentity::Hostile;
  evidence.dimension = pyramid::domain_model::BattleDimension::SeaSurface;
  evidence.position.latitude = 51.0 * kDegToRad;
  evidence.position.longitude = 0.0;
  evidence.quality = 0.95;
  evidence.creation_time = 1.0;
  return evidence;
}

}  // namespace

int main(int argc, char* argv[]) {
  const char* bus_name = "pyramid_tobj_bridge";
  int timeout_secs = 25;
  ClientState state;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--bus") == 0 && i + 1 < argc) {
      bus_name = argv[++i];
    } else if (std::strcmp(argv[i], "--timeout") == 0 && i + 1 < argc) {
      timeout_secs = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--content-type") == 0 && i + 1 < argc) {
      state.content_type = argv[++i];
    }
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  pcl_executor_t* exec = pcl_executor_create();
  if (!exec) {
    std::fprintf(stderr, "[evidence_client] FAIL: could not create executor\n");
    return 1;
  }

  pcl_shared_memory_transport_t* transport =
      pcl_shared_memory_transport_create(bus_name, "evidence_client", exec);
  if (!transport) {
    std::fprintf(stderr,
                 "[evidence_client] FAIL: could not join shared-memory bus '%s'\n",
                 bus_name);
    pcl_executor_destroy(exec);
    return 1;
  }

  pcl_executor_set_transport(exec,
                             pcl_shared_memory_transport_get_transport(transport));

  pcl_container_t* gateway =
      pcl_shared_memory_transport_gateway_container(transport);
  pcl_container_configure(gateway);
  pcl_container_activate(gateway);
  pcl_executor_add(exec, gateway);

  pcl_callbacks_t callbacks{};
  callbacks.on_configure = on_configure;
  pcl_container_t* container =
      pcl_container_create("pyramid_bridge_evidence_client", &callbacks, &state);
  if (!container) {
    std::fprintf(stderr, "[evidence_client] FAIL: could not create container\n");
    pcl_shared_memory_transport_destroy(transport);
    pcl_executor_destroy(exec);
    return 1;
  }

  pcl_container_configure(container);
  pcl_container_activate(container);
  pcl_executor_add(exec, container);

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(timeout_secs);
  auto next_publish = std::chrono::steady_clock::now();

  while (!g_shutdown.load() && std::chrono::steady_clock::now() < deadline) {
    pcl_executor_spin_once(exec, 0);

    const auto now = std::chrono::steady_clock::now();
    if (now >= next_publish && state.publisher) {
      const auto evidence = make_evidence();
      const pcl_status_t rc = consumed::publishObjectEvidence(
          state.publisher, evidence, state.content_type.c_str());
      if (rc == PCL_OK) {
        ++state.published_count;
        std::fprintf(stderr,
                     "[evidence_client] published standard.object_evidence"
                     " id=%s source=%s count=%d\n",
                     evidence.id.c_str(), evidence.entity_source.c_str(),
                     state.published_count);
      } else {
        std::fprintf(stderr,
                     "[evidence_client] publish standard.object_evidence"
                     " failed rc=%d\n",
                     static_cast<int>(rc));
      }
      std::fflush(stderr);
      next_publish = now + std::chrono::milliseconds(500);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  pcl_container_destroy(container);
  pcl_shared_memory_transport_destroy(transport);
  pcl_executor_destroy(exec);

  return state.published_count > 0 ? 0 : 1;
}
