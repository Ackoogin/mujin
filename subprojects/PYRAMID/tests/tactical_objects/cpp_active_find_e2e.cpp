// cpp_active_find_e2e.cpp
//
// E2E example for the standard ActiveFind flow via StandardBridge (C++).
//
// Two reusable components connect to the server over TCP socket transport:
//   1. TobjInterestClient   -- calls create_requirement, receives entity_matches
//   2. TobjEvidenceProvider -- receives evidence_requirements, publishes evidence
//
// Both are hand-written pcl::Component subclasses; the interest client owns a
// generated ConsumedService binding for the create_requirement RPC.
//
// Usage: cpp_active_find_e2e [--host 127.0.0.1] [--port 19123]
//                            [--content-type application/json]

#include "tobj_evidence_provider.hpp"
#include "tobj_interest_client.hpp"

#include <pcl/executor.hpp>
#include <pcl/pcl_transport_socket.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <future>
#include <thread>

using namespace tobj_example;

namespace {

constexpr double kPi       = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

void log(const char* msg) {
  std::fprintf(stderr, "[cpp_active_find] %s\n", msg);
  std::fflush(stderr);
}

}  // namespace

int main(int argc, char** argv) {
  const char* host = "127.0.0.1";
  uint16_t    port = 19000;
  const char* content_type = Provided::kJsonContentType;

  for (int i = 1; i < argc - 1; ++i) {
    if (std::strcmp(argv[i], "--host") == 0) host = argv[++i];
    if (std::strcmp(argv[i], "--port") == 0)
      port = static_cast<uint16_t>(std::atoi(argv[++i]));
    if (std::strcmp(argv[i], "--content-type") == 0) content_type = argv[++i];
  }

  char connect_msg[256];
  std::snprintf(connect_msg, sizeof(connect_msg),
                "Connecting to %s:%u content-type=%s",
                host, port, content_type);
  log(connect_msg);

  pcl::Executor executor;

  pcl_socket_transport_t* transport =
      pcl_socket_transport_create_client(host, port, executor.handle());
  if (!transport) {
    log("FAIL: could not connect to server");
    return 1;
  }
  executor.setTransport(pcl_socket_transport_get_transport(transport));

  TobjInterestClient   client{executor, content_type};
  TobjEvidenceProvider provider{content_type};

  if (client.configure()       != PCL_OK ||
      client.activate()        != PCL_OK ||
      executor.add(client)     != PCL_OK) {
    log("FAIL: could not bring up client component");
    pcl_socket_transport_destroy(transport);
    return 1;
  }
  if (provider.configure()     != PCL_OK ||
      provider.activate()      != PCL_OK ||
      executor.add(provider)   != PCL_OK) {
    log("FAIL: could not bring up provider component");
    pcl_socket_transport_destroy(transport);
    return 1;
  }

  // -- Issue create_requirement -------------------------------------------
  //
  // Business logic: request entities in SEA_SURFACE dimension within a
  // bounding box covering the English Channel (50-52 degN, 1 degW-1 degE).
  log("create_requirement request (proto-native typed)");
  auto svc_future = client.createRequirementAsync(makeActiveFindRequirement(
      DataPolicy::Obtain,
      BattleDimension::SeaSurface,
      50.0 * kDegToRad,
      52.0 * kDegToRad,
      -1.0 * kDegToRad,
       1.0 * kDegToRad));

  // -- Spin until the full standard flow completes ------------------------
  log("Spinning to drive standard ActiveFind flow...");
  bool         svc_ready          = false;
  Identifier   interest_id;
  pcl_status_t svc_status         = PCL_OK;

  for (int i = 0; i < 400; ++i) {
    executor.spinOnce(0);
    if (!svc_ready &&
        svc_future.wait_for(std::chrono::milliseconds(0)) ==
            std::future_status::ready) {
      auto result   = svc_future.get();
      svc_ready     = true;
      svc_status    = result.status;
      interest_id   = std::move(result.value);
    }
    if (svc_ready && client.matchesReceived() > 0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (svc_ready && svc_status == PCL_OK) {
    log("create_requirement OK");
    if (!interest_id.empty()) {
      log(("  interest_id: " + interest_id).c_str());
    } else {
      log("  WARNING: interest_id NOT found in response");
    }
  } else {
    log("create_requirement TIMEOUT: no response");
  }

  log("--- Results ---");
  {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "  Evidence requirement received: %s",
                  provider.evidenceRequirementReceived() ? "true" : "false");
    log(buf);
    std::snprintf(buf, sizeof(buf), "  Standard observation sent:     %s",
                  provider.observationSent() ? "true" : "false");
    log(buf);
    std::snprintf(buf, sizeof(buf), "  Standard entity matches:       %d",
                  client.matchesReceived());
    log(buf);
    std::snprintf(buf, sizeof(buf), "  Found HOSTILE entity:          %s",
                  client.foundHostileObject() ? "true" : "false");
    log(buf);
  }

  const bool passed = provider.evidenceRequirementReceived()
                   && provider.observationSent()
                   && client.matchesReceived() > 0
                   && client.foundHostileObject();

  if (passed) {
    log("PASS: Standard ActiveFind flow -- evidence provider drove entity "
        "creation via bridge, client received HOSTILE via standard.entity_matches");
  } else {
    log("FAIL: Standard ActiveFind flow incomplete");
  }

  pcl_socket_transport_destroy(transport);
  return passed ? 0 : 1;
}
