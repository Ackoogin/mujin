// cpp_active_find_e2e.cpp
//
// E2E example for the standard ActiveFind flow via StandardBridge (C++).
//
// Two reusable components connect to the server over TCP socket transport:
//   1. Interest Client   — calls create_requirement, receives entity_matches
//   2. Evidence Provider — receives evidence_requirements, publishes evidence
//
// Architecture: main (this) > component logic > service binding > PCL
//
// Business/test logic (what to request, what to assert) is authored here.
// Serialisation is handled by the generated proto-native codecs.
//
// Usage: cpp_active_find_e2e [--host 127.0.0.1] [--port 19123]

#include "tobj_interest_client.hpp"
#include "tobj_evidence_provider.hpp"

#include <pcl/pcl_executor.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_transport_socket.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

using namespace tobj_example;
using namespace pyramid::services::tactical_objects;

namespace {

constexpr double kPi        = 3.14159265358979323846;
constexpr double kDegToRad  = kPi / 180.0;

static void log(const char* msg) {
    std::fprintf(stderr, "[cpp_active_find] %s\n", msg);
    std::fflush(stderr);
}

} // namespace

int main(int argc, char** argv) {
    // -- Parse args -----------------------------------------------------------

    const char* host = "127.0.0.1";
    uint16_t    port = 19000;

    for (int i = 1; i < argc - 1; ++i) {
        if (std::strcmp(argv[i], "--host") == 0) host = argv[++i];
        if (std::strcmp(argv[i], "--port") == 0) port = static_cast<uint16_t>(std::atoi(argv[++i]));
    }

    char connect_msg[256];
    std::snprintf(connect_msg, sizeof(connect_msg),
                  "Connecting to %s:%u", host, port);
    log(connect_msg);

    // -- Create executor ------------------------------------------------------

    pcl_executor_t* exec = pcl_executor_create();
    if (!exec) { log("FAIL: could not create executor"); return 1; }

    // -- Connect via socket client transport ----------------------------------

    pcl_socket_transport_t* transport =
        pcl_socket_transport_create_client(host, port, exec);
    if (!transport) {
        log("FAIL: could not connect to server");
        pcl_executor_destroy(exec);
        return 1;
    }
    pcl_executor_set_transport(exec,
                               pcl_socket_transport_get_transport(transport));

    // -- Create interest client container -------------------------------------

    InterestClientState client_state;
    pcl_callbacks_t client_cbs{};
    client_cbs.on_configure = interestClientOnConfigure;
    pcl_container_t* client_c =
        pcl_container_create("cpp_client", &client_cbs, &client_state);
    if (!client_c) {
        log("FAIL: could not create client container");
        pcl_socket_transport_destroy(transport);
        pcl_executor_destroy(exec);
        return 1;
    }
    pcl_container_configure(client_c);
    pcl_container_activate(client_c);
    pcl_executor_add(exec, client_c);

    // -- Create evidence provider container -----------------------------------

    EvidenceProviderState provider_state;
    provider_state.executor = exec;
    pcl_callbacks_t prov_cbs{};
    prov_cbs.on_configure = evidenceProviderOnConfigure;
    pcl_container_t* provider_c =
        pcl_container_create("cpp_provider", &prov_cbs, &provider_state);
    if (!provider_c) {
        log("FAIL: could not create provider container");
        pcl_container_destroy(client_c);
        pcl_socket_transport_destroy(transport);
        pcl_executor_destroy(exec);
        return 1;
    }
    pcl_container_configure(provider_c);
    pcl_container_activate(provider_c);
    pcl_executor_add(exec, provider_c);

    // -- Send create_requirement (ActiveFind via bridge) ----------------------
    //
    // Business logic: request HOSTILE entities in SEA_SURFACE dimension within
    // a bounding box covering the English Channel (50–52°N, 1°W–1°E).
    // Typed enum values avoid stringly-typed JSON construction here.

    sendCreateRequirement(
        transport,
        &client_state,
        DataPolicy::Obtain,
        StandardIdentity::Hostile,
        BattleDimension::SeaSurface,
        50.0 * kDegToRad,   // min_lat_rad
        52.0 * kDegToRad,   // max_lat_rad
        -1.0 * kDegToRad,   // min_lon_rad
         1.0 * kDegToRad);  // max_lon_rad

    // -- Spin until the full standard flow completes --------------------------

    log("Spinning to drive standard ActiveFind flow...");
    for (int i = 0; i < 400; ++i) {
        pcl_executor_spin_once(exec, 0);
        if (client_state.svc_response_ready.load() &&
            client_state.matches_received.load() > 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (client_state.svc_response_ready.load()) {
        log("create_requirement OK");
        if (client_state.interest_id_received.load())
            log("  interest_id present in response");
        else
            log("  WARNING: interest_id NOT found in response");
    } else {
        log("create_requirement TIMEOUT: no response");
    }

    // -- Report pass/fail -----------------------------------------------------

    log("--- Results ---");
    {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "  Evidence requirement received: %s",
                      provider_state.evidence_req_received.load() ? "true" : "false");
        log(buf);
        std::snprintf(buf, sizeof(buf), "  Standard observation sent:     %s",
                      provider_state.observation_sent.load() ? "true" : "false");
        log(buf);
        std::snprintf(buf, sizeof(buf), "  Standard entity matches:       %d",
                      client_state.matches_received.load());
        log(buf);
        std::snprintf(buf, sizeof(buf), "  Found HOSTILE entity:          %s",
                      client_state.found_hostile_entity.load() ? "true" : "false");
        log(buf);
    }

    bool passed = provider_state.evidence_req_received.load()
               && provider_state.observation_sent.load()
               && client_state.matches_received.load() > 0
               && client_state.found_hostile_entity.load();

    if (passed) {
        log("PASS: Standard ActiveFind flow — evidence provider drove entity "
            "creation via bridge, client received HOSTILE via standard.entity_matches");
    } else {
        log("FAIL: Standard ActiveFind flow incomplete");
    }

    // -- Cleanup --------------------------------------------------------------

    pcl_socket_transport_destroy(transport);
    pcl_container_destroy(provider_c);
    pcl_container_destroy(client_c);
    pcl_executor_destroy(exec);

    return passed ? 0 : 1;
}
