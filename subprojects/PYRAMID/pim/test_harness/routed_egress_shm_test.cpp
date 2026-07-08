// \file routed_egress_shm_test.cpp
// \brief Phase A of doc/plans/PYRAMID/agra_pubsub_shm_udp_proving_plan.md --
//        proves that contract-generated typed topic helpers for the existing
//        pim_osprey sensor_products contract actually traverse a *real*
//        shared-memory transport stood up from a routing manifest
//        (pcl_transport_routing_load loading libpcl_transport_shared_memory_plugin),
//        rather than the executor's local in-process dispatch that
//        components_comms_test.cpp exercises.
//
// Two scenarios, both driving the request/requirement/information topics
// through the generated publish*/subscribe* helpers and the JSON codec:
//   1. Two executors, one process, sharing one SHM bus.
//   2. The same routing setup split across two OS processes (fork + re-exec
//      of this same binary with --role=provider|consumer), mirroring the
//      spawn/synchronization pattern used by
//      subprojects/PCL/tests/pcl_shm_peer_helper.cpp, but going through the
//      manifest-driven routing path instead of the direct transport API.

#include "pyramid_services_pim_osprey_sensor_products_provided.hpp"

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport_routing.h>
}

extern "C" const pcl_codec_t* pcl_codec_plugin_entry(const char* config_json);

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <process.h>
#include <windows.h>
#else
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace prov = pyramid::components::pim_osprey::sensor_products::services::provided;

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("%s  %s\n", ok ? "OK  " : "FAIL", what);
    if (!ok) ++g_failures;
}

std::string get_option(int argc, char** argv, const char* prefix) {
    const size_t prefix_len = std::strlen(prefix);
    for (int i = 1; i < argc; ++i) {
        if (std::strncmp(argv[i], prefix, prefix_len) == 0) {
            return std::string(argv[i] + prefix_len);
        }
    }
    return {};
}

bool write_text(const std::string& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out << text;
    return out.good();
}

bool read_text(const std::string& path, std::string* out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out->assign((std::istreambuf_iterator<char>(in)),
                std::istreambuf_iterator<char>());
    return true;
}

bool file_exists(const std::string& path) {
    std::ifstream in(path);
    return in.good();
}

// -- Manifest authoring ---------------------------------------------------
//
// Hand-authored per subjects/PCL/include/pcl/pcl_transport_routing.h's
// documented format. contract_routing_manifest.py is not reused here: it is
// tuned for the NULL-vtable contract_transport_plugin.c stub (one pubsub
// peer covering both directions of a topic pair); a real SHM peer needs
// bus_name/participant_id config the script doesn't emit, and this harness
// wants one topic per route line with the direction explicit.
//
// The manifest's "transport <peer_id> ..." line names a *local* alias that
// route lines reference for two, otherwise-independent, purposes: compose-
// time validation (pcl_executor_validate_endpoint_route) requires it to
// match a locally-registered transport, while runtime ingress filtering on a
// subscriber port (pcl_executor.c's peer_is_allowed) compares route peer
// names against the *remote* sender's own participant_id (the SHM plugin
// posts pcl_executor_post_remote_incoming(..., frame->source_id, ...), i.e.
// the far side's bus identity, not any local alias). For a two-party
// correlated pair this is reconcilable without any PCL change: alias the
// local transport line after the *counterpart's* participant_id, so the one
// string satisfies both checks. own_participant_id is this side's own SHM
// bus identity (used when it publishes); peer_participant_id is the
// counterpart's, reused as the local peer alias.
bool write_manifest(const std::string& path,
                    const std::string& plugin_path,
                    const std::string& bus_name,
                    const std::string& own_participant_id,
                    const std::string& peer_participant_id,
                    bool is_provider) {
    std::ofstream out(path, std::ios::trunc);
    if (!out) return false;
    const std::string& peer_alias = peer_participant_id;
    out << "transport " << peer_alias << " " << plugin_path
        << " {\"bus_name\":\"" << bus_name << "\",\"participant_id\":\""
        << own_participant_id << "\"}\n";
    if (is_provider) {
        out << "route " << prov::kTopicPimOspreySprRequirementRequest
            << " subscriber " << peer_alias << " reliable\n";
        out << "route " << prov::kTopicPimOspreySprRequirementRequirement
            << " publisher " << peer_alias << " reliable\n";
        out << "route " << prov::kTopicPimOspreySprInformationInformation
            << " publisher " << peer_alias << " reliable\n";
    } else {
        out << "route " << prov::kTopicPimOspreySprRequirementRequest
            << " publisher " << peer_alias << " reliable\n";
        out << "route " << prov::kTopicPimOspreySprRequirementRequirement
            << " subscriber " << peer_alias << " reliable\n";
        out << "route " << prov::kTopicPimOspreySprInformationInformation
            << " subscriber " << peer_alias << " reliable\n";
    }
    return out.good();
}

// -- Shared role state ------------------------------------------------------

struct ProviderState {
    pcl_port_t* information_pub = nullptr;
    pcl_port_t* requirement_pub = nullptr;
    bool request_seen = false;
    bool cancel_seen = false;
    bool information_published = false;
};

struct ConsumerState {
    pcl_port_t* request_pub = nullptr;
    bool information_seen = false;
    bool requirement_seen = false;
    unsigned requirement_count = 0;
};

void publish_requirement_transition(ProviderState* state) {
    prov::SPRRequirement_Service_Requirement requirement;
    requirement.sprrequirement.emplace();
    const pcl_status_t rc = prov::publishPimOspreySprRequirementRequirement(
        state->requirement_pub, requirement, prov::kJsonContentType);
    check(rc == PCL_OK, "provider published requirement transition over SHM route");
}

void publish_information_once(ProviderState* state) {
    if (state->information_published) return;
    prov::SPRInformation_Service_Information info;
    info.sprinformation.emplace();
    const pcl_status_t rc = prov::publishPimOspreySprInformationInformation(
        state->information_pub, info, prov::kJsonContentType);
    check(rc == PCL_OK, "provider published information topic over SHM route");
    state->information_published = true;
}

void on_request_topic(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* state = static_cast<ProviderState*>(ud);
    prov::SPRRequirement_Service_Request request;
    if (!prov::decodePimOspreySprRequirementRequest(msg, &request)) {
        check(false, "provider decoded request-topic payload received over SHM");
        return;
    }
    if (request.sprrequest) {
        state->request_seen = true;
        publish_information_once(state);
    }
    if (request.cancel) {
        state->cancel_seen = true;
    }
    publish_requirement_transition(state);
}

void on_information_topic(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* state = static_cast<ConsumerState*>(ud);
    prov::SPRInformation_Service_Information payload;
    state->information_seen =
        prov::decodePimOspreySprInformationInformation(msg, &payload) &&
        static_cast<bool>(payload.sprinformation);
}

void on_requirement_topic(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* state = static_cast<ConsumerState*>(ud);
    prov::SPRRequirement_Service_Requirement payload;
    const bool ok =
        prov::decodePimOspreySprRequirementRequirement(msg, &payload) &&
        static_cast<bool>(payload.sprrequirement);
    if (ok) {
        state->requirement_seen = true;
        ++state->requirement_count;
    }
}

pcl_status_t provider_on_configure(pcl_container_t* c, void* ud) {
    auto* state = static_cast<ProviderState*>(ud);
    state->information_pub = pcl_container_add_publisher(
        c, prov::kTopicPimOspreySprInformationInformation, prov::kJsonContentType);
    state->requirement_pub = pcl_container_add_publisher(
        c, prov::kTopicPimOspreySprRequirementRequirement, prov::kJsonContentType);
    pcl_port_t* sub = prov::subscribePimOspreySprRequirementRequest(
        c, on_request_topic, state, prov::kJsonContentType);
    return (state->information_pub && state->requirement_pub && sub)
        ? PCL_OK : PCL_ERR_NOMEM;
}

pcl_status_t consumer_on_configure(pcl_container_t* c, void* ud) {
    auto* state = static_cast<ConsumerState*>(ud);
    state->request_pub = pcl_container_add_publisher(
        c, prov::kTopicPimOspreySprRequirementRequest, prov::kJsonContentType);
    pcl_port_t* info_sub = prov::subscribePimOspreySprInformationInformation(
        c, on_information_topic, state, prov::kJsonContentType);
    pcl_port_t* req_sub = prov::subscribePimOspreySprRequirementRequirement(
        c, on_requirement_topic, state, prov::kJsonContentType);
    return (state->request_pub && info_sub && req_sub) ? PCL_OK : PCL_ERR_NOMEM;
}

void register_json_codec() {
    const pcl_codec_t* codec = pcl_codec_plugin_entry(nullptr);
    check(pcl_codec_registry_register(pcl_codec_registry_default(), codec) == PCL_OK,
          "register JSON codec plugin");
}

// -- Scenario 1: two executors, one process, sharing one SHM bus -----------
//
// Loads the real SHM plugin twice (once per executor) via
// pcl_transport_routing_load, bound to the same bus_name, and drives the
// correlated request/requirement pair plus the information topic through the
// manifest-installed routes -- proving the routing path, not local dispatch.
void run_same_process_scenario(const std::string& plugin_path,
                               const std::string& manifest_dir) {
    std::printf("\n== routed-egress SHM: two executors, one process ==\n");

    const std::string bus_name =
        "phaseA_sp_" + std::to_string(static_cast<long long>(::getpid()));

    const std::string provider_manifest = manifest_dir + "/same_process_provider.pcl";
    const std::string consumer_manifest = manifest_dir + "/same_process_consumer.pcl";
    check(write_manifest(provider_manifest, plugin_path, bus_name, "provider", "consumer", true),
          "wrote provider routing manifest");
    check(write_manifest(consumer_manifest, plugin_path, bus_name, "consumer", "provider", false),
          "wrote consumer routing manifest");

    pcl_executor_t* provider_exec = pcl_executor_create();
    pcl_executor_t* consumer_exec = pcl_executor_create();
    check(provider_exec != nullptr, "create provider executor");
    check(consumer_exec != nullptr, "create consumer executor");

    pcl_transport_routing_t* provider_routing = nullptr;
    pcl_transport_routing_t* consumer_routing = nullptr;
    char diag[512] = "";

    pcl_status_t rc = pcl_transport_routing_load(
        provider_exec, provider_manifest.c_str(), &provider_routing, diag, sizeof(diag));
    check(rc == PCL_OK, "provider routing manifest loaded (real SHM plugin, real bus)");
    if (rc != PCL_OK) std::printf("     diag: %s\n", diag);

    rc = pcl_transport_routing_load(
        consumer_exec, consumer_manifest.c_str(), &consumer_routing, diag, sizeof(diag));
    check(rc == PCL_OK, "consumer routing manifest loaded (real SHM plugin, real bus)");
    if (rc != PCL_OK) std::printf("     diag: %s\n", diag);

    ProviderState provider_state;
    ConsumerState consumer_state;
    pcl_callbacks_t provider_cbs{};
    provider_cbs.on_configure = provider_on_configure;
    pcl_callbacks_t consumer_cbs{};
    consumer_cbs.on_configure = consumer_on_configure;

    pcl_container_t* provider = pcl_container_create(
        "routed_egress_provider", &provider_cbs, &provider_state);
    pcl_container_t* consumer = pcl_container_create(
        "routed_egress_consumer", &consumer_cbs, &consumer_state);
    check(provider != nullptr, "create provider container");
    check(consumer != nullptr, "create consumer container");
    check(pcl_container_configure(provider) == PCL_OK, "configure provider container");
    check(pcl_container_configure(consumer) == PCL_OK, "configure consumer container");
    check(pcl_container_activate(provider) == PCL_OK, "activate provider container");
    check(pcl_container_activate(consumer) == PCL_OK, "activate consumer container");
    check(pcl_executor_add(provider_exec, provider) == PCL_OK, "add provider to its executor");
    check(pcl_executor_add(consumer_exec, consumer) == PCL_OK, "add consumer to its executor");

    auto pump = [&](std::chrono::milliseconds budget) {
        const auto deadline = std::chrono::steady_clock::now() + budget;
        while (std::chrono::steady_clock::now() < deadline) {
            pcl_executor_spin_once(provider_exec, 0);
            pcl_executor_spin_once(consumer_exec, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    };

    prov::SPRRequirement_Service_Request create_request;
    create_request.sprrequest.emplace();
    check(prov::publishPimOspreySprRequirementRequest(
              consumer_state.request_pub, create_request, prov::kJsonContentType) == PCL_OK,
          "consumer published request topic over SHM route");
    pump(std::chrono::milliseconds(500));
    check(provider_state.request_seen,
          "provider observed request topic crossing the real SHM bus");
    check(consumer_state.information_seen,
          "consumer observed information topic crossing the real SHM bus");
    check(consumer_state.requirement_seen,
          "consumer observed requirement transition crossing the real SHM bus");
    check(consumer_state.requirement_count >= 1,
          "conformance: request followed by requirement transition (routed)");

    consumer_state.requirement_seen = false;
    const unsigned before_cancel = consumer_state.requirement_count;
    prov::SPRRequirement_Service_Request cancel_request;
    cancel_request.cancel = std::string("cancel-spr-same-process");
    check(prov::publishPimOspreySprRequirementRequest(
              consumer_state.request_pub, cancel_request, prov::kJsonContentType) == PCL_OK,
          "consumer published cancel on request topic over SHM route");
    pump(std::chrono::milliseconds(500));
    check(provider_state.cancel_seen,
          "provider observed cancel topic payload crossing the real SHM bus");
    check(consumer_state.requirement_seen,
          "consumer observed post-cancel requirement transition (routed)");
    check(consumer_state.requirement_count > before_cancel,
          "conformance: cancel followed by requirement transition (routed)");

    pcl_transport_routing_destroy(provider_routing);
    pcl_transport_routing_destroy(consumer_routing);
    pcl_executor_destroy(provider_exec);
    pcl_executor_destroy(consumer_exec);
    pcl_container_destroy(provider);
    pcl_container_destroy(consumer);
}

// -- Scenario 2: same routing setup, split across two OS processes ---------

int run_role(const std::string& role,
            const std::string& plugin_path,
            const std::string& bus_name,
            const std::string& manifest_path,
            const std::string& ready_file,
            const std::string& peer_ready_file,
            const std::string& result_file,
            uint32_t timeout_ms) {
    const bool is_provider = (role == "provider");
    register_json_codec();

    if (!write_manifest(manifest_path, plugin_path, bus_name,
                        is_provider ? "provider" : "consumer",
                        is_provider ? "consumer" : "provider", is_provider)) {
        return 2;
    }

    pcl_executor_t* exec = pcl_executor_create();
    if (!exec) return 2;

    pcl_transport_routing_t* routing = nullptr;
    char diag[512] = "";
    if (pcl_transport_routing_load(exec, manifest_path.c_str(), &routing, diag,
                                   sizeof(diag)) != PCL_OK) {
        std::fprintf(stderr, "[%s] routing load failed: %s\n", role.c_str(), diag);
        pcl_executor_destroy(exec);
        return 2;
    }

    ProviderState provider_state;
    ConsumerState consumer_state;
    pcl_callbacks_t cbs{};
    cbs.on_configure = is_provider ? provider_on_configure : consumer_on_configure;
    void* user_data = is_provider ? static_cast<void*>(&provider_state)
                                  : static_cast<void*>(&consumer_state);

    pcl_container_t* container = pcl_container_create(
        is_provider ? "routed_egress_ip_provider" : "routed_egress_ip_consumer",
        &cbs, user_data);
    if (!container ||
        pcl_container_configure(container) != PCL_OK ||
        pcl_container_activate(container) != PCL_OK ||
        pcl_executor_add(exec, container) != PCL_OK) {
        if (container) pcl_container_destroy(container);
        pcl_transport_routing_destroy(routing);
        pcl_executor_destroy(exec);
        return 2;
    }

    write_text(ready_file, "ready");

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    if (is_provider) {
        while (std::chrono::steady_clock::now() < deadline &&
              !(provider_state.request_seen && provider_state.cancel_seen)) {
            pcl_executor_spin_once(exec, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        const bool ok = provider_state.request_seen && provider_state.cancel_seen;
        if (ok) write_text(result_file, "request_seen=1 cancel_seen=1");
        pcl_executor_remove(exec, container);
        pcl_container_destroy(container);
        pcl_transport_routing_destroy(routing);
        pcl_executor_destroy(exec);
        return ok ? 0 : 3;
    }

    // Consumer: wait for the provider's ready-file (so its subscriber is
    // installed before we publish), then drive the create/cancel sequence.
    while (std::chrono::steady_clock::now() < deadline && !file_exists(peer_ready_file)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!file_exists(peer_ready_file)) {
        pcl_executor_remove(exec, container);
        pcl_container_destroy(container);
        pcl_transport_routing_destroy(routing);
        pcl_executor_destroy(exec);
        return 3;
    }

    prov::SPRRequirement_Service_Request create_request;
    create_request.sprrequest.emplace();
    prov::publishPimOspreySprRequirementRequest(
        consumer_state.request_pub, create_request, prov::kJsonContentType);

    while (std::chrono::steady_clock::now() < deadline &&
          !(consumer_state.information_seen && consumer_state.requirement_count >= 1)) {
        pcl_executor_spin_once(exec, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const unsigned before_cancel = consumer_state.requirement_count;

    prov::SPRRequirement_Service_Request cancel_request;
    cancel_request.cancel = std::string("cancel-spr-cross-process");
    prov::publishPimOspreySprRequirementRequest(
        consumer_state.request_pub, cancel_request, prov::kJsonContentType);

    while (std::chrono::steady_clock::now() < deadline &&
          !(consumer_state.requirement_count > before_cancel)) {
        pcl_executor_spin_once(exec, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const bool ok = consumer_state.information_seen &&
                    consumer_state.requirement_count > before_cancel;
    if (ok) {
        write_text(result_file, "information_seen=1 requirement_count=" +
                                     std::to_string(consumer_state.requirement_count));
    }
    pcl_executor_remove(exec, container);
    pcl_container_destroy(container);
    pcl_transport_routing_destroy(routing);
    pcl_executor_destroy(exec);
    return ok ? 0 : 3;
}

#if !defined(_WIN32)
struct ChildProcess {
    pid_t pid = -1;

    bool spawn(const std::string& self_path, const std::vector<std::string>& args) {
        pid = fork();
        if (pid < 0) return false;
        if (pid == 0) {
            std::vector<char*> argv;
            argv.push_back(const_cast<char*>(self_path.c_str()));
            for (const auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
            argv.push_back(nullptr);
            execv(self_path.c_str(), argv.data());
            _exit(127);
        }
        return true;
    }

    int wait(uint32_t timeout_ms) {
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        for (;;) {
            int status = 0;
            const pid_t r = waitpid(pid, &status, WNOHANG);
            if (r == pid) {
                pid = -1;
                return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
            }
            if (std::chrono::steady_clock::now() >= deadline) return -2;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    ~ChildProcess() {
        if (pid > 0) {
            kill(pid, SIGTERM);
            int status = 0;
            waitpid(pid, &status, 0);
        }
    }
};

void run_cross_process_scenario(const std::string& self_path,
                                const std::string& plugin_path,
                                const std::string& manifest_dir) {
    std::printf("\n== routed-egress SHM: two executors, two processes ==\n");

    const long long run_id = static_cast<long long>(::getpid());
    const std::string bus_name = "phaseA_xp_" + std::to_string(run_id);
    const std::string provider_manifest = manifest_dir + "/cross_process_provider.pcl";
    const std::string consumer_manifest = manifest_dir + "/cross_process_consumer.pcl";
    const std::string provider_ready = manifest_dir + "/provider.ready";
    const std::string consumer_ready = manifest_dir + "/consumer.ready";
    const std::string provider_result = manifest_dir + "/provider.result";
    const std::string consumer_result = manifest_dir + "/consumer.result";
    std::remove(provider_ready.c_str());
    std::remove(consumer_ready.c_str());
    std::remove(provider_result.c_str());
    std::remove(consumer_result.c_str());

    ChildProcess provider;
    ChildProcess consumer;
    check(provider.spawn(self_path,
                         {"--role=provider", "--plugin=" + plugin_path,
                          "--bus=" + bus_name, "--manifest=" + provider_manifest,
                          "--ready-file=" + provider_ready,
                          "--peer-ready-file=" + consumer_ready,
                          "--result-file=" + provider_result, "--timeout-ms=5000"}),
          "spawned provider process");
    check(consumer.spawn(self_path,
                         {"--role=consumer", "--plugin=" + plugin_path,
                          "--bus=" + bus_name, "--manifest=" + consumer_manifest,
                          "--ready-file=" + consumer_ready,
                          "--peer-ready-file=" + provider_ready,
                          "--result-file=" + consumer_result, "--timeout-ms=5000"}),
          "spawned consumer process");

    const int provider_rc = provider.wait(6000);
    const int consumer_rc = consumer.wait(6000);
    check(provider_rc == 0, "provider process exited 0 (observed request + cancel over SHM)");
    check(consumer_rc == 0, "consumer process exited 0 (observed information + requirement transitions over SHM)");

    std::string provider_result_text;
    std::string consumer_result_text;
    check(read_text(provider_result, &provider_result_text) && !provider_result_text.empty(),
          "provider cross-process result file written");
    check(read_text(consumer_result, &consumer_result_text) && !consumer_result_text.empty(),
          "consumer cross-process result file written");
    std::printf("     provider result: %s\n", provider_result_text.c_str());
    std::printf("     consumer result: %s\n", consumer_result_text.c_str());
}
#endif  // !_WIN32

}  // namespace

int main(int argc, char** argv) {
    const std::string role = get_option(argc, argv, "--role=");
    if (!role.empty()) {
        // Re-exec'd child process: run a single role's executor and exit.
        // (run_role registers the JSON codec itself.)
        return run_role(role,
                        get_option(argc, argv, "--plugin="),
                        get_option(argc, argv, "--bus="),
                        get_option(argc, argv, "--manifest="),
                        get_option(argc, argv, "--ready-file="),
                        get_option(argc, argv, "--peer-ready-file="),
                        get_option(argc, argv, "--result-file="),
                        static_cast<uint32_t>(std::stoul(
                            get_option(argc, argv, "--timeout-ms="))));
    }

    if (argc < 3) {
        std::fprintf(stderr,
                     "usage: %s <shm-plugin.so> <manifest-scratch-dir>\n", argv[0]);
        return 2;
    }
    const std::string plugin_path = argv[1];
    const std::string manifest_dir = argv[2];

    register_json_codec();

    run_same_process_scenario(plugin_path, manifest_dir);
#if !defined(_WIN32)
    run_cross_process_scenario(argv[0], plugin_path, manifest_dir);
#else
    std::printf("\n== routed-egress SHM: two processes (skipped on Windows in this harness) ==\n");
#endif

    pcl_codec_registry_clear(pcl_codec_registry_default());
    std::printf("\n%s (%d failure(s))\n", g_failures ? "FAIL" : "PASS", g_failures);
    return g_failures ? 1 : 0;
}
