// \file agra_udp_proof_test.cpp
// \brief Phase D of doc/plans/PYRAMID/agra_pubsub_shm_udp_proving_plan.md --
//        the UDP proof, including the fail-closed negative gate.
//
// Two scenarios against the A-GRA example contract (pim/agra_example/):
//   1. Negative gate: a manifest routing agra.ma_action.entity (the
//      correlated pair's RELIABLE-stamped topic) over the real
//      libpcl_transport_udp_plugin.so (BEST_EFFORT-declared) must fail
//      closed at pcl_transport_routing_load -- the first exercise of the
//      transport-codec-plugin-system's §5.4 reconciliation rule against a
//      real BEST_EFFORT transport. Compose-time only, single process.
//   2. Positive proof: agra.ma_action_plan.information (BEST_EFFORT-
//      stamped -- the contractually legitimate floor for this transport)
//      round-trips over real UDP datagrams between two processes on
//      per-run-unique loopback ports: MA publishes MA_ActionPlan objects,
//      C2 subscribes and decodes them.
//
// UDP's peer_id is a config-time label the transport itself stamps on
// every post_remote_incoming call (pcl_transport_udp.c: ctx->peer_id,
// "default" unless overridden by the "peer_id" config key) -- unlike SHM's
// shared multi-participant bus, UDP here is strictly point-to-point, so
// the local transport's manifest peer alias and its ingress peer_id can
// simply be the same string; no counterpart-alias convention is needed.

#include "pyramid_services_agra_c2_station_consumed.hpp"
#include "pyramid_services_agra_mission_autonomy_provided.hpp"

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport_routing.h>
}

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

namespace ma = pyramid::components::agra::mission_autonomy::services::provided;
namespace c2 = pyramid::components::agra::c2_station::services::consumed;

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
    out->assign((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    return true;
}

bool file_exists(const std::string& path) {
    std::ifstream in(path);
    return in.good();
}

uint16_t port_for(long long run_id, int offset) {
    // Keep well clear of the well-known range and spread runs apart.
    return static_cast<uint16_t>(20000 + ((run_id * 2 + offset) % 20000));
}

// -- Scenario 1: negative gate (compose-time only, one process) -----------

void run_negative_gate(const std::string& udp_plugin_path, const std::string& scratch_dir) {
    std::printf("\n== agra UDP proof: negative gate (RELIABLE topic over BEST_EFFORT UDP) ==\n");

    const long long run_id = static_cast<long long>(::getpid());
    const uint16_t local_port = port_for(run_id, 0);
    const uint16_t remote_port = port_for(run_id, 1);

    const std::string manifest_path = scratch_dir + "/negative_gate.pcl";
    {
        std::ofstream out(manifest_path, std::ios::trunc);
        out << "transport peer_udp " << udp_plugin_path
            << " {\"remote_host\":\"127.0.0.1\",\"remote_port\":" << remote_port
            << ",\"local_port\":" << local_port << ",\"peer_id\":\"peer_udp\"}\n";
        // agra.ma_action.entity is stamped RELIABLE in the contract
        // (pim/agra_example's provided proto, §3.3); UDP only ever offers
        // BEST_EFFORT. This route must fail closed.
        out << "route " << ma::kTopicAgraMaActionEntity
            << " publisher peer_udp reliable\n";
    }

    pcl_executor_t* exec = pcl_executor_create();
    check(exec != nullptr, "create executor");

    pcl_transport_routing_t* routing = nullptr;
    char diag[512] = "";
    const pcl_status_t rc = pcl_transport_routing_load(
        exec, manifest_path.c_str(), &routing, diag, sizeof(diag));

    check(rc != PCL_OK,
          "RELIABLE route over the real BEST_EFFORT UDP plugin fails closed");
    check(rc == PCL_ERR_STATE,
          "failure is PCL_ERR_STATE (QoS floor unmet), not a config/parse error");
    const std::string diag_text(diag);
    check(diag_text.find("reliable") != std::string::npos,
          "diagnostic names the required reliability ('reliable')");
    check(diag_text.find("best_effort") != std::string::npos,
          "diagnostic names the offered reliability ('best_effort')");
    check(diag_text.find(ma::kTopicAgraMaActionEntity) != std::string::npos,
          "diagnostic names the offending endpoint");
    std::printf("     diag: %s\n", diag);

    check(pcl_transport_routing_transport_count(routing) == 0,
          "failed load leaves nothing registered (rollback)");

    pcl_transport_routing_destroy(routing);
    pcl_executor_destroy(exec);
}

// -- Scenario 2: positive proof, information topic over real UDP ----------

struct MaState {
    pcl_port_t* info_pub = nullptr;
};

struct C2State {
    std::vector<pyramid::domain_model::agra::MA_ActionPlan> received;
};

void c2_on_information(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* state = static_cast<C2State*>(ud);
    c2::MAActionPlan_Service_Information payload;
    if (!c2::decodeAgraMaActionPlanInformation(msg, &payload)) return;
    if (payload.ma_action_plan) state->received.push_back(*payload.ma_action_plan);
}

pcl_status_t ma_on_configure(pcl_container_t* c, void* ud) {
    auto* state = static_cast<MaState*>(ud);
    state->info_pub = pcl_container_add_publisher(
        c, ma::kTopicAgraMaActionPlanInformation, ma::kJsonContentType);
    return state->info_pub ? PCL_OK : PCL_ERR_NOMEM;
}

pcl_status_t c2_on_configure(pcl_container_t* c, void* ud) {
    auto* state = static_cast<C2State*>(ud);
    pcl_port_t* sub = c2::subscribeAgraMaActionPlanInformation(
        c, c2_on_information, state, c2::kJsonContentType);
    return sub ? PCL_OK : PCL_ERR_NOMEM;
}

int run_role(const std::string& role,
            const std::string& udp_plugin_path,
            const std::string& json_plugin_path,
            uint16_t own_port,
            uint16_t peer_port,
            const std::string& ready_file,
            const std::string& peer_ready_file,
            const std::string& result_file,
            uint32_t timeout_ms) {
    const bool is_ma = (role == "ma");

    pcl_plugin_handle_t* json_handle = nullptr;
    if (pcl_plugin_load_codec(json_plugin_path.c_str(),
                              "{\"source\":\"agra_udp_proof_test\"}",
                              pcl_codec_registry_default(), &json_handle) != PCL_OK) {
        return 2;
    }

    const std::string manifest_path = ready_file + ".pcl";
    {
        std::ofstream out(manifest_path, std::ios::trunc);
        out << "transport peer_udp " << udp_plugin_path
            << " {\"remote_host\":\"127.0.0.1\",\"remote_port\":" << peer_port
            << ",\"local_port\":" << own_port << ",\"peer_id\":\"peer_udp\"}\n";
        const char* kind = is_ma ? "publisher" : "subscriber";
        out << "route " << ma::kTopicAgraMaActionPlanInformation
            << " " << kind << " peer_udp best_effort\n";
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

    MaState ma_state;
    C2State c2_state;
    pcl_callbacks_t cbs{};
    cbs.on_configure = is_ma ? ma_on_configure : c2_on_configure;
    void* user_data = is_ma ? static_cast<void*>(&ma_state) : static_cast<void*>(&c2_state);

    pcl_container_t* container = pcl_container_create(
        is_ma ? "agra_udp_ma" : "agra_udp_c2", &cbs, user_data);
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

    if (is_ma) {
        while (std::chrono::steady_clock::now() < deadline && !file_exists(peer_ready_file)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        // UDP is unreliable by nature (this is the point): publish the same
        // three plan objects a few times so the proof isn't sensitive to an
        // occasional dropped datagram on loopback.
        bool all_ok = true;
        for (int attempt = 0; attempt < 5; ++attempt) {
            for (int i = 0; i < 3; ++i) {
                pyramid::domain_model::agra::MA_ActionPlan plan;
                plan.id = "plan-" + std::to_string(i);
                plan.plan_id = "plan-" + std::to_string(i);
                plan.action_id = "action-1";
                plan.summary = "candidate route " + std::to_string(i);
                ma::MAActionPlan_Service_Information wrapper;
                wrapper.ma_action_plan = plan;
                const pcl_status_t rc = ma::publishAgraMaActionPlanInformation(
                    ma_state.info_pub, wrapper, ma::kJsonContentType);
                all_ok = all_ok && (rc == PCL_OK);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        check(all_ok, "MA published information topic over real UDP datagrams");
        if (all_ok) write_text(result_file, "published=1");
        pcl_executor_remove(exec, container);
        pcl_container_destroy(container);
        pcl_transport_routing_destroy(routing);
        pcl_executor_destroy(exec);
        pcl_plugin_unload(json_handle);
        pcl_codec_registry_clear(pcl_codec_registry_default());
        return all_ok ? 0 : 3;
    }

    while (std::chrono::steady_clock::now() < deadline && !file_exists(peer_ready_file)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    while (std::chrono::steady_clock::now() < deadline && c2_state.received.size() < 3) {
        pcl_executor_spin_once(exec, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    bool summary_ok = c2_state.received.size() >= 3;
    if (summary_ok) {
        for (const auto& plan : c2_state.received) {
            if (plan.action_id != "action-1" || plan.summary.find("candidate route") ==
                                                     std::string::npos) {
                summary_ok = false;
                break;
            }
        }
    }
    if (summary_ok) {
        write_text(result_file,
                   "received=" + std::to_string(c2_state.received.size()));
    }
    pcl_executor_remove(exec, container);
    pcl_container_destroy(container);
    pcl_transport_routing_destroy(routing);
    pcl_executor_destroy(exec);
    pcl_plugin_unload(json_handle);
    pcl_codec_registry_clear(pcl_codec_registry_default());
    return summary_ok ? 0 : 3;
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

void run_positive_proof(const std::string& self_path,
                        const std::string& udp_plugin_path,
                        const std::string& ma_json_plugin_path,
                        const std::string& c2_json_plugin_path,
                        const std::string& scratch_dir) {
    std::printf("\n== agra UDP proof: information topic, two processes, real datagrams ==\n");

    const long long run_id = static_cast<long long>(::getpid());
    const uint16_t ma_port = port_for(run_id, 10);
    const uint16_t c2_port = port_for(run_id, 11);
    const std::string ma_ready = scratch_dir + "/udp_ma.ready";
    const std::string c2_ready = scratch_dir + "/udp_c2.ready";
    const std::string ma_result = scratch_dir + "/udp_ma.result";
    const std::string c2_result = scratch_dir + "/udp_c2.result";
    for (const auto& f : {ma_ready, c2_ready, ma_result, c2_result}) std::remove(f.c_str());

    ChildProcess ma_proc;
    ChildProcess c2_proc;
    check(ma_proc.spawn(self_path,
                        {"--role=ma", "--udp-plugin=" + udp_plugin_path,
                         "--json-plugin=" + ma_json_plugin_path,
                         "--own-port=" + std::to_string(ma_port),
                         "--peer-port=" + std::to_string(c2_port),
                         "--ready-file=" + ma_ready, "--peer-ready-file=" + c2_ready,
                         "--result-file=" + ma_result, "--timeout-ms=6000"}),
          "spawned MA process (UDP publisher)");
    check(c2_proc.spawn(self_path,
                        {"--role=c2", "--udp-plugin=" + udp_plugin_path,
                         "--json-plugin=" + c2_json_plugin_path,
                         "--own-port=" + std::to_string(c2_port),
                         "--peer-port=" + std::to_string(ma_port),
                         "--ready-file=" + c2_ready, "--peer-ready-file=" + ma_ready,
                         "--result-file=" + c2_result, "--timeout-ms=6000"}),
          "spawned C2 process (UDP subscriber)");

    const int ma_rc = ma_proc.wait(7000);
    const int c2_rc = c2_proc.wait(7000);
    check(ma_rc == 0, "MA process exited 0 (published over real UDP)");
    check(c2_rc == 0,
          "C2 process exited 0 (decoded >=3 MA_ActionPlan objects over real UDP, "
          "content correct)");

    std::string ma_result_text;
    std::string c2_result_text;
    check(read_text(ma_result, &ma_result_text) && !ma_result_text.empty(),
          "MA cross-process result file written");
    check(read_text(c2_result, &c2_result_text) && !c2_result_text.empty(),
          "C2 cross-process result file written");
    std::printf("     MA result: %s\n", ma_result_text.c_str());
    std::printf("     C2 result: %s\n", c2_result_text.c_str());
}
#endif  // !_WIN32

}  // namespace

int main(int argc, char** argv) {
    const std::string role = get_option(argc, argv, "--role=");
    if (!role.empty()) {
        return run_role(role,
                        get_option(argc, argv, "--udp-plugin="),
                        get_option(argc, argv, "--json-plugin="),
                        static_cast<uint16_t>(std::stoul(
                            get_option(argc, argv, "--own-port="))),
                        static_cast<uint16_t>(std::stoul(
                            get_option(argc, argv, "--peer-port="))),
                        get_option(argc, argv, "--ready-file="),
                        get_option(argc, argv, "--peer-ready-file="),
                        get_option(argc, argv, "--result-file="),
                        static_cast<uint32_t>(std::stoul(
                            get_option(argc, argv, "--timeout-ms="))));
    }

#if defined(_WIN32)
    std::fprintf(stderr, "agra_udp_proof_test: cross-process scenario not run on Windows in this harness\n");
    return 2;
#else
    if (argc < 5) {
        std::fprintf(stderr,
                     "usage: %s <udp-plugin.so> <ma-json-codec.so> <c2-json-codec.so> <scratch-dir>\n",
                     argv[0]);
        return 2;
    }
    const std::string udp_plugin_path = argv[1];
    const std::string ma_json_plugin_path = argv[2];
    const std::string c2_json_plugin_path = argv[3];
    const std::string scratch_dir = argv[4];

    run_negative_gate(udp_plugin_path, scratch_dir);
    run_positive_proof(argv[0], udp_plugin_path, ma_json_plugin_path,
                       c2_json_plugin_path, scratch_dir);

    std::printf("\n%s (%d failure(s))\n", g_failures ? "FAIL" : "PASS", g_failures);
    return g_failures ? 1 : 0;
#endif
}
