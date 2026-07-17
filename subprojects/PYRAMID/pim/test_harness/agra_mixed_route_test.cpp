// \file agra_mixed_route_test.cpp
// \brief Phase E (plan-terminal) of
//        doc/plans/PYRAMID/agra_pubsub_shm_udp_proving_plan.md -- the A-GRA
//        C2/MS split in miniature: MissionAutonomy and C2Station each load
//        TWO transports from ONE routing manifest --
//
//          transport pair_shm  libpcl_transport_shared_memory_plugin.so {...}
//          transport info_udp  libpcl_transport_udp_plugin.so           {...}
//          route agra.ma_action.request           subscriber pair_shm  reliable
//          route agra.ma_action.entity       publisher  pair_shm  reliable
//          route agra.ma_action_plan.information  publisher  info_udp best_effort
//
// -- and run the complete worked-example sequence: action tasking and
// status over the SHM correlated pair (Phase C), plan publication over UDP
// (Phase D), cancel, all sequence checks, both codecs. The UDP fail-closed
// negative gate (Phase D) is replayed here too (compose-time only, against
// a throwaway executor) so the terminal harness carries its own evidence
// rather than depending on Phase D having been run separately.

#include "pyramid_services_agra_c2_station_consumed.hpp"
#include "pyramid_services_agra_mission_autonomy_provided.hpp"

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport_routing.h>
}

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
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
namespace dm_common = pyramid::domain_model::common;
namespace dm_agra = pyramid::domain_model::agra;

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
    return static_cast<uint16_t>(20000 + ((run_id * 2 + offset) % 20000));
}

constexpr const char* kActionOne = "action-1";
constexpr const char* kActionTwo = "action-2";

// -- Negative gate replay (Phase D, compose-time only) ---------------------

void run_negative_gate_replay(const std::string& udp_plugin_path,
                              const std::string& scratch_dir) {
    std::printf("\n== agra mixed-route: negative gate replay (RELIABLE topic over BEST_EFFORT UDP) ==\n");

    const long long run_id = static_cast<long long>(::getpid());
    const uint16_t local_port = port_for(run_id, 900);
    const uint16_t remote_port = port_for(run_id, 901);
    const std::string manifest_path = scratch_dir + "/negative_gate_replay.pcl";
    {
        std::ofstream out(manifest_path, std::ios::trunc);
        out << "transport peer_udp " << udp_plugin_path
            << " {\"remote_host\":\"127.0.0.1\",\"remote_port\":" << remote_port
            << ",\"local_port\":" << local_port << ",\"peer_id\":\"peer_udp\"}\n";
        out << "route " << ma::kTopicAgraMaActionEntity
            << " publisher peer_udp reliable\n";
    }

    pcl_executor_t* exec = pcl_executor_create();
    pcl_transport_routing_t* routing = nullptr;
    char diag[512] = "";
    const pcl_status_t rc = pcl_transport_routing_load(
        exec, manifest_path.c_str(), &routing, diag, sizeof(diag));

    check(rc == PCL_ERR_STATE,
          "RELIABLE route over real BEST_EFFORT UDP still fails closed (replayed)");
    const std::string diag_text(diag);
    check(diag_text.find("reliable") != std::string::npos &&
              diag_text.find("best_effort") != std::string::npos,
          "diagnostic still names both reliabilities");

    pcl_transport_routing_destroy(routing);
    pcl_executor_destroy(exec);
}

// -- Mixed-route manifest: two transports, three routes ---------------------
//
// pair_shm's peer alias follows Phase A/C's counterpart-participant-id
// convention (required for a shared multi-participant bus); info_udp's
// peer alias is an arbitrary shared label (Phase D found UDP's peer_id is
// a purely local config value, not derived from the sender, since this
// transport is strictly point-to-point).
bool write_manifest(const std::string& path,
                    const std::string& shm_plugin_path,
                    const std::string& udp_plugin_path,
                    const std::string& bus_name,
                    const std::string& own_participant_id,
                    const std::string& peer_participant_id,
                    uint16_t own_port,
                    uint16_t peer_port,
                    bool is_ma) {
    std::ofstream out(path, std::ios::trunc);
    if (!out) return false;
    const std::string& shm_peer_alias = peer_participant_id;
    out << "transport " << shm_peer_alias << " " << shm_plugin_path
        << " {\"bus_name\":\"" << bus_name << "\",\"participant_id\":\""
        << own_participant_id << "\"}\n";
    out << "transport info_udp " << udp_plugin_path
        << " {\"remote_host\":\"127.0.0.1\",\"remote_port\":" << peer_port
        << ",\"local_port\":" << own_port << ",\"peer_id\":\"info_udp\"}\n";
    if (is_ma) {
        out << "route " << ma::kTopicAgraMaActionRequest
            << " subscriber " << shm_peer_alias << " reliable\n";
        out << "route " << ma::kTopicAgraMaActionEntity
            << " publisher " << shm_peer_alias << " reliable\n";
        out << "route " << ma::kTopicAgraMaActionPlanInformation
            << " publisher info_udp best_effort\n";
    } else {
        out << "route " << c2::kTopicAgraMaActionRequest
            << " publisher " << shm_peer_alias << " reliable\n";
        out << "route " << c2::kTopicAgraMaActionEntity
            << " subscriber " << shm_peer_alias << " reliable\n";
        out << "route " << c2::kTopicAgraMaActionPlanInformation
            << " subscriber info_udp best_effort\n";
    }
    return out.good();
}

struct RequirementLog {
    std::vector<dm_common::Progress> progress;
    std::vector<dm_common::AcceptanceState> acceptance;
};

// -- MissionAutonomy (provider) role ---------------------------------------

struct MaState {
    pcl_port_t* requirement_pub = nullptr;
    pcl_port_t* info_pub = nullptr;
    std::string content_type;
    bool saw_action_one_create = false;
    bool saw_action_two_cancel = false;
};

void ma_publish_transition(MaState* state, const std::string& id,
                           dm_common::Progress progress,
                           dm_common::AcceptanceState acceptance) {
    dm_common::Requirement req;
    req.id = id;
    req.status.id = id;
    req.status.status = progress;
    req.status.acceptance = acceptance;
    ma::MAAction_Service_Entity wrapper;
    wrapper.ma_action_status = req;
    const pcl_status_t rc = ma::publishAgraMaActionEntity(
        state->requirement_pub, wrapper, state->content_type.c_str());
    check(rc == PCL_OK, "MA published entity transition over the SHM route");
}

void ma_on_request(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* state = static_cast<MaState*>(ud);
    ma::MAAction_Service_Request request;
    if (!ma::decodeAgraMaActionRequest(msg, &request)) {
        check(false, "MA decoded request-topic payload received over the SHM route");
        return;
    }
    if (request.ma_action) {
        state->saw_action_one_create = true;
        const std::string id = request.ma_action->id;
        ma_publish_transition(state, id, dm_common::Progress::NotStarted,
                              dm_common::AcceptanceState::Received);
        ma_publish_transition(state, id, dm_common::Progress::InProgress,
                              dm_common::AcceptanceState::Received);
        ma_publish_transition(state, id, dm_common::Progress::Completed,
                              dm_common::AcceptanceState::Received);

        // The plan publication (Data-1) rides the UDP leg, per §3.3/§4
        // Phase E -- published once the tasking is underway.
        dm_agra::MA_ActionPlan plan;
        plan.id = "plan-1";
        plan.plan_id = "plan-1";
        plan.action_id = id;
        plan.summary = "candidate route for " + id;
        ma::MAActionPlan_Service_Information info_wrapper;
        info_wrapper.ma_action_plan = plan;
        for (int attempt = 0; attempt < 5; ++attempt) {
            ma::publishAgraMaActionPlanInformation(state->info_pub, info_wrapper,
                                                   state->content_type.c_str());
        }
    }
    if (request.cancel) {
        state->saw_action_two_cancel = true;
        ma_publish_transition(state, *request.cancel, dm_common::Progress::Cancelled,
                              dm_common::AcceptanceState::Received);
    }
}

pcl_status_t ma_on_configure(pcl_container_t* c, void* ud) {
    auto* state = static_cast<MaState*>(ud);
    state->requirement_pub = pcl_container_add_publisher(
        c, ma::kTopicAgraMaActionEntity, state->content_type.c_str());
    state->info_pub = pcl_container_add_publisher(
        c, ma::kTopicAgraMaActionPlanInformation, state->content_type.c_str());
    pcl_port_t* sub = ma::subscribeAgraMaActionRequest(
        c, ma_on_request, state, state->content_type.c_str());
    return (state->requirement_pub && state->info_pub && sub) ? PCL_OK : PCL_ERR_NOMEM;
}

// -- C2Station (consumer) role ----------------------------------------------

struct C2State {
    pcl_port_t* request_pub = nullptr;
    std::string content_type;
    std::map<std::string, RequirementLog> log;
    std::vector<dm_agra::MA_ActionPlan> info_received;
};

void c2_on_requirement(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* state = static_cast<C2State*>(ud);
    c2::MAAction_Service_Entity payload;
    if (!c2::decodeAgraMaActionEntity(msg, &payload)) {
        check(false, "C2 decoded entity-topic payload received over the SHM route");
        return;
    }
    if (!payload.ma_action_status) return;
    const auto& r = *payload.ma_action_status;
    RequirementLog& entry = state->log[r.id];
    entry.progress.push_back(r.status.status);
    entry.acceptance.push_back(r.status.acceptance);
}

void c2_on_information(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* state = static_cast<C2State*>(ud);
    c2::MAActionPlan_Service_Information payload;
    if (!c2::decodeAgraMaActionPlanInformation(msg, &payload)) return;
    if (payload.ma_action_plan) state->info_received.push_back(*payload.ma_action_plan);
}

pcl_status_t c2_on_configure(pcl_container_t* c, void* ud) {
    auto* state = static_cast<C2State*>(ud);
    state->request_pub = pcl_container_add_publisher(
        c, c2::kTopicAgraMaActionRequest, state->content_type.c_str());
    pcl_port_t* req_sub = c2::subscribeAgraMaActionEntity(
        c, c2_on_requirement, state, state->content_type.c_str());
    pcl_port_t* info_sub = c2::subscribeAgraMaActionPlanInformation(
        c, c2_on_information, state, state->content_type.c_str());
    return (state->request_pub && req_sub && info_sub) ? PCL_OK : PCL_ERR_NOMEM;
}

// -- Per-process role runner -------------------------------------------------

int run_role(const std::string& role,
            const std::string& shm_plugin_path,
            const std::string& udp_plugin_path,
            const std::string& json_plugin_path,
            const std::string& flat_plugin_path,
            const std::string& content_type,
            const std::string& bus_name,
            uint16_t own_port,
            uint16_t peer_port,
            const std::string& manifest_path,
            const std::string& ready_file,
            const std::string& peer_ready_file,
            const std::string& result_file,
            uint32_t timeout_ms) {
    const bool is_ma = (role == "ma");

    pcl_plugin_handle_t* json_handle = nullptr;
    if (pcl_plugin_load_codec(json_plugin_path.c_str(),
                              "{\"source\":\"agra_mixed_route_test\"}",
                              pcl_codec_registry_default(), &json_handle) != PCL_OK) {
        return 2;
    }
    pcl_plugin_handle_t* flat_handle = nullptr;
    if (content_type == ma::kFlatBuffersContentType) {
        if (pcl_plugin_load_codec(flat_plugin_path.c_str(),
                                  "{\"source\":\"agra_mixed_route_test\"}",
                                  pcl_codec_registry_default(), &flat_handle) != PCL_OK) {
            return 2;
        }
    }

    if (!write_manifest(manifest_path, shm_plugin_path, udp_plugin_path, bus_name,
                        is_ma ? "ma" : "c2", is_ma ? "c2" : "ma", own_port, peer_port,
                        is_ma)) {
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
    const size_t transport_count = pcl_transport_routing_transport_count(routing);
    check(transport_count == 2, (role + ": exactly two transports loaded from one manifest").c_str());

    MaState ma_state;
    C2State c2_state;
    ma_state.content_type = content_type;
    c2_state.content_type = content_type;

    pcl_callbacks_t cbs{};
    cbs.on_configure = is_ma ? ma_on_configure : c2_on_configure;
    void* user_data = is_ma ? static_cast<void*>(&ma_state) : static_cast<void*>(&c2_state);

    pcl_container_t* container = pcl_container_create(
        is_ma ? "agra_mixed_ma" : "agra_mixed_c2", &cbs, user_data);
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

    auto unload_and_return = [&](bool ok) {
        pcl_executor_remove(exec, container);
        pcl_container_destroy(container);
        pcl_transport_routing_destroy(routing);
        pcl_executor_destroy(exec);
        if (flat_handle) pcl_plugin_unload(flat_handle);
        pcl_plugin_unload(json_handle);
        pcl_codec_registry_clear(pcl_codec_registry_default());
        return ok ? 0 : 3;
    };

    if (is_ma) {
        while (std::chrono::steady_clock::now() < deadline &&
              !(ma_state.saw_action_one_create && ma_state.saw_action_two_cancel)) {
            pcl_executor_spin_once(exec, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        const bool ok = ma_state.saw_action_one_create && ma_state.saw_action_two_cancel;
        if (ok) {
            write_text(result_file, "create=1 cancel=1 transports=" +
                                         std::to_string(transport_count));
        }
        return unload_and_return(ok);
    }

    while (std::chrono::steady_clock::now() < deadline && !file_exists(peer_ready_file)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    c2::MAAction_Service_Request create_request;
    dm_agra::MA_Action action_one;
    action_one.id = kActionOne;
    action_one.action_type = dm_agra::ActionType::FindSearch;
    action_one.target_object = "AOI-1";
    action_one.action_constraints = {"EMCON_PASSIVE", "KEEP_OUT_WEST_OF_AOI1"};
    create_request.ma_action = action_one;
    c2::publishAgraMaActionRequest(c2_state.request_pub, create_request,
                                   content_type.c_str());

    const auto has_transitions = [&](const std::string& id, size_t n) {
        auto it = c2_state.log.find(id);
        return it != c2_state.log.end() && it->second.progress.size() >= n;
    };
    while (std::chrono::steady_clock::now() < deadline &&
          !(has_transitions(kActionOne, 3) && !c2_state.info_received.empty())) {
        pcl_executor_spin_once(exec, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    c2::MAAction_Service_Request cancel_request;
    cancel_request.cancel = std::string(kActionTwo);
    c2::publishAgraMaActionRequest(c2_state.request_pub, cancel_request,
                                   content_type.c_str());

    while (std::chrono::steady_clock::now() < deadline && !has_transitions(kActionTwo, 1)) {
        pcl_executor_spin_once(exec, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const RequirementLog& log_one = c2_state.log[kActionOne];
    const RequirementLog& log_two = c2_state.log[kActionTwo];
    const bool shm_ok =
        log_one.progress.size() >= 3 &&
        log_one.progress.back() == dm_common::Progress::Completed &&
        log_one.acceptance.front() == dm_common::AcceptanceState::Received &&
        std::find(log_one.progress.begin(), log_one.progress.end(),
                  dm_common::Progress::Cancelled) == log_one.progress.end() &&
        log_two.progress.size() >= 1 &&
        log_two.progress.back() == dm_common::Progress::Cancelled &&
        std::find(log_two.progress.begin(), log_two.progress.end(),
                  dm_common::Progress::Completed) == log_two.progress.end();
    const bool udp_ok = !c2_state.info_received.empty() &&
                        c2_state.info_received.front().action_id == kActionOne;

    check(shm_ok, "C2: SHM-routed correlated pair conformance (RECEIVED/IN_PROGRESS/"
                  "COMPLETED + CANCELLED, correlated by id, non-conflated)");
    check(udp_ok, "C2: UDP-routed information topic received with correct correlation");

    const bool ok = shm_ok && udp_ok;
    if (ok) {
        write_text(result_file,
                   "action1_transitions=" + std::to_string(log_one.progress.size()) +
                       " action2_transitions=" + std::to_string(log_two.progress.size()) +
                       " info_received=" + std::to_string(c2_state.info_received.size()) +
                       " transports=" + std::to_string(transport_count));
    }
    return unload_and_return(ok);
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

void run_scenario(const std::string& self_path,
                  const std::string& shm_plugin_path,
                  const std::string& udp_plugin_path,
                  const std::string& ma_json_plugin_path,
                  const std::string& c2_json_plugin_path,
                  const std::string& ma_flat_plugin_path,
                  const std::string& c2_flat_plugin_path,
                  const std::string& content_type,
                  const std::string& scratch_dir,
                  const std::string& label) {
    std::printf("\n== agra mixed-route: %s, two transports, two processes ==\n", label.c_str());

    const long long run_id =
        static_cast<long long>(::getpid()) * 1000 +
        std::chrono::steady_clock::now().time_since_epoch().count() % 1000;
    const std::string bus_name = "agra_mixed_" + std::to_string(run_id);
    const uint16_t ma_port = port_for(run_id, 20);
    const uint16_t c2_port = port_for(run_id, 21);
    const std::string ma_manifest = scratch_dir + "/ma_" + label + ".pcl";
    const std::string c2_manifest = scratch_dir + "/c2_" + label + ".pcl";
    const std::string ma_ready = scratch_dir + "/ma_" + label + ".ready";
    const std::string c2_ready = scratch_dir + "/c2_" + label + ".ready";
    const std::string ma_result = scratch_dir + "/ma_" + label + ".result";
    const std::string c2_result = scratch_dir + "/c2_" + label + ".result";
    for (const auto& f : {ma_ready, c2_ready, ma_result, c2_result}) std::remove(f.c_str());

    ChildProcess ma_proc;
    ChildProcess c2_proc;
    check(ma_proc.spawn(self_path,
                        {"--role=ma", "--shm-plugin=" + shm_plugin_path,
                         "--udp-plugin=" + udp_plugin_path,
                         "--json-plugin=" + ma_json_plugin_path,
                         "--flat-plugin=" + ma_flat_plugin_path,
                         "--content-type=" + content_type, "--bus=" + bus_name,
                         "--own-port=" + std::to_string(ma_port),
                         "--peer-port=" + std::to_string(c2_port),
                         "--manifest=" + ma_manifest, "--ready-file=" + ma_ready,
                         "--peer-ready-file=" + c2_ready, "--result-file=" + ma_result,
                         "--timeout-ms=6000"}),
          "spawned MA process (SHM + UDP)");
    check(c2_proc.spawn(self_path,
                        {"--role=c2", "--shm-plugin=" + shm_plugin_path,
                         "--udp-plugin=" + udp_plugin_path,
                         "--json-plugin=" + c2_json_plugin_path,
                         "--flat-plugin=" + c2_flat_plugin_path,
                         "--content-type=" + content_type, "--bus=" + bus_name,
                         "--own-port=" + std::to_string(c2_port),
                         "--peer-port=" + std::to_string(ma_port),
                         "--manifest=" + c2_manifest, "--ready-file=" + c2_ready,
                         "--peer-ready-file=" + ma_ready, "--result-file=" + c2_result,
                         "--timeout-ms=6000"}),
          "spawned C2 process (SHM + UDP)");

    const int ma_rc = ma_proc.wait(7000);
    const int c2_rc = c2_proc.wait(7000);
    check(ma_rc == 0, "MA process exited 0");
    check(c2_rc == 0,
          "C2 process exited 0 (SHM correlated pair + UDP information topic, both bytes moved)");

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
                        get_option(argc, argv, "--shm-plugin="),
                        get_option(argc, argv, "--udp-plugin="),
                        get_option(argc, argv, "--json-plugin="),
                        get_option(argc, argv, "--flat-plugin="),
                        get_option(argc, argv, "--content-type="),
                        get_option(argc, argv, "--bus="),
                        static_cast<uint16_t>(std::stoul(
                            get_option(argc, argv, "--own-port="))),
                        static_cast<uint16_t>(std::stoul(
                            get_option(argc, argv, "--peer-port="))),
                        get_option(argc, argv, "--manifest="),
                        get_option(argc, argv, "--ready-file="),
                        get_option(argc, argv, "--peer-ready-file="),
                        get_option(argc, argv, "--result-file="),
                        static_cast<uint32_t>(std::stoul(
                            get_option(argc, argv, "--timeout-ms="))));
    }

#if defined(_WIN32)
    std::fprintf(stderr, "agra_mixed_route_test: cross-process scenario not run on Windows in this harness\n");
    return 2;
#else
    if (argc < 8) {
        std::fprintf(stderr,
                     "usage: %s <shm-plugin.so> <udp-plugin.so> <ma-json-codec.so> "
                     "<c2-json-codec.so> <ma-flatbuffers-codec.so> <c2-flatbuffers-codec.so> "
                     "<scratch-dir>\n",
                     argv[0]);
        return 2;
    }
    const std::string shm_plugin_path = argv[1];
    const std::string udp_plugin_path = argv[2];
    const std::string ma_json_plugin_path = argv[3];
    const std::string c2_json_plugin_path = argv[4];
    const std::string ma_flat_plugin_path = argv[5];
    const std::string c2_flat_plugin_path = argv[6];
    const std::string scratch_dir = argv[7];

    run_negative_gate_replay(udp_plugin_path, scratch_dir);

    run_scenario(argv[0], shm_plugin_path, udp_plugin_path, ma_json_plugin_path,
                c2_json_plugin_path, ma_flat_plugin_path, c2_flat_plugin_path,
                ma::kJsonContentType, scratch_dir, "json");
    run_scenario(argv[0], shm_plugin_path, udp_plugin_path, ma_json_plugin_path,
                c2_json_plugin_path, ma_flat_plugin_path, c2_flat_plugin_path,
                ma::kFlatBuffersContentType, scratch_dir, "flatbuffers");

    std::printf("\n%s (%d failure(s))\n", g_failures ? "FAIL" : "PASS", g_failures);
    return g_failures ? 1 : 0;
#endif
}
