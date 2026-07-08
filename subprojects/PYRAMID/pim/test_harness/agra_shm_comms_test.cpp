// \file agra_shm_comms_test.cpp
// \brief Phase C of doc/plans/PYRAMID/agra_pubsub_shm_udp_proving_plan.md --
//        the correlated request/requirement pair for the A-GRA example
//        contract (pim/agra_example/), cross-process over a real shared-
//        memory bus loaded via pcl_transport_routing_load (the manifest-
//        driven path Phase A proved out, not the direct transport API).
//
// MissionAutonomy (MA) provider process and C2Station (C2) consumer
// process, each with its own executor and routing manifest, on a shared
// per-run-unique SHM bus:
//   1. C2 publishes MA_Action (FIND_SEARCH, target, constraints) on
//      agra.ma_action.request, correlated by Entity.id = "action-1".
//   2. MA observes it and publishes three requirement transitions on
//      agra.ma_action.requirement (all bearing id "action-1"): acceptance
//      RECEIVED -> progress IN_PROGRESS -> COMPLETED. C2 observes each.
//   3. C2 publishes the cancel variant for a second, concurrently-tracked
//      action ("action-2") on the same flat topic; MA publishes the
//      CANCELLED transition (id "action-2"); C2 observes it.
//   4. Sequence-conformance + correlation checks: action-1's log never
//      contains a CANCELLED transition and action-2's never contains a
//      COMPLETED one, proving correlation is by Entity.id on a flat topic
//      space rather than by accident of ordering.
//   5. The whole sequence runs twice, cross-process each time, on fresh
//      per-run bus names: JSON codec first, FlatBuffers second.

#include "pyramid_services_agra_c2_station_consumed.hpp"
#include "pyramid_services_agra_mission_autonomy_provided.hpp"

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport_routing.h>
}

// The generated per-component JSON/FlatBuffers codec plugins both export the
// identical extern "C" symbol pcl_codec_plugin_entry -- statically linking
// both packages' plugin .cpp files into one binary would collide at link
// time. Each is instead built as its own loadable .so (build script) and
// dlopen'd per-process via pcl_plugin_load_codec, exactly like the
// FlatBuffers codec in components_comms_test.cpp -- just applied to JSON
// too, since here there are two distinct generated packages instead of one.

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

// See routed_egress_shm_test.cpp's write_manifest for why the local
// transport peer alias must be the *counterpart's* participant_id: compose-
// time validation requires the alias to name a locally-registered
// transport, while runtime ingress filtering compares route peers against
// the remote sender's own participant_id -- the two checks only agree for
// a shared-bus transport like SHM if the alias is chosen this way.
bool write_manifest(const std::string& path,
                    const std::string& plugin_path,
                    const std::string& bus_name,
                    const std::string& own_participant_id,
                    const std::string& peer_participant_id,
                    bool is_ma) {
    std::ofstream out(path, std::ios::trunc);
    if (!out) return false;
    const std::string& peer_alias = peer_participant_id;
    out << "transport " << peer_alias << " " << plugin_path
        << " {\"bus_name\":\"" << bus_name << "\",\"participant_id\":\""
        << own_participant_id << "\"}\n";
    if (is_ma) {
        out << "route " << ma::kTopicAgraMaActionRequest
            << " subscriber " << peer_alias << " reliable\n";
        out << "route " << ma::kTopicAgraMaActionRequirement
            << " publisher " << peer_alias << " reliable\n";
    } else {
        out << "route " << c2::kTopicAgraMaActionRequest
            << " publisher " << peer_alias << " reliable\n";
        out << "route " << c2::kTopicAgraMaActionRequirement
            << " subscriber " << peer_alias << " reliable\n";
    }
    return out.good();
}

constexpr const char* kActionOne = "action-1";
constexpr const char* kActionTwo = "action-2";

struct RequirementLog {
    std::vector<dm_common::Progress> progress;
    std::vector<dm_common::AcceptanceState> acceptance;
};

// -- MissionAutonomy (provider) role -----------------------------------

struct MaState {
    pcl_port_t* requirement_pub = nullptr;
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
    ma::MAAction_Service_Requirement wrapper;
    wrapper.ma_action_status = req;
    const pcl_status_t rc = ma::publishAgraMaActionRequirement(
        state->requirement_pub, wrapper, state->content_type.c_str());
    check(rc == PCL_OK, "MA published requirement transition over SHM route");
}

void ma_on_request(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* state = static_cast<MaState*>(ud);
    ma::MAAction_Service_Request request;
    if (!ma::decodeAgraMaActionRequest(msg, &request)) {
        check(false, "MA decoded request-topic payload received over SHM");
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
        c, ma::kTopicAgraMaActionRequirement, state->content_type.c_str());
    pcl_port_t* sub = ma::subscribeAgraMaActionRequest(
        c, ma_on_request, state, state->content_type.c_str());
    return (state->requirement_pub && sub) ? PCL_OK : PCL_ERR_NOMEM;
}

// -- C2Station (consumer) role ------------------------------------------

struct C2State {
    pcl_port_t* request_pub = nullptr;
    std::string content_type;
    std::map<std::string, RequirementLog> log;
};

void c2_on_requirement(pcl_container_t*, const pcl_msg_t* msg, void* ud) {
    auto* state = static_cast<C2State*>(ud);
    c2::MAAction_Service_Requirement payload;
    if (!c2::decodeAgraMaActionRequirement(msg, &payload)) {
        check(false, "C2 decoded requirement-topic payload received over SHM");
        return;
    }
    if (!payload.ma_action_status) return;
    const auto& r = *payload.ma_action_status;
    RequirementLog& entry = state->log[r.id];
    entry.progress.push_back(r.status.status);
    entry.acceptance.push_back(r.status.acceptance);
}

pcl_status_t c2_on_configure(pcl_container_t* c, void* ud) {
    auto* state = static_cast<C2State*>(ud);
    state->request_pub = pcl_container_add_publisher(
        c, c2::kTopicAgraMaActionRequest, state->content_type.c_str());
    pcl_port_t* sub = c2::subscribeAgraMaActionRequirement(
        c, c2_on_requirement, state, state->content_type.c_str());
    return (state->request_pub && sub) ? PCL_OK : PCL_ERR_NOMEM;
}

// -- Per-process role runner (cross-process split) -----------------------

int run_role(const std::string& role,
            const std::string& plugin_path,
            const std::string& json_plugin_path,
            const std::string& flat_plugin_path,
            const std::string& content_type,
            const std::string& bus_name,
            const std::string& manifest_path,
            const std::string& ready_file,
            const std::string& peer_ready_file,
            const std::string& result_file,
            uint32_t timeout_ms) {
    const bool is_ma = (role == "ma");

    pcl_plugin_handle_t* json_handle = nullptr;
    if (pcl_plugin_load_codec(json_plugin_path.c_str(),
                              "{\"source\":\"agra_shm_comms_test\"}",
                              pcl_codec_registry_default(), &json_handle) != PCL_OK) {
        return 2;
    }

    pcl_plugin_handle_t* flat_handle = nullptr;
    if (content_type == ma::kFlatBuffersContentType) {
        if (pcl_plugin_load_codec(flat_plugin_path.c_str(),
                                  "{\"source\":\"agra_shm_comms_test\"}",
                                  pcl_codec_registry_default(), &flat_handle) != PCL_OK) {
            return 2;
        }
    }

    if (!write_manifest(manifest_path, plugin_path, bus_name,
                        is_ma ? "ma" : "c2", is_ma ? "c2" : "ma", is_ma)) {
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

    MaState ma_state;
    C2State c2_state;
    ma_state.content_type = content_type;
    c2_state.content_type = content_type;

    pcl_callbacks_t cbs{};
    cbs.on_configure = is_ma ? ma_on_configure : c2_on_configure;
    void* user_data = is_ma ? static_cast<void*>(&ma_state) : static_cast<void*>(&c2_state);

    pcl_container_t* container = pcl_container_create(
        is_ma ? "agra_shm_ma" : "agra_shm_c2", &cbs, user_data);
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
        while (std::chrono::steady_clock::now() < deadline &&
              !(ma_state.saw_action_one_create && ma_state.saw_action_two_cancel)) {
            pcl_executor_spin_once(exec, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        const bool ok = ma_state.saw_action_one_create && ma_state.saw_action_two_cancel;
        if (ok) write_text(result_file, "create=1 cancel=1");
        pcl_executor_remove(exec, container);
        pcl_container_destroy(container);
        pcl_transport_routing_destroy(routing);
        pcl_executor_destroy(exec);
        if (flat_handle) pcl_plugin_unload(flat_handle);
        pcl_plugin_unload(json_handle);
        pcl_codec_registry_clear(pcl_codec_registry_default());
        return ok ? 0 : 3;
    }

    // C2: wait for MA's ready-file (subscriber must be up before we
    // publish), then drive the create + cancel sequence.
    while (std::chrono::steady_clock::now() < deadline && !file_exists(peer_ready_file)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    c2::MAAction_Service_Request create_request;
    pyramid::domain_model::agra::MA_Action action_one;
    action_one.id = kActionOne;
    action_one.action_type = pyramid::domain_model::agra::ActionType::FindSearch;
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
          !has_transitions(kActionOne, 3)) {
        pcl_executor_spin_once(exec, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    c2::MAAction_Service_Request cancel_request;
    cancel_request.cancel = std::string(kActionTwo);
    c2::publishAgraMaActionRequest(c2_state.request_pub, cancel_request,
                                   content_type.c_str());

    while (std::chrono::steady_clock::now() < deadline &&
          !has_transitions(kActionTwo, 1)) {
        pcl_executor_spin_once(exec, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const RequirementLog& log_one = c2_state.log[kActionOne];
    const RequirementLog& log_two = c2_state.log[kActionTwo];
    const bool ok =
        log_one.progress.size() >= 3 &&
        log_one.progress.back() == dm_common::Progress::Completed &&
        !log_one.progress.empty() &&
        log_one.acceptance.front() == dm_common::AcceptanceState::Received &&
        std::find(log_one.progress.begin(), log_one.progress.end(),
                  dm_common::Progress::Cancelled) == log_one.progress.end() &&
        log_two.progress.size() >= 1 &&
        log_two.progress.back() == dm_common::Progress::Cancelled &&
        std::find(log_two.progress.begin(), log_two.progress.end(),
                  dm_common::Progress::Completed) == log_two.progress.end();

    if (ok) {
        write_text(result_file, "action1_transitions=" +
                                     std::to_string(log_one.progress.size()) +
                                     " action2_transitions=" +
                                     std::to_string(log_two.progress.size()));
    }
    pcl_executor_remove(exec, container);
    pcl_container_destroy(container);
    pcl_transport_routing_destroy(routing);
    pcl_executor_destroy(exec);
    if (flat_handle) pcl_plugin_unload(flat_handle);
    pcl_plugin_unload(json_handle);
    pcl_codec_registry_clear(pcl_codec_registry_default());
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

void run_scenario(const std::string& self_path,
                  const std::string& plugin_path,
                  const std::string& ma_json_plugin_path,
                  const std::string& c2_json_plugin_path,
                  const std::string& ma_flat_plugin_path,
                  const std::string& c2_flat_plugin_path,
                  const std::string& content_type,
                  const std::string& scratch_dir,
                  const std::string& label) {
    std::printf("\n== agra SHM comms: %s, two processes ==\n", label.c_str());

    const long long run_id =
        static_cast<long long>(::getpid()) * 1000 +
        std::chrono::steady_clock::now().time_since_epoch().count() % 1000;
    const std::string bus_name = "agra_shm_" + std::to_string(run_id);
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
                        {"--role=ma", "--plugin=" + plugin_path,
                         "--json-plugin=" + ma_json_plugin_path,
                         "--flat-plugin=" + ma_flat_plugin_path,
                         "--content-type=" + content_type, "--bus=" + bus_name,
                         "--manifest=" + ma_manifest, "--ready-file=" + ma_ready,
                         "--peer-ready-file=" + c2_ready, "--result-file=" + ma_result,
                         "--timeout-ms=6000"}),
          "spawned MA process");
    check(c2_proc.spawn(self_path,
                        {"--role=c2", "--plugin=" + plugin_path,
                         "--json-plugin=" + c2_json_plugin_path,
                         "--flat-plugin=" + c2_flat_plugin_path,
                         "--content-type=" + content_type, "--bus=" + bus_name,
                         "--manifest=" + c2_manifest, "--ready-file=" + c2_ready,
                         "--peer-ready-file=" + ma_ready, "--result-file=" + c2_result,
                         "--timeout-ms=6000"}),
          "spawned C2 process");

    const int ma_rc = ma_proc.wait(7000);
    const int c2_rc = c2_proc.wait(7000);
    check(ma_rc == 0, "MA process exited 0 (observed create + cancel over SHM)");
    check(c2_rc == 0,
          "C2 process exited 0 (action-1 RECEIVED/IN_PROGRESS/COMPLETED + "
          "action-2 CANCELLED, correlated by id, non-conflated)");

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
                        get_option(argc, argv, "--plugin="),
                        get_option(argc, argv, "--json-plugin="),
                        get_option(argc, argv, "--flat-plugin="),
                        get_option(argc, argv, "--content-type="),
                        get_option(argc, argv, "--bus="),
                        get_option(argc, argv, "--manifest="),
                        get_option(argc, argv, "--ready-file="),
                        get_option(argc, argv, "--peer-ready-file="),
                        get_option(argc, argv, "--result-file="),
                        static_cast<uint32_t>(std::stoul(
                            get_option(argc, argv, "--timeout-ms="))));
    }

#if defined(_WIN32)
    std::fprintf(stderr, "agra_shm_comms_test: cross-process scenario not run on Windows in this harness\n");
    return 2;
#else
    if (argc < 7) {
        std::fprintf(stderr,
                     "usage: %s <shm-plugin.so> <ma-json-codec.so> <c2-json-codec.so> "
                     "<ma-flatbuffers-codec.so> <c2-flatbuffers-codec.so> <scratch-dir>\n",
                     argv[0]);
        return 2;
    }
    const std::string plugin_path = argv[1];
    const std::string ma_json_plugin_path = argv[2];
    const std::string c2_json_plugin_path = argv[3];
    const std::string ma_flat_plugin_path = argv[4];
    const std::string c2_flat_plugin_path = argv[5];
    const std::string scratch_dir = argv[6];

    run_scenario(argv[0], plugin_path, ma_json_plugin_path, c2_json_plugin_path,
                ma_flat_plugin_path, c2_flat_plugin_path, ma::kJsonContentType,
                scratch_dir, "json");
    run_scenario(argv[0], plugin_path, ma_json_plugin_path, c2_json_plugin_path,
                ma_flat_plugin_path, c2_flat_plugin_path, ma::kFlatBuffersContentType,
                scratch_dir, "flatbuffers");

    std::printf("\n%s (%d failure(s))\n", g_failures ? "FAIL" : "PASS", g_failures);
    return g_failures ? 1 : 0;
#endif
}
