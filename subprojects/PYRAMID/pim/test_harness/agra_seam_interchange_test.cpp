// \file agra_seam_interchange_test.cpp
// \brief Phase 5 (plan-terminal) of
//        doc/plans/PYRAMID/rpc_pubsub_interchangeability_plan.md -- proves
//        the interaction facade's RPC/pub-sub interchangeability claim
//        end-to-end: the same MissionAutonomy (MA) provider and C2Station
//        (C2) consumer component code, cross-process over a real shared-
//        memory bus, carries the MAAction_Service request/requirement legs
//        under four independently-selectable realizations -- purely by
//        changing the PCL routing manifest and one configureInteractionBinding()
//        JSON string, never the component logic.
//
// Unlike agra_shm_comms_test.cpp (Phase C of the prior proving plan) and
// agra_mixed_route_test.cpp (its Phase E), which both hand-roll
// publish*/subscribe*/decode* calls directly against the generated topic
// primitives, this harness drives MaactionRequestPortProvider /
// MaactionRequestPortClient (the Phase 2/3 interaction facade) exclusively
// -- component code here has no idea whether a leg is realized as RPC or
// pub/sub.
//
// Run matrix (one child-process pair per run, ONE already-built binary
// re-executed with different CLI args and a fresh manifest each time --
// "byte-identical component objects across runs" holds trivially, since it
// is the same compiled file every time):
//   1. request=rpc,    requirement=rpc     -- facade over classic RPC
//   2. request=pubsub, requirement=pubsub  -- Phase C's scenario via the facade
//   3. request=rpc,    requirement=pubsub  -- mixed legs, neither end aware
//   4. negative: both realizations of the request leg routed at once --
//      Phase 1's D5 compose-time exclusivity must fail closed (replayed
//      compose-time-only, no processes spawned, same style as
//      agra_mixed_route_test.cpp's run_negative_gate_replay).
//
// Each of runs 1-3 replays the full worked-example sequence (task ->
// RECEIVED -> IN_PROGRESS -> COMPLETED; a concurrent second action
// cancelled; correlation and non-conflation checks) and additionally
// checks D3's honest-ack promise: remoteAck() is populated if and only if
// the request leg is RPC-realized, for both the accepted create and the
// accepted cancel. JSON codec first, FlatBuffers second witness, per run.

#include "pyramid_services_agra_c2_station_consumed_components.hpp"
#include "pyramid_services_agra_mission_autonomy_provided_components.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

extern "C" {
#include <pcl/pcl_container.h>
}

extern "C" {
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_plugin_loader.h>
#include <pcl/pcl_transport_routing.h>
}

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <future>
#include <map>
#include <string>
#include <thread>
#include <utility>
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

void check(bool ok, const std::string& what) {
    std::printf("%s  %s\n", ok ? "OK  " : "FAIL", what.c_str());
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

constexpr const char* kActionOne = "action-1";
constexpr const char* kActionTwo = "action-2";

// -- Manifest: one leg realization choice per role, D5 exclusive groups
//    declared either way so the fail-closed guarantee is exercised on
//    every run, not just the negative one. ------------------------------
//
// Follows agra_shm_comms_test.cpp's write_manifest convention: the local
// transport peer alias is the *counterpart's* participant_id (required for
// compose-time validation and runtime ingress filtering to agree on a
// shared-bus transport like SHM).
bool write_manifest(const std::string& path,
                    const std::string& plugin_path,
                    const std::string& bus_name,
                    const std::string& own_participant_id,
                    const std::string& peer_participant_id,
                    bool is_ma,
                    const std::string& request_leg,
                    const std::string& requirement_leg) {
    std::ofstream out(path, std::ios::trunc);
    if (!out) return false;
    const std::string& peer_alias = peer_participant_id;
    out << "transport " << peer_alias << " " << plugin_path
        << " {\"bus_name\":\"" << bus_name << "\",\"participant_id\":\""
        << own_participant_id << "\"}\n";

    out << "exclusive request_leg "
        << ma::kSvcMaactionCreate << "," << ma::kSvcMaactionUpdate << ","
        << ma::kSvcMaactionCancel << " " << ma::kTopicAgraMaActionRequest << "\n";
    out << "exclusive requirement_leg "
        << ma::kSvcMaactionRead << " " << ma::kTopicAgraMaActionRequirement << "\n";

    const char* rpc_command_kind = is_ma ? "provided" : "consumed";
    const char* rpc_read_kind = is_ma ? "stream_provided" : "consumed";
    const char* request_topic_kind = is_ma ? "subscriber" : "publisher";
    const char* requirement_topic_kind = is_ma ? "publisher" : "subscriber";

    if (request_leg == "rpc") {
        out << "route " << ma::kSvcMaactionCreate << " " << rpc_command_kind
            << " " << peer_alias << " reliable\n";
        out << "route " << ma::kSvcMaactionUpdate << " " << rpc_command_kind
            << " " << peer_alias << " reliable\n";
        out << "route " << ma::kSvcMaactionCancel << " " << rpc_command_kind
            << " " << peer_alias << " reliable\n";
    } else {
        out << "route " << ma::kTopicAgraMaActionRequest << " " << request_topic_kind
            << " " << peer_alias << " reliable\n";
    }

    if (requirement_leg == "rpc") {
        out << "route " << ma::kSvcMaactionRead << " " << rpc_read_kind
            << " " << peer_alias << " reliable\n";
    } else {
        out << "route " << ma::kTopicAgraMaActionRequirement << " "
            << requirement_topic_kind << " " << peer_alias << " reliable\n";
    }
    return out.good();
}

// -- MissionAutonomy (provider) role -------------------------------------

class SeamHandler final : public ma::MaactionRequestPortHandler {
public:
    void bindWriter(ma::MaactionRequestPortProvider::TransitionWriter writer) {
        writer_.emplace(std::move(writer));
    }

    ma::Ack onCreate(const ma::MAAction_Service_Request& request) override {
        if (request.ma_action) {
            saw_create_ = true;
            const std::string id = request.ma_action->id;
            sendTransition(id, dm_common::Progress::NotStarted,
                           dm_common::AcceptanceState::Received);
            sendTransition(id, dm_common::Progress::InProgress,
                           dm_common::AcceptanceState::Received);
            sendTransition(id, dm_common::Progress::Completed,
                           dm_common::AcceptanceState::Received);
        }
        return ma::Ack{true};
    }

    ma::Ack onUpdate(const ma::MAAction_Service_Requirement&) override {
        return ma::Ack{true};
    }

    ma::Ack onCancel(const ma::Identifier& id) override {
        saw_cancel_ = true;
        sendTransition(id, dm_common::Progress::Cancelled,
                       dm_common::AcceptanceState::Received);
        return ma::Ack{true};
    }

    bool sawCreate() const { return saw_create_; }
    bool sawCancel() const { return saw_cancel_; }

private:
    void sendTransition(const std::string& id, dm_common::Progress progress,
                        dm_common::AcceptanceState acceptance) {
        dm_common::Requirement req;
        req.id = id;
        req.status.id = id;
        req.status.status = progress;
        req.status.acceptance = acceptance;
        ma::MAAction_Service_Requirement wrapper;
        wrapper.ma_action_status = req;
        check(writer_.has_value(), "MA transition writer bound before dispatch");
        if (writer_) {
            const pcl_status_t rc = writer_->send(wrapper);
            check(rc == PCL_OK, "MA sent requirement transition via TransitionWriter");
        }
    }

    tl::optional<ma::MaactionRequestPortProvider::TransitionWriter> writer_;
    bool saw_create_ = false;
    bool saw_cancel_ = false;
};

class MaComponent final : public pcl::Component {
public:
    MaComponent(pcl::Executor& executor, SeamHandler& handler,
               std::string content_type)
        : pcl::Component("agra_seam_ma"),
          provider_(*this, executor, handler, std::move(content_type)) {}

    ma::MaactionRequestPortProvider& provider() { return provider_; }

protected:
    pcl_status_t on_configure() override { return provider_.bind(); }

private:
    ma::MaactionRequestPortProvider provider_;
};

// -- C2Station (consumer) role --------------------------------------------

class C2Component final : public pcl::Component {
public:
    C2Component(pcl::Executor& executor, std::string content_type)
        : pcl::Component("agra_seam_c2"),
          client_(*this, executor, std::move(content_type)) {}

    c2::MaactionRequestPortClient& client() { return client_; }

protected:
    pcl_status_t on_configure() override { return client_.bind(); }

private:
    c2::MaactionRequestPortClient client_;
};

struct RequirementLog {
    std::vector<dm_common::Progress> progress;
    std::vector<dm_common::AcceptanceState> acceptance;
};

// submit()'s returned std::future<SubmitResult> is std::launch::deferred:
// under RPC realization its body blocks on a *separate*, genuinely async
// future/promise pair (populated by a response callback that only fires
// from inside executor.spinOnce()). wait_for() on a deferred future always
// reports std::future_status::deferred -- never ready, never timeout -- so
// it cannot be polled directly; the only way to run it is .get()/.wait(),
// which then blocks for real. Bridge the two by running .get() on its own
// thread (std::launch::async, whose future *does* report ready/timeout
// correctly) while this function's caller keeps spinning the executor on
// the calling thread -- the response callback and the waiting .get() then
// hand off through SubmitResult's own promise/future, the standard
// cross-thread use case for that pair.
template <typename SubmitFuture,
         typename Result = decltype(std::declval<SubmitFuture&>().get())>
tl::optional<Result> await_submit(
        pcl::Executor& executor, SubmitFuture submit_future,
        std::chrono::steady_clock::time_point deadline) {
    auto async_result = std::async(
        std::launch::async,
        [f = std::move(submit_future)]() mutable { return f.get(); });
    while (std::chrono::steady_clock::now() < deadline &&
          async_result.wait_for(std::chrono::milliseconds(0)) !=
              std::future_status::ready) {
        executor.spinOnce(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (async_result.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready) {
        return tl::nullopt;
    }
    return async_result.get();
}

// -- Per-process role runner ------------------------------------------------

int run_role(const std::string& role,
            const std::string& plugin_path,
            const std::string& json_plugin_path,
            const std::string& flat_plugin_path,
            const std::string& content_type,
            const std::string& bus_name,
            const std::string& request_leg,
            const std::string& requirement_leg,
            const std::string& manifest_path,
            const std::string& ready_file,
            const std::string& peer_ready_file,
            const std::string& result_file,
            uint32_t timeout_ms) {
    const bool is_ma = (role == "ma");

    pcl_plugin_handle_t* json_handle = nullptr;
    if (pcl_plugin_load_codec(json_plugin_path.c_str(),
                              "{\"source\":\"agra_seam_interchange_test\"}",
                              pcl_codec_registry_default(), &json_handle) != PCL_OK) {
        return 2;
    }
    pcl_plugin_handle_t* flat_handle = nullptr;
    if (content_type == ma::kFlatBuffersContentType) {
        if (pcl_plugin_load_codec(flat_plugin_path.c_str(),
                                  "{\"source\":\"agra_seam_interchange_test\"}",
                                  pcl_codec_registry_default(), &flat_handle) != PCL_OK) {
            return 2;
        }
    }

    if (!write_manifest(manifest_path, plugin_path, bus_name,
                        is_ma ? "ma" : "c2", is_ma ? "c2" : "ma", is_ma,
                        request_leg, requirement_leg)) {
        return 2;
    }

    pcl::Executor executor;
    pcl_transport_routing_t* routing = nullptr;
    char diag[512] = "";
    if (pcl_transport_routing_load(executor.handle(), manifest_path.c_str(),
                                   &routing, diag, sizeof(diag)) != PCL_OK) {
        std::fprintf(stderr, "[%s] routing load failed: %s\n", role.c_str(), diag);
        return 2;
    }

    auto unload_and_return = [&](bool ok) {
        pcl_transport_routing_destroy(routing);
        if (flat_handle) pcl_plugin_unload(flat_handle);
        pcl_plugin_unload(json_handle);
        pcl_codec_registry_clear(pcl_codec_registry_default());
        return ok ? 0 : 3;
    };

    // A component that PROVIDES a `provided`/`stream_provided` endpoint over
    // the shared-memory transport must retrieve its "gateway" container
    // (pcl_transport_routing_get_gateway) and add it to the executor itself
    // -- the manifest loader has no way to know a deployment intends to
    // serve rpc-realized endpoints at all, so it cannot wire this
    // automatically (see pcl_transport_routing.h). Only MA (the provider
    // role) needs this; C2 (pure consumer) does not -- inbound *responses*
    // to a `consumed`-kind invoke are delivered directly, no gateway
    // subscriber required for that direction.
    if (is_ma) {
        pcl_container_t* gateway = nullptr;
        if (pcl_transport_routing_get_gateway(routing, "c2", &gateway) != PCL_OK ||
            !gateway) {
            std::fprintf(stderr, "[%s] transport exposed no gateway container\n",
                        role.c_str());
            return unload_and_return(false);
        }
        if (pcl_container_configure(gateway) != PCL_OK ||
            pcl_container_activate(gateway) != PCL_OK ||
            pcl_executor_add(executor.handle(), gateway) != PCL_OK) {
            return unload_and_return(false);
        }
    }

    const std::string requirement_binding_json =
        std::string(R"({"requirement_leg":")") + requirement_leg + "\"}";

    if (is_ma) {
        SeamHandler handler;
        MaComponent component(executor, handler, content_type);
        handler.bindWriter(component.provider().transitionWriter());
        if (component.provider().configureInteractionBinding(requirement_binding_json) !=
            PCL_OK) {
            return unload_and_return(false);
        }
        if (component.configure() != PCL_OK || component.activate() != PCL_OK ||
            executor.add(component) != PCL_OK) {
            return unload_and_return(false);
        }

        write_text(ready_file, "ready");
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline &&
              !(handler.sawCreate() && handler.sawCancel())) {
            executor.spinOnce(0);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        const bool ok = handler.sawCreate() && handler.sawCancel();
        if (ok) write_text(result_file, "create=1 cancel=1");
        executor.remove(component);
        return unload_and_return(ok);
    }

    // C2: bind, subscribe transitions(), announce ready, then wait for MA's
    // ready-file (server/subscriber must be up before we act) before driving
    // the create + cancel sequence through the facade.
    C2Component component(executor, content_type);
    const std::string request_binding_json =
        std::string(R"({"request_leg":")") + request_leg +
        R"(","requirement_leg":")" + requirement_leg + "\"}";
    if (component.client().configureInteractionBinding(request_binding_json) != PCL_OK) {
        return unload_and_return(false);
    }
    if (component.configure() != PCL_OK || component.activate() != PCL_OK ||
        executor.add(component) != PCL_OK) {
        return unload_and_return(false);
    }

    std::map<std::string, RequirementLog> log;
    auto sub = component.client().transitions(
        c2::Query{},
        [&](const c2::MAAction_Service_Requirement& frame) {
            if (!frame.ma_action_status) return;
            const auto& r = *frame.ma_action_status;
            RequirementLog& entry = log[r.id];
            entry.progress.push_back(r.status.status);
            entry.acceptance.push_back(r.status.acceptance);
        });
    check(sub.valid(), "C2 transitions() subscription established");

    write_text(ready_file, "ready");
    const auto peer_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < peer_deadline && !file_exists(peer_ready_file)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    c2::MAAction_Service_Request create_request;
    dm_agra::MA_Action action_one;
    action_one.id = kActionOne;
    action_one.action_type = dm_agra::ActionType::FindSearch;
    action_one.target_object = "AOI-1";
    action_one.action_constraints = {"EMCON_PASSIVE", "KEEP_OUT_WEST_OF_AOI1"};
    create_request.ma_action = action_one;

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    auto create_result_opt = await_submit(
        executor, component.client().submit(create_request), deadline);
    check(create_result_opt.has_value(), "C2 create submit() completed within timeout");
    if (create_result_opt) {
        const auto& create_result = *create_result_opt;
        check(create_result.accepted, "C2 create submit() accepted");
        if (request_leg == "rpc") {
            check(create_result.remoteAck().has_value() && create_result.remoteAck()->success,
                  "D3: create remoteAck() populated under RPC realization");
        } else {
            check(!create_result.remoteAck().has_value(),
                  "D3: create remoteAck() empty under pub/sub realization (no synthesized ack)");
        }
    }

    const auto has_transitions = [&](const std::string& id, size_t n) {
        auto it = log.find(id);
        return it != log.end() && it->second.progress.size() >= n;
    };
    while (std::chrono::steady_clock::now() < deadline && !has_transitions(kActionOne, 3)) {
        executor.spinOnce(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    c2::Identifier cancel_id = kActionTwo;
    auto cancel_result_opt = await_submit(
        executor, component.client().submit(cancel_id), deadline);
    check(cancel_result_opt.has_value(), "C2 cancel submit() completed within timeout");
    if (cancel_result_opt) {
        const auto& cancel_result = *cancel_result_opt;
        check(cancel_result.accepted, "C2 cancel submit() accepted");
        if (request_leg == "rpc") {
            check(cancel_result.remoteAck().has_value() && cancel_result.remoteAck()->success,
                  "D3: cancel remoteAck() populated under RPC realization");
        } else {
            check(!cancel_result.remoteAck().has_value(),
                  "D3: cancel remoteAck() empty under pub/sub realization (no synthesized ack)");
        }
    }

    while (std::chrono::steady_clock::now() < deadline && !has_transitions(kActionTwo, 1)) {
        executor.spinOnce(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const RequirementLog& log_one = log[kActionOne];
    const RequirementLog& log_two = log[kActionTwo];
    const bool ok =
        log_one.progress.size() >= 3 &&
        log_one.progress.back() == dm_common::Progress::Completed &&
        !log_one.acceptance.empty() &&
        log_one.acceptance.front() == dm_common::AcceptanceState::Received &&
        std::find(log_one.progress.begin(), log_one.progress.end(),
                  dm_common::Progress::Cancelled) == log_one.progress.end() &&
        log_two.progress.size() >= 1 &&
        log_two.progress.back() == dm_common::Progress::Cancelled &&
        std::find(log_two.progress.begin(), log_two.progress.end(),
                  dm_common::Progress::Completed) == log_two.progress.end();

    check(ok, "C2: transitions() delivered RECEIVED/IN_PROGRESS/COMPLETED + CANCELLED, "
              "correlated by id, non-conflated");

    if (ok) {
        write_text(result_file, "action1_transitions=" +
                                     std::to_string(log_one.progress.size()) +
                                     " action2_transitions=" +
                                     std::to_string(log_two.progress.size()));
    }
    sub.cancel();
    executor.remove(component);
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
                  const std::string& plugin_path,
                  const std::string& ma_json_plugin_path,
                  const std::string& c2_json_plugin_path,
                  const std::string& ma_flat_plugin_path,
                  const std::string& c2_flat_plugin_path,
                  const std::string& content_type,
                  const std::string& scratch_dir,
                  const std::string& label,
                  const std::string& request_leg,
                  const std::string& requirement_leg) {
    std::printf("\n== agra seam interchange: %s (request=%s, requirement=%s) ==\n",
               label.c_str(), request_leg.c_str(), requirement_leg.c_str());

    const long long run_id =
        static_cast<long long>(::getpid()) * 1000 +
        std::chrono::steady_clock::now().time_since_epoch().count() % 1000;
    const std::string bus_name = "agra_seam_" + std::to_string(run_id);
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
                         "--request-leg=" + request_leg,
                         "--requirement-leg=" + requirement_leg,
                         "--manifest=" + ma_manifest, "--ready-file=" + ma_ready,
                         "--peer-ready-file=" + c2_ready, "--result-file=" + ma_result,
                         "--timeout-ms=6000"}),
          "spawned MA process");
    check(c2_proc.spawn(self_path,
                        {"--role=c2", "--plugin=" + plugin_path,
                         "--json-plugin=" + c2_json_plugin_path,
                         "--flat-plugin=" + c2_flat_plugin_path,
                         "--content-type=" + content_type, "--bus=" + bus_name,
                         "--request-leg=" + request_leg,
                         "--requirement-leg=" + requirement_leg,
                         "--manifest=" + c2_manifest, "--ready-file=" + c2_ready,
                         "--peer-ready-file=" + ma_ready, "--result-file=" + c2_result,
                         "--timeout-ms=6000"}),
          "spawned C2 process");

    const int ma_rc = ma_proc.wait(7000);
    const int c2_rc = c2_proc.wait(7000);
    check(ma_rc == 0, "MA process exited 0");
    check(c2_rc == 0, "C2 process exited 0");

    std::string ma_result_text;
    std::string c2_result_text;
    check(read_text(ma_result, &ma_result_text) && !ma_result_text.empty(),
          "MA cross-process result file written");
    check(read_text(c2_result, &c2_result_text) && !c2_result_text.empty(),
          "C2 cross-process result file written");
    std::printf("     MA result: %s\n", ma_result_text.c_str());
    std::printf("     C2 result: %s\n", c2_result_text.c_str());
}

// -- Run 4: negative gate, compose-time only (no processes) -----------------
//
// Deliberately routes BOTH sides of the request leg's exclusive group at
// once (RPC command endpoints + the request topic) and asserts
// pcl_transport_routing_load fails closed with the D5 diagnostic -- Phase
// 1's compose-time exclusivity check, replayed here so this terminal
// harness carries its own evidence of the negative case, matching
// agra_mixed_route_test.cpp's run_negative_gate_replay precedent.
void run_negative_gate_replay(const std::string& plugin_path,
                              const std::string& scratch_dir) {
    std::printf("\n== agra seam interchange: negative gate replay "
               "(request leg dual-routed) ==\n");

    const std::string manifest_path = scratch_dir + "/negative_gate_replay.pcl";
    {
        std::ofstream out(manifest_path, std::ios::trunc);
        out << "transport c2 " << plugin_path
            << " {\"bus_name\":\"agra_seam_negative\",\"participant_id\":\"ma\"}\n";
        out << "exclusive request_leg "
            << ma::kSvcMaactionCreate << "," << ma::kSvcMaactionUpdate << ","
            << ma::kSvcMaactionCancel << " " << ma::kTopicAgraMaActionRequest << "\n";
        out << "route " << ma::kSvcMaactionCreate << " provided c2 reliable\n";
        out << "route " << ma::kTopicAgraMaActionRequest << " subscriber c2 reliable\n";
    }

    pcl::Executor executor;
    pcl_transport_routing_t* routing = nullptr;
    char diag[512] = "";
    const pcl_status_t rc = pcl_transport_routing_load(
        executor.handle(), manifest_path.c_str(), &routing, diag, sizeof(diag));

    check(rc == PCL_ERR_STATE,
          "dual-routed request leg still fails closed (D5 exclusivity, replayed)");
    const std::string diag_text(diag);
    check(diag_text.find("request_leg") != std::string::npos &&
              diag_text.find("opposite sides") != std::string::npos,
          "diagnostic names the request_leg group and the conflict");

    pcl_transport_routing_destroy(routing);
}
#endif  // !_WIN32

}  // namespace

int main(int argc, char** argv) {
    // Every check()/printf below must survive a SIGTERM kill (ChildProcess's
    // destructor sends one on timeout) reaching the process while stdout is
    // still block-buffered (this binary's stdout is not a tty once run
    // under the build script's `| tail`/redirection) -- line-buffer so nothing
    // written before a hang is silently lost.
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string role = get_option(argc, argv, "--role=");
    if (!role.empty()) {
        return run_role(role,
                        get_option(argc, argv, "--plugin="),
                        get_option(argc, argv, "--json-plugin="),
                        get_option(argc, argv, "--flat-plugin="),
                        get_option(argc, argv, "--content-type="),
                        get_option(argc, argv, "--bus="),
                        get_option(argc, argv, "--request-leg="),
                        get_option(argc, argv, "--requirement-leg="),
                        get_option(argc, argv, "--manifest="),
                        get_option(argc, argv, "--ready-file="),
                        get_option(argc, argv, "--peer-ready-file="),
                        get_option(argc, argv, "--result-file="),
                        static_cast<uint32_t>(std::stoul(
                            get_option(argc, argv, "--timeout-ms="))));
    }

#if defined(_WIN32)
    std::fprintf(stderr, "agra_seam_interchange_test: cross-process scenario not run on Windows in this harness\n");
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

    run_negative_gate_replay(plugin_path, scratch_dir);

    struct RunSpec {
        const char* label;
        const char* request_leg;
        const char* requirement_leg;
    };
    const RunSpec runs[] = {
        {"run1_rpc_rpc", "rpc", "rpc"},
        {"run2_pubsub_pubsub", "pubsub", "pubsub"},
        {"run3_rpc_pubsub", "rpc", "pubsub"},
    };
    const char* content_types[] = {ma::kJsonContentType, ma::kFlatBuffersContentType};

    for (const auto& run : runs) {
        for (const char* content_type : content_types) {
            const std::string label = std::string(run.label) + "_" +
                (content_type == ma::kJsonContentType ? "json" : "flatbuffers");
            run_scenario(argv[0], plugin_path, ma_json_plugin_path, c2_json_plugin_path,
                        ma_flat_plugin_path, c2_flat_plugin_path, content_type,
                        scratch_dir, label, run.request_leg, run.requirement_leg);
        }
    }

    std::printf("\n%s (%d failure(s))\n", g_failures ? "FAIL" : "PASS", g_failures);
    return g_failures ? 1 : 0;
#endif
}
