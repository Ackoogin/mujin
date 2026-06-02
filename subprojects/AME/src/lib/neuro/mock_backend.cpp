#include "ame/neuro/mock_backend.h"

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

namespace ame::neuro {

// ---- NullBackend -----------------------------------------------------------

NullBackend::NullBackend(std::string id) : id_(std::move(id)) {}

BackendInfo NullBackend::info() const {
    return {id_, "null", 0.0, false};
}

std::future<NeuralResponse> NullBackend::submit(const NeuralRequest&, CancelToken) {
    std::promise<NeuralResponse> p;
    auto f = p.get_future();
    p.set_value(NeuralResponse{false, "", "null_backend", 0.0, id_, ""});
    return f;
}

// ---- MockBackend -----------------------------------------------------------

struct MockBackend::State {
    std::string id;
    bool cooperative;
    std::mutex mx;
    std::vector<MockScript> scripts;
    size_t script_pos = 0;
    std::atomic<unsigned> call_count{0};
    std::atomic<unsigned> cancelled_count{0};
};

MockBackend::MockBackend(std::string id, bool cooperative)
    : state_(std::make_shared<State>()) {
    state_->id = std::move(id);
    state_->cooperative = cooperative;
}

MockBackend::~MockBackend() = default;

void MockBackend::add_script(MockScript script) {
    std::lock_guard<std::mutex> lk(state_->mx);
    state_->scripts.push_back(std::move(script));
}

void MockBackend::set_cooperative(bool c) {
    std::lock_guard<std::mutex> lk(state_->mx);
    state_->cooperative = c;
}

BackendInfo MockBackend::info() const {
    std::lock_guard<std::mutex> lk(state_->mx);
    return {state_->id, "mock", 0.0, state_->cooperative};
}

unsigned MockBackend::call_count() const {
    return state_->call_count.load();
}

unsigned MockBackend::cancelled_count() const {
    return state_->cancelled_count.load();
}

std::future<NeuralResponse> MockBackend::submit(const NeuralRequest& /*req*/,
                                                 CancelToken cancel) {
    ++state_->call_count;

    MockScript script;
    {
        std::lock_guard<std::mutex> lk(state_->mx);
        if (!state_->scripts.empty()) {
            size_t pos = std::min(state_->script_pos, state_->scripts.size() - 1);
            script = state_->scripts[pos];
            if (state_->script_pos < state_->scripts.size() - 1)
                ++state_->script_pos;
        }
    }

    std::promise<NeuralResponse> promise;
    auto future = promise.get_future();
    auto shared = state_; // keep State alive for thread

    std::thread([script, cancel, shared, p = std::move(promise)]() mutable {
        if (script.hang) {
            double hang_ms = (script.latency_ms > 0.0) ? script.latency_ms : 60000.0;
            auto deadline = std::chrono::steady_clock::now() +
                std::chrono::microseconds(static_cast<long long>(hang_ms * 1000.0));

            while (std::chrono::steady_clock::now() < deadline) {
                // cooperative: check cancel every 10ms
                if (!script.non_cooperative && cancel.cancelled()) {
                    ++shared->cancelled_count;
                    NeuralResponse r;
                    r.ok = false;
                    r.error = "cancelled";
                    r.backend_id = shared->id;
                    try { p.set_value(std::move(r)); } catch (...) {}
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        } else if (script.latency_ms > 0.0) {
            std::this_thread::sleep_for(
                std::chrono::microseconds(
                    static_cast<long long>(script.latency_ms * 1000.0)));
        }

        NeuralResponse r;
        r.ok = script.ok;
        r.payload = script.response_payload;
        r.error = script.error;
        r.backend_id = shared->id;
        try { p.set_value(std::move(r)); } catch (...) {}
    }).detach();

    return future;
}

} // namespace ame::neuro
