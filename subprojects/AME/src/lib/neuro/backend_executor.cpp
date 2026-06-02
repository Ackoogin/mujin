#include "ame/neuro/backend_executor.h"

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

namespace ame::neuro {

struct BackendExecutor::State {
    std::mutex mx;
    unsigned in_flight = 0;
    unsigned consecutive_failures = 0;
    bool circuit_open = false;
    double circuit_open_at_ms = 0.0;
};

static double now_ms() noexcept {
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

BackendExecutor::BackendExecutor(std::shared_ptr<INeuralBackend> backend, BackendExecutorConfig cfg)
    : backend_(std::move(backend)), cfg_(cfg), state_(std::make_shared<State>()) {}

BackendExecutor::~BackendExecutor() = default;

BackendInfo BackendExecutor::info() const {
    if (!backend_) return {};
    return backend_->info();
}

void BackendExecutor::reset_circuit() {
    std::lock_guard<std::mutex> lk(state_->mx);
    state_->circuit_open = false;
    state_->consecutive_failures = 0;
    state_->in_flight = 0;
}

void BackendExecutor::on_abandoned() {
    std::lock_guard<std::mutex> lk(state_->mx);
    ++state_->consecutive_failures;
    if (state_->consecutive_failures >= cfg_.failure_threshold) {
        state_->circuit_open = true;
        state_->circuit_open_at_ms = now_ms();
    }
}

bool BackendExecutor::available() const {
    std::lock_guard<std::mutex> lk(state_->mx);
    if (state_->circuit_open) {
        if (now_ms() - state_->circuit_open_at_ms > cfg_.recovery_window_ms) {
            state_->circuit_open = false;
            state_->consecutive_failures = 0;
        } else {
            return false;
        }
    }
    return state_->in_flight < cfg_.max_in_flight;
}

std::future<NeuralResponse> BackendExecutor::submit(const NeuralRequest& req,
                                                     CancelToken cancel,
                                                     std::shared_ptr<std::atomic<bool>> abandoned_flag) {
    // Check availability under lock; increment in_flight if proceeding.
    {
        std::lock_guard<std::mutex> lk(state_->mx);
        if (state_->circuit_open) {
            if (now_ms() - state_->circuit_open_at_ms > cfg_.recovery_window_ms) {
                state_->circuit_open = false;
                state_->consecutive_failures = 0;
            } else {
                std::promise<NeuralResponse> p;
                auto f = p.get_future();
                p.set_value(NeuralResponse{false, "", "circuit_open"});
                return f;
            }
        }
        if (state_->in_flight >= cfg_.max_in_flight) {
            std::promise<NeuralResponse> p;
            auto f = p.get_future();
            p.set_value(NeuralResponse{false, "", "pool_saturated"});
            return f;
        }
        ++state_->in_flight;
    }

    // Wrap the backend call in a detached thread.  The thread holds shared_ptr<State>
    // so it can safely decrement in_flight even after BackendExecutor is destroyed.
    auto promise_ptr = std::make_shared<std::promise<NeuralResponse>>();
    auto future = promise_ptr->get_future();
    auto state = state_;
    auto backend = backend_; // shared_ptr copy keeps backend alive for the thread's lifetime
    auto cfg = cfg_;

    std::thread([state, backend, req, cancel, promise_ptr, cfg, abandoned_flag]() mutable {
        double t0 = now_ms();
        NeuralResponse resp;
        try {
            auto bfut = backend->submit(req, cancel);
            bfut.wait();
            resp = bfut.get();
        } catch (...) {
            resp.ok = false;
            resp.error = "exception_in_backend";
        }
        resp.latency_ms = now_ms() - t0;

        // Deliver result unconditionally; set_value() on a valid promise always
        // succeeds regardless of whether the caller still holds the future.
        promise_ptr->set_value(resp);

        // If the caller passed an abandoned_flag and set it to true, it already
        // counted this call as a failure via on_abandoned().  Do not reset the
        // consecutive_failures streak on a late ok=true result.
        const bool is_abandoned = abandoned_flag && abandoned_flag->load();

        {
            std::lock_guard<std::mutex> lk(state->mx);
            --state->in_flight;
            if (!resp.ok) {
                // Hard backend error: always count regardless of abandonment.
                ++state->consecutive_failures;
                if (state->consecutive_failures >= cfg.failure_threshold) {
                    state->circuit_open = true;
                    state->circuit_open_at_ms = now_ms();
                }
            } else if (!is_abandoned) {
                // Non-abandoned success: clear failure streak.
                state->consecutive_failures = 0;
            }
            // else: late ok=true on an abandoned call — on_abandoned() already
            // incremented consecutive_failures; do not undo that count.
        }
    }).detach();

    return future;
}

} // namespace ame::neuro
