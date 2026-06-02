#pragma once

#include "ame/neuro/neural_backend.h"

#include <memory>

namespace ame::neuro {

// Configuration for the pool and circuit breaker.
struct BackendExecutorConfig {
    unsigned max_in_flight = 4;         // max concurrent outstanding calls (incl. abandoned)
    unsigned failure_threshold = 5;     // consecutive failures before circuit opens
    double recovery_window_ms = 10000.0; // ms before circuit resets after opening
};

// Wraps a backend with a bounded worker pool and circuit breaker.
// Each submit() launches the backend call on a detached thread (via promise/future).
// Abandoned (timed-out) futures do not block the caller; their threads complete
// independently, then reclaim the slot. Once slots from abandoned calls saturate
// the pool, the circuit opens and further submits report unavailable immediately.
class BackendExecutor {
public:
    // Takes shared ownership of the backend so detached worker threads remain
    // safe even if the executor or its registry is destroyed before they finish.
    BackendExecutor(std::shared_ptr<INeuralBackend> backend, BackendExecutorConfig cfg = {});
    ~BackendExecutor();

    // Submit a request. Returns a future the caller may abandon after budget expires.
    // If pool is saturated or circuit is open, returns NeuralResponse{ok=false} immediately.
    std::future<NeuralResponse> submit(const NeuralRequest& req, CancelToken cancel);

    // True if the circuit is closed and the pool has available slots.
    bool available() const;

    // Called by the Advisor when it abandons a future (budget elapsed).
    // Counts toward consecutive failures and may open the circuit.
    void on_abandoned();

    // Reset circuit state (for tests).
    void reset_circuit();

    BackendInfo info() const;

private:
    struct State;
    std::shared_ptr<INeuralBackend> backend_;
    BackendExecutorConfig cfg_;
    std::shared_ptr<State> state_;
};

} // namespace ame::neuro
