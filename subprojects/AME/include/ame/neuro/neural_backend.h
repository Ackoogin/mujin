#pragma once

#include "ame/neuro/cancel_token.h"

#include <future>
#include <string>

namespace ame::neuro {

struct NeuralRequest {
    std::string kind;     // integration kind, e.g. "goal_interpreter"
    std::string payload;  // serialised prompt or feature blob
};

struct NeuralResponse {
    bool ok = false;
    std::string payload;
    std::string error;
    double latency_ms = 0.0;
    std::string backend_id;
    std::string model_id;
};

struct BackendInfo {
    std::string id;
    std::string modality;           // "llm", "onnx", "mock", "null"
    double nominal_latency_ms = 0.0;
    bool cooperative = false;       // true if backend honours CancelToken
};

// Transport abstraction: the single seam for all proposal requests.
//
// Contract:
//  1. Cooperation expected, not assumed. A backend that ignores cancel is legal;
//     the Advisor bounds wall-time via wait_for() regardless.
//  2. Self-terminating deadline. Each backend MUST impose its own transport
//     timeout so an abandoned call eventually completes and frees resources.
//  3. Abandonment is normal. The caller may discard the returned future after
//     its budget elapses; the in-flight call runs on an isolated worker whose
//     eventual result is discarded.
//  4. info().cooperative advertises whether the backend honours cancel, letting
//     the policy/registry treat non-cooperative backends more conservatively.
class INeuralBackend {
public:
    virtual ~INeuralBackend() = default;
    virtual BackendInfo info() const = 0;

    // Async submit. Caller may abandon the returned future after budget elapses.
    // Implementations MUST eventually complete/error (self-terminating deadline).
    virtual std::future<NeuralResponse> submit(const NeuralRequest& req,
                                                CancelToken cancel) = 0;
};

} // namespace ame::neuro
