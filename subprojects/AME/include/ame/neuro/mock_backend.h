#pragma once

#include "ame/neuro/neural_backend.h"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace ame::neuro {

// NullBackend: always unavailable. Safe no-op default.
class NullBackend : public INeuralBackend {
public:
    explicit NullBackend(std::string id = "null");
    BackendInfo info() const override;
    std::future<NeuralResponse> submit(const NeuralRequest&, CancelToken) override;
private:
    std::string id_;
};

// Scripted response entry for MockBackend.
struct MockScript {
    std::string response_payload;
    bool ok = true;
    std::string error;
    double latency_ms = 0.0;  // simulated processing delay
    bool hang = false;        // block until cancel or latency_ms elapses
    bool non_cooperative = false; // when hang=true: ignore cancel signal
};

// MockBackend: scripted test double with injectable latency, errors, and hangs.
// Uses shared internal state so background threads remain safe after destruction.
class MockBackend : public INeuralBackend {
public:
    explicit MockBackend(std::string id = "mock", bool cooperative = true);
    ~MockBackend();

    void add_script(MockScript script);
    void set_cooperative(bool c);

    BackendInfo info() const override;
    std::future<NeuralResponse> submit(const NeuralRequest& req, CancelToken cancel) override;

    unsigned call_count() const;
    unsigned cancelled_count() const;

private:
    struct State;
    std::shared_ptr<State> state_;
};

} // namespace ame::neuro
