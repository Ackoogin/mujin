#pragma once

#include <string>

namespace ame::neuro {

// What to do when the verifier rejects a proposal.
enum class OnRejectAction { FallBack, RetryWithFeedback };

// Deployment tier governs default budget/retry settings.
enum class DeploymentTier { Hot, Warm, Cold };

// All timing and retry rules in one struct. Policy is data, not code.
struct FallbackPolicy {
    bool enabled = false;              // false = Disabled outcome immediately
    double latency_budget_ms = 500.0;  // total wall-time budget across all attempts
    unsigned max_retries = 0;          // extra attempts after the first (0 = no retry)
    double retry_backoff_ms = 50.0;    // sleep between retry attempts
    OnRejectAction on_reject = OnRejectAction::FallBack;
    DeploymentTier tier = DeploymentTier::Warm;
    std::string backend_id;            // empty = first available
    std::string model_id;              // version pin (recorded in audit)
    std::string authority_view;        // "All", "ConfirmedThenBelieved", "ConfirmedOnly"
    int verbosity = 0;                 // 0=digest only, 1=full payload in audit

    static FallbackPolicy disabled() { return {}; }

    static FallbackPolicy hot_path(std::string bid = "", std::string mid = "") {
        FallbackPolicy p;
        p.enabled = true;
        p.latency_budget_ms = 50.0;
        p.max_retries = 0;
        p.tier = DeploymentTier::Hot;
        p.backend_id = std::move(bid);
        p.model_id = std::move(mid);
        return p;
    }

    static FallbackPolicy warm_path(std::string bid = "", std::string mid = "") {
        FallbackPolicy p;
        p.enabled = true;
        p.latency_budget_ms = 500.0;
        p.max_retries = 1;
        p.retry_backoff_ms = 50.0;
        p.tier = DeploymentTier::Warm;
        p.backend_id = std::move(bid);
        p.model_id = std::move(mid);
        return p;
    }

    static FallbackPolicy cold_path(std::string bid = "", std::string mid = "") {
        FallbackPolicy p;
        p.enabled = true;
        p.latency_budget_ms = 5000.0;
        p.max_retries = 2;
        p.retry_backoff_ms = 200.0;
        p.tier = DeploymentTier::Cold;
        p.backend_id = std::move(bid);
        p.model_id = std::move(mid);
        return p;
    }
};

// Injectable clock function (returns current time in ms). Default: steady_clock.
using ClockFn = double (*)();
double default_clock_ms() noexcept;

} // namespace ame::neuro
