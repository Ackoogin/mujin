#pragma once

#include "ame/neuro/fallback_policy.h"

#include <string>
#include <vector>

namespace ame::neuro {

// Per-integration configuration block.
struct IntegrationConfig {
    std::string integration_kind; // e.g. "goal_interpreter"
    FallbackPolicy policy;
};

// Top-level neuro configuration.
// One declarative place to enable/disable integrations, bind them to backend
// tiers, set budgets, and pin model versions.
struct NeuroConfig {
    bool enabled = false;                         // global kill-switch
    std::vector<IntegrationConfig> integrations;

    static NeuroConfig all_disabled() { return {}; }

    // Load from a JSON string (minimal hand-rolled parser; subset of JSON).
    static NeuroConfig from_json(const std::string& json_text);

    // Find config for a named integration; returns nullptr if not found.
    const IntegrationConfig* find(const std::string& kind) const;

    // Build a FallbackPolicy for an integration (respects global kill-switch).
    FallbackPolicy policy_for(const std::string& kind) const;
};

} // namespace ame::neuro
