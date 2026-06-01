#pragma once

#include "ame/world_model.h"
#include "ame/plan_compiler.h"

#include <functional>
#include <string>
#include <vector>

namespace ame {

// Result of a planning attempt
struct PlanResult {
    bool success = false;
    std::vector<PlanStep> steps;
    float cost = 0.0f;
    unsigned expanded = 0;
    unsigned generated = 0;
    double solve_time_ms = 0.0;  // Wall-clock solve duration
    std::string error_msg;
    std::string heuristic_source = "symbolic"; // "symbolic" or backend id if hook fired
};

#if defined(AME_NEURO)
// Heuristic score: higher = prefer this action earlier in search.
struct ActionScore {
    unsigned ground_action_id;
    float score;
};

// Optional hook for neural heuristic guidance (Options A/D).
// Called before search; returns per-action preference scores used to
// reorder ground actions supplied to the planner.  The symbolic solver
// remains the authority: all returned plans satisfy PDDL preconditions.
// If the hook returns an empty vector, default ordering is used.
using HeuristicHook =
    std::function<std::vector<ActionScore>(const WorldModel&,
                                           const std::vector<unsigned>& goal_ids)>;
#endif

class Planner {
public:
    // Solve: project WorldModel to STRIPS, run BRFS, return plan steps.
    // The returned PlanStep::action_index values correspond to WorldModel::groundActions().
    PlanResult solve(const WorldModel& wm) const;

#if defined(AME_NEURO)
    // Attach a neural heuristic hook. Null-object default (no hook) preserves
    // byte-for-byte baseline behaviour.
    void setHeuristicHook(HeuristicHook hook) { heuristic_hook_ = std::move(hook); }
    void clearHeuristicHook() { heuristic_hook_ = nullptr; }
    bool hasHeuristicHook() const { return static_cast<bool>(heuristic_hook_); }

private:
    HeuristicHook heuristic_hook_;
#endif
};

} // namespace ame
