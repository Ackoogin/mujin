#pragma once

#include "ame/world_model.h"
#include "ame/plan_compiler.h"

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
};

class Planner {
public:
    // Solve: project WorldModel to STRIPS, run BRFS, return plan steps
    // The returned PlanStep::action_index values correspond to WorldModel::groundActions()
    PlanResult solve(const WorldModel& wm) const;
};

} // namespace ame
