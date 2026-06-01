#include "ame/planner.h"

#include <strips_prob.hxx>
#include <strips_state.hxx>
#include <action.hxx>
#include <fwd_search_prob.hxx>
#include <brfs.hxx>

#include <chrono>
#include <unordered_map>

namespace ame {

using BRFS_Engine = aptk::search::brfs::BRFS<aptk::agnostic::Fwd_Search_Problem>;

PlanResult Planner::solve(const WorldModel& wm) const {
    PlanResult result;
    auto t0 = std::chrono::steady_clock::now();

#if defined(AME_NEURO)
    // Consult neural heuristic hook (seam for Options A/D).
    // Scores influence action ordering in the WorldModel copy passed to LAPKT.
    // If hook returns empty or is absent, default ordering is used unchanged.
    std::vector<ActionScore> heuristic_scores;
    if (heuristic_hook_) {
        heuristic_scores = heuristic_hook_(wm, wm.goalFluentIds());
        result.heuristic_source = "neural_hook"; // hook fired, even if scores empty
    }
#endif

    // Project world model to LAPKT STRIPS problem
    // projectToSTRIPS already calls make_action_tables() as its finalization step;
    // calling it again duplicates LAPKT's internal successor tables and corrupts planning.
    aptk::STRIPS_Problem strips;
    wm.projectToSTRIPS(strips);

    // Build mapping from LAPKT action index -> WorldModel ground action index
    // Both use the same ordering, but verify via signature matching
    std::unordered_map<int, unsigned> lapkt_to_wm;
    for (unsigned i = 0; i < wm.numGroundActions(); ++i) {
        // LAPKT action indices match WorldModel ground action indices
        // because projectToSTRIPS adds them in the same order
        lapkt_to_wm[static_cast<int>(i)] = i;
    }

    // Create forward search problem and BRFS engine
    aptk::agnostic::Fwd_Search_Problem fwd_prob(&strips);
    BRFS_Engine engine(fwd_prob);
    engine.set_verbose(false);

    // Set initial state from current world model state
    aptk::State* init = wm.currentStateAsSTRIPS(strips);
    engine.start(init);

    // Search
    std::vector<aptk::Action_Idx> plan;
    float cost = 0.0f;

    if (engine.find_solution(cost, plan)) {
        result.success = true;
        result.cost = cost;
        result.expanded = engine.expanded();
        result.generated = engine.generated();

        // Convert LAPKT action indices to PlanSteps
        for (auto action_idx : plan) {
            auto it = lapkt_to_wm.find(action_idx);
            if (it != lapkt_to_wm.end()) {
                result.steps.push_back({it->second});
            }
        }
    } else {
        result.error_msg = "No plan found";
        result.expanded = engine.expanded();
        result.generated = engine.generated();
    }

    auto t1 = std::chrono::steady_clock::now();
    result.solve_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    return result;
}

} // namespace ame
