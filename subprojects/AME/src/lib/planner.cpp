#include "ame/planner.h"

#include <strips_prob.hxx>
#include <strips_state.hxx>
#include <action.hxx>
#include <fwd_search_prob.hxx>
#include <brfs.hxx>

#include <algorithm>
#include <chrono>
#include <numeric>
#include <unordered_map>

namespace ame {

using BRFS_Engine = aptk::search::brfs::BRFS<aptk::agnostic::Fwd_Search_Problem>;

PlanResult Planner::solve(const WorldModel& wm) const {
    PlanResult result;
    auto t0 = std::chrono::steady_clock::now();

    // Build action traversal order for LAPKT; default is identity [0, 1, ..., N-1].
    const unsigned n_actions = wm.numGroundActions();
    std::vector<unsigned> action_order(n_actions);
    std::iota(action_order.begin(), action_order.end(), 0u);

#if defined(AME_NEURO)
    // Consult neural heuristic hook (seam for Options A/D).
    // Non-empty scores reorder the action traversal passed to LAPKT so
    // higher-scored actions are expanded first in BRFS tie-breaking.
    // If hook returns empty or is absent, default ordering is used unchanged.
    if (heuristic_hook_) {
        auto heuristic_scores = heuristic_hook_(wm, wm.goalFluentIds());
        result.heuristic_source = "neural_hook"; // set whenever hook fires

        if (!heuristic_scores.empty()) {
            std::unordered_map<unsigned, float> score_map;
            score_map.reserve(heuristic_scores.size());
            for (const auto& s : heuristic_scores)
                score_map[s.ground_action_id] = s.score;

            // Stable sort preserves tie-breaking order among unscored actions.
            std::stable_sort(action_order.begin(), action_order.end(),
                             [&](unsigned a, unsigned b) {
                                 auto it_a = score_map.find(a);
                                 auto it_b = score_map.find(b);
                                 float sa = (it_a != score_map.end()) ? it_a->second : 0.0f;
                                 float sb = (it_b != score_map.end()) ? it_b->second : 0.0f;
                                 return sa > sb; // descending
                             });
        }
    }
#endif

    // Project world model to LAPKT STRIPS problem using the (possibly reordered)
    // action list.  projectToSTRIPS calls make_action_tables() internally —
    // do NOT call it again afterwards.
    aptk::STRIPS_Problem strips;
    wm.projectToSTRIPS(strips, action_order);

    // Build mapping from LAPKT action index -> WorldModel ground action index.
    // LAPKT assigns index i to the i-th action added, which is action_order[i].
    std::unordered_map<int, unsigned> lapkt_to_wm;
    lapkt_to_wm.reserve(n_actions);
    for (unsigned i = 0; i < n_actions; ++i)
        lapkt_to_wm[static_cast<int>(i)] = action_order[i];

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
