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
    // Disjunctive goals: try each alternative in order and return the first that
    // is solvable. With a single alternative this is exactly solve(goalFluentIds).
    const auto& alternatives = wm.goalAlternatives();
    if (alternatives.size() <= 1) {
        return solve(wm, wm.goalFluentIds());
    }

    PlanResult last;
    for (const auto& alt : alternatives) {
        PlanResult r = solve(wm, alt);
        if (r.success) {
            return r;
        }
        last = std::move(r);
    }
    // None solvable: report the last attempt's diagnostics.
    if (last.error_msg.empty()) {
        last.error_msg = "No plan found for any goal alternative";
    }
    return last;
}

PlanResult Planner::solve(const WorldModel& wm,
                          const std::vector<unsigned>& goal_ids) const {
    PlanResult result;
    auto t0 = std::chrono::steady_clock::now();

    if (goal_ids.empty()) {
        result.error_msg = "Planner::solve: empty goal set";
        auto t1 = std::chrono::steady_clock::now();
        result.solve_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        return result;
    }
    for (auto goal_id : goal_ids) {
        if (goal_id >= wm.numFluents()) {
            result.error_msg = "Planner::solve: goal fluent id out of range";
            auto t1 = std::chrono::steady_clock::now();
            result.solve_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            return result;
        }
    }

    // Build action traversal order for LAPKT; default is identity [0, 1, ..., N-1].
    const unsigned n_actions = wm.numGroundActions();
    std::vector<unsigned> action_order(n_actions);
    std::iota(action_order.begin(), action_order.end(), 0u);

#if defined(AME_NEURO)
    // Consult neural heuristic hook (seam for Options A/D).
    // Non-empty scores reorder the action traversal passed to LAPKT so
    // higher-scored actions are expanded first in BRFS tie-breaking.
    // If hook returns empty, is absent, or throws, default ordering is used.
    // Exceptions are swallowed so a neural outage never blocks the symbolic plan.
    if (heuristic_hook_) {
        try {
            auto heuristic_scores = heuristic_hook_(wm, goal_ids);
            if (!heuristic_scores.empty()) {
                result.heuristic_source = "neural_hook";
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
        } catch (...) {
            // Hook/backend/codec failure: continue with default ordering.
            // heuristic_source stays "symbolic" — hook did not influence the plan.
        }
    }
#endif

    // Project world model to LAPKT STRIPS problem using the (possibly reordered)
    // action list.  projectToSTRIPS calls make_action_tables() internally —
    // do NOT call it again afterwards.
    aptk::STRIPS_Problem strips;
    wm.projectToSTRIPS(strips, action_order, goal_ids);

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
