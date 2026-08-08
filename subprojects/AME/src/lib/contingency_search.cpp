#include "ame/contingency_search.h"

#include "ame/pddl_parser.h"
#include "ame/planner.h"

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

namespace ame {

namespace {

unsigned popcount(unsigned value) {
    value = value - ((value >> 1) & 0x55555555u);
    value = (value & 0x33333333u) + ((value >> 2) & 0x33333333u);
    return (((value + (value >> 4)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24;
}

bool isSubset(unsigned a, unsigned b) {
    return (a & b) == a;
}

bool isSuperset(unsigned a, unsigned b) {
    return (a & b) == b;
}

std::string joinTrueFluents(const WorldModel& wm,
                            const std::vector<bool>& state) {
    std::string out;
    for (unsigned id = 0; id < wm.numFluents(); ++id) {
        if (id >= state.size() || !state[id]) {
            continue;
        }
        if (!out.empty()) {
            out += "; ";
        }
        out += wm.fluentName(id);
    }
    return out;
}

std::vector<bool> currentStateVector(const WorldModel& wm) {
    std::vector<bool> state(wm.numFluents(), false);
    for (unsigned id = 0; id < wm.numFluents(); ++id) {
        state[id] = wm.getFact(id);
    }
    return state;
}

std::vector<bool> applyPlanToState(const WorldModel& wm,
                                   const PlanResult& result,
                                   std::vector<bool> state) {
    for (const auto& step : result.steps) {
        const auto& action = wm.groundActions()[step.action_index];
        for (const unsigned id : action.del_effects) {
            if (id < state.size()) {
                state[id] = false;
            }
        }
        for (const unsigned id : action.add_effects) {
            if (id < state.size()) {
                state[id] = true;
            }
        }
    }
    return state;
}

std::string formatGoalOption(const std::vector<std::string>& goal_keys) {
    std::string out;
    for (size_t i = 0; i < goal_keys.size(); ++i) {
        if (i != 0) {
            out += " AND ";
        }
        out += goal_keys[i];
    }
    return out;
}

/// The schema name inside a grounded signature: "move(uav1,base)" -> "move".
std::string schemaName(const std::string& signature) {
    const auto paren = signature.find('(');
    return paren == std::string::npos ? signature : signature.substr(0, paren);
}

/// Plan for one combination, from a world model built fresh for it.
ContingencyCase solveCombination(
    const std::string& domain_pddl,
    const std::string& problem_pddl,
    const std::vector<ContextFact>& context_facts,
    unsigned combination,
    const std::vector<std::vector<std::string>>& goal_options) {
    ContingencyCase result;
    result.combination = combination;

    // A fresh world model per combination, so nothing carries over from the
    // last one: the planner is asked about this context and no other.
    WorldModel wm;
    PddlParser::parseFromString(domain_pddl, problem_pddl, wm);

    for (size_t i = 0; i < context_facts.size(); ++i) {
        const bool on = ((combination >> i) & 1u) != 0u;
        wm.setFact(context_facts[i].fluent_id, on);
    }
    const std::vector<bool> initial_state = currentStateVector(wm);
    result.initial_state = joinTrueFluents(wm, initial_state);

    Planner planner;
    PlanResult best;
    std::vector<std::string> best_goal;
    bool solved = false;

    for (const auto& goal_keys : goal_options) {
        wm.setGoal(goal_keys);
        const PlanResult attempt = planner.solve(wm);
        result.solve_time_ms += attempt.solve_time_ms;
        best = attempt;
        best_goal = goal_keys;
        if (attempt.success) {
            solved = true;
            break;
        }
    }

    if (!solved) {
        result.outcome = ContingencyOutcome::Unreachable;
        result.selected_goal = "(none reachable)";
        result.end_state = "(no safe end state)";
        return result;
    }

    result.outcome = ContingencyOutcome::Reachable;
    result.plan_length = static_cast<unsigned>(best.steps.size());
    result.selected_goal = formatGoalOption(best_goal);

    std::string names;
    std::string full_plan;
    for (const auto& step : best.steps) {
        const std::string& signature = wm.groundActions()[step.action_index].signature;
        const std::string name = schemaName(signature);
        if (names.find(name) == std::string::npos) {
            if (!names.empty()) {
                names += ", ";
            }
            names += name;
        }
        if (!full_plan.empty()) {
            full_plan += " -> ";
        }
        full_plan += signature;
    }
    result.action_names = names;
    result.full_plan = full_plan;
    result.end_state = joinTrueFluents(wm, applyPlanToState(wm, best, initial_state));
    return result;
}

}  // namespace

size_t ContingencySearchReport::reachableCount() const {
    return static_cast<size_t>(
        std::count_if(cases.begin(), cases.end(),
                      [](const ContingencyCase& c) { return c.reachable(); }));
}

size_t ContingencySearchReport::unreachableCount() const {
    return cases.size() - reachableCount();
}

bool ContingencySearchReport::everyCombinationReachable() const {
    return unreachableCount() == 0;
}

std::vector<ContextFact> ContingencySearch::identifyContextFacts(
    const WorldModel& wm) {
    std::vector<bool> in_effects(wm.numFluents(), false);
    std::vector<bool> in_preconditions(wm.numFluents(), false);

    for (unsigned i = 0; i < wm.numGroundActions(); ++i) {
        const auto& action = wm.groundActions()[i];
        for (const unsigned id : action.add_effects) {
            in_effects[id] = true;
        }
        for (const unsigned id : action.del_effects) {
            in_effects[id] = true;
        }
        for (const unsigned id : action.preconditions) {
            in_preconditions[id] = true;
        }
    }

    std::vector<ContextFact> facts;
    for (unsigned id = 0; id < wm.numFluents(); ++id) {
        if (in_effects[id] || !in_preconditions[id]) {
            continue;
        }
        const std::string& name = wm.fluentName(id);
        facts.push_back(ContextFact{id, name, name.substr(1, name.size() - 2),
                                    wm.getFact(id)});
    }
    return facts;
}

bool ContingencySearch::pruningIsSound(const WorldModel& wm) {
    for (unsigned i = 0; i < wm.numGroundActions(); ++i) {
        if (!wm.groundActions()[i].neg_preconditions.empty()) {
            return false;
        }
    }
    return true;
}

std::string ContingencySearch::combinationLabel(
    const std::vector<ContextFact>& facts, unsigned combination) {
    std::string label;
    for (size_t i = 0; i < facts.size(); ++i) {
        if (!label.empty()) {
            label += "  ";
        }
        label += facts[i].short_name;
        label += (((combination >> i) & 1u) != 0u) ? "=ON" : "=off";
    }
    return label;
}

ContingencySearchReport ContingencySearch::run(
    const std::string& domain_pddl,
    const std::string& problem_pddl,
    const ContingencySearchOptions& options) {
    const auto wall_start = std::chrono::steady_clock::now();

    ContingencySearchReport report;
    report.pruning_requested = options.prune;

    WorldModel template_wm;
    PddlParser::parseFromString(domain_pddl, problem_pddl, template_wm);
    report.context_facts = identifyContextFacts(template_wm);

    if (!options.only_predicates.empty()) {
        std::vector<ContextFact> wanted;
        for (const ContextFact& fact : report.context_facts) {
            // "(sensor-operational uav1)" belongs to the predicate
            // "sensor-operational", which is the first word inside the brackets.
            const std::string predicate =
                fact.short_name.substr(0, fact.short_name.find(' '));
            if (std::find(options.only_predicates.begin(),
                          options.only_predicates.end(),
                          predicate) != options.only_predicates.end()) {
                wanted.push_back(fact);
            }
        }
        report.context_facts = std::move(wanted);
    }

    report.goal_options = options.goal_options;
    if (report.goal_options.empty()) {
        std::vector<std::string> goal_keys;
        for (const unsigned id : template_wm.goalFluentIds()) {
            goal_keys.push_back(template_wm.fluentName(id));
        }
        report.goal_options.push_back(std::move(goal_keys));
    }

    bool prune = options.prune;
    if (prune && !pruningIsSound(template_wm)) {
        prune = false;
        report.pruning_unsound_for_domain = true;
    }
    report.pruning_used = prune;

    const unsigned fact_count = static_cast<unsigned>(report.context_facts.size());
    const unsigned combination_count = 1u << fact_count;
    report.cases.resize(combination_count);
    for (unsigned i = 0; i < combination_count; ++i) {
        report.cases[i].combination = i;
        report.cases[i].outcome = ContingencyOutcome::Unreachable;
    }
    std::vector<bool> decided(combination_count, false);

    // Fewest facts first, because a combination that is reachable with little
    // available settles every richer combination in one call, which is where
    // most of the saving comes from.
    std::vector<unsigned> order(combination_count);
    for (unsigned i = 0; i < combination_count; ++i) {
        order[i] = i;
    }
    std::sort(order.begin(), order.end(), [](unsigned a, unsigned b) {
        return popcount(a) < popcount(b);
    });

    for (const unsigned combination : order) {
        if (decided[combination]) {
            continue;
        }

        if (prune) {
            bool settled = false;
            for (unsigned other = 0; other < combination_count && !settled; ++other) {
                if (!decided[other] || other == combination) {
                    continue;
                }
                if (report.cases[other].outcome == ContingencyOutcome::Reachable &&
                    isSuperset(combination, other)) {
                    report.cases[combination].outcome =
                        ContingencyOutcome::ImpliedReachable;
                    report.cases[combination].implied_by = other;
                    report.cases[combination].has_implied_by = true;
                    decided[combination] = true;
                    ++report.implied_reachable;
                    settled = true;
                }
            }
            for (unsigned other = 0; other < combination_count && !settled; ++other) {
                if (!decided[other] || other == combination) {
                    continue;
                }
                if (report.cases[other].outcome == ContingencyOutcome::Unreachable &&
                    isSubset(combination, other)) {
                    report.cases[combination].outcome =
                        ContingencyOutcome::ImpliedUnreachable;
                    report.cases[combination].implied_by = other;
                    report.cases[combination].has_implied_by = true;
                    decided[combination] = true;
                    ++report.implied_unreachable;
                    settled = true;
                }
            }
            if (settled) {
                continue;
            }
        }

        report.cases[combination] =
            solveCombination(domain_pddl, problem_pddl, report.context_facts,
                             combination, report.goal_options);
        decided[combination] = true;
        ++report.solver_calls;

        if (!prune) {
            continue;
        }

        // Carry the conclusion to every combination it settles, so those are
        // never planned for at all.
        if (report.cases[combination].outcome == ContingencyOutcome::Reachable) {
            for (unsigned other = 0; other < combination_count; ++other) {
                if (decided[other] || !isSuperset(other, combination)) {
                    continue;
                }
                report.cases[other].outcome = ContingencyOutcome::ImpliedReachable;
                report.cases[other].implied_by = combination;
                report.cases[other].has_implied_by = true;
                decided[other] = true;
                ++report.implied_reachable;
            }
        } else if (report.cases[combination].outcome ==
                   ContingencyOutcome::Unreachable) {
            for (unsigned other = 0; other < combination_count; ++other) {
                if (decided[other] || !isSubset(other, combination)) {
                    continue;
                }
                report.cases[other].outcome = ContingencyOutcome::ImpliedUnreachable;
                report.cases[other].implied_by = combination;
                report.cases[other].has_implied_by = true;
                decided[other] = true;
                ++report.implied_unreachable;
            }
        }
    }

    const auto wall_end = std::chrono::steady_clock::now();
    report.wall_time_ms =
        std::chrono::duration<double, std::milli>(wall_end - wall_start).count();
    return report;
}

}  // namespace ame
