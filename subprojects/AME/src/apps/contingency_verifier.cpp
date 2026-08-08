// =========================================================================
// contingency_verifier — exhaustive safe-state reachability analysis
// =========================================================================
// Accepts a PDDL domain + template problem, automatically identifies
// context predicates (system health, environment state), enumerates all
// 2^N combinations, and proves whether every possible world state has
// a valid plan to reach the goal.
//
// Context predicates represent externally-determined facts — hardware
// health, sensor status, comms availability, weather — that change due
// to real-world events outside the planner's control. In the real
// system these are always CONFIRMED by perception, never BELIEVED by
// the planner. They are identified as predicates that gate actions
// (appear in preconditions) but are never produced or consumed by any
// action (absent from all add/delete effects). The planner treats each
// snapshot of these predicates as given context and plans accordingly.
//
// This tool enumerates every possible context snapshot and verifies
// that the planner can always find a path to the safety goal.
//
// Optimisation: monotone dominance pruning.
// Because context predicates only appear as positive preconditions,
// additional capabilities can only enable actions, never disable them.
// If a plan exists in context S, the same plan is valid in any context
// S' ⊇ S. This lets us skip exponentially many solver calls.
//
// Usage:
//   contingency_verifier <domain.pddl> <template_problem.pddl> [options]
//
// Options:
//   --no-prune     Disable monotonicity pruning (solve every combination)
//   --json <file>  Write machine-readable report to JSON file
//   --safety-goal <expr>
//                  Add an acceptable safety goal. Repeat for OR alternatives.
//   --verbose      Show full plan signatures
// =========================================================================

#include "ame/contingency_search.h"
#include "ame/pddl_parser.h"
#include "ame/planner.h"
#include "ame/world_model.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

// -------------------------------------------------------------------------
// Context predicate detection
// -------------------------------------------------------------------------
// A context predicate is one that the planner cannot change — it never
// appears in any action's add or delete effects. In real systems these
// represent externally-determined facts (hardware health, sensor status,
// comms link, weather) that are always CONFIRMED by perception. They
// change due to real-world events between replanning cycles, but within
// any single planning snapshot they are fixed context.
//
// Finding them, enumerating their combinations and carrying conclusions
// between those combinations all live in ame_core, so that this tool, the
// authoring tool and the generated assurance report cannot disagree about
// what a domain's contingencies are. See ame/contingency_search.h. What stays
// here is this tool's own presentation of the answer.

using HealthVar = ame::ContextFact;

// -------------------------------------------------------------------------
// Combination result
// -------------------------------------------------------------------------

enum class ResultKind {
    SOLVED,            // planner found a plan
    UNSOLVABLE,        // planner exhausted search space
    IMPLIED_SAFE,      // monotonicity: superset of a solved state
    IMPLIED_UNSAFE,    // monotonicity: subset of an unsolvable state
};

struct ComboResult {
    unsigned combo;
    ResultKind kind;
    unsigned plan_length = 0;
    double solve_time_ms = 0.0;
    std::string selected_goal;
    std::string initial_state;
    std::string end_state;
    std::string actions;
    std::string full_plan;
};

static std::string joinFluents(const ame::WorldModel& wm,
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

static std::vector<bool> currentStateVector(const ame::WorldModel& wm) {
    std::vector<bool> state(wm.numFluents(), false);
    for (unsigned id = 0; id < wm.numFluents(); ++id) {
        state[id] = wm.getFact(id);
    }
    return state;
}

static std::vector<bool> applyPlanToState(
    const ame::WorldModel& wm,
    const ame::PlanResult& result,
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

// The forward declaration stays because the reachable-state expansion below
// formats goal options as well.
static std::string formatGoalOption(const std::vector<std::string>& goal_keys);


static bool hasFluent(const ame::WorldModel& wm, const std::string& key) {
    for (unsigned id = 0; id < wm.numFluents(); ++id) {
        if (wm.fluentName(id) == key) {
            return true;
        }
    }
    return false;
}

static std::string trim(const std::string& text) {
    const size_t begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const size_t end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

static bool isAndExpression(const std::string& text) {
    if (text.size() < 5 || text.front() != '(' || text.back() != ')') {
        return false;
    }
    size_t pos = 1;
    while (pos < text.size() &&
           (text[pos] == ' ' || text[pos] == '\t' ||
            text[pos] == '\r' || text[pos] == '\n')) {
        ++pos;
    }
    return text.compare(pos, 3, "and") == 0 &&
           (pos + 3 == text.size() - 1 ||
            text[pos + 3] == ' ' || text[pos + 3] == '\t' ||
            text[pos + 3] == '\r' || text[pos + 3] == '\n');
}

static std::string stripAndWrapper(const std::string& text) {
    size_t pos = 1;
    while (pos < text.size() &&
           (text[pos] == ' ' || text[pos] == '\t' ||
            text[pos] == '\r' || text[pos] == '\n')) {
        ++pos;
    }
    pos += 3;
    return trim(text.substr(pos, text.size() - pos - 1));
}

static std::vector<std::string> extractFactExpressions(const std::string& text) {
    const std::string trimmed = trim(text);
    const std::string expr = isAndExpression(trimmed)
                                 ? stripAndWrapper(trimmed)
                                 : trimmed;

    std::vector<std::string> facts;
    int depth = 0;
    size_t start = std::string::npos;
    for (size_t i = 0; i < expr.size(); ++i) {
        const char ch = expr[i];
        if (ch == '(') {
            if (depth == 0) {
                start = i;
            }
            ++depth;
        } else if (ch == ')') {
            --depth;
            if (depth < 0) {
                throw std::runtime_error("unmatched ')' in safety goal: " + text);
            }
            if (depth == 0 && start != std::string::npos) {
                facts.push_back(expr.substr(start, i - start + 1));
                start = std::string::npos;
            }
        }
    }

    if (depth != 0) {
        throw std::runtime_error("unclosed '(' in safety goal: " + text);
    }
    if (facts.empty()) {
        throw std::runtime_error("safety goal has no facts: " + text);
    }

    return facts;
}

static void validateGoalOption(const ame::WorldModel& wm,
                               const std::vector<std::string>& goal_keys) {
    for (const auto& key : goal_keys) {
        if (!hasFluent(wm, key)) {
            throw std::runtime_error("safety goal fluent is not grounded: " + key);
        }
    }
}

static std::string formatGoalOption(const std::vector<std::string>& goal_keys) {
    std::string text;
    for (size_t i = 0; i < goal_keys.size(); ++i) {
        if (i > 0) text += " AND ";
        text += goal_keys[i];
    }
    return text;
}

static const char* kindStr(ResultKind k);
static std::string escapeJson(const std::string& s);

// -------------------------------------------------------------------------
// Reachable-state expansion sidecar
// -------------------------------------------------------------------------

struct FactFilter {
    std::string name;
    std::vector<unsigned> all_true;
    std::vector<unsigned> all_false;
};

struct ContextVariant {
    std::string name;
    std::vector<unsigned> set_true;
    std::vector<unsigned> set_false;
};

struct ContextDimension {
    std::string name;
    std::vector<ContextVariant> variants;
};

struct ExpansionSidecar {
    std::string name;
    unsigned max_reachable_depth = 6;
    std::vector<ContextDimension> dimensions;
    std::vector<FactFilter> invalid_when;
};

struct ReachableState {
    unsigned id = 0;
    std::vector<bool> state;
    std::vector<std::string> path;
};

struct ExpansionCaseResult {
    unsigned case_index = 0;
    unsigned base_state_id = 0;
    std::string base_path;
    std::string context_label;
    ResultKind kind = ResultKind::UNSOLVABLE;
    unsigned plan_length = 0;
    double solve_time_ms = 0.0;
    std::string selected_goal;
    std::string initial_state;
    std::string end_state;
    std::string actions;
    std::string full_plan;
};

static unsigned requireFluentId(const ame::WorldModel& wm,
                                const std::string& key,
                                const std::string& field_name) {
    try {
        return wm.fluentIndex(key);
    } catch (const std::exception&) {
        throw std::runtime_error(
            "sidecar " + field_name + " references unknown fluent: " + key);
    }
}

static std::vector<unsigned> parseFactList(const ame::WorldModel& wm,
                                           const nlohmann::json& object,
                                           const std::string& key,
                                           const std::string& field_name) {
    std::vector<unsigned> ids;
    if (!object.contains(key)) {
        return ids;
    }
    if (!object.at(key).is_array()) {
        throw std::runtime_error("sidecar " + field_name + "." + key +
                                 " must be an array");
    }
    for (const auto& item : object.at(key)) {
        if (!item.is_string()) {
            throw std::runtime_error("sidecar " + field_name + "." + key +
                                     " entries must be strings");
        }
        ids.push_back(requireFluentId(wm, item.get<std::string>(),
                                      field_name + "." + key));
    }
    return ids;
}

static FactFilter parseFactFilter(const ame::WorldModel& wm,
                                  const nlohmann::json& object,
                                  const std::string& fallback_name,
                                  const std::string& field_name) {
    FactFilter filter;
    filter.name = object.value("name", fallback_name);
    filter.all_true = parseFactList(wm, object, "all_true", field_name);
    filter.all_false = parseFactList(wm, object, "all_false", field_name);
    return filter;
}

static ExpansionSidecar loadExpansionSidecar(const std::string& path,
                                             const ame::WorldModel& wm) {
    std::ifstream file(path);
    if (!file.good()) {
        throw std::runtime_error("cannot open expansion sidecar: " + path);
    }

    nlohmann::json json;
    file >> json;

    ExpansionSidecar sidecar;
    sidecar.name = json.value("name", std::string("reachable-state expansion"));
    sidecar.max_reachable_depth = json.value("max_reachable_depth", 6u);

    const char* domains_key = json.contains("finite_domains")
                                  ? "finite_domains"
                                  : "dimensions";
    if (!json.contains(domains_key) || !json.at(domains_key).is_array()) {
        throw std::runtime_error(
            "sidecar must contain a finite_domains array");
    }
    for (const auto& dimension_json : json.at(domains_key)) {
        ContextDimension dimension;
        dimension.name = dimension_json.value("name", std::string());
        if (dimension.name.empty()) {
            throw std::runtime_error("sidecar dimension missing name");
        }
        if (!dimension_json.contains("variants") ||
            !dimension_json.at("variants").is_array()) {
            throw std::runtime_error("sidecar dimension '" + dimension.name +
                                     "' must contain a variants array");
        }
        for (const auto& variant_json : dimension_json.at("variants")) {
            ContextVariant variant;
            variant.name = variant_json.value("name", std::string());
            if (variant.name.empty()) {
                throw std::runtime_error("sidecar dimension '" + dimension.name +
                                         "' has variant missing name");
            }
            const std::string field_name =
                "dimension '" + dimension.name + "' variant '" +
                variant.name + "'";
            variant.set_true =
                parseFactList(wm, variant_json, "set_true", field_name);
            variant.set_false =
                parseFactList(wm, variant_json, "set_false", field_name);
            dimension.variants.push_back(std::move(variant));
        }
        sidecar.dimensions.push_back(std::move(dimension));
    }

    if (json.contains("invalid_when")) {
        if (!json.at("invalid_when").is_array()) {
            throw std::runtime_error("sidecar invalid_when must be an array");
        }
        for (size_t i = 0; i < json.at("invalid_when").size(); ++i) {
            sidecar.invalid_when.push_back(parseFactFilter(
                wm, json.at("invalid_when").at(i),
                "invalid rule " + std::to_string(i), "invalid_when"));
        }
    }

    return sidecar;
}

static bool matchesFilter(const std::vector<bool>& state,
                          const FactFilter& filter) {
    for (const unsigned id : filter.all_true) {
        if (id >= state.size() || !state[id]) {
            return false;
        }
    }
    for (const unsigned id : filter.all_false) {
        if (id < state.size() && state[id]) {
            return false;
        }
    }
    return true;
}

static bool isInvalidState(const std::vector<bool>& state,
                           const ExpansionSidecar& sidecar) {
    for (const auto& invalid : sidecar.invalid_when) {
        if (matchesFilter(state, invalid)) {
            return true;
        }
    }
    return false;
}

static bool actionApplicable(const ame::GroundAction& action,
                             const std::vector<bool>& state) {
    for (const unsigned id : action.preconditions) {
        if (id >= state.size() || !state[id]) {
            return false;
        }
    }
    return true;
}

static std::vector<bool> applyActionToState(const ame::GroundAction& action,
                                            std::vector<bool> state) {
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
    return state;
}

static std::string stateKey(const std::vector<bool>& state) {
    std::string key;
    key.reserve(state.size());
    for (const bool value : state) {
        key.push_back(value ? '1' : '0');
    }
    return key;
}

static std::string joinPath(const std::vector<std::string>& path) {
    if (path.empty()) {
        return "(template initial state)";
    }
    std::string text;
    for (const auto& step : path) {
        if (!text.empty()) {
            text += " -> ";
        }
        text += step;
    }
    return text;
}

static std::vector<ReachableState> enumerateReachableStates(
    const ame::WorldModel& wm,
    const ExpansionSidecar& sidecar) {
    std::vector<ReachableState> states;
    std::deque<ReachableState> queue;
    std::unordered_set<std::string> seen;

    ReachableState initial;
    initial.state = currentStateVector(wm);
    initial.id = 0;
    seen.insert(stateKey(initial.state));
    queue.push_back(initial);

    while (!queue.empty()) {
        ReachableState current = queue.front();
        queue.pop_front();
        current.id = static_cast<unsigned>(states.size());
        states.push_back(current);

        if (current.path.size() >= sidecar.max_reachable_depth) {
            continue;
        }

        for (const auto& action : wm.groundActions()) {
            if (!actionApplicable(action, current.state)) {
                continue;
            }

            ReachableState next;
            next.state = applyActionToState(action, current.state);
            if (isInvalidState(next.state, sidecar)) {
                continue;
            }

            const std::string key = stateKey(next.state);
            if (seen.find(key) != seen.end()) {
                continue;
            }

            seen.insert(key);
            next.path = current.path;
            next.path.push_back(action.signature);
            queue.push_back(std::move(next));
        }
    }

    return states;
}

static ExpansionCaseResult solveExpandedState(
    const std::string& domain_pddl,
    const std::string& problem_pddl_template,
    const std::vector<bool>& state,
    const std::vector<std::vector<std::string>>& goal_options) {
    ExpansionCaseResult result;

    ame::WorldModel wm;
    ame::PddlParser::parseFromString(domain_pddl, problem_pddl_template, wm);
    for (unsigned id = 0; id < wm.numFluents(); ++id) {
        wm.setFact(id, id < state.size() && state[id]);
    }

    result.initial_state = joinFluents(wm, state);

    ame::Planner planner;
    ame::PlanResult best_result;
    std::vector<std::string> best_goal;
    bool solved = false;

    for (const auto& goal_keys : goal_options) {
        wm.setGoal(goal_keys);
        auto plan_result = planner.solve(wm);
        result.solve_time_ms += plan_result.solve_time_ms;
        best_result = plan_result;
        best_goal = goal_keys;
        if (plan_result.success) {
            solved = true;
            break;
        }
    }

    if (!solved) {
        result.kind = ResultKind::UNSOLVABLE;
        result.selected_goal = "(none reachable)";
        result.end_state = "(no safe end state)";
        return result;
    }

    result.kind = ResultKind::SOLVED;
    result.plan_length = static_cast<unsigned>(best_result.steps.size());
    result.selected_goal = formatGoalOption(best_goal);

    std::set<std::string> action_names;
    for (const auto& step : best_result.steps) {
        const auto& signature = wm.groundActions()[step.action_index].signature;
        const auto paren = signature.find('(');
        action_names.insert(paren == std::string::npos
                                ? signature
                                : signature.substr(0, paren));
        if (!result.full_plan.empty()) {
            result.full_plan += " -> ";
        }
        result.full_plan += signature;
    }

    for (const auto& action_name : action_names) {
        if (!result.actions.empty()) {
            result.actions += ", ";
        }
        result.actions += action_name;
    }
    if (result.actions.empty()) {
        result.actions = "(already at goal)";
    }

    result.end_state =
        joinFluents(wm, applyPlanToState(wm, best_result, state));
    return result;
}

static void applyVariant(std::vector<bool>& state,
                         const ContextVariant& variant) {
    for (const unsigned id : variant.set_false) {
        if (id < state.size()) {
            state[id] = false;
        }
    }
    for (const unsigned id : variant.set_true) {
        if (id < state.size()) {
            state[id] = true;
        }
    }
}

static void enumerateExpansionCasesForBase(
    const std::string& domain_pddl,
    const std::string& problem_pddl,
    const ExpansionSidecar& sidecar,
    const ReachableState& base_state,
    const std::vector<std::vector<std::string>>& goal_options,
    size_t dimension_index,
    std::vector<bool> current_state,
    std::string context_label,
    std::vector<ExpansionCaseResult>& results) {
    if (dimension_index == sidecar.dimensions.size()) {
        if (isInvalidState(current_state, sidecar)) {
            return;
        }

        ExpansionCaseResult result = solveExpandedState(
            domain_pddl, problem_pddl, current_state, goal_options);
        result.case_index = static_cast<unsigned>(results.size());
        result.base_state_id = base_state.id;
        result.base_path = joinPath(base_state.path);
        result.context_label = context_label;
        results.push_back(std::move(result));
        return;
    }

    const auto& dimension = sidecar.dimensions[dimension_index];
    for (const auto& variant : dimension.variants) {
        std::vector<bool> next_state = current_state;
        applyVariant(next_state, variant);
        std::string next_label = context_label;
        if (!next_label.empty()) {
            next_label += "  ";
        }
        next_label += dimension.name + "=" + variant.name;
        enumerateExpansionCasesForBase(domain_pddl, problem_pddl, sidecar,
                                       base_state, goal_options,
                                       dimension_index + 1, next_state,
                                       next_label, results);
    }
}

static void writeExpansionJson(
    const std::string& path,
    const std::string& domain_file,
    const std::string& problem_file,
    const std::string& sidecar_file,
    const ExpansionSidecar& sidecar,
    const std::vector<ReachableState>& reachable_states,
    const std::vector<ExpansionCaseResult>& results,
    double total_time_ms) {
    std::ofstream f(path);
    if (!f.good()) {
        std::cerr << "Error: cannot write to " << path << "\n";
        return;
    }

    unsigned safe = 0;
    unsigned gaps = 0;
    for (const auto& result : results) {
        const bool is_safe = result.kind == ResultKind::SOLVED;
        if (is_safe) {
            safe++;
        } else {
            gaps++;
        }
    }

    f << "{\n";
    f << "  \"mode\": \"reachable_state_expansion\",\n";
    f << "  \"name\": \"" << escapeJson(sidecar.name) << "\",\n";
    f << "  \"domain\": \"" << escapeJson(domain_file) << "\",\n";
    f << "  \"problem_template\": \"" << escapeJson(problem_file) << "\",\n";
    f << "  \"sidecar\": \"" << escapeJson(sidecar_file) << "\",\n";
    f << "  \"reachable_state_count\": " << reachable_states.size() << ",\n";
    f << "  \"verified_state_count\": " << reachable_states.size() << ",\n";
    f << "  \"total_cases\": " << results.size() << ",\n";
    f << "  \"safe_count\": " << safe << ",\n";
    f << "  \"gap_count\": " << gaps << ",\n";
    f << "  \"total_time_ms\": " << std::fixed << std::setprecision(1)
      << total_time_ms << ",\n";

    f << "  \"cases\": [\n";
    for (size_t i = 0; i < results.size(); ++i) {
        const auto& result = results[i];
        f << "    {\n";
        f << "      \"case\": " << result.case_index << ",\n";
        f << "      \"base_state\": " << result.base_state_id << ",\n";
        f << "      \"base_path\": \"" << escapeJson(result.base_path) << "\",\n";
        f << "      \"context\": \"" << escapeJson(result.context_label) << "\",\n";
        f << "      \"result\": \"" << kindStr(result.kind) << "\",\n";
        f << "      \"plan_length\": " << result.plan_length << ",\n";
        f << "      \"solve_time_ms\": " << std::fixed << std::setprecision(2)
          << result.solve_time_ms << ",\n";
        f << "      \"selected_goal\": \""
          << escapeJson(result.selected_goal) << "\",\n";
        f << "      \"initial_state\": \""
          << escapeJson(result.initial_state) << "\",\n";
        f << "      \"end_state\": \"" << escapeJson(result.end_state) << "\",\n";
        f << "      \"actions\": \"" << escapeJson(result.actions) << "\",\n";
        f << "      \"plan\": \"" << escapeJson(result.full_plan) << "\"\n";
        f << "    }";
        if (i + 1 < results.size()) {
            f << ",";
        }
        f << "\n";
    }
    f << "  ]\n";
    f << "}\n";
}

static int runReachableExpansion(
    const std::string& domain_file,
    const std::string& problem_file,
    const std::string& sidecar_file,
    const std::string& json_file,
    const std::string& domain_pddl,
    const std::string& problem_pddl,
    const ame::WorldModel& template_wm,
    const std::vector<std::vector<std::string>>& goal_options,
    bool verbose) {
    ExpansionSidecar sidecar;
    try {
        sidecar = loadExpansionSidecar(sidecar_file, template_wm);
    } catch (const std::exception& e) {
        std::cerr << "Error parsing expansion sidecar: " << e.what() << "\n";
        return 1;
    }

    const auto wall_start = std::chrono::steady_clock::now();
    const std::vector<ReachableState> reachable_states =
        enumerateReachableStates(template_wm, sidecar);

    std::vector<ExpansionCaseResult> results;
    for (const auto& base_state : reachable_states) {
        enumerateExpansionCasesForBase(domain_pddl, problem_pddl, sidecar,
                                       base_state, goal_options, 0,
                                       base_state.state, "", results);
    }

    const auto wall_end = std::chrono::steady_clock::now();
    const double total_ms = std::chrono::duration<double, std::milli>(
                                wall_end - wall_start).count();

    unsigned safe_count = 0;
    unsigned gap_count = 0;
    for (const auto& result : results) {
        if (result.kind == ResultKind::SOLVED) {
            safe_count++;
        } else {
            gap_count++;
        }
    }

    std::cout << "\n"
              << "============================================================\n"
              << "CONTINGENCY VERIFIER — REACHABLE-STATE EXPANSION\n"
              << "============================================================\n"
              << "\n"
              << "Domain:    " << domain_file << "\n"
              << "Template:  " << problem_file << "\n"
              << "Sidecar:   " << sidecar_file << "\n"
              << "Analysis:  " << sidecar.name << "\n"
              << "Depth:     " << sidecar.max_reachable_depth << "\n"
              << "\n"
              << "Reachable planner states:  " << reachable_states.size() << "\n"
              << "Finite-domain cases:       " << results.size() << "\n"
              << "\nSafety goals:\n";
    for (size_t i = 0; i < goal_options.size(); ++i) {
        std::cout << "  [" << (i + 1) << "] "
                  << formatGoalOption(goal_options[i]) << "\n";
    }

    std::cout << "\nVerified reachable states:\n";
    for (const auto& base_state : reachable_states) {
        std::cout << "  [" << base_state.id << "] "
                  << joinPath(base_state.path) << "\n";
    }

    size_t context_width = 10;
    for (const auto& result : results) {
        context_width = std::max(context_width, result.context_label.size());
    }
    context_width += 2;

    std::cout << "\n"
              << std::left
              << std::setw(7) << "Case"
              << std::setw(8) << "Base"
              << std::setw(static_cast<int>(context_width)) << "Context"
              << std::setw(10) << "Result"
              << "Actions\n"
              << std::string(context_width + 65, '-') << "\n";

    for (const auto& result : results) {
        std::string details;
        if (result.kind == ResultKind::UNSOLVABLE) {
            details = "(no plan)";
        } else {
            details = verbose && !result.full_plan.empty()
                          ? result.full_plan
                          : result.actions;
        }

        std::cout << std::left
                  << std::setw(7) << result.case_index
                  << std::setw(8) << result.base_state_id
                  << std::setw(static_cast<int>(context_width))
                  << result.context_label
                  << std::setw(10) << kindStr(result.kind)
                  << details << "\n";
    }

    std::cout << "\n"
              << "============================================================\n"
              << "GAP DETAILS\n"
              << "============================================================\n";
    for (const auto& result : results) {
        if (result.kind == ResultKind::SOLVED) {
            continue;
        }
        std::cout << "\n[GAP] case " << result.case_index
                  << " base " << result.base_state_id << "\n"
                  << "  Base path: " << result.base_path << "\n"
                  << "  Context:   " << result.context_label << "\n";
        std::cout << "  Start true fluents: " << result.initial_state << "\n"
                  << "  Gap: no plan reaches any configured safety goal.\n";
    }

    std::cout << "\n"
              << "============================================================\n"
              << "SUMMARY\n"
              << "============================================================\n"
              << "  Reachable planner states:  " << reachable_states.size() << "\n"
              << "  Finite-domain cases:       " << results.size() << "\n"
              << "  Wall-clock time:           " << std::fixed << std::setprecision(1)
              << total_ms << " ms\n"
              << "\n"
              << "  SAFE:  " << safe_count << "/" << results.size() << "\n"
              << "  GAPS:  " << gap_count << "/" << results.size() << "\n";

    if (gap_count == 0) {
        std::cout << "\n  RESULT: ALL EXPANDED STATES REACH SAFE-STATE\n";
    }
    std::cout << "============================================================\n\n";

    if (!json_file.empty()) {
        writeExpansionJson(json_file, domain_file, problem_file, sidecar_file,
                           sidecar, reachable_states, results, total_ms);
        std::cout << "JSON report written to: " << json_file << "\n\n";
    }

    return gap_count > 0 ? 1 : 0;
}

// -------------------------------------------------------------------------
// Label formatting
// -------------------------------------------------------------------------

static std::string comboLabel(const std::vector<HealthVar>& vars, unsigned combo) {
    return ame::ContingencySearch::combinationLabel(vars, combo);
}

static const char* kindStr(ResultKind k) {
    switch (k) {
        case ResultKind::SOLVED:         return "SAFE";
        case ResultKind::UNSOLVABLE:     return "GAP";
        case ResultKind::IMPLIED_SAFE:   return "SAFE(implied)";
        case ResultKind::IMPLIED_UNSAFE: return "GAP(implied)";
    }
    return "?";
}

// -------------------------------------------------------------------------
// JSON output
// -------------------------------------------------------------------------

static std::string escapeJson(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '"') out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else out += c;
    }
    return out;
}

static void writeJson(const std::string& path,
                      const std::string& domain_file,
                      const std::string& problem_file,
                      const std::vector<HealthVar>& vars,
                      const std::vector<std::vector<std::string>>& goal_options,
                      const std::vector<ComboResult>& results,
                      double total_time_ms,
                      unsigned solved_count,
                      unsigned pruned_count) {
    std::ofstream f(path);
    if (!f.good()) {
        std::cerr << "Error: cannot write to " << path << "\n";
        return;
    }

    f << "{\n";
    f << "  \"domain\": \"" << escapeJson(domain_file) << "\",\n";
    f << "  \"problem_template\": \"" << escapeJson(problem_file) << "\",\n";
    f << "  \"total_combinations\": " << results.size() << ",\n";
    f << "  \"solver_calls\": " << (results.size() - pruned_count) << ",\n";
    f << "  \"pruned\": " << pruned_count << ",\n";
    f << "  \"total_time_ms\": " << std::fixed << std::setprecision(1)
      << total_time_ms << ",\n";

    f << "  \"safety_goals\": [\n";
    for (size_t i = 0; i < goal_options.size(); ++i) {
        f << "    \"" << escapeJson(formatGoalOption(goal_options[i])) << "\"";
        if (i + 1 < goal_options.size()) f << ",";
        f << "\n";
    }
    f << "  ],\n";

    f << "  \"health_variables\": [\n";
    for (size_t i = 0; i < vars.size(); ++i) {
        f << "    {\"name\": \"" << escapeJson(vars[i].short_name)
          << "\", \"fluent\": \"" << escapeJson(vars[i].fluent_name) << "\"}";
        if (i + 1 < vars.size()) f << ",";
        f << "\n";
    }
    f << "  ],\n";

    unsigned safe = 0, gap = 0;
    for (auto& r : results) {
        if (r.kind == ResultKind::SOLVED || r.kind == ResultKind::IMPLIED_SAFE)
            safe++;
        else
            gap++;
    }

    f << "  \"safe_count\": " << safe << ",\n";
    f << "  \"gap_count\": " << gap << ",\n";

    f << "  \"combinations\": [\n";
    for (size_t i = 0; i < results.size(); ++i) {
        auto& r = results[i];
        f << "    {\n";
        f << "      \"combo\": " << r.combo << ",\n";
        f << "      \"label\": \"" << escapeJson(comboLabel(vars, r.combo)) << "\",\n";

        // Individual flags
        f << "      \"flags\": {";
        for (size_t j = 0; j < vars.size(); ++j) {
            if (j > 0) f << ", ";
            f << "\"" << escapeJson(vars[j].short_name) << "\": "
              << (((r.combo >> j) & 1) ? "true" : "false");
        }
        f << "},\n";

        f << "      \"result\": \"" << kindStr(r.kind) << "\",\n";
        f << "      \"plan_length\": " << r.plan_length << ",\n";
        f << "      \"solve_time_ms\": " << std::fixed << std::setprecision(2)
          << r.solve_time_ms << ",\n";
        f << "      \"selected_goal\": \"" << escapeJson(r.selected_goal) << "\",\n";
        f << "      \"initial_state\": \"" << escapeJson(r.initial_state) << "\",\n";
        f << "      \"end_state\": \"" << escapeJson(r.end_state) << "\",\n";
        f << "      \"actions\": \"" << escapeJson(r.actions) << "\",\n";
        f << "      \"plan\": \"" << escapeJson(r.full_plan) << "\"\n";
        f << "    }";
        if (i + 1 < results.size()) f << ",";
        f << "\n";
    }
    f << "  ]\n";
    f << "}\n";
}

// -------------------------------------------------------------------------
// Main
// -------------------------------------------------------------------------

static void printUsage(const char* prog) {
    std::cerr
        << "Usage: " << prog << " <domain.pddl> <template_problem.pddl> [options]\n"
        << "\n"
        << "Exhaustive safe-state reachability analysis for PDDL contingency domains.\n"
        << "Automatically identifies context predicates (system health, environment\n"
        << "state) and enumerates all 2^N combinations, proving whether every possible\n"
        << "world state has a valid plan to reach the goal.\n"
        << "\n"
        << "Context predicates are externally-determined facts (hardware health, sensor\n"
        << "status, comms, weather) that change due to real-world events. They are\n"
        << "always CONFIRMED by perception, never produced by plan actions.\n"
        << "\n"
        << "Options:\n"
        << "  --no-prune     Disable monotonicity pruning (solve every combination)\n"
        << "  --json <file>  Write machine-readable report to JSON file\n"
        << "  --expansion-sidecar <file>\n"
        << "                 Run reachable-state expansion with sidecar-defined\n"
        << "                 finite-domain variables and invalid-state invariants.\n"
        << "  --safety-goal <expr>\n"
        << "                 Add an acceptable safety goal. Repeat for alternatives.\n"
        << "                 Expr may be '(fact)', '(fact1) (fact2)', or '(and ...)'.\n"
        << "  --safety-invariant <expr>\n"
        << "                 Alias for --safety-goal.\n"
        << "  --verbose      Show full plan signatures in the report\n"
        << "  -h, --help     Show this help\n"
        << "\n"
        << "Monotonicity pruning (default ON):\n"
        << "  Context predicates cannot be changed by any action. If a plan exists\n"
        << "  with fewer capabilities, it also works with more. This can skip\n"
        << "  exponentially many solver calls.\n";
}

int main(int argc, char* argv[]) {
    // --- Parse arguments ---
    if (argc < 3) {
        if (argc == 2) {
            const std::string arg = argv[1];
            if (arg == "--help" || arg == "-h") {
                printUsage(argv[0]);
                return 0;
            }
        }
        printUsage(argv[0]);
        return 1;
    }

    std::string domain_file, problem_file, json_file, expansion_sidecar_file;
    std::vector<std::string> safety_goal_args;
    bool prune = true;
    bool verbose = false;

    // First two positional args are domain and problem
    int positional = 0;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--no-prune") {
            prune = false;
        } else if (arg == "--json" && i + 1 < argc) {
            json_file = argv[++i];
        } else if (arg == "--expansion-sidecar" && i + 1 < argc) {
            expansion_sidecar_file = argv[++i];
        } else if ((arg == "--safety-goal" ||
                    arg == "--safety-invariant") &&
                   i + 1 < argc) {
            safety_goal_args.push_back(argv[++i]);
        } else if (arg == "--verbose" || arg == "-v") {
            verbose = true;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (arg[0] != '-') {
            if (positional == 0) domain_file = arg;
            else if (positional == 1) problem_file = arg;
            positional++;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    if (domain_file.empty() || problem_file.empty()) {
        std::cerr << "Error: both domain and problem files are required\n";
        printUsage(argv[0]);
        return 1;
    }

    // --- Read domain and problem files ---
    auto readFile = [](const std::string& path) -> std::string {
        std::ifstream f(path);
        if (!f.good()) {
            std::cerr << "Error: cannot open " << path << "\n";
            return "";
        }
        std::stringstream ss;
        ss << f.rdbuf();
        return ss.str();
    };

    std::string domain_pddl = readFile(domain_file);
    std::string problem_pddl = readFile(problem_file);
    if (domain_pddl.empty() || problem_pddl.empty()) return 1;

    // --- Parse template to identify health variables ---
    ame::WorldModel template_wm;
    try {
        ame::PddlParser::parseFromString(domain_pddl, problem_pddl, template_wm);
    } catch (const std::exception& e) {
        std::cerr << "Error parsing PDDL: " << e.what() << "\n";
        return 1;
    }

    auto vars = ame::ContingencySearch::identifyContextFacts(template_wm);
    if (vars.empty()) {
        std::cerr << "No context predicates found in domain.\n"
                  << "Context predicates are grounded fluents that appear in\n"
                  << "action preconditions and never in any action's effects.\n"
                  << "These represent externally-determined facts (health,\n"
                  << "sensor status, comms, weather) that the planner cannot\n"
                  << "change.\n";
        return 1;
    }

    // --- Negative-precondition guard ---
    // Monotone dominance pruning is only sound when context predicates appear
    // exclusively as POSITIVE preconditions: adding more context facts can only
    // enable more actions, never disable them. A negative precondition breaks
    // that monotonicity (adding a fact can disable an action), so pruning would
    // be unsound. Disable it automatically with an explicit message.
    const bool domain_has_neg_preconditions =
        !ame::ContingencySearch::pruningIsSound(template_wm);
    if (domain_has_neg_preconditions && prune) {
        prune = false;
        std::cout << "\nNOTE: domain uses negative preconditions; monotone\n"
                  << "dominance pruning is UNSOUND for such domains and has been\n"
                  << "DISABLED automatically. Every combination will be solved\n"
                  << "directly. (Pass --no-prune to silence this notice.)\n";
    }

    // --- Extract safety goal alternatives ---
    auto& goal_ids = template_wm.goalFluentIds();
    std::vector<std::string> goal_keys;
    for (auto id : goal_ids) {
        goal_keys.push_back(template_wm.fluentName(id));
    }
    std::vector<std::vector<std::string>> goal_options;
    try {
        if (safety_goal_args.empty()) {
            goal_options.push_back(goal_keys);
        } else {
            for (const auto& goal_arg : safety_goal_args) {
                goal_options.push_back(extractFactExpressions(goal_arg));
            }
        }
        for (const auto& option : goal_options) {
            validateGoalOption(template_wm, option);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error parsing safety goals: " << e.what() << "\n";
        return 1;
    }

    if (!expansion_sidecar_file.empty()) {
        return runReachableExpansion(domain_file, problem_file,
                                     expansion_sidecar_file, json_file,
                                     domain_pddl, problem_pddl, template_wm,
                                     goal_options, verbose);
    }

    // --- Report header ---
    unsigned N = static_cast<unsigned>(vars.size());
    unsigned num_combos = 1u << N;

    std::cout << "\n"
              << "============================================================\n"
              << "CONTINGENCY VERIFIER — SAFE-STATE REACHABILITY ANALYSIS\n"
              << "============================================================\n"
              << "\n"
              << "Domain:    " << domain_file << "\n"
              << "Template:  " << problem_file << "\n"
              << "Pruning:   " << (prune ? "monotone dominance" : "disabled") << "\n"
              << "\n"
              << "Identified " << N << " context predicate(s):\n";

    for (auto& v : vars) {
        std::cout << "  " << std::left << std::setw(4)
                  << (v.initial_value ? "[ON]" : "[--]")
                  << " " << v.fluent_name << "\n";
    }

    std::cout << "\nSafety goals:\n";
    for (size_t i = 0; i < goal_options.size(); ++i) {
        std::cout << "  [" << (i + 1) << "] "
                  << formatGoalOption(goal_options[i]) << "\n";
    }
    std::cout << "\n"
              << "Total combinations: " << num_combos << "\n";

    if (prune) {
        std::cout << "\n"
            << "PRUNING JUSTIFICATION (Monotone Dominance)\n"
            << "------------------------------------------------------------\n"
            << "The identified context predicates represent externally-\n"
            << "determined facts (hardware health, sensor status, comms,\n"
            << "weather) that change due to real-world events and are always\n"
            << "CONFIRMED by perception. They are not producible or\n"
            << "consumable by any plan action — no action adds or deletes\n"
            << "them. The planner treats each combination as a fixed context\n"
            << "snapshot and plans within it.\n"
            << "\n"
            << "Within any single planning snapshot, these predicates are\n"
            << "fixed and appear only as POSITIVE preconditions (the STRIPS\n"
            << "parser does not support negated preconditions).\n"
            << "\n"
            << "Theorem: if a goal is reachable from context S, it is also\n"
            << "reachable from any context S' where S' has all facts of S\n"
            << "plus additional context facts.\n"
            << "\n"
            << "Proof: any plan P valid in S is also valid in S'. Each\n"
            << "action's preconditions are a conjunction of positive atoms.\n"
            << "Since S ⊆ S', every precondition satisfied in S is also\n"
            << "satisfied in S'. Effects are identical (context predicates\n"
            << "are not in effects), so the plan produces the same state\n"
            << "transitions and achieves the same goal. QED.\n"
            << "\n"
            << "Corollary (safe propagation): if context S is SAFE, all\n"
            << "supersets of S are also SAFE.\n"
            << "Corollary (gap propagation): if context S is a GAP, all\n"
            << "subsets of S are also GAPS.\n"
            << "\n"
            << "Validity conditions (verified for each context predicate):\n"
            << "  1. Not in any action's add effects  (planner cannot set)\n"
            << "  2. Not in any action's delete effects  (planner cannot clear)\n"
            << "  3. Appears only as positive precondition  (STRIPS guarantee)\n"
            << "\n";
        for (auto& v : vars) {
            std::cout << "  " << v.fluent_name
                      << " — conditions 1-3: VERIFIED\n";
        }
        std::cout << "------------------------------------------------------------\n";
    }

    std::cout << "\n";

    // --- Enumerate combinations, carrying conclusions where that is sound ---
    // The search itself is ame_core's, so that this tool, the authoring tool
    // and the generated assurance report all answer this question the same way.
    ame::ContingencySearchOptions search_options;
    search_options.prune = prune;
    search_options.goal_options = goal_options;

    ame::ContingencySearchReport search;
    try {
        search = ame::ContingencySearch::run(domain_pddl, problem_pddl,
                                             search_options);
    } catch (const std::exception& e) {
        std::cerr << "Error during contingency search: " << e.what() << "\n";
        return 1;
    }

    const double total_ms = search.wall_time_ms;
    const unsigned solver_calls = search.solver_calls;
    const unsigned pruned_safe = search.implied_reachable;
    const unsigned pruned_unsafe = search.implied_unreachable;

    // Present the search's answer in this tool's own vocabulary.
    std::vector<ComboResult> results(num_combos);
    for (unsigned combo = 0; combo < num_combos && combo < search.cases.size();
         ++combo) {
        const ame::ContingencyCase& c = search.cases[combo];
        ComboResult& r = results[combo];
        r.combo = c.combination;
        r.plan_length = c.plan_length;
        r.solve_time_ms = c.solve_time_ms;
        r.selected_goal = c.selected_goal;
        r.initial_state = c.initial_state;
        r.end_state = c.end_state;
        r.actions = c.action_names;
        r.full_plan = c.full_plan;
        switch (c.outcome) {
            case ame::ContingencyOutcome::Reachable:
                r.kind = ResultKind::SOLVED;
                break;
            case ame::ContingencyOutcome::Unreachable:
                r.kind = ResultKind::UNSOLVABLE;
                break;
            case ame::ContingencyOutcome::ImpliedReachable:
                r.kind = ResultKind::IMPLIED_SAFE;
                break;
            case ame::ContingencyOutcome::ImpliedUnreachable:
                r.kind = ResultKind::IMPLIED_UNSAFE;
                break;
        }
        if (c.has_implied_by) {
            r.actions = "(implied by " + comboLabel(vars, c.implied_by) + ")";
        }
    }

    // --- Print results table ---
    // Determine column width from longest label
    size_t label_width = 10;
    for (unsigned i = 0; i < num_combos; ++i) {
        label_width = std::max(label_width, comboLabel(vars, i).size());
    }
    label_width += 2;

    std::cout << std::left
              << std::setw(static_cast<int>(label_width)) << "Configuration"
              << std::setw(16) << "Result"
              << std::setw(6) << "Steps"
              << std::setw(10) << "Time(ms)"
              << "Recovery Actions\n";
    std::cout << std::string(label_width + 16 + 6 + 10 + 40, '-') << "\n";

    unsigned safe_count = 0, gap_count = 0;
    std::vector<std::string> gaps;

    for (unsigned combo = 0; combo < num_combos; ++combo) {
        auto& r = results[combo];
        std::string label = comboLabel(vars, combo);
        bool is_safe = (r.kind == ResultKind::SOLVED ||
                        r.kind == ResultKind::IMPLIED_SAFE);

        std::cout << std::left
                  << std::setw(static_cast<int>(label_width)) << label
                  << std::setw(16) << kindStr(r.kind)
                  << std::setw(6)
                  << (is_safe ? std::to_string(r.plan_length) : "-")
                  << std::setw(10) << std::fixed << std::setprecision(1)
                  << r.solve_time_ms;

        if (verbose && !r.full_plan.empty()) {
            std::cout << r.full_plan;
        } else {
            std::cout << r.actions;
        }
        std::cout << "\n";

        if (is_safe)
            safe_count++;
        else {
            gap_count++;
            gaps.push_back(label);
        }
    }

    std::cout << "\n"
              << "============================================================\n"
              << "REVIEW DETAILS\n"
              << "============================================================\n";
    for (unsigned combo = 0; combo < num_combos; ++combo) {
        const auto& r = results[combo];
        const bool is_safe = (r.kind == ResultKind::SOLVED ||
                              r.kind == ResultKind::IMPLIED_SAFE);
        std::cout << "\n[" << kindStr(r.kind) << "] "
                  << comboLabel(vars, combo) << "\n";
        if (!r.initial_state.empty()) {
            std::cout << "  Start true fluents: " << r.initial_state << "\n";
        }
        if (is_safe) {
            if (!r.selected_goal.empty()) {
                std::cout << "  Safety goal reached: " << r.selected_goal << "\n";
            }
            if (!r.full_plan.empty()) {
                std::cout << "  Plan: " << r.full_plan << "\n";
            } else {
                std::cout << "  Plan: " << r.actions << "\n";
            }
            if (!r.end_state.empty()) {
                std::cout << "  End true fluents: " << r.end_state << "\n";
            }
        } else {
            std::cout << "  Gap: no plan reaches any configured safety goal.\n";
            if (!r.end_state.empty()) {
                std::cout << "  End state: " << r.end_state << "\n";
            }
        }
    }

    // --- Summary ---
    std::cout << "\n"
              << "============================================================\n"
              << "SUMMARY\n"
              << "============================================================\n"
              << "  Total combinations:  " << num_combos << "\n"
              << "  Solver calls:        " << solver_calls
              << " (of " << num_combos << ")\n"
              << "  Pruned (safe):       " << pruned_safe << "\n"
              << "  Pruned (unsafe):     " << pruned_unsafe << "\n"
              << "  Wall-clock time:     " << std::fixed << std::setprecision(1)
              << total_ms << " ms\n"
              << "\n"
              << "  SAFE:  " << safe_count << "/" << num_combos << "\n"
              << "  GAPS:  " << gap_count << "/" << num_combos << "\n";

    if (!gaps.empty()) {
        std::cout << "\n  DESIGN GAPS (no path to goal):\n";
        for (auto& g : gaps) {
            std::cout << "    * " << g << "\n";
        }
    }

    if (gap_count == 0) {
        std::cout << "\n  RESULT: ALL STATES REACH SAFE-STATE\n";
    }

    std::cout << "============================================================\n\n";

    // --- JSON output ---
    if (!json_file.empty()) {
        writeJson(json_file, domain_file, problem_file, vars, goal_options, results,
                  total_ms, solver_calls, pruned_safe + pruned_unsafe);
        std::cout << "JSON report written to: " << json_file << "\n\n";
    }

    return gap_count > 0 ? 1 : 0;
}
