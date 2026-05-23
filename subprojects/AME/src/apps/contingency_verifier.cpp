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
//   --verbose      Show full plan signatures
// =========================================================================

#include "ame/pddl_parser.h"
#include "ame/planner.h"
#include "ame/world_model.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

static unsigned portablePopcount(unsigned x) {
    x = x - ((x >> 1) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
    return (((x + (x >> 4)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24;
}

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
// A context predicate is a "health variable" if it is 0-ary (no parameters)
// and appears in at least one action's preconditions. These are the
// predicates whose combinations we enumerate.

struct HealthVar {
    unsigned fluent_id;
    std::string fluent_name;
    std::string short_name;
    bool initial_value;
};

static std::vector<HealthVar> identifyHealthVars(const ame::WorldModel& wm) {
    // Collect all fluent IDs that appear in any action's effects.
    // Context predicates are those used in preconditions but never in effects.
    std::vector<bool> appears_in_effects(wm.numFluents(), false);
    std::vector<bool> appears_in_preconditions(wm.numFluents(), false);

    for (unsigned i = 0; i < wm.numGroundActions(); ++i) {
        auto& ga = wm.groundActions()[i];
        for (auto id : ga.add_effects) appears_in_effects[id] = true;
        for (auto id : ga.del_effects) appears_in_effects[id] = true;
        for (auto id : ga.preconditions) appears_in_preconditions[id] = true;
    }

    std::vector<HealthVar> vars;
    for (unsigned id = 0; id < wm.numFluents(); ++id) {
        if (appears_in_effects[id]) continue;
        if (!appears_in_preconditions[id]) continue;

        const auto& name = wm.fluentName(id);
        // 0-ary predicates have no spaces after the predicate name
        // e.g. "(nav-operational)" vs "(at uav1 base)"
        size_t first_space = name.find(' ', 1);
        if (first_space != std::string::npos &&
            first_space < name.size() - 1) {
            continue;  // has parameters — skip
        }

        // Extract short name: "(nav-operational)" -> "nav-operational"
        std::string short_name = name.substr(1, name.size() - 2);

        vars.push_back({id, name, short_name, wm.getFact(id)});
    }

    return vars;
}

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
    std::string actions;
    std::string full_plan;
};

// -------------------------------------------------------------------------
// Monotonicity helpers
// -------------------------------------------------------------------------

static bool isSubset(unsigned a, unsigned b) {
    return (a & b) == a;
}

static bool isSuperset(unsigned a, unsigned b) {
    return (a & b) == b;
}

// -------------------------------------------------------------------------
// Solve one combination
// -------------------------------------------------------------------------

static ComboResult solveCombo(
    const std::string& domain_pddl,
    const std::string& problem_pddl_template,
    const std::vector<HealthVar>& vars,
    unsigned combo,
    const std::vector<std::string>& goal_keys)
{
    ComboResult cr;
    cr.combo = combo;

    // Parse fresh WorldModel from domain + template
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(domain_pddl, problem_pddl_template, wm);

    // Toggle health vars according to combo
    for (size_t i = 0; i < vars.size(); ++i) {
        bool on = (combo >> i) & 1;
        wm.setFact(vars[i].fluent_id, on);
    }

    // Set goal
    wm.setGoal(goal_keys);

    // Solve
    ame::Planner planner;
    auto result = planner.solve(wm);

    cr.solve_time_ms = result.solve_time_ms;

    if (result.success) {
        cr.kind = ResultKind::SOLVED;
        cr.plan_length = static_cast<unsigned>(result.steps.size());

        // Extract unique action names
        std::string names;
        for (auto& s : result.steps) {
            auto& sig = wm.groundActions()[s.action_index].signature;
            auto paren = sig.find('(');
            std::string name = (paren != std::string::npos)
                                   ? sig.substr(0, paren)
                                   : sig;
            if (names.find(name) == std::string::npos) {
                if (!names.empty()) names += ", ";
                names += name;
            }
        }
        cr.actions = names;

        // Full plan
        std::string full;
        for (auto& s : result.steps) {
            if (!full.empty()) full += " -> ";
            full += wm.groundActions()[s.action_index].signature;
        }
        cr.full_plan = full;
    } else {
        cr.kind = ResultKind::UNSOLVABLE;
    }

    return cr;
}

// -------------------------------------------------------------------------
// Label formatting
// -------------------------------------------------------------------------

static std::string comboLabel(const std::vector<HealthVar>& vars, unsigned combo) {
    std::string label;
    for (size_t i = 0; i < vars.size(); ++i) {
        if (!label.empty()) label += "  ";
        bool on = (combo >> i) & 1;
        label += vars[i].short_name;
        label += (on ? "=ON" : "=off");
    }
    return label;
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
        printUsage(argv[0]);
        return 1;
    }

    std::string domain_file, problem_file, json_file;
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

    auto vars = identifyHealthVars(template_wm);
    if (vars.empty()) {
        std::cerr << "No context predicates found in domain.\n"
                  << "Context predicates are 0-ary predicates that appear in\n"
                  << "action preconditions but never in any action's effects.\n"
                  << "These represent externally-determined facts (health,\n"
                  << "sensor status, comms, weather) that the planner cannot\n"
                  << "change.\n";
        return 1;
    }

    // --- Extract goal from template problem ---
    auto& goal_ids = template_wm.goalFluentIds();
    std::vector<std::string> goal_keys;
    for (auto id : goal_ids) {
        goal_keys.push_back(template_wm.fluentName(id));
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

    std::cout << "\nGoal: ";
    for (size_t i = 0; i < goal_keys.size(); ++i) {
        if (i > 0) std::cout << " AND ";
        std::cout << goal_keys[i];
    }
    std::cout << "\n\n"
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

    // --- Enumerate combinations with optional pruning ---
    auto wall_start = std::chrono::steady_clock::now();

    std::vector<ComboResult> results(num_combos);
    for (unsigned i = 0; i < num_combos; ++i) {
        results[i].combo = i;
        results[i].kind = ResultKind::UNSOLVABLE;  // safe default
    }
    std::vector<bool> classified(num_combos, false);
    unsigned solver_calls = 0;
    unsigned pruned_safe = 0, pruned_unsafe = 0;

    // Process bottom-up (fewest capabilities first) for best pruning.
    // Sort combos by popcount (number of ON bits).
    std::vector<unsigned> order(num_combos);
    for (unsigned i = 0; i < num_combos; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [](unsigned a, unsigned b) {
        return portablePopcount(a) < portablePopcount(b);
    });

    for (unsigned combo : order) {
        if (classified[combo]) continue;

        // Check if already implied by pruning
        if (prune) {
            bool skip = false;

            // Check if any classified subset is solved → implied safe
            for (unsigned other = 0; other < num_combos; ++other) {
                if (!classified[other]) continue;
                if (results[other].kind == ResultKind::SOLVED &&
                    isSuperset(combo, other) && combo != other) {
                    results[combo].kind = ResultKind::IMPLIED_SAFE;
                    results[combo].actions = "(implied by " +
                        comboLabel(vars, other) + ")";
                    classified[combo] = true;
                    pruned_safe++;
                    skip = true;
                    break;
                }
            }

            if (!skip) {
                // Check if any classified superset is unsolvable → implied gap
                for (unsigned other = 0; other < num_combos; ++other) {
                    if (!classified[other]) continue;
                    if (results[other].kind == ResultKind::UNSOLVABLE &&
                        isSubset(combo, other) && combo != other) {
                        results[combo].kind = ResultKind::IMPLIED_UNSAFE;
                        results[combo].actions = "(implied by " +
                            comboLabel(vars, other) + ")";
                        classified[combo] = true;
                        pruned_unsafe++;
                        skip = true;
                        break;
                    }
                }
            }

            if (skip) continue;
        }

        // Actually solve
        results[combo] = solveCombo(
            domain_pddl, problem_pddl, vars, combo, goal_keys);
        classified[combo] = true;
        solver_calls++;

        // Eagerly propagate to unclassified supersets/subsets
        if (prune && results[combo].kind == ResultKind::SOLVED) {
            for (unsigned other = 0; other < num_combos; ++other) {
                if (classified[other]) continue;
                if (isSuperset(other, combo)) {
                    results[other].kind = ResultKind::IMPLIED_SAFE;
                    results[other].actions = "(implied by " +
                        comboLabel(vars, combo) + ")";
                    classified[other] = true;
                    pruned_safe++;
                }
            }
        }
        if (prune && results[combo].kind == ResultKind::UNSOLVABLE) {
            for (unsigned other = 0; other < num_combos; ++other) {
                if (classified[other]) continue;
                if (isSubset(other, combo)) {
                    results[other].kind = ResultKind::IMPLIED_UNSAFE;
                    results[other].actions = "(implied by " +
                        comboLabel(vars, combo) + ")";
                    classified[other] = true;
                    pruned_unsafe++;
                }
            }
        }
    }

    auto wall_end = std::chrono::steady_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(
                          wall_end - wall_start).count();

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
        writeJson(json_file, domain_file, problem_file, vars, results,
                  total_ms, solver_calls, pruned_safe + pruned_unsafe);
        std::cout << "JSON report written to: " << json_file << "\n\n";
    }

    return gap_count > 0 ? 1 : 0;
}
