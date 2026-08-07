#include "ame/plan_compiler.h"
#include "ame/detail/escape.h"

#include <algorithm>
#include <cctype>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace ame {

namespace {

std::string joinFacts(const std::vector<unsigned>& fact_ids,
                      const WorldModel& wm) {
    std::ostringstream encoded;
    for (size_t i = 0; i < fact_ids.size(); ++i) {
        if (i != 0) {
            encoded << ';';
        }
        encoded << wm.fluentName(fact_ids[i]);
    }
    return encoded.str();
}

std::string joinSelectedFacts(const std::vector<unsigned>& fact_ids,
                              const WorldModel& wm,
                              bool want_confirmed) {
    std::ostringstream encoded;
    bool first = true;
    for (const auto id : fact_ids) {
        const std::string& name = wm.fluentName(id);
        if (wm.isConfirmedFact(name) != want_confirmed) {
            continue;
        }
        if (!first) {
            encoded << ';';
        }
        encoded << name;
        first = false;
    }
    return encoded.str();
}

void emitContractAttributes(std::ostringstream& xml,
                            const GroundAction& action,
                            const WorldModel& wm,
                            bool reactive) {
    // Preconditions split by what the domain says each predicate's evidence has
    // to be. Anything on a (:confirmed-predicates ...) predicate goes to the
    // confirmed list, which the action node satisfies only from observed state.
    xml << " ame_preconditions=\""
        << detail::xmlAttrEscape(
               joinSelectedFacts(action.preconditions, wm, false)) << "\""
        << " ame_confirmed_preconditions=\""
        << detail::xmlAttrEscape(
               joinSelectedFacts(action.preconditions, wm, true)) << "\""
        << " ame_neg_preconditions=\""
        << detail::xmlAttrEscape(joinFacts(action.neg_preconditions, wm)) << "\""
        << " ame_add_effects=\""
        << detail::xmlAttrEscape(joinFacts(action.add_effects, wm)) << "\""
        << " ame_del_effects=\""
        << detail::xmlAttrEscape(joinFacts(action.del_effects, wm)) << "\""
        << " ame_reactive=\"" << (reactive ? "true" : "false") << "\"";
}

}  // namespace

// =========================================================================
// Helpers
// =========================================================================

std::string PlanCompiler::actionName(const std::string& signature) {
    auto paren = signature.find('(');
    std::string name =
        (paren == std::string::npos) ? signature : signature.substr(0, paren);
    // Strip a disjunct tag ("deliver#0" -> "deliver"). Disjunctive-precondition
    // actions register one schema per DNF disjunct under a "#k"-suffixed name;
    // all disjuncts resolve to the same registered BT implementation.
    auto hash = name.rfind('#');
    if (hash != std::string::npos && hash + 1 < name.size()) {
        bool all_digits = true;
        for (size_t i = hash + 1; i < name.size(); ++i) {
            if (!std::isdigit(static_cast<unsigned char>(name[i]))) {
                all_digits = false;
                break;
            }
        }
        if (all_digits) name = name.substr(0, hash);
    }
    return name;
}

std::vector<std::string> PlanCompiler::actionParams(const std::string& signature) {
    std::vector<std::string> params;
    auto open = signature.find('(');
    auto close = signature.rfind(')');
    if (open == std::string::npos || close == std::string::npos || close <= open + 1) {
        return params;
    }
    std::string args = signature.substr(open + 1, close - open - 1);
    std::istringstream ss(args);
    std::string token;
    while (std::getline(ss, token, ',')) {
        params.push_back(token);
    }
    return params;
}

// =========================================================================
// Action Unit Generation
// =========================================================================

std::string PlanCompiler::emitActionUnit(const GroundAction& ga,
                                         const WorldModel& wm,
                                         const ActionRegistry& registry) const {
    std::string name = actionName(ga.signature);
    auto params = actionParams(ga.signature);
    std::ostringstream xml;

    // Production execution fails closed on an action with no BT binding. The
    // authoring/devenv preview enables stub mode instead, where the action
    // compiles to a SimulatedAction that still carries the state contract, so
    // the preview shows the plan's real precondition and effect structure.
    if (!registry.hasAction(name)) {
        if (!stub_unregistered_) {
            throw std::runtime_error("Unregistered action in plan: " + name);
        }
        xml << "<SimulatedAction name=\""
            << detail::xmlAttrEscape(ga.signature) << "\"";
        emitContractAttributes(xml, ga, wm, false);
        xml << "/>";
        return xml.str();
    }

    auto impl = registry.resolve(name, params);
    if (impl.is_subtree) {
        xml << "<PlannedAction name=\""
            << detail::xmlAttrEscape(ga.signature) << "\"";
        emitContractAttributes(xml, ga, wm, impl.reactive);
        xml << ">\n  " << impl.xml << "\n</PlannedAction>";
        return xml.str();
    }

    xml << '<' << impl.node_type << " name=\""
        << detail::xmlAttrEscape(ga.signature) << "\"";
    for (size_t i = 0; i < impl.param_bindings.size(); ++i) {
        xml << " param" << i << "=\""
            << detail::xmlAttrEscape(impl.param_bindings[i]) << "\"";
    }
    emitContractAttributes(xml, ga, wm, impl.reactive);
    xml << "/>";

    return xml.str();
}

// =========================================================================
// Causal Graph Analysis
// =========================================================================

struct CausalGraph {
    unsigned num_steps;
    // adj[i] = set of step indices that depend on step i
    std::vector<std::set<unsigned>> adj;
    // in_degree[i] = number of steps that must precede step i
    std::vector<unsigned> in_degree;

    CausalGraph(unsigned n) : num_steps(n), adj(n), in_degree(n, 0) {}

    void addEdge(unsigned from, unsigned to) {
        if (adj[from].insert(to).second) {
            ++in_degree[to];
        }
    }
};

// True if the two sorted-or-unsorted fluent-id lists share any element.
static bool intersects(const std::vector<unsigned>& a,
                       const std::vector<unsigned>& b) {
    for (auto x : a) {
        for (auto y : b) {
            if (x == y) return true;
        }
    }
    return false;
}

static CausalGraph buildCausalGraph(const std::vector<PlanStep>& plan,
                                     const WorldModel& wm) {
    CausalGraph cg(static_cast<unsigned>(plan.size()));

    for (unsigned i = 0; i < plan.size(); ++i) {
        auto& ga_i = wm.groundActions()[plan[i].action_index];

        for (unsigned j = i + 1; j < plan.size(); ++j) {
            auto& ga_j = wm.groundActions()[plan[j].action_index];

            // A dependency (causal link or threat) between earlier step i and
            // later step j forces them into the same sequential flow, preserving
            // the planner-validated order. Only fully independent steps end up in
            // parallel flows.
            //
            // Positive-only interactions (original behaviour):
            //   add(i) ∩ pre(j)   support: i establishes a precondition of j
            //   del(i) ∩ pre(j)   threat:  i deletes a positive precondition of j
            //   del(j) ∩ add(i)   interference: j undoes an add-effect of i
            //
            // Signed-precondition interactions (negative preconditions):
            //   add(i) ∩ negpre(j)  threat:  i makes p true, j needs p false
            //   del(i) ∩ negpre(j)  support: i makes p false, j needs p false
            //   negpre(i) ∩ add(j)  a later add(p) conflicts with i needing not-p
            //   pre(i)   ∩ del(j)   a later del(p) conflicts with i needing p
            const bool has_dependency =
                intersects(ga_i.add_effects, ga_j.preconditions) ||
                intersects(ga_i.del_effects, ga_j.preconditions) ||
                intersects(ga_j.del_effects, ga_i.add_effects) ||
                intersects(ga_i.add_effects, ga_j.neg_preconditions) ||
                intersects(ga_i.del_effects, ga_j.neg_preconditions) ||
                intersects(ga_i.neg_preconditions, ga_j.add_effects) ||
                intersects(ga_i.preconditions, ga_j.del_effects);

            if (has_dependency) {
                cg.addEdge(i, j);
            }
        }
    }

    return cg;
}

// Extract conservative execution phases from the dependency graph.
//
// Each phase contains every currently-ready step, ordered by original plan index.
// The BT emitter places a barrier between phases, so later steps only start after
// all actions in the previous ready set have finished.
static std::vector<std::vector<unsigned>> extractExecutionPhases(const CausalGraph& cg) {
    std::vector<std::vector<unsigned>> phases;
    std::vector<unsigned> remaining_in(cg.in_degree);
    std::vector<bool> scheduled(cg.num_steps, false);
    unsigned scheduled_count = 0;

    while (scheduled_count < cg.num_steps) {
        std::vector<unsigned> phase;

        for (unsigned i = 0; i < cg.num_steps; ++i) {
            if (!scheduled[i] && remaining_in[i] == 0) {
                phase.push_back(i);
            }
        }

        if (phase.empty()) {
            throw std::runtime_error("Causal graph contains a cycle");
        }

        for (auto step : phase) {
            scheduled[step] = true;
            ++scheduled_count;

            for (auto next : cg.adj[step]) {
                --remaining_in[next];
            }
        }

        phases.push_back(std::move(phase));
    }

    return phases;
}

// =========================================================================
// Compilation
// =========================================================================

void PlanCompiler::emitGoalGuardOpen(std::ostringstream& xml,
                                     const WorldModel& wm,
                                     const std::string& indent) const {
    emitGoalGuardOpen(xml, wm, wm.goalFluentIds(), indent);
}

void PlanCompiler::emitGoalGuardOpen(std::ostringstream& xml,
                                     const WorldModel& wm,
                                     const std::vector<unsigned>& goal_ids,
                                     const std::string& indent) const {
    if (goal_ids.empty()) return;

    // Disjunctive goals: when the caller is using the world model's default goal
    // (goal_ids == the primary alternative) and there is more than one
    // alternative, the guard succeeds if ANY alternative holds. Explicit-goal
    // callers (a custom goal set) keep the single-conjunction guard.
    const auto& alternatives = wm.goalAlternatives();
    const bool multi = alternatives.size() > 1 && goal_ids == wm.goalFluentIds();

    // Helper: emit a goal-check node for one alternative's fluent ids. One
    // GoalReached carries the whole conjunction, so an alternative is a single
    // node whatever its arity.
    auto emitCheck = [&](const std::vector<unsigned>& ids, const std::string& ind,
                         const std::string& check_name) {
        xml << ind << "<GoalReached name=\"" << check_name << "\" goals=\""
            << detail::xmlAttrEscape(joinFacts(ids, wm)) << "\"/>\n";
    };

    // ReactiveFallback: if the goal-check child succeeds (goal met), the
    // fallback returns SUCCESS without re-running the plan.
    xml << indent << "<ReactiveFallback>\n";

    if (multi) {
        // Goal met if any alternative's conjunction holds.
        xml << indent << "  <Fallback name=\"GoalAltCheck\">\n";
        for (size_t a = 0; a < alternatives.size(); ++a) {
            emitCheck(alternatives[a], indent + "    ", "GoalAlt" + std::to_string(a));
        }
        xml << indent << "  </Fallback>\n";
    } else {
        emitCheck(goal_ids, indent + "  ", "GoalReached");
    }
}

void PlanCompiler::emitGoalGuardClose(std::ostringstream& xml,
                                      const WorldModel& wm,
                                      const std::string& indent) const {
    emitGoalGuardClose(xml, wm.goalFluentIds(), indent);
}

void PlanCompiler::emitGoalGuardClose(std::ostringstream& xml,
                                      const std::vector<unsigned>& goal_ids,
                                      const std::string& indent) const {
    if (goal_ids.empty()) return;
    xml << indent << "</ReactiveFallback>\n";
}

std::string PlanCompiler::compileSequential(const std::vector<PlanStep>& plan,
                                            const WorldModel& wm,
                                            const ActionRegistry& registry) const {
    return compileSequential(plan, wm, registry, wm.goalFluentIds());
}

std::string PlanCompiler::compileSequential(const std::vector<PlanStep>& plan,
                                            const WorldModel& wm,
                                            const ActionRegistry& registry,
                                            const std::vector<unsigned>& goal_ids) const {
    return compileSequentialWithMetadata(plan, wm, registry, goal_ids).xml;
}

CompiledPlan PlanCompiler::compileSequentialWithMetadata(const std::vector<PlanStep>& plan,
                                                         const WorldModel& wm,
                                                         const ActionRegistry& registry,
                                                         const std::vector<unsigned>& goal_ids) const {
    CompiledPlan compiled;
    compiled.has_goal_guard = !goal_ids.empty();
    compiled.steps.reserve(plan.size());

    for (const auto& step : plan) {
        const auto& ga = wm.groundActions()[step.action_index];
        const std::string name = actionName(ga.signature);
        if (!registry.hasAction(name) && !stub_unregistered_) {
            throw std::runtime_error("Unregistered action in plan: " + name);
        }
    }

    std::ostringstream xml;

    xml << "<root BTCPP_format=\"4\">\n";
    xml << "  <BehaviorTree ID=\"MainTree\">\n";

    emitGoalGuardOpen(xml, wm, goal_ids, "    ");

    xml << "    <Sequence>\n";

    for (unsigned step_idx = 0; step_idx < plan.size(); ++step_idx) {
        const auto& step = plan[step_idx];
        auto& ga = wm.groundActions()[step.action_index];
        compiled.steps.push_back({step_idx, ga.signature});
        std::string unit = emitActionUnit(ga, wm, registry);
        // Indent each line
        std::istringstream lines(unit);
        std::string line;
        while (std::getline(lines, line)) {
            xml << "      " << line << "\n";
        }
    }

    xml << "    </Sequence>\n";

    emitGoalGuardClose(xml, goal_ids, "    ");

    xml << "  </BehaviorTree>\n";
    xml << "</root>\n";

    compiled.xml = xml.str();
    return compiled;
}

std::string PlanCompiler::compile(const std::vector<PlanStep>& plan,
                                  const WorldModel& wm,
                                  const ActionRegistry& registry) const {
    return compile(plan, wm, registry, wm.goalFluentIds());
}

std::string PlanCompiler::compile(const std::vector<PlanStep>& plan,
                                  const WorldModel& wm,
                                  const ActionRegistry& registry,
                                  const std::vector<unsigned>& goal_ids) const {
    return compileWithMetadata(plan, wm, registry, goal_ids).xml;
}

CompiledPlan PlanCompiler::compileWithMetadata(const std::vector<PlanStep>& plan,
                                               const WorldModel& wm,
                                               const ActionRegistry& registry,
                                               const std::vector<unsigned>& goal_ids) const {
    if (plan.empty()) {
        CompiledPlan compiled;
        compiled.xml = "<root BTCPP_format=\"4\">\n"
                       "  <BehaviorTree ID=\"MainTree\">\n"
                       "    <Sequence/>\n"
                       "  </BehaviorTree>\n"
                       "</root>\n";
        return compiled;
    }

    for (const auto& step : plan) {
        const auto& ga = wm.groundActions()[step.action_index];
        const std::string name = actionName(ga.signature);
        if (!registry.hasAction(name) && !stub_unregistered_) {
            throw std::runtime_error("Unregistered action in plan: " + name);
        }
    }

    if (plan.size() == 1) {
        return compileSequentialWithMetadata(plan, wm, registry, goal_ids);
    }

    auto cg = buildCausalGraph(plan, wm);
    auto phases = extractExecutionPhases(cg);

    const bool has_parallel_phase =
        std::any_of(phases.begin(), phases.end(),
                    [](const std::vector<unsigned>& phase) { return phase.size() > 1; });

    if (!has_parallel_phase) {
        // No independent ready set -- sequential
        return compileSequentialWithMetadata(plan, wm, registry, goal_ids);
    }

    // Emit a conservative sequence of execution phases. Multi-step phases run in
    // parallel; phase boundaries are explicit synchronization barriers.
    CompiledPlan compiled;
    compiled.has_goal_guard = !goal_ids.empty();
    compiled.steps.reserve(plan.size());
    for (unsigned step_idx = 0; step_idx < plan.size(); ++step_idx) {
        auto& ga = wm.groundActions()[plan[step_idx].action_index];
        compiled.steps.push_back({step_idx, ga.signature});
    }

    std::ostringstream xml;

    xml << "<root BTCPP_format=\"4\">\n";
    xml << "  <BehaviorTree ID=\"MainTree\">\n";

    emitGoalGuardOpen(xml, wm, goal_ids, "    ");

    xml << "    <Sequence>\n";

    for (auto& phase : phases) {
        if (phase.size() == 1) {
            auto& ga = wm.groundActions()[plan[phase[0]].action_index];
            std::string unit = emitActionUnit(ga, wm, registry);
            std::istringstream lines(unit);
            std::string line;
            while (std::getline(lines, line)) {
                xml << "      " << line << "\n";
            }
        } else {
            xml << "      <Parallel success_count=\"" << phase.size()
                << "\" failure_count=\"1\">\n";
            for (auto step_idx : phase) {
                auto& ga = wm.groundActions()[plan[step_idx].action_index];
                std::string unit = emitActionUnit(ga, wm, registry);
                std::istringstream lines(unit);
                std::string line;
                while (std::getline(lines, line)) {
                    xml << "        " << line << "\n";
                }
            }
            xml << "      </Parallel>\n";
        }
    }

    xml << "    </Sequence>\n";

    emitGoalGuardClose(xml, goal_ids, "    ");

    xml << "  </BehaviorTree>\n";
    xml << "</root>\n";

    compiled.xml = xml.str();
    return compiled;
}

std::string PlanCompiler::compile(const std::vector<PlanStep>& plan,
                                  const WorldModel& wm,
                                  const ActionRegistry& registry,
                                  const std::string& agent_id) const {
    return compile(plan, wm, registry, agent_id, wm.goalFluentIds());
}

std::string PlanCompiler::compile(const std::vector<PlanStep>& plan,
                                  const WorldModel& wm,
                                  const ActionRegistry& registry,
                                  const std::string& agent_id,
                                  const std::vector<unsigned>& goal_ids) const {
    if (plan.empty()) {
        return "<root BTCPP_format=\"4\">\n"
               "  <BehaviorTree ID=\"MainTree\">\n"
               "    <Sequence/>\n"
               "  </BehaviorTree>\n"
               "</root>\n";
    }

    // For agent-scoped plans, we wrap the tree in a SetBlackboard node
    // to inject the agent context, then use the standard compilation.
    std::string inner_xml = compile(plan, wm, registry, goal_ids);

    // Extract the content between <BehaviorTree> tags
    auto bt_start = inner_xml.find("<BehaviorTree");
    auto bt_end = inner_xml.rfind("</BehaviorTree>");
    if (bt_start == std::string::npos || bt_end == std::string::npos) {
        return inner_xml;  // Fallback if parsing fails
    }

    // Find the end of the opening BehaviorTree tag
    auto bt_open_end = inner_xml.find('>', bt_start);
    if (bt_open_end == std::string::npos) {
        return inner_xml;
    }

    // Extract the inner content (the actual tree nodes)
    std::string bt_opening = inner_xml.substr(bt_start, bt_open_end - bt_start + 1);
    std::string bt_content = inner_xml.substr(bt_open_end + 1, bt_end - bt_open_end - 1);

    // Rebuild with agent context wrapper
    std::ostringstream xml;
    xml << "<root BTCPP_format=\"4\">\n";
    xml << "  " << bt_opening << "\n";
    xml << "    <Sequence>\n";
    xml << "      <SetBlackboard output_key=\"executing_agent\" value=\""
        << detail::xmlAttrEscape(agent_id) << "\"/>\n";
    // Re-indent the inner content
    std::istringstream content_stream(bt_content);
    std::string line;
    while (std::getline(content_stream, line)) {
        if (!line.empty() && line.find_first_not_of(" \t\n") != std::string::npos) {
            xml << "  " << line << "\n";
        }
    }
    xml << "    </Sequence>\n";
    xml << "  </BehaviorTree>\n";
    xml << "</root>\n";

    return xml.str();
}

} // namespace ame
