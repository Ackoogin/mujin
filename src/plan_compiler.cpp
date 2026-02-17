#include "mujin/plan_compiler.h"

#include <algorithm>
#include <functional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace mujin {

// =========================================================================
// Helpers
// =========================================================================

std::string PlanCompiler::actionName(const std::string& signature) {
    auto paren = signature.find('(');
    if (paren == std::string::npos) return signature;
    return signature.substr(0, paren);
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

    bool reactive = false;
    if (registry.hasAction(name)) {
        reactive = registry.isReactive(name);
    }

    std::string seq_type = reactive ? "ReactiveSequence" : "Sequence";
    std::ostringstream xml;

    xml << "<" << seq_type << " name=\"" << ga.signature << "\">\n";

    // Precondition checks
    for (auto pre_id : ga.preconditions) {
        xml << "    <CheckWorldPredicate predicate=\""
            << wm.fluentName(pre_id) << "\"/>\n";
    }

    // The actual action implementation (from ActionRegistry)
    if (registry.hasAction(name)) {
        auto impl = registry.resolve(name, params);
        xml << "    " << impl.xml << "\n";
    }

    // Add effects
    for (auto add_id : ga.add_effects) {
        xml << "    <SetWorldPredicate predicate=\""
            << wm.fluentName(add_id) << "\" value=\"true\"/>\n";
    }

    // Delete effects
    for (auto del_id : ga.del_effects) {
        xml << "    <SetWorldPredicate predicate=\""
            << wm.fluentName(del_id) << "\" value=\"false\"/>\n";
    }

    xml << "</" << seq_type << ">";

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

static CausalGraph buildCausalGraph(const std::vector<PlanStep>& plan,
                                     const WorldModel& wm) {
    CausalGraph cg(static_cast<unsigned>(plan.size()));

    for (unsigned i = 0; i < plan.size(); ++i) {
        auto& ga_i = wm.groundActions()[plan[i].action_index];

        for (unsigned j = i + 1; j < plan.size(); ++j) {
            auto& ga_j = wm.groundActions()[plan[j].action_index];

            bool has_dependency = false;

            // Check if any add-effect of step i is a precondition of step j
            for (auto add_id : ga_i.add_effects) {
                for (auto pre_id : ga_j.preconditions) {
                    if (add_id == pre_id) {
                        has_dependency = true;
                        break;
                    }
                }
                if (has_dependency) break;
            }

            // Check delete-effect conflicts: if step i deletes something step j needs
            if (!has_dependency) {
                for (auto del_id : ga_i.del_effects) {
                    for (auto pre_id : ga_j.preconditions) {
                        if (del_id == pre_id) {
                            has_dependency = true;
                            break;
                        }
                    }
                    if (has_dependency) break;
                }
            }

            // Check if step j deletes something step i adds (interference)
            if (!has_dependency) {
                for (auto del_id : ga_j.del_effects) {
                    for (auto add_id : ga_i.add_effects) {
                        if (del_id == add_id) {
                            has_dependency = true;
                            break;
                        }
                    }
                    if (has_dependency) break;
                }
            }

            if (has_dependency) {
                cg.addEdge(i, j);
            }
        }
    }

    return cg;
}

// Extract independent flows (connected components in the dependency graph)
static std::vector<std::vector<unsigned>> extractFlows(const CausalGraph& cg) {
    // Topological sort
    std::vector<unsigned> sorted;
    std::vector<unsigned> remaining_in(cg.in_degree);
    std::vector<unsigned> queue;

    for (unsigned i = 0; i < cg.num_steps; ++i) {
        if (remaining_in[i] == 0) queue.push_back(i);
    }

    while (!queue.empty()) {
        unsigned curr = queue.back();
        queue.pop_back();
        sorted.push_back(curr);
        for (auto next : cg.adj[curr]) {
            if (--remaining_in[next] == 0) {
                queue.push_back(next);
            }
        }
    }

    // Assign each step to a flow via union-find
    std::vector<unsigned> parent(cg.num_steps);
    for (unsigned i = 0; i < cg.num_steps; ++i) parent[i] = i;

    std::function<unsigned(unsigned)> uf_find = [&](unsigned x) -> unsigned {
        return parent[x] == x ? x : parent[x] = uf_find(parent[x]);
    };
    auto uf_unite = [&](unsigned a, unsigned b) {
        parent[uf_find(a)] = uf_find(b);
    };

    // Steps connected by causal edges belong to the same flow
    for (unsigned i = 0; i < cg.num_steps; ++i) {
        for (auto j : cg.adj[i]) {
            uf_unite(i, j);
        }
    }

    // Group by flow root
    std::unordered_map<unsigned, std::vector<unsigned>> groups;
    for (auto step : sorted) {
        groups[uf_find(step)].push_back(step);
    }

    std::vector<std::vector<unsigned>> flows;
    flows.reserve(groups.size());
    for (auto& [root, steps] : groups) {
        flows.push_back(std::move(steps));
    }

    return flows;
}

// =========================================================================
// Compilation
// =========================================================================

std::string PlanCompiler::compileSequential(const std::vector<PlanStep>& plan,
                                            const WorldModel& wm,
                                            const ActionRegistry& registry) const {
    std::ostringstream xml;

    xml << "<root BTCPP_format=\"4\">\n";
    xml << "  <BehaviorTree ID=\"MainTree\">\n";
    xml << "    <Sequence>\n";

    for (auto& step : plan) {
        auto& ga = wm.groundActions()[step.action_index];
        std::string unit = emitActionUnit(ga, wm, registry);
        // Indent each line
        std::istringstream lines(unit);
        std::string line;
        while (std::getline(lines, line)) {
            xml << "      " << line << "\n";
        }
    }

    xml << "    </Sequence>\n";
    xml << "  </BehaviorTree>\n";
    xml << "</root>\n";

    return xml.str();
}

std::string PlanCompiler::compile(const std::vector<PlanStep>& plan,
                                  const WorldModel& wm,
                                  const ActionRegistry& registry) const {
    if (plan.empty()) {
        return "<root BTCPP_format=\"4\">\n"
               "  <BehaviorTree ID=\"MainTree\">\n"
               "    <Sequence/>\n"
               "  </BehaviorTree>\n"
               "</root>\n";
    }

    if (plan.size() == 1) {
        return compileSequential(plan, wm, registry);
    }

    auto cg = buildCausalGraph(plan, wm);
    auto flows = extractFlows(cg);

    if (flows.size() <= 1) {
        // All steps are in one flow — sequential
        return compileSequential(plan, wm, registry);
    }

    // Multiple independent flows — emit Parallel
    std::ostringstream xml;

    xml << "<root BTCPP_format=\"4\">\n";
    xml << "  <BehaviorTree ID=\"MainTree\">\n";
    xml << "    <Parallel success_count=\"" << flows.size()
        << "\" failure_count=\"1\">\n";

    for (auto& flow : flows) {
        if (flow.size() == 1) {
            auto& ga = wm.groundActions()[plan[flow[0]].action_index];
            std::string unit = emitActionUnit(ga, wm, registry);
            std::istringstream lines(unit);
            std::string line;
            while (std::getline(lines, line)) {
                xml << "      " << line << "\n";
            }
        } else {
            xml << "      <Sequence>\n";
            for (auto step_idx : flow) {
                auto& ga = wm.groundActions()[plan[step_idx].action_index];
                std::string unit = emitActionUnit(ga, wm, registry);
                std::istringstream lines(unit);
                std::string line;
                while (std::getline(lines, line)) {
                    xml << "        " << line << "\n";
                }
            }
            xml << "      </Sequence>\n";
        }
    }

    xml << "    </Parallel>\n";
    xml << "  </BehaviorTree>\n";
    xml << "</root>\n";

    return xml.str();
}

} // namespace mujin
