#pragma once

#include "ame/action_registry.h"
#include "ame/world_model.h"

#include <string>
#include <vector>

namespace ame {

// A plan step: an index into the WorldModel's ground actions
struct PlanStep {
    unsigned action_index;  // index into WorldModel::groundActions()
};

class PlanCompiler {
public:
    // Compile a plan (sequence of action indices) into BT XML.
    // Uses causal graph analysis for parallelism when possible.
    std::string compile(const std::vector<PlanStep>& plan,
                        const WorldModel& wm,
                        const ActionRegistry& registry) const;

    // Compile with agent context - injects agent_id into the BT blackboard scope.
    // Used by DelegateToAgent for multi-agent execution.
    std::string compile(const std::vector<PlanStep>& plan,
                        const WorldModel& wm,
                        const ActionRegistry& registry,
                        const std::string& agent_id) const;

    // Sequential fallback: emit all steps as a single Sequence (no causal analysis).
    std::string compileSequential(const std::vector<PlanStep>& plan,
                                  const WorldModel& wm,
                                  const ActionRegistry& registry) const;

private:
    // Generate XML for a single action unit (precondition checks + action + effects)
    std::string emitActionUnit(const GroundAction& ga,
                               const WorldModel& wm,
                               const ActionRegistry& registry) const;

    // Emit ReactiveFallback goal guard around the plan body.
    // If all goals are already satisfied, the tree returns SUCCESS
    // without re-executing the plan -- safe for continuous ticking.
    void emitGoalGuardOpen(std::ostringstream& xml,
                           const WorldModel& wm,
                           const std::string& indent) const;
    void emitGoalGuardClose(std::ostringstream& xml,
                            const WorldModel& wm,
                            const std::string& indent) const;

    // Parse action name from signature "move(uav1,base,sector_a)" -> "move"
    static std::string actionName(const std::string& signature);

    // Parse params from signature "move(uav1,base,sector_a)" -> {"uav1","base","sector_a"}
    static std::vector<std::string> actionParams(const std::string& signature);
};

} // namespace ame
