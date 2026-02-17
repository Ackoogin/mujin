#pragma once

#include "mujin/action_registry.h"
#include "mujin/world_model.h"

#include <string>
#include <vector>

namespace mujin {

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

    // Sequential fallback: emit all steps as a single Sequence (no causal analysis).
    std::string compileSequential(const std::vector<PlanStep>& plan,
                                  const WorldModel& wm,
                                  const ActionRegistry& registry) const;

private:
    // Generate XML for a single action unit (precondition checks + action + effects)
    std::string emitActionUnit(const GroundAction& ga,
                               const WorldModel& wm,
                               const ActionRegistry& registry) const;

    // Parse action name from signature "move(uav1,base,sector_a)" -> "move"
    static std::string actionName(const std::string& signature);

    // Parse params from signature "move(uav1,base,sector_a)" -> {"uav1","base","sector_a"}
    static std::vector<std::string> actionParams(const std::string& signature);
};

} // namespace mujin
