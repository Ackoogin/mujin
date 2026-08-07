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

/// \brief Metadata for one action-unit wrapper emitted into a compiled BT.
struct PlanStepMeta {
    unsigned step_index;
    std::string action_unit_name;
};

/// \brief Compiled BT XML plus sidecar metadata for execution repair hooks.
struct CompiledPlan {
    std::string xml;
    std::vector<PlanStepMeta> steps;
    bool has_goal_guard = false;
};

class PlanCompiler {
public:
    // Compile a plan (sequence of action indices) into BT XML.
    // Uses causal graph analysis for parallelism when possible.
    std::string compile(const std::vector<PlanStep>& plan,
                        const WorldModel& wm,
                        const ActionRegistry& registry) const;
    std::string compile(const std::vector<PlanStep>& plan,
                        const WorldModel& wm,
                        const ActionRegistry& registry,
                        const std::vector<unsigned>& goal_ids) const;

    /// \brief Compile a plan to BT XML with per-step sidecar metadata.
    CompiledPlan compileWithMetadata(const std::vector<PlanStep>& plan,
                                     const WorldModel& wm,
                                     const ActionRegistry& registry,
                                     const std::vector<unsigned>& goal_ids) const;

    // Compile with agent context - injects agent_id into the BT blackboard scope.
    // Used by DelegateToAgent for multi-agent execution.
    std::string compile(const std::vector<PlanStep>& plan,
                        const WorldModel& wm,
                        const ActionRegistry& registry,
                        const std::string& agent_id) const;
    std::string compile(const std::vector<PlanStep>& plan,
                        const WorldModel& wm,
                        const ActionRegistry& registry,
                        const std::string& agent_id,
                        const std::vector<unsigned>& goal_ids) const;

    // Sequential fallback: emit all steps as a single Sequence (no causal analysis).
    std::string compileSequential(const std::vector<PlanStep>& plan,
                                  const WorldModel& wm,
                                  const ActionRegistry& registry) const;
    std::string compileSequential(const std::vector<PlanStep>& plan,
                                  const WorldModel& wm,
                                  const ActionRegistry& registry,
                                  const std::vector<unsigned>& goal_ids) const;

    /// \brief Control how the compiler handles a planned action that has no
    /// registered BT binding.
    ///
    /// Production execution must fail closed on an unbound action, so the
    /// default is to throw "Unregistered action in plan". The authoring /
    /// devenv tooling enables stub mode to preview a plan before every action
    /// has a real BT binding: each unregistered action compiles to a stub unit
    /// (its precondition checks and effect-predicate writes wrapped around a
    /// trivially-succeeding body) instead of aborting the whole compile.
    void setStubUnregisteredActions(bool enabled) { stub_unregistered_ = enabled; }
    bool stubUnregisteredActions() const { return stub_unregistered_; }

private:
    // Generate one planned-action XML element with its state contract as ports.
    std::string emitActionUnit(const GroundAction& ga,
                               const WorldModel& wm,
                               const ActionRegistry& registry) const;

    // Emit a ReactiveFallback with one GoalReached condition around the plan.
    void emitGoalGuardOpen(std::ostringstream& xml,
                           const WorldModel& wm,
                           const std::string& indent) const;
    void emitGoalGuardOpen(std::ostringstream& xml,
                           const WorldModel& wm,
                           const std::vector<unsigned>& goal_ids,
                           const std::string& indent) const;
    void emitGoalGuardClose(std::ostringstream& xml,
                            const WorldModel& wm,
                            const std::string& indent) const;
    void emitGoalGuardClose(std::ostringstream& xml,
                            const std::vector<unsigned>& goal_ids,
                            const std::string& indent) const;

    // Metadata-producing sequential compile used by compileSequential().
    CompiledPlan compileSequentialWithMetadata(const std::vector<PlanStep>& plan,
                                               const WorldModel& wm,
                                               const ActionRegistry& registry,
                                               const std::vector<unsigned>& goal_ids) const;

    // Parse action name from signature "move(uav1,base,sector_a)" -> "move"
    static std::string actionName(const std::string& signature);

    // Parse params from signature "move(uav1,base,sector_a)" -> {"uav1","base","sector_a"}
    static std::vector<std::string> actionParams(const std::string& signature);

    // When true, planned actions with no registered BT binding compile to a
    // stub unit instead of throwing. Off by default so production fails closed.
    bool stub_unregistered_ = false;
};

} // namespace ame
