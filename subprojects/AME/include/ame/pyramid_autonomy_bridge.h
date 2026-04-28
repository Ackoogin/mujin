#pragma once

#include "ame/action_registry.h"
#include "ame/execution_sink.h"
#include "ame/planner.h"
#include "ame/plan_compiler.h"
#include "ame/world_model.h"

#include "pyramid_services_autonomy_backend_provided.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace ame {

/// \brief Construction options for the PYRAMID-facing AME bridge.
struct PyramidAutonomyBridgeOptions {
  std::shared_ptr<IExecutionSink> execution_sink;
  ExecutionBindingPolicy execution_policy =
      ExecutionBindingPolicy::PreferTypedPlacement;
  bool include_default_execution_bindings = true;
  std::vector<ExecutionBinding> execution_bindings;
};

/// \brief PYRAMID EntityActions bridge for the AME planning and execution contract.
class PyramidAutonomyBridge
    : public pyramid::components::autonomy_backend::services::provided::ServiceHandler {
public:
  using Ack = pyramid::domain_model::Ack;
  using Capabilities = pyramid::domain_model::Capabilities;
  using ExecutionRequirement = pyramid::domain_model::ExecutionRequirement;
  using ExecutionRun = pyramid::domain_model::ExecutionRun;
  using Identifier = pyramid::domain_model::Identifier;
  using Plan = pyramid::domain_model::Plan;
  using PlanningRequirement = pyramid::domain_model::PlanningRequirement;
  using Query = pyramid::domain_model::Query;
  using RequirementPlacement = pyramid::domain_model::RequirementPlacement;
  using StateUpdate = pyramid::domain_model::StateUpdate;

  /// \brief Construct a bridge over the current AME planning stack.
  PyramidAutonomyBridge(WorldModel& world_model,
                        const ActionRegistry& action_registry,
                        const Planner& planner = Planner(),
                        const PlanCompiler& plan_compiler = PlanCompiler(),
                        std::shared_ptr<IExecutionSink> execution_sink =
                            nullptr);

  /// \brief Construct a bridge with explicit execution binding options.
  PyramidAutonomyBridge(WorldModel& world_model,
                        const ActionRegistry& action_registry,
                        const PyramidAutonomyBridgeOptions& options,
                        const Planner& planner = Planner(),
                        const PlanCompiler& plan_compiler = PlanCompiler());

  /// \brief Built-in example bindings used by the default PYRAMID wrapper.
  static std::vector<ExecutionBinding> defaultExecutionBindings();

  /// \brief Register or replace a command-to-requirement execution binding.
  ///
  /// Returns false when this bridge was constructed with a custom sink that
  /// does not support binding registration.
  bool registerExecutionBinding(const ExecutionBinding& binding);

  std::vector<Capabilities> handleReadCapabilities(
      const Query& request) override;
  Identifier handleCreatePlanningRequirement(
      const PlanningRequirement& request) override;
  std::vector<PlanningRequirement> handleReadPlanningRequirement(
      const Query& request) override;
  Ack handleUpdatePlanningRequirement(
      const PlanningRequirement& request) override;
  Ack handleDeletePlanningRequirement(const Identifier& request) override;
  Identifier handleCreateExecutionRequirement(
      const ExecutionRequirement& request) override;
  std::vector<ExecutionRequirement> handleReadExecutionRequirement(
      const Query& request) override;
  Ack handleUpdateExecutionRequirement(
      const ExecutionRequirement& request) override;
  Ack handleDeleteExecutionRequirement(const Identifier& request) override;
  Identifier handleCreateState(const StateUpdate& request) override;
  Ack handleUpdateState(const StateUpdate& request) override;
  Ack handleDeleteState(const Identifier& request) override;
  Identifier handleCreatePlan(const Plan& request) override;
  std::vector<Plan> handleReadPlan(const Query& request) override;
  Ack handleUpdatePlan(const Plan& request) override;
  Ack handleDeletePlan(const Identifier& request) override;
  std::vector<ExecutionRun> handleReadRun(const Query& request) override;
  std::vector<RequirementPlacement> handleReadPlacement(
      const Query& request) override;

private:
  struct PlacementContext {
    std::string execution_requirement_id;
    std::string requirement_id;
    std::string plan_id;
    std::string plan_step_id;
  };

  static FactAuthority toWorldModelAuthority(
      pyramid::domain_model::FactAuthorityLevel authority);
  static std::string actionName(const std::string& signature);
  static std::vector<std::string> actionParameters(
      const std::string& signature);
  static bool queryIncludes(const Query& query, const std::string& id);
  static pyramid::domain_model::common::Progress progressFromPlacementState(
      RequirementPlacementState state);

  std::string nextId(const std::string& prefix);
  void applyPlanningContext(const PlanningRequirement& requirement);
  void applyExecutionContext(const ExecutionRequirement& requirement);
  Plan makePlan(const PlanningRequirement& requirement,
                const std::string& requirement_id);
  ExecutionRun makeRun(const ExecutionRequirement& requirement,
                       const Plan& plan) const;
  RequirementPlacement makePlacement(const std::string& execution_requirement_id,
                                     const Plan& plan,
                                     const pyramid::domain_model::PlanStep& step) const;
  ActionCommand makeCommand(const std::string& execution_requirement_id,
                            const pyramid::domain_model::PlanStep& step) const;
  RequirementPlacement makePlacement(
      const RequirementPlacementRecord& record) const;
  void syncPlacementRecordsFromSink();
  bool requirementGoalsSatisfied(
      const ExecutionRequirement& requirement) const;
  void refreshExecutionProgress(const std::string& execution_requirement_id);
  std::vector<RequirementPlacement> outstandingPlacements(
      const std::string& execution_requirement_id,
      const std::string& plan_id) const;
  void applyStateUpdate(const StateUpdate& update);

  WorldModel& world_model_;
  const ActionRegistry& action_registry_;
  Planner planner_;
  PlanCompiler plan_compiler_;
  std::shared_ptr<IExecutionSink> execution_sink_;
  uint64_t next_id_ = 0;

  std::unordered_map<std::string, PlanningRequirement> planning_requirements_;
  std::unordered_map<std::string, ExecutionRequirement> execution_requirements_;
  std::unordered_map<std::string, Plan> plans_;
  std::unordered_map<std::string, ExecutionRun> runs_;
  std::unordered_map<std::string, RequirementPlacement> placements_;
  std::unordered_map<std::string, PlacementContext> placement_contexts_;
};

}  // namespace ame
