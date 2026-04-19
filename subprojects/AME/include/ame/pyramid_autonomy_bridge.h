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

/// \brief PYRAMID EntityActions bridge for the AME planning/execution contract.
class PyramidAutonomyBridge
    : public pyramid::services::autonomy_backend::provided::ServiceHandler {
public:
  using Ack = pyramid::data_model::Ack;
  using Capabilities = pyramid::data_model::Capabilities;
  using ExecutionRun = pyramid::data_model::ExecutionRun;
  using Identifier = pyramid::data_model::Identifier;
  using Plan = pyramid::data_model::Plan;
  using PlanningExecutionRequirement =
      pyramid::data_model::PlanningExecutionRequirement;
  using Query = pyramid::data_model::Query;
  using RequirementPlacement = pyramid::data_model::RequirementPlacement;
  using StateUpdate = pyramid::data_model::StateUpdate;

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
  Identifier handleCreateRequirement(
      const PlanningExecutionRequirement& request) override;
  std::vector<PlanningExecutionRequirement> handleReadRequirement(
      const Query& request) override;
  Ack handleUpdateRequirement(
      const PlanningExecutionRequirement& request) override;
  Ack handleDeleteRequirement(const Identifier& request) override;
  Identifier handleCreateState(const StateUpdate& request) override;
  Ack handleUpdateState(const StateUpdate& request) override;
  Ack handleDeleteState(const Identifier& request) override;
  std::vector<Plan> handleReadPlan(const Query& request) override;
  std::vector<ExecutionRun> handleReadRun(const Query& request) override;
  std::vector<RequirementPlacement> handleReadPlacement(
      const Query& request) override;

private:
  struct PlacementContext {
    std::string requirement_id;
    std::string plan_id;
    std::string plan_step_id;
  };

  static FactAuthority toWorldModelAuthority(
      pyramid::data_model::FactAuthorityLevel authority);
  static std::string actionName(const std::string& signature);
  static std::vector<std::string> actionParameters(
      const std::string& signature);
  static bool queryIncludes(const Query& query, const std::string& id);
  static pyramid::data_model::common::Progress progressFromPlacementState(
      RequirementPlacementState state);

  std::string nextId(const std::string& prefix);
  Plan makePlan(const PlanningExecutionRequirement& requirement,
                const std::string& requirement_id);
  ExecutionRun makeRun(const PlanningExecutionRequirement& requirement,
                       const Plan& plan) const;
  RequirementPlacement makePlacement(const std::string& requirement_id,
                                     const Plan& plan,
                                     const pyramid::data_model::PlanStep& step) const;
  ActionCommand makeCommand(const std::string& requirement_id,
                            const pyramid::data_model::PlanStep& step) const;
  RequirementPlacement makePlacement(
      const RequirementPlacementRecord& record) const;
  void syncPlacementRecordsFromSink();
  bool requirementGoalsSatisfied(
      const PlanningExecutionRequirement& requirement) const;
  void refreshExecutionProgress(const std::string& requirement_id);
  std::vector<RequirementPlacement> outstandingPlacements(
      const std::string& requirement_id,
      const std::string& plan_id) const;
  void applyStateUpdate(const StateUpdate& update);

  WorldModel& world_model_;
  const ActionRegistry& action_registry_;
  Planner planner_;
  PlanCompiler plan_compiler_;
  std::shared_ptr<IExecutionSink> execution_sink_;
  uint64_t next_id_ = 0;

  std::unordered_map<std::string, PlanningExecutionRequirement> requirements_;
  std::unordered_map<std::string, Plan> plans_;
  std::unordered_map<std::string, ExecutionRun> runs_;
  std::unordered_map<std::string, RequirementPlacement> placements_;
  std::unordered_map<std::string, PlacementContext> placement_contexts_;
};

}  // namespace ame
