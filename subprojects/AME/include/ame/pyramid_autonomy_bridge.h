#pragma once

#include "ame/action_registry.h"
#include "ame/planner.h"
#include "ame/plan_compiler.h"
#include "ame/world_model.h"

#include "pyramid_services_autonomy_backend_provided.hpp"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace ame {

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
                        const PlanCompiler& plan_compiler = PlanCompiler());

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
  static FactAuthority toWorldModelAuthority(
      pyramid::data_model::FactAuthorityLevel authority);
  static std::string actionName(const std::string& signature);
  static bool queryIncludes(const Query& query, const std::string& id);

  std::string nextId(const std::string& prefix);
  Plan makePlan(const PlanningExecutionRequirement& requirement,
                const std::string& requirement_id);
  ExecutionRun makeRun(const PlanningExecutionRequirement& requirement,
                       const Plan& plan) const;
  RequirementPlacement makePlacement(const std::string& requirement_id,
                                     const Plan& plan,
                                     const pyramid::data_model::PlanStep& step) const;
  void applyStateUpdate(const StateUpdate& update);

  WorldModel& world_model_;
  const ActionRegistry& action_registry_;
  Planner planner_;
  PlanCompiler plan_compiler_;
  uint64_t next_id_ = 0;

  std::unordered_map<std::string, PlanningExecutionRequirement> requirements_;
  std::unordered_map<std::string, Plan> plans_;
  std::unordered_map<std::string, ExecutionRun> runs_;
  std::unordered_map<std::string, RequirementPlacement> placements_;
};

}  // namespace ame
