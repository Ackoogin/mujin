#pragma once

#include "ame/action_registry.h"
#include "ame/autonomy_backend.h"
#include "ame/executor_component.h"
#include "ame/goal_allocator.h"
#include "ame/planner.h"
#include "ame/plan_compiler.h"
#include "ame/world_model.h"

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace ame {

/// \brief Adapter that wraps the current AME world-model + planner + compiler flow.
///
/// This class intentionally does not rewrite planner/compiler internals. It exposes
/// the current AME stack behind a whole-system ingress/egress boundary so the stack
/// can be treated as one replaceable autonomy backend.
class CurrentAmeBackendAdapter : public IAutonomyBackend {
public:
  /// \brief Construct adapter around existing AME core objects.
  CurrentAmeBackendAdapter(WorldModel& world_model,
                           const ActionRegistry& action_registry,
                           const Planner& planner = Planner(),
                           const PlanCompiler& plan_compiler = PlanCompiler());
  ~CurrentAmeBackendAdapter();

  AutonomyBackendCapabilities describeCapabilities() const override;
  void start(const SessionRequest& request) override;
  void pushState(const StateUpdate& update) override;
  void pushIntent(const MissionIntent& intent) override;
  void step() override;
  std::vector<ActionCommand> pullCommands() override;
  std::vector<GoalDispatch> pullGoalDispatches() override;
  std::vector<DecisionRecord> pullDecisionRecords() override;
  void pushCommandResult(const CommandResult& result) override;
  void pushDispatchResult(const DispatchResult& result) override;
  void requestStop(StopMode mode) override;
  AutonomyBackendSnapshot readSnapshot() const override;

private:
  class BackendPyramidServiceProxy;

  struct CommandTracking {
    ActionCommand command;
    CommandStatus status = CommandStatus::PENDING;
  };
  struct DispatchTracking {
    GoalDispatch dispatch;
    CommandStatus status = CommandStatus::PENDING;
  };

  static std::string actionName(const std::string& signature);
  static std::vector<std::string> actionParameters(const std::string& signature);
  static FactAuthority toWorldModelAuthority(FactAuthorityLevel authority);
  std::vector<AgentState> collectAgentStates() const;

  void resetTransientQueues();
  void resetExecutionForReplan();
  void loadAndStartExecution(const std::string& bt_xml);
  bool maybeEmitGoalDispatches();
  void restoreDispatchedAgents();
  bool goalsSatisfied() const;

  WorldModel& world_model_;
  const ActionRegistry& action_registry_;
  Planner planner_;
  PlanCompiler plan_compiler_;
  GoalAllocator goal_allocator_;
  ExecutorComponent executor_;
  std::unique_ptr<BackendPyramidServiceProxy> pyramid_proxy_;
  std::string session_id_;
  PolicyEnvelope policy_;
  AutonomyBackendState state_ = AutonomyBackendState::IDLE;
  unsigned replan_count_ = 0;
  std::unordered_map<std::string, CommandTracking> command_tracking_;
  std::unordered_map<std::string, DispatchTracking> dispatch_tracking_;
  std::vector<ActionCommand> pending_command_queue_;
  std::vector<GoalDispatch> pending_goal_dispatch_queue_;
  std::vector<DecisionRecord> pending_decision_records_;
  std::vector<DecisionRecord> decision_history_;
  uint64_t command_counter_ = 0;
  uint64_t dispatch_counter_ = 0;
};

}  // namespace ame
