#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace ame {

/// \brief Lifecycle state of a whole-system autonomy backend session.
enum class AutonomyBackendState {
  IDLE,
  READY,
  WAITING_FOR_RESULTS,
  COMPLETE,
  FAILED,
  STOPPED,
};

/// \brief Reported outcome status for an externally dispatched command.
enum class CommandStatus {
  PENDING,
  RUNNING,
  SUCCEEDED,
  FAILED_TRANSIENT,
  FAILED_PERMANENT,
  CANCELLED,
};

/// \brief Requested stop mode for an autonomy backend session.
enum class StopMode {
  DRAIN,
  IMMEDIATE,
};

/// \brief Backend-neutral authority tag for a fact update crossing the shell boundary.
enum class FactAuthorityLevel {
  BELIEVED,
  CONFIRMED,
};

/// \brief Single fact update applied through the backend ingress surface.
struct FactUpdate {
  std::string key;
  bool value = false;
  std::string source;
  FactAuthorityLevel authority = FactAuthorityLevel::CONFIRMED;
};

/// \brief State ingress payload for authoritative world updates.
struct StateUpdate {
  std::vector<FactUpdate> fact_updates;
};

/// \brief Mission intent passed to the backend shell.
struct MissionIntent {
  std::vector<std::string> goal_fluents;
};

/// \brief Backend-neutral view of an agent/resource that can accept delegated work.
struct AgentState {
  std::string agent_id;
  std::string agent_type;
  bool available = true;
};

/// \brief Policy knobs exposed at the whole-system wrapper boundary.
struct PolicyEnvelope {
  unsigned max_replans = 3;
  bool enable_goal_dispatch = false;
};

/// \brief Session bootstrap parameters for a backend run.
struct SessionRequest {
  std::string session_id;
  MissionIntent intent;
  PolicyEnvelope policy;
  std::vector<AgentState> available_agents;
};

/// \brief Declared top-level capabilities of an autonomy backend.
struct AutonomyBackendCapabilities {
  std::string backend_id;
  bool supports_batch_planning = true;
  bool supports_external_command_dispatch = true;
  bool supports_replanning = true;
};

/// \brief Command emitted by the backend egress surface for external dispatch.
struct ActionCommand {
  std::string command_id;
  std::string action_name;
  std::string signature;
  std::string service_name;
  std::string operation;
  std::unordered_map<std::string, std::string> request_fields;
};

/// \brief Goal/task dispatch emitted to an assigned agent or subordinate backend.
struct GoalDispatch {
  std::string dispatch_id;
  std::string agent_id;
  std::vector<std::string> goals;
};

/// \brief Decision/audit record emitted when the backend forms a new plan.
struct DecisionRecord {
  std::string session_id;
  std::string backend_id;
  uint64_t world_version = 0;
  unsigned replan_count = 0;
  bool plan_success = false;
  double solve_time_ms = 0.0;
  std::vector<std::string> planned_action_signatures;
  std::string compiled_bt_xml;
};

/// \brief Result returned by the external dispatcher for a command.
struct CommandResult {
  std::string command_id;
  CommandStatus status = CommandStatus::PENDING;
  std::vector<FactUpdate> observed_updates;
  std::string source;
};

/// \brief Result returned for a previously emitted goal dispatch.
struct DispatchResult {
  std::string dispatch_id;
  CommandStatus status = CommandStatus::PENDING;
  std::vector<FactUpdate> observed_updates;
  std::string source;
};

/// \brief Snapshot of the wrapper-visible backend state.
struct AutonomyBackendSnapshot {
  std::string session_id;
  AutonomyBackendState state = AutonomyBackendState::IDLE;
  uint64_t world_version = 0;
  unsigned replan_count = 0;
  std::vector<AgentState> agent_states;
  std::vector<ActionCommand> outstanding_commands;
  std::vector<GoalDispatch> outstanding_goal_dispatches;
  std::vector<DecisionRecord> decision_history;
};

/// \brief Whole-system swap surface that wraps both state ingress and action egress.
class IAutonomyBackend {
public:
  virtual ~IAutonomyBackend() = default;

  /// \brief Describe capabilities of this backend implementation.
  virtual AutonomyBackendCapabilities describeCapabilities() const = 0;

  /// \brief Start or reset a backend session.
  virtual void start(const SessionRequest& request) = 0;

  /// \brief Push authoritative state changes into the backend shell.
  virtual void pushState(const StateUpdate& update) = 0;

  /// \brief Replace mission intent for the active session.
  virtual void pushIntent(const MissionIntent& intent) = 0;

  /// \brief Advance backend deliberation and populate egress records as needed.
  virtual void step() = 0;

  /// \brief Pull commands that are ready for external dispatch.
  virtual std::vector<ActionCommand> pullCommands() = 0;

  /// \brief Pull goal dispatches that are ready for external delegation.
  virtual std::vector<GoalDispatch> pullGoalDispatches() = 0;

  /// \brief Pull decision records generated since the previous read.
  virtual std::vector<DecisionRecord> pullDecisionRecords() = 0;

  /// \brief Push an externally observed command result back into the backend.
  virtual void pushCommandResult(const CommandResult& result) = 0;

  /// \brief Push an externally observed goal-dispatch result back into the backend.
  virtual void pushDispatchResult(const DispatchResult& result) = 0;

  /// \brief Request stop for the active session.
  virtual void requestStop(StopMode mode) = 0;

  /// \brief Read current backend session snapshot.
  virtual AutonomyBackendSnapshot readSnapshot() const = 0;
};

}  // namespace ame
