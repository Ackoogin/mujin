#pragma once

#include "ame/autonomy_backend.h"

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace ame {

/// \brief Policy used when an AME command may be converted to placed work.
enum class ExecutionBindingPolicy {
  CommandOnly,
  PreferTypedPlacement,
  RequireTypedPlacement,
};

/// \brief Observable state of a command-to-requirement placement binding.
enum class RequirementPlacementState {
  Unspecified,
  Pending,
  Running,
  Completed,
  Failed,
  Cancelled,
  Unsupported,
};

/// \brief Static binding from an AME command shape to a target requirement CRUD.
struct ExecutionBinding {
  std::string action_name;
  std::string service_name;
  std::string operation;
  std::string target_component;
  std::string target_service;
  std::string target_type;
};

/// \brief Runtime record for a command converted to a placed requirement.
struct RequirementPlacementRecord {
  std::string placement_id;
  std::string command_id;
  std::string action_name;
  std::string signature;
  std::string target_component;
  std::string target_service;
  std::string target_type;
  std::string target_requirement_id;
  RequirementPlacementState state = RequirementPlacementState::Unspecified;
};

/// \brief Result of submitting an AME execution command to an execution sink.
struct ExecutionSubmission {
  bool accepted = false;
  bool command_egress_visible = false;
  std::optional<RequirementPlacementRecord> placement;
  std::string rejection_reason;
};

/// \brief AME-internal execution egress abstraction.
///
/// Implementations may expose commands directly, convert them to typed
/// requirement placements, or combine both behaviours. This keeps AME usable as
/// a non-PYRAMID library while letting wrappers present PYRAMID-style contracts.
class IExecutionSink {
public:
  virtual ~IExecutionSink() = default;

  /// \brief Reset transient sink state for a new backend session.
  virtual void reset(const std::string& session_id) = 0;

  /// \brief Submit a command emitted by AME execution.
  virtual ExecutionSubmission submit(const ActionCommand& command) = 0;

  /// \brief Pull command egress visible to non-PYRAMID callers.
  virtual std::vector<ActionCommand> pullCommands() = 0;

  /// \brief Push a result observed by the configured execution integration.
  virtual void pushResult(const CommandResult& result) = 0;

  /// \brief Cancel a pending command.
  virtual void cancel(const std::string& command_id) = 0;

  /// \brief Return the latest result for a command, if terminal.
  virtual std::optional<CommandResult> resultFor(
      const std::string& command_id) const = 0;

  /// \brief True while the command is pending or running.
  virtual bool isPending(const std::string& command_id) const = 0;

  /// \brief Read retained requirement placement records.
  virtual std::vector<RequirementPlacementRecord> readPlacements() const = 0;
};

/// \brief Default AME sink: expose commands directly and wait for results.
class CommandQueueExecutionSink : public IExecutionSink {
public:
  void reset(const std::string& session_id) override;
  ExecutionSubmission submit(const ActionCommand& command) override;
  std::vector<ActionCommand> pullCommands() override;
  void pushResult(const CommandResult& result) override;
  void cancel(const std::string& command_id) override;
  std::optional<CommandResult> resultFor(
      const std::string& command_id) const override;
  bool isPending(const std::string& command_id) const override;
  std::vector<RequirementPlacementRecord> readPlacements() const override;

private:
  std::string session_id_;
  std::unordered_map<std::string, CommandResult> results_;
  std::unordered_map<std::string, bool> pending_;
  std::vector<ActionCommand> command_queue_;
};

/// \brief Sink that can convert AME commands to typed requirement placements.
class RequirementBindingExecutionSink : public IExecutionSink {
public:
  RequirementBindingExecutionSink(
      ExecutionBindingPolicy policy,
      std::vector<ExecutionBinding> bindings = {});

  void reset(const std::string& session_id) override;
  ExecutionSubmission submit(const ActionCommand& command) override;
  std::vector<ActionCommand> pullCommands() override;
  void pushResult(const CommandResult& result) override;
  void cancel(const std::string& command_id) override;
  std::optional<CommandResult> resultFor(
      const std::string& command_id) const override;
  bool isPending(const std::string& command_id) const override;
  std::vector<RequirementPlacementRecord> readPlacements() const override;

  /// \brief Add or replace a command-to-requirement binding.
  void addBinding(const ExecutionBinding& binding);

private:
  const ExecutionBinding* findBinding(const ActionCommand& command) const;
  static bool bindingMatches(const ExecutionBinding& binding,
                             const ActionCommand& command);
  static RequirementPlacementState stateFromStatus(CommandStatus status);

  ExecutionBindingPolicy policy_ = ExecutionBindingPolicy::CommandOnly;
  std::vector<ExecutionBinding> bindings_;
  CommandQueueExecutionSink fallback_commands_;
  std::string session_id_;
  uint64_t next_placement_id_ = 0;
  std::unordered_map<std::string, RequirementPlacementRecord> placements_;
  std::unordered_map<std::string, CommandResult> results_;
  std::unordered_map<std::string, bool> pending_;
};

}  // namespace ame
