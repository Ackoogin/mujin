#include "ame/execution_sink.h"

#include <stdexcept>
#include <utility>

namespace ame {

namespace {

bool isTerminal(CommandStatus status) {
  return status == CommandStatus::SUCCEEDED ||
         status == CommandStatus::FAILED_TRANSIENT ||
         status == CommandStatus::FAILED_PERMANENT ||
         status == CommandStatus::CANCELLED;
}

CommandResult makeStatusResult(const std::string& command_id,
                               CommandStatus status,
                               const std::string& source) {
  CommandResult result;
  result.command_id = command_id;
  result.status = status;
  result.source = source;
  return result;
}

}  // namespace

void CommandQueueExecutionSink::reset(const std::string& session_id) {
  session_id_ = session_id;
  results_.clear();
  pending_.clear();
  command_queue_.clear();
}

ExecutionSubmission CommandQueueExecutionSink::submit(
    const ActionCommand& command) {
  pending_[command.command_id] = true;
  command_queue_.push_back(command);

  ExecutionSubmission submission;
  submission.accepted = true;
  submission.command_egress_visible = true;
  return submission;
}

std::vector<ActionCommand> CommandQueueExecutionSink::pullCommands() {
  auto commands = command_queue_;
  command_queue_.clear();
  return commands;
}

void CommandQueueExecutionSink::pushResult(const CommandResult& result) {
  if (pending_.find(result.command_id) == pending_.end()) {
    throw std::invalid_argument("Unknown command_id in execution sink result");
  }
  results_[result.command_id] = result;
  if (isTerminal(result.status)) {
    pending_[result.command_id] = false;
  }
}

void CommandQueueExecutionSink::cancel(const std::string& command_id) {
  if (pending_.find(command_id) == pending_.end()) {
    return;
  }
  results_[command_id] =
      makeStatusResult(command_id, CommandStatus::CANCELLED, "ame:cancel");
  pending_[command_id] = false;
}

std::optional<CommandResult> CommandQueueExecutionSink::resultFor(
    const std::string& command_id) const {
  auto it = results_.find(command_id);
  if (it == results_.end() || !isTerminal(it->second.status)) {
    return std::nullopt;
  }
  return it->second;
}

bool CommandQueueExecutionSink::isPending(const std::string& command_id) const {
  auto it = pending_.find(command_id);
  return it != pending_.end() && it->second;
}

std::vector<RequirementPlacementRecord>
CommandQueueExecutionSink::readPlacements() const {
  return {};
}

RequirementBindingExecutionSink::RequirementBindingExecutionSink(
    ExecutionBindingPolicy policy,
    std::vector<ExecutionBinding> bindings)
    : policy_(policy), bindings_(std::move(bindings)) {}

void RequirementBindingExecutionSink::reset(const std::string& session_id) {
  session_id_ = session_id;
  next_placement_id_ = 0;
  placements_.clear();
  results_.clear();
  pending_.clear();
  fallback_commands_.reset(session_id);
}

ExecutionSubmission RequirementBindingExecutionSink::submit(
    const ActionCommand& command) {
  if (policy_ == ExecutionBindingPolicy::CommandOnly) {
    return fallback_commands_.submit(command);
  }

  const auto* binding = findBinding(command);
  if (binding == nullptr) {
    if (policy_ == ExecutionBindingPolicy::PreferTypedPlacement) {
      return fallback_commands_.submit(command);
    }

    ExecutionSubmission rejected;
    rejected.accepted = false;
    rejected.rejection_reason = "No typed requirement binding for command";
    results_[command.command_id] =
        makeStatusResult(command.command_id,
                         CommandStatus::FAILED_PERMANENT,
                         "ame:execution-binding");
    pending_[command.command_id] = false;
    return rejected;
  }

  RequirementPlacementRecord placement;
  placement.placement_id =
      session_id_ + "/placement/" + std::to_string(++next_placement_id_);
  placement.command_id = command.command_id;
  placement.action_name = command.action_name;
  placement.signature = command.signature;
  placement.target_component = binding->target_component;
  placement.target_service = binding->target_service;
  placement.target_type = binding->target_type;
  placement.target_requirement_id = placement.placement_id + "/requirement";
  placement.state = RequirementPlacementState::Running;
  placements_[command.command_id] = placement;
  pending_[command.command_id] = true;

  ExecutionSubmission submission;
  submission.accepted = true;
  submission.command_egress_visible = false;
  submission.placement = placement;
  return submission;
}

std::vector<ActionCommand> RequirementBindingExecutionSink::pullCommands() {
  return fallback_commands_.pullCommands();
}

void RequirementBindingExecutionSink::pushResult(const CommandResult& result) {
  auto placement_it = placements_.find(result.command_id);
  if (placement_it == placements_.end()) {
    fallback_commands_.pushResult(result);
    return;
  }

  results_[result.command_id] = result;
  placement_it->second.state = stateFromStatus(result.status);
  if (isTerminal(result.status)) {
    pending_[result.command_id] = false;
  }
}

void RequirementBindingExecutionSink::cancel(const std::string& command_id) {
  auto placement_it = placements_.find(command_id);
  if (placement_it == placements_.end()) {
    fallback_commands_.cancel(command_id);
    return;
  }

  placement_it->second.state = RequirementPlacementState::Cancelled;
  results_[command_id] =
      makeStatusResult(command_id, CommandStatus::CANCELLED, "ame:cancel");
  pending_[command_id] = false;
}

std::optional<CommandResult> RequirementBindingExecutionSink::resultFor(
    const std::string& command_id) const {
  auto it = results_.find(command_id);
  if (it != results_.end() && isTerminal(it->second.status)) {
    return it->second;
  }
  return fallback_commands_.resultFor(command_id);
}

bool RequirementBindingExecutionSink::isPending(
    const std::string& command_id) const {
  auto it = pending_.find(command_id);
  if (it != pending_.end()) {
    return it->second;
  }
  return fallback_commands_.isPending(command_id);
}

std::vector<RequirementPlacementRecord>
RequirementBindingExecutionSink::readPlacements() const {
  std::vector<RequirementPlacementRecord> records;
  records.reserve(placements_.size());
  for (const auto& [command_id, placement] : placements_) {
    records.push_back(placement);
  }
  return records;
}

void RequirementBindingExecutionSink::addBinding(
    const ExecutionBinding& binding) {
  for (auto& existing : bindings_) {
    if (existing.action_name == binding.action_name &&
        existing.service_name == binding.service_name &&
        existing.operation == binding.operation) {
      existing = binding;
      return;
    }
  }
  bindings_.push_back(binding);
}

const ExecutionBinding* RequirementBindingExecutionSink::findBinding(
    const ActionCommand& command) const {
  for (const auto& binding : bindings_) {
    if (bindingMatches(binding, command)) {
      return &binding;
    }
  }
  return nullptr;
}

bool RequirementBindingExecutionSink::bindingMatches(
    const ExecutionBinding& binding,
    const ActionCommand& command) {
  const auto action_matches =
      binding.action_name.empty() || binding.action_name == command.action_name;
  const auto service_matches =
      binding.service_name.empty() ||
      binding.service_name == command.service_name;
  const auto operation_matches =
      binding.operation.empty() || binding.operation == command.operation;
  return action_matches && service_matches && operation_matches;
}

RequirementPlacementState RequirementBindingExecutionSink::stateFromStatus(
    CommandStatus status) {
  switch (status) {
    case CommandStatus::PENDING:
      return RequirementPlacementState::Pending;
    case CommandStatus::RUNNING:
      return RequirementPlacementState::Running;
    case CommandStatus::SUCCEEDED:
      return RequirementPlacementState::Completed;
    case CommandStatus::FAILED_TRANSIENT:
    case CommandStatus::FAILED_PERMANENT:
      return RequirementPlacementState::Failed;
    case CommandStatus::CANCELLED:
      return RequirementPlacementState::Cancelled;
  }
  return RequirementPlacementState::Unspecified;
}

}  // namespace ame
