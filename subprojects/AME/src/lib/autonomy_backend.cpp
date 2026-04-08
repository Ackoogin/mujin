#include "ame/current_ame_backend_adapter.h"

#include <stdexcept>

namespace ame {

CurrentAmeBackendAdapter::CurrentAmeBackendAdapter(
    WorldModel& world_model,
    const ActionRegistry& action_registry,
    const Planner& planner,
    const PlanCompiler& plan_compiler)
    : world_model_(world_model),
      action_registry_(action_registry),
      planner_(planner),
      plan_compiler_(plan_compiler) {}

AutonomyBackendCapabilities CurrentAmeBackendAdapter::describeCapabilities() const {
  return {"ame.current_stack", true, true, true};
}

void CurrentAmeBackendAdapter::start(const SessionRequest& request) {
  session_id_ = request.session_id;
  policy_ = request.policy;
  state_ = AutonomyBackendState::READY;
  replan_count_ = 0;
  command_counter_ = 0;
  command_tracking_.clear();
  pending_command_queue_.clear();
  pending_decision_records_.clear();
  decision_history_.clear();
  world_model_.setGoal(request.intent.goal_fluents);
}

void CurrentAmeBackendAdapter::pushState(const StateUpdate& update) {
  for (const auto& fact_update : update.fact_updates) {
    world_model_.setFact(fact_update.key,
                         fact_update.value,
                         fact_update.source,
                         toWorldModelAuthority(fact_update.authority));
  }
}

void CurrentAmeBackendAdapter::pushIntent(const MissionIntent& intent) {
  world_model_.setGoal(intent.goal_fluents);
  if (state_ != AutonomyBackendState::STOPPED &&
      state_ != AutonomyBackendState::FAILED) {
    state_ = AutonomyBackendState::READY;
  }
  command_tracking_.clear();
  pending_command_queue_.clear();
}

void CurrentAmeBackendAdapter::step() {
  if (state_ == AutonomyBackendState::STOPPED ||
      state_ == AutonomyBackendState::FAILED ||
      state_ == AutonomyBackendState::COMPLETE) {
    return;
  }

  if (goalsSatisfied()) {
    state_ = AutonomyBackendState::COMPLETE;
    return;
  }

  for (const auto& [command_id, tracking] : command_tracking_) {
    if (tracking.status == CommandStatus::PENDING ||
        tracking.status == CommandStatus::RUNNING) {
      state_ = AutonomyBackendState::WAITING_FOR_RESULTS;
      return;
    }
  }

  if (!pending_command_queue_.empty()) {
    state_ = AutonomyBackendState::WAITING_FOR_RESULTS;
    return;
  }

  if (replan_count_ >= policy_.max_replans) {
    state_ = AutonomyBackendState::FAILED;
    return;
  }

  auto result = planner_.solve(world_model_);
  DecisionRecord record;
  record.session_id = session_id_;
  record.backend_id = describeCapabilities().backend_id;
  record.world_version = world_model_.version();
  record.replan_count = replan_count_;
  record.plan_success = result.success;
  record.solve_time_ms = result.solve_time_ms;

  for (const auto& step_result : result.steps) {
    const auto& action = world_model_.groundActions()[step_result.action_index];
    record.planned_action_signatures.push_back(action.signature);
  }
  record.compiled_bt_xml =
      plan_compiler_.compileSequential(result.steps, world_model_, action_registry_);

  pending_decision_records_.push_back(record);
  decision_history_.push_back(record);

  if (!result.success) {
    ++replan_count_;
    if (replan_count_ >= policy_.max_replans) {
      state_ = AutonomyBackendState::FAILED;
    }
    return;
  }

  buildCommandsFromPlan(result.steps);
  state_ = pending_command_queue_.empty() ? AutonomyBackendState::COMPLETE
                                          : AutonomyBackendState::WAITING_FOR_RESULTS;
}

std::vector<ActionCommand> CurrentAmeBackendAdapter::pullCommands() {
  auto commands = pending_command_queue_;
  pending_command_queue_.clear();
  return commands;
}

std::vector<DecisionRecord> CurrentAmeBackendAdapter::pullDecisionRecords() {
  auto records = pending_decision_records_;
  pending_decision_records_.clear();
  return records;
}

void CurrentAmeBackendAdapter::pushCommandResult(const CommandResult& result) {
  auto it = command_tracking_.find(result.command_id);
  if (it == command_tracking_.end()) {
    throw std::invalid_argument("Unknown command_id in pushCommandResult");
  }

  it->second.status = result.status;

  if (!result.observed_updates.empty()) {
    pushState({result.observed_updates});
  } else if (result.status == CommandStatus::SUCCEEDED) {
    applyPredictedEffects(it->second.command, result.source);
  }

  if (result.status == CommandStatus::FAILED_PERMANENT) {
    ++replan_count_;
    if (replan_count_ >= policy_.max_replans) {
      state_ = AutonomyBackendState::FAILED;
      return;
    }
    command_tracking_.clear();
    state_ = AutonomyBackendState::READY;
    return;
  }

  if (result.status == CommandStatus::FAILED_TRANSIENT ||
      result.status == CommandStatus::CANCELLED) {
    ++replan_count_;
    if (replan_count_ >= policy_.max_replans) {
      state_ = AutonomyBackendState::FAILED;
      return;
    }
    command_tracking_.clear();
    state_ = AutonomyBackendState::READY;
    return;
  }

  bool waiting = false;
  for (const auto& [command_id, tracking] : command_tracking_) {
    if (tracking.status == CommandStatus::PENDING ||
        tracking.status == CommandStatus::RUNNING) {
      waiting = true;
      break;
    }
  }

  if (waiting) {
    state_ = AutonomyBackendState::WAITING_FOR_RESULTS;
    return;
  }

  command_tracking_.clear();
  state_ = goalsSatisfied() ? AutonomyBackendState::COMPLETE
                            : AutonomyBackendState::READY;
}

void CurrentAmeBackendAdapter::requestStop(StopMode mode) {
  if (mode == StopMode::IMMEDIATE) {
    resetTransientQueues();
  }
  state_ = AutonomyBackendState::STOPPED;
}

AutonomyBackendSnapshot CurrentAmeBackendAdapter::readSnapshot() const {
  AutonomyBackendSnapshot snapshot;
  snapshot.session_id = session_id_;
  snapshot.state = state_;
  snapshot.world_version = world_model_.version();
  snapshot.replan_count = replan_count_;
  snapshot.decision_history = decision_history_;
  for (const auto& [command_id, tracking] : command_tracking_) {
    if (tracking.status == CommandStatus::PENDING ||
        tracking.status == CommandStatus::RUNNING) {
      snapshot.outstanding_commands.push_back(tracking.command);
    }
  }
  return snapshot;
}

std::string CurrentAmeBackendAdapter::actionName(const std::string& signature) {
  const auto paren = signature.find('(');
  if (paren == std::string::npos) {
    return signature;
  }
  return signature.substr(0, paren);
}

std::vector<std::string> CurrentAmeBackendAdapter::actionParameters(
    const std::string& signature) {
  std::vector<std::string> params;
  const auto open = signature.find('(');
  const auto close = signature.rfind(')');
  if (open == std::string::npos || close == std::string::npos || close <= open + 1) {
    return params;
  }

  std::string token;
  for (size_t i = open + 1; i < close; ++i) {
    if (signature[i] == ',') {
      params.push_back(token);
      token.clear();
      continue;
    }
    token.push_back(signature[i]);
  }
  if (!token.empty()) {
    params.push_back(token);
  }
  return params;
}

FactAuthority CurrentAmeBackendAdapter::toWorldModelAuthority(
    FactAuthorityLevel authority) {
  switch (authority) {
    case FactAuthorityLevel::BELIEVED:
      return FactAuthority::BELIEVED;
    case FactAuthorityLevel::CONFIRMED:
      return FactAuthority::CONFIRMED;
  }
  return FactAuthority::CONFIRMED;
}

void CurrentAmeBackendAdapter::resetTransientQueues() {
  command_tracking_.clear();
  pending_command_queue_.clear();
  pending_decision_records_.clear();
}

void CurrentAmeBackendAdapter::buildCommandsFromPlan(
    const std::vector<PlanStep>& steps) {
  command_tracking_.clear();
  pending_command_queue_.clear();

  for (const auto& step : steps) {
    const auto& action = world_model_.groundActions()[step.action_index];

    ActionCommand command;
    command.command_id = session_id_ + "/cmd/" + std::to_string(++command_counter_);
    command.action_name = actionName(action.signature);
    command.signature = action.signature;
    command.parameters = actionParameters(action.signature);

    for (const auto pre_id : action.preconditions) {
      command.expected_preconditions.push_back(world_model_.fluentName(pre_id));
    }
    for (const auto add_id : action.add_effects) {
      command.predicted_add_effects.push_back(world_model_.fluentName(add_id));
    }
    for (const auto del_id : action.del_effects) {
      command.predicted_del_effects.push_back(world_model_.fluentName(del_id));
    }

    command_tracking_[command.command_id] = {command, CommandStatus::PENDING};
    pending_command_queue_.push_back(command);
  }
}

void CurrentAmeBackendAdapter::applyPredictedEffects(const ActionCommand& command,
                                                     const std::string& source) {
  const std::string effect_source = source.empty() ? command.command_id : source;
  for (const auto& fact : command.predicted_add_effects) {
    world_model_.setFact(fact, true, effect_source, FactAuthority::BELIEVED);
  }
  for (const auto& fact : command.predicted_del_effects) {
    world_model_.setFact(fact, false, effect_source, FactAuthority::BELIEVED);
  }
}

bool CurrentAmeBackendAdapter::goalsSatisfied() const {
  for (const auto goal_id : world_model_.goalFluentIds()) {
    if (!world_model_.getFact(goal_id)) {
      return false;
    }
  }
  return true;
}

}  // namespace ame
