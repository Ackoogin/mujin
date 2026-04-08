#include "ame/current_ame_backend_adapter.h"

#include "ame/bt_nodes/invoke_service.h"
#include "ame/pyramid_service.h"

#include <stdexcept>
#include <utility>

namespace ame {

class CurrentAmeBackendAdapter::BackendPyramidServiceProxy : public IPyramidService {
public:
  struct PendingCall {
    ActionCommand command;
    bool emitted = false;
    bool has_result = false;
    AsyncCallStatus result_status = AsyncCallStatus::PENDING;
    ServiceMessage response;
  };

  explicit BackendPyramidServiceProxy(std::string session_id)
      : session_id_(std::move(session_id)) {}

  void reset(std::string session_id) {
    session_id_ = std::move(session_id);
    next_request_id_ = 0;
    pending_calls_.clear();
    emitted_commands_.clear();
  }

  std::vector<ActionCommand> drainEmittedCommands() {
    auto commands = emitted_commands_;
    emitted_commands_.clear();
    for (auto& command : commands) {
      auto it = pending_calls_.find(command.command_id);
      if (it != pending_calls_.end()) {
        it->second.emitted = true;
      }
    }
    return commands;
  }

  void submitResult(const CommandResult& result) {
    auto it = pending_calls_.find(result.command_id);
    if (it == pending_calls_.end()) {
      throw std::invalid_argument("Unknown command_id in proxy result");
    }

    it->second.has_result = true;
    it->second.response.fields.clear();
    for (const auto& update : result.observed_updates) {
      it->second.response.set(update.key, update.value ? "true" : "false");
    }

    switch (result.status) {
      case CommandStatus::PENDING:
      case CommandStatus::RUNNING:
        it->second.result_status = AsyncCallStatus::PENDING;
        break;
      case CommandStatus::SUCCEEDED:
        it->second.result_status = AsyncCallStatus::SUCCESS;
        break;
      case CommandStatus::FAILED_TRANSIENT:
      case CommandStatus::FAILED_PERMANENT:
        it->second.result_status = AsyncCallStatus::FAILURE;
        break;
      case CommandStatus::CANCELLED:
        it->second.result_status = AsyncCallStatus::CANCELLED;
        break;
    }
  }

  bool hasPendingCalls() const {
    for (const auto& [command_id, call] : pending_calls_) {
      if (!call.has_result) {
        return true;
      }
    }
    return false;
  }

  void forgetCompletedCalls() {
    for (auto it = pending_calls_.begin(); it != pending_calls_.end();) {
      if (it->second.has_result &&
          it->second.result_status != AsyncCallStatus::PENDING) {
        it = pending_calls_.erase(it);
      } else {
        ++it;
      }
    }
  }

  bool call(const std::string&, const std::string&,
            const ServiceMessage&, ServiceMessage&) override {
    return false;
  }

  uint64_t callAsync(const std::string& service_name,
                     const std::string& operation,
                     const ServiceMessage& request) override {
    const auto request_id = ++next_request_id_;
    ActionCommand command;
    command.command_id = session_id_ + "/svc/" + std::to_string(request_id);
    command.action_name = operation;
    command.signature = service_name + "::" + operation;
    command.service_name = service_name;
    command.operation = operation;
    command.request_fields = request.fields;

    PendingCall pending;
    pending.command = command;
    pending_calls_[command.command_id] = pending;
    emitted_commands_.push_back(command);
    request_id_to_command_[request_id] = command.command_id;
    return request_id;
  }

  AsyncCallStatus pollResult(uint64_t request_id, ServiceMessage& response) override {
    auto cmd_it = request_id_to_command_.find(request_id);
    if (cmd_it == request_id_to_command_.end()) {
      return AsyncCallStatus::FAILURE;
    }

    auto pending_it = pending_calls_.find(cmd_it->second);
    if (pending_it == pending_calls_.end()) {
      return AsyncCallStatus::FAILURE;
    }

    if (!pending_it->second.has_result) {
      return AsyncCallStatus::PENDING;
    }

    response = pending_it->second.response;
    return pending_it->second.result_status;
  }

  void cancelCall(uint64_t request_id) override {
    auto cmd_it = request_id_to_command_.find(request_id);
    if (cmd_it == request_id_to_command_.end()) {
      return;
    }
    auto pending_it = pending_calls_.find(cmd_it->second);
    if (pending_it == pending_calls_.end()) {
      return;
    }
    pending_it->second.has_result = true;
    pending_it->second.result_status = AsyncCallStatus::CANCELLED;
  }

private:
  std::string session_id_;
  uint64_t next_request_id_ = 0;
  std::unordered_map<uint64_t, std::string> request_id_to_command_;
  std::unordered_map<std::string, PendingCall> pending_calls_;
  std::vector<ActionCommand> emitted_commands_;
};

CurrentAmeBackendAdapter::CurrentAmeBackendAdapter(
    WorldModel& world_model,
    const ActionRegistry& action_registry,
    const Planner& planner,
    const PlanCompiler& plan_compiler)
    : world_model_(world_model),
      action_registry_(action_registry),
      planner_(planner),
      plan_compiler_(plan_compiler),
      pyramid_proxy_(std::make_unique<BackendPyramidServiceProxy>("")) {
  executor_.setInProcessWorldModel(&world_model_);
  executor_.setBlackboardInitializer([this](const BT::Blackboard::Ptr& blackboard) {
    blackboard->set("pyramid_service",
                    static_cast<IPyramidService*>(pyramid_proxy_.get()));
  });
  executor_.factory().registerNodeType<InvokeService>("InvokeService");
  if (executor_.configure() != PCL_OK || executor_.activate() != PCL_OK) {
    throw std::runtime_error("Failed to initialize ExecutorComponent");
  }
}

CurrentAmeBackendAdapter::~CurrentAmeBackendAdapter() {
  executor_.haltExecution();
  executor_.deactivate();
  executor_.cleanup();
  executor_.shutdown();
}

AutonomyBackendCapabilities CurrentAmeBackendAdapter::describeCapabilities() const {
  return {"ame.current_stack", true, true, true};
}

void CurrentAmeBackendAdapter::start(const SessionRequest& request) {
  session_id_ = request.session_id;
  policy_ = request.policy;
  state_ = AutonomyBackendState::READY;
  replan_count_ = 0;
  dispatch_counter_ = 0;
  command_tracking_.clear();
  dispatch_tracking_.clear();
  pending_command_queue_.clear();
  pending_goal_dispatch_queue_.clear();
  pending_decision_records_.clear();
  decision_history_.clear();
  pyramid_proxy_->reset(session_id_);
  executor_.haltExecution();
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
  resetExecutionForReplan();
  if (state_ != AutonomyBackendState::STOPPED &&
      state_ != AutonomyBackendState::FAILED) {
    state_ = AutonomyBackendState::READY;
  }
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

  bool waiting_on_dispatch = false;
  for (const auto& [dispatch_id, tracking] : dispatch_tracking_) {
    if (tracking.status == CommandStatus::PENDING ||
        tracking.status == CommandStatus::RUNNING) {
      waiting_on_dispatch = true;
      break;
    }
  }
  if (waiting_on_dispatch || !pending_goal_dispatch_queue_.empty()) {
    state_ = AutonomyBackendState::WAITING_FOR_RESULTS;
    return;
  }

  if (!executor_.isExecuting()) {
    if (maybeEmitGoalDispatches()) {
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

    loadAndStartExecution(record.compiled_bt_xml);
  }

  executor_.tickOnce();

  auto emitted_commands = pyramid_proxy_->drainEmittedCommands();
  if (!emitted_commands.empty()) {
    for (const auto& command : emitted_commands) {
      command_tracking_[command.command_id] = {command, CommandStatus::PENDING};
      pending_command_queue_.push_back(command);
    }
    state_ = AutonomyBackendState::WAITING_FOR_RESULTS;
    return;
  }

  if (executor_.isExecuting()) {
    if (executor_.lastStatus() == BT::NodeStatus::RUNNING) {
      state_ = pyramid_proxy_->hasPendingCalls()
                   ? AutonomyBackendState::WAITING_FOR_RESULTS
                   : AutonomyBackendState::READY;
      return;
    }
  }

  if (executor_.lastStatus() == BT::NodeStatus::SUCCESS) {
    pyramid_proxy_->forgetCompletedCalls();
    command_tracking_.clear();
    state_ = goalsSatisfied() ? AutonomyBackendState::COMPLETE
                              : AutonomyBackendState::READY;
    executor_.haltExecution();
    return;
  }

  if (executor_.lastStatus() == BT::NodeStatus::FAILURE) {
    ++replan_count_;
    resetExecutionForReplan();
    state_ = (replan_count_ >= policy_.max_replans)
                 ? AutonomyBackendState::FAILED
                 : AutonomyBackendState::READY;
  }
}

std::vector<ActionCommand> CurrentAmeBackendAdapter::pullCommands() {
  auto commands = pending_command_queue_;
  pending_command_queue_.clear();
  return commands;
}

std::vector<GoalDispatch> CurrentAmeBackendAdapter::pullGoalDispatches() {
  auto dispatches = pending_goal_dispatch_queue_;
  pending_goal_dispatch_queue_.clear();
  return dispatches;
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
  }
  pyramid_proxy_->submitResult(result);
}

void CurrentAmeBackendAdapter::pushDispatchResult(const DispatchResult& result) {
  auto it = dispatch_tracking_.find(result.dispatch_id);
  if (it == dispatch_tracking_.end()) {
    throw std::invalid_argument("Unknown dispatch_id in pushDispatchResult");
  }

  it->second.status = result.status;
  if (!result.observed_updates.empty()) {
    pushState({result.observed_updates});
  }

  if (result.status == CommandStatus::FAILED_PERMANENT ||
      result.status == CommandStatus::FAILED_TRANSIENT ||
      result.status == CommandStatus::CANCELLED) {
    ++replan_count_;
    dispatch_tracking_.clear();
    pending_goal_dispatch_queue_.clear();
    state_ = (replan_count_ >= policy_.max_replans)
                 ? AutonomyBackendState::FAILED
                 : AutonomyBackendState::READY;
    return;
  }

  bool waiting = false;
  for (const auto& [dispatch_id, tracking] : dispatch_tracking_) {
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

  dispatch_tracking_.clear();
  state_ = goalsSatisfied() ? AutonomyBackendState::COMPLETE
                            : AutonomyBackendState::READY;
}

void CurrentAmeBackendAdapter::requestStop(StopMode mode) {
  if (mode == StopMode::IMMEDIATE) {
    resetTransientQueues();
    executor_.haltExecution();
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
  for (const auto& [dispatch_id, tracking] : dispatch_tracking_) {
    if (tracking.status == CommandStatus::PENDING ||
        tracking.status == CommandStatus::RUNNING) {
      snapshot.outstanding_goal_dispatches.push_back(tracking.dispatch);
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
  dispatch_tracking_.clear();
  pending_command_queue_.clear();
  pending_goal_dispatch_queue_.clear();
  pending_decision_records_.clear();
  pyramid_proxy_->reset(session_id_);
}

void CurrentAmeBackendAdapter::resetExecutionForReplan() {
  executor_.haltExecution();
  command_tracking_.clear();
  dispatch_tracking_.clear();
  pending_command_queue_.clear();
  pending_goal_dispatch_queue_.clear();
  pyramid_proxy_->reset(session_id_);
}

void CurrentAmeBackendAdapter::loadAndStartExecution(const std::string& bt_xml) {
  pyramid_proxy_->reset(session_id_);
  executor_.loadAndExecute(bt_xml);
}

bool CurrentAmeBackendAdapter::maybeEmitGoalDispatches() {
  if (!policy_.enable_goal_dispatch || world_model_.numAgents() == 0) {
    return false;
  }
  if (!dispatch_tracking_.empty() || !pending_goal_dispatch_queue_.empty()) {
    return true;
  }

  std::vector<std::string> goals;
  for (const auto goal_id : world_model_.goalFluentIds()) {
    goals.push_back(world_model_.fluentName(goal_id));
  }
  if (goals.empty()) {
    return false;
  }

  auto assignments = goal_allocator_.allocate(goals, world_model_);
  if (assignments.empty()) {
    return false;
  }

  for (const auto& assignment : assignments) {
    GoalDispatch dispatch;
    dispatch.dispatch_id = session_id_ + "/dispatch/" + std::to_string(++dispatch_counter_);
    dispatch.agent_id = assignment.agent_id;
    dispatch.goals = assignment.goals;
    dispatch_tracking_[dispatch.dispatch_id] = {dispatch, CommandStatus::PENDING};
    pending_goal_dispatch_queue_.push_back(dispatch);
  }
  return true;
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
