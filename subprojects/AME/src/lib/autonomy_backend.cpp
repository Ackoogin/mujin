#include "ame/current_ame_backend_adapter.h"

#include "ame/bt_nodes/invoke_service.h"
#include "ame/execution_sink.h"
#include "ame/pyramid_service.h"

#include <stdexcept>
#include <utility>

namespace ame {

class CurrentAmeBackendAdapter::BackendPyramidServiceProxy : public IPyramidService {
public:
  struct SubmissionRecord {
    ActionCommand command;
    ExecutionSubmission submission;
  };

  struct PendingCall {
    ActionCommand command;
    bool accepted = true;
    std::string rejection_reason;
  };

  BackendPyramidServiceProxy(std::string session_id,
                             std::shared_ptr<IExecutionSink> execution_sink)
      : session_id_(std::move(session_id)),
        execution_sink_(std::move(execution_sink)) {
    if (!execution_sink_) {
      execution_sink_ = std::make_shared<CommandQueueExecutionSink>();
    }
    execution_sink_->reset(session_id_);
  }

  void reset(std::string session_id) {
    session_id_ = std::move(session_id);
    next_request_id_ = 0;
    pending_calls_.clear();
    submitted_records_.clear();
    execution_sink_->reset(session_id_);
  }

  std::vector<SubmissionRecord> drainSubmissionRecords() {
    auto records = submitted_records_;
    submitted_records_.clear();
    return records;
  }

  std::vector<ActionCommand> drainEmittedCommands() {
    return execution_sink_->pullCommands();
  }

  void submitResult(const CommandResult& result) {
    auto it = pending_calls_.find(result.command_id);
    if (it == pending_calls_.end()) {
      throw std::invalid_argument("Unknown command_id in proxy result");
    }
    execution_sink_->pushResult(result);
  }

  bool hasPendingCalls() const {
    for (const auto& [command_id, call] : pending_calls_) {
      if (execution_sink_->isPending(command_id)) {
        return true;
      }
    }
    return false;
  }

  void forgetCompletedCalls() {
    for (auto it = pending_calls_.begin(); it != pending_calls_.end();) {
      if (!execution_sink_->isPending(it->first)) {
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
    auto submission = execution_sink_->submit(command);
    pending.accepted = submission.accepted;
    pending.rejection_reason = submission.rejection_reason;
    pending_calls_[command.command_id] = pending;
    submitted_records_.push_back({command, submission});
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

    if (!pending_it->second.accepted) {
      response.set("error", pending_it->second.rejection_reason);
      return AsyncCallStatus::FAILURE;
    }

    auto result = execution_sink_->resultFor(cmd_it->second);
    if (!result.has_value()) {
      return AsyncCallStatus::PENDING;
    }

    response.fields.clear();
    for (const auto& update : result->observed_updates) {
      response.set(update.key, update.value ? "true" : "false");
    }

    switch (result->status) {
      case CommandStatus::PENDING:
      case CommandStatus::RUNNING:
        return AsyncCallStatus::PENDING;
      case CommandStatus::SUCCEEDED:
        return AsyncCallStatus::SUCCESS;
      case CommandStatus::FAILED_TRANSIENT:
      case CommandStatus::FAILED_PERMANENT:
        return AsyncCallStatus::FAILURE;
      case CommandStatus::CANCELLED:
        return AsyncCallStatus::CANCELLED;
    }
    return AsyncCallStatus::FAILURE;
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
    execution_sink_->cancel(pending_it->second.command.command_id);
  }

private:
  std::string session_id_;
  std::shared_ptr<IExecutionSink> execution_sink_;
  uint64_t next_request_id_ = 0;
  std::unordered_map<uint64_t, std::string> request_id_to_command_;
  std::unordered_map<std::string, PendingCall> pending_calls_;
  std::vector<SubmissionRecord> submitted_records_;
};

CurrentAmeBackendAdapter::CurrentAmeBackendAdapter(
    WorldModel& world_model,
    const ActionRegistry& action_registry,
    const Planner& planner,
    const PlanCompiler& plan_compiler,
    std::shared_ptr<IExecutionSink> execution_sink)
    : world_model_(world_model),
      action_registry_(action_registry),
      planner_(planner),
      plan_compiler_(plan_compiler),
      execution_sink_(std::move(execution_sink)) {
  if (!execution_sink_) {
    execution_sink_ = std::make_shared<CommandQueueExecutionSink>();
  }
  pyramid_proxy_ =
      std::make_unique<BackendPyramidServiceProxy>("", execution_sink_);
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
  for (const auto& agent : request.available_agents) {
    world_model_.registerAgent(agent.agent_id, agent.agent_type);
    if (auto* wm_agent = world_model_.getAgent(agent.agent_id)) {
      wm_agent->available = agent.available;
    }
  }
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

  auto submitted_records = pyramid_proxy_->drainSubmissionRecords();
  for (const auto& record : submitted_records) {
    if (record.submission.accepted) {
      command_tracking_[record.command.command_id] =
          {record.command, CommandStatus::PENDING};
    }
  }

  auto emitted_commands = pyramid_proxy_->drainEmittedCommands();
  if (!emitted_commands.empty()) {
    for (const auto& command : emitted_commands) {
      if (command_tracking_.find(command.command_id) == command_tracking_.end()) {
        command_tracking_[command.command_id] =
            {command, CommandStatus::PENDING};
      }
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
    restoreDispatchedAgents();
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

  restoreDispatchedAgents();
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
  snapshot.agent_states = collectAgentStates();
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

std::vector<AgentState> CurrentAmeBackendAdapter::collectAgentStates() const {
  std::vector<AgentState> states;
  for (const auto& agent : world_model_.agents()) {
    states.push_back({agent.id, agent.type, agent.available});
  }
  return states;
}

void CurrentAmeBackendAdapter::resetTransientQueues() {
  restoreDispatchedAgents();
  command_tracking_.clear();
  dispatch_tracking_.clear();
  pending_command_queue_.clear();
  pending_goal_dispatch_queue_.clear();
  pending_decision_records_.clear();
  pyramid_proxy_->reset(session_id_);
}

void CurrentAmeBackendAdapter::resetExecutionForReplan() {
  executor_.haltExecution();
  restoreDispatchedAgents();
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
    if (auto* agent = world_model_.getAgent(dispatch.agent_id)) {
      agent->available = false;
    }
    dispatch_tracking_[dispatch.dispatch_id] = {dispatch, CommandStatus::PENDING};
    pending_goal_dispatch_queue_.push_back(dispatch);
  }
  return true;
}

void CurrentAmeBackendAdapter::restoreDispatchedAgents() {
  for (const auto& [dispatch_id, tracking] : dispatch_tracking_) {
    if (auto* agent = world_model_.getAgent(tracking.dispatch.agent_id)) {
      agent->available = true;
    }
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
