#include "ame/agra_ma_bridge.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ame {

namespace {

constexpr const char* kAgraSchemaVersion =
    "005.0a.ASK-20260423-f1380e7";

template <typename T>
std::vector<T> drain(std::vector<T>& values) {
  auto result = std::move(values);
  values.clear();
  return result;
}

std::string utcTimestamp(unsigned offset_seconds = 0) {
  const auto point = std::chrono::system_clock::now() +
                     std::chrono::seconds(offset_seconds);
  const auto time = std::chrono::system_clock::to_time_t(point);
  std::tm utc{};
#if defined(_WIN32)
  gmtime_s(&utc, &time);
#else
  gmtime_r(&time, &utc);
#endif
  std::ostringstream stream;
  stream << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

uint64_t fnv1a(const std::string& value, uint64_t seed) {
  uint64_t hash = seed;
  for (const unsigned char byte : value) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  return hash;
}

}  // namespace

AgraMaBridge::AgraMaBridge(IAutonomyBackend& backend,
                           AgraMaBridgeOptions options)
    : backend_(backend), options_(std::move(options)) {
  requireUuid(options_.system_uuid, "AgraMaBridgeOptions.system_uuid");
  if (options_.message_mode == agra::MessageModeEnum::Unspecified) {
    throw std::invalid_argument(
        "AgraMaBridgeOptions.message_mode must be explicit");
  }
  if (options_.backend_policy.require_plan_approval) {
    requireUuid(options_.approval_authority_system_uuid,
                "AgraMaBridgeOptions.approval_authority_system_uuid");
    if (options_.approval_timeout_seconds == 0) {
      throw std::invalid_argument(
          "AgraMaBridgeOptions.approval_timeout_seconds must be non-zero "
          "when plan approval is required");
    }
  }
}

std::string AgraMaBridge::deterministicUuid(const std::string& value) {
  if (value.empty()) {
    throw std::invalid_argument(
        "Cannot derive an A-GRA UUID from an empty value");
  }
  const std::array<uint64_t, 2> hashes = {
      fnv1a(value, 1469598103934665603ULL),
      fnv1a(value, 1099511628211ULL)};
  std::string uuid(16, '\0');
  for (size_t half = 0; half < hashes.size(); ++half) {
    for (size_t byte = 0; byte < 8; ++byte) {
      uuid[half * 8 + byte] = static_cast<char>(
          (hashes[half] >> ((7 - byte) * 8)) & 0xff);
    }
  }
  uuid[6] = static_cast<char>(
      (static_cast<unsigned char>(uuid[6]) & 0x0f) | 0x50);
  uuid[8] = static_cast<char>(
      (static_cast<unsigned char>(uuid[8]) & 0x3f) | 0x80);
  return uuid;
}

void AgraMaBridge::registerTaskGrounding(
    const agra::MA_TaskMT& task,
    const std::vector<std::string>& goal_fluents) {
  validateHeader(task.message_header, "MA_Task");
  if (!task.message_header.mission_id.has_value()) {
    throw std::invalid_argument(
        "MA_Task.message_header.mission_id is missing");
  }
  requireUuid(task.message_header.mission_id->base.uuid,
              "MA_Task.message_header.mission_id");
  const auto key = taskKey(task.message_data.task_id);
  if (goal_fluents.empty()) {
    throw std::invalid_argument(
        "MA_Task '" + uuidHex(key) +
        "' has no AME MissionIntent.goal_fluents grounding");
  }
  for (const auto& goal : goal_fluents) {
    if (goal.empty()) {
      throw std::invalid_argument(
          "MA_Task '" + uuidHex(key) +
          "' contains an empty AME goal fluent");
    }
  }

  // Carry the supplied task's marking rather than clearing it. Clearing looked
  // like refusing to assert a classification this system has no authority over,
  // but the result is not an unclassified message -- it is an unsendable one,
  // because the OMS-JSON codec rejects an unset classification outright.
  auto published_task = task;
  published_task.message_header =
      makeHeader(task.message_header.mission_id.value());
  task_groundings_[key] = {published_task, goal_fluents};

  agra_c2_provided::MA_Task_Service_Information information;
  information.ma_task = published_task;
  tasks_.push_back(std::move(information));
}

void AgraMaBridge::supplyPlanGrounding(
    const std::string& mission_plan_command_uuid,
    const std::vector<AgraPlanElementGrounding>& grounding) {
  const auto command_uuid =
      requireUuid(mission_plan_command_uuid,
                  "mission_plan_command_uuid");
  if (grounding.empty()) {
    throw std::invalid_argument(
        "Plan grounding for MA_MissionPlanCommand '" +
        uuidHex(command_uuid) + "' is empty");
  }
  for (const auto& element : grounding) {
    if (element.action_signature.empty()) {
      throw std::invalid_argument(
          "Plan grounding for MA_MissionPlanCommand '" +
          uuidHex(command_uuid) + "' has an empty action signature");
    }
  }
  plan_groundings_[command_uuid] = grounding;
}

std::vector<agra_c2_consumed::MA_ApprovalRequest_Service_Information>
AgraMaBridge::pullApprovalRequests() {
  return drain(approval_requests_);
}

const AgraPlanElementGrounding& AgraMaBridge::groundingForCommand(
    const ActionCommand& command) const {
  const auto plan_grounding = plan_groundings_.find(active_command_uuid_);
  if (plan_grounding == plan_groundings_.end()) {
    throw std::runtime_error(
        "Execution command '" + command.command_id +
        "' has no externally supplied plan grounding");
  }

  auto grounding = plan_grounding->second.end();
  size_t matches = 0;
  const auto operation_prefix = command.operation + "(";
  for (auto candidate = plan_grounding->second.begin();
       candidate != plan_grounding->second.end();
       ++candidate) {
    if (candidate->action_signature == command.signature ||
        candidate->action_signature == command.operation ||
        candidate->action_signature.compare(
            0, operation_prefix.size(), operation_prefix) == 0) {
      grounding = candidate;
      ++matches;
    }
  }
  if (matches == 0) {
    throw std::runtime_error(
        "Execution command '" + command.command_id +
        "' has no grounding for action signature '" +
        command.signature + "'");
  }
  if (matches != 1) {
    throw std::runtime_error(
        "Execution command '" + command.command_id +
        "' has ambiguous external groundings for operation '" +
        command.operation + "'");
  }
  return *grounding;
}

std::vector<ActionCommand> AgraMaBridge::pullCommands() {
  if (!activation_received_ &&
      options_.backend_policy.require_plan_approval) {
    return {};
  }
  auto commands = backend_.pullCommands();
  for (auto& command : commands) {
    const auto& grounding = groundingForCommand(command);
    for (const auto& parameter : grounding.command_parameters) {
      const auto existing = command.request_fields.find(parameter.first);
      if (existing != command.request_fields.end() &&
          existing->second != parameter.second) {
        throw std::runtime_error(
            "Execution command '" + command.command_id +
            "' has conflicting externally supplied parameter '" +
            parameter.first + "'");
      }
      command.request_fields[parameter.first] = parameter.second;
    }
  }
  return commands;
}

agra::CommandProcessingStateEnum AgraMaBridge::commandProcessingState(
    CommandStatus status) {
  switch (status) {
    case CommandStatus::PENDING:
      return agra::CommandProcessingStateEnum::Received;
    case CommandStatus::RUNNING:
    case CommandStatus::SUCCEEDED:
      return agra::CommandProcessingStateEnum::Accepted;
    case CommandStatus::FAILED_TRANSIENT:
    case CommandStatus::FAILED_PERMANENT:
      return agra::CommandProcessingStateEnum::Rejected;
    case CommandStatus::CANCELLED:
      return agra::CommandProcessingStateEnum::Canceled;
  }
  throw std::invalid_argument("Unsupported AME CommandStatus");
}

CommandStatus AgraMaBridge::commandStatus(
    agra::CommandProcessingStateEnum state) {
  switch (state) {
    case agra::CommandProcessingStateEnum::Received:
      return CommandStatus::PENDING;
    case agra::CommandProcessingStateEnum::Accepted:
      return CommandStatus::SUCCEEDED;
    case agra::CommandProcessingStateEnum::Rejected:
      return CommandStatus::FAILED_PERMANENT;
    case agra::CommandProcessingStateEnum::Canceled:
      return CommandStatus::CANCELLED;
    case agra::CommandProcessingStateEnum::Unspecified:
      break;
  }
  throw std::invalid_argument(
      "MA_ActionCommandStatus has unspecified command_processing_state");
}

agra_c2_provided::MA_ActionCommand_Service_Information
AgraMaBridge::actionCommandMessage(
    const ActionCommand& command,
    const agra::SecurityInformationType& security_information,
    const agra::HeaderType& header) {
  agra::MA_ActionCommandMT message;
  message.security_information = security_information;
  message.message_header = header;

  agra::MA_ActionCapabilityCommandType capability;
  capability.base.command_id.uuid =
      deterministicUuid(command.command_id);
  capability.base.command_id.descriptive_label = command.command_id;
  capability.base.command_state = agra::CommandStateEnum::New;
  capability.capability_id.uuid =
      deterministicUuid("action-registry-capability/" +
                        command.action_name);
  capability.capability_id.descriptive_label = command.action_name;
  capability.action_id.base.uuid = deterministicUuid(
      command.action_name + "\x1f" + command.signature);
  capability.action_id.base.descriptive_label =
      command.signature.empty() ? command.action_name
                                : command.signature;

  agra::MA_ActionCommandType item;
  item.capability = std::move(capability);
  message.message_data.command.push_back(std::move(item));

  agra_c2_provided::MA_ActionCommand_Service_Information information;
  information.ma_action_command = std::move(message);
  return information;
}

std::vector<agra_c2_provided::MA_ActionCommand_Service_Information>
AgraMaBridge::pullActionCommandMessages() {
  auto commands = pullCommands();
  std::vector<
      agra_c2_provided::MA_ActionCommand_Service_Information>
      messages;
  messages.reserve(commands.size());
  for (const auto& command : commands) {
    const auto& grounding = groundingForCommand(command);
    const auto command_uuid = deterministicUuid(command.command_id);
    const auto [existing, inserted] =
        backend_action_command_by_agra_.emplace(
            command_uuid, command.command_id);
    if (!inserted && existing->second != command.command_id) {
      throw std::runtime_error(
          "Deterministic MA_ActionCommand command_id collision");
    }
    messages.push_back(actionCommandMessage(
        command, grounding.action.security_information, makeHeader()));
  }
  return messages;
}

void AgraMaBridge::pushActionCommandStatus(
    const agra_c2_provided::
        MA_ActionCommandStatus_Service_Information& information) {
  if (!information.ma_action_command_status.has_value()) {
    throw std::invalid_argument(
        "MA_ActionCommandStatus ingress is missing its status payload");
  }
  const auto& status = information.ma_action_command_status.value();
  validateHeader(status.message_header, "MA_ActionCommandStatus");
  const auto command_uuid = requireUuid(
      status.message_data.base.command_id.uuid,
      "MA_ActionCommandStatus.message_data.base.command_id");
  const auto correlation =
      backend_action_command_by_agra_.find(command_uuid);
  if (correlation == backend_action_command_by_agra_.end()) {
    throw std::invalid_argument(
        "MA_ActionCommandStatus refused unknown or stale command_id '" +
        uuidHex(command_uuid) + "'");
  }

  CommandResult result;
  result.command_id = correlation->second;
  result.status = commandStatus(
      status.message_data.base.command_processing_state);
  pushCommandResult(result);
  if (result.status != CommandStatus::PENDING &&
      result.status != CommandStatus::RUNNING) {
    backend_action_command_by_agra_.erase(correlation);
  }
}

void AgraMaBridge::pushCommandResult(const CommandResult& result) {
  if (!activation_received_ &&
      options_.backend_policy.require_plan_approval) {
    throw std::logic_error(
        "Command result '" + result.command_id +
        "' refused before MA_MissionPlanActivationCommand");
  }
  backend_.pushCommandResult(result);
  backend_.step();
  collectDecisionRecords(active_command_uuid_);
  refreshExecutionStatus();
}

void AgraMaBridge::step() {
  if (!activation_received_ &&
      options_.backend_policy.require_plan_approval) {
    return;
  }
  backend_.step();
  collectDecisionRecords(active_command_uuid_);
  refreshExecutionStatus();
}

void AgraMaBridge::ingestApprovalStatus(
    const agra_c2_consumed::
        MA_ApprovalRequestStatus_Service_Information& information) {
  if (!information.ma_approval_request_status.has_value()) {
    throw std::invalid_argument(
        "MA_ApprovalRequestStatus ingress is missing its status payload");
  }
  const auto& status = information.ma_approval_request_status.value();
  validateHeader(status.message_header, "MA_ApprovalRequestStatus");
  const auto request_uuid =
      requireUuid(status.message_data.request_id.uuid,
                  "MA_ApprovalRequestStatus.message_data.request_id");
  const auto mapping = approval_plan_by_request_.find(request_uuid);
  if (mapping == approval_plan_by_request_.end()) {
    throw std::invalid_argument(
        "MA_ApprovalRequestStatus refused unknown or stale request_id '" +
        uuidHex(request_uuid) + "'");
  }
  const auto& plan_id = mapping->second;
  const auto& supplied_label =
      status.message_data.request_id.descriptive_label;
  if (supplied_label.empty()) {
    throw std::invalid_argument(
        "MA_ApprovalRequestStatus refused request_id '" +
        uuidHex(request_uuid) + "' without its correlated plan_id '" +
        plan_id + "'");
  }
  if (supplied_label != plan_id) {
    throw std::invalid_argument(
        "MA_ApprovalRequestStatus refused unknown or stale plan_id '" +
        supplied_label + "'; request_id is for plan_id '" + plan_id + "'");
  }

  switch (status.message_data.approval_request_processing_state) {
    case agra::ApprovalStatusEnum::Approved:
      backend_.approvePlan(plan_id);
      break;
    case agra::ApprovalStatusEnum::Rejected: {
      if (!status.message_data
               .approval_request_processing_state_reason.has_value() ||
          status.message_data.approval_request_processing_state_reason
              ->description.empty()) {
        throw std::invalid_argument(
            "MA_ApprovalRequestStatus rejection for plan_id '" + plan_id +
            "' has no reason");
      }
      backend_.rejectPlan(
          plan_id,
          status.message_data.approval_request_processing_state_reason
              ->description);
      current_backend_plan_id_.clear();
      current_agra_plan_uuid_.clear();
      activation_received_ = false;
      break;
    }
    default:
      throw std::invalid_argument(
          "MA_ApprovalRequestStatus for plan_id '" + plan_id +
          "' must be APPROVED or REJECTED");
  }

  approval_plan_by_request_.erase(mapping);
  refreshExecutionStatus();
}

void AgraMaBridge::onCommand(
    const agra_c2_provided::
        MA_MissionPlanCommand_Service_Information& information) {
  if (!information.ma_mission_plan_command.has_value()) {
    throw std::invalid_argument(
        "MA_MissionPlanCommand ingress is missing its command payload");
  }
  const auto& command = information.ma_mission_plan_command.value();
  validateHeader(command.message_header, "MA_MissionPlanCommand");
  const auto command_uuid =
      requireUuid(command.message_data.command_id.uuid,
                  "MA_MissionPlanCommand.message_data.command_id");
  if (command.message_data.command_state != agra::CommandStateEnum::New) {
    throw std::invalid_argument(
        "MA_MissionPlanCommand '" + uuidHex(command_uuid) +
        "' must have command_state NEW");
  }
  if (!command.message_data.inputs.proposed_requirements.has_value()) {
    throw std::invalid_argument(
        "MA_MissionPlanCommand '" + uuidHex(command_uuid) +
        "' is missing inputs.proposed_requirements");
  }
  const auto& proposed_tasks =
      command.message_data.inputs.proposed_requirements->proposed_task;
  if (proposed_tasks.size() != 1) {
    throw std::invalid_argument(
        "MA_MissionPlanCommand '" + uuidHex(command_uuid) +
        "' must reference exactly one MA_Task under G1");
  }
  const auto task_key = taskKey(proposed_tasks.front().task_id);
  const auto task = task_groundings_.find(task_key);
  if (task == task_groundings_.end()) {
    throw std::invalid_argument(
        "MA_MissionPlanCommand '" + uuidHex(command_uuid) +
        "' references MA_Task '" + uuidHex(task_key) +
        "' without a registered goal-fluent grounding");
  }
  if (plan_groundings_.find(command_uuid) == plan_groundings_.end()) {
    throw std::invalid_argument(
        "MA_MissionPlanCommand '" + uuidHex(command_uuid) +
        "' has no externally supplied symbolic/kinematic plan grounding");
  }

  SessionRequest session;
  session.session_id = uuidHex(command_uuid);
  session.intent.goal_fluents = task->second.goal_fluents;
  session.policy = options_.backend_policy;
  active_command_uuid_ = command_uuid;
  active_task_key_ = task_key;
  active_command_security_ = command.security_information;
  activation_received_ = false;
  current_backend_plan_id_.clear();
  current_agra_plan_uuid_.clear();
  projected_replan_count_ = 0;

  queueMissionPlanCommandStatus(
      command_uuid,
      agra::CommandProcessingStateEnum::Accepted,
      agra::ProcessingStatusEnum::Processing);
  backend_.start(session);
  try {
    backend_.step();
    collectDecisionRecords(command_uuid);
  } catch (const std::exception& error) {
    queueMissionPlanCommandStatus(
        command_uuid,
        agra::CommandProcessingStateEnum::Rejected,
        agra::ProcessingStatusEnum::Failed,
        {},
        error.what());
    backend_.requestStop(StopMode::IMMEDIATE);
    throw;
  } catch (...) {
    backend_.requestStop(StopMode::IMMEDIATE);
    throw;
  }
  refreshExecutionStatus();
}

std::vector<
    agra_c2_provided::MA_MissionPlanCommandStatus_Service_Information>
AgraMaBridge::handleMaMissionplancommandstatusRead(
    const agra_c2_provided::Empty&) {
  return drain(mission_plan_command_statuses_);
}

void AgraMaBridge::onCommand(
    const agra_c2_provided::
        MA_MissionPlanActivationCommand_Service_Information& information) {
  if (!information.ma_mission_plan_activation_command.has_value()) {
    throw std::invalid_argument(
        "MA_MissionPlanActivationCommand ingress is missing its command "
        "payload");
  }
  const auto& command =
      information.ma_mission_plan_activation_command.value();
  validateHeader(command.message_header,
                 "MA_MissionPlanActivationCommand");
  const auto command_uuid =
      requireUuid(command.message_data.command_id.uuid,
                  "MA_MissionPlanActivationCommand.message_data.command_id");
  if (command.message_data.command_state != agra::CommandStateEnum::New) {
    throw std::invalid_argument(
        "MA_MissionPlanActivationCommand '" + uuidHex(command_uuid) +
        "' must have command_state NEW");
  }
  if (command.message_data.command.size() != 1) {
    throw std::invalid_argument(
        "MA_MissionPlanActivationCommand '" + uuidHex(command_uuid) +
        "' must identify exactly one MissionPlan");
  }
  const auto& activation = command.message_data.command.front();
  const auto requested_plan_uuid = planKey(activation.mission_plan_id);
  const auto known_plan =
      backend_plan_by_agra_plan_.find(requested_plan_uuid);
  if (known_plan == backend_plan_by_agra_plan_.end() ||
      known_plan->second != current_backend_plan_id_) {
    throw std::invalid_argument(
        "MA_MissionPlanActivationCommand '" + uuidHex(command_uuid) +
        "' refused unknown or stale plan_id '" +
        activation.mission_plan_id.base.descriptive_label + "' (" +
        uuidHex(requested_plan_uuid) + ")");
  }
  if (!activation.activation_details.by_mission_plan.has_value() ||
      activation.activation_details.by_mission_plan->activation_command !=
          agra::PlanActivationCommandEnum::Activate) {
    throw std::invalid_argument(
        "MA_MissionPlanActivationCommand '" + uuidHex(command_uuid) +
        "' must request by_mission_plan ACTIVATE");
  }
  if (backend_.readSnapshot().state ==
      AutonomyBackendState::PENDING_APPROVAL) {
    const auto error =
        "Activation refused while plan_id '" + current_backend_plan_id_ +
        "' is PENDING_APPROVAL";
    queueActivationCommandStatus(
        command_uuid, command.security_information,
        agra::CommandProcessingStateEnum::Rejected,
        agra::ProcessingStatusEnum::Failed,
        error);
    throw std::logic_error(error);
  }

  activation_received_ = true;
  queueActivationCommandStatus(
      command_uuid, command.security_information,
      agra::CommandProcessingStateEnum::Accepted,
      agra::ProcessingStatusEnum::Completed);

  agra::MA_MissionPlanActivationStatusMT status;
  status.message_header = makeHeader();
  status.message_data.mission_plan_id = activation.mission_plan_id;
  status.message_data.plan_activation_state =
      agra::PlanActivationStateEnum::Activated;
  agra_c2_provided::
      MA_MissionPlanActivationStatus_Service_Information
          activation_information;
  activation_information.ma_mission_plan_activation_status =
      std::move(status);
  activation_statuses_.push_back(std::move(activation_information));

  backend_.step();
  collectDecisionRecords(active_command_uuid_);
  refreshExecutionStatus();
}

std::vector<
    agra_c2_provided::
        MA_MissionPlanActivationCommandStatus_Service_Information>
AgraMaBridge::handleMaMissionplanactivationcommandstatusRead(
    const agra_c2_provided::Empty&) {
  return drain(activation_command_statuses_);
}

void AgraMaBridge::onCommand(
    const agra_c2_provided::MA_TaskCommand_Service_Information&
        information) {
  if (!information.ma_task_command.has_value()) {
    throw std::invalid_argument(
        "MA_TaskCommand ingress is missing its command payload");
  }
  const auto& message = information.ma_task_command.value();
  validateHeader(message.message_header, "MA_TaskCommand");
  if (message.message_data.command.size() != 1) {
    throw std::invalid_argument(
        "MA_TaskCommand must contain exactly one command under G1");
  }
  const auto& command = message.message_data.command.front().capability;
  const auto& command_id = command.base.command_id;
  const auto command_uuid =
      requireUuid(command_id.uuid,
                  "MA_TaskCommand.command.capability.command_id");
  if (command.base.command_state != agra::CommandStateEnum::New) {
    throw std::invalid_argument(
        "MA_TaskCommand '" + uuidHex(command_uuid) +
        "' must have command_state NEW");
  }
  const auto task_key = taskKey(command.task_id);
  if (task_groundings_.find(task_key) == task_groundings_.end()) {
    const auto error =
        "MA_TaskCommand '" + uuidHex(command_uuid) +
        "' references MA_Task '" + uuidHex(task_key) +
        "' without a registered goal-fluent grounding";
    queueTaskCommandStatus(
        command_id, message.security_information,
        agra::CommandProcessingStateEnum::Rejected, error);
    throw std::invalid_argument(error);
  }
  active_task_key_ = task_key;
  queueTaskCommandStatus(
      command_id, message.security_information,
      agra::CommandProcessingStateEnum::Accepted);
}

std::vector<agra_c2_provided::MA_TaskCommandStatus_Service_Information>
AgraMaBridge::handleMaTaskcommandstatusRead(
    const agra_c2_provided::Empty&) {
  return drain(task_command_statuses_);
}

std::vector<agra_c2_provided::MA_Action_Service_Information>
AgraMaBridge::handleMaActionRead(const agra_c2_provided::Empty&) {
  return drain(actions_);
}

std::vector<agra_c2_provided::MA_ActionPlan_Service_Information>
AgraMaBridge::handleMaActionplanRead(const agra_c2_provided::Empty&) {
  return drain(action_plans_);
}

std::vector<agra_c2_provided::MA_RoutePlan_Service_Information>
AgraMaBridge::handleMaRouteplanRead(const agra_c2_provided::Empty&) {
  return drain(route_plans_);
}

std::vector<agra_c2_provided::MA_MissionPlan_Service_Information>
AgraMaBridge::handleMaMissionplanRead(
    const agra_c2_provided::Empty&) {
  return drain(mission_plans_);
}

std::vector<agra_c2_provided::MA_PlanningFunction_Service_Information>
AgraMaBridge::handleMaPlanningfunctionRead(
    const agra_c2_provided::Empty&) {
  const auto capabilities = backend_.describeCapabilities();
  agra::MA_PlanningFunctionMT function;
  function.message_header = makeHeader();
  function.object_state = agra::ObjectStateEnum::New;
  function.message_data.planning_function_id.uuid =
      deterministicUuid(capabilities.backend_id + "/planning-function");
  function.message_data.planning_function_id.descriptive_label =
      capabilities.backend_id;
  function.message_data.system_id.uuid = options_.system_uuid;
  function.message_data.system_id.descriptive_label =
      capabilities.backend_id;

  agra::PlanningInterfacesType interface;
  interface.plan_type = agra::InterfacePlanTypeEnum::MissionPlan;
  if (capabilities.supports_batch_planning) {
    interface.plan_interface.push_back(
        agra::PlanningInterfaceEnum::PlanCommand);
  }
  if (capabilities.supports_requirement_management) {
    interface.plan_interface.push_back(
        agra::PlanningInterfaceEnum::RequirementManagement);
  }
  if (capabilities.supports_plan_validation) {
    interface.plan_interface.push_back(
        agra::PlanningInterfaceEnum::PlanValidation);
  }
  interface.plan_simultaneity = agra::PlanSimultaneityEnum::Single;
  function.message_data.planning_interfaces.push_back(interface);

  agra::MA_PlanningInterfaceDetailsType details;
  agra::MA_MissionPlanProcessType processes;
  const auto process_uuid =
      deterministicUuid(capabilities.backend_id + "/mission-plan-process");
  processes.default_planning_process_id.uuid = process_uuid;
  agra::MA_MissionPlanProcessDescriptionType process;
  process.planning_process_id.uuid = process_uuid;
  if (capabilities.supports_batch_planning) {
    process.validation_supported.push_back(
        agra::SupportedFunctionEnum::Planning);
  }
  if (capabilities.supports_plan_validation) {
    process.validation_supported.push_back(
        agra::SupportedFunctionEnum::Validation);
  }
  process.output.action_plan = agra::ActionPlanPartsType{};
  processes.process.push_back(std::move(process));
  details.mission_plan = std::move(processes);
  function.message_data.planning_interface_details =
      std::move(details);

  agra_c2_provided::MA_PlanningFunction_Service_Information information;
  information.ma_planning_function = std::move(function);
  return {std::move(information)};
}

std::vector<agra_c2_provided::MA_Task_Service_Information>
AgraMaBridge::handleMaTaskRead(const agra_c2_provided::Empty&) {
  return drain(tasks_);
}

std::vector<
    agra_c2_provided::
        MA_MissionPlanActivationStatus_Service_Information>
AgraMaBridge::handleMaMissionplanactivationstatusRead(
    const agra_c2_provided::Empty&) {
  return drain(activation_statuses_);
}

std::vector<
    agra_c2_provided::
        MA_MissionPlanExecutionStatus_Service_Information>
AgraMaBridge::handleMaMissionplanexecutionstatusRead(
    const agra_c2_provided::Empty&) {
  return drain(execution_statuses_);
}

std::vector<agra_c2_provided::MA_TaskStatus_Service_Information>
AgraMaBridge::handleMaTaskstatusRead(
    const agra_c2_provided::Empty&) {
  return drain(task_statuses_);
}

std::vector<
    agra_c2_provided::MissionContingencyAlert_Service_Information>
AgraMaBridge::handleMissioncontingencyalertRead(
    const agra_c2_provided::Empty&) {
  return drain(contingency_alerts_);
}

std::string AgraMaBridge::requireUuid(const std::string& uuid,
                                      const std::string& field_name) {
  if (uuid.size() != 16) {
    throw std::invalid_argument(
        field_name + " must contain exactly 16 UUID bytes; received " +
        std::to_string(uuid.size()));
  }
  return uuid;
}

std::string AgraMaBridge::uuidHex(const std::string& uuid) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const unsigned char byte : uuid) {
    stream << std::setw(2) << static_cast<unsigned>(byte);
  }
  return stream.str();
}

agra::CannotComplyType AgraMaBridge::cannotComply(
    agra::CannotComplyEnum reason,
    const std::string& description) {
  agra::CannotComplyType result;
  result.reason = reason;
  result.description = description;
  return result;
}

agra::RequirementExecutionStateEnum AgraMaBridge::requirementState(
    AutonomyBackendState state) {
  switch (state) {
    case AutonomyBackendState::IDLE:
    case AutonomyBackendState::READY:
      return agra::RequirementExecutionStateEnum::AwaitingPlanActivation;
    case AutonomyBackendState::PENDING_APPROVAL:
      return agra::RequirementExecutionStateEnum::AwaitingExecutionApproval;
    case AutonomyBackendState::EXECUTING:
    case AutonomyBackendState::WAITING_FOR_RESULTS:
      return agra::RequirementExecutionStateEnum::Executing;
    case AutonomyBackendState::COMPLETE:
      return agra::RequirementExecutionStateEnum::Completed;
    case AutonomyBackendState::FAILED:
      return agra::RequirementExecutionStateEnum::Failed;
    case AutonomyBackendState::STOPPED:
      return agra::RequirementExecutionStateEnum::Canceled;
  }
  throw std::runtime_error("Unknown AutonomyBackendState");
}

agra::PlanExecutionStateEnum AgraMaBridge::planExecutionState(
    AutonomyBackendState state) {
  switch (state) {
    case AutonomyBackendState::IDLE:
    case AutonomyBackendState::READY:
    case AutonomyBackendState::PENDING_APPROVAL:
      return agra::PlanExecutionStateEnum::Pending;
    case AutonomyBackendState::EXECUTING:
    case AutonomyBackendState::WAITING_FOR_RESULTS:
      return agra::PlanExecutionStateEnum::Executing;
    case AutonomyBackendState::COMPLETE:
      return agra::PlanExecutionStateEnum::Complete;
    case AutonomyBackendState::FAILED:
      return agra::PlanExecutionStateEnum::Failed;
    case AutonomyBackendState::STOPPED:
      return agra::PlanExecutionStateEnum::Canceled;
  }
  throw std::runtime_error("Unknown AutonomyBackendState");
}

agra::HeaderType AgraMaBridge::makeHeader() const {
  agra::HeaderType header;
  header.system_id.uuid = options_.system_uuid;
  header.system_id.descriptive_label =
      backend_.describeCapabilities().backend_id;
  header.timestamp = utcTimestamp();
  header.schema_version = kAgraSchemaVersion;
  header.mode = options_.message_mode;
  return header;
}

agra::HeaderType AgraMaBridge::makeHeader(
    const agra::MissionID_Type& mission_id) const {
  requireUuid(mission_id.base.uuid, "A-GRA mission_id");
  auto header = makeHeader();
  header.mission_id = mission_id;
  return header;
}

void AgraMaBridge::validateHeader(const agra::HeaderType& header,
                                  const std::string& message_name) const {
  requireUuid(header.system_id.uuid,
              message_name + ".message_header.system_id");
  if (header.timestamp.empty()) {
    throw std::invalid_argument(
        message_name + ".message_header.timestamp is missing");
  }
  if (header.schema_version != kAgraSchemaVersion) {
    throw std::invalid_argument(
        message_name + ".message_header.schema_version '" +
        header.schema_version + "' does not match '" +
        kAgraSchemaVersion + "'");
  }
  if (header.mode == agra::MessageModeEnum::Unspecified) {
    throw std::invalid_argument(
        message_name + ".message_header.mode is UNSPECIFIED");
  }
}

std::string AgraMaBridge::taskKey(
    const agra::TaskID_Type& task_id) const {
  return requireUuid(task_id.base.uuid, "MA_Task.task_id");
}

std::string AgraMaBridge::planKey(
    const agra::MissionPlanID_Type& plan_id) const {
  return requireUuid(plan_id.base.uuid, "MA_MissionPlan.mission_plan_id");
}

agra::MissionPlanID_Type AgraMaBridge::missionPlanId(
    const std::string& backend_plan_id) const {
  agra::MissionPlanID_Type result;
  result.base.uuid = deterministicUuid(backend_plan_id);
  result.base.descriptive_label = backend_plan_id;
  result.version = static_cast<uint32_t>(projected_replan_count_ + 1);
  return result;
}

void AgraMaBridge::collectDecisionRecords(
    const std::string& command_uuid) {
  for (const auto& record : backend_.pullDecisionRecords()) {
    if (!record.plan_success) {
      queueMissionPlanCommandStatus(
          command_uuid,
          agra::CommandProcessingStateEnum::Accepted,
          agra::ProcessingStatusEnum::Failed,
          record.plan_id,
          "AME planning failed for plan_id '" + record.plan_id + "'");
      continue;
    }
    projectPlan(record, command_uuid);
    queueMissionPlanCommandStatus(
        command_uuid,
        agra::CommandProcessingStateEnum::Accepted,
        agra::ProcessingStatusEnum::Completed,
        record.plan_id);

    if (backend_.readSnapshot().state ==
        AutonomyBackendState::PENDING_APPROVAL) {
      const auto task = task_groundings_.find(active_task_key_);
      if (task == task_groundings_.end() ||
          !task->second.task.message_header.mission_id.has_value()) {
        throw std::runtime_error(
            "MA_ApprovalRequest producer has no active A-GRA mission_id");
      }
      const auto request_uuid =
          deterministicUuid(record.plan_id + "/approval-request");
      agra::MA_ApprovalRequestMT request;
      request.security_information = active_command_security_;
      request.message_header = makeHeader(
          task->second.task.message_header.mission_id.value());
      request.message_data.request_id.uuid = request_uuid;
      request.message_data.request_id.descriptive_label = record.plan_id;
      request.message_data.request_state = agra::RequestStateEnum::New;
      agra::SystemServiceType approver;
      approver.system_id.uuid =
          options_.approval_authority_system_uuid;
      request.message_data.approver.non_operator_identifier =
          std::move(approver);
      request.message_data.approval_references.approval_policy_id.uuid =
          deterministicUuid(
              backend_.describeCapabilities().backend_id +
              "/plan-approval-policy");
      agra::PlanReferenceID_ChoiceType plan_reference;
      plan_reference.mission_plan_id = missionPlanId(record.plan_id);
      request.message_data.approval_references.approval_item
          .plan_approval = std::move(plan_reference);
      request.message_data.respond_by =
          utcTimestamp(options_.approval_timeout_seconds);
      approval_plan_by_request_[request_uuid] = record.plan_id;
      agra_c2_consumed::MA_ApprovalRequest_Service_Information
          information;
      information.ma_approval_request = std::move(request);
      approval_requests_.push_back(std::move(information));
    }
  }
}

void AgraMaBridge::projectPlan(const DecisionRecord& record,
                               const std::string& command_uuid) {
  if (record.plan_id.empty()) {
    throw std::runtime_error(
        "DecisionRecord for MA_MissionPlanCommand '" +
        uuidHex(command_uuid) + "' has no plan_id");
  }
  const auto groundings = plan_groundings_.find(command_uuid);
  if (groundings == plan_groundings_.end()) {
    throw std::runtime_error(
        "Plan projection for plan_id '" + record.plan_id +
        "' has no external grounding");
  }

  std::vector<agra::ActionID_Type> action_ids;
  std::vector<agra::RoutePlanID_Type> route_plan_ids;
  std::vector<agra_c2_provided::MA_Action_Service_Information>
      projected_actions;
  std::vector<agra_c2_provided::MA_RoutePlan_Service_Information>
      projected_route_plans;
  std::vector<bool> used(groundings->second.size(), false);
  for (const auto& signature : record.planned_action_signatures) {
    size_t match = groundings->second.size();
    for (size_t index = 0; index < groundings->second.size(); ++index) {
      if (!used[index] &&
          groundings->second[index].action_signature == signature) {
        match = index;
        break;
      }
    }
    if (match == groundings->second.size()) {
      throw std::runtime_error(
          "Plan projection for plan_id '" + record.plan_id +
          "' is missing grounding for action signature '" + signature +
          "'");
    }
    used[match] = true;
    const auto& grounding = groundings->second[match];
    const auto action_uuid = requireUuid(
        grounding.action.message_data.action_id.base.uuid,
        "AgraPlanElementGrounding.action.message_data.action_id");
    if (grounding.action.message_data.action_type ==
        agra::ActionTypeEnum::Unspecified) {
      throw std::runtime_error(
          "Plan projection for action signature '" + signature +
          "' is missing MA_Action.action_type");
    }

    if (grounding.requires_kinematics) {
      if (grounding.route_plan_id.empty()) {
        throw std::runtime_error(
            "Plan projection for action signature '" + signature +
            "' requires kinematics but route_plan_id was not supplied");
      }
      if (grounding.waypoints.size() < 2) {
        throw std::runtime_error(
            "Plan projection for action signature '" + signature +
            "' requires kinematics but fewer than two waypoints were "
            "supplied");
      }
      if (grounding.command_parameters.empty()) {
        throw std::runtime_error(
            "Plan projection for action signature '" + signature +
            "' requires kinematics but last_command_params were not "
            "supplied");
      }
      for (const auto& parameter : grounding.command_parameters) {
        if (parameter.first.empty() || parameter.second.empty()) {
          throw std::runtime_error(
              "Plan projection for action signature '" + signature +
              "' contains an empty externally supplied command parameter");
        }
      }

      agra::MA_RoutePlanMT route_plan;
      route_plan.message_header = makeHeader();
      route_plan.object_state = agra::ObjectStateEnum::New;
      route_plan.message_data.route_plan_id.base.uuid =
          deterministicUuid(grounding.route_plan_id);
      route_plan.message_data.route_plan_id.base.descriptive_label =
          grounding.route_plan_id;
      route_plan.message_data.route_plan_id.version =
          static_cast<uint32_t>(record.replan_count + 1);
      agra::RoutePlanCommandID_ChoiceType route_command;
      agra::MissionPlanCommandID_Type route_command_id;
      route_command_id.uuid = command_uuid;
      route_command.mission_plan_command_id = route_command_id;
      route_plan.message_data.plan_command_id = route_command;

      agra::SystemID_Type route_system;
      route_system.uuid = options_.system_uuid;
      route_system.descriptive_label =
          backend_.describeCapabilities().backend_id;
      route_plan.message_data.plan.applicability.planned_for_id =
          route_system;
      route_plan.message_data.plan.applicability.applicable_to_i_ds
          .system_id = route_system;
      route_plan.message_data.plan.parts.route_type.push_back(
          agra::PathTypeEnum::Primary);
      route_plan.message_data.plan.route.detailed = false;
      route_plan.message_data.plan.route.route_projection =
          agra::LineProjectionEnum::GreatCircle;

      agra::MA_RoutePathType path;
      path.path_id.base.uuid =
          deterministicUuid(grounding.route_plan_id + "/path/0");
      path.path_id.version =
          static_cast<uint32_t>(record.replan_count + 1);
      path.path_type = agra::MA_PathTypeEnum::Primary;
      path.first_in_path_segment_id.base.uuid =
          deterministicUuid(grounding.route_plan_id + "/segment/0");
      path.first_in_path_segment_id.version =
          static_cast<uint32_t>(record.replan_count + 1);
      route_plan.message_data.plan.route.first_in_route_path_id =
          path.path_id;

      constexpr double kDegreesToRadians =
          3.14159265358979323846 / 180.0;
      for (size_t waypoint_index = 0;
           waypoint_index < grounding.waypoints.size();
           ++waypoint_index) {
        const auto& supplied = grounding.waypoints[waypoint_index];
        if (!std::isfinite(supplied.latitude_deg) ||
            !std::isfinite(supplied.longitude_deg) ||
            !std::isfinite(supplied.altitude_m) ||
            supplied.latitude_deg < -90.0 ||
            supplied.latitude_deg > 90.0 ||
            supplied.longitude_deg < -180.0 ||
            supplied.longitude_deg > 180.0) {
          throw std::runtime_error(
              "Plan projection for action signature '" + signature +
              "' contains invalid externally supplied waypoint " +
              std::to_string(waypoint_index));
        }
        agra::MA_PathSegmentType segment;
        segment.path_segment_id.base.uuid = deterministicUuid(
            grounding.route_plan_id + "/segment/" +
            std::to_string(waypoint_index));
        segment.path_segment_id.version =
            static_cast<uint32_t>(record.replan_count + 1);
        segment.source = agra::PathSegmentSourceEnum::OperatorDefined;
        agra::WayPointType waypoint;
        agra::Point2D_Type point;
        point.latitude = supplied.latitude_deg * kDegreesToRadians;
        point.longitude = supplied.longitude_deg * kDegreesToRadians;
        point.altitude = supplied.altitude_m;
        point.altitude_reference = agra::AltitudeReferenceEnum::WgsHae;
        waypoint.point_choice.point2_d = point;
        waypoint.waypoint_type =
            waypoint_index + 1 == grounding.waypoints.size()
                ? agra::WaypointTypeEnum::EndOfPath
                : agra::WaypointTypeEnum::NavOnly;
        segment.end_point.way_point = waypoint;
        if (waypoint_index + 1 < grounding.waypoints.size()) {
          agra::NextPathSegmentType next;
          next.path_segment_id.base.uuid = deterministicUuid(
              grounding.route_plan_id + "/segment/" +
              std::to_string(waypoint_index + 1));
          next.path_segment_id.version =
              static_cast<uint32_t>(record.replan_count + 1);
          segment.next_path_segment = next;
        }
        path.path_segment.push_back(std::move(segment));
      }
      route_plan.message_data.plan.route.path.push_back(std::move(path));
      route_plan_ids.push_back(route_plan.message_data.route_plan_id);
      agra_c2_provided::MA_RoutePlan_Service_Information
          route_information;
      route_information.ma_route_plan = std::move(route_plan);
      projected_route_plans.push_back(std::move(route_information));
    } else if (!grounding.route_plan_id.empty() ||
               !grounding.waypoints.empty() ||
               !grounding.command_parameters.empty()) {
      throw std::runtime_error(
          "Plan projection for symbolic action '" + signature +
          "' received kinematic/command data without declaring that "
          "kinematics are required");
    }

    // As with the republished MA_Task: keep the marking the grounding supplied,
    // so the projected action can actually be encoded.
    auto action = grounding.action;
    action.message_header = makeHeader();
    action.object_state = agra::ObjectStateEnum::New;
    action.message_data.action_id.base.uuid = action_uuid;
    agra::RequirementMetadataType metadata =
        action.message_data.metadata.value_or(
            agra::RequirementMetadataType{});
    agra::TraceabilityType traceability =
        metadata.traceability.value_or(agra::TraceabilityType{});
    agra::RequirementInstanceID_ChoiceType task_reference;
    task_reference.task_id =
        task_groundings_.at(active_task_key_).task.message_data.task_id;
    traceability.requirement.push_back(std::move(task_reference));
    metadata.traceability = std::move(traceability);
    action.message_data.metadata = std::move(metadata);
    action_ids.push_back(action.message_data.action_id);
    agra_c2_provided::MA_Action_Service_Information information;
    information.ma_action = std::move(action);
    projected_actions.push_back(std::move(information));
  }

  for (size_t index = 0; index < used.size(); ++index) {
    if (!used[index]) {
      throw std::runtime_error(
          "Plan projection for plan_id '" + record.plan_id +
          "' contains unused external grounding for action signature '" +
          groundings->second[index].action_signature + "'");
    }
  }

  agra::SystemID_Type system;
  system.uuid = options_.system_uuid;
  system.descriptive_label =
      backend_.describeCapabilities().backend_id;

  agra::MA_ActionPlanMT action_plan;
  action_plan.message_header = makeHeader();
  action_plan.object_state = agra::ObjectStateEnum::New;
  action_plan.message_data.action_plan_id.base.uuid =
      deterministicUuid(record.plan_id + "/action-plan");
  action_plan.message_data.action_plan_id.base.descriptive_label =
      record.plan_id + "/action-plan";
  action_plan.message_data.action_plan_id.version =
      static_cast<uint32_t>(record.replan_count + 1);
  agra::ActionPlanCommandID_ChoiceType action_command;
  agra::MissionPlanCommandID_Type action_command_id;
  action_command_id.uuid = command_uuid;
  action_command.mission_plan_command_id = action_command_id;
  action_plan.message_data.plan_command_id = action_command;
  action_plan.message_data.plan.applicability.planned_for_id = system;
  action_plan.message_data.plan.applicability.applicable_to_i_ds.system_id =
      system;
  agra::ActionPlanPartsType action_parts;
  for (const auto& grounding : groundings->second) {
    action_parts.action_plan.push_back(
        grounding.action.message_data.action_type);
  }
  action_plan.message_data.plan.parts = action_parts;
  for (const auto& action_id : action_ids) {
    agra::ActionAllocationType allocation;
    allocation.action_id = action_id;
    action_plan.message_data.plan.allocated_action.push_back(
        std::move(allocation));
  }
  const auto action_plan_id = action_plan.message_data.action_plan_id;
  agra_c2_provided::MA_ActionPlan_Service_Information
      action_plan_information;
  action_plan_information.ma_action_plan = std::move(action_plan);

  agra::MA_MissionPlanMT plan;
  plan.security_information = active_command_security_;
  plan.message_header = makeHeader();
  plan.object_state = agra::ObjectStateEnum::New;
  plan.message_data.mission_plan_id = missionPlanId(record.plan_id);
  agra::MissionPlanCommandID_ChoiceType originating_command;
  agra::MissionPlanCommandID_Type command_id;
  command_id.uuid = command_uuid;
  originating_command.mission_plan_command_id = command_id;
  plan.message_data.mission_plan_command_id =
      std::move(originating_command);
  plan.message_data.applicability.planned_for_id = system;
  plan.message_data.applicability.applicable_to_i_ds.system_id =
      system;
  agra::MA_PlansReferenceBaseType sub_plans;
  sub_plans.action_plan_id.push_back(action_plan_id);
  sub_plans.route_plan_id = route_plan_ids;
  plan.message_data.sub_plans = std::move(sub_plans);
  if (!route_plan_ids.empty()) {
    agra::ExecutionSequenceType execution_sequence;
    for (size_t index = 0; index < route_plan_ids.size(); ++index) {
      agra::RouteExecutionPlanSetType plan_set;
      plan_set.execution_plan_set_id.uuid = deterministicUuid(
          record.plan_id + "/execution-plan-set/" +
          std::to_string(index));
      plan_set.execution_plan_set_id.descriptive_label =
          record.plan_id + "/execution-plan-set/" +
          std::to_string(index);
      plan_set.route_plan_id = route_plan_ids[index];
      if (index + 1 < route_plan_ids.size()) {
        agra::ExecutionPlanSetID_Type next_id;
        next_id.uuid = deterministicUuid(
            record.plan_id + "/execution-plan-set/" +
            std::to_string(index + 1));
        next_id.descriptive_label =
            record.plan_id + "/execution-plan-set/" +
            std::to_string(index + 1);
        plan_set.next_execution_plan_set_id = next_id;
      }
      execution_sequence.route_execution_plan_set.push_back(
          std::move(plan_set));
    }
    execution_sequence.initial_execution_plan_set_id =
        execution_sequence.route_execution_plan_set.front()
            .execution_plan_set_id;
    plan.message_data.execution_sequence =
        std::move(execution_sequence);
  }
  plan.message_data.for_planning_use_only = false;

  if (record.replan_count > projected_replan_count_) {
    agra::MissionContingencyAlertMT alert;
    alert.security_information = active_command_security_;
    alert.message_header = makeHeader();
    alert.object_state = agra::ObjectStateEnum::New;
    alert.message_data.mission_contingency_alert_id.uuid =
        deterministicUuid(record.plan_id + "/contingency");
    alert.message_data.mission_contingency_alert_id.descriptive_label =
        record.plan_id;
    alert.message_data.source_system_id = system;
    agra::MissionContingencyConditionType condition;
    condition.conflicted_system_id = system;
    condition.conflict_state = agra::ConflictStateEnum::Conflicted;
    alert.message_data.contingency_condition.push_back(
        std::move(condition));
    agra_c2_provided::MissionContingencyAlert_Service_Information
        alert_information;
    alert_information.mission_contingency_alert = std::move(alert);
    contingency_alerts_.push_back(std::move(alert_information));
  }

  current_backend_plan_id_ = record.plan_id;
  current_agra_plan_uuid_ =
      plan.message_data.mission_plan_id.base.uuid;
  backend_plan_by_agra_plan_[current_agra_plan_uuid_] =
      current_backend_plan_id_;
  projected_replan_count_ = record.replan_count;

  agra_c2_provided::MA_MissionPlan_Service_Information information;
  information.ma_mission_plan = std::move(plan);
  actions_.insert(
      actions_.end(),
      std::make_move_iterator(projected_actions.begin()),
      std::make_move_iterator(projected_actions.end()));
  route_plans_.insert(
      route_plans_.end(),
      std::make_move_iterator(projected_route_plans.begin()),
      std::make_move_iterator(projected_route_plans.end()));
  action_plans_.push_back(std::move(action_plan_information));
  mission_plans_.push_back(std::move(information));
}

void AgraMaBridge::queueMissionPlanCommandStatus(
    const std::string& command_uuid,
    agra::CommandProcessingStateEnum processing_state,
    agra::ProcessingStatusEnum status,
    const std::string& plan_id,
    const std::string& error) {
  agra::MA_MissionPlanCommandStatusMT message;
  message.security_information = active_command_security_;
  message.message_header = makeHeader();
  message.message_data.command_id.uuid = command_uuid;
  message.message_data.planning_status.command_processing_state =
      processing_state;
  message.message_data.planning_status.command_status = status;
  if (!error.empty()) {
    message.message_data.planning_status
        .command_processing_state_reason = cannotComply(
            agra::CannotComplyEnum::Other, error);
  }
  if (!plan_id.empty()) {
    agra::MA_PlansReferenceType plans;
    plans.mission_plan_id.push_back(missionPlanId(plan_id));
    message.message_data.resulting_plan_identifier =
        std::move(plans);
  }
  agra_c2_provided::
      MA_MissionPlanCommandStatus_Service_Information entity;
  entity.ma_mission_plan_command_status = std::move(message);
  mission_plan_command_statuses_.push_back(std::move(entity));
}

void AgraMaBridge::queueTaskCommandStatus(
    const agra::CommandID_Type& command_id,
    const agra::SecurityInformationType& security_information,
    agra::CommandProcessingStateEnum processing_state,
    const std::string& error) {
  agra::MA_TaskCommandStatusMT message;
  message.security_information = security_information;
  message.message_header = makeHeader();
  message.message_data.base.command_id = command_id;
  message.message_data.base.command_processing_state = processing_state;
  if (!error.empty()) {
    message.message_data.base.command_processing_state_reason =
        cannotComply(agra::CannotComplyEnum::Other, error);
  }
  agra_c2_provided::MA_TaskCommandStatus_Service_Information
      information;
  information.ma_task_command_status = std::move(message);
  task_command_statuses_.push_back(std::move(information));
}

void AgraMaBridge::queueActivationCommandStatus(
    const std::string& command_uuid,
    const agra::SecurityInformationType& security_information,
    agra::CommandProcessingStateEnum processing_state,
    agra::ProcessingStatusEnum status,
    const std::string& error) {
  agra::MA_MissionPlanActivationCommandStatusMT message;
  message.security_information = security_information;
  message.message_header = makeHeader();
  message.message_data.command_id.uuid = command_uuid;
  message.message_data.activation_status.command_processing_state =
      processing_state;
  message.message_data.activation_status.command_status = status;
  if (!error.empty()) {
    message.message_data.activation_status
        .command_processing_state_reason = cannotComply(
            agra::CannotComplyEnum::Other, error);
  }
  agra_c2_provided::
      MA_MissionPlanActivationCommandStatus_Service_Information entity;
  entity.ma_mission_plan_activation_command_status =
      std::move(message);
  activation_command_statuses_.push_back(std::move(entity));
}

void AgraMaBridge::refreshExecutionStatus() {
  if (current_backend_plan_id_.empty() ||
      current_agra_plan_uuid_.empty()) {
    return;
  }
  const auto snapshot = backend_.readSnapshot();

  agra::MA_MissionPlanExecutionStatusMT status;
  status.security_information = active_command_security_;
  status.message_header = makeHeader();
  status.message_data.system_id.uuid = options_.system_uuid;
  status.message_data.system_id.descriptive_label =
      backend_.describeCapabilities().backend_id;
  status.message_data.source = agra::SystemSourceEnum::Actual;
  agra::MA_MissionPlanExecutionStateType plan_status;
  plan_status.mission_plan_id = missionPlanId(current_backend_plan_id_);
  plan_status.plan_execution_state =
      planExecutionState(snapshot.state);
  status.message_data.plan_execution_status.push_back(
      std::move(plan_status));
  agra_c2_provided::
      MA_MissionPlanExecutionStatus_Service_Information information;
  information.ma_mission_plan_execution_status = std::move(status);
  execution_statuses_.push_back(std::move(information));

  const auto task = task_groundings_.find(active_task_key_);
  if (task != task_groundings_.end()) {
    agra::MA_TaskStatusMT task_status;
    task_status.security_information = active_command_security_;
    task_status.message_header = makeHeader();
    task_status.message_data.task_id =
        task->second.task.message_data.task_id;
    task_status.message_data.execution_state =
        requirementState(snapshot.state);
    switch (snapshot.state) {
      case AutonomyBackendState::COMPLETE:
        task_status.message_data.percent_completed = 100.0;
        break;
      case AutonomyBackendState::EXECUTING:
      case AutonomyBackendState::WAITING_FOR_RESULTS:
        task_status.message_data.percent_completed = 50.0;
        break;
      default:
        task_status.message_data.percent_completed = 0.0;
        break;
    }
    agra_c2_provided::MA_TaskStatus_Service_Information
        task_information;
    task_information.ma_task_status = std::move(task_status);
    task_statuses_.push_back(std::move(task_information));
  }
}

}  // namespace ame
