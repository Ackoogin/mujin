#include "agra_p3_example/messages.hpp"

namespace agra_p3_example {

c2::MA_Action_Service_Information makeActionInformation(unsigned sequence) {
  constexpr const char* kComponentName = "mission_autonomy";
  pyramid::domain_model::agra::MA_ActionMT action;
  action.message_header.system_id.uuid = kComponentName;
  action.message_header.system_id.descriptive_label = "Mission Autonomy";
  action.message_header.schema_version = "A-GRA-5.0a-P3";
  action.message_data.action_id.base.uuid =
      std::string(kComponentName) + "-action-" + std::to_string(sequence);
  action.message_data.action_id.base.descriptive_label =
      "Reference action from Mission Autonomy";

  c2::MA_Action_Service_Information information;
  information.ma_action = std::move(action);
  return information;
}

std::string actionId(const c2::MA_Action_Service_Information& information) {
  return information.ma_action
             ? information.ma_action->message_data.action_id.base.uuid
             : std::string("<missing MA-Action payload>");
}

c2::MA_ActionCommand_Service_Request makeActionRequest() {
  pyramid::domain_model::agra::ActivityCommandBaseType activity;
  activity.command_id.uuid = "c2-action-command";
  activity.command_id.descriptive_label =
      "C2 request for Mission Autonomy";
  activity.activity_id.uuid = "c2-requested-activity";

  pyramid::domain_model::agra::MA_ActionCommandType command;
  command.activity = std::move(activity);

  pyramid::domain_model::agra::MA_ActionCommandMT payload;
  payload.message_header.system_id.uuid = "c2";
  payload.message_header.schema_version = "A-GRA-5.0a-P3";
  payload.message_data.command.push_back(std::move(command));

  c2::MA_ActionCommand_Service_Request request;
  request.ma_action_command = std::move(payload);
  return request;
}

std::string actionCommandId(
    const c2::MA_ActionCommand_Service_Request& request) {
  if (!request.ma_action_command ||
      request.ma_action_command->message_data.command.empty()) {
    return "<missing command payload>";
  }
  const auto& command = request.ma_action_command->message_data.command.front();
  if (command.activity) {
    return command.activity->command_id.uuid;
  }
  if (command.capability) {
    return command.capability->base.command_id.uuid;
  }
  return "<missing command choice>";
}

ms_information::MA_Task_Service_Information makeTaskInformation(
    unsigned sequence) {
  pyramid::domain_model::agra::MA_TaskMT task;
  task.message_header.system_id.uuid = "mission_autonomy";
  task.message_header.system_id.descriptive_label = "Mission Autonomy";
  task.message_header.schema_version = "A-GRA-5.0a-P3";
  task.message_data.task_id.base.uuid =
      "mission-autonomy-task-" + std::to_string(sequence);
  task.message_data.task_id.base.descriptive_label =
      "Reference task for Mission System";

  ms_information::MA_Task_Service_Information information;
  information.ma_task = std::move(task);
  return information;
}

std::string taskId(
    const ms_information::MA_Task_Service_Information& information) {
  return information.ma_task
             ? information.ma_task->message_data.task_id.base.uuid
             : std::string("<missing MA-Task payload>");
}

ms_request::MA_TaskCommand_Service_Request makeTaskRequest(
    const std::string& action_command_id) {
  pyramid::domain_model::agra::MA_TaskCommandType command;
  command.capability.base.command_id.uuid =
      "mission-autonomy-task-command";
  command.capability.base.command_id.descriptive_label =
      "Mission Autonomy request to Mission System";
  command.capability.capability_id.uuid = "reference-mission-system";
  command.capability.task_id.base.uuid = action_command_id + "-task";

  pyramid::domain_model::agra::MA_TaskCommandMT payload;
  payload.message_header.system_id.uuid = "mission_autonomy";
  payload.message_header.schema_version = "A-GRA-5.0a-P3";
  payload.message_data.command.push_back(std::move(command));

  ms_request::MA_TaskCommand_Service_Request request;
  request.ma_task_command = std::move(payload);
  return request;
}

std::string taskCommandId(
    const ms_request::MA_TaskCommand_Service_Request& request) {
  if (!request.ma_task_command ||
      request.ma_task_command->message_data.command.empty()) {
    return "<missing task command payload>";
  }
  return request.ma_task_command->message_data.command.front()
      .capability.base.command_id.uuid;
}

}  // namespace agra_p3_example
