#include "agra_p3_example/messages.hpp"

namespace agra_p3_example {

p3::MA_Action_Service_Information makeActionInformation(
    const std::string& component_name, unsigned sequence) {
  pyramid::domain_model::agra::MA_ActionMT action;
  action.message_header.system_id.uuid = component_name;
  action.message_header.system_id.descriptive_label = component_name;
  action.message_header.schema_version = "A-GRA-5.0a-P3";
  action.message_data.action_id.base.uuid =
      component_name + "-action-" + std::to_string(sequence);
  action.message_data.action_id.base.descriptive_label =
      "Reference action from " + component_name;

  p3::MA_Action_Service_Information information;
  information.ma_action = std::move(action);
  return information;
}

std::string actionId(const p3::MA_Action_Service_Information& information) {
  return information.ma_action
             ? information.ma_action->message_data.action_id.base.uuid
             : std::string("<missing MA-Action payload>");
}

p3::MA_ActionCommand_Service_Request makeActionRequest(
    const std::string& component_name) {
  pyramid::domain_model::agra::ActivityCommandBaseType activity;
  activity.command_id.uuid = component_name + "-action-command";
  activity.command_id.descriptive_label =
      "Reference request from " + component_name;
  activity.activity_id.uuid = component_name + "-activity";

  pyramid::domain_model::agra::MA_ActionCommandType command;
  command.activity = std::move(activity);

  pyramid::domain_model::agra::MA_ActionCommandMT payload;
  payload.message_header.system_id.uuid = component_name;
  payload.message_header.schema_version = "A-GRA-5.0a-P3";
  payload.message_data.command.push_back(std::move(command));

  p3::MA_ActionCommand_Service_Request request;
  request.ma_action_command = std::move(payload);
  return request;
}

std::string commandId(const p3::MA_ActionCommand_Service_Request& request) {
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

}  // namespace agra_p3_example
