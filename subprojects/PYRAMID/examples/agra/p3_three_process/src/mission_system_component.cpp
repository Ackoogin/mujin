#include "agra_p3_example/mission_system_component.hpp"

#include <iostream>

namespace agra_p3_example {

MissionSystemComponent::MissionSystemComponent(pcl::Executor& executor)
    : pcl::Component("mission_system"),
      task_request_port_(*this, executor, *this),
      task_information_port_(*this, executor) {
  setTickRateHz(5.0);
}

pcl_status_t MissionSystemComponent::on_configure() {
  if (auto status = task_request_port_.bind(); status != PCL_OK) {
    return status;
  }
  if (auto status = task_information_port_.bind(); status != PCL_OK) {
    return status;
  }
  task_subscription_ = task_information_port_.subscribe(
      [this](const ms_information::MA_Task_Service_Information& information) {
        receiveTask(information);
      });
  return task_subscription_ ? PCL_OK : PCL_ERR_STATE;
}

ms_request::Ack MissionSystemComponent::onCreate(
    const ms_request::MA_TaskCommand_Service_Request& request) {
  const std::string command_id = taskCommandId(request);
  if (handled_task_command_ids_.insert(command_id).second) {
    std::cout << "[mission_system] accepted MA-TaskCommand Create "
              << command_id << std::endl;
  } else {
    std::cout << "[mission_system] ignored duplicate MA-TaskCommand "
              << command_id << std::endl;
  }
  return ms_request::Ack{true};
}

void MissionSystemComponent::receiveTask(
    const ms_information::MA_Task_Service_Information& information) {
  std::cout << "[mission_system] received MA-Task information "
            << taskId(information) << std::endl;
}

}  // namespace agra_p3_example
