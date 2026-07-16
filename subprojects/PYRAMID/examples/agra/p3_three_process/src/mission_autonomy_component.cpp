#include "agra_p3_example/mission_autonomy_component.hpp"

#include <chrono>
#include <iostream>

namespace agra_p3_example {

MissionAutonomyComponent::MissionAutonomyComponent(pcl::Executor& executor)
    : pcl::Component("mission_autonomy"),
      c2_action_request_port_(*this, executor, *this),
      c2_action_information_port_(*this, executor),
      ms_task_request_port_(*this, executor),
      ms_task_information_port_(*this, executor) {
  setTickRateHz(5.0);
}

pcl_status_t MissionAutonomyComponent::on_configure() {
  if (auto status = c2_action_request_port_.bind(); status != PCL_OK) {
    return status;
  }
  if (auto status = c2_action_information_port_.bind(); status != PCL_OK) {
    return status;
  }
  if (auto status = ms_task_request_port_.bind(); status != PCL_OK) {
    return status;
  }
  return ms_task_information_port_.bind();
}

pcl_status_t MissionAutonomyComponent::on_tick(double dt) {
  publish_elapsed_ += dt;
  if (publish_elapsed_ >= 1.0) {
    publish_elapsed_ = 0.0;
    const auto action = makeActionInformation(++action_sequence_);
    if (auto status = c2_action_information_port_.publish(action);
        status != PCL_OK) {
      return status;
    }
    std::cout << "[mission_autonomy] published MA-Action information "
              << actionId(action) << std::endl;

    const auto task = makeTaskInformation(++task_sequence_);
    if (auto status = ms_task_information_port_.publish(task);
        status != PCL_OK) {
      return status;
    }
    std::cout << "[mission_autonomy] published MA-Task information "
              << taskId(task) << std::endl;
  }

  if (!pending_task_request_.valid() ||
      pending_task_request_.wait_for(std::chrono::seconds(0)) !=
          std::future_status::ready) {
    return PCL_OK;
  }
  const auto result = pending_task_request_.get();
  const bool remote_ack =
      result.remoteAck() && result.remoteAck()->success;
  std::cout << "[mission_autonomy] MA-TaskCommand response: status="
            << static_cast<int>(result.status)
            << ", remote_ack=" << (remote_ack ? "true" : "false")
            << std::endl;
  return result.status;
}

c2::Ack MissionAutonomyComponent::onCreate(
    const c2::MA_ActionCommand_Service_Request& request) {
  const std::string command_id = actionCommandId(request);
  if (handled_action_command_ids_.insert(command_id).second) {
    std::cout << "[mission_autonomy] accepted MA-ActionCommand Create "
              << command_id << std::endl;
    startTaskRequest(command_id);
  } else {
    std::cout << "[mission_autonomy] ignored duplicate MA-ActionCommand "
              << command_id << std::endl;
  }
  return c2::Ack{true};
}

void MissionAutonomyComponent::startTaskRequest(
    const std::string& action_command_id) {
  if (task_request_started_) {
    return;
  }
  task_request_started_ = true;
  auto request =
      ms_task_request_port_.submit(makeTaskRequest(action_command_id));
  pending_task_request_ = std::async(
      std::launch::async,
      [request = std::move(request)]() mutable { return request.get(); });
  std::cout << "[mission_autonomy] submitted MA-TaskCommand Create request"
            << std::endl;
}

}  // namespace agra_p3_example
