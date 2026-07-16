#include "agra_p3_example/mission_coordinator_component.hpp"

#include <iostream>

namespace agra_p3_example {

MissionCoordinatorComponent::MissionCoordinatorComponent()
    : pcl::Component("mission_coordinator") {
  setTickRateHz(5.0);
}

p3::Ack MissionCoordinatorComponent::handleMaActioncommandCreate(
    const p3::MA_ActionCommand_Service_Request& request) {
  const std::string command_id = commandId(request);
  if (handled_command_ids_.insert(command_id).second) {
    std::cout << "[mission_coordinator] accepted Create request "
              << command_id << std::endl;
  } else {
    std::cout << "[mission_coordinator] ignored duplicate Create request "
              << command_id << std::endl;
  }
  return p3::Ack{true};
}

pcl_status_t MissionCoordinatorComponent::on_configure() {
  request_service_ = addService(
      p3::kSvcMaActioncommandCreate, p3::kJsonContentType,
      &MissionCoordinatorComponent::dispatchCreate, this);
  information_publisher_ =
      addPublisher(p3::kTopicMaAction, p3::kJsonContentType);
  information_subscriber_ = addSubscriber(
      p3::kTopicMaAction, p3::kJsonContentType,
      &MissionCoordinatorComponent::receiveAction, this);
  return request_service_ && information_publisher_ && information_subscriber_
             ? PCL_OK
             : PCL_ERR_NOMEM;
}

pcl_status_t MissionCoordinatorComponent::on_tick(double dt) {
  publish_elapsed_ += dt;
  if (publish_elapsed_ < 1.0) {
    return PCL_OK;
  }
  publish_elapsed_ = 0.0;
  const auto information =
      makeActionInformation(name(), ++publish_sequence_);
  const std::string payload = p3::toJson(information);
  const pcl_status_t status =
      information_publisher_.publish(payload, p3::kJsonContentType);
  if (status == PCL_OK) {
    std::cout << "[mission_coordinator] published MA-Action "
              << actionId(information) << std::endl;
  }
  return status;
}

pcl_status_t MissionCoordinatorComponent::dispatchCreate(
    pcl_container_t*, const pcl_msg_t* request, pcl_msg_t* response,
    pcl_svc_context_t*, void* user_data) {
  auto* self = static_cast<MissionCoordinatorComponent*>(user_data);
  if (!self || !response) {
    return PCL_ERR_INVALID;
  }

  try {
    const std::string request_json(
        request && request->data ? static_cast<const char*>(request->data) : "",
        request ? request->size : 0u);
    const auto typed_request = p3::fromJson(
        request_json,
        static_cast<p3::MA_ActionCommand_Service_Request*>(nullptr));
    const p3::Ack ack = self->handleMaActioncommandCreate(typed_request);
    self->response_storage_ =
        pyramid::domain_model::agra_port_grammar::toJson(ack);
  } catch (const std::exception& error) {
    std::cerr << "[mission_coordinator] Create decode failed: "
              << error.what() << std::endl;
    return PCL_ERR_INVALID;
  }
  response->data = self->response_storage_.empty()
                       ? nullptr
                       : self->response_storage_.data();
  response->size = static_cast<uint32_t>(self->response_storage_.size());
  response->type_name = p3::kJsonContentType;
  return PCL_OK;
}

void MissionCoordinatorComponent::receiveAction(
    pcl_container_t*, const pcl_msg_t* message, void*) {
  try {
    const std::string payload(
        message && message->data ? static_cast<const char*>(message->data) : "",
        message ? message->size : 0u);
    const auto information = p3::fromJson(
        payload, static_cast<p3::MA_Action_Service_Information*>(nullptr));
    std::cout << "[mission_coordinator] received MA-Action "
              << actionId(information) << std::endl;
  } catch (const std::exception& error) {
    std::cerr << "[mission_coordinator] MA-Action decode failed: "
              << error.what() << std::endl;
  }
}

}  // namespace agra_p3_example
