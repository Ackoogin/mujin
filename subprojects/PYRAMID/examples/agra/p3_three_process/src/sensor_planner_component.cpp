#include "agra_p3_example/sensor_planner_component.hpp"

#include <iostream>

namespace agra_p3_example {

SensorPlannerComponent::SensorPlannerComponent(pcl::Executor& executor)
    : pcl::Component("sensor_planner"), executor_(&executor) {
  setTickRateHz(5.0);
}

pcl_status_t SensorPlannerComponent::on_configure() {
  information_publisher_ =
      addPublisher(p3::kTopicMaAction, p3::kJsonContentType);
  information_subscriber_ = addSubscriber(
      p3::kTopicMaAction, p3::kJsonContentType,
      &SensorPlannerComponent::receiveAction, this);
  return information_publisher_ && information_subscriber_
             ? PCL_OK
             : PCL_ERR_NOMEM;
}

pcl_status_t SensorPlannerComponent::on_tick(double dt) {
  process_elapsed_ += dt;
  publish_elapsed_ += dt;

  if (publish_elapsed_ >= 1.0) {
    publish_elapsed_ = 0.0;
    const auto information =
        makeActionInformation(name(), ++publish_sequence_);
    const std::string payload = p3::toJson(information);
    const pcl_status_t status =
        information_publisher_.publish(payload, p3::kJsonContentType);
    if (status != PCL_OK) {
      return status;
    }
    std::cout << "[sensor_planner] published MA-Action "
              << actionId(information) << std::endl;
  }

  if (!request_started_ && process_elapsed_ >= 2.0) {
    startRequest();
  }
  return PCL_OK;
}

void SensorPlannerComponent::startRequest() {
  request_started_ = true;
  request_storage_ = p3::toJson(makeActionRequest(name()));
  const pcl_msg_t request = {request_storage_.data(),
                             static_cast<uint32_t>(request_storage_.size()),
                             p3::kJsonContentType};
  const pcl_status_t status = pcl_executor_invoke_async(
      executor_->handle(), p3::kSvcMaActioncommandCreate, &request,
      &SensorPlannerComponent::receiveCreateResponse, this);
  std::cout << "[sensor_planner] submitted Create request: status="
            << static_cast<int>(status) << std::endl;
}

void SensorPlannerComponent::receiveCreateResponse(const pcl_msg_t* response,
                                                   void*) {
  try {
    const std::string payload(
        response && response->data
            ? static_cast<const char*>(response->data)
            : "",
        response ? response->size : 0u);
    const auto ack = pyramid::domain_model::agra_port_grammar::fromJson(
        payload, static_cast<p3::Ack*>(nullptr));
    std::cout << "[sensor_planner] Create response: remote_ack="
              << (ack.success ? "true" : "false") << std::endl;
  } catch (const std::exception& error) {
    std::cerr << "[sensor_planner] Create response decode failed: "
              << error.what() << std::endl;
  }
}

void SensorPlannerComponent::receiveAction(
    pcl_container_t*, const pcl_msg_t* message, void*) {
  try {
    const std::string payload(
        message && message->data ? static_cast<const char*>(message->data) : "",
        message ? message->size : 0u);
    const auto information = p3::fromJson(
        payload, static_cast<p3::MA_Action_Service_Information*>(nullptr));
    std::cout << "[sensor_planner] received MA-Action "
              << actionId(information) << std::endl;
  } catch (const std::exception& error) {
    std::cerr << "[sensor_planner] MA-Action decode failed: "
              << error.what() << std::endl;
  }
}

}  // namespace agra_p3_example
