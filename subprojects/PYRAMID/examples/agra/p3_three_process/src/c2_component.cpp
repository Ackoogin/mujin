#include "agra_p3_example/c2_component.hpp"

#include <chrono>
#include <iostream>

namespace agra_p3_example {

C2Component::C2Component(pcl::Executor& executor)
    : pcl::Component("c2"),
      action_request_port_(*this, executor),
      action_information_port_(*this, executor) {
  setTickRateHz(5.0);
}

pcl_status_t C2Component::on_configure() {
  if (auto status = action_request_port_.bind(); status != PCL_OK) {
    return status;
  }
  if (auto status = action_information_port_.bind(); status != PCL_OK) {
    return status;
  }
  action_subscription_ = action_information_port_.subscribe(
      [this](const c2::MA_Action_Service_Information& information) {
        receiveAction(information);
      });
  return action_subscription_ ? PCL_OK : PCL_ERR_STATE;
}

pcl_status_t C2Component::on_tick(double dt) {
  request_elapsed_ += dt;
  if (!request_started_ && request_elapsed_ >= 2.0) {
    startActionRequest();
  }
  if (!pending_action_request_.valid() ||
      pending_action_request_.wait_for(std::chrono::seconds(0)) !=
          std::future_status::ready) {
    return PCL_OK;
  }

  const auto result = pending_action_request_.get();
  const bool remote_ack =
      result.remoteAck() && result.remoteAck()->success;
  std::cout << "[c2] MA-ActionCommand response: status="
            << static_cast<int>(result.status)
            << ", remote_ack=" << (remote_ack ? "true" : "false")
            << std::endl;
  return result.status;
}

void C2Component::startActionRequest() {
  request_started_ = true;
  auto request = action_request_port_.submit(makeActionRequest());
  pending_action_request_ = std::async(
      std::launch::async,
      [request = std::move(request)]() mutable { return request.get(); });
  std::cout << "[c2] submitted MA-ActionCommand Create request" << std::endl;
}

void C2Component::receiveAction(
    const c2::MA_Action_Service_Information& information) {
  std::cout << "[c2] received MA-Action information "
            << actionId(information) << std::endl;
}

}  // namespace agra_p3_example
