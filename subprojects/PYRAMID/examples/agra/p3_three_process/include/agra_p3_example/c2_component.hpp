#pragma once

#include "agra_p3_example/messages.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

#include <future>

namespace agra_p3_example {

/// \brief Sends C2 action requests and receives MA-Action information.
class C2Component final : public pcl::Component {
 public:
  /// \brief Compose the generated C2 request client and information sink.
  explicit C2Component(pcl::Executor& executor);

 protected:
  pcl_status_t on_configure() override;
  pcl_status_t on_tick(double dt) override;

 private:
  void startActionRequest();
  void receiveAction(const c2::MA_Action_Service_Information& information);

  c2::MaActioncommandRequestPortClient action_request_port_;
  c2::MaActionInformationPortSink action_information_port_;
  c2::SubscriptionHandle action_subscription_;
  std::future<c2::MaActioncommandRequestPortClient::SubmitResult>
      pending_action_request_;
  double request_elapsed_ = 0.0;
  bool request_started_ = false;
};

}  // namespace agra_p3_example
