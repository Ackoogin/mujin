#pragma once

#include "agra_p3_example/messages.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

#include <string>
#include <unordered_set>

namespace agra_p3_example {

/// \brief Accepts MA task requests and receives MA-Task information.
class MissionSystemComponent final
    : public pcl::Component,
      private ms_request::MaTaskcommandRequestPortHandler {
 public:
  /// \brief Compose the generated Mission System request and information ports.
  explicit MissionSystemComponent(pcl::Executor& executor);

 protected:
  pcl_status_t on_configure() override;

 private:
  ms_request::Ack onCreate(
      const ms_request::MA_TaskCommand_Service_Request& request) override;
  void receiveTask(
      const ms_information::MA_Task_Service_Information& information);

  ms_request::MaTaskcommandRequestPortProvider task_request_port_;
  ms_information::MaTaskInformationPortSink task_information_port_;
  ms_information::SubscriptionHandle task_subscription_;
  std::unordered_set<std::string> handled_task_command_ids_;
};

}  // namespace agra_p3_example
