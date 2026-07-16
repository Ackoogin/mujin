#pragma once

#include "agra_p3_example/messages.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

#include <future>
#include <string>
#include <unordered_set>

namespace agra_p3_example {

/// \brief Bridges C2 action intent into Mission System tasks.
class MissionAutonomyComponent final
    : public pcl::Component,
      private c2::MaActioncommandRequestPortHandler {
 public:
  /// \brief Compose separate C2 and Mission System request/information ports.
  explicit MissionAutonomyComponent(pcl::Executor& executor);

 protected:
  pcl_status_t on_configure() override;
  pcl_status_t on_tick(double dt) override;

 private:
  c2::Ack onCreate(
      const c2::MA_ActionCommand_Service_Request& request) override;
  void startTaskRequest(const std::string& action_command_id);

  c2::MaActioncommandRequestPortProvider c2_action_request_port_;
  c2::MaActionInformationPortSource c2_action_information_port_;
  ms_request::MaTaskcommandRequestPortClient ms_task_request_port_;
  ms_information::MaTaskInformationPortSource ms_task_information_port_;
  std::future<ms_request::MaTaskcommandRequestPortClient::SubmitResult>
      pending_task_request_;
  std::unordered_set<std::string> handled_action_command_ids_;
  double publish_elapsed_ = 0.0;
  unsigned action_sequence_ = 0;
  unsigned task_sequence_ = 0;
  bool task_request_started_ = false;
};

}  // namespace agra_p3_example
