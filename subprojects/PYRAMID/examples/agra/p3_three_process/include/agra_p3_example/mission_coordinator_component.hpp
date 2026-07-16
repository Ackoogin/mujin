#pragma once

#include "agra_p3_example/messages.hpp"

#include <pcl/component.hpp>

#include <string>
#include <unordered_set>

namespace agra_p3_example {

/// \brief Provides mission-action requests and exchanges MA-Action information.
class MissionCoordinatorComponent final : public pcl::Component {
 public:
  /// \brief Compose the coordinator's generated P3 types and PCL ports.
  MissionCoordinatorComponent();

  /// \brief Handle a mission-action Create request from either planner.
  p3::Ack handleMaActioncommandCreate(
      const p3::MA_ActionCommand_Service_Request& request);

 protected:
  pcl_status_t on_configure() override;
  pcl_status_t on_tick(double dt) override;

 private:
  static pcl_status_t dispatchCreate(pcl_container_t* container,
                                     const pcl_msg_t* request,
                                     pcl_msg_t* response,
                                     pcl_svc_context_t* context,
                                     void* user_data);
  static void receiveAction(pcl_container_t* container,
                            const pcl_msg_t* message,
                            void* user_data);

  pcl::Port request_service_;
  pcl::Port information_publisher_;
  pcl::Port information_subscriber_;
  std::string response_storage_;
  std::unordered_set<std::string> handled_command_ids_;
  double publish_elapsed_ = 0.0;
  unsigned publish_sequence_ = 0;
};

}  // namespace agra_p3_example
