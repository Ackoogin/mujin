#pragma once

#include "agra_p3_example/messages.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

#include <string>

namespace agra_p3_example {

/// \brief Consumes requests and exchanges vehicle-planning MA-Action data.
class VehiclePlannerComponent final : public pcl::Component {
 public:
  /// \brief Compose the vehicle planner's generated P3 types and PCL ports.
  explicit VehiclePlannerComponent(pcl::Executor& executor);

 protected:
  pcl_status_t on_configure() override;
  pcl_status_t on_tick(double dt) override;

 private:
  void startRequest();
  static void receiveCreateResponse(const pcl_msg_t* response,
                                    void* user_data);
  static void receiveAction(pcl_container_t* container,
                            const pcl_msg_t* message,
                            void* user_data);

  pcl::Executor* executor_ = nullptr;
  pcl::Port information_publisher_;
  pcl::Port information_subscriber_;
  std::string request_storage_;
  double process_elapsed_ = 0.0;
  double publish_elapsed_ = 0.0;
  unsigned publish_sequence_ = 0;
  bool request_started_ = false;
};

}  // namespace agra_p3_example
