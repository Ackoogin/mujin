// Auto-generated ROS2 transport projection — do not edit
// Backend: ros2
#pragma once

#include "pyramid_ros2_transport_support.hpp"

#include <pcl/pcl_types.h>

namespace pyramid::components::tactical_objects::services::provided::ros2_transport {

class ServiceBinder {
 public:
  ServiceBinder(pyramid::transport::ros2::Adapter& adapter,
                pcl_executor_t* executor);
  void bind();

 private:
  pyramid::transport::ros2::Adapter& adapter_;
  pcl_executor_t* executor_;
};

}  // namespace pyramid::components::tactical_objects::services::provided::ros2_transport
