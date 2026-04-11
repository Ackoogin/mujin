// Auto-generated ROS2 transport projection — do not edit

#include "pyramid_components_tactical_objects_services_provided_ros2_transport.hpp"

namespace {

constexpr const char* kSvcmatching_objects_ReadMatch = "matching_objects.read_match";
constexpr const char* kSvcobject_of_interest_CreateRequirement = "object_of_interest.create_requirement";
constexpr const char* kSvcobject_of_interest_ReadRequirement = "object_of_interest.read_requirement";
constexpr const char* kSvcobject_of_interest_UpdateRequirement = "object_of_interest.update_requirement";
constexpr const char* kSvcobject_of_interest_DeleteRequirement = "object_of_interest.delete_requirement";
constexpr const char* kSvcspecific_object_detail_ReadDetail = "specific_object_detail.read_detail";

}  // namespace

namespace pyramid::components::tactical_objects::services::provided::ros2_transport {

ServiceBinder::ServiceBinder(
    pyramid::transport::ros2::Adapter& adapter,
    pcl_executor_t* executor)
    : adapter_(adapter), executor_(executor) {}

void ServiceBinder::bind() {
  pyramid::transport::ros2::bindStreamServiceIngress(adapter_, executor_, kSvcmatching_objects_ReadMatch);
  pyramid::transport::ros2::bindUnaryServiceIngress(adapter_, executor_, kSvcobject_of_interest_CreateRequirement);
  pyramid::transport::ros2::bindStreamServiceIngress(adapter_, executor_, kSvcobject_of_interest_ReadRequirement);
  pyramid::transport::ros2::bindUnaryServiceIngress(adapter_, executor_, kSvcobject_of_interest_UpdateRequirement);
  pyramid::transport::ros2::bindUnaryServiceIngress(adapter_, executor_, kSvcobject_of_interest_DeleteRequirement);
  pyramid::transport::ros2::bindStreamServiceIngress(adapter_, executor_, kSvcspecific_object_detail_ReadDetail);
}

}  // namespace pyramid::components::tactical_objects::services::provided::ros2_transport
