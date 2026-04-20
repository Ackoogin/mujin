// Auto-generated ROS2 transport projection — do not edit

#include "pyramid_components_tactical_objects_ros2_transport.hpp"

namespace {

constexpr const char* kSvctactical_object_CreateTacticalObject = "tactical_object.create_tactical_object";
constexpr const char* kSvctactical_object_ReadTacticalObject = "tactical_object.read_tactical_object";
constexpr const char* kSvctactical_object_UpdateTacticalObject = "tactical_object.update_tactical_object";
constexpr const char* kSvctactical_object_DeleteTacticalObject = "tactical_object.delete_tactical_object";
constexpr const char* kSvczone_CreateZone = "zone.create_zone";
constexpr const char* kSvczone_ReadZone = "zone.read_zone";
constexpr const char* kSvczone_UpdateZone = "zone.update_zone";
constexpr const char* kSvczone_DeleteZone = "zone.delete_zone";
constexpr const char* kSvcobservation_ingress_CreateObservation = "observation_ingress.create_observation";

}  // namespace

namespace pyramid::components::tactical_objects::ros2_transport {

ServiceBinder::ServiceBinder(
    pyramid::transport::ros2::Adapter& adapter,
    pcl_executor_t* executor)
    : adapter_(adapter), executor_(executor) {}

void ServiceBinder::bind() {
  pyramid::transport::ros2::bindUnaryServiceIngress(adapter_, executor_, kSvctactical_object_CreateTacticalObject);
  pyramid::transport::ros2::bindStreamServiceIngress(adapter_, executor_, kSvctactical_object_ReadTacticalObject);
  pyramid::transport::ros2::bindUnaryServiceIngress(adapter_, executor_, kSvctactical_object_UpdateTacticalObject);
  pyramid::transport::ros2::bindUnaryServiceIngress(adapter_, executor_, kSvctactical_object_DeleteTacticalObject);
  pyramid::transport::ros2::bindUnaryServiceIngress(adapter_, executor_, kSvczone_CreateZone);
  pyramid::transport::ros2::bindStreamServiceIngress(adapter_, executor_, kSvczone_ReadZone);
  pyramid::transport::ros2::bindUnaryServiceIngress(adapter_, executor_, kSvczone_UpdateZone);
  pyramid::transport::ros2::bindUnaryServiceIngress(adapter_, executor_, kSvczone_DeleteZone);
  pyramid::transport::ros2::bindUnaryServiceIngress(adapter_, executor_, kSvcobservation_ingress_CreateObservation);
}

}  // namespace pyramid::components::tactical_objects::ros2_transport
