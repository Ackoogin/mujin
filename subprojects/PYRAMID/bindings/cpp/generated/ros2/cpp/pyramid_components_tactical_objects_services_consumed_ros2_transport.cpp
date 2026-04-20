// Auto-generated ROS2 transport projection — do not edit

#include "pyramid_components_tactical_objects_services_consumed_ros2_transport.hpp"

namespace {

constexpr const char* kSvcobject_evidence_ReadDetail = "object_evidence.read_detail";
constexpr const char* kSvcobject_solution_evidence_CreateRequirement = "object_solution_evidence.create_requirement";
constexpr const char* kSvcobject_solution_evidence_ReadRequirement = "object_solution_evidence.read_requirement";
constexpr const char* kSvcobject_solution_evidence_UpdateRequirement = "object_solution_evidence.update_requirement";
constexpr const char* kSvcobject_solution_evidence_DeleteRequirement = "object_solution_evidence.delete_requirement";
constexpr const char* kSvcobject_source_capability_ReadCapability = "object_source_capability.read_capability";

}  // namespace

namespace pyramid::components::tactical_objects::services::consumed::ros2_transport {

ServiceBinder::ServiceBinder(
    pyramid::transport::ros2::Adapter& adapter,
    pcl_executor_t* executor)
    : adapter_(adapter), executor_(executor) {}

void ServiceBinder::bind() {
  pyramid::transport::ros2::bindStreamServiceIngress(adapter_, executor_, kSvcobject_evidence_ReadDetail);
  pyramid::transport::ros2::bindUnaryServiceIngress(adapter_, executor_, kSvcobject_solution_evidence_CreateRequirement);
  pyramid::transport::ros2::bindStreamServiceIngress(adapter_, executor_, kSvcobject_solution_evidence_ReadRequirement);
  pyramid::transport::ros2::bindUnaryServiceIngress(adapter_, executor_, kSvcobject_solution_evidence_UpdateRequirement);
  pyramid::transport::ros2::bindUnaryServiceIngress(adapter_, executor_, kSvcobject_solution_evidence_DeleteRequirement);
  pyramid::transport::ros2::bindStreamServiceIngress(adapter_, executor_, kSvcobject_source_capability_ReadCapability);
}

}  // namespace pyramid::components::tactical_objects::services::consumed::ros2_transport
