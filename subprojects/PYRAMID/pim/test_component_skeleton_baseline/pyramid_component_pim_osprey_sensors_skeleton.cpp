#include "pyramid_component_pim_osprey_sensors_skeleton.hpp"

namespace pyramid::components::pim_osprey::sensors {

SensorsSkeleton::SensorsSkeleton(
    pcl::Executor& executor, std::string name)
    : pcl::Component(name),
      authorisation_dependency_request_handler_(*this),
      sen_requirement_request_handler_(*this),
      authorisation_dependency_request_port_(*this, executor, authorisation_dependency_request_handler_),
      capability_evidence_information_port_(*this, executor),
      capability_information_port_(*this, executor),
      sen_requirement_request_port_(*this, executor, sen_requirement_request_handler_) {}

pcl_status_t SensorsSkeleton::on_configure() {
  if (const auto status = authorisation_dependency_request_port_.bind();
      status != PCL_OK) {
    return status;
  }
  if (const auto status = capability_evidence_information_port_.bind();
      status != PCL_OK) {
    return status;
  }
  capability_evidence_information_subscription_ =
      capability_evidence_information_port_.subscribe(
          [this](const services::consumed::Capabilities& item) { onCapabilityEvidence(item); });
  if (!capability_evidence_information_subscription_) return PCL_ERR_STATE;
  if (const auto status = capability_information_port_.bind();
      status != PCL_OK) {
    return status;
  }
  if (const auto status = sen_requirement_request_port_.bind();
      status != PCL_OK) {
    return status;
  }
  return on_user_configure();
}

std::vector<pcl::DeploymentPort> SensorsSkeleton::deploymentPorts() {
  return {
      {"authorisation_dependency_request", services::provided::AuthorisationDependencyRequestPortProvider::deploymentDescriptor()},
      {"capability_evidence_information", services::consumed::CapabilityEvidenceInformationPortSink::deploymentDescriptor()},
      {"capability_information", services::provided::CapabilityInformationPortSource::deploymentDescriptor()},
      {"sen_requirement_request", services::provided::SenrequirementRequestPortProvider::deploymentDescriptor()},
  };
}

}  // namespace pyramid::components::pim_osprey::sensors
