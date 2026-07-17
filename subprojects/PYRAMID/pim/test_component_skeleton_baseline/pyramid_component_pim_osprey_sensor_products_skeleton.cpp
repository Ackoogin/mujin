#include "pyramid_component_pim_osprey_sensor_products_skeleton.hpp"

namespace pyramid::components::pim_osprey::sensor_products {

SensorProductsSkeleton::SensorProductsSkeleton(
    pcl::Executor& executor, std::string name)
    : pcl::Component(name),
      authorisation_dependency_request_handler_(*this),
      spr_requirement_request_handler_(*this),
      authorisation_dependency_request_port_(*this, executor, authorisation_dependency_request_handler_),
      capability_evidence_information_port_(*this, executor),
      capability_information_port_(*this, executor),
      spr_information_information_port_(*this, executor),
      spr_measured_information_information_port_(*this, executor),
      spr_requirement_request_port_(*this, executor, spr_requirement_request_handler_) {}

pcl_status_t SensorProductsSkeleton::on_configure() {
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
  if (const auto status = spr_information_information_port_.bind();
      status != PCL_OK) {
    return status;
  }
  if (const auto status = spr_measured_information_information_port_.bind();
      status != PCL_OK) {
    return status;
  }
  spr_measured_information_information_subscription_ =
      spr_measured_information_information_port_.subscribe(
          [this](const services::consumed::SPRMeasuredInformation_Service_Information& item) { onSprmeasuredinformation(item); });
  if (!spr_measured_information_information_subscription_) return PCL_ERR_STATE;
  if (const auto status = spr_requirement_request_port_.bind();
      status != PCL_OK) {
    return status;
  }
  return on_user_configure();
}

std::vector<pcl::DeploymentPort> SensorProductsSkeleton::deploymentPorts() {
  return {
      {"authorisation_dependency_request", services::provided::AuthorisationDependencyRequestPortProvider::deploymentDescriptor()},
      {"capability_evidence_information", services::consumed::CapabilityEvidenceInformationPortSink::deploymentDescriptor()},
      {"capability_information", services::provided::CapabilityInformationPortSource::deploymentDescriptor()},
      {"spr_information_information", services::provided::SprinformationInformationPortSource::deploymentDescriptor()},
      {"spr_measured_information_information", services::consumed::SprmeasuredinformationInformationPortSink::deploymentDescriptor()},
      {"spr_requirement_request", services::provided::SprrequirementRequestPortProvider::deploymentDescriptor()},
  };
}

}  // namespace pyramid::components::pim_osprey::sensor_products
