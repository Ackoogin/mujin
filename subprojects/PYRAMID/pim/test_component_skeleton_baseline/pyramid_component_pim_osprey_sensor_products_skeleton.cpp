#include "pyramid_component_pim_osprey_sensor_products_skeleton.hpp"

namespace pyramid::components::pim_osprey::sensor_products {

SensorProductsSkeleton::SensorProductsSkeleton(
    pcl::Executor& executor, Handlers handlers, std::string name)
    : pcl::Component(name),
      handlers_(std::move(handlers)),
      authorisation_dependency_request_port_(*this, executor, handlers_.authorisation_dependency_request.get()),
      capability_evidence_information_port_(*this, executor),
      capability_information_port_(*this, executor),
      spr_information_information_port_(*this, executor),
      spr_measured_information_information_port_(*this, executor),
      spr_requirement_request_port_(*this, executor, handlers_.spr_requirement_request.get()) {}

pcl_status_t SensorProductsSkeleton::on_configure() {
  if (!handlers_.capability_evidence_information) {
    logError("required handler slot capability_evidence_information is empty");
    return PCL_ERR_STATE;
  }
  if (!handlers_.spr_measured_information_information) {
    logError("required handler slot spr_measured_information_information is empty");
    return PCL_ERR_STATE;
  }
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
          handlers_.capability_evidence_information);
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
          handlers_.spr_measured_information_information);
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
