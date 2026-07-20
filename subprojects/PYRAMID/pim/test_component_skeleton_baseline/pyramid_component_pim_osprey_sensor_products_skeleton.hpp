// Auto-generated component skeleton. Regenerated on every run.
#pragma once

#include "pyramid_services_pim_osprey_sensor_products_consumed_components.hpp"
#include "pyramid_services_pim_osprey_sensor_products_provided_components.hpp"
#include "pyramid_component_skeleton_support.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>
#include <pcl/process_runtime.hpp>

#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace pyramid::components::pim_osprey::sensor_products {

class SensorProductsSkeleton : public pcl::Component {
public:
  struct Handlers {
    pyramid::component_skeleton::HandlerSlot<
        services::provided::AuthorisationDependencyRequestPortHandler>
        authorisation_dependency_request;
    std::function<void(const services::consumed::Capabilities&)>
        capability_evidence_information;
    std::function<void(const services::consumed::SPRMeasuredInformation_Service_Information&)>
        spr_measured_information_information;
    pyramid::component_skeleton::HandlerSlot<
        services::provided::SprrequirementRequestPortHandler>
        spr_requirement_request;
  };

  explicit SensorProductsSkeleton(
      pcl::Executor& executor,
      Handlers handlers,
      pyramid::component_skeleton::ContentTypeResolver codec_for = {},
      std::string name = "sensor_products");
  ~SensorProductsSkeleton() override = default;

  static std::vector<pcl::DeploymentPort> deploymentPorts();

  SensorProductsSkeleton(const SensorProductsSkeleton&) = delete;
  SensorProductsSkeleton& operator=(const SensorProductsSkeleton&) = delete;

protected:
  pcl_status_t on_configure() override;
  virtual pcl_status_t on_user_configure() { return PCL_OK; }
  services::provided::AuthorisationDependencyRequestPortProvider& authorisationDependencyRequestPort() {
    return authorisation_dependency_request_port_;
  }
  services::consumed::CapabilityEvidenceInformationPortSink& capabilityEvidenceInformationPort() {
    return capability_evidence_information_port_;
  }
  services::provided::CapabilityInformationPortSource& capabilityInformationPort() {
    return capability_information_port_;
  }
  services::provided::SprinformationInformationPortSource& sprInformationInformationPort() {
    return spr_information_information_port_;
  }
  services::consumed::SprmeasuredinformationInformationPortSink& sprMeasuredInformationInformationPort() {
    return spr_measured_information_information_port_;
  }
  services::provided::SprrequirementRequestPortProvider& sprRequirementRequestPort() {
    return spr_requirement_request_port_;
  }

private:
  Handlers handlers_;
  services::provided::AuthorisationDependencyRequestPortProvider authorisation_dependency_request_port_;
  services::consumed::CapabilityEvidenceInformationPortSink capability_evidence_information_port_;
  services::consumed::SubscriptionHandle capability_evidence_information_subscription_;
  services::provided::CapabilityInformationPortSource capability_information_port_;
  services::provided::SprinformationInformationPortSource spr_information_information_port_;
  services::consumed::SprmeasuredinformationInformationPortSink spr_measured_information_information_port_;
  services::consumed::SubscriptionHandle spr_measured_information_information_subscription_;
  services::provided::SprrequirementRequestPortProvider spr_requirement_request_port_;
};

}  // namespace pyramid::components::pim_osprey::sensor_products
