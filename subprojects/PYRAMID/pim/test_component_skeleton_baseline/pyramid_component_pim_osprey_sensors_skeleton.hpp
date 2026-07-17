// Auto-generated component skeleton. Regenerated on every run.
#pragma once

#include "pyramid_services_pim_osprey_sensors_consumed_components.hpp"
#include "pyramid_services_pim_osprey_sensors_provided_components.hpp"
#include "pyramid_component_skeleton_support.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>
#include <pcl/process_runtime.hpp>

#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace pyramid::components::pim_osprey::sensors {

class SensorsSkeleton : public pcl::Component {
public:
  struct Handlers {
    pyramid::component_skeleton::HandlerSlot<
        services::provided::AuthorisationDependencyRequestPortHandler>
        authorisation_dependency_request;
    std::function<void(const services::consumed::Capabilities&)>
        capability_evidence_information;
    pyramid::component_skeleton::HandlerSlot<
        services::provided::SenrequirementRequestPortHandler>
        sen_requirement_request;
  };

  explicit SensorsSkeleton(
      pcl::Executor& executor,
      Handlers handlers,
      std::string name = "sensors");
  ~SensorsSkeleton() override = default;

  static std::vector<pcl::DeploymentPort> deploymentPorts();

  SensorsSkeleton(const SensorsSkeleton&) = delete;
  SensorsSkeleton& operator=(const SensorsSkeleton&) = delete;

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
  services::provided::SenrequirementRequestPortProvider& senRequirementRequestPort() {
    return sen_requirement_request_port_;
  }

private:
  Handlers handlers_;
  services::provided::AuthorisationDependencyRequestPortProvider authorisation_dependency_request_port_;
  services::consumed::CapabilityEvidenceInformationPortSink capability_evidence_information_port_;
  services::consumed::SubscriptionHandle capability_evidence_information_subscription_;
  services::provided::CapabilityInformationPortSource capability_information_port_;
  services::provided::SenrequirementRequestPortProvider sen_requirement_request_port_;
};

}  // namespace pyramid::components::pim_osprey::sensors
