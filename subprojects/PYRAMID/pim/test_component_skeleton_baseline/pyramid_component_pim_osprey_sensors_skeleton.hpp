// Auto-generated component skeleton. Regenerated on every run.
#pragma once

#include "pyramid_services_pim_osprey_sensors_consumed_components.hpp"
#include "pyramid_services_pim_osprey_sensors_provided_components.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>
#include <pcl/process_runtime.hpp>

#include <future>
#include <string>
#include <vector>

namespace pyramid::components::pim_osprey::sensors {

class SensorsSkeleton : public pcl::Component {
private:
  struct AuthorisationDependencyHandlerAdapter final : services::provided::AuthorisationDependencyRequestPortHandler {
    explicit AuthorisationDependencyHandlerAdapter(SensorsSkeleton& owner)
        : owner_(owner) {}
    services::provided::Ack onCancel(const services::provided::Identifier& request) override {
      return owner_.onAuthorisationDependencyCancel(request);
    }
    services::provided::Ack onCreate(const services::provided::Authorisation_Dependency_Service_Request& request) override {
      return owner_.onAuthorisationDependencyCreate(request);
    }
    services::provided::Ack onUpdate(const services::provided::Authorisation_Dependency_Service_Requirement& request) override {
      return owner_.onAuthorisationDependencyUpdate(request);
    }
    SensorsSkeleton& owner_;
  };
  struct SenrequirementHandlerAdapter final : services::provided::SenrequirementRequestPortHandler {
    explicit SenrequirementHandlerAdapter(SensorsSkeleton& owner)
        : owner_(owner) {}
    services::provided::Ack onCancel(const services::provided::Identifier& request) override {
      return owner_.onSenrequirementCancel(request);
    }
    services::provided::Ack onCreate(const services::provided::SENRequirement_Service_Request& request) override {
      return owner_.onSenrequirementCreate(request);
    }
    services::provided::Ack onUpdate(const services::provided::SENRequirement_Service_Requirement& request) override {
      return owner_.onSenrequirementUpdate(request);
    }
    SensorsSkeleton& owner_;
  };

public:
  explicit SensorsSkeleton(
      pcl::Executor& executor,
      std::string name = "sensors");
  ~SensorsSkeleton() override = default;

  static std::vector<pcl::DeploymentPort> deploymentPorts();

  SensorsSkeleton(const SensorsSkeleton&) = delete;
  SensorsSkeleton& operator=(const SensorsSkeleton&) = delete;

protected:
  pcl_status_t on_configure() override;
  virtual pcl_status_t on_user_configure() { return PCL_OK; }

  // Provider callbacks are deliberately pure: the generated
  // facade handlers have defaults, but a component skeleton does not.
  virtual services::provided::Ack onAuthorisationDependencyCancel(const services::provided::Identifier& request) = 0;
  virtual services::provided::Ack onAuthorisationDependencyCreate(const services::provided::Authorisation_Dependency_Service_Request& request) = 0;
  virtual services::provided::Ack onAuthorisationDependencyUpdate(const services::provided::Authorisation_Dependency_Service_Requirement& request) = 0;
  pcl_status_t sendAuthorisationDependencyTransition(const services::provided::Authorisation_Dependency_Service_Requirement& item) {
    return authorisation_dependency_request_port_.transitionWriter().send(item);
  }
  services::provided::AuthorisationDependencyRequestPortProvider& authorisationDependencyRequestPort() {
    return authorisation_dependency_request_port_;
  }
  virtual void onCapabilityEvidence(const services::consumed::Capabilities& item) = 0;
  services::consumed::CapabilityEvidenceInformationPortSink& capabilityEvidenceInformationPort() {
    return capability_evidence_information_port_;
  }
  pcl_status_t publishCapability(const services::provided::Capabilities& item) {
    return capability_information_port_.publish(item);
  }
  services::provided::CapabilityInformationPortSource& capabilityInformationPort() {
    return capability_information_port_;
  }
  virtual services::provided::Ack onSenrequirementCancel(const services::provided::Identifier& request) = 0;
  virtual services::provided::Ack onSenrequirementCreate(const services::provided::SENRequirement_Service_Request& request) = 0;
  virtual services::provided::Ack onSenrequirementUpdate(const services::provided::SENRequirement_Service_Requirement& request) = 0;
  pcl_status_t sendSenrequirementTransition(const services::provided::SENRequirement_Service_Requirement& item) {
    return sen_requirement_request_port_.transitionWriter().send(item);
  }
  services::provided::SenrequirementRequestPortProvider& senRequirementRequestPort() {
    return sen_requirement_request_port_;
  }

private:
  AuthorisationDependencyHandlerAdapter authorisation_dependency_request_handler_;
  SenrequirementHandlerAdapter sen_requirement_request_handler_;
  services::provided::AuthorisationDependencyRequestPortProvider authorisation_dependency_request_port_;
  services::consumed::CapabilityEvidenceInformationPortSink capability_evidence_information_port_;
  services::consumed::SubscriptionHandle capability_evidence_information_subscription_;
  services::provided::CapabilityInformationPortSource capability_information_port_;
  services::provided::SenrequirementRequestPortProvider sen_requirement_request_port_;
};

}  // namespace pyramid::components::pim_osprey::sensors
