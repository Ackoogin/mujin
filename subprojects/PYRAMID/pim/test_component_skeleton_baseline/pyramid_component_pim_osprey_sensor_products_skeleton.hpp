// Auto-generated component skeleton. Regenerated on every run.
#pragma once

#include "pyramid_services_pim_osprey_sensor_products_consumed_components.hpp"
#include "pyramid_services_pim_osprey_sensor_products_provided_components.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>
#include <pcl/process_runtime.hpp>

#include <future>
#include <string>
#include <vector>

namespace pyramid::components::pim_osprey::sensor_products {

class SensorProductsSkeleton : public pcl::Component {
private:
  struct AuthorisationDependencyHandlerAdapter final : services::provided::AuthorisationDependencyRequestPortHandler {
    explicit AuthorisationDependencyHandlerAdapter(SensorProductsSkeleton& owner)
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
    SensorProductsSkeleton& owner_;
  };
  struct SprrequirementHandlerAdapter final : services::provided::SprrequirementRequestPortHandler {
    explicit SprrequirementHandlerAdapter(SensorProductsSkeleton& owner)
        : owner_(owner) {}
    services::provided::Ack onCancel(const services::provided::Identifier& request) override {
      return owner_.onSprrequirementCancel(request);
    }
    services::provided::Ack onCreate(const services::provided::SPRRequirement_Service_Request& request) override {
      return owner_.onSprrequirementCreate(request);
    }
    services::provided::Ack onUpdate(const services::provided::SPRRequirement_Service_Requirement& request) override {
      return owner_.onSprrequirementUpdate(request);
    }
    SensorProductsSkeleton& owner_;
  };

public:
  explicit SensorProductsSkeleton(
      pcl::Executor& executor,
      std::string name = "sensor_products");
  ~SensorProductsSkeleton() override = default;

  static std::vector<pcl::DeploymentPort> deploymentPorts();

  SensorProductsSkeleton(const SensorProductsSkeleton&) = delete;
  SensorProductsSkeleton& operator=(const SensorProductsSkeleton&) = delete;

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
  pcl_status_t publishSprinformation(const services::provided::SPRInformation_Service_Information& item) {
    return spr_information_information_port_.publish(item);
  }
  services::provided::SprinformationInformationPortSource& sprInformationInformationPort() {
    return spr_information_information_port_;
  }
  virtual void onSprmeasuredinformation(const services::consumed::SPRMeasuredInformation_Service_Information& item) = 0;
  services::consumed::SprmeasuredinformationInformationPortSink& sprMeasuredInformationInformationPort() {
    return spr_measured_information_information_port_;
  }
  virtual services::provided::Ack onSprrequirementCancel(const services::provided::Identifier& request) = 0;
  virtual services::provided::Ack onSprrequirementCreate(const services::provided::SPRRequirement_Service_Request& request) = 0;
  virtual services::provided::Ack onSprrequirementUpdate(const services::provided::SPRRequirement_Service_Requirement& request) = 0;
  pcl_status_t sendSprrequirementTransition(const services::provided::SPRRequirement_Service_Requirement& item) {
    return spr_requirement_request_port_.transitionWriter().send(item);
  }
  services::provided::SprrequirementRequestPortProvider& sprRequirementRequestPort() {
    return spr_requirement_request_port_;
  }

private:
  AuthorisationDependencyHandlerAdapter authorisation_dependency_request_handler_;
  SprrequirementHandlerAdapter spr_requirement_request_handler_;
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
