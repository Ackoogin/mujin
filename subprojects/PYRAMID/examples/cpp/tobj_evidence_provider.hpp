// TobjEvidenceProvider -- pcl::Component that drives the standard evidence
// flow as a publisher.
//
// Subscribes to standard.evidence_requirements and publishes a typed
// ObjectDetail observation on standard.object_evidence the first time a
// requirement arrives. Pure pub/sub: no service binding required.
#pragma once

#include "pyramid_services_tactical_objects_consumed.hpp"
#include "pyramid_services_tactical_objects_provided.hpp"

#include <pcl/component.hpp>

#include <string>

namespace tobj_example {

namespace Provided = pyramid::components::tactical_objects::services::provided;
namespace Consumed = pyramid::components::tactical_objects::services::consumed;
using namespace pyramid::domain_model;

class TobjEvidenceProvider : public pcl::Component {
public:
  explicit TobjEvidenceProvider(std::string content_type =
                                    Provided::kJsonContentType)
      : pcl::Component("cpp_provider"),
        content_type_(std::move(content_type)) {}

  bool evidenceRequirementReceived() const { return evidence_req_received_; }
  bool observationSent() const             { return observation_sent_; }

protected:
  pcl_status_t on_configure() override {
    Provided::subscribeEvidenceRequirements(
        handle(), &TobjEvidenceProvider::trampolineEvidence, this,
        content_type_.c_str());
    publisher_ = pcl_container_add_publisher(
        handle(), Consumed::kTopicObjectEvidence, content_type_.c_str());
    return publisher_ ? PCL_OK : PCL_ERR_CALLBACK;
  }

private:
  static void trampolineEvidence(pcl_container_t*, const pcl_msg_t* msg,
                                  void* user_data) {
    static_cast<TobjEvidenceProvider*>(user_data)->onEvidenceRequirement(msg);
  }

  void onEvidenceRequirement(const pcl_msg_t* msg);

  std::string  content_type_;
  pcl_port_t*  publisher_              = nullptr;
  bool         evidence_req_received_  = false;
  bool         observation_sent_       = false;
};

} // namespace tobj_example
