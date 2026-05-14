#include "tobj_service_provider_component.hpp"

#include <utility>

namespace tobj_example {

ObjectInterestServiceComponent::ObjectInterestServiceComponent(
    std::string content_type,
    svc::ServiceHandler& handler)
    : pcl::Component("object_interest_service"),
      content_type_(std::move(content_type)),
      services_(*this, handler, content_type_) {}

pcl_status_t ObjectInterestServiceComponent::on_configure() {
  if (!svc::supportsContentType(content_type_.c_str())) {
    return PCL_ERR_INVALID;
  }

  // Register the three generated service operations needed for this example.
  // The registry hides the raw PCL callback shape and forwards to svc::dispatch.
  if (!services_.addRemote(
          svc::kSvcObjectOfInterestCreateRequirement,
          svc::ServiceChannel::ObjectOfInterestCreateRequirement,
          "client")) {
    return PCL_ERR_NOMEM;
  }
  if (!services_.addRemote(
          svc::kSvcObjectOfInterestReadRequirement,
          svc::ServiceChannel::ObjectOfInterestReadRequirement,
          "client")) {
    return PCL_ERR_NOMEM;
  }
  if (!services_.addRemote(
          svc::kSvcObjectOfInterestDeleteRequirement,
          svc::ServiceChannel::ObjectOfInterestDeleteRequirement,
          "client")) {
    return PCL_ERR_NOMEM;
  }

  return PCL_OK;
}

}  // namespace tobj_example
