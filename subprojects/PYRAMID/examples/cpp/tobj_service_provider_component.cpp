#include "tobj_service_provider_component.hpp"

#include <utility>

namespace tobj_example {

ObjectInterestServiceComponent::ObjectInterestServiceComponent(
    std::string content_type,
    DemoObjectInterestHandler& handler)
    : pcl::Component("object_interest_service"),
      content_type_(std::move(content_type)),
      services_(*this, handler, content_type_),
      handler_(handler) {
  setTickRateHz(100.0);
}

pcl_status_t ObjectInterestServiceComponent::on_configure() {
  if (!svc::supportsContentType(content_type_.c_str())) {
    return PCL_ERR_INVALID;
  }

  if (!services_.addRemote(
          svc::kSvcObjectOfInterestCreateRequirement,
          svc::ServiceChannel::ObjectOfInterestCreateRequirement,
          "client")) {
    return PCL_ERR_NOMEM;
  }
  // read_requirement is a server-streaming RPC (returns stream Xxx in proto).
  if (!services_.addRemoteStream(
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

pcl_status_t ObjectInterestServiceComponent::on_tick(double) {
  // Calling pcl_stream_end from inside a stream handler would free the context
  // before the PCL executor finishes with it. The handler defers the call here.
  handler_.flushPendingStreamEnd();
  return PCL_OK;
}

}  // namespace tobj_example
