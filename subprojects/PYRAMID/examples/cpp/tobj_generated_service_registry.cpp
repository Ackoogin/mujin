#include "tobj_generated_service_registry.hpp"

#include <cstdlib>
#include <cstdint>
#include <utility>

namespace tobj_example {

GeneratedServiceRegistry::GeneratedServiceRegistry(
    pcl::Component& owner,
    svc::ServiceHandler& handler,
    std::string content_type)
    : owner_(owner),
      handler_(handler),
      content_type_(std::move(content_type)) {}

bool GeneratedServiceRegistry::addRemote(const char* service_name,
                                         svc::ServiceChannel channel,
                                         const char* peer_id) {
  // Each service port carries the generated service channel needed to dispatch
  // the raw request buffer to the correct typed ServiceHandler method.
  bindings_.push_back(Binding{&handler_, channel, content_type_, {}});
  Binding& binding = bindings_.back();

  pcl::Port port = owner_.addService(
      service_name,
      content_type_.c_str(),
      &GeneratedServiceRegistry::dispatch,
      &binding);
  if (!port) {
    bindings_.pop_back();
    return false;
  }

  ports_.push_back(port);
  return ports_.back().routeRemote(peer_id) == PCL_OK;
}

pcl_status_t GeneratedServiceRegistry::dispatch(pcl_container_t*,
                                                const pcl_msg_t* request,
                                                pcl_msg_t* response,
                                                pcl_svc_context_t*,
                                                void* user_data) {
  auto* binding = static_cast<Binding*>(user_data);
  if (!binding || !binding->handler || !response) {
    return PCL_ERR_INVALID;
  }

  // Generated dispatch owns decoding, handler invocation, and response
  // encoding. Component business logic remains typed and codec-agnostic.
  void* response_buf = nullptr;
  size_t response_size = 0;
  svc::dispatch(
      *binding->handler,
      binding->channel,
      request ? request->data : nullptr,
      request ? request->size : 0u,
      request && request->type_name ? request->type_name
                                    : binding->content_type.c_str(),
      &response_buf,
      &response_size);

  // PCL expects response->data to stay valid until the immediate callback path
  // consumes it, so the binding owns storage for its latest response.
  binding->response_storage.clear();
  if (response_buf && response_size > 0u) {
    binding->response_storage.assign(static_cast<const char*>(response_buf),
                                     response_size);
    std::free(response_buf);
  }

  response->data = binding->response_storage.empty()
                       ? nullptr
                       : const_cast<char*>(binding->response_storage.data());
  response->size = static_cast<uint32_t>(binding->response_storage.size());
  response->type_name = request && request->type_name
                            ? request->type_name
                            : binding->content_type.c_str();
  return PCL_OK;
}

}  // namespace tobj_example
