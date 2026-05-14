// Small C++ helper that bridges generated PYRAMID service dispatch to PCL.
//
// Current C++ generated bindings expose ServiceHandler + dispatch(...), while
// PCL exposes Component::addService(...). Unlike the Ada generated bindings,
// there is not currently a generated C++ Register_Services helper. This
// example-local helper keeps the PCL callback boilerplate in one place.
#pragma once

#include "pyramid_services_tactical_objects_provided.hpp"

#include <pcl/component.hpp>

#include <list>
#include <string>
#include <vector>

namespace tobj_example {

namespace svc = pyramid::components::tactical_objects::services::provided;

class GeneratedServiceRegistry {
public:
  GeneratedServiceRegistry(pcl::Component& owner,
                           svc::ServiceHandler& handler,
                           std::string content_type);

  // Register one generated service operation as a PCL service port.
  // The returned port is routed to the named remote peer.
  bool addRemote(const char* service_name,
                 svc::ServiceChannel channel,
                 const char* peer_id);

private:
  struct Binding {
    svc::ServiceHandler* handler = nullptr;
    svc::ServiceChannel channel =
        svc::ServiceChannel::ObjectOfInterestCreateRequirement;
    std::string content_type;
    std::string response_storage;
  };

  static pcl_status_t dispatch(pcl_container_t* container,
                               const pcl_msg_t* request,
                               pcl_msg_t* response,
                               pcl_svc_context_t* context,
                               void* user_data);

  pcl::Component& owner_;
  svc::ServiceHandler& handler_;
  std::string content_type_;

  // std::list keeps Binding addresses stable after subsequent registrations;
  // PCL stores the Binding pointer as the service callback user_data.
  std::list<Binding> bindings_;
  std::vector<pcl::Port> ports_;
};

}  // namespace tobj_example
