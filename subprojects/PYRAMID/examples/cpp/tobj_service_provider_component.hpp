// PCL component that provides generated Tactical Objects services.
//
// The component owns lifecycle, routing, and stream-end flushing. Business
// logic lives in DemoObjectInterestHandler. Serialization is in the generated
// binding. The concrete handler type is needed here so on_tick can flush
// deferred pcl_stream_end calls (pcl_stream_end frees the context so it cannot
// be called from inside the PCL stream handler callback).
#pragma once

#include "tobj_generated_service_registry.hpp"
#include "tobj_service_binding_handler.hpp"

#include <pcl/component.hpp>

#include <string>

namespace tobj_example {

class ObjectInterestServiceComponent final : public pcl::Component {
public:
  ObjectInterestServiceComponent(std::string content_type,
                                 DemoObjectInterestHandler& handler);

protected:
  pcl_status_t on_configure() override;
  pcl_status_t on_tick(double dt) override;

private:
  std::string content_type_;
  GeneratedServiceRegistry services_;
  DemoObjectInterestHandler& handler_;
};

}  // namespace tobj_example
