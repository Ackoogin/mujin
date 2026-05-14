// PCL component that provides generated Tactical Objects services.
//
// This file shows the component-facing registration surface. The component
// owns lifecycle and routing; the handler owns business logic; the generated
// binding owns serialization.
#pragma once

#include "tobj_generated_service_registry.hpp"

#include <pcl/component.hpp>

#include <string>

namespace tobj_example {

class ObjectInterestServiceComponent final : public pcl::Component {
public:
  ObjectInterestServiceComponent(std::string content_type,
                                 svc::ServiceHandler& handler);

protected:
  pcl_status_t on_configure() override;

private:
  std::string content_type_;
  GeneratedServiceRegistry services_;
};

}  // namespace tobj_example
