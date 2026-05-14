// PCL component that consumes generated Tactical Objects services.
//
// The client side is already concise in the standard generated C++ binding:
// call typed invoke* helpers, then decode typed responses in the callback.
#pragma once

#include "pyramid_services_tactical_objects_provided.hpp"

#include <pcl/component.hpp>
#include <pcl/pcl_executor.h>

#include <atomic>
#include <string>
#include <vector>

namespace tobj_example {

namespace svc = pyramid::components::tactical_objects::services::provided;
namespace model = pyramid::domain_model;

class ObjectInterestClientComponent final : public pcl::Component {
public:
  explicit ObjectInterestClientComponent(std::string content_type);

  bool runCreateReadDelete(pcl_executor_t* executor);

protected:
  pcl_status_t on_configure() override;

private:
  struct ResponseState {
    std::atomic<bool> done{false};
    bool decoded = false;
    model::Identifier id;
    std::vector<model::ObjectInterestRequirement> requirements;
    model::Ack ack{};
  };

  bool waitForResponse(pcl_executor_t* executor, ResponseState& state);
  bool invokeCreate(pcl_executor_t* executor,
                    const model::ObjectInterestRequirement& request,
                    model::Identifier* out);
  bool invokeRead(pcl_executor_t* executor,
                  const model::Query& request,
                  std::vector<model::ObjectInterestRequirement>* out);
  bool invokeDelete(pcl_executor_t* executor,
                    const model::Identifier& request,
                    model::Ack* out);

  std::string content_type_;
};

}  // namespace tobj_example
