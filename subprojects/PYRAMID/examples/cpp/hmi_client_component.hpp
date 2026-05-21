// Hand-written PYRAMID component that composes one generated ConsumedService
// binding (tactical_objects.services.provided) to drive the Tactical Objects
// services as a client. Exposes typed accessors used by the showcase's main
// loop.
#pragma once

#include "pyramid_services_tactical_objects_provided_components.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

#include <functional>
#include <future>
#include <string>
#include <string_view>

namespace tobj_example {

namespace ProvidedSvc =
    pyramid::components::tactical_objects::services::provided;
namespace model = pyramid::domain_model;

class HmiClientComponent : public pcl::Component {
public:
  HmiClientComponent(pcl::Executor& executor,
                     std::string content_type =
                         ProvidedSvc::kJsonContentType)
      : pcl::Component("hmi_client"),
        tobj_consumed_(*this, executor, std::move(content_type)) {}

  /// \brief Route every consumed RPC to the executor's default transport.
  pcl_status_t routeProvidedDefault() {
    return tobj_consumed_.routeAllRemote();
  }

  /// \brief Route every consumed RPC to a named peer.
  pcl_status_t routeProvidedTo(std::string_view peer_id) {
    return tobj_consumed_.routeAllRemote(peer_id);
  }

  // -- Typed accessors used by the showcase ---------------------------------

  std::future<ProvidedSvc::Result<model::Identifier>>
  createRequirementAsync(const model::ObjectInterestRequirement& req) {
    return tobj_consumed_.objectOfInterestCreateRequirementAsync(req);
  }

  std::future<ProvidedSvc::Result<model::Ack>>
  deleteRequirementAsync(const model::Identifier& id) {
    return tobj_consumed_.objectOfInterestDeleteRequirementAsync(id);
  }

  ProvidedSvc::StreamHandle
  streamReadRequirement(
      const model::Query& query,
      std::function<void(const model::ObjectInterestRequirement&)> on_frame,
      std::function<void(pcl_status_t)> on_end = {}) {
    return tobj_consumed_.objectOfInterestReadRequirementStreaming(
        query, std::move(on_frame), std::move(on_end));
  }

private:
  ProvidedSvc::ConsumedService tobj_consumed_;
};

}  // namespace tobj_example
