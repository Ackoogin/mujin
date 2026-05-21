// Hand-written PYRAMID component that composes one generated ProvidedService
// binding (tactical_objects.services.provided) with a user-supplied
// InterestStore handler. Represents how a real deployable component owns its
// service bindings as members and installs them onto itself in on_configure().
#pragma once

#include "tobj_interest_store.hpp"

#include "pyramid_services_tactical_objects_provided_components.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

#include <string>
#include <string_view>

namespace tobj_example {

namespace ProvidedSvc =
    pyramid::components::tactical_objects::services::provided;

class TacticalObjectsComponent : public pcl::Component {
public:
  TacticalObjectsComponent(pcl::Executor& executor,
                           std::string content_type =
                               ProvidedSvc::kJsonContentType)
      : pcl::Component("tactical_objects"),
        tobj_provided_(*this, executor, store_, std::move(content_type)) {}

  /// \brief Restrict every provided RPC to a single peer.
  pcl_status_t routeProvidedTo(std::string_view peer_id) {
    return tobj_provided_.routeAllRemote(peer_id);
  }

  /// \brief Direct access to the in-memory store for assertions in tests.
  InterestStore& store() { return store_; }
  const InterestStore& store() const { return store_; }

protected:
  pcl_status_t on_configure() override { return tobj_provided_.bind(); }

private:
  InterestStore store_;
  ProvidedSvc::ProvidedService tobj_provided_;
};

}  // namespace tobj_example
