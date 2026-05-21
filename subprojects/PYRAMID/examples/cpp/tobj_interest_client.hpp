// TobjInterestClient -- pcl::Component that exercises the Tactical Objects
// provided services as a client.
//
// Subscribes to standard.entity_matches (pub/sub) and issues
// object_of_interest.create_requirement (RPC) via a ConsumedService binding.
//
// Composes one generated ConsumedService binding so the RPC is fully typed
// and async-shaped. The entity-matches subscription uses the standard PCL
// publisher/subscriber path (it is not RPC).
#pragma once

#include "pyramid_services_tactical_objects_provided.hpp"
#include "pyramid_services_tactical_objects_provided_components.hpp"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

#include <future>
#include <string>

namespace tobj_example {

namespace Provided = pyramid::components::tactical_objects::services::provided;
using namespace pyramid::domain_model;

class TobjInterestClient : public pcl::Component {
public:
  TobjInterestClient(pcl::Executor& executor,
                     std::string content_type = Provided::kJsonContentType)
      : pcl::Component("cpp_client"),
        content_type_(std::move(content_type)),
        consumer_(*this, executor, content_type_) {}

  std::future<Provided::Result<Identifier>>
  createRequirementAsync(const ObjectInterestRequirement& request) {
    return consumer_.objectOfInterestCreateRequirementAsync(request);
  }

  int  matchesReceived()  const { return matches_received_; }
  bool foundHostileObject() const { return found_hostile_; }

protected:
  pcl_status_t on_configure() override {
    Provided::subscribeEntityMatches(
        handle(), &TobjInterestClient::trampolineEntityMatches, this,
        content_type_.c_str());
    return PCL_OK;
  }

private:
  static void trampolineEntityMatches(pcl_container_t*, const pcl_msg_t* msg,
                                       void* user_data) {
    static_cast<TobjInterestClient*>(user_data)->onEntityMatches(msg);
  }

  void onEntityMatches(const pcl_msg_t* msg);

  std::string                content_type_;
  Provided::ConsumedService  consumer_;
  int                        matches_received_ = 0;
  bool                       found_hostile_    = false;
};

/// \brief Build an ObjectInterestRequirement for the standard ActiveFind
///        flow (point or bounding-box query in the given dimension).
ObjectInterestRequirement makeActiveFindRequirement(
    DataPolicy       policy,
    BattleDimension  dimension,
    double           min_lat_rad,
    double           max_lat_rad,
    double           min_lon_rad,
    double           max_lon_rad);

} // namespace tobj_example
