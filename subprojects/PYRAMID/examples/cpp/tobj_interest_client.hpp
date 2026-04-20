// tobj_interest_client.hpp
//
// Interest client component: subscribes to entity_matches, sends
// create_requirement via the standard bridge.
// Uses generated service bindings and proto-native data-model codecs.
//
// Architecture: main > TobjInterestClient (this) > service binding > PCL
#pragma once

#include "generated/pyramid_services_tactical_objects_provided.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <atomic>
#include <string>

namespace tobj_example {

namespace Provided  = pyramid::services::tactical_objects::provided;
using namespace pyramid::services::tactical_objects;
using namespace pyramid::data_model;

/// \brief Shared state visible to the e2e driver.
struct InterestClientState {
    std::string       content_type{Provided::kJsonContentType};
    std::atomic<int>  matches_received{0};
    std::atomic<bool> found_hostile_entity{false};
    std::atomic<bool> svc_response_ready{false};
    std::atomic<bool> interest_id_received{false};

    void reset() {
        matches_received.store(0);
        found_hostile_entity.store(false);
        svc_response_ready.store(false);
        interest_id_received.store(false);
    }
};

/// \brief PCL on_configure callback: subscribes to entity_matches.
pcl_status_t interestClientOnConfigure(pcl_container_t* c, void* user_data);

/// \brief PCL subscriber callback for standard.entity_matches.
void onEntityMatches(pcl_container_t* c, const pcl_msg_t* msg, void* user_data);

/// \brief Async response callback for invokeCreateRequirement.
void onCreateRequirementResponse(const pcl_msg_t* resp, void* user_data);

/// \brief Build and send a create_requirement via the standard bridge.
///
/// Business logic (which enums to use, bounding box) lives in the caller;
/// all wire serialisation is handled by the generated service binding.
pcl_status_t sendCreateRequirement(
    pcl_executor_t*         executor,
    InterestClientState*    state,
    const char*             content_type,
    DataPolicy              policy,
    StandardIdentity        identity,
    BattleDimension         dimension  = BattleDimension::Unspecified,
    double                  min_lat_rad = 0.0,
    double                  max_lat_rad = 0.0,
    double                  min_lon_rad = 0.0,
    double                  max_lon_rad = 0.0);

} // namespace tobj_example
