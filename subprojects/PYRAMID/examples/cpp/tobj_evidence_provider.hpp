// tobj_evidence_provider.hpp
//
// Evidence provider component: subscribes to evidence_requirements,
// publishes typed observations via standard.object_evidence.
// Uses generated service bindings and proto-native data-model codecs.
//
// Architecture: main > TobjEvidenceProvider (this) > service binding > PCL
#pragma once

#include "generated/pyramid_services_tactical_objects_provided.hpp"
#include "generated/pyramid_services_tactical_objects_consumed.hpp"
#include "generated/pyramid_data_model_tactical_codec.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <atomic>

namespace tobj_example {

namespace Provided  = pyramid::services::tactical_objects::provided;
namespace Consumed  = pyramid::services::tactical_objects::consumed;
namespace TacticalCodec = pyramid::data_model::tactical;
using namespace pyramid::services::tactical_objects;
using namespace pyramid::data_model;

/// \brief Shared state visible to the e2e driver.
struct EvidenceProviderState {
    pcl_executor_t*      executor          = nullptr;
    pcl_port_t*          publisher         = nullptr;
    std::atomic<bool>    evidence_req_received{false};
    std::atomic<bool>    observation_sent{false};

    void reset() {
        evidence_req_received.store(false);
        observation_sent.store(false);
    }
};

/// \brief PCL on_configure callback: subscribes to evidence_requirements,
///        registers the object_evidence publisher.
pcl_status_t evidenceProviderOnConfigure(pcl_container_t* c, void* user_data);

/// \brief PCL subscriber callback for standard.evidence_requirements.
void onEvidenceRequirement(pcl_container_t* c, const pcl_msg_t* msg,
                           void* user_data);

} // namespace tobj_example
