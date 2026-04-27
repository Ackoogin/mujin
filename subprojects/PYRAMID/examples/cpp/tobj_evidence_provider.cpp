// tobj_evidence_provider.cpp
//
// Component body: subscribes to evidence_requirements, publishes observations.
// Uses generated service bindings and proto-native data-model codecs.

#include "tobj_evidence_provider.hpp"

#include <cstdio>
#include <cmath>
#include <string>

namespace tobj_example {

static constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

static void log(const char* msg) {
    std::fprintf(stderr, "[evidence_provider] %s\n", msg);
    std::fflush(stderr);
}

// -- on_configure -------------------------------------------------------------

pcl_status_t evidenceProviderOnConfigure(pcl_container_t* c, void* user_data) {
    auto* state = static_cast<EvidenceProviderState*>(user_data);
    Provided::subscribeEvidenceRequirements(
        c, onEvidenceRequirement, user_data, state->content_type.c_str());
    state->publisher = pcl_container_add_publisher(
        c, Consumed::kTopicObjectEvidence, state->content_type.c_str());
    return state->publisher ? PCL_OK : PCL_ERR_CALLBACK;
}

// -- onEvidenceRequirement ----------------------------------------------------

void onEvidenceRequirement(pcl_container_t* /*c*/, const pcl_msg_t* msg,
                           void* user_data) {
    auto* state = static_cast<EvidenceProviderState*>(user_data);
    if (!msg || !msg->data || msg->size == 0) return;

    state->evidence_req_received.store(true);

    ObjectEvidenceRequirement req;
    if (!Provided::decodeEvidenceRequirements(msg, &req)) {
        return;
    }
    (void)req;  // parsed for completeness; not needed to build the response

    if (!state->observation_sent.load() && state->publisher) {
        // Business logic: publish a typed observation.
        // Position: 51.0 degN 0.0 degE; HOSTILE; SEA_SURFACE dimension.
        ObjectDetail obs;
        obs.id            = "obj-1";
        obs.identity      = StandardIdentity::Hostile;
        obs.dimension     = BattleDimension::SeaSurface;
        obs.position.latitude  = 51.0 * kDegToRad;
        obs.position.longitude = 0.0;
        obs.quality       = 0.9;
        obs.creation_time = 0.5;

        log(("Publishing standard observation on "
             + std::string(Consumed::kTopicObjectEvidence)).c_str());

        Consumed::publishObjectEvidence(
            state->publisher, obs, state->content_type.c_str());
        state->observation_sent.store(true);
        log("Standard observation published");
    }
}

} // namespace tobj_example
