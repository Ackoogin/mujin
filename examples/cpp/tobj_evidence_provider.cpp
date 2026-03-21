// tobj_evidence_provider.cpp
//
// Component body: subscribes to evidence_requirements, publishes observations.
// Uses generated service bindings and JsonCodec for all serialisation.

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
    Provided::subscribeEvidenceRequirements(c, onEvidenceRequirement, user_data);
    return PCL_OK;
}

// -- onEvidenceRequirement ----------------------------------------------------

void onEvidenceRequirement(pcl_container_t* /*c*/, const pcl_msg_t* msg,
                           void* user_data) {
    auto* state = static_cast<EvidenceProviderState*>(user_data);
    if (!msg || !msg->data || msg->size == 0) return;

    state->evidence_req_received.store(true);

    std::string body = Provided::msgToString(msg->data, msg->size);
    log(("evidence requirement: " + body).c_str());

    auto req = JsonCodec::evidenceRequirementFromJson(body);
    (void)req;  // parsed for completeness; not needed to build the response

    if (!state->observation_sent.load() && state->executor) {
        // Business logic: publish a typed observation.
        // Position: 51.0°N 0.0°E; HOSTILE; SEA_SURFACE dimension.
        JsonCodec::ObjectEvidence obs;
        obs.identity      = StandardIdentity::Hostile;
        obs.dimension     = BattleDimension::SeaSurface;
        obs.latitude_rad  = 51.0 * kDegToRad;
        obs.longitude_rad = 0.0;
        obs.confidence    = 0.9;
        obs.observed_at   = 0.5;

        std::string obs_json = JsonCodec::toJson(obs);
        log(("Publishing standard observation on "
             + std::string(Consumed::kTopicObjectEvidence)).c_str());

        // Publish via the standard.object_evidence topic using the executor
        // as an in-process publisher (dispatch_incoming re-uses the topic).
        pcl_msg_t out{};
        out.data      = obs_json.data();
        out.size      = static_cast<uint32_t>(obs_json.size());
        out.type_name = "application/json";
        pcl_executor_dispatch_incoming(state->executor,
                                       Consumed::kTopicObjectEvidence, &out);
        state->observation_sent.store(true);
        log("Standard observation published");
    }
}

} // namespace tobj_example
