// tobj_interest_client.cpp
//
// Component body: subscribes to entity_matches, invokes create_requirement.
// Uses generated service bindings and JsonCodec for all serialisation.

#include "tobj_interest_client.hpp"

#include <cstdio>
#include <string>

namespace tobj_example {

static void log(const char* msg) {
    std::fprintf(stderr, "[interest_client] %s\n", msg);
    std::fflush(stderr);
}

// -- on_configure -------------------------------------------------------------

pcl_status_t interestClientOnConfigure(pcl_container_t* c, void* user_data) {
    Provided::subscribeEntityMatches(c, onEntityMatches, user_data);
    return PCL_OK;
}

// -- onEntityMatches ----------------------------------------------------------

void onEntityMatches(pcl_container_t* /*c*/, const pcl_msg_t* msg,
                     void* user_data) {
    auto* state = static_cast<InterestClientState*>(user_data);
    if (!msg || !msg->data || msg->size == 0) return;

    std::string body = Provided::msgToString(msg->data, msg->size);
    log(("standard.entity_matches: " + body).c_str());

    auto matches = JsonCodec::entityMatchesFromJson(body);
    for (const auto& m : matches) {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "  entity id=%s identity=%s",
                      m.object_id.c_str(),
                      JsonCodec::toString(m.identity).c_str());
        log(buf);
        state->matches_received.fetch_add(1);
        if (m.identity == StandardIdentity::Hostile) {
            state->found_hostile_entity.store(true);
        }
    }
}

// -- onCreateRequirementResponse ----------------------------------------------

void onCreateRequirementResponse(const pcl_msg_t* resp, void* user_data) {
    auto* state = static_cast<InterestClientState*>(user_data);
    if (resp && resp->data && resp->size > 0) {
        std::string body = Provided::msgToString(resp->data, resp->size);
        log(("create_requirement response: " + body).c_str());
        auto r = JsonCodec::createRequirementResponseFromJson(body);
        if (!r.interest_id.empty()) {
            state->interest_id_received.store(true);
        }
    }
    state->svc_response_ready.store(true);
}

// -- sendCreateRequirement ----------------------------------------------------

pcl_status_t sendCreateRequirement(
    pcl_socket_transport_t* transport,
    InterestClientState*    state,
    DataPolicy              policy,
    StandardIdentity        identity,
    BattleDimension         dimension,
    double                  min_lat_rad,
    double                  max_lat_rad,
    double                  min_lon_rad,
    double                  max_lon_rad)
{
    JsonCodec::CreateRequirementRequest req;
    req.policy      = policy;
    req.identity    = identity;
    req.dimension   = dimension;
    req.min_lat_rad = min_lat_rad;
    req.max_lat_rad = max_lat_rad;
    req.min_lon_rad = min_lon_rad;
    req.max_lon_rad = max_lon_rad;

    std::string req_str = JsonCodec::toJson(req);
    log(("create_requirement request: " + req_str).c_str());

    return Provided::invokeCreateRequirement(
        transport, req_str, onCreateRequirementResponse, state);
}

} // namespace tobj_example
