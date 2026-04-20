// tobj_interest_client.cpp
//
// Component body: subscribes to entity_matches, invokes create_requirement.
// Uses generated service bindings and proto-native data-model codecs.

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
    auto* state = static_cast<InterestClientState*>(user_data);
    Provided::subscribeEntityMatches(
        c, onEntityMatches, user_data,
        state ? state->content_type.c_str() : Provided::kJsonContentType);
    return PCL_OK;
}

// -- onEntityMatches ----------------------------------------------------------

void onEntityMatches(pcl_container_t* /*c*/, const pcl_msg_t* msg,
                     void* user_data) {
    auto* state = static_cast<InterestClientState*>(user_data);
    if (!msg || !msg->data || msg->size == 0) return;

    std::vector<ObjectMatch> matches;
    if (!Provided::decodeEntityMatches(msg, &matches)) {
        return;
    }
    log("standard.entity_matches received");
    for (const auto& m : matches) {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "  entity id=%s matching_object_id=%s",
                      m.id.c_str(),
                      m.matching_object_id.c_str());
        log(buf);
        state->matches_received.fetch_add(1);
        if (!m.matching_object_id.empty()) {
            state->found_hostile_entity.store(true);
        }
    }
}

// -- onCreateRequirementResponse ----------------------------------------------

void onCreateRequirementResponse(const pcl_msg_t* resp, void* user_data) {
    auto* state = static_cast<InterestClientState*>(user_data);
    if (resp && resp->data && resp->size > 0) {
        Identifier interest_id;
        if (Provided::decodeCreateRequirementResponse(resp, &interest_id) &&
            !interest_id.empty()) {
            log(("create_requirement response id: " + interest_id).c_str());
            state->interest_id_received.store(true);
        }
    }
    state->svc_response_ready.store(true);
}

// -- sendCreateRequirement ----------------------------------------------------

pcl_status_t sendCreateRequirement(
    pcl_executor_t*         executor,
    InterestClientState*    state,
    const char*             content_type,
    DataPolicy              policy,
    StandardIdentity        identity,
    BattleDimension         dimension,
    double                  min_lat_rad,
    double                  max_lat_rad,
    double                  min_lon_rad,
    double                  max_lon_rad)
{
    (void)identity;

    ObjectInterestRequirement req;
    req.policy = policy;
    req.dimension.push_back(dimension);
    if (min_lat_rad == max_lat_rad && min_lon_rad == max_lon_rad) {
        common::Point point{};
        point.position.latitude = min_lat_rad;
        point.position.longitude = min_lon_rad;
        req.point = point;
    } else {
        common::PolyArea poly{};
        poly.points.push_back({min_lat_rad, min_lon_rad});
        poly.points.push_back({min_lat_rad, max_lon_rad});
        poly.points.push_back({max_lat_rad, max_lon_rad});
        poly.points.push_back({max_lat_rad, min_lon_rad});
        req.poly_area = poly;
    }

    log("create_requirement request (proto-native typed)");
    return Provided::invokeCreateRequirement(
        executor, req, onCreateRequirementResponse, state, nullptr,
        content_type);
}

} // namespace tobj_example
