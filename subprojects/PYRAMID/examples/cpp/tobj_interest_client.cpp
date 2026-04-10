// tobj_interest_client.cpp
//
// Component body: subscribes to entity_matches, invokes create_requirement.
// Uses generated service bindings and proto-native data-model codecs.

#include "tobj_interest_client.hpp"

#include <nlohmann/json.hpp>

#include <cstdio>
#include <cstring>
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

    std::vector<ObjectMatch> matches;
    try {
        auto arr = nlohmann::json::parse(body);
        if (!arr.is_array()) {
            return;
        }
        for (const auto& item : arr) {
            matches.push_back(TacticalCodec::fromJson(
                item.dump(), static_cast<ObjectMatch*>(nullptr)));
        }
    } catch (...) {
        return;
    }
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
        std::string body = Provided::msgToString(resp->data, resp->size);
        log(("create_requirement response: " + body).c_str());
        try {
            auto parsed = nlohmann::json::parse(body);
            if (parsed.is_string() && !parsed.get<std::string>().empty()) {
                state->interest_id_received.store(true);
            }
        } catch (...) {
        }
        if (state->interest_id_received.load()) {
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

    std::string req_str = TacticalCodec::toJson(req);
    log(("create_requirement request: " + req_str).c_str());

    pcl_msg_t request_msg{};
    request_msg.data = req_str.data();
    request_msg.size = static_cast<uint32_t>(req_str.size());
    request_msg.type_name = "application/json";

    return pcl_socket_transport_invoke_remote_async(
        transport, Provided::kSvcCreateRequirement, &request_msg,
        onCreateRequirementResponse, state);
}

} // namespace tobj_example
