#pragma once

/// \file pcl_msg_json.h
/// \brief PCL message serialisation helpers for AME.
///
/// All `ame_pack_*` functions return a JSON string.  Build a `pcl_msg_t` that
/// borrows from the string with `ame_make_pcl_msg()`.  The string buffer
/// **must outlive** the `pcl_msg_t`:
///
/// \code
/// std::string json = ame_pack_get_fact_request("(at uav1 base)");
/// pcl_msg_t msg;
/// ame_make_pcl_msg(json, "ame/GetFact_Request", msg);
/// pcl_port_publish(port, &msg);
/// \endcode
///
/// Wire format: JSON ("application/json"), consistent with the PYRAMID
/// audit-log convention and PYRAMID StandardBridge.
///
/// All `ame_unpack_*` functions accept a `const pcl_msg_t*` and return the
/// decoded struct.  Unknown fields are silently ignored.

#include <ame/world_model_component.h>
#include <pcl/pcl_types.h>

#include <cstdint>
#include <string>
#include <vector>

namespace ame {

// ---------------------------------------------------------------------------
// Detection (PCL subscriber topic: "detections")
// Mirrors ame_ros2::msg::Detection without any ROS2 dependency.
// ---------------------------------------------------------------------------

struct Detection {
    float confidence = 0.0f;
    std::string sensor_source;
    std::string entity_id;
    std::vector<std::string> property_keys;
    std::vector<std::string> property_values;
};

// ---------------------------------------------------------------------------
// SetFact service request (service: "set_fact")
// ---------------------------------------------------------------------------

struct SetFactRequest {
    std::string key;
    bool value = false;
    std::string source;
};

// ---------------------------------------------------------------------------
// LoadDomain service request/response (service: "load_domain")
// ---------------------------------------------------------------------------

struct LoadDomainRequest {
    std::string domain_id;
    std::string domain_pddl;
    std::string problem_pddl;
};

struct LoadDomainResponse {
    bool success = false;
    std::string error_msg;
    uint32_t num_fluents = 0;
    uint32_t num_ground_actions = 0;
};

// ---------------------------------------------------------------------------
// Plan service request/response (service: "plan")
// ---------------------------------------------------------------------------

struct PlanRequest {
    std::vector<std::string> goal_fluents;
};

struct PlanResponse {
    bool success = false;
    std::string bt_xml;
    std::vector<std::string> plan_actions;
    double solve_time_ms = 0.0;
    std::string error_msg;
};

// ---------------------------------------------------------------------------
// DispatchGoals service request/response (service: "dispatch_goals")
// ---------------------------------------------------------------------------

struct DispatchGoalsRequest {
    std::vector<std::string> goal_fluents;
    std::vector<std::string> agent_ids;
};

struct DispatchGoalsResponse {
    bool success = false;
    std::vector<std::string> dispatched_agents;
    std::string error_msg;
};

// ---------------------------------------------------------------------------
// Utility: fill a pcl_msg_t that borrows from a string buffer
// ---------------------------------------------------------------------------

/// \brief Fill \p out so it references \p buf.  \p buf must outlive \p out.
inline void ame_make_pcl_msg(const std::string& buf,
                              const char* type_name,
                              pcl_msg_t& out) noexcept {
    out.data      = buf.c_str();
    out.size      = static_cast<uint32_t>(buf.size());
    out.type_name = type_name;
}

/// \brief Extract string content from a pcl_msg_t (plain-string messages).
inline std::string ame_msg_to_string(const pcl_msg_t* msg) {
    if (!msg || !msg->data || msg->size == 0) {
        return {};
    }
    return std::string(static_cast<const char*>(msg->data), msg->size);
}

// ---------------------------------------------------------------------------
// WorldState  (topic: "world_state")
// ---------------------------------------------------------------------------

std::string ame_pack_world_state(const WorldStateSnapshot& snap);
WorldStateSnapshot ame_unpack_world_state(const pcl_msg_t* msg);

// ---------------------------------------------------------------------------
// Detection  (topic: "detections")
// ---------------------------------------------------------------------------

std::string ame_pack_detection(const Detection& det);
Detection ame_unpack_detection(const pcl_msg_t* msg);

// ---------------------------------------------------------------------------
// GetFact service  (service: "get_fact")
// ---------------------------------------------------------------------------

std::string ame_pack_get_fact_request(const std::string& key);
std::string ame_pack_get_fact_response(const GetFactResult& result);
std::string ame_unpack_get_fact_request_key(const pcl_msg_t* msg);
GetFactResult ame_unpack_get_fact_response(const pcl_msg_t* msg);

// ---------------------------------------------------------------------------
// SetFact service  (service: "set_fact")
// ---------------------------------------------------------------------------

std::string ame_pack_set_fact_request(const SetFactRequest& req);
std::string ame_pack_set_fact_response(const SetFactResult& result);
SetFactRequest ame_unpack_set_fact_request(const pcl_msg_t* msg);
SetFactResult ame_unpack_set_fact_response(const pcl_msg_t* msg);

// ---------------------------------------------------------------------------
// QueryState service  (service: "query_state")
// ---------------------------------------------------------------------------

std::string ame_pack_query_state_request(const std::vector<std::string>& keys);
std::string ame_pack_query_state_response(const WorldStateSnapshot& snap);
std::vector<std::string> ame_unpack_query_state_request_keys(const pcl_msg_t* msg);
WorldStateSnapshot ame_unpack_query_state_response(const pcl_msg_t* msg);

// ---------------------------------------------------------------------------
// LoadDomain service  (service: "load_domain")
// ---------------------------------------------------------------------------

std::string ame_pack_load_domain_request(const LoadDomainRequest& req);
std::string ame_pack_load_domain_response(const LoadDomainResponse& resp);
LoadDomainRequest ame_unpack_load_domain_request(const pcl_msg_t* msg);
LoadDomainResponse ame_unpack_load_domain_response(const pcl_msg_t* msg);

// ---------------------------------------------------------------------------
// Plan service  (service: "plan")
// ---------------------------------------------------------------------------

std::string ame_pack_plan_request(const PlanRequest& req);
std::string ame_pack_plan_response(const PlanResponse& resp);
PlanRequest ame_unpack_plan_request(const pcl_msg_t* msg);
PlanResponse ame_unpack_plan_response(const pcl_msg_t* msg);

// ---------------------------------------------------------------------------
// DispatchGoals service  (service: "dispatch_goals")
// ---------------------------------------------------------------------------

std::string ame_pack_dispatch_goals_request(const DispatchGoalsRequest& req);
std::string ame_pack_dispatch_goals_response(const DispatchGoalsResponse& resp);
DispatchGoalsRequest ame_unpack_dispatch_goals_request(const pcl_msg_t* msg);
DispatchGoalsResponse ame_unpack_dispatch_goals_response(const pcl_msg_t* msg);

}  // namespace ame
