// Auto-generated service binding implementation
// Generated from: provided.proto by generate_bindings.py
// Namespace: pyramid::services::tactical_objects::provided

#include "pyramid_services_tactical_objects_provided.hpp"

#include "pyramid_services_tactical_objects_json_codec.hpp"
#if __has_include("flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.hpp")
#include "flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.hpp"
#define PYRAMID_HAVE_SERVICE_FLATBUFFERS 1
#else
#define PYRAMID_HAVE_SERVICE_FLATBUFFERS 0
#endif
#include "pyramid_data_model_common_codec.hpp"
#include "pyramid_data_model_tactical_codec.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport.h>

#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::provided {

// Bring data model codec functions into scope
using pyramid::data_model::common::toJson;
using pyramid::data_model::common::fromJson;
using pyramid::data_model::tactical::toJson;
using pyramid::data_model::tactical::fromJson;
namespace wire_types = pyramid::services::tactical_objects::wire_types;
namespace json_codec = pyramid::services::tactical_objects::json_codec;
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
namespace flatbuffers_codec = pyramid::services::tactical_objects::flatbuffers_codec;
#endif

// ---------------------------------------------------------------------------
// PCL message utility
// ---------------------------------------------------------------------------

std::string msgToString(const void* data, unsigned size) {
    return std::string(static_cast<const char*>(data), size);
}

// ---------------------------------------------------------------------------
// ServiceHandler — default stub implementations
// ---------------------------------------------------------------------------

std::vector<ObjectMatch>
ServiceHandler::handleReadMatch(const Query& /*request*/) {
    return {};
}

Identifier
ServiceHandler::handleCreateRequirement(const ObjectInterestRequirement& /*request*/) {
    return {};
}

std::vector<ObjectInterestRequirement>
ServiceHandler::handleReadRequirement(const Query& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleUpdateRequirement(const ObjectInterestRequirement& /*request*/) {
    return pyramid::data_model::kAckOk;
}

Ack
ServiceHandler::handleDeleteRequirement(const Identifier& /*request*/) {
    return pyramid::data_model::kAckOk;
}

std::vector<ObjectDetail>
ServiceHandler::handleReadDetail(const Query& /*request*/) {
    return {};
}

// ---------------------------------------------------------------------------
// Internal PCL helpers
// ---------------------------------------------------------------------------

namespace {

constexpr const char* kJsonContentType = "application/json";
constexpr const char* kFlatBuffersContentType = "application/flatbuffers";

bool is_json_content_type(const char* content_type)
{
    return !content_type || std::strcmp(content_type, kJsonContentType) == 0;
}

bool is_flatbuffers_content_type(const char* content_type)
{
    return content_type && std::strcmp(content_type, kFlatBuffersContentType) == 0;
}

std::string json_request_body(const void* data, size_t size)
{
    if (!data && size != 0) return {};
    return std::string(static_cast<const char*>(data), size);
}

ObjectInterestRequirement to_domain_create_requirement_request(
    const wire_types::CreateRequirementRequest& wire_req)
{
    ObjectInterestRequirement result{};
    result.policy = wire_req.policy;
    if (wire_req.dimension != pyramid::data_model::BattleDimension::Unspecified)
        result.dimension.push_back(wire_req.dimension);
    if (wire_req.min_lat_rad != 0.0 || wire_req.max_lat_rad != 0.0
        || wire_req.min_lon_rad != 0.0 || wire_req.max_lon_rad != 0.0) {
        pyramid::data_model::common::PolyArea area{};
        area.points.push_back({wire_req.min_lat_rad, wire_req.min_lon_rad});
        area.points.push_back({wire_req.min_lat_rad, wire_req.max_lon_rad});
        area.points.push_back({wire_req.max_lat_rad, wire_req.max_lon_rad});
        area.points.push_back({wire_req.max_lat_rad, wire_req.min_lon_rad});
        result.poly_area = area;
    }
    return result;
}

wire_types::CreateRequirementResponse to_wire_create_requirement_response(
    const Identifier& interest_id)
{
    wire_types::CreateRequirementResponse result{};
    result.interest_id = interest_id;
    return result;
}

pcl_status_t invoke_async(pcl_executor_t* executor,
                           const char*             service_name,
                           const std::string&      payload,
                           pcl_resp_cb_fn_t        callback,
                           void*                   user_data,
                           const pcl_endpoint_route_t* route,
                           const char*             content_type)
{
    pcl_msg_t msg{};
    msg.data      = payload.data();
    msg.size      = static_cast<uint32_t>(payload.size());
    msg.type_name = content_type;
    if (route) {
        const pcl_status_t route_rc = pcl_executor_set_endpoint_route(executor, route);
        if (route_rc != PCL_OK) {
            return route_rc;
        }
    }
    return pcl_executor_invoke_async(
        executor, service_name, &msg, callback, user_data);
}

} // namespace

// ---------------------------------------------------------------------------
// PCL subscribe wrappers
// ---------------------------------------------------------------------------

pcl_port_t* subscribeEntityMatches(pcl_container_t*   container,
                                   pcl_sub_callback_t  callback,
                                   void*              user_data,
                                   const char*        content_type)
{
    return pcl_container_add_subscriber(container,
                                 kTopicEntityMatches,
                                 content_type,
                                 callback,
                                 user_data);
}

pcl_port_t* subscribeEvidenceRequirements(pcl_container_t*   container,
                                          pcl_sub_callback_t  callback,
                                          void*              user_data,
                                          const char*        content_type)
{
    return pcl_container_add_subscriber(container,
                                 kTopicEvidenceRequirements,
                                 content_type,
                                 callback,
                                 user_data);
}

// ---------------------------------------------------------------------------
// Typed invoke wrappers — serialise and dispatch via executor transport
// ---------------------------------------------------------------------------

pcl_status_t invokeReadMatch(pcl_executor_t* executor,
                             const Query&                 request,
                             pcl_resp_cb_fn_t        callback,
                             void*                   user_data,
                             const pcl_endpoint_route_t* route,
                             const char*       content_type)
{
    std::string payload = toJson(request);
    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        payload = flatbuffers_codec::wrapPayload(payload);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    }
    return invoke_async(executor, kSvcReadMatch, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeCreateRequirement(pcl_executor_t* executor,
                                     const wire_types::CreateRequirementRequest& request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data,
                                     const pcl_endpoint_route_t* route,
                                     const char*       content_type)
{
    std::string payload = json_codec::toJson(request);
    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        payload = flatbuffers_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    }
    return invoke_async(executor, kSvcCreateRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeReadRequirement(pcl_executor_t* executor,
                                   const Query&                 request,
                                   pcl_resp_cb_fn_t        callback,
                                   void*                   user_data,
                                   const pcl_endpoint_route_t* route,
                                   const char*       content_type)
{
    std::string payload = toJson(request);
    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        payload = flatbuffers_codec::wrapPayload(payload);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    }
    return invoke_async(executor, kSvcReadRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeUpdateRequirement(pcl_executor_t* executor,
                                     const wire_types::CreateRequirementRequest& request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data,
                                     const pcl_endpoint_route_t* route,
                                     const char*       content_type)
{
    std::string payload = json_codec::toJson(request);
    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        payload = flatbuffers_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    }
    return invoke_async(executor, kSvcUpdateRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeDeleteRequirement(pcl_executor_t* executor,
                                     const Identifier&            request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data,
                                     const pcl_endpoint_route_t* route,
                                     const char*       content_type)
{
    std::string payload = request;
    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        payload = flatbuffers_codec::wrapPayload(payload);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    }
    return invoke_async(executor, kSvcDeleteRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeReadDetail(pcl_executor_t* executor,
                              const Query&                 request,
                              pcl_resp_cb_fn_t        callback,
                              void*                   user_data,
                              const pcl_endpoint_route_t* route,
                              const char*       content_type)
{
    std::string payload = toJson(request);
    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        payload = flatbuffers_codec::wrapPayload(payload);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    }
    return invoke_async(executor, kSvcReadDetail, payload, callback, user_data, route, content_type);
}

// ---------------------------------------------------------------------------
// Dispatch — deserialise request, call handler, serialise response
// ---------------------------------------------------------------------------

void dispatch(ServiceHandler& handler,
              ServiceChannel  channel,
              const void*     request_buf,
              size_t          request_size,
              const char*     content_type,
              void**          response_buf,
              size_t*         response_size)
{
    std::string req_str;
    std::string rsp_payload;

    bool rsp_is_binary = false;

    if (is_json_content_type(content_type)) {
        req_str = json_request_body(request_buf, request_size);
        if (request_size != 0 && req_str.empty()) {
            *response_buf = nullptr;
            *response_size = 0;
            return;
        }
    } else if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        try {
            req_str = flatbuffers_codec::unwrapPayload(request_buf, request_size);
        } catch (...) {
            *response_buf = nullptr;
            *response_size = 0;
            return;
        }
#else
        *response_buf = nullptr;
        *response_size = 0;
        return;
#endif
    } else {
        *response_buf = nullptr;
        *response_size = 0;
        return;
    }

    switch (channel) {
    case ServiceChannel::ReadMatch: {
        auto req = fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadMatch(req);
        rsp_payload = "[";
        for (size_t i = 0; i < rsp.size(); ++i) {
            if (i > 0) rsp_payload += ",";
            rsp_payload += toJson(rsp[i]);
        }
        rsp_payload += "]";
        break;
    }
    case ServiceChannel::CreateRequirement: {
        auto wire_req = is_flatbuffers_content_type(content_type)
            ? flatbuffers_codec::fromBinaryCreateRequirementRequest(request_buf, request_size)
            : json_codec::createRequirementRequestFromJson(req_str);
        auto req = to_domain_create_requirement_request(wire_req);
        auto rsp = handler.handleCreateRequirement(req);
        {
            auto wire_rsp = to_wire_create_requirement_response(rsp);
            if (is_flatbuffers_content_type(content_type)) {
                rsp_payload = flatbuffers_codec::toBinary(wire_rsp);
                rsp_is_binary = true;
            } else {
                rsp_payload = json_codec::toJson(wire_rsp);
            }
        }
        break;
    }
    case ServiceChannel::ReadRequirement: {
        auto req = fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadRequirement(req);
        rsp_payload = "[";
        for (size_t i = 0; i < rsp.size(); ++i) {
            if (i > 0) rsp_payload += ",";
            rsp_payload += toJson(rsp[i]);
        }
        rsp_payload += "]";
        break;
    }
    case ServiceChannel::UpdateRequirement: {
        auto wire_req = is_flatbuffers_content_type(content_type)
            ? flatbuffers_codec::fromBinaryCreateRequirementRequest(request_buf, request_size)
            : json_codec::createRequirementRequestFromJson(req_str);
        auto req = to_domain_create_requirement_request(wire_req);
        auto rsp = handler.handleUpdateRequirement(req);
        rsp_payload = toJson(rsp);
        break;
    }
    case ServiceChannel::DeleteRequirement: {
        auto& req = req_str;
        auto rsp = handler.handleDeleteRequirement(req);
        rsp_payload = toJson(rsp);
        break;
    }
    case ServiceChannel::ReadDetail: {
        auto req = fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadDetail(req);
        rsp_payload = "[";
        for (size_t i = 0; i < rsp.size(); ++i) {
            if (i > 0) rsp_payload += ",";
            rsp_payload += toJson(rsp[i]);
        }
        rsp_payload += "]";
        break;
    }
    }

    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        if (!rsp_is_binary) {
            rsp_payload = flatbuffers_codec::wrapPayload(rsp_payload);
        }
#else
        *response_buf = nullptr;
        *response_size = 0;
        return;
#endif
    }

    if (!rsp_payload.empty()) {
        *response_size = rsp_payload.size();
        *response_buf = std::malloc(rsp_payload.size());
        std::memcpy(*response_buf, rsp_payload.data(), rsp_payload.size());
    } else {
        *response_buf = nullptr;
        *response_size = 0;
    }
}

} // namespace pyramid::services::tactical_objects::provided
