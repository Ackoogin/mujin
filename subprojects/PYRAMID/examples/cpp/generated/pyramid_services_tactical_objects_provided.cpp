// Auto-generated service binding implementation
// Generated from: provided.proto by generate_bindings.py
// Namespace: pyramid::services::tactical_objects::provided

#include "pyramid_services_tactical_objects_provided.hpp"

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
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::provided {

// Bring data model codec functions into scope
using pyramid::data_model::common::toJson;
using pyramid::data_model::common::fromJson;
using pyramid::data_model::tactical::toJson;
using pyramid::data_model::tactical::fromJson;
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

std::string encode_identifier_payload(const Identifier& value)
{
    return nlohmann::json(value).dump();
}

Identifier decode_identifier_payload(const std::string& payload)
{
    if (payload.empty()) {
        return {};
    }
    try {
        auto j = nlohmann::json::parse(payload);
        if (j.is_string()) {
            return j.get<std::string>();
        }
        if (j.is_object() && j.contains("uuid") && j["uuid"].is_string()) {
            return j["uuid"].get<std::string>();
        }
    } catch (...) {
    }
    return payload;
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
    std::string payload;
    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        payload = flatbuffers_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcReadMatch, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeCreateRequirement(pcl_executor_t* executor,
                                     const ObjectInterestRequirement& request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data,
                                     const pcl_endpoint_route_t* route,
                                     const char*       content_type)
{
    std::string payload;
    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        payload = flatbuffers_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
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
    std::string payload;
    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        payload = flatbuffers_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcReadRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeUpdateRequirement(pcl_executor_t* executor,
                                     const ObjectInterestRequirement& request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data,
                                     const pcl_endpoint_route_t* route,
                                     const char*       content_type)
{
    std::string payload;
    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        payload = flatbuffers_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
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
    std::string payload;
    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        payload = flatbuffers_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = encode_identifier_payload(request);
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
    std::string payload;
    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        payload = flatbuffers_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
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
#if !PYRAMID_HAVE_SERVICE_FLATBUFFERS
        *response_buf = nullptr;
        *response_size = 0;
        return;
#endif
    } else {
        *response_buf = nullptr;
        *response_size = 0;
        return;
    }

    try {
    switch (channel) {
    case ServiceChannel::ReadMatch: {
        auto req = is_flatbuffers_content_type(content_type)
            ? flatbuffers_codec::fromBinaryQuery(request_buf, request_size)
            : fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadMatch(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            rsp_payload = "[";
            for (size_t i = 0; i < rsp.size(); ++i) {
                if (i > 0) rsp_payload += ",";
                rsp_payload += toJson(rsp[i]);
            }
            rsp_payload += "]";
        }
        break;
    }
    case ServiceChannel::CreateRequirement: {
        auto req = is_flatbuffers_content_type(content_type)
            ? flatbuffers_codec::fromBinaryObjectInterestRequirement(request_buf, request_size)
            : fromJson(req_str, static_cast<ObjectInterestRequirement*>(nullptr));
        auto rsp = handler.handleCreateRequirement(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            rsp_payload = encode_identifier_payload(rsp);
        }
        break;
    }
    case ServiceChannel::ReadRequirement: {
        auto req = is_flatbuffers_content_type(content_type)
            ? flatbuffers_codec::fromBinaryQuery(request_buf, request_size)
            : fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadRequirement(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            rsp_payload = "[";
            for (size_t i = 0; i < rsp.size(); ++i) {
                if (i > 0) rsp_payload += ",";
                rsp_payload += toJson(rsp[i]);
            }
            rsp_payload += "]";
        }
        break;
    }
    case ServiceChannel::UpdateRequirement: {
        auto req = is_flatbuffers_content_type(content_type)
            ? flatbuffers_codec::fromBinaryObjectInterestRequirement(request_buf, request_size)
            : fromJson(req_str, static_cast<ObjectInterestRequirement*>(nullptr));
        auto rsp = handler.handleUpdateRequirement(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            rsp_payload = toJson(rsp);
        }
        break;
    }
    case ServiceChannel::DeleteRequirement: {
        auto req = is_flatbuffers_content_type(content_type)
            ? flatbuffers_codec::fromBinaryIdentifier(request_buf, request_size)
            : decode_identifier_payload(req_str);
        auto rsp = handler.handleDeleteRequirement(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            rsp_payload = toJson(rsp);
        }
        break;
    }
    case ServiceChannel::ReadDetail: {
        auto req = is_flatbuffers_content_type(content_type)
            ? flatbuffers_codec::fromBinaryQuery(request_buf, request_size)
            : fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadDetail(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            rsp_payload = "[";
            for (size_t i = 0; i < rsp.size(); ++i) {
                if (i > 0) rsp_payload += ",";
                rsp_payload += toJson(rsp[i]);
            }
            rsp_payload += "]";
        }
        break;
    }
    }
    } catch (...) {
        *response_buf = nullptr;
        *response_size = 0;
        return;
    }

    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        (void) rsp_is_binary;
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
