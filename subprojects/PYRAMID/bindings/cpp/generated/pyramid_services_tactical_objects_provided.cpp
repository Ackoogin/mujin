// Auto-generated service binding implementation
// Generated from: pyramid.components.tactical_objects.services.provided.proto by generate_bindings.py
// Namespace: pyramid::components::tactical_objects::services::provided

#include "pyramid_services_tactical_objects_provided.hpp"

#include "flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.hpp"
#include "pyramid_data_model_autonomy_codec.hpp"
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

namespace pyramid::components::tactical_objects::services::provided {

// Bring data model codec functions into scope
using pyramid::domain_model::autonomy::toJson;
using pyramid::domain_model::autonomy::fromJson;
using pyramid::domain_model::common::toJson;
using pyramid::domain_model::common::fromJson;
using pyramid::domain_model::tactical::toJson;
using pyramid::domain_model::tactical::fromJson;
namespace flatbuffers_codec = pyramid::services::tactical_objects::flatbuffers_codec;

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
    return pyramid::domain_model::kAckOk;
}

Ack
ServiceHandler::handleDeleteRequirement(const Identifier& /*request*/) {
    return pyramid::domain_model::kAckOk;
}

std::vector<ObjectDetail>
ServiceHandler::handleReadDetail(const Query& /*request*/) {
    return {};
}

// ---------------------------------------------------------------------------
// Internal PCL helpers
// ---------------------------------------------------------------------------

namespace {

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

void ignore_async_response(const pcl_msg_t*, void*) {}

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
// Content-type support metadata
// ---------------------------------------------------------------------------

std::vector<const char*> supportedContentTypes()
{
    std::vector<const char*> result{kJsonContentType};
    result.push_back(kFlatBuffersContentType);
    return result;
}

bool supportsContentType(const char* content_type)
{
    if (is_json_content_type(content_type)) {
        return true;
    }
    if (is_flatbuffers_content_type(content_type)) {
        return true;
    }
    return false;
}

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
// Typed response decode wrappers
// ---------------------------------------------------------------------------

bool decodeReadMatchResponse(const pcl_msg_t* msg,
                             std::vector<ObjectMatch>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryObjectMatchArray(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        const auto arr = nlohmann::json::parse(payload);
        out->clear();
        for (const auto& item : arr) {
            out->push_back(fromJson(item.dump(), static_cast<ObjectMatch*>(nullptr)));
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool decodeCreateRequirementResponse(const pcl_msg_t* msg,
                                     Identifier* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryIdentifier(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        *out = decode_identifier_payload(payload);
        return true;
    } catch (...) {
        return false;
    }
}

bool decodeReadRequirementResponse(const pcl_msg_t* msg,
                                   std::vector<ObjectInterestRequirement>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryObjectInterestRequirementArray(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        const auto arr = nlohmann::json::parse(payload);
        out->clear();
        for (const auto& item : arr) {
            out->push_back(fromJson(item.dump(), static_cast<ObjectInterestRequirement*>(nullptr)));
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool decodeUpdateRequirementResponse(const pcl_msg_t* msg,
                                     Ack* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryAck(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        *out = fromJson(payload, static_cast<Ack*>(nullptr));
        return true;
    } catch (...) {
        return false;
    }
}

bool decodeDeleteRequirementResponse(const pcl_msg_t* msg,
                                     Ack* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryAck(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        *out = fromJson(payload, static_cast<Ack*>(nullptr));
        return true;
    } catch (...) {
        return false;
    }
}

bool decodeReadDetailResponse(const pcl_msg_t* msg,
                              std::vector<ObjectDetail>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryObjectDetailArray(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        const auto arr = nlohmann::json::parse(payload);
        out->clear();
        for (const auto& item : arr) {
            out->push_back(fromJson(item.dump(), static_cast<ObjectDetail*>(nullptr)));
        }
        return true;
    } catch (...) {
        return false;
    }
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
    if (is_json_content_type(content_type)) {
        payload = toJson(request);
    } else if (is_flatbuffers_content_type(content_type)) {
        payload = flatbuffers_codec::toBinary(request);
    } else {
        return PCL_ERR_INVALID;
    }
    return invoke_async(executor, kSvcReadMatch, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeReadMatch(pcl_executor_t* executor,
                             const Query&                 request,
                             const char*       content_type,
                             const pcl_endpoint_route_t* route)
{
    return invokeReadMatch(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeCreateRequirement(pcl_executor_t* executor,
                                     const ObjectInterestRequirement& request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data,
                                     const pcl_endpoint_route_t* route,
                                     const char*       content_type)
{
    std::string payload;
    if (is_json_content_type(content_type)) {
        payload = toJson(request);
    } else if (is_flatbuffers_content_type(content_type)) {
        payload = flatbuffers_codec::toBinary(request);
    } else {
        return PCL_ERR_INVALID;
    }
    return invoke_async(executor, kSvcCreateRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeCreateRequirement(pcl_executor_t* executor,
                                     const ObjectInterestRequirement& request,
                                     const char*       content_type,
                                     const pcl_endpoint_route_t* route)
{
    return invokeCreateRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeReadRequirement(pcl_executor_t* executor,
                                   const Query&                 request,
                                   pcl_resp_cb_fn_t        callback,
                                   void*                   user_data,
                                   const pcl_endpoint_route_t* route,
                                   const char*       content_type)
{
    std::string payload;
    if (is_json_content_type(content_type)) {
        payload = toJson(request);
    } else if (is_flatbuffers_content_type(content_type)) {
        payload = flatbuffers_codec::toBinary(request);
    } else {
        return PCL_ERR_INVALID;
    }
    return invoke_async(executor, kSvcReadRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeReadRequirement(pcl_executor_t* executor,
                                   const Query&                 request,
                                   const char*       content_type,
                                   const pcl_endpoint_route_t* route)
{
    return invokeReadRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeUpdateRequirement(pcl_executor_t* executor,
                                     const ObjectInterestRequirement& request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data,
                                     const pcl_endpoint_route_t* route,
                                     const char*       content_type)
{
    std::string payload;
    if (is_json_content_type(content_type)) {
        payload = toJson(request);
    } else if (is_flatbuffers_content_type(content_type)) {
        payload = flatbuffers_codec::toBinary(request);
    } else {
        return PCL_ERR_INVALID;
    }
    return invoke_async(executor, kSvcUpdateRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeUpdateRequirement(pcl_executor_t* executor,
                                     const ObjectInterestRequirement& request,
                                     const char*       content_type,
                                     const pcl_endpoint_route_t* route)
{
    return invokeUpdateRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeDeleteRequirement(pcl_executor_t* executor,
                                     const Identifier&            request,
                                     pcl_resp_cb_fn_t        callback,
                                     void*                   user_data,
                                     const pcl_endpoint_route_t* route,
                                     const char*       content_type)
{
    std::string payload;
    if (is_json_content_type(content_type)) {
        payload = encode_identifier_payload(request);
    } else if (is_flatbuffers_content_type(content_type)) {
        payload = flatbuffers_codec::toBinary(request);
    } else {
        return PCL_ERR_INVALID;
    }
    return invoke_async(executor, kSvcDeleteRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeDeleteRequirement(pcl_executor_t* executor,
                                     const Identifier&            request,
                                     const char*       content_type,
                                     const pcl_endpoint_route_t* route)
{
    return invokeDeleteRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeReadDetail(pcl_executor_t* executor,
                              const Query&                 request,
                              pcl_resp_cb_fn_t        callback,
                              void*                   user_data,
                              const pcl_endpoint_route_t* route,
                              const char*       content_type)
{
    std::string payload;
    if (is_json_content_type(content_type)) {
        payload = toJson(request);
    } else if (is_flatbuffers_content_type(content_type)) {
        payload = flatbuffers_codec::toBinary(request);
    } else {
        return PCL_ERR_INVALID;
    }
    return invoke_async(executor, kSvcReadDetail, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeReadDetail(pcl_executor_t* executor,
                              const Query&                 request,
                              const char*       content_type,
                              const pcl_endpoint_route_t* route)
{
    return invokeReadDetail(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

// ---------------------------------------------------------------------------
// PCL topic wrappers
// ---------------------------------------------------------------------------

bool encodeEntityMatches(const std::vector<ObjectMatch>& payload,
                         const char*        content_type,
                         std::string*       out)
{
    if (!out) {
        return false;
    }
    std::string wire_payload;
    if (is_json_content_type(content_type)) {
        wire_payload = "[";
        bool first = true;
        for (const auto& item : payload) {
            if (!first) wire_payload += ",";
            first = false;
            wire_payload += toJson(item);
        }
        wire_payload += "]";
    } else if (is_flatbuffers_content_type(content_type)) {
        wire_payload = flatbuffers_codec::toBinary(payload);
    } else {
        return false;
    }
    *out = wire_payload;
    return true;
}

pcl_status_t publishEntityMatches(pcl_port_t*        publisher,
                                  const std::vector<ObjectMatch>& payload,
                                  const char*        content_type)
{
    std::string wire_payload;
    if (!encodeEntityMatches(payload, content_type, &wire_payload)) {
        return PCL_ERR_INVALID;
    }
    return publishEntityMatches(publisher, wire_payload, content_type);
}

pcl_status_t publishEntityMatches(pcl_port_t*        publisher,
                                  const std::string& payload,
                                  const char*        content_type)
{
    pcl_msg_t msg{};
    msg.data      = payload.data();
    msg.size      = static_cast<uint32_t>(payload.size());
    msg.type_name = content_type;
    return pcl_port_publish(publisher, &msg);
}

bool decodeEntityMatches(const pcl_msg_t* msg,
                         std::vector<ObjectMatch>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryObjectMatchArray(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        const auto arr = nlohmann::json::parse(payload);
        out->clear();
        for (const auto& item : arr) {
            out->push_back(fromJson(item.dump(), static_cast<ObjectMatch*>(nullptr)));
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool encodeEvidenceRequirements(const ObjectEvidenceRequirement& payload,
                                const char*        content_type,
                                std::string*       out)
{
    if (!out) {
        return false;
    }
    std::string wire_payload;
    if (is_json_content_type(content_type)) {
        wire_payload = toJson(payload);
    } else if (is_flatbuffers_content_type(content_type)) {
        wire_payload = flatbuffers_codec::toBinary(payload);
    } else {
        return false;
    }
    *out = wire_payload;
    return true;
}

pcl_status_t publishEvidenceRequirements(pcl_port_t*        publisher,
                                         const ObjectEvidenceRequirement& payload,
                                         const char*        content_type)
{
    std::string wire_payload;
    if (!encodeEvidenceRequirements(payload, content_type, &wire_payload)) {
        return PCL_ERR_INVALID;
    }
    return publishEvidenceRequirements(publisher, wire_payload, content_type);
}

pcl_status_t publishEvidenceRequirements(pcl_port_t*        publisher,
                                         const std::string& payload,
                                         const char*        content_type)
{
    pcl_msg_t msg{};
    msg.data      = payload.data();
    msg.size      = static_cast<uint32_t>(payload.size());
    msg.type_name = content_type;
    return pcl_port_publish(publisher, &msg);
}

bool decodeEvidenceRequirements(const pcl_msg_t* msg,
                                ObjectEvidenceRequirement* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryObjectEvidenceRequirement(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        *out = fromJson(payload, static_cast<ObjectEvidenceRequirement*>(nullptr));
        return true;
    } catch (...) {
        return false;
    }
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
    } else {
        *response_buf = nullptr;
        *response_size = 0;
        return;
    }

    try {
    switch (channel) {
    case ServiceChannel::ReadMatch: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleReadMatch(req);
        if (is_json_content_type(content_type)) {
            rsp_payload = "[";
            for (size_t i = 0; i < rsp.size(); ++i) {
                if (i > 0) rsp_payload += ",";
                rsp_payload += toJson(rsp[i]);
            }
            rsp_payload += "]";
        } else if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            break;
        }
        break;
    }
    case ServiceChannel::CreateRequirement: {
        ObjectInterestRequirement req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<ObjectInterestRequirement*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryObjectInterestRequirement(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleCreateRequirement(req);
        if (is_json_content_type(content_type)) {
            rsp_payload = encode_identifier_payload(rsp);
        } else if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            break;
        }
        break;
    }
    case ServiceChannel::ReadRequirement: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleReadRequirement(req);
        if (is_json_content_type(content_type)) {
            rsp_payload = "[";
            for (size_t i = 0; i < rsp.size(); ++i) {
                if (i > 0) rsp_payload += ",";
                rsp_payload += toJson(rsp[i]);
            }
            rsp_payload += "]";
        } else if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            break;
        }
        break;
    }
    case ServiceChannel::UpdateRequirement: {
        ObjectInterestRequirement req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<ObjectInterestRequirement*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryObjectInterestRequirement(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleUpdateRequirement(req);
        if (is_json_content_type(content_type)) {
            rsp_payload = toJson(rsp);
        } else if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            break;
        }
        break;
    }
    case ServiceChannel::DeleteRequirement: {
        Identifier req;
        if (is_json_content_type(content_type))
            req = decode_identifier_payload(req_str);
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryIdentifier(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleDeleteRequirement(req);
        if (is_json_content_type(content_type)) {
            rsp_payload = toJson(rsp);
        } else if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            break;
        }
        break;
    }
    case ServiceChannel::ReadDetail: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleReadDetail(req);
        if (is_json_content_type(content_type)) {
            rsp_payload = "[";
            for (size_t i = 0; i < rsp.size(); ++i) {
                if (i > 0) rsp_payload += ",";
                rsp_payload += toJson(rsp[i]);
            }
            rsp_payload += "]";
        } else if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
        } else {
            break;
        }
        break;
    }
    }
    } catch (...) {
        *response_buf = nullptr;
        *response_size = 0;
        return;
    }

    (void) rsp_is_binary;

    if (!rsp_payload.empty()) {
        *response_size = rsp_payload.size();
        *response_buf = std::malloc(rsp_payload.size());
        std::memcpy(*response_buf, rsp_payload.data(), rsp_payload.size());
    } else {
        *response_buf = nullptr;
        *response_size = 0;
    }
}

} // namespace pyramid::components::tactical_objects::services::provided
