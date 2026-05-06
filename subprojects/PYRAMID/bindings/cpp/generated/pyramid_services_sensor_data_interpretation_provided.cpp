// Auto-generated service binding implementation
// Generated from: pyramid.components.sensor_data_interpretation.services.provided.proto by generate_bindings.py
// Namespace: pyramid::components::sensor_data_interpretation::services::provided

#include "pyramid_services_sensor_data_interpretation_provided.hpp"

#include "flatbuffers/cpp/pyramid_services_sensor_data_interpretation_flatbuffers_codec.hpp"
#include "pyramid_data_model_common_codec.hpp"
#include "pyramid_data_model_sensors_codec.hpp"

#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport.h>

#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace pyramid::components::sensor_data_interpretation::services::provided {

// Bring data model codec functions into scope
using pyramid::domain_model::common::toJson;
using pyramid::domain_model::common::fromJson;
using pyramid::domain_model::sensors::toJson;
using pyramid::domain_model::sensors::fromJson;
namespace flatbuffers_codec = pyramid::services::sensor_data_interpretation::flatbuffers_codec;

// ---------------------------------------------------------------------------
// PCL message utility
// ---------------------------------------------------------------------------

std::string msgToString(const void* data, unsigned size) {
    return std::string(static_cast<const char*>(data), size);
}

// ---------------------------------------------------------------------------
// ServiceHandler — default stub implementations
// ---------------------------------------------------------------------------

std::vector<Capability>
ServiceHandler::handleInterpretationRequirementReadCapability(const Query& /*request*/) {
    return {};
}

Identifier
ServiceHandler::handleInterpretationRequirementCreateRequirement(const InterpretationRequirement& /*request*/) {
    return {};
}

std::vector<InterpretationRequirement>
ServiceHandler::handleInterpretationRequirementReadRequirement(const Query& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleInterpretationRequirementUpdateRequirement(const InterpretationRequirement& /*request*/) {
    return pyramid::domain_model::kAckOk;
}

Ack
ServiceHandler::handleInterpretationRequirementDeleteRequirement(const Identifier& /*request*/) {
    return pyramid::domain_model::kAckOk;
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
// Typed response decode wrappers
// ---------------------------------------------------------------------------

bool decodeInterpretationRequirementReadCapabilityResponse(const pcl_msg_t* msg,
                                                           std::vector<Capability>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryCapabilityArray(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        const auto arr = nlohmann::json::parse(payload);
        out->clear();
        for (const auto& item : arr) {
            out->push_back(fromJson(item.dump(), static_cast<Capability*>(nullptr)));
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool decodeInterpretationRequirementCreateRequirementResponse(const pcl_msg_t* msg,
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

bool decodeInterpretationRequirementReadRequirementResponse(const pcl_msg_t* msg,
                                                            std::vector<InterpretationRequirement>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryInterpretationRequirementArray(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        const auto arr = nlohmann::json::parse(payload);
        out->clear();
        for (const auto& item : arr) {
            out->push_back(fromJson(item.dump(), static_cast<InterpretationRequirement*>(nullptr)));
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool decodeInterpretationRequirementUpdateRequirementResponse(const pcl_msg_t* msg,
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

bool decodeInterpretationRequirementDeleteRequirementResponse(const pcl_msg_t* msg,
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

// ---------------------------------------------------------------------------
// Typed invoke wrappers — serialise and dispatch via executor transport
// ---------------------------------------------------------------------------

pcl_status_t invokeInterpretationRequirementReadCapability(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcInterpretationRequirementReadCapability, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeInterpretationRequirementReadCapability(pcl_executor_t* executor,
                                                           const Query&                 request,
                                                           const char*       content_type,
                                                           const pcl_endpoint_route_t* route)
{
    return invokeInterpretationRequirementReadCapability(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeInterpretationRequirementCreateRequirement(pcl_executor_t* executor,
                                                              const InterpretationRequirement& request,
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
    return invoke_async(executor, kSvcInterpretationRequirementCreateRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeInterpretationRequirementCreateRequirement(pcl_executor_t* executor,
                                                              const InterpretationRequirement& request,
                                                              const char*       content_type,
                                                              const pcl_endpoint_route_t* route)
{
    return invokeInterpretationRequirementCreateRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeInterpretationRequirementReadRequirement(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcInterpretationRequirementReadRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeInterpretationRequirementReadRequirement(pcl_executor_t* executor,
                                                            const Query&                 request,
                                                            const char*       content_type,
                                                            const pcl_endpoint_route_t* route)
{
    return invokeInterpretationRequirementReadRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeInterpretationRequirementUpdateRequirement(pcl_executor_t* executor,
                                                              const InterpretationRequirement& request,
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
    return invoke_async(executor, kSvcInterpretationRequirementUpdateRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeInterpretationRequirementUpdateRequirement(pcl_executor_t* executor,
                                                              const InterpretationRequirement& request,
                                                              const char*       content_type,
                                                              const pcl_endpoint_route_t* route)
{
    return invokeInterpretationRequirementUpdateRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeInterpretationRequirementDeleteRequirement(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcInterpretationRequirementDeleteRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeInterpretationRequirementDeleteRequirement(pcl_executor_t* executor,
                                                              const Identifier&            request,
                                                              const char*       content_type,
                                                              const pcl_endpoint_route_t* route)
{
    return invokeInterpretationRequirementDeleteRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
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
    case ServiceChannel::InterpretationRequirementReadCapability: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleInterpretationRequirementReadCapability(req);
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
    case ServiceChannel::InterpretationRequirementCreateRequirement: {
        InterpretationRequirement req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<InterpretationRequirement*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryInterpretationRequirement(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleInterpretationRequirementCreateRequirement(req);
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
    case ServiceChannel::InterpretationRequirementReadRequirement: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleInterpretationRequirementReadRequirement(req);
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
    case ServiceChannel::InterpretationRequirementUpdateRequirement: {
        InterpretationRequirement req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<InterpretationRequirement*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryInterpretationRequirement(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleInterpretationRequirementUpdateRequirement(req);
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
    case ServiceChannel::InterpretationRequirementDeleteRequirement: {
        Identifier req;
        if (is_json_content_type(content_type))
            req = decode_identifier_payload(req_str);
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryIdentifier(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleInterpretationRequirementDeleteRequirement(req);
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

} // namespace pyramid::components::sensor_data_interpretation::services::provided
