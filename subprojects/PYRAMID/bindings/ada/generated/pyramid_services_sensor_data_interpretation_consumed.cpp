// Auto-generated service binding implementation
// Generated from: pyramid.components.sensor_data_interpretation.services.consumed.proto by generate_bindings.py
// Namespace: pyramid::components::sensor_data_interpretation::services::consumed

#include "pyramid_services_sensor_data_interpretation_consumed.hpp"

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

namespace pyramid::components::sensor_data_interpretation::services::consumed {

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

Identifier
ServiceHandler::handleDataProvisionDependencyCreateRequirement(const ObjectEvidenceProvisionRequirement& /*request*/) {
    return {};
}

std::vector<ObjectEvidenceProvisionRequirement>
ServiceHandler::handleDataProvisionDependencyReadRequirement(const Query& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleDataProvisionDependencyUpdateRequirement(const ObjectEvidenceProvisionRequirement& /*request*/) {
    return pyramid::domain_model::kAckOk;
}

Ack
ServiceHandler::handleDataProvisionDependencyDeleteRequirement(const Identifier& /*request*/) {
    return pyramid::domain_model::kAckOk;
}

Identifier
ServiceHandler::handleDataProcessingDependencyCreateRequirement(const ObjectAquisitionRequirement& /*request*/) {
    return {};
}

std::vector<ObjectAquisitionRequirement>
ServiceHandler::handleDataProcessingDependencyReadRequirement(const Query& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleDataProcessingDependencyUpdateRequirement(const ObjectAquisitionRequirement& /*request*/) {
    return pyramid::domain_model::kAckOk;
}

Ack
ServiceHandler::handleDataProcessingDependencyDeleteRequirement(const Identifier& /*request*/) {
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

bool decodeDataProvisionDependencyCreateRequirementResponse(const pcl_msg_t* msg,
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

bool decodeDataProvisionDependencyReadRequirementResponse(const pcl_msg_t* msg,
                                                          std::vector<ObjectEvidenceProvisionRequirement>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryObjectEvidenceProvisionRequirementArray(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        const auto arr = nlohmann::json::parse(payload);
        out->clear();
        for (const auto& item : arr) {
            out->push_back(fromJson(item.dump(), static_cast<ObjectEvidenceProvisionRequirement*>(nullptr)));
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool decodeDataProvisionDependencyUpdateRequirementResponse(const pcl_msg_t* msg,
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

bool decodeDataProvisionDependencyDeleteRequirementResponse(const pcl_msg_t* msg,
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

bool decodeDataProcessingDependencyCreateRequirementResponse(const pcl_msg_t* msg,
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

bool decodeDataProcessingDependencyReadRequirementResponse(const pcl_msg_t* msg,
                                                           std::vector<ObjectAquisitionRequirement>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryObjectAquisitionRequirementArray(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        const auto arr = nlohmann::json::parse(payload);
        out->clear();
        for (const auto& item : arr) {
            out->push_back(fromJson(item.dump(), static_cast<ObjectAquisitionRequirement*>(nullptr)));
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool decodeDataProcessingDependencyUpdateRequirementResponse(const pcl_msg_t* msg,
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

bool decodeDataProcessingDependencyDeleteRequirementResponse(const pcl_msg_t* msg,
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

pcl_status_t invokeDataProvisionDependencyCreateRequirement(pcl_executor_t* executor,
                                                            const ObjectEvidenceProvisionRequirement& request,
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
    return invoke_async(executor, kSvcDataProvisionDependencyCreateRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeDataProvisionDependencyCreateRequirement(pcl_executor_t* executor,
                                                            const ObjectEvidenceProvisionRequirement& request,
                                                            const char*       content_type,
                                                            const pcl_endpoint_route_t* route)
{
    return invokeDataProvisionDependencyCreateRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeDataProvisionDependencyReadRequirement(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcDataProvisionDependencyReadRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeDataProvisionDependencyReadRequirement(pcl_executor_t* executor,
                                                          const Query&                 request,
                                                          const char*       content_type,
                                                          const pcl_endpoint_route_t* route)
{
    return invokeDataProvisionDependencyReadRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeDataProvisionDependencyUpdateRequirement(pcl_executor_t* executor,
                                                            const ObjectEvidenceProvisionRequirement& request,
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
    return invoke_async(executor, kSvcDataProvisionDependencyUpdateRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeDataProvisionDependencyUpdateRequirement(pcl_executor_t* executor,
                                                            const ObjectEvidenceProvisionRequirement& request,
                                                            const char*       content_type,
                                                            const pcl_endpoint_route_t* route)
{
    return invokeDataProvisionDependencyUpdateRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeDataProvisionDependencyDeleteRequirement(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcDataProvisionDependencyDeleteRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeDataProvisionDependencyDeleteRequirement(pcl_executor_t* executor,
                                                            const Identifier&            request,
                                                            const char*       content_type,
                                                            const pcl_endpoint_route_t* route)
{
    return invokeDataProvisionDependencyDeleteRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeDataProcessingDependencyCreateRequirement(pcl_executor_t* executor,
                                                             const ObjectAquisitionRequirement& request,
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
    return invoke_async(executor, kSvcDataProcessingDependencyCreateRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeDataProcessingDependencyCreateRequirement(pcl_executor_t* executor,
                                                             const ObjectAquisitionRequirement& request,
                                                             const char*       content_type,
                                                             const pcl_endpoint_route_t* route)
{
    return invokeDataProcessingDependencyCreateRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeDataProcessingDependencyReadRequirement(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcDataProcessingDependencyReadRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeDataProcessingDependencyReadRequirement(pcl_executor_t* executor,
                                                           const Query&                 request,
                                                           const char*       content_type,
                                                           const pcl_endpoint_route_t* route)
{
    return invokeDataProcessingDependencyReadRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeDataProcessingDependencyUpdateRequirement(pcl_executor_t* executor,
                                                             const ObjectAquisitionRequirement& request,
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
    return invoke_async(executor, kSvcDataProcessingDependencyUpdateRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeDataProcessingDependencyUpdateRequirement(pcl_executor_t* executor,
                                                             const ObjectAquisitionRequirement& request,
                                                             const char*       content_type,
                                                             const pcl_endpoint_route_t* route)
{
    return invokeDataProcessingDependencyUpdateRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeDataProcessingDependencyDeleteRequirement(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcDataProcessingDependencyDeleteRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeDataProcessingDependencyDeleteRequirement(pcl_executor_t* executor,
                                                             const Identifier&            request,
                                                             const char*       content_type,
                                                             const pcl_endpoint_route_t* route)
{
    return invokeDataProcessingDependencyDeleteRequirement(executor, request, ignore_async_response,
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
    case ServiceChannel::DataProvisionDependencyCreateRequirement: {
        ObjectEvidenceProvisionRequirement req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<ObjectEvidenceProvisionRequirement*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryObjectEvidenceProvisionRequirement(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleDataProvisionDependencyCreateRequirement(req);
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
    case ServiceChannel::DataProvisionDependencyReadRequirement: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleDataProvisionDependencyReadRequirement(req);
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
    case ServiceChannel::DataProvisionDependencyUpdateRequirement: {
        ObjectEvidenceProvisionRequirement req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<ObjectEvidenceProvisionRequirement*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryObjectEvidenceProvisionRequirement(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleDataProvisionDependencyUpdateRequirement(req);
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
    case ServiceChannel::DataProvisionDependencyDeleteRequirement: {
        Identifier req;
        if (is_json_content_type(content_type))
            req = decode_identifier_payload(req_str);
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryIdentifier(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleDataProvisionDependencyDeleteRequirement(req);
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
    case ServiceChannel::DataProcessingDependencyCreateRequirement: {
        ObjectAquisitionRequirement req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<ObjectAquisitionRequirement*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryObjectAquisitionRequirement(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleDataProcessingDependencyCreateRequirement(req);
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
    case ServiceChannel::DataProcessingDependencyReadRequirement: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleDataProcessingDependencyReadRequirement(req);
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
    case ServiceChannel::DataProcessingDependencyUpdateRequirement: {
        ObjectAquisitionRequirement req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<ObjectAquisitionRequirement*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryObjectAquisitionRequirement(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleDataProcessingDependencyUpdateRequirement(req);
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
    case ServiceChannel::DataProcessingDependencyDeleteRequirement: {
        Identifier req;
        if (is_json_content_type(content_type))
            req = decode_identifier_payload(req_str);
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryIdentifier(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleDataProcessingDependencyDeleteRequirement(req);
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

} // namespace pyramid::components::sensor_data_interpretation::services::consumed
