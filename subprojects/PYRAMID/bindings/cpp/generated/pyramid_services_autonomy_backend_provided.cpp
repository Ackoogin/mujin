// Auto-generated service binding implementation
// Generated from: provided.proto by generate_bindings.py
// Namespace: pyramid::services::autonomy_backend::provided

#include "pyramid_services_autonomy_backend_provided.hpp"

#include "flatbuffers/cpp/pyramid_services_autonomy_backend_flatbuffers_codec.hpp"
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

namespace pyramid::services::autonomy_backend::provided {

// Bring data model codec functions into scope
using pyramid::data_model::autonomy::toJson;
using pyramid::data_model::autonomy::fromJson;
using pyramid::data_model::common::toJson;
using pyramid::data_model::common::fromJson;
using pyramid::data_model::tactical::toJson;
using pyramid::data_model::tactical::fromJson;
namespace flatbuffers_codec = pyramid::services::autonomy_backend::flatbuffers_codec;

// ---------------------------------------------------------------------------
// PCL message utility
// ---------------------------------------------------------------------------

std::string msgToString(const void* data, unsigned size) {
    return std::string(static_cast<const char*>(data), size);
}

// ---------------------------------------------------------------------------
// ServiceHandler — default stub implementations
// ---------------------------------------------------------------------------

std::vector<Capabilities>
ServiceHandler::handleReadCapabilities(const Query& /*request*/) {
    return {};
}

Identifier
ServiceHandler::handleCreateRequirement(const PlanningExecutionRequirement& /*request*/) {
    return {};
}

std::vector<PlanningExecutionRequirement>
ServiceHandler::handleReadRequirement(const Query& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleUpdateRequirement(const PlanningExecutionRequirement& /*request*/) {
    return pyramid::data_model::kAckOk;
}

Ack
ServiceHandler::handleDeleteRequirement(const Identifier& /*request*/) {
    return pyramid::data_model::kAckOk;
}

Identifier
ServiceHandler::handleCreateState(const StateUpdate& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleUpdateState(const StateUpdate& /*request*/) {
    return pyramid::data_model::kAckOk;
}

Ack
ServiceHandler::handleDeleteState(const Identifier& /*request*/) {
    return pyramid::data_model::kAckOk;
}

std::vector<Plan>
ServiceHandler::handleReadPlan(const Query& /*request*/) {
    return {};
}

std::vector<ExecutionRun>
ServiceHandler::handleReadRun(const Query& /*request*/) {
    return {};
}

std::vector<RequirementPlacement>
ServiceHandler::handleReadPlacement(const Query& /*request*/) {
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
// Typed response decode wrappers
// ---------------------------------------------------------------------------

bool decodeReadCapabilitiesResponse(const pcl_msg_t* msg,
                                    std::vector<Capabilities>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryCapabilitiesArray(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        const auto arr = nlohmann::json::parse(payload);
        out->clear();
        for (const auto& item : arr) {
            out->push_back(fromJson(item.dump(), static_cast<Capabilities*>(nullptr)));
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
                                   std::vector<PlanningExecutionRequirement>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryPlanningExecutionRequirementArray(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        const auto arr = nlohmann::json::parse(payload);
        out->clear();
        for (const auto& item : arr) {
            out->push_back(fromJson(item.dump(), static_cast<PlanningExecutionRequirement*>(nullptr)));
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

bool decodeCreateStateResponse(const pcl_msg_t* msg,
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

bool decodeUpdateStateResponse(const pcl_msg_t* msg,
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

bool decodeDeleteStateResponse(const pcl_msg_t* msg,
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

bool decodeReadPlanResponse(const pcl_msg_t* msg,
                            std::vector<Plan>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryPlanArray(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        const auto arr = nlohmann::json::parse(payload);
        out->clear();
        for (const auto& item : arr) {
            out->push_back(fromJson(item.dump(), static_cast<Plan*>(nullptr)));
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool decodeReadRunResponse(const pcl_msg_t* msg,
                           std::vector<ExecutionRun>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryExecutionRunArray(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        const auto arr = nlohmann::json::parse(payload);
        out->clear();
        for (const auto& item : arr) {
            out->push_back(fromJson(item.dump(), static_cast<ExecutionRun*>(nullptr)));
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool decodeReadPlacementResponse(const pcl_msg_t* msg,
                                 std::vector<RequirementPlacement>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryRequirementPlacementArray(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        const auto arr = nlohmann::json::parse(payload);
        out->clear();
        for (const auto& item : arr) {
            out->push_back(fromJson(item.dump(), static_cast<RequirementPlacement*>(nullptr)));
        }
        return true;
    } catch (...) {
        return false;
    }
}

// ---------------------------------------------------------------------------
// Typed invoke wrappers — serialise and dispatch via executor transport
// ---------------------------------------------------------------------------

pcl_status_t invokeReadCapabilities(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcReadCapabilities, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeReadCapabilities(pcl_executor_t* executor,
                                    const Query&                 request,
                                    const char*       content_type,
                                    const pcl_endpoint_route_t* route)
{
    return invokeReadCapabilities(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeCreateRequirement(pcl_executor_t* executor,
                                     const PlanningExecutionRequirement& request,
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
                                     const PlanningExecutionRequirement& request,
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
                                     const PlanningExecutionRequirement& request,
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
                                     const PlanningExecutionRequirement& request,
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

pcl_status_t invokeCreateState(pcl_executor_t* executor,
                               const StateUpdate&           request,
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
    return invoke_async(executor, kSvcCreateState, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeCreateState(pcl_executor_t* executor,
                               const StateUpdate&           request,
                               const char*       content_type,
                               const pcl_endpoint_route_t* route)
{
    return invokeCreateState(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeUpdateState(pcl_executor_t* executor,
                               const StateUpdate&           request,
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
    return invoke_async(executor, kSvcUpdateState, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeUpdateState(pcl_executor_t* executor,
                               const StateUpdate&           request,
                               const char*       content_type,
                               const pcl_endpoint_route_t* route)
{
    return invokeUpdateState(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeDeleteState(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcDeleteState, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeDeleteState(pcl_executor_t* executor,
                               const Identifier&            request,
                               const char*       content_type,
                               const pcl_endpoint_route_t* route)
{
    return invokeDeleteState(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeReadPlan(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcReadPlan, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeReadPlan(pcl_executor_t* executor,
                            const Query&                 request,
                            const char*       content_type,
                            const pcl_endpoint_route_t* route)
{
    return invokeReadPlan(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeReadRun(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcReadRun, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeReadRun(pcl_executor_t* executor,
                           const Query&                 request,
                           const char*       content_type,
                           const pcl_endpoint_route_t* route)
{
    return invokeReadRun(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeReadPlacement(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcReadPlacement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeReadPlacement(pcl_executor_t* executor,
                                 const Query&                 request,
                                 const char*       content_type,
                                 const pcl_endpoint_route_t* route)
{
    return invokeReadPlacement(executor, request, ignore_async_response,
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
    case ServiceChannel::ReadCapabilities: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleReadCapabilities(req);
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
        PlanningExecutionRequirement req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<PlanningExecutionRequirement*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryPlanningExecutionRequirement(request_buf, request_size);
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
        PlanningExecutionRequirement req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<PlanningExecutionRequirement*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryPlanningExecutionRequirement(request_buf, request_size);
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
    case ServiceChannel::CreateState: {
        StateUpdate req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<StateUpdate*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryStateUpdate(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleCreateState(req);
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
    case ServiceChannel::UpdateState: {
        StateUpdate req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<StateUpdate*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryStateUpdate(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleUpdateState(req);
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
    case ServiceChannel::DeleteState: {
        Identifier req;
        if (is_json_content_type(content_type))
            req = decode_identifier_payload(req_str);
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryIdentifier(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleDeleteState(req);
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
    case ServiceChannel::ReadPlan: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleReadPlan(req);
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
    case ServiceChannel::ReadRun: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleReadRun(req);
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
    case ServiceChannel::ReadPlacement: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleReadPlacement(req);
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

} // namespace pyramid::services::autonomy_backend::provided
