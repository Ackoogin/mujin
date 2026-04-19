// Auto-generated service binding implementation
// Generated from: provided.proto by generate_bindings.py
// Namespace: pyramid::services::autonomy_backend::provided

#include "pyramid_services_autonomy_backend_provided.hpp"

#if __has_include("flatbuffers/cpp/pyramid_services_autonomy_backend_flatbuffers_codec.hpp")
#include "flatbuffers/cpp/pyramid_services_autonomy_backend_flatbuffers_codec.hpp"
#define PYRAMID_HAVE_SERVICE_FLATBUFFERS 1
#else
#define PYRAMID_HAVE_SERVICE_FLATBUFFERS 0
#endif
#if __has_include("pyramid_services_autonomy_backend_protobuf_codec.hpp")
#include "pyramid_services_autonomy_backend_protobuf_codec.hpp"
#define PYRAMID_HAVE_SERVICE_PROTOBUF 1
#else
#define PYRAMID_HAVE_SERVICE_PROTOBUF 0
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

namespace pyramid::services::autonomy_backend::provided {

// Bring data model codec functions into scope
using pyramid::data_model::common::toJson;
using pyramid::data_model::common::fromJson;
using pyramid::data_model::tactical::toJson;
using pyramid::data_model::tactical::fromJson;
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
namespace flatbuffers_codec = pyramid::services::autonomy_backend::flatbuffers_codec;
#endif
#if PYRAMID_HAVE_SERVICE_PROTOBUF
namespace protobuf_codec = pyramid::services::autonomy_backend::protobuf_codec;
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

std::vector<Capabilities>
ServiceHandler::handleReadCapabilities(const Query& /*request*/) {
    return {};
}

Identifier
ServiceHandler::handleCreateSession(const Session& /*request*/) {
    return {};
}

std::vector<SessionSnapshot>
ServiceHandler::handleReadSession(const Query& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleUpdateSession(const SessionStepRequest& /*request*/) {
    return pyramid::data_model::kAckOk;
}

Ack
ServiceHandler::handleDeleteSession(const SessionStopRequest& /*request*/) {
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

Identifier
ServiceHandler::handleCreateIntent(const MissionIntent& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleUpdateIntent(const MissionIntent& /*request*/) {
    return pyramid::data_model::kAckOk;
}

Ack
ServiceHandler::handleDeleteIntent(const Identifier& /*request*/) {
    return pyramid::data_model::kAckOk;
}

std::vector<Command>
ServiceHandler::handleReadCommand(const Query& /*request*/) {
    return {};
}

std::vector<GoalDispatch>
ServiceHandler::handleReadGoalDispatch(const Query& /*request*/) {
    return {};
}

std::vector<DecisionRecord>
ServiceHandler::handleReadDecisionRecord(const Query& /*request*/) {
    return {};
}

Identifier
ServiceHandler::handleCreateCommandResult(const CommandResult& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleUpdateCommandResult(const CommandResult& /*request*/) {
    return pyramid::data_model::kAckOk;
}

Ack
ServiceHandler::handleDeleteCommandResult(const Identifier& /*request*/) {
    return pyramid::data_model::kAckOk;
}

Identifier
ServiceHandler::handleCreateDispatchResult(const DispatchResult& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleUpdateDispatchResult(const DispatchResult& /*request*/) {
    return pyramid::data_model::kAckOk;
}

Ack
ServiceHandler::handleDeleteDispatchResult(const Identifier& /*request*/) {
    return pyramid::data_model::kAckOk;
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

bool is_protobuf_content_type(const char* content_type)
{
    return content_type && std::strcmp(content_type, kProtobufContentType) == 0;
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
// Content-type support metadata
// ---------------------------------------------------------------------------

std::vector<const char*> supportedContentTypes()
{
    std::vector<const char*> result{kJsonContentType};
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
    result.push_back(kFlatBuffersContentType);
#endif
#if PYRAMID_HAVE_SERVICE_PROTOBUF
    result.push_back(kProtobufContentType);
#endif
    return result;
}

bool supportsContentType(const char* content_type)
{
    if (is_json_content_type(content_type)) {
        return true;
    }
    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        return true;
#else
        return false;
#endif
    }
    if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        return true;
#else
        return false;
#endif
    }
    return false;
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
    if (is_flatbuffers_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS
        payload = flatbuffers_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcReadCapabilities, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeCreateSession(pcl_executor_t* executor,
                                 const Session&               request,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcCreateSession, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeReadSession(pcl_executor_t* executor,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcReadSession, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeUpdateSession(pcl_executor_t* executor,
                                 const SessionStepRequest&    request,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcUpdateSession, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeDeleteSession(pcl_executor_t* executor,
                                 const SessionStopRequest&    request,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcDeleteSession, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeCreateState(pcl_executor_t* executor,
                               const StateUpdate&           request,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcCreateState, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeUpdateState(pcl_executor_t* executor,
                               const StateUpdate&           request,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcUpdateState, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeDeleteState(pcl_executor_t* executor,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = encode_identifier_payload(request);
    }
    return invoke_async(executor, kSvcDeleteState, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeCreateIntent(pcl_executor_t* executor,
                                const MissionIntent&         request,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcCreateIntent, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeUpdateIntent(pcl_executor_t* executor,
                                const MissionIntent&         request,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcUpdateIntent, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeDeleteIntent(pcl_executor_t* executor,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = encode_identifier_payload(request);
    }
    return invoke_async(executor, kSvcDeleteIntent, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeReadCommand(pcl_executor_t* executor,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcReadCommand, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeReadGoalDispatch(pcl_executor_t* executor,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcReadGoalDispatch, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeReadDecisionRecord(pcl_executor_t* executor,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcReadDecisionRecord, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeCreateCommandResult(pcl_executor_t* executor,
                                       const CommandResult&         request,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcCreateCommandResult, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeUpdateCommandResult(pcl_executor_t* executor,
                                       const CommandResult&         request,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcUpdateCommandResult, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeDeleteCommandResult(pcl_executor_t* executor,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = encode_identifier_payload(request);
    }
    return invoke_async(executor, kSvcDeleteCommandResult, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeCreateDispatchResult(pcl_executor_t* executor,
                                        const DispatchResult&        request,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcCreateDispatchResult, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeUpdateDispatchResult(pcl_executor_t* executor,
                                        const DispatchResult&        request,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = toJson(request);
    }
    return invoke_async(executor, kSvcUpdateDispatchResult, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeDeleteDispatchResult(pcl_executor_t* executor,
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
    } else if (is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        payload = protobuf_codec::toBinary(request);
#else
        return PCL_ERR_INVALID;
#endif
    } else if (!is_json_content_type(content_type)) {
        return PCL_ERR_INVALID;
    } else {
        payload = encode_identifier_payload(request);
    }
    return invoke_async(executor, kSvcDeleteDispatchResult, payload, callback, user_data, route, content_type);
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
    } else if (is_protobuf_content_type(content_type)) {
#if !PYRAMID_HAVE_SERVICE_PROTOBUF
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
    case ServiceChannel::ReadCapabilities: {
        Query req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryQuery(request_buf, request_size);
#endif
        else
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadCapabilities(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
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
    case ServiceChannel::CreateSession: {
        Session req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinarySession(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinarySession(request_buf, request_size);
#endif
        else
            req = fromJson(req_str, static_cast<Session*>(nullptr));
        auto rsp = handler.handleCreateSession(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
        } else {
            rsp_payload = encode_identifier_payload(rsp);
        }
        break;
    }
    case ServiceChannel::ReadSession: {
        Query req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryQuery(request_buf, request_size);
#endif
        else
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadSession(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
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
    case ServiceChannel::UpdateSession: {
        SessionStepRequest req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinarySessionStepRequest(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinarySessionStepRequest(request_buf, request_size);
#endif
        else
            req = fromJson(req_str, static_cast<SessionStepRequest*>(nullptr));
        auto rsp = handler.handleUpdateSession(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
        } else {
            rsp_payload = toJson(rsp);
        }
        break;
    }
    case ServiceChannel::DeleteSession: {
        SessionStopRequest req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinarySessionStopRequest(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinarySessionStopRequest(request_buf, request_size);
#endif
        else
            req = fromJson(req_str, static_cast<SessionStopRequest*>(nullptr));
        auto rsp = handler.handleDeleteSession(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
        } else {
            rsp_payload = toJson(rsp);
        }
        break;
    }
    case ServiceChannel::CreateState: {
        StateUpdate req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryStateUpdate(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryStateUpdate(request_buf, request_size);
#endif
        else
            req = fromJson(req_str, static_cast<StateUpdate*>(nullptr));
        auto rsp = handler.handleCreateState(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
        } else {
            rsp_payload = encode_identifier_payload(rsp);
        }
        break;
    }
    case ServiceChannel::UpdateState: {
        StateUpdate req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryStateUpdate(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryStateUpdate(request_buf, request_size);
#endif
        else
            req = fromJson(req_str, static_cast<StateUpdate*>(nullptr));
        auto rsp = handler.handleUpdateState(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
        } else {
            rsp_payload = toJson(rsp);
        }
        break;
    }
    case ServiceChannel::DeleteState: {
        Identifier req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryIdentifier(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryIdentifier(request_buf, request_size);
#endif
        else
            req = decode_identifier_payload(req_str);
        auto rsp = handler.handleDeleteState(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
        } else {
            rsp_payload = toJson(rsp);
        }
        break;
    }
    case ServiceChannel::CreateIntent: {
        MissionIntent req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryMissionIntent(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryMissionIntent(request_buf, request_size);
#endif
        else
            req = fromJson(req_str, static_cast<MissionIntent*>(nullptr));
        auto rsp = handler.handleCreateIntent(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
        } else {
            rsp_payload = encode_identifier_payload(rsp);
        }
        break;
    }
    case ServiceChannel::UpdateIntent: {
        MissionIntent req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryMissionIntent(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryMissionIntent(request_buf, request_size);
#endif
        else
            req = fromJson(req_str, static_cast<MissionIntent*>(nullptr));
        auto rsp = handler.handleUpdateIntent(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
        } else {
            rsp_payload = toJson(rsp);
        }
        break;
    }
    case ServiceChannel::DeleteIntent: {
        Identifier req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryIdentifier(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryIdentifier(request_buf, request_size);
#endif
        else
            req = decode_identifier_payload(req_str);
        auto rsp = handler.handleDeleteIntent(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
        } else {
            rsp_payload = toJson(rsp);
        }
        break;
    }
    case ServiceChannel::ReadCommand: {
        Query req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryQuery(request_buf, request_size);
#endif
        else
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadCommand(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
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
    case ServiceChannel::ReadGoalDispatch: {
        Query req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryQuery(request_buf, request_size);
#endif
        else
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadGoalDispatch(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
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
    case ServiceChannel::ReadDecisionRecord: {
        Query req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryQuery(request_buf, request_size);
#endif
        else
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        auto rsp = handler.handleReadDecisionRecord(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
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
    case ServiceChannel::CreateCommandResult: {
        CommandResult req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryCommandResult(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryCommandResult(request_buf, request_size);
#endif
        else
            req = fromJson(req_str, static_cast<CommandResult*>(nullptr));
        auto rsp = handler.handleCreateCommandResult(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
        } else {
            rsp_payload = encode_identifier_payload(rsp);
        }
        break;
    }
    case ServiceChannel::UpdateCommandResult: {
        CommandResult req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryCommandResult(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryCommandResult(request_buf, request_size);
#endif
        else
            req = fromJson(req_str, static_cast<CommandResult*>(nullptr));
        auto rsp = handler.handleUpdateCommandResult(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
        } else {
            rsp_payload = toJson(rsp);
        }
        break;
    }
    case ServiceChannel::DeleteCommandResult: {
        Identifier req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryIdentifier(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryIdentifier(request_buf, request_size);
#endif
        else
            req = decode_identifier_payload(req_str);
        auto rsp = handler.handleDeleteCommandResult(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
        } else {
            rsp_payload = toJson(rsp);
        }
        break;
    }
    case ServiceChannel::CreateDispatchResult: {
        DispatchResult req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryDispatchResult(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryDispatchResult(request_buf, request_size);
#endif
        else
            req = fromJson(req_str, static_cast<DispatchResult*>(nullptr));
        auto rsp = handler.handleCreateDispatchResult(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
        } else {
            rsp_payload = encode_identifier_payload(rsp);
        }
        break;
    }
    case ServiceChannel::UpdateDispatchResult: {
        DispatchResult req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryDispatchResult(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryDispatchResult(request_buf, request_size);
#endif
        else
            req = fromJson(req_str, static_cast<DispatchResult*>(nullptr));
        auto rsp = handler.handleUpdateDispatchResult(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
        } else {
            rsp_payload = toJson(rsp);
        }
        break;
    }
    case ServiceChannel::DeleteDispatchResult: {
        Identifier req;
        if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryIdentifier(request_buf, request_size);
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        else if (is_protobuf_content_type(content_type))
            req = protobuf_codec::fromBinaryIdentifier(request_buf, request_size);
#endif
        else
            req = decode_identifier_payload(req_str);
        auto rsp = handler.handleDeleteDispatchResult(req);
        if (is_flatbuffers_content_type(content_type)) {
            rsp_payload = flatbuffers_codec::toBinary(rsp);
            rsp_is_binary = true;
#if PYRAMID_HAVE_SERVICE_PROTOBUF
        } else if (is_protobuf_content_type(content_type)) {
            rsp_payload = protobuf_codec::toBinary(rsp);
            rsp_is_binary = true;
#endif
        } else {
            rsp_payload = toJson(rsp);
        }
        break;
    }
    }
    } catch (...) {
        *response_buf = nullptr;
        *response_size = 0;
        return;
    }

    if (is_flatbuffers_content_type(content_type) ||
        is_protobuf_content_type(content_type)) {
#if PYRAMID_HAVE_SERVICE_FLATBUFFERS || PYRAMID_HAVE_SERVICE_PROTOBUF
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

} // namespace pyramid::services::autonomy_backend::provided
