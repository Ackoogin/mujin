// Auto-generated service binding implementation
// Generated from: pyramid.components.autonomy_backend.services.provided.proto by generate_bindings.py
// Namespace: pyramid::components::autonomy_backend::services::provided

#include "pyramid_services_autonomy_backend_provided.hpp"

#include "flatbuffers/cpp/pyramid_services_autonomy_backend_flatbuffers_codec.hpp"
#include "pyramid_data_model_autonomy_codec.hpp"
#include "pyramid_data_model_common_codec.hpp"
#include "pyramid_data_model_autonomy_cabi_marshal.hpp"
#include "pyramid_data_model_common_cabi_marshal.hpp"

extern "C" {
#include <pcl/pcl_codec.h>
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport.h>
}

#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace pyramid::components::autonomy_backend::services::provided {

// Bring data model codec functions into scope
using pyramid::domain_model::autonomy::toJson;
using pyramid::domain_model::autonomy::fromJson;
using pyramid::domain_model::common::toJson;
using pyramid::domain_model::common::fromJson;
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
ServiceHandler::handleCapabilitiesReadCapabilities(const Query& /*request*/) {
    return {};
}

pcl_status_t
ServiceHandler::streamCapabilitiesReadCapabilities(const Query& /*request*/,
    pcl_stream_context_t* /*stream_context*/,
    const char* /*content_type*/) {
    return PCL_ERR_INVALID;
}

Identifier
ServiceHandler::handlePlanningRequirementCreatePlanningRequirement(const PlanningRequirement& /*request*/) {
    return {};
}

std::vector<PlanningRequirement>
ServiceHandler::handlePlanningRequirementReadPlanningRequirement(const Query& /*request*/) {
    return {};
}

pcl_status_t
ServiceHandler::streamPlanningRequirementReadPlanningRequirement(const Query& /*request*/,
    pcl_stream_context_t* /*stream_context*/,
    const char* /*content_type*/) {
    return PCL_ERR_INVALID;
}

Ack
ServiceHandler::handlePlanningRequirementUpdatePlanningRequirement(const PlanningRequirement& /*request*/) {
    return pyramid::domain_model::kAckOk;
}

Ack
ServiceHandler::handlePlanningRequirementDeletePlanningRequirement(const Identifier& /*request*/) {
    return pyramid::domain_model::kAckOk;
}

Identifier
ServiceHandler::handleExecutionRequirementCreateExecutionRequirement(const ExecutionRequirement& /*request*/) {
    return {};
}

std::vector<ExecutionRequirement>
ServiceHandler::handleExecutionRequirementReadExecutionRequirement(const Query& /*request*/) {
    return {};
}

pcl_status_t
ServiceHandler::streamExecutionRequirementReadExecutionRequirement(const Query& /*request*/,
    pcl_stream_context_t* /*stream_context*/,
    const char* /*content_type*/) {
    return PCL_ERR_INVALID;
}

Ack
ServiceHandler::handleExecutionRequirementUpdateExecutionRequirement(const ExecutionRequirement& /*request*/) {
    return pyramid::domain_model::kAckOk;
}

Ack
ServiceHandler::handleExecutionRequirementDeleteExecutionRequirement(const Identifier& /*request*/) {
    return pyramid::domain_model::kAckOk;
}

Identifier
ServiceHandler::handleStateCreateState(const StateUpdate& /*request*/) {
    return {};
}

Ack
ServiceHandler::handleStateUpdateState(const StateUpdate& /*request*/) {
    return pyramid::domain_model::kAckOk;
}

Ack
ServiceHandler::handleStateDeleteState(const Identifier& /*request*/) {
    return pyramid::domain_model::kAckOk;
}

Identifier
ServiceHandler::handlePlanCreatePlan(const Plan& /*request*/) {
    return {};
}

std::vector<Plan>
ServiceHandler::handlePlanReadPlan(const Query& /*request*/) {
    return {};
}

pcl_status_t
ServiceHandler::streamPlanReadPlan(const Query& /*request*/,
    pcl_stream_context_t* /*stream_context*/,
    const char* /*content_type*/) {
    return PCL_ERR_INVALID;
}

Ack
ServiceHandler::handlePlanUpdatePlan(const Plan& /*request*/) {
    return pyramid::domain_model::kAckOk;
}

Ack
ServiceHandler::handlePlanDeletePlan(const Identifier& /*request*/) {
    return pyramid::domain_model::kAckOk;
}

std::vector<ExecutionRun>
ServiceHandler::handleExecutionRunReadRun(const Query& /*request*/) {
    return {};
}

pcl_status_t
ServiceHandler::streamExecutionRunReadRun(const Query& /*request*/,
    pcl_stream_context_t* /*stream_context*/,
    const char* /*content_type*/) {
    return PCL_ERR_INVALID;
}

std::vector<RequirementPlacement>
ServiceHandler::handleRequirementPlacementReadPlacement(const Query& /*request*/) {
    return {};
}

pcl_status_t
ServiceHandler::streamRequirementPlacementReadPlacement(const Query& /*request*/,
    pcl_stream_context_t* /*stream_context*/,
    const char* /*content_type*/) {
    return PCL_ERR_INVALID;
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

static int pyramid_cabi_encode(const pcl_codec_t* c,
                               const char* schema_id,
                               const void* value,
                               pcl_msg_t* out_msg)
{
    if (std::strcmp(schema_id, "Query") == 0) {
        pyramid_Query_c cs;
        pyramid::cabi::to_c(
            *static_cast<const pyramid::domain_model::Query*>(value), &cs);
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &cs, out_msg);
        pyramid_Query_c_free(&cs);
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "Capabilities") == 0) {
        pyramid_Capabilities_c cs;
        pyramid::cabi::to_c(
            *static_cast<const pyramid::domain_model::Capabilities*>(value), &cs);
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &cs, out_msg);
        pyramid_Capabilities_c_free(&cs);
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "PlanningRequirement") == 0) {
        pyramid_PlanningRequirement_c cs;
        pyramid::cabi::to_c(
            *static_cast<const pyramid::domain_model::PlanningRequirement*>(value), &cs);
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &cs, out_msg);
        pyramid_PlanningRequirement_c_free(&cs);
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "Ack") == 0) {
        pyramid_Ack_c cs;
        pyramid::cabi::to_c(
            *static_cast<const pyramid::domain_model::Ack*>(value), &cs);
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &cs, out_msg);
        pyramid_Ack_c_free(&cs);
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "ExecutionRequirement") == 0) {
        pyramid_ExecutionRequirement_c cs;
        pyramid::cabi::to_c(
            *static_cast<const pyramid::domain_model::ExecutionRequirement*>(value), &cs);
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &cs, out_msg);
        pyramid_ExecutionRequirement_c_free(&cs);
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "StateUpdate") == 0) {
        pyramid_StateUpdate_c cs;
        pyramid::cabi::to_c(
            *static_cast<const pyramid::domain_model::StateUpdate*>(value), &cs);
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &cs, out_msg);
        pyramid_StateUpdate_c_free(&cs);
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "Plan") == 0) {
        pyramid_Plan_c cs;
        pyramid::cabi::to_c(
            *static_cast<const pyramid::domain_model::Plan*>(value), &cs);
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &cs, out_msg);
        pyramid_Plan_c_free(&cs);
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "ExecutionRun") == 0) {
        pyramid_ExecutionRun_c cs;
        pyramid::cabi::to_c(
            *static_cast<const pyramid::domain_model::ExecutionRun*>(value), &cs);
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &cs, out_msg);
        pyramid_ExecutionRun_c_free(&cs);
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "RequirementPlacement") == 0) {
        pyramid_RequirementPlacement_c cs;
        pyramid::cabi::to_c(
            *static_cast<const pyramid::domain_model::RequirementPlacement*>(value), &cs);
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &cs, out_msg);
        pyramid_RequirementPlacement_c_free(&cs);
        return rc == PCL_OK ? 1 : -1;
    }
    (void)c; (void)value; (void)out_msg;
    return 0;
}

static int pyramid_cabi_decode(const pcl_codec_t* c,
                               const char* schema_id,
                               const pcl_msg_t* msg,
                               void* out_value)
{
    if (std::strcmp(schema_id, "Query") == 0) {
        pyramid_Query_c cs;
        std::memset(&cs, 0, sizeof(cs));
        if (c->decode(c->codec_ctx, schema_id, msg, &cs)
                != PCL_OK) {
            pyramid_Query_c_free(&cs);
            return -1;
        }
        pyramid::cabi::from_c(
            &cs, *static_cast<pyramid::domain_model::Query*>(out_value));
        pyramid_Query_c_free(&cs);
        return 1;
    }
    if (std::strcmp(schema_id, "Capabilities") == 0) {
        pyramid_Capabilities_c cs;
        std::memset(&cs, 0, sizeof(cs));
        if (c->decode(c->codec_ctx, schema_id, msg, &cs)
                != PCL_OK) {
            pyramid_Capabilities_c_free(&cs);
            return -1;
        }
        pyramid::cabi::from_c(
            &cs, *static_cast<pyramid::domain_model::Capabilities*>(out_value));
        pyramid_Capabilities_c_free(&cs);
        return 1;
    }
    if (std::strcmp(schema_id, "PlanningRequirement") == 0) {
        pyramid_PlanningRequirement_c cs;
        std::memset(&cs, 0, sizeof(cs));
        if (c->decode(c->codec_ctx, schema_id, msg, &cs)
                != PCL_OK) {
            pyramid_PlanningRequirement_c_free(&cs);
            return -1;
        }
        pyramid::cabi::from_c(
            &cs, *static_cast<pyramid::domain_model::PlanningRequirement*>(out_value));
        pyramid_PlanningRequirement_c_free(&cs);
        return 1;
    }
    if (std::strcmp(schema_id, "Ack") == 0) {
        pyramid_Ack_c cs;
        std::memset(&cs, 0, sizeof(cs));
        if (c->decode(c->codec_ctx, schema_id, msg, &cs)
                != PCL_OK) {
            pyramid_Ack_c_free(&cs);
            return -1;
        }
        pyramid::cabi::from_c(
            &cs, *static_cast<pyramid::domain_model::Ack*>(out_value));
        pyramid_Ack_c_free(&cs);
        return 1;
    }
    if (std::strcmp(schema_id, "ExecutionRequirement") == 0) {
        pyramid_ExecutionRequirement_c cs;
        std::memset(&cs, 0, sizeof(cs));
        if (c->decode(c->codec_ctx, schema_id, msg, &cs)
                != PCL_OK) {
            pyramid_ExecutionRequirement_c_free(&cs);
            return -1;
        }
        pyramid::cabi::from_c(
            &cs, *static_cast<pyramid::domain_model::ExecutionRequirement*>(out_value));
        pyramid_ExecutionRequirement_c_free(&cs);
        return 1;
    }
    if (std::strcmp(schema_id, "StateUpdate") == 0) {
        pyramid_StateUpdate_c cs;
        std::memset(&cs, 0, sizeof(cs));
        if (c->decode(c->codec_ctx, schema_id, msg, &cs)
                != PCL_OK) {
            pyramid_StateUpdate_c_free(&cs);
            return -1;
        }
        pyramid::cabi::from_c(
            &cs, *static_cast<pyramid::domain_model::StateUpdate*>(out_value));
        pyramid_StateUpdate_c_free(&cs);
        return 1;
    }
    if (std::strcmp(schema_id, "Plan") == 0) {
        pyramid_Plan_c cs;
        std::memset(&cs, 0, sizeof(cs));
        if (c->decode(c->codec_ctx, schema_id, msg, &cs)
                != PCL_OK) {
            pyramid_Plan_c_free(&cs);
            return -1;
        }
        pyramid::cabi::from_c(
            &cs, *static_cast<pyramid::domain_model::Plan*>(out_value));
        pyramid_Plan_c_free(&cs);
        return 1;
    }
    if (std::strcmp(schema_id, "ExecutionRun") == 0) {
        pyramid_ExecutionRun_c cs;
        std::memset(&cs, 0, sizeof(cs));
        if (c->decode(c->codec_ctx, schema_id, msg, &cs)
                != PCL_OK) {
            pyramid_ExecutionRun_c_free(&cs);
            return -1;
        }
        pyramid::cabi::from_c(
            &cs, *static_cast<pyramid::domain_model::ExecutionRun*>(out_value));
        pyramid_ExecutionRun_c_free(&cs);
        return 1;
    }
    if (std::strcmp(schema_id, "RequirementPlacement") == 0) {
        pyramid_RequirementPlacement_c cs;
        std::memset(&cs, 0, sizeof(cs));
        if (c->decode(c->codec_ctx, schema_id, msg, &cs)
                != PCL_OK) {
            pyramid_RequirementPlacement_c_free(&cs);
            return -1;
        }
        pyramid::cabi::from_c(
            &cs, *static_cast<pyramid::domain_model::RequirementPlacement*>(out_value));
        pyramid_RequirementPlacement_c_free(&cs);
        return 1;
    }
    (void)c; (void)msg; (void)out_value;
    return 0;
}

static int pyramid_try_registry_encode(const char* content_type,
                                       const char* schema_id,
                                       const void* value,
                                       std::string* out)
{
    pcl_codec_registry_t* reg = pcl_codec_registry_default();
    for (uint32_t i = 0; ; ++i) {
        const pcl_codec_t* c =
            pcl_codec_registry_get_at(reg, content_type, i);
        if (!c) {
            break;
        }
        if (!c->encode) {
            continue;
        }
        pcl_msg_t m;
        m.data = nullptr;
        m.size = 0;
        m.type_name = nullptr;
        const int r = pyramid_cabi_encode(c, schema_id, value, &m);
        if (r == 1) {
            out->assign(static_cast<const char*>(m.data), m.size);
            if (c->free_msg) {
                c->free_msg(c->codec_ctx, &m);
            }
            return 1;
        }
        if (r == 0) {
            return -1;
        }
    }
    return -1;
}

static int pyramid_try_registry_decode(const pcl_msg_t* msg,
                                       const char* schema_id,
                                       void* out_value)
{
    if (!msg) {
        return -1;
    }
    pcl_codec_registry_t* reg = pcl_codec_registry_default();
    for (uint32_t i = 0; ; ++i) {
        const pcl_codec_t* c =
            pcl_codec_registry_get_at(reg, msg->type_name, i);
        if (!c) {
            break;
        }
        if (!c->decode) {
            continue;
        }
        const int r =
            pyramid_cabi_decode(c, schema_id, msg, out_value);
        if (r == 1) {
            return 1;
        }
        if (r == 0) {
            return -1;
        }
    }
    return -1;
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
    if (pcl_codec_registry_get(pcl_codec_registry_default(), content_type) != nullptr) {
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Typed response decode wrappers
// ---------------------------------------------------------------------------

bool decodeCapabilitiesReadCapabilitiesResponse(const pcl_msg_t* msg,
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

bool encodeCapabilitiesReadCapabilitiesStreamFrame(const Capabilities& payload,
                                                   const char*        content_type,
                                                   std::string*       out)
{
    if (!out) {
        return false;
    }
    { int r = pyramid_try_registry_encode(content_type, "Capabilities", &payload, out); if (r == 1) return true; }
    if (is_json_content_type(content_type)) {
        *out = toJson(payload);
    } else if (is_flatbuffers_content_type(content_type)) {
        *out = flatbuffers_codec::toBinary(payload);
    } else {
        return false;
    }
    return true;
}

bool decodeCapabilitiesReadCapabilitiesStreamFrame(const pcl_msg_t* msg,
                                                   Capabilities* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Capabilities", out); if (r == 1) return true; }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryCapabilities(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        *out = fromJson(payload, static_cast<Capabilities*>(nullptr));
        return true;
    } catch (...) {
        return false;
    }
}

pcl_status_t sendCapabilitiesReadCapabilitiesStreamFrame(pcl_stream_context_t* stream_context,
                                                         const Capabilities& payload,
                                                         const char*        content_type)
{
    std::string wire_payload;
    if (!encodeCapabilitiesReadCapabilitiesStreamFrame(payload, content_type, &wire_payload)) {
        return PCL_ERR_INVALID;
    }
    pcl_msg_t msg{};
    msg.data = wire_payload.data();
    msg.size = static_cast<uint32_t>(wire_payload.size());
    msg.type_name = content_type;
    return pcl_stream_send(stream_context, &msg);
}

bool decodePlanningRequirementCreatePlanningRequirementResponse(const pcl_msg_t* msg,
                                                                Identifier* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Identifier", out); if (r == 1) return true; }
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

bool decodePlanningRequirementReadPlanningRequirementResponse(const pcl_msg_t* msg,
                                                              std::vector<PlanningRequirement>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryPlanningRequirementArray(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        const auto arr = nlohmann::json::parse(payload);
        out->clear();
        for (const auto& item : arr) {
            out->push_back(fromJson(item.dump(), static_cast<PlanningRequirement*>(nullptr)));
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool encodePlanningRequirementReadPlanningRequirementStreamFrame(const PlanningRequirement& payload,
                                                                 const char*        content_type,
                                                                 std::string*       out)
{
    if (!out) {
        return false;
    }
    { int r = pyramid_try_registry_encode(content_type, "PlanningRequirement", &payload, out); if (r == 1) return true; }
    if (is_json_content_type(content_type)) {
        *out = toJson(payload);
    } else if (is_flatbuffers_content_type(content_type)) {
        *out = flatbuffers_codec::toBinary(payload);
    } else {
        return false;
    }
    return true;
}

bool decodePlanningRequirementReadPlanningRequirementStreamFrame(const pcl_msg_t* msg,
                                                                 PlanningRequirement* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "PlanningRequirement", out); if (r == 1) return true; }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryPlanningRequirement(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        *out = fromJson(payload, static_cast<PlanningRequirement*>(nullptr));
        return true;
    } catch (...) {
        return false;
    }
}

pcl_status_t sendPlanningRequirementReadPlanningRequirementStreamFrame(pcl_stream_context_t* stream_context,
                                                                       const PlanningRequirement& payload,
                                                                       const char*        content_type)
{
    std::string wire_payload;
    if (!encodePlanningRequirementReadPlanningRequirementStreamFrame(payload, content_type, &wire_payload)) {
        return PCL_ERR_INVALID;
    }
    pcl_msg_t msg{};
    msg.data = wire_payload.data();
    msg.size = static_cast<uint32_t>(wire_payload.size());
    msg.type_name = content_type;
    return pcl_stream_send(stream_context, &msg);
}

bool decodePlanningRequirementUpdatePlanningRequirementResponse(const pcl_msg_t* msg,
                                                                Ack* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Ack", out); if (r == 1) return true; }
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

bool decodePlanningRequirementDeletePlanningRequirementResponse(const pcl_msg_t* msg,
                                                                Ack* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Ack", out); if (r == 1) return true; }
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

bool decodeExecutionRequirementCreateExecutionRequirementResponse(const pcl_msg_t* msg,
                                                                  Identifier* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Identifier", out); if (r == 1) return true; }
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

bool decodeExecutionRequirementReadExecutionRequirementResponse(const pcl_msg_t* msg,
                                                                std::vector<ExecutionRequirement>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryExecutionRequirementArray(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        const auto arr = nlohmann::json::parse(payload);
        out->clear();
        for (const auto& item : arr) {
            out->push_back(fromJson(item.dump(), static_cast<ExecutionRequirement*>(nullptr)));
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool encodeExecutionRequirementReadExecutionRequirementStreamFrame(const ExecutionRequirement& payload,
                                                                   const char*        content_type,
                                                                   std::string*       out)
{
    if (!out) {
        return false;
    }
    { int r = pyramid_try_registry_encode(content_type, "ExecutionRequirement", &payload, out); if (r == 1) return true; }
    if (is_json_content_type(content_type)) {
        *out = toJson(payload);
    } else if (is_flatbuffers_content_type(content_type)) {
        *out = flatbuffers_codec::toBinary(payload);
    } else {
        return false;
    }
    return true;
}

bool decodeExecutionRequirementReadExecutionRequirementStreamFrame(const pcl_msg_t* msg,
                                                                   ExecutionRequirement* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "ExecutionRequirement", out); if (r == 1) return true; }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryExecutionRequirement(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        *out = fromJson(payload, static_cast<ExecutionRequirement*>(nullptr));
        return true;
    } catch (...) {
        return false;
    }
}

pcl_status_t sendExecutionRequirementReadExecutionRequirementStreamFrame(pcl_stream_context_t* stream_context,
                                                                         const ExecutionRequirement& payload,
                                                                         const char*        content_type)
{
    std::string wire_payload;
    if (!encodeExecutionRequirementReadExecutionRequirementStreamFrame(payload, content_type, &wire_payload)) {
        return PCL_ERR_INVALID;
    }
    pcl_msg_t msg{};
    msg.data = wire_payload.data();
    msg.size = static_cast<uint32_t>(wire_payload.size());
    msg.type_name = content_type;
    return pcl_stream_send(stream_context, &msg);
}

bool decodeExecutionRequirementUpdateExecutionRequirementResponse(const pcl_msg_t* msg,
                                                                  Ack* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Ack", out); if (r == 1) return true; }
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

bool decodeExecutionRequirementDeleteExecutionRequirementResponse(const pcl_msg_t* msg,
                                                                  Ack* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Ack", out); if (r == 1) return true; }
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

bool decodeStateCreateStateResponse(const pcl_msg_t* msg,
                                    Identifier* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Identifier", out); if (r == 1) return true; }
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

bool decodeStateUpdateStateResponse(const pcl_msg_t* msg,
                                    Ack* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Ack", out); if (r == 1) return true; }
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

bool decodeStateDeleteStateResponse(const pcl_msg_t* msg,
                                    Ack* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Ack", out); if (r == 1) return true; }
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

bool decodePlanCreatePlanResponse(const pcl_msg_t* msg,
                                  Identifier* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Identifier", out); if (r == 1) return true; }
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

bool decodePlanReadPlanResponse(const pcl_msg_t* msg,
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

bool encodePlanReadPlanStreamFrame(const Plan& payload,
                                   const char*        content_type,
                                   std::string*       out)
{
    if (!out) {
        return false;
    }
    { int r = pyramid_try_registry_encode(content_type, "Plan", &payload, out); if (r == 1) return true; }
    if (is_json_content_type(content_type)) {
        *out = toJson(payload);
    } else if (is_flatbuffers_content_type(content_type)) {
        *out = flatbuffers_codec::toBinary(payload);
    } else {
        return false;
    }
    return true;
}

bool decodePlanReadPlanStreamFrame(const pcl_msg_t* msg,
                                   Plan* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Plan", out); if (r == 1) return true; }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryPlan(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        *out = fromJson(payload, static_cast<Plan*>(nullptr));
        return true;
    } catch (...) {
        return false;
    }
}

pcl_status_t sendPlanReadPlanStreamFrame(pcl_stream_context_t* stream_context,
                                         const Plan& payload,
                                         const char*        content_type)
{
    std::string wire_payload;
    if (!encodePlanReadPlanStreamFrame(payload, content_type, &wire_payload)) {
        return PCL_ERR_INVALID;
    }
    pcl_msg_t msg{};
    msg.data = wire_payload.data();
    msg.size = static_cast<uint32_t>(wire_payload.size());
    msg.type_name = content_type;
    return pcl_stream_send(stream_context, &msg);
}

bool decodePlanUpdatePlanResponse(const pcl_msg_t* msg,
                                  Ack* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Ack", out); if (r == 1) return true; }
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

bool decodePlanDeletePlanResponse(const pcl_msg_t* msg,
                                  Ack* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Ack", out); if (r == 1) return true; }
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

bool decodeExecutionRunReadRunResponse(const pcl_msg_t* msg,
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

bool encodeExecutionRunReadRunStreamFrame(const ExecutionRun& payload,
                                          const char*        content_type,
                                          std::string*       out)
{
    if (!out) {
        return false;
    }
    { int r = pyramid_try_registry_encode(content_type, "ExecutionRun", &payload, out); if (r == 1) return true; }
    if (is_json_content_type(content_type)) {
        *out = toJson(payload);
    } else if (is_flatbuffers_content_type(content_type)) {
        *out = flatbuffers_codec::toBinary(payload);
    } else {
        return false;
    }
    return true;
}

bool decodeExecutionRunReadRunStreamFrame(const pcl_msg_t* msg,
                                          ExecutionRun* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "ExecutionRun", out); if (r == 1) return true; }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryExecutionRun(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        *out = fromJson(payload, static_cast<ExecutionRun*>(nullptr));
        return true;
    } catch (...) {
        return false;
    }
}

pcl_status_t sendExecutionRunReadRunStreamFrame(pcl_stream_context_t* stream_context,
                                                const ExecutionRun& payload,
                                                const char*        content_type)
{
    std::string wire_payload;
    if (!encodeExecutionRunReadRunStreamFrame(payload, content_type, &wire_payload)) {
        return PCL_ERR_INVALID;
    }
    pcl_msg_t msg{};
    msg.data = wire_payload.data();
    msg.size = static_cast<uint32_t>(wire_payload.size());
    msg.type_name = content_type;
    return pcl_stream_send(stream_context, &msg);
}

bool decodeRequirementPlacementReadPlacementResponse(const pcl_msg_t* msg,
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

bool encodeRequirementPlacementReadPlacementStreamFrame(const RequirementPlacement& payload,
                                                        const char*        content_type,
                                                        std::string*       out)
{
    if (!out) {
        return false;
    }
    { int r = pyramid_try_registry_encode(content_type, "RequirementPlacement", &payload, out); if (r == 1) return true; }
    if (is_json_content_type(content_type)) {
        *out = toJson(payload);
    } else if (is_flatbuffers_content_type(content_type)) {
        *out = flatbuffers_codec::toBinary(payload);
    } else {
        return false;
    }
    return true;
}

bool decodeRequirementPlacementReadPlacementStreamFrame(const pcl_msg_t* msg,
                                                        RequirementPlacement* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "RequirementPlacement", out); if (r == 1) return true; }
    try {
        if (!is_json_content_type(msg->type_name)) {
            if (is_flatbuffers_content_type(msg->type_name)) {
                *out = flatbuffers_codec::fromBinaryRequirementPlacement(msg->data, msg->size);
                return true;
            }
            return false;
        }
        const std::string payload = msgToString(msg->data, msg->size);
        *out = fromJson(payload, static_cast<RequirementPlacement*>(nullptr));
        return true;
    } catch (...) {
        return false;
    }
}

pcl_status_t sendRequirementPlacementReadPlacementStreamFrame(pcl_stream_context_t* stream_context,
                                                              const RequirementPlacement& payload,
                                                              const char*        content_type)
{
    std::string wire_payload;
    if (!encodeRequirementPlacementReadPlacementStreamFrame(payload, content_type, &wire_payload)) {
        return PCL_ERR_INVALID;
    }
    pcl_msg_t msg{};
    msg.data = wire_payload.data();
    msg.size = static_cast<uint32_t>(wire_payload.size());
    msg.type_name = content_type;
    return pcl_stream_send(stream_context, &msg);
}

// ---------------------------------------------------------------------------
// Typed invoke wrappers — serialise and dispatch via executor transport
// ---------------------------------------------------------------------------

pcl_status_t invokeCapabilitiesReadCapabilities(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcCapabilitiesReadCapabilities, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeCapabilitiesReadCapabilities(pcl_executor_t* executor,
                                                const Query&                 request,
                                                const char*       content_type,
                                                const pcl_endpoint_route_t* route)
{
    return invokeCapabilitiesReadCapabilities(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeCapabilitiesReadCapabilitiesStream(pcl_executor_t* executor,
                                                      const Query&                 request,
                                                      pcl_stream_msg_fn_t   callback,
                                                      void*                   user_data,
                                                      pcl_stream_context_t** out_context,
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
    if (route) {
        const pcl_status_t route_rc = pcl_executor_set_endpoint_route(executor, route);
        if (route_rc != PCL_OK) {
            return route_rc;
        }
    }
    pcl_msg_t msg{};
    msg.data = payload.data();
    msg.size = static_cast<uint32_t>(payload.size());
    msg.type_name = content_type;
    return pcl_executor_invoke_stream(executor, kSvcCapabilitiesReadCapabilities,
                                      &msg, callback, user_data,
                                      out_context);
}

pcl_status_t invokePlanningRequirementCreatePlanningRequirement(pcl_executor_t* executor,
                                                                const PlanningRequirement&   request,
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
    return invoke_async(executor, kSvcPlanningRequirementCreatePlanningRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokePlanningRequirementCreatePlanningRequirement(pcl_executor_t* executor,
                                                                const PlanningRequirement&   request,
                                                                const char*       content_type,
                                                                const pcl_endpoint_route_t* route)
{
    return invokePlanningRequirementCreatePlanningRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokePlanningRequirementReadPlanningRequirement(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcPlanningRequirementReadPlanningRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokePlanningRequirementReadPlanningRequirement(pcl_executor_t* executor,
                                                              const Query&                 request,
                                                              const char*       content_type,
                                                              const pcl_endpoint_route_t* route)
{
    return invokePlanningRequirementReadPlanningRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokePlanningRequirementReadPlanningRequirementStream(pcl_executor_t* executor,
                                                                    const Query&                 request,
                                                                    pcl_stream_msg_fn_t   callback,
                                                                    void*                   user_data,
                                                                    pcl_stream_context_t** out_context,
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
    if (route) {
        const pcl_status_t route_rc = pcl_executor_set_endpoint_route(executor, route);
        if (route_rc != PCL_OK) {
            return route_rc;
        }
    }
    pcl_msg_t msg{};
    msg.data = payload.data();
    msg.size = static_cast<uint32_t>(payload.size());
    msg.type_name = content_type;
    return pcl_executor_invoke_stream(executor, kSvcPlanningRequirementReadPlanningRequirement,
                                      &msg, callback, user_data,
                                      out_context);
}

pcl_status_t invokePlanningRequirementUpdatePlanningRequirement(pcl_executor_t* executor,
                                                                const PlanningRequirement&   request,
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
    return invoke_async(executor, kSvcPlanningRequirementUpdatePlanningRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokePlanningRequirementUpdatePlanningRequirement(pcl_executor_t* executor,
                                                                const PlanningRequirement&   request,
                                                                const char*       content_type,
                                                                const pcl_endpoint_route_t* route)
{
    return invokePlanningRequirementUpdatePlanningRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokePlanningRequirementDeletePlanningRequirement(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcPlanningRequirementDeletePlanningRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokePlanningRequirementDeletePlanningRequirement(pcl_executor_t* executor,
                                                                const Identifier&            request,
                                                                const char*       content_type,
                                                                const pcl_endpoint_route_t* route)
{
    return invokePlanningRequirementDeletePlanningRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeExecutionRequirementCreateExecutionRequirement(pcl_executor_t* executor,
                                                                  const ExecutionRequirement&  request,
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
    return invoke_async(executor, kSvcExecutionRequirementCreateExecutionRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeExecutionRequirementCreateExecutionRequirement(pcl_executor_t* executor,
                                                                  const ExecutionRequirement&  request,
                                                                  const char*       content_type,
                                                                  const pcl_endpoint_route_t* route)
{
    return invokeExecutionRequirementCreateExecutionRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeExecutionRequirementReadExecutionRequirement(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcExecutionRequirementReadExecutionRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeExecutionRequirementReadExecutionRequirement(pcl_executor_t* executor,
                                                                const Query&                 request,
                                                                const char*       content_type,
                                                                const pcl_endpoint_route_t* route)
{
    return invokeExecutionRequirementReadExecutionRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeExecutionRequirementReadExecutionRequirementStream(pcl_executor_t* executor,
                                                                      const Query&                 request,
                                                                      pcl_stream_msg_fn_t   callback,
                                                                      void*                   user_data,
                                                                      pcl_stream_context_t** out_context,
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
    if (route) {
        const pcl_status_t route_rc = pcl_executor_set_endpoint_route(executor, route);
        if (route_rc != PCL_OK) {
            return route_rc;
        }
    }
    pcl_msg_t msg{};
    msg.data = payload.data();
    msg.size = static_cast<uint32_t>(payload.size());
    msg.type_name = content_type;
    return pcl_executor_invoke_stream(executor, kSvcExecutionRequirementReadExecutionRequirement,
                                      &msg, callback, user_data,
                                      out_context);
}

pcl_status_t invokeExecutionRequirementUpdateExecutionRequirement(pcl_executor_t* executor,
                                                                  const ExecutionRequirement&  request,
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
    return invoke_async(executor, kSvcExecutionRequirementUpdateExecutionRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeExecutionRequirementUpdateExecutionRequirement(pcl_executor_t* executor,
                                                                  const ExecutionRequirement&  request,
                                                                  const char*       content_type,
                                                                  const pcl_endpoint_route_t* route)
{
    return invokeExecutionRequirementUpdateExecutionRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeExecutionRequirementDeleteExecutionRequirement(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcExecutionRequirementDeleteExecutionRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeExecutionRequirementDeleteExecutionRequirement(pcl_executor_t* executor,
                                                                  const Identifier&            request,
                                                                  const char*       content_type,
                                                                  const pcl_endpoint_route_t* route)
{
    return invokeExecutionRequirementDeleteExecutionRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeStateCreateState(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcStateCreateState, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeStateCreateState(pcl_executor_t* executor,
                                    const StateUpdate&           request,
                                    const char*       content_type,
                                    const pcl_endpoint_route_t* route)
{
    return invokeStateCreateState(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeStateUpdateState(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcStateUpdateState, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeStateUpdateState(pcl_executor_t* executor,
                                    const StateUpdate&           request,
                                    const char*       content_type,
                                    const pcl_endpoint_route_t* route)
{
    return invokeStateUpdateState(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeStateDeleteState(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcStateDeleteState, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeStateDeleteState(pcl_executor_t* executor,
                                    const Identifier&            request,
                                    const char*       content_type,
                                    const pcl_endpoint_route_t* route)
{
    return invokeStateDeleteState(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokePlanCreatePlan(pcl_executor_t* executor,
                                  const Plan&                  request,
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
    return invoke_async(executor, kSvcPlanCreatePlan, payload, callback, user_data, route, content_type);
}

pcl_status_t invokePlanCreatePlan(pcl_executor_t* executor,
                                  const Plan&                  request,
                                  const char*       content_type,
                                  const pcl_endpoint_route_t* route)
{
    return invokePlanCreatePlan(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokePlanReadPlan(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcPlanReadPlan, payload, callback, user_data, route, content_type);
}

pcl_status_t invokePlanReadPlan(pcl_executor_t* executor,
                                const Query&                 request,
                                const char*       content_type,
                                const pcl_endpoint_route_t* route)
{
    return invokePlanReadPlan(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokePlanReadPlanStream(pcl_executor_t* executor,
                                      const Query&                 request,
                                      pcl_stream_msg_fn_t   callback,
                                      void*                   user_data,
                                      pcl_stream_context_t** out_context,
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
    if (route) {
        const pcl_status_t route_rc = pcl_executor_set_endpoint_route(executor, route);
        if (route_rc != PCL_OK) {
            return route_rc;
        }
    }
    pcl_msg_t msg{};
    msg.data = payload.data();
    msg.size = static_cast<uint32_t>(payload.size());
    msg.type_name = content_type;
    return pcl_executor_invoke_stream(executor, kSvcPlanReadPlan,
                                      &msg, callback, user_data,
                                      out_context);
}

pcl_status_t invokePlanUpdatePlan(pcl_executor_t* executor,
                                  const Plan&                  request,
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
    return invoke_async(executor, kSvcPlanUpdatePlan, payload, callback, user_data, route, content_type);
}

pcl_status_t invokePlanUpdatePlan(pcl_executor_t* executor,
                                  const Plan&                  request,
                                  const char*       content_type,
                                  const pcl_endpoint_route_t* route)
{
    return invokePlanUpdatePlan(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokePlanDeletePlan(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcPlanDeletePlan, payload, callback, user_data, route, content_type);
}

pcl_status_t invokePlanDeletePlan(pcl_executor_t* executor,
                                  const Identifier&            request,
                                  const char*       content_type,
                                  const pcl_endpoint_route_t* route)
{
    return invokePlanDeletePlan(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeExecutionRunReadRun(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcExecutionRunReadRun, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeExecutionRunReadRun(pcl_executor_t* executor,
                                       const Query&                 request,
                                       const char*       content_type,
                                       const pcl_endpoint_route_t* route)
{
    return invokeExecutionRunReadRun(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeExecutionRunReadRunStream(pcl_executor_t* executor,
                                             const Query&                 request,
                                             pcl_stream_msg_fn_t   callback,
                                             void*                   user_data,
                                             pcl_stream_context_t** out_context,
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
    if (route) {
        const pcl_status_t route_rc = pcl_executor_set_endpoint_route(executor, route);
        if (route_rc != PCL_OK) {
            return route_rc;
        }
    }
    pcl_msg_t msg{};
    msg.data = payload.data();
    msg.size = static_cast<uint32_t>(payload.size());
    msg.type_name = content_type;
    return pcl_executor_invoke_stream(executor, kSvcExecutionRunReadRun,
                                      &msg, callback, user_data,
                                      out_context);
}

pcl_status_t invokeRequirementPlacementReadPlacement(pcl_executor_t* executor,
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
    return invoke_async(executor, kSvcRequirementPlacementReadPlacement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeRequirementPlacementReadPlacement(pcl_executor_t* executor,
                                                     const Query&                 request,
                                                     const char*       content_type,
                                                     const pcl_endpoint_route_t* route)
{
    return invokeRequirementPlacementReadPlacement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeRequirementPlacementReadPlacementStream(pcl_executor_t* executor,
                                                           const Query&                 request,
                                                           pcl_stream_msg_fn_t   callback,
                                                           void*                   user_data,
                                                           pcl_stream_context_t** out_context,
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
    if (route) {
        const pcl_status_t route_rc = pcl_executor_set_endpoint_route(executor, route);
        if (route_rc != PCL_OK) {
            return route_rc;
        }
    }
    pcl_msg_t msg{};
    msg.data = payload.data();
    msg.size = static_cast<uint32_t>(payload.size());
    msg.type_name = content_type;
    return pcl_executor_invoke_stream(executor, kSvcRequirementPlacementReadPlacement,
                                      &msg, callback, user_data,
                                      out_context);
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
    case ServiceChannel::CapabilitiesReadCapabilities: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleCapabilitiesReadCapabilities(req);
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
    case ServiceChannel::PlanningRequirementCreatePlanningRequirement: {
        PlanningRequirement req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<PlanningRequirement*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryPlanningRequirement(request_buf, request_size);
        else
            break;
        auto rsp = handler.handlePlanningRequirementCreatePlanningRequirement(req);
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
    case ServiceChannel::PlanningRequirementReadPlanningRequirement: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handlePlanningRequirementReadPlanningRequirement(req);
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
    case ServiceChannel::PlanningRequirementUpdatePlanningRequirement: {
        PlanningRequirement req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<PlanningRequirement*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryPlanningRequirement(request_buf, request_size);
        else
            break;
        auto rsp = handler.handlePlanningRequirementUpdatePlanningRequirement(req);
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
    case ServiceChannel::PlanningRequirementDeletePlanningRequirement: {
        Identifier req;
        if (is_json_content_type(content_type))
            req = decode_identifier_payload(req_str);
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryIdentifier(request_buf, request_size);
        else
            break;
        auto rsp = handler.handlePlanningRequirementDeletePlanningRequirement(req);
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
    case ServiceChannel::ExecutionRequirementCreateExecutionRequirement: {
        ExecutionRequirement req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<ExecutionRequirement*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryExecutionRequirement(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleExecutionRequirementCreateExecutionRequirement(req);
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
    case ServiceChannel::ExecutionRequirementReadExecutionRequirement: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleExecutionRequirementReadExecutionRequirement(req);
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
    case ServiceChannel::ExecutionRequirementUpdateExecutionRequirement: {
        ExecutionRequirement req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<ExecutionRequirement*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryExecutionRequirement(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleExecutionRequirementUpdateExecutionRequirement(req);
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
    case ServiceChannel::ExecutionRequirementDeleteExecutionRequirement: {
        Identifier req;
        if (is_json_content_type(content_type))
            req = decode_identifier_payload(req_str);
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryIdentifier(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleExecutionRequirementDeleteExecutionRequirement(req);
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
    case ServiceChannel::StateCreateState: {
        StateUpdate req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<StateUpdate*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryStateUpdate(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleStateCreateState(req);
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
    case ServiceChannel::StateUpdateState: {
        StateUpdate req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<StateUpdate*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryStateUpdate(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleStateUpdateState(req);
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
    case ServiceChannel::StateDeleteState: {
        Identifier req;
        if (is_json_content_type(content_type))
            req = decode_identifier_payload(req_str);
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryIdentifier(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleStateDeleteState(req);
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
    case ServiceChannel::PlanCreatePlan: {
        Plan req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Plan*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryPlan(request_buf, request_size);
        else
            break;
        auto rsp = handler.handlePlanCreatePlan(req);
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
    case ServiceChannel::PlanReadPlan: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handlePlanReadPlan(req);
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
    case ServiceChannel::PlanUpdatePlan: {
        Plan req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Plan*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryPlan(request_buf, request_size);
        else
            break;
        auto rsp = handler.handlePlanUpdatePlan(req);
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
    case ServiceChannel::PlanDeletePlan: {
        Identifier req;
        if (is_json_content_type(content_type))
            req = decode_identifier_payload(req_str);
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryIdentifier(request_buf, request_size);
        else
            break;
        auto rsp = handler.handlePlanDeletePlan(req);
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
    case ServiceChannel::ExecutionRunReadRun: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleExecutionRunReadRun(req);
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
    case ServiceChannel::RequirementPlacementReadPlacement: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        auto rsp = handler.handleRequirementPlacementReadPlacement(req);
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

// ---------------------------------------------------------------------------
// Stream dispatch — deserialise request and open stream
// ---------------------------------------------------------------------------

pcl_status_t dispatchStream(ServiceHandler& handler,
                            ServiceChannel  channel,
                            const void*     request_buf,
                            size_t          request_size,
                            const char*     content_type,
                            pcl_stream_context_t* stream_context)
{
    std::string req_str;

    if (is_json_content_type(content_type)) {
        req_str = json_request_body(request_buf, request_size);
        if (request_size != 0 && req_str.empty()) {
            return PCL_ERR_INVALID;
        }
    } else if (is_flatbuffers_content_type(content_type)) {
    } else {
        return PCL_ERR_INVALID;
    }

    try {
    switch (channel) {
    case ServiceChannel::CapabilitiesReadCapabilities: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        return handler.streamCapabilitiesReadCapabilities(req, stream_context, content_type);
    }
    case ServiceChannel::PlanningRequirementReadPlanningRequirement: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        return handler.streamPlanningRequirementReadPlanningRequirement(req, stream_context, content_type);
    }
    case ServiceChannel::ExecutionRequirementReadExecutionRequirement: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        return handler.streamExecutionRequirementReadExecutionRequirement(req, stream_context, content_type);
    }
    case ServiceChannel::PlanReadPlan: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        return handler.streamPlanReadPlan(req, stream_context, content_type);
    }
    case ServiceChannel::ExecutionRunReadRun: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        return handler.streamExecutionRunReadRun(req, stream_context, content_type);
    }
    case ServiceChannel::RequirementPlacementReadPlacement: {
        Query req;
        if (is_json_content_type(content_type))
            req = fromJson(req_str, static_cast<Query*>(nullptr));
        else if (is_flatbuffers_content_type(content_type))
            req = flatbuffers_codec::fromBinaryQuery(request_buf, request_size);
        else
            break;
        return handler.streamRequirementPlacementReadPlacement(req, stream_context, content_type);
    }
    }
    } catch (...) {
        return PCL_ERR_INVALID;
    }
    return PCL_ERR_INVALID;
}

} // namespace pyramid::components::autonomy_backend::services::provided
