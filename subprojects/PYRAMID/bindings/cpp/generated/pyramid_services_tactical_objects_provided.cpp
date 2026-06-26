// Auto-generated service binding implementation
// Generated from: pyramid.components.tactical_objects.services.provided.proto by generate_bindings.py
// Namespace: pyramid::components::tactical_objects::services::provided

#include "pyramid_services_tactical_objects_provided.hpp"

#include "pyramid_data_model_common_cabi_marshal.hpp"
#include "pyramid_data_model_tactical_cabi_marshal.hpp"

extern "C" {
#include <pcl/pcl_codec.h>
#include <pcl/pcl_codec_registry.h>
#include <pcl/pcl_container.h>
#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport.h>
#include "pyramid_datamodel_cabi.h"
}

#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace pyramid::components::tactical_objects::services::provided {


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
ServiceHandler::handleMatchingObjectsReadMatch(const Query& /*request*/) {
    return {};
}

pcl_status_t
ServiceHandler::streamMatchingObjectsReadMatch(const Query& /*request*/,
    pcl_stream_context_t* /*stream_context*/,
    const char* /*content_type*/) {
    return PCL_ERR_INVALID;
}

Identifier
ServiceHandler::handleObjectOfInterestCreateRequirement(const ObjectInterestRequirement& /*request*/) {
    return {};
}

std::vector<ObjectInterestRequirement>
ServiceHandler::handleObjectOfInterestReadRequirement(const Query& /*request*/) {
    return {};
}

pcl_status_t
ServiceHandler::streamObjectOfInterestReadRequirement(const Query& /*request*/,
    pcl_stream_context_t* /*stream_context*/,
    const char* /*content_type*/) {
    return PCL_ERR_INVALID;
}

Ack
ServiceHandler::handleObjectOfInterestUpdateRequirement(const ObjectInterestRequirement& /*request*/) {
    return pyramid::domain_model::kAckOk;
}

Ack
ServiceHandler::handleObjectOfInterestDeleteRequirement(const Identifier& /*request*/) {
    return pyramid::domain_model::kAckOk;
}

std::vector<ObjectDetail>
ServiceHandler::handleSpecificObjectDetailReadDetail(const Query& /*request*/) {
    return {};
}

pcl_status_t
ServiceHandler::streamSpecificObjectDetailReadDetail(const Query& /*request*/,
    pcl_stream_context_t* /*stream_context*/,
    const char* /*content_type*/) {
    return PCL_ERR_INVALID;
}

// ---------------------------------------------------------------------------
// Internal PCL helpers
// ---------------------------------------------------------------------------

namespace {

static int pyramid_cabi_encode(const pcl_codec_t* c,
                               const char* schema_id,
                               const void* value,
                               pcl_msg_t* out_msg)
{
    if (std::strcmp(schema_id, "Identifier") == 0) {
        const auto& native = *static_cast<const pyramid::domain_model::Identifier*>(value);
        if (native.size() > std::numeric_limits<uint32_t>::max()) {
            return -1;
        }
        pyramid_str_t cs;
        cs.ptr = native.data();
        cs.len = static_cast<uint32_t>(native.size());
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &cs, out_msg);
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "Query") == 0) {
        pyramid_Query_c cs;
        pyramid::cabi::to_c(
            *static_cast<const pyramid::domain_model::Query*>(value), &cs);
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &cs, out_msg);
        pyramid_Query_c_free(&cs);
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "ObjectDetail") == 0) {
        pyramid_ObjectDetail_c cs;
        pyramid::cabi::to_c(
            *static_cast<const pyramid::domain_model::ObjectDetail*>(value), &cs);
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &cs, out_msg);
        pyramid_ObjectDetail_c_free(&cs);
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "ObjectEvidenceRequirement") == 0) {
        pyramid_ObjectEvidenceRequirement_c cs;
        pyramid::cabi::to_c(
            *static_cast<const pyramid::domain_model::ObjectEvidenceRequirement*>(value), &cs);
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &cs, out_msg);
        pyramid_ObjectEvidenceRequirement_c_free(&cs);
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
    if (std::strcmp(schema_id, "Capability") == 0) {
        pyramid_Capability_c cs;
        pyramid::cabi::to_c(
            *static_cast<const pyramid::domain_model::Capability*>(value), &cs);
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &cs, out_msg);
        pyramid_Capability_c_free(&cs);
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "ObjectMatch") == 0) {
        pyramid_ObjectMatch_c cs;
        pyramid::cabi::to_c(
            *static_cast<const pyramid::domain_model::ObjectMatch*>(value), &cs);
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &cs, out_msg);
        pyramid_ObjectMatch_c_free(&cs);
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "ObjectInterestRequirement") == 0) {
        pyramid_ObjectInterestRequirement_c cs;
        pyramid::cabi::to_c(
            *static_cast<const pyramid::domain_model::ObjectInterestRequirement*>(value), &cs);
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &cs, out_msg);
        pyramid_ObjectInterestRequirement_c_free(&cs);
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "ObjectDetailArray") == 0) {
        const auto& native = *static_cast<const std::vector<pyramid::domain_model::ObjectDetail>*>(value);
        if (native.size() > std::numeric_limits<uint32_t>::max()) {
            return -1;
        }
        std::vector<pyramid_ObjectDetail_c> values(native.size());
        if (!values.empty()) {
            std::memset(values.data(), 0, values.size() * sizeof(values[0]));
        }
        for (std::size_t i = 0; i < native.size(); ++i) {
            pyramid::cabi::to_c(native[i], &values[i]);
        }
        pyramid_slice_t slice;
        slice.ptr = values.empty() ? nullptr : values.data();
        slice.len = static_cast<uint32_t>(values.size());
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &slice, out_msg);
        for (auto& item : values) {
            pyramid_ObjectDetail_c_free(&item);
        }
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "ObjectEvidenceRequirementArray") == 0) {
        const auto& native = *static_cast<const std::vector<pyramid::domain_model::ObjectEvidenceRequirement>*>(value);
        if (native.size() > std::numeric_limits<uint32_t>::max()) {
            return -1;
        }
        std::vector<pyramid_ObjectEvidenceRequirement_c> values(native.size());
        if (!values.empty()) {
            std::memset(values.data(), 0, values.size() * sizeof(values[0]));
        }
        for (std::size_t i = 0; i < native.size(); ++i) {
            pyramid::cabi::to_c(native[i], &values[i]);
        }
        pyramid_slice_t slice;
        slice.ptr = values.empty() ? nullptr : values.data();
        slice.len = static_cast<uint32_t>(values.size());
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &slice, out_msg);
        for (auto& item : values) {
            pyramid_ObjectEvidenceRequirement_c_free(&item);
        }
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "CapabilityArray") == 0) {
        const auto& native = *static_cast<const std::vector<pyramid::domain_model::Capability>*>(value);
        if (native.size() > std::numeric_limits<uint32_t>::max()) {
            return -1;
        }
        std::vector<pyramid_Capability_c> values(native.size());
        if (!values.empty()) {
            std::memset(values.data(), 0, values.size() * sizeof(values[0]));
        }
        for (std::size_t i = 0; i < native.size(); ++i) {
            pyramid::cabi::to_c(native[i], &values[i]);
        }
        pyramid_slice_t slice;
        slice.ptr = values.empty() ? nullptr : values.data();
        slice.len = static_cast<uint32_t>(values.size());
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &slice, out_msg);
        for (auto& item : values) {
            pyramid_Capability_c_free(&item);
        }
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "ObjectMatchArray") == 0) {
        const auto& native = *static_cast<const std::vector<pyramid::domain_model::ObjectMatch>*>(value);
        if (native.size() > std::numeric_limits<uint32_t>::max()) {
            return -1;
        }
        std::vector<pyramid_ObjectMatch_c> values(native.size());
        if (!values.empty()) {
            std::memset(values.data(), 0, values.size() * sizeof(values[0]));
        }
        for (std::size_t i = 0; i < native.size(); ++i) {
            pyramid::cabi::to_c(native[i], &values[i]);
        }
        pyramid_slice_t slice;
        slice.ptr = values.empty() ? nullptr : values.data();
        slice.len = static_cast<uint32_t>(values.size());
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &slice, out_msg);
        for (auto& item : values) {
            pyramid_ObjectMatch_c_free(&item);
        }
        return rc == PCL_OK ? 1 : -1;
    }
    if (std::strcmp(schema_id, "ObjectInterestRequirementArray") == 0) {
        const auto& native = *static_cast<const std::vector<pyramid::domain_model::ObjectInterestRequirement>*>(value);
        if (native.size() > std::numeric_limits<uint32_t>::max()) {
            return -1;
        }
        std::vector<pyramid_ObjectInterestRequirement_c> values(native.size());
        if (!values.empty()) {
            std::memset(values.data(), 0, values.size() * sizeof(values[0]));
        }
        for (std::size_t i = 0; i < native.size(); ++i) {
            pyramid::cabi::to_c(native[i], &values[i]);
        }
        pyramid_slice_t slice;
        slice.ptr = values.empty() ? nullptr : values.data();
        slice.len = static_cast<uint32_t>(values.size());
        const pcl_status_t rc =
            c->encode(c->codec_ctx, schema_id, &slice, out_msg);
        for (auto& item : values) {
            pyramid_ObjectInterestRequirement_c_free(&item);
        }
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
    if (std::strcmp(schema_id, "Identifier") == 0) {
        pyramid_str_t cs;
        std::memset(&cs, 0, sizeof(cs));
        if (c->decode(c->codec_ctx, schema_id, msg, &cs)
                != PCL_OK) {
            return -1;
        }
        auto* native = static_cast<pyramid::domain_model::Identifier*>(out_value);
        native->assign(cs.ptr ? cs.ptr : "", cs.len);
        std::free(const_cast<char*>(cs.ptr));
        return 1;
    }
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
    if (std::strcmp(schema_id, "ObjectDetail") == 0) {
        pyramid_ObjectDetail_c cs;
        std::memset(&cs, 0, sizeof(cs));
        if (c->decode(c->codec_ctx, schema_id, msg, &cs)
                != PCL_OK) {
            pyramid_ObjectDetail_c_free(&cs);
            return -1;
        }
        pyramid::cabi::from_c(
            &cs, *static_cast<pyramid::domain_model::ObjectDetail*>(out_value));
        pyramid_ObjectDetail_c_free(&cs);
        return 1;
    }
    if (std::strcmp(schema_id, "ObjectEvidenceRequirement") == 0) {
        pyramid_ObjectEvidenceRequirement_c cs;
        std::memset(&cs, 0, sizeof(cs));
        if (c->decode(c->codec_ctx, schema_id, msg, &cs)
                != PCL_OK) {
            pyramid_ObjectEvidenceRequirement_c_free(&cs);
            return -1;
        }
        pyramid::cabi::from_c(
            &cs, *static_cast<pyramid::domain_model::ObjectEvidenceRequirement*>(out_value));
        pyramid_ObjectEvidenceRequirement_c_free(&cs);
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
    if (std::strcmp(schema_id, "Capability") == 0) {
        pyramid_Capability_c cs;
        std::memset(&cs, 0, sizeof(cs));
        if (c->decode(c->codec_ctx, schema_id, msg, &cs)
                != PCL_OK) {
            pyramid_Capability_c_free(&cs);
            return -1;
        }
        pyramid::cabi::from_c(
            &cs, *static_cast<pyramid::domain_model::Capability*>(out_value));
        pyramid_Capability_c_free(&cs);
        return 1;
    }
    if (std::strcmp(schema_id, "ObjectMatch") == 0) {
        pyramid_ObjectMatch_c cs;
        std::memset(&cs, 0, sizeof(cs));
        if (c->decode(c->codec_ctx, schema_id, msg, &cs)
                != PCL_OK) {
            pyramid_ObjectMatch_c_free(&cs);
            return -1;
        }
        pyramid::cabi::from_c(
            &cs, *static_cast<pyramid::domain_model::ObjectMatch*>(out_value));
        pyramid_ObjectMatch_c_free(&cs);
        return 1;
    }
    if (std::strcmp(schema_id, "ObjectInterestRequirement") == 0) {
        pyramid_ObjectInterestRequirement_c cs;
        std::memset(&cs, 0, sizeof(cs));
        if (c->decode(c->codec_ctx, schema_id, msg, &cs)
                != PCL_OK) {
            pyramid_ObjectInterestRequirement_c_free(&cs);
            return -1;
        }
        pyramid::cabi::from_c(
            &cs, *static_cast<pyramid::domain_model::ObjectInterestRequirement*>(out_value));
        pyramid_ObjectInterestRequirement_c_free(&cs);
        return 1;
    }
    if (std::strcmp(schema_id, "ObjectDetailArray") == 0) {
        pyramid_slice_t slice;
        std::memset(&slice, 0, sizeof(slice));
        if (c->decode(c->codec_ctx, schema_id, msg, &slice)
                != PCL_OK) {
            return -1;
        }
        auto* native = static_cast<std::vector<pyramid::domain_model::ObjectDetail>*>(out_value);
        native->clear();
        native->reserve(slice.len);
        auto* values = static_cast<pyramid_ObjectDetail_c*>(const_cast<void*>(slice.ptr));
        for (uint32_t i = 0; i < slice.len; ++i) {
            pyramid::domain_model::ObjectDetail item;
            pyramid::cabi::from_c(&values[i], item);
            native->push_back(std::move(item));
            pyramid_ObjectDetail_c_free(&values[i]);
        }
        std::free(values);
        return 1;
    }
    if (std::strcmp(schema_id, "ObjectEvidenceRequirementArray") == 0) {
        pyramid_slice_t slice;
        std::memset(&slice, 0, sizeof(slice));
        if (c->decode(c->codec_ctx, schema_id, msg, &slice)
                != PCL_OK) {
            return -1;
        }
        auto* native = static_cast<std::vector<pyramid::domain_model::ObjectEvidenceRequirement>*>(out_value);
        native->clear();
        native->reserve(slice.len);
        auto* values = static_cast<pyramid_ObjectEvidenceRequirement_c*>(const_cast<void*>(slice.ptr));
        for (uint32_t i = 0; i < slice.len; ++i) {
            pyramid::domain_model::ObjectEvidenceRequirement item;
            pyramid::cabi::from_c(&values[i], item);
            native->push_back(std::move(item));
            pyramid_ObjectEvidenceRequirement_c_free(&values[i]);
        }
        std::free(values);
        return 1;
    }
    if (std::strcmp(schema_id, "CapabilityArray") == 0) {
        pyramid_slice_t slice;
        std::memset(&slice, 0, sizeof(slice));
        if (c->decode(c->codec_ctx, schema_id, msg, &slice)
                != PCL_OK) {
            return -1;
        }
        auto* native = static_cast<std::vector<pyramid::domain_model::Capability>*>(out_value);
        native->clear();
        native->reserve(slice.len);
        auto* values = static_cast<pyramid_Capability_c*>(const_cast<void*>(slice.ptr));
        for (uint32_t i = 0; i < slice.len; ++i) {
            pyramid::domain_model::Capability item;
            pyramid::cabi::from_c(&values[i], item);
            native->push_back(std::move(item));
            pyramid_Capability_c_free(&values[i]);
        }
        std::free(values);
        return 1;
    }
    if (std::strcmp(schema_id, "ObjectMatchArray") == 0) {
        pyramid_slice_t slice;
        std::memset(&slice, 0, sizeof(slice));
        if (c->decode(c->codec_ctx, schema_id, msg, &slice)
                != PCL_OK) {
            return -1;
        }
        auto* native = static_cast<std::vector<pyramid::domain_model::ObjectMatch>*>(out_value);
        native->clear();
        native->reserve(slice.len);
        auto* values = static_cast<pyramid_ObjectMatch_c*>(const_cast<void*>(slice.ptr));
        for (uint32_t i = 0; i < slice.len; ++i) {
            pyramid::domain_model::ObjectMatch item;
            pyramid::cabi::from_c(&values[i], item);
            native->push_back(std::move(item));
            pyramid_ObjectMatch_c_free(&values[i]);
        }
        std::free(values);
        return 1;
    }
    if (std::strcmp(schema_id, "ObjectInterestRequirementArray") == 0) {
        pyramid_slice_t slice;
        std::memset(&slice, 0, sizeof(slice));
        if (c->decode(c->codec_ctx, schema_id, msg, &slice)
                != PCL_OK) {
            return -1;
        }
        auto* native = static_cast<std::vector<pyramid::domain_model::ObjectInterestRequirement>*>(out_value);
        native->clear();
        native->reserve(slice.len);
        auto* values = static_cast<pyramid_ObjectInterestRequirement_c*>(const_cast<void*>(slice.ptr));
        for (uint32_t i = 0; i < slice.len; ++i) {
            pyramid::domain_model::ObjectInterestRequirement item;
            pyramid::cabi::from_c(&values[i], item);
            native->push_back(std::move(item));
            pyramid_ObjectInterestRequirement_c_free(&values[i]);
        }
        std::free(values);
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
            if (m.data && m.size != 0) {
                out->assign(static_cast<const char*>(m.data), m.size);
            } else {
                out->clear();
            }
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
    std::vector<const char*> result;
    pcl_codec_registry_t* reg = pcl_codec_registry_default();
    if (pcl_codec_registry_get(reg, kJsonContentType) != nullptr) {
        result.push_back(kJsonContentType);
    }
    if (pcl_codec_registry_get(reg, kFlatBuffersContentType) != nullptr) {
        result.push_back(kFlatBuffersContentType);
    }
    return result;
}

bool supportsContentType(const char* content_type)
{
    return content_type
        && pcl_codec_registry_get(pcl_codec_registry_default(), content_type)
            != nullptr;
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

bool decodeMatchingObjectsReadMatchResponse(const pcl_msg_t* msg,
                                            std::vector<ObjectMatch>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "ObjectMatchArray", out); if (r == 1) return true; }
    return false;
}

bool encodeMatchingObjectsReadMatchStreamFrame(const ObjectMatch& payload,
                                               const char*        content_type,
                                               std::string*       out)
{
    if (!out) {
        return false;
    }
    { int r = pyramid_try_registry_encode(content_type, "ObjectMatch", &payload, out); if (r == 1) return true; }
    return false;
}

bool decodeMatchingObjectsReadMatchStreamFrame(const pcl_msg_t* msg,
                                               ObjectMatch* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "ObjectMatch", out); if (r == 1) return true; }
    return false;
}

pcl_status_t sendMatchingObjectsReadMatchStreamFrame(pcl_stream_context_t* stream_context,
                                                     const ObjectMatch& payload,
                                                     const char*        content_type)
{
    std::string wire_payload;
    if (!encodeMatchingObjectsReadMatchStreamFrame(payload, content_type, &wire_payload)) {
        return PCL_ERR_INVALID;
    }
    pcl_msg_t msg{};
    msg.data = wire_payload.data();
    msg.size = static_cast<uint32_t>(wire_payload.size());
    msg.type_name = content_type;
    return pcl_stream_send(stream_context, &msg);
}

bool decodeObjectOfInterestCreateRequirementResponse(const pcl_msg_t* msg,
                                                     Identifier* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Identifier", out); if (r == 1) return true; }
    return false;
}

bool decodeObjectOfInterestReadRequirementResponse(const pcl_msg_t* msg,
                                                   std::vector<ObjectInterestRequirement>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "ObjectInterestRequirementArray", out); if (r == 1) return true; }
    return false;
}

bool encodeObjectOfInterestReadRequirementStreamFrame(const ObjectInterestRequirement& payload,
                                                      const char*        content_type,
                                                      std::string*       out)
{
    if (!out) {
        return false;
    }
    { int r = pyramid_try_registry_encode(content_type, "ObjectInterestRequirement", &payload, out); if (r == 1) return true; }
    return false;
}

bool decodeObjectOfInterestReadRequirementStreamFrame(const pcl_msg_t* msg,
                                                      ObjectInterestRequirement* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "ObjectInterestRequirement", out); if (r == 1) return true; }
    return false;
}

pcl_status_t sendObjectOfInterestReadRequirementStreamFrame(pcl_stream_context_t* stream_context,
                                                            const ObjectInterestRequirement& payload,
                                                            const char*        content_type)
{
    std::string wire_payload;
    if (!encodeObjectOfInterestReadRequirementStreamFrame(payload, content_type, &wire_payload)) {
        return PCL_ERR_INVALID;
    }
    pcl_msg_t msg{};
    msg.data = wire_payload.data();
    msg.size = static_cast<uint32_t>(wire_payload.size());
    msg.type_name = content_type;
    return pcl_stream_send(stream_context, &msg);
}

bool decodeObjectOfInterestUpdateRequirementResponse(const pcl_msg_t* msg,
                                                     Ack* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Ack", out); if (r == 1) return true; }
    return false;
}

bool decodeObjectOfInterestDeleteRequirementResponse(const pcl_msg_t* msg,
                                                     Ack* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "Ack", out); if (r == 1) return true; }
    return false;
}

bool decodeSpecificObjectDetailReadDetailResponse(const pcl_msg_t* msg,
                                                  std::vector<ObjectDetail>* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "ObjectDetailArray", out); if (r == 1) return true; }
    return false;
}

bool encodeSpecificObjectDetailReadDetailStreamFrame(const ObjectDetail& payload,
                                                     const char*        content_type,
                                                     std::string*       out)
{
    if (!out) {
        return false;
    }
    { int r = pyramid_try_registry_encode(content_type, "ObjectDetail", &payload, out); if (r == 1) return true; }
    return false;
}

bool decodeSpecificObjectDetailReadDetailStreamFrame(const pcl_msg_t* msg,
                                                     ObjectDetail* out)
{
    if (!msg || !msg->data || msg->size == 0 || !out) {
        return false;
    }
    { int r = pyramid_try_registry_decode(msg, "ObjectDetail", out); if (r == 1) return true; }
    return false;
}

pcl_status_t sendSpecificObjectDetailReadDetailStreamFrame(pcl_stream_context_t* stream_context,
                                                           const ObjectDetail& payload,
                                                           const char*        content_type)
{
    std::string wire_payload;
    if (!encodeSpecificObjectDetailReadDetailStreamFrame(payload, content_type, &wire_payload)) {
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

pcl_status_t invokeMatchingObjectsReadMatch(pcl_executor_t* executor,
                                            const Query&                 request,
                                            pcl_resp_cb_fn_t        callback,
                                            void*                   user_data,
                                            const pcl_endpoint_route_t* route,
                                            const char*       content_type)
{
    std::string payload;
    if (pyramid_try_registry_encode(content_type, "Query", &request, &payload) != 1) {
        return PCL_ERR_NOT_FOUND;
    }
    return invoke_async(executor, kSvcMatchingObjectsReadMatch, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeMatchingObjectsReadMatch(pcl_executor_t* executor,
                                            const Query&                 request,
                                            const char*       content_type,
                                            const pcl_endpoint_route_t* route)
{
    return invokeMatchingObjectsReadMatch(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeMatchingObjectsReadMatchStream(pcl_executor_t* executor,
                                                  const Query&                 request,
                                                  pcl_stream_msg_fn_t   callback,
                                                  void*                   user_data,
                                                  pcl_stream_context_t** out_context,
                                                  const pcl_endpoint_route_t* route,
                                                  const char*       content_type)
{
    std::string payload;
    if (pyramid_try_registry_encode(content_type, "Query", &request, &payload) != 1) {
        return PCL_ERR_NOT_FOUND;
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
    return pcl_executor_invoke_stream(executor, kSvcMatchingObjectsReadMatch,
                                      &msg, callback, user_data,
                                      out_context);
}

pcl_status_t invokeObjectOfInterestCreateRequirement(pcl_executor_t* executor,
                                                     const ObjectInterestRequirement& request,
                                                     pcl_resp_cb_fn_t        callback,
                                                     void*                   user_data,
                                                     const pcl_endpoint_route_t* route,
                                                     const char*       content_type)
{
    std::string payload;
    if (pyramid_try_registry_encode(content_type, "ObjectInterestRequirement", &request, &payload) != 1) {
        return PCL_ERR_NOT_FOUND;
    }
    return invoke_async(executor, kSvcObjectOfInterestCreateRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeObjectOfInterestCreateRequirement(pcl_executor_t* executor,
                                                     const ObjectInterestRequirement& request,
                                                     const char*       content_type,
                                                     const pcl_endpoint_route_t* route)
{
    return invokeObjectOfInterestCreateRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeObjectOfInterestReadRequirement(pcl_executor_t* executor,
                                                   const Query&                 request,
                                                   pcl_resp_cb_fn_t        callback,
                                                   void*                   user_data,
                                                   const pcl_endpoint_route_t* route,
                                                   const char*       content_type)
{
    std::string payload;
    if (pyramid_try_registry_encode(content_type, "Query", &request, &payload) != 1) {
        return PCL_ERR_NOT_FOUND;
    }
    return invoke_async(executor, kSvcObjectOfInterestReadRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeObjectOfInterestReadRequirement(pcl_executor_t* executor,
                                                   const Query&                 request,
                                                   const char*       content_type,
                                                   const pcl_endpoint_route_t* route)
{
    return invokeObjectOfInterestReadRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeObjectOfInterestReadRequirementStream(pcl_executor_t* executor,
                                                         const Query&                 request,
                                                         pcl_stream_msg_fn_t   callback,
                                                         void*                   user_data,
                                                         pcl_stream_context_t** out_context,
                                                         const pcl_endpoint_route_t* route,
                                                         const char*       content_type)
{
    std::string payload;
    if (pyramid_try_registry_encode(content_type, "Query", &request, &payload) != 1) {
        return PCL_ERR_NOT_FOUND;
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
    return pcl_executor_invoke_stream(executor, kSvcObjectOfInterestReadRequirement,
                                      &msg, callback, user_data,
                                      out_context);
}

pcl_status_t invokeObjectOfInterestUpdateRequirement(pcl_executor_t* executor,
                                                     const ObjectInterestRequirement& request,
                                                     pcl_resp_cb_fn_t        callback,
                                                     void*                   user_data,
                                                     const pcl_endpoint_route_t* route,
                                                     const char*       content_type)
{
    std::string payload;
    if (pyramid_try_registry_encode(content_type, "ObjectInterestRequirement", &request, &payload) != 1) {
        return PCL_ERR_NOT_FOUND;
    }
    return invoke_async(executor, kSvcObjectOfInterestUpdateRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeObjectOfInterestUpdateRequirement(pcl_executor_t* executor,
                                                     const ObjectInterestRequirement& request,
                                                     const char*       content_type,
                                                     const pcl_endpoint_route_t* route)
{
    return invokeObjectOfInterestUpdateRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeObjectOfInterestDeleteRequirement(pcl_executor_t* executor,
                                                     const Identifier&            request,
                                                     pcl_resp_cb_fn_t        callback,
                                                     void*                   user_data,
                                                     const pcl_endpoint_route_t* route,
                                                     const char*       content_type)
{
    std::string payload;
    if (pyramid_try_registry_encode(content_type, "Identifier", &request, &payload) != 1) {
        return PCL_ERR_NOT_FOUND;
    }
    return invoke_async(executor, kSvcObjectOfInterestDeleteRequirement, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeObjectOfInterestDeleteRequirement(pcl_executor_t* executor,
                                                     const Identifier&            request,
                                                     const char*       content_type,
                                                     const pcl_endpoint_route_t* route)
{
    return invokeObjectOfInterestDeleteRequirement(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeSpecificObjectDetailReadDetail(pcl_executor_t* executor,
                                                  const Query&                 request,
                                                  pcl_resp_cb_fn_t        callback,
                                                  void*                   user_data,
                                                  const pcl_endpoint_route_t* route,
                                                  const char*       content_type)
{
    std::string payload;
    if (pyramid_try_registry_encode(content_type, "Query", &request, &payload) != 1) {
        return PCL_ERR_NOT_FOUND;
    }
    return invoke_async(executor, kSvcSpecificObjectDetailReadDetail, payload, callback, user_data, route, content_type);
}

pcl_status_t invokeSpecificObjectDetailReadDetail(pcl_executor_t* executor,
                                                  const Query&                 request,
                                                  const char*       content_type,
                                                  const pcl_endpoint_route_t* route)
{
    return invokeSpecificObjectDetailReadDetail(executor, request, ignore_async_response,
                         nullptr, route, content_type);
}

pcl_status_t invokeSpecificObjectDetailReadDetailStream(pcl_executor_t* executor,
                                                        const Query&                 request,
                                                        pcl_stream_msg_fn_t   callback,
                                                        void*                   user_data,
                                                        pcl_stream_context_t** out_context,
                                                        const pcl_endpoint_route_t* route,
                                                        const char*       content_type)
{
    std::string payload;
    if (pyramid_try_registry_encode(content_type, "Query", &request, &payload) != 1) {
        return PCL_ERR_NOT_FOUND;
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
    return pcl_executor_invoke_stream(executor, kSvcSpecificObjectDetailReadDetail,
                                      &msg, callback, user_data,
                                      out_context);
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
    { int r = pyramid_try_registry_encode(content_type, "ObjectMatchArray", &payload, out); if (r == 1) return true; }
    return false;
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
    { int r = pyramid_try_registry_decode(msg, "ObjectMatchArray", out); if (r == 1) return true; }
    return false;
}

bool encodeEvidenceRequirements(const ObjectEvidenceRequirement& payload,
                                const char*        content_type,
                                std::string*       out)
{
    if (!out) {
        return false;
    }
    { int r = pyramid_try_registry_encode(content_type, "ObjectEvidenceRequirement", &payload, out); if (r == 1) return true; }
    return false;
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
    { int r = pyramid_try_registry_decode(msg, "ObjectEvidenceRequirement", out); if (r == 1) return true; }
    return false;
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
    if (!response_buf || !response_size) {
        return;
    }
    *response_buf = nullptr;
    *response_size = 0;
    pcl_msg_t req_msg{};
    req_msg.data = request_buf;
    req_msg.size = static_cast<uint32_t>(request_size);
    req_msg.type_name = content_type;
    std::string rsp_payload;

    try {
    switch (channel) {
    case ServiceChannel::MatchingObjectsReadMatch: {
        Query req;
        if (pyramid_try_registry_decode(&req_msg, "Query", &req) != 1) {
            break;
        }
        auto rsp = handler.handleMatchingObjectsReadMatch(req);
        if (pyramid_try_registry_encode(content_type, "ObjectMatchArray", &rsp, &rsp_payload) != 1) {
            break;
        }
        break;
    }
    case ServiceChannel::ObjectOfInterestCreateRequirement: {
        ObjectInterestRequirement req;
        if (pyramid_try_registry_decode(&req_msg, "ObjectInterestRequirement", &req) != 1) {
            break;
        }
        auto rsp = handler.handleObjectOfInterestCreateRequirement(req);
        if (pyramid_try_registry_encode(content_type, "Identifier", &rsp, &rsp_payload) != 1) {
            break;
        }
        break;
    }
    case ServiceChannel::ObjectOfInterestReadRequirement: {
        Query req;
        if (pyramid_try_registry_decode(&req_msg, "Query", &req) != 1) {
            break;
        }
        auto rsp = handler.handleObjectOfInterestReadRequirement(req);
        if (pyramid_try_registry_encode(content_type, "ObjectInterestRequirementArray", &rsp, &rsp_payload) != 1) {
            break;
        }
        break;
    }
    case ServiceChannel::ObjectOfInterestUpdateRequirement: {
        ObjectInterestRequirement req;
        if (pyramid_try_registry_decode(&req_msg, "ObjectInterestRequirement", &req) != 1) {
            break;
        }
        auto rsp = handler.handleObjectOfInterestUpdateRequirement(req);
        if (pyramid_try_registry_encode(content_type, "Ack", &rsp, &rsp_payload) != 1) {
            break;
        }
        break;
    }
    case ServiceChannel::ObjectOfInterestDeleteRequirement: {
        Identifier req;
        if (pyramid_try_registry_decode(&req_msg, "Identifier", &req) != 1) {
            break;
        }
        auto rsp = handler.handleObjectOfInterestDeleteRequirement(req);
        if (pyramid_try_registry_encode(content_type, "Ack", &rsp, &rsp_payload) != 1) {
            break;
        }
        break;
    }
    case ServiceChannel::SpecificObjectDetailReadDetail: {
        Query req;
        if (pyramid_try_registry_decode(&req_msg, "Query", &req) != 1) {
            break;
        }
        auto rsp = handler.handleSpecificObjectDetailReadDetail(req);
        if (pyramid_try_registry_encode(content_type, "ObjectDetailArray", &rsp, &rsp_payload) != 1) {
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
    pcl_msg_t req_msg{};
    req_msg.data = request_buf;
    req_msg.size = static_cast<uint32_t>(request_size);
    req_msg.type_name = content_type;

    try {
    switch (channel) {
    case ServiceChannel::MatchingObjectsReadMatch: {
        Query req;
        if (pyramid_try_registry_decode(&req_msg, "Query", &req) != 1) {
            return PCL_ERR_NOT_FOUND;
        }
        return handler.streamMatchingObjectsReadMatch(req, stream_context, content_type);
    }
    case ServiceChannel::ObjectOfInterestReadRequirement: {
        Query req;
        if (pyramid_try_registry_decode(&req_msg, "Query", &req) != 1) {
            return PCL_ERR_NOT_FOUND;
        }
        return handler.streamObjectOfInterestReadRequirement(req, stream_context, content_type);
    }
    case ServiceChannel::SpecificObjectDetailReadDetail: {
        Query req;
        if (pyramid_try_registry_decode(&req_msg, "Query", &req) != 1) {
            return PCL_ERR_NOT_FOUND;
        }
        return handler.streamSpecificObjectDetailReadDetail(req, stream_context, content_type);
    }
    }
    } catch (...) {
        return PCL_ERR_INVALID;
    }
    return PCL_ERR_INVALID;
}

} // namespace pyramid::components::tactical_objects::services::provided
