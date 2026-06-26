// Auto-generated PCL codec plugin
// Backend: flatbuffers | Content-Type: application/flatbuffers

#include "pyramid_data_model_types.hpp"
#include "flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.hpp"
#include "pyramid_data_model_common_cabi_marshal.hpp"
#include "pyramid_data_model_tactical_cabi_marshal.hpp"

extern "C" {
#include <pcl/pcl_codec.h>
#include "pyramid_datamodel_cabi.h"
}

#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#if defined(_WIN32)
#  define PCL_CODEC_PLUGIN_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#  define PCL_CODEC_PLUGIN_EXPORT __attribute__((visibility("default")))
#else
#  define PCL_CODEC_PLUGIN_EXPORT
#endif

namespace {

namespace data_model = pyramid::domain_model;
namespace flatbuffers_codec = pyramid::services::tactical_objects::flatbuffers_codec;

pcl_status_t assign_payload(const std::string& payload,
                            const char* content_type,
                            pcl_msg_t* out_msg)
{
    if (!out_msg) {
        return PCL_ERR_INVALID;
    }
    if (payload.size() > std::numeric_limits<uint32_t>::max()) {
        return PCL_ERR_INVALID;
    }
    void* copy = nullptr;
    if (!payload.empty()) {
        copy = std::malloc(payload.size());
        if (!copy) {
            return PCL_ERR_NOMEM;
        }
        std::memcpy(copy, payload.data(), payload.size());
    }
    out_msg->data = copy;
    out_msg->size = static_cast<uint32_t>(payload.size());
    out_msg->type_name = content_type;
    return PCL_OK;
}

} // namespace

extern "C" {

static pcl_status_t plugin_encode(void*       codec_ctx,
                                  const char* schema_id,
                                  const void* value,
                                  pcl_msg_t*  out_msg)
{
    (void)codec_ctx;
    if (!schema_id || !value || !out_msg) {
        return PCL_ERR_INVALID;
    }
    try {
        if (std::strcmp(schema_id, "Query") == 0) {
            const auto* cs = static_cast<const pyramid_Query_c*>(value);
            data_model::Query native;
            pyramid::cabi::from_c(cs, native);
            return assign_payload(flatbuffers_codec::toBinary(native),
                                  "application/flatbuffers", out_msg);
        }
        if (std::strcmp(schema_id, "ObjectDetail") == 0) {
            const auto* cs = static_cast<const pyramid_ObjectDetail_c*>(value);
            data_model::ObjectDetail native;
            pyramid::cabi::from_c(cs, native);
            return assign_payload(flatbuffers_codec::toBinary(native),
                                  "application/flatbuffers", out_msg);
        }
        if (std::strcmp(schema_id, "ObjectEvidenceRequirement") == 0) {
            const auto* cs = static_cast<const pyramid_ObjectEvidenceRequirement_c*>(value);
            data_model::ObjectEvidenceRequirement native;
            pyramid::cabi::from_c(cs, native);
            return assign_payload(flatbuffers_codec::toBinary(native),
                                  "application/flatbuffers", out_msg);
        }
        if (std::strcmp(schema_id, "Identifier") == 0) {
            const auto* cs = static_cast<const pyramid_str_t*>(value);
            const char* data = cs && cs->ptr ? cs->ptr : "";
            data_model::Identifier native(data, cs ? cs->len : 0u);
            return assign_payload(flatbuffers_codec::toBinary(native),
                                  "application/flatbuffers", out_msg);
        }
        if (std::strcmp(schema_id, "Ack") == 0) {
            const auto* cs = static_cast<const pyramid_Ack_c*>(value);
            data_model::Ack native;
            pyramid::cabi::from_c(cs, native);
            return assign_payload(flatbuffers_codec::toBinary(native),
                                  "application/flatbuffers", out_msg);
        }
        if (std::strcmp(schema_id, "Capability") == 0) {
            const auto* cs = static_cast<const pyramid_Capability_c*>(value);
            data_model::Capability native;
            pyramid::cabi::from_c(cs, native);
            return assign_payload(flatbuffers_codec::toBinary(native),
                                  "application/flatbuffers", out_msg);
        }
        if (std::strcmp(schema_id, "ObjectMatch") == 0) {
            const auto* cs = static_cast<const pyramid_ObjectMatch_c*>(value);
            data_model::ObjectMatch native;
            pyramid::cabi::from_c(cs, native);
            return assign_payload(flatbuffers_codec::toBinary(native),
                                  "application/flatbuffers", out_msg);
        }
        if (std::strcmp(schema_id, "ObjectInterestRequirement") == 0) {
            const auto* cs = static_cast<const pyramid_ObjectInterestRequirement_c*>(value);
            data_model::ObjectInterestRequirement native;
            pyramid::cabi::from_c(cs, native);
            return assign_payload(flatbuffers_codec::toBinary(native),
                                  "application/flatbuffers", out_msg);
        }
        if (std::strcmp(schema_id, "ObjectDetailArray") == 0) {
            const auto* slice = static_cast<const pyramid_slice_t*>(value);
            if (!slice || (!slice->ptr && slice->len != 0u)) {
                return PCL_ERR_INVALID;
            }
            const auto* values = static_cast<const pyramid_ObjectDetail_c*>(slice->ptr);
            std::vector<data_model::ObjectDetail> native;
            native.reserve(slice->len);
            for (uint32_t i = 0; i < slice->len; ++i) {
                data_model::ObjectDetail item;
                pyramid::cabi::from_c(&values[i], item);
                native.push_back(std::move(item));
            }
            return assign_payload(flatbuffers_codec::toBinary(native),
                                  "application/flatbuffers", out_msg);
        }
        if (std::strcmp(schema_id, "ObjectEvidenceRequirementArray") == 0) {
            const auto* slice = static_cast<const pyramid_slice_t*>(value);
            if (!slice || (!slice->ptr && slice->len != 0u)) {
                return PCL_ERR_INVALID;
            }
            const auto* values = static_cast<const pyramid_ObjectEvidenceRequirement_c*>(slice->ptr);
            std::vector<data_model::ObjectEvidenceRequirement> native;
            native.reserve(slice->len);
            for (uint32_t i = 0; i < slice->len; ++i) {
                data_model::ObjectEvidenceRequirement item;
                pyramid::cabi::from_c(&values[i], item);
                native.push_back(std::move(item));
            }
            return assign_payload(flatbuffers_codec::toBinary(native),
                                  "application/flatbuffers", out_msg);
        }
        if (std::strcmp(schema_id, "CapabilityArray") == 0) {
            const auto* slice = static_cast<const pyramid_slice_t*>(value);
            if (!slice || (!slice->ptr && slice->len != 0u)) {
                return PCL_ERR_INVALID;
            }
            const auto* values = static_cast<const pyramid_Capability_c*>(slice->ptr);
            std::vector<data_model::Capability> native;
            native.reserve(slice->len);
            for (uint32_t i = 0; i < slice->len; ++i) {
                data_model::Capability item;
                pyramid::cabi::from_c(&values[i], item);
                native.push_back(std::move(item));
            }
            return assign_payload(flatbuffers_codec::toBinary(native),
                                  "application/flatbuffers", out_msg);
        }
        if (std::strcmp(schema_id, "ObjectMatchArray") == 0) {
            const auto* slice = static_cast<const pyramid_slice_t*>(value);
            if (!slice || (!slice->ptr && slice->len != 0u)) {
                return PCL_ERR_INVALID;
            }
            const auto* values = static_cast<const pyramid_ObjectMatch_c*>(slice->ptr);
            std::vector<data_model::ObjectMatch> native;
            native.reserve(slice->len);
            for (uint32_t i = 0; i < slice->len; ++i) {
                data_model::ObjectMatch item;
                pyramid::cabi::from_c(&values[i], item);
                native.push_back(std::move(item));
            }
            return assign_payload(flatbuffers_codec::toBinary(native),
                                  "application/flatbuffers", out_msg);
        }
        if (std::strcmp(schema_id, "ObjectInterestRequirementArray") == 0) {
            const auto* slice = static_cast<const pyramid_slice_t*>(value);
            if (!slice || (!slice->ptr && slice->len != 0u)) {
                return PCL_ERR_INVALID;
            }
            const auto* values = static_cast<const pyramid_ObjectInterestRequirement_c*>(slice->ptr);
            std::vector<data_model::ObjectInterestRequirement> native;
            native.reserve(slice->len);
            for (uint32_t i = 0; i < slice->len; ++i) {
                data_model::ObjectInterestRequirement item;
                pyramid::cabi::from_c(&values[i], item);
                native.push_back(std::move(item));
            }
            return assign_payload(flatbuffers_codec::toBinary(native),
                                  "application/flatbuffers", out_msg);
        }
    } catch (...) {
        return PCL_ERR_CALLBACK;
    }
    return PCL_ERR_NOT_FOUND;
}

static pcl_status_t plugin_decode(void*            codec_ctx,
                                  const char*      schema_id,
                                  const pcl_msg_t* msg,
                                  void*            out_value)
{
    (void)codec_ctx;
    if (!schema_id || !msg || (!msg->data && msg->size != 0) || !out_value) {
        return PCL_ERR_INVALID;
    }
    try {
        const std::string payload = msg->data
            ? std::string(static_cast<const char*>(msg->data), msg->size)
            : std::string();
        if (std::strcmp(schema_id, "Query") == 0) {
            data_model::Query native;
            native = flatbuffers_codec::fromBinaryQuery(
                msg->data, msg->size);
            auto* cs = static_cast<pyramid_Query_c*>(out_value);
            pyramid::cabi::to_c(native, cs);
            return PCL_OK;
        }
        if (std::strcmp(schema_id, "ObjectDetail") == 0) {
            data_model::ObjectDetail native;
            native = flatbuffers_codec::fromBinaryObjectDetail(
                msg->data, msg->size);
            auto* cs = static_cast<pyramid_ObjectDetail_c*>(out_value);
            pyramid::cabi::to_c(native, cs);
            return PCL_OK;
        }
        if (std::strcmp(schema_id, "ObjectEvidenceRequirement") == 0) {
            data_model::ObjectEvidenceRequirement native;
            native = flatbuffers_codec::fromBinaryObjectEvidenceRequirement(
                msg->data, msg->size);
            auto* cs = static_cast<pyramid_ObjectEvidenceRequirement_c*>(out_value);
            pyramid::cabi::to_c(native, cs);
            return PCL_OK;
        }
        if (std::strcmp(schema_id, "Identifier") == 0) {
            data_model::Identifier native;
            native = flatbuffers_codec::fromBinaryIdentifier(
                msg->data, msg->size);
            auto* cs = static_cast<pyramid_str_t*>(out_value);
            cs->ptr = nullptr;
            cs->len = 0u;
            if (native.size() > std::numeric_limits<uint32_t>::max()) {
                return PCL_ERR_INVALID;
            }
            if (!native.empty()) {
                void* copy = std::malloc(native.size());
                if (!copy) {
                    return PCL_ERR_NOMEM;
                }
                std::memcpy(copy, native.data(), native.size());
                cs->ptr = static_cast<const char*>(copy);
                cs->len = static_cast<uint32_t>(native.size());
            }
            return PCL_OK;
        }
        if (std::strcmp(schema_id, "Ack") == 0) {
            data_model::Ack native;
            native = flatbuffers_codec::fromBinaryAck(
                msg->data, msg->size);
            auto* cs = static_cast<pyramid_Ack_c*>(out_value);
            pyramid::cabi::to_c(native, cs);
            return PCL_OK;
        }
        if (std::strcmp(schema_id, "Capability") == 0) {
            data_model::Capability native;
            native = flatbuffers_codec::fromBinaryCapability(
                msg->data, msg->size);
            auto* cs = static_cast<pyramid_Capability_c*>(out_value);
            pyramid::cabi::to_c(native, cs);
            return PCL_OK;
        }
        if (std::strcmp(schema_id, "ObjectMatch") == 0) {
            data_model::ObjectMatch native;
            native = flatbuffers_codec::fromBinaryObjectMatch(
                msg->data, msg->size);
            auto* cs = static_cast<pyramid_ObjectMatch_c*>(out_value);
            pyramid::cabi::to_c(native, cs);
            return PCL_OK;
        }
        if (std::strcmp(schema_id, "ObjectInterestRequirement") == 0) {
            data_model::ObjectInterestRequirement native;
            native = flatbuffers_codec::fromBinaryObjectInterestRequirement(
                msg->data, msg->size);
            auto* cs = static_cast<pyramid_ObjectInterestRequirement_c*>(out_value);
            pyramid::cabi::to_c(native, cs);
            return PCL_OK;
        }
        if (std::strcmp(schema_id, "ObjectDetailArray") == 0) {
            std::vector<data_model::ObjectDetail> native;
            native = flatbuffers_codec::fromBinaryObjectDetailArray(
                msg->data, msg->size);
            if (native.size() > std::numeric_limits<uint32_t>::max()) {
                return PCL_ERR_INVALID;
            }
            auto* slice = static_cast<pyramid_slice_t*>(out_value);
            slice->ptr = nullptr;
            slice->len = 0u;
            if (!native.empty()) {
                auto* values = static_cast<pyramid_ObjectDetail_c*>(
                    std::calloc(native.size(), sizeof(pyramid_ObjectDetail_c)));
                if (!values) {
                    return PCL_ERR_NOMEM;
                }
                for (std::size_t i = 0; i < native.size(); ++i) {
                    pyramid::cabi::to_c(native[i], &values[i]);
                }
                slice->ptr = values;
                slice->len = static_cast<uint32_t>(native.size());
            }
            return PCL_OK;
        }
        if (std::strcmp(schema_id, "ObjectEvidenceRequirementArray") == 0) {
            std::vector<data_model::ObjectEvidenceRequirement> native;
            native = flatbuffers_codec::fromBinaryObjectEvidenceRequirementArray(
                msg->data, msg->size);
            if (native.size() > std::numeric_limits<uint32_t>::max()) {
                return PCL_ERR_INVALID;
            }
            auto* slice = static_cast<pyramid_slice_t*>(out_value);
            slice->ptr = nullptr;
            slice->len = 0u;
            if (!native.empty()) {
                auto* values = static_cast<pyramid_ObjectEvidenceRequirement_c*>(
                    std::calloc(native.size(), sizeof(pyramid_ObjectEvidenceRequirement_c)));
                if (!values) {
                    return PCL_ERR_NOMEM;
                }
                for (std::size_t i = 0; i < native.size(); ++i) {
                    pyramid::cabi::to_c(native[i], &values[i]);
                }
                slice->ptr = values;
                slice->len = static_cast<uint32_t>(native.size());
            }
            return PCL_OK;
        }
        if (std::strcmp(schema_id, "CapabilityArray") == 0) {
            std::vector<data_model::Capability> native;
            native = flatbuffers_codec::fromBinaryCapabilityArray(
                msg->data, msg->size);
            if (native.size() > std::numeric_limits<uint32_t>::max()) {
                return PCL_ERR_INVALID;
            }
            auto* slice = static_cast<pyramid_slice_t*>(out_value);
            slice->ptr = nullptr;
            slice->len = 0u;
            if (!native.empty()) {
                auto* values = static_cast<pyramid_Capability_c*>(
                    std::calloc(native.size(), sizeof(pyramid_Capability_c)));
                if (!values) {
                    return PCL_ERR_NOMEM;
                }
                for (std::size_t i = 0; i < native.size(); ++i) {
                    pyramid::cabi::to_c(native[i], &values[i]);
                }
                slice->ptr = values;
                slice->len = static_cast<uint32_t>(native.size());
            }
            return PCL_OK;
        }
        if (std::strcmp(schema_id, "ObjectMatchArray") == 0) {
            std::vector<data_model::ObjectMatch> native;
            native = flatbuffers_codec::fromBinaryObjectMatchArray(
                msg->data, msg->size);
            if (native.size() > std::numeric_limits<uint32_t>::max()) {
                return PCL_ERR_INVALID;
            }
            auto* slice = static_cast<pyramid_slice_t*>(out_value);
            slice->ptr = nullptr;
            slice->len = 0u;
            if (!native.empty()) {
                auto* values = static_cast<pyramid_ObjectMatch_c*>(
                    std::calloc(native.size(), sizeof(pyramid_ObjectMatch_c)));
                if (!values) {
                    return PCL_ERR_NOMEM;
                }
                for (std::size_t i = 0; i < native.size(); ++i) {
                    pyramid::cabi::to_c(native[i], &values[i]);
                }
                slice->ptr = values;
                slice->len = static_cast<uint32_t>(native.size());
            }
            return PCL_OK;
        }
        if (std::strcmp(schema_id, "ObjectInterestRequirementArray") == 0) {
            std::vector<data_model::ObjectInterestRequirement> native;
            native = flatbuffers_codec::fromBinaryObjectInterestRequirementArray(
                msg->data, msg->size);
            if (native.size() > std::numeric_limits<uint32_t>::max()) {
                return PCL_ERR_INVALID;
            }
            auto* slice = static_cast<pyramid_slice_t*>(out_value);
            slice->ptr = nullptr;
            slice->len = 0u;
            if (!native.empty()) {
                auto* values = static_cast<pyramid_ObjectInterestRequirement_c*>(
                    std::calloc(native.size(), sizeof(pyramid_ObjectInterestRequirement_c)));
                if (!values) {
                    return PCL_ERR_NOMEM;
                }
                for (std::size_t i = 0; i < native.size(); ++i) {
                    pyramid::cabi::to_c(native[i], &values[i]);
                }
                slice->ptr = values;
                slice->len = static_cast<uint32_t>(native.size());
            }
            return PCL_OK;
        }
    } catch (...) {
        return PCL_ERR_CALLBACK;
    }
    return PCL_ERR_NOT_FOUND;
}

static void plugin_free_msg(void* codec_ctx, pcl_msg_t* msg)
{
    (void)codec_ctx;
    if (!msg) {
        return;
    }
    std::free(const_cast<void*>(msg->data));
    msg->data = nullptr;
    msg->size = 0u;
    msg->type_name = nullptr;
}

static pcl_codec_t k_codec = {
    PCL_CODEC_ABI_VERSION,
    "application/flatbuffers",
    plugin_encode,
    plugin_decode,
    plugin_free_msg,
    nullptr
};

// Opaque, plugin-specific configuration threaded through the loader.
// Stored here and exposed via codec_ctx so encode/decode can honor it.
static std::string k_config_json;

PCL_CODEC_PLUGIN_EXPORT const pcl_codec_t* pcl_codec_plugin_entry(
    const char* config_json)
{
    k_config_json = config_json ? config_json : "";
    k_codec.codec_ctx = k_config_json.empty() ? nullptr : &k_config_json;
    return &k_codec;
}

} // extern "C"
