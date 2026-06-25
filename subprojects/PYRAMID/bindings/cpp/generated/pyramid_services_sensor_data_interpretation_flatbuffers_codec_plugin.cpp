// Auto-generated PCL codec plugin
// Backend: flatbuffers | Content-Type: application/flatbuffers

#include "pyramid_data_model_types.hpp"
#include "flatbuffers/cpp/pyramid_services_sensor_data_interpretation_flatbuffers_codec.hpp"
#include "pyramid_data_model_common_cabi_marshal.hpp"
#include "pyramid_data_model_sensors_cabi_marshal.hpp"

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
namespace flatbuffers_codec = pyramid::services::sensor_data_interpretation::flatbuffers_codec;

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
        if (std::strcmp(schema_id, "ObjectEvidenceProvisionRequirement") == 0) {
            const auto* cs = static_cast<const pyramid_ObjectEvidenceProvisionRequirement_c*>(value);
            data_model::ObjectEvidenceProvisionRequirement native;
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
        if (std::strcmp(schema_id, "Query") == 0) {
            const auto* cs = static_cast<const pyramid_Query_c*>(value);
            data_model::Query native;
            pyramid::cabi::from_c(cs, native);
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
        if (std::strcmp(schema_id, "ObjectAquisitionRequirement") == 0) {
            const auto* cs = static_cast<const pyramid_ObjectAquisitionRequirement_c*>(value);
            data_model::ObjectAquisitionRequirement native;
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
        if (std::strcmp(schema_id, "InterpretationRequirement") == 0) {
            const auto* cs = static_cast<const pyramid_InterpretationRequirement_c*>(value);
            data_model::InterpretationRequirement native;
            pyramid::cabi::from_c(cs, native);
            return assign_payload(flatbuffers_codec::toBinary(native),
                                  "application/flatbuffers", out_msg);
        }
        if (std::strcmp(schema_id, "ObjectEvidenceProvisionRequirementArray") == 0) {
            const auto* slice = static_cast<const pyramid_slice_t*>(value);
            if (!slice || (!slice->ptr && slice->len != 0u)) {
                return PCL_ERR_INVALID;
            }
            const auto* values = static_cast<const pyramid_ObjectEvidenceProvisionRequirement_c*>(slice->ptr);
            std::vector<data_model::ObjectEvidenceProvisionRequirement> native;
            native.reserve(slice->len);
            for (uint32_t i = 0; i < slice->len; ++i) {
                data_model::ObjectEvidenceProvisionRequirement item;
                pyramid::cabi::from_c(&values[i], item);
                native.push_back(std::move(item));
            }
            return assign_payload(flatbuffers_codec::toBinary(native),
                                  "application/flatbuffers", out_msg);
        }
        if (std::strcmp(schema_id, "ObjectAquisitionRequirementArray") == 0) {
            const auto* slice = static_cast<const pyramid_slice_t*>(value);
            if (!slice || (!slice->ptr && slice->len != 0u)) {
                return PCL_ERR_INVALID;
            }
            const auto* values = static_cast<const pyramid_ObjectAquisitionRequirement_c*>(slice->ptr);
            std::vector<data_model::ObjectAquisitionRequirement> native;
            native.reserve(slice->len);
            for (uint32_t i = 0; i < slice->len; ++i) {
                data_model::ObjectAquisitionRequirement item;
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
        if (std::strcmp(schema_id, "InterpretationRequirementArray") == 0) {
            const auto* slice = static_cast<const pyramid_slice_t*>(value);
            if (!slice || (!slice->ptr && slice->len != 0u)) {
                return PCL_ERR_INVALID;
            }
            const auto* values = static_cast<const pyramid_InterpretationRequirement_c*>(slice->ptr);
            std::vector<data_model::InterpretationRequirement> native;
            native.reserve(slice->len);
            for (uint32_t i = 0; i < slice->len; ++i) {
                data_model::InterpretationRequirement item;
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
        if (std::strcmp(schema_id, "ObjectEvidenceProvisionRequirement") == 0) {
            data_model::ObjectEvidenceProvisionRequirement native;
            native = flatbuffers_codec::fromBinaryObjectEvidenceProvisionRequirement(
                msg->data, msg->size);
            auto* cs = static_cast<pyramid_ObjectEvidenceProvisionRequirement_c*>(out_value);
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
        if (std::strcmp(schema_id, "Query") == 0) {
            data_model::Query native;
            native = flatbuffers_codec::fromBinaryQuery(
                msg->data, msg->size);
            auto* cs = static_cast<pyramid_Query_c*>(out_value);
            pyramid::cabi::to_c(native, cs);
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
        if (std::strcmp(schema_id, "ObjectAquisitionRequirement") == 0) {
            data_model::ObjectAquisitionRequirement native;
            native = flatbuffers_codec::fromBinaryObjectAquisitionRequirement(
                msg->data, msg->size);
            auto* cs = static_cast<pyramid_ObjectAquisitionRequirement_c*>(out_value);
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
        if (std::strcmp(schema_id, "InterpretationRequirement") == 0) {
            data_model::InterpretationRequirement native;
            native = flatbuffers_codec::fromBinaryInterpretationRequirement(
                msg->data, msg->size);
            auto* cs = static_cast<pyramid_InterpretationRequirement_c*>(out_value);
            pyramid::cabi::to_c(native, cs);
            return PCL_OK;
        }
        if (std::strcmp(schema_id, "ObjectEvidenceProvisionRequirementArray") == 0) {
            std::vector<data_model::ObjectEvidenceProvisionRequirement> native;
            native = flatbuffers_codec::fromBinaryObjectEvidenceProvisionRequirementArray(
                msg->data, msg->size);
            if (native.size() > std::numeric_limits<uint32_t>::max()) {
                return PCL_ERR_INVALID;
            }
            auto* slice = static_cast<pyramid_slice_t*>(out_value);
            slice->ptr = nullptr;
            slice->len = 0u;
            if (!native.empty()) {
                auto* values = static_cast<pyramid_ObjectEvidenceProvisionRequirement_c*>(
                    std::calloc(native.size(), sizeof(pyramid_ObjectEvidenceProvisionRequirement_c)));
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
        if (std::strcmp(schema_id, "ObjectAquisitionRequirementArray") == 0) {
            std::vector<data_model::ObjectAquisitionRequirement> native;
            native = flatbuffers_codec::fromBinaryObjectAquisitionRequirementArray(
                msg->data, msg->size);
            if (native.size() > std::numeric_limits<uint32_t>::max()) {
                return PCL_ERR_INVALID;
            }
            auto* slice = static_cast<pyramid_slice_t*>(out_value);
            slice->ptr = nullptr;
            slice->len = 0u;
            if (!native.empty()) {
                auto* values = static_cast<pyramid_ObjectAquisitionRequirement_c*>(
                    std::calloc(native.size(), sizeof(pyramid_ObjectAquisitionRequirement_c)));
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
        if (std::strcmp(schema_id, "InterpretationRequirementArray") == 0) {
            std::vector<data_model::InterpretationRequirement> native;
            native = flatbuffers_codec::fromBinaryInterpretationRequirementArray(
                msg->data, msg->size);
            if (native.size() > std::numeric_limits<uint32_t>::max()) {
                return PCL_ERR_INVALID;
            }
            auto* slice = static_cast<pyramid_slice_t*>(out_value);
            slice->ptr = nullptr;
            slice->len = 0u;
            if (!native.empty()) {
                auto* values = static_cast<pyramid_InterpretationRequirement_c*>(
                    std::calloc(native.size(), sizeof(pyramid_InterpretationRequirement_c)));
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

static const pcl_codec_t k_codec = {
    PCL_CODEC_ABI_VERSION,
    "application/flatbuffers",
    plugin_encode,
    plugin_decode,
    plugin_free_msg,
    nullptr
};

PCL_CODEC_PLUGIN_EXPORT const pcl_codec_t* pcl_codec_plugin_entry(void)
{
    return &k_codec;
}

} // extern "C"
