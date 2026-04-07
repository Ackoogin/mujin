// Auto-generated codec dispatch — do not edit
// Namespace: pyramid::data_model::tactical::codec_dispatch
//
// Port-level codec routing: reads pcl_msg_t.type_name to select
// the correct codec for serialisation/deserialisation.
//
// Supported codecs (compile-time feature flags):
//   "application/json"        — always available
//   "application/flatbuffers"  — requires CODEC_FLATBUFFERS
//   "application/protobuf"     — requires CODEC_PROTOBUF
#pragma once

#include <pcl/pcl_types.h>
#include <string>
#include <stdexcept>
#include <cstring>

// JSON codec (always available)
#include "pyramid_data_model_tactical_json_codec.hpp"

#if defined(CODEC_FLATBUFFERS)
#include "pyramid_data_model_tactical_flatbuffers_codec.hpp"
#endif

#if defined(CODEC_PROTOBUF)
#include "pyramid_data_model_tactical_protobuf_codec.hpp"
#endif

namespace pyramid::data_model::tactical::codec_dispatch {

// ---------------------------------------------------------------------------
// Content type constants
// ---------------------------------------------------------------------------

constexpr const char* kJson        = "application/json";
constexpr const char* kFlatBuffers  = "application/flatbuffers";
constexpr const char* kProtobuf     = "application/protobuf";

// ---------------------------------------------------------------------------
// Codec-dispatched serialisation
//
// Each function takes a content_type string (from port config) and
// routes to the appropriate codec.  Component logic calls these
// instead of codec-specific toJson/toBinary directly.
// ---------------------------------------------------------------------------

/// Serialize ObjectDetail using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::tactical::json_codec::ObjectDetail& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::tactical::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::tactical::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::tactical::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize ObjectDetail using the codec identified by content_type.
inline pyramid::data_model::tactical::json_codec::ObjectDetail deserializeObjectDetail(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::tactical::json_codec::objectDetailFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize ObjectEvidenceRequirement using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::tactical::json_codec::ObjectEvidenceRequirement& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::tactical::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::tactical::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::tactical::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize ObjectEvidenceRequirement using the codec identified by content_type.
inline pyramid::data_model::tactical::json_codec::ObjectEvidenceRequirement deserializeObjectEvidenceRequirement(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::tactical::json_codec::objectEvidenceRequirementFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize ObjectInterestRequirement using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::tactical::json_codec::ObjectInterestRequirement& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::tactical::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::tactical::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::tactical::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize ObjectInterestRequirement using the codec identified by content_type.
inline pyramid::data_model::tactical::json_codec::ObjectInterestRequirement deserializeObjectInterestRequirement(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::tactical::json_codec::objectInterestRequirementFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize ObjectMatch using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::tactical::json_codec::ObjectMatch& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::tactical::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::tactical::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::tactical::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize ObjectMatch using the codec identified by content_type.
inline pyramid::data_model::tactical::json_codec::ObjectMatch deserializeObjectMatch(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::tactical::json_codec::objectMatchFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

// ---------------------------------------------------------------------------
// PCL message helpers
// ---------------------------------------------------------------------------

/// Build a pcl_msg_t from a serialized payload and content type.
/// The caller must keep `payload` alive for the lifetime of the returned msg.
inline pcl_msg_t makeMsg(const std::string& payload,
                         const char* content_type) {
    pcl_msg_t msg{};
    msg.data      = payload.data();
    msg.size      = static_cast<uint32_t>(payload.size());
    msg.type_name = content_type;
    return msg;
}

} // namespace pyramid::data_model::tactical::codec_dispatch
