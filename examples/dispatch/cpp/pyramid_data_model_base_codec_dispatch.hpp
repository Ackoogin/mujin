// Auto-generated codec dispatch — do not edit
// Namespace: pyramid::data_model::base::codec_dispatch
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
#include "pyramid_data_model_base_json_codec.hpp"

#if defined(CODEC_FLATBUFFERS)
#include "pyramid_data_model_base_flatbuffers_codec.hpp"
#endif

#if defined(CODEC_PROTOBUF)
#include "pyramid_data_model_base_protobuf_codec.hpp"
#endif

namespace pyramid::data_model::base::codec_dispatch {

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

/// Serialize Angle using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::base::json_codec::Angle& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::base::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::base::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::base::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize Angle using the codec identified by content_type.
inline pyramid::data_model::base::json_codec::Angle deserializeAngle(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::base::json_codec::angleFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize Length using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::base::json_codec::Length& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::base::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::base::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::base::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize Length using the codec identified by content_type.
inline pyramid::data_model::base::json_codec::Length deserializeLength(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::base::json_codec::lengthFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize Timestamp using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::base::json_codec::Timestamp& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::base::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::base::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::base::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize Timestamp using the codec identified by content_type.
inline pyramid::data_model::base::json_codec::Timestamp deserializeTimestamp(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::base::json_codec::timestampFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize Identifier using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::base::json_codec::Identifier& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::base::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::base::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::base::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize Identifier using the codec identified by content_type.
inline pyramid::data_model::base::json_codec::Identifier deserializeIdentifier(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::base::json_codec::identifierFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize Speed using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::base::json_codec::Speed& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::base::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::base::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::base::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize Speed using the codec identified by content_type.
inline pyramid::data_model::base::json_codec::Speed deserializeSpeed(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::base::json_codec::speedFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize Percentage using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::base::json_codec::Percentage& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::base::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::base::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::base::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize Percentage using the codec identified by content_type.
inline pyramid::data_model::base::json_codec::Percentage deserializePercentage(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::base::json_codec::percentageFromJson(
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

} // namespace pyramid::data_model::base::codec_dispatch
