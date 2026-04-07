// Auto-generated codec dispatch — do not edit
// Namespace: pyramid::data_model::common::codec_dispatch
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
#include "pyramid_data_model_common_json_codec.hpp"

#if defined(CODEC_FLATBUFFERS)
#include "pyramid_data_model_common_flatbuffers_codec.hpp"
#endif

#if defined(CODEC_PROTOBUF)
#include "pyramid_data_model_common_protobuf_codec.hpp"
#endif

namespace pyramid::data_model::common::codec_dispatch {

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

/// Serialize GeodeticPosition using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::common::json_codec::GeodeticPosition& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::common::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::common::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize GeodeticPosition using the codec identified by content_type.
inline pyramid::data_model::common::json_codec::GeodeticPosition deserializeGeodeticPosition(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::geodeticPositionFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize PolyArea using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::common::json_codec::PolyArea& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::common::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::common::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize PolyArea using the codec identified by content_type.
inline pyramid::data_model::common::json_codec::PolyArea deserializePolyArea(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::polyAreaFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize Achievement using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::common::json_codec::Achievement& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::common::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::common::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize Achievement using the codec identified by content_type.
inline pyramid::data_model::common::json_codec::Achievement deserializeAchievement(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::achievementFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize Requirement using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::common::json_codec::Requirement& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::common::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::common::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize Requirement using the codec identified by content_type.
inline pyramid::data_model::common::json_codec::Requirement deserializeRequirement(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::requirementFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize Capability using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::common::json_codec::Capability& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::common::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::common::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize Capability using the codec identified by content_type.
inline pyramid::data_model::common::json_codec::Capability deserializeCapability(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::capabilityFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize Entity using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::common::json_codec::Entity& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::common::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::common::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize Entity using the codec identified by content_type.
inline pyramid::data_model::common::json_codec::Entity deserializeEntity(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::entityFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize CircleArea using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::common::json_codec::CircleArea& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::common::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::common::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize CircleArea using the codec identified by content_type.
inline pyramid::data_model::common::json_codec::CircleArea deserializeCircleArea(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::circleAreaFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize Point using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::common::json_codec::Point& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::common::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::common::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize Point using the codec identified by content_type.
inline pyramid::data_model::common::json_codec::Point deserializePoint(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::pointFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize Contraint using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::common::json_codec::Contraint& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::common::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::common::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize Contraint using the codec identified by content_type.
inline pyramid::data_model::common::json_codec::Contraint deserializeContraint(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::contraintFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize Ack using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::common::json_codec::Ack& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::common::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::common::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize Ack using the codec identified by content_type.
inline pyramid::data_model::common::json_codec::Ack deserializeAck(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::ackFromJson(
            std::string(static_cast<const char*>(data), size));
    // FlatBuffers and Protobuf deserialize to their own types;
    // conversion to the common type requires a mapping layer.
    throw std::runtime_error(
        std::string("unsupported codec for deserialization: ") + content_type);
}

/// Serialize Query using the codec identified by content_type.
inline std::string serialize(const pyramid::data_model::common::json_codec::Query& msg,
                             const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::toJson(msg);
#if defined(CODEC_FLATBUFFERS)
    if (std::strcmp(content_type, kFlatBuffers) == 0)
        return pyramid::data_model::common::flatbuffers_codec::toBinary(msg);
#endif
#if defined(CODEC_PROTOBUF)
    if (std::strcmp(content_type, kProtobuf) == 0)
        return pyramid::data_model::common::protobuf_codec::toBinary(msg);
#endif
    throw std::runtime_error(
        std::string("unsupported codec: ") + content_type);
}

/// Deserialize Query using the codec identified by content_type.
inline pyramid::data_model::common::json_codec::Query deserializeQuery(
    const void* data, size_t size, const char* content_type) {
    if (std::strcmp(content_type, kJson) == 0)
        return pyramid::data_model::common::json_codec::queryFromJson(
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

} // namespace pyramid::data_model::common::codec_dispatch
