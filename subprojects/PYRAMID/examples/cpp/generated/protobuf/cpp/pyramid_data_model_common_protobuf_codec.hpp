// Auto-generated Protobuf PCL codec — do not edit
// Backend: protobuf | Namespace: pyramid::data_model::common::protobuf_codec
//
// Wraps protoc-generated SerializeToString / ParseFromString
// into the PCL codec API surface (toBinary / fromBinary).
//
// Requires: protoc-generated pyramid/data_model/common.pb.h
// Link with: libprotobuf
#pragma once

#include "pyramid/data_model/common.pb.h"

#include <string>
#include <cstdint>

namespace pyramid::data_model::common::protobuf_codec {

static constexpr const char* kContentType = "application/protobuf";

/// Serialise GeodeticPosition via protobuf.
inline std::string toBinary(const pyramid::data_model::common::GeodeticPosition& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise GeodeticPosition from protobuf wire format.
inline pyramid::data_model::common::GeodeticPosition fromBinaryGeodeticPosition(const void* data, size_t size) {
    pyramid::data_model::common::GeodeticPosition result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::common::GeodeticPosition fromBinaryGeodeticPosition(const std::string& s) {
    return fromBinaryGeodeticPosition(s.data(), s.size());
}

/// Serialise PolyArea via protobuf.
inline std::string toBinary(const pyramid::data_model::common::PolyArea& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise PolyArea from protobuf wire format.
inline pyramid::data_model::common::PolyArea fromBinaryPolyArea(const void* data, size_t size) {
    pyramid::data_model::common::PolyArea result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::common::PolyArea fromBinaryPolyArea(const std::string& s) {
    return fromBinaryPolyArea(s.data(), s.size());
}

/// Serialise Achievement via protobuf.
inline std::string toBinary(const pyramid::data_model::common::Achievement& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise Achievement from protobuf wire format.
inline pyramid::data_model::common::Achievement fromBinaryAchievement(const void* data, size_t size) {
    pyramid::data_model::common::Achievement result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::common::Achievement fromBinaryAchievement(const std::string& s) {
    return fromBinaryAchievement(s.data(), s.size());
}

/// Serialise Requirement via protobuf.
inline std::string toBinary(const pyramid::data_model::common::Requirement& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise Requirement from protobuf wire format.
inline pyramid::data_model::common::Requirement fromBinaryRequirement(const void* data, size_t size) {
    pyramid::data_model::common::Requirement result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::common::Requirement fromBinaryRequirement(const std::string& s) {
    return fromBinaryRequirement(s.data(), s.size());
}

/// Serialise Capability via protobuf.
inline std::string toBinary(const pyramid::data_model::common::Capability& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise Capability from protobuf wire format.
inline pyramid::data_model::common::Capability fromBinaryCapability(const void* data, size_t size) {
    pyramid::data_model::common::Capability result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::common::Capability fromBinaryCapability(const std::string& s) {
    return fromBinaryCapability(s.data(), s.size());
}

/// Serialise Entity via protobuf.
inline std::string toBinary(const pyramid::data_model::common::Entity& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise Entity from protobuf wire format.
inline pyramid::data_model::common::Entity fromBinaryEntity(const void* data, size_t size) {
    pyramid::data_model::common::Entity result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::common::Entity fromBinaryEntity(const std::string& s) {
    return fromBinaryEntity(s.data(), s.size());
}

/// Serialise CircleArea via protobuf.
inline std::string toBinary(const pyramid::data_model::common::CircleArea& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise CircleArea from protobuf wire format.
inline pyramid::data_model::common::CircleArea fromBinaryCircleArea(const void* data, size_t size) {
    pyramid::data_model::common::CircleArea result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::common::CircleArea fromBinaryCircleArea(const std::string& s) {
    return fromBinaryCircleArea(s.data(), s.size());
}

/// Serialise Point via protobuf.
inline std::string toBinary(const pyramid::data_model::common::Point& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise Point from protobuf wire format.
inline pyramid::data_model::common::Point fromBinaryPoint(const void* data, size_t size) {
    pyramid::data_model::common::Point result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::common::Point fromBinaryPoint(const std::string& s) {
    return fromBinaryPoint(s.data(), s.size());
}

/// Serialise Contraint via protobuf.
inline std::string toBinary(const pyramid::data_model::common::Contraint& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise Contraint from protobuf wire format.
inline pyramid::data_model::common::Contraint fromBinaryContraint(const void* data, size_t size) {
    pyramid::data_model::common::Contraint result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::common::Contraint fromBinaryContraint(const std::string& s) {
    return fromBinaryContraint(s.data(), s.size());
}

/// Serialise Ack via protobuf.
inline std::string toBinary(const pyramid::data_model::common::Ack& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise Ack from protobuf wire format.
inline pyramid::data_model::common::Ack fromBinaryAck(const void* data, size_t size) {
    pyramid::data_model::common::Ack result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::common::Ack fromBinaryAck(const std::string& s) {
    return fromBinaryAck(s.data(), s.size());
}

/// Serialise Query via protobuf.
inline std::string toBinary(const pyramid::data_model::common::Query& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise Query from protobuf wire format.
inline pyramid::data_model::common::Query fromBinaryQuery(const void* data, size_t size) {
    pyramid::data_model::common::Query result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::common::Query fromBinaryQuery(const std::string& s) {
    return fromBinaryQuery(s.data(), s.size());
}

} // namespace pyramid::data_model::common::protobuf_codec
