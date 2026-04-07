// Auto-generated Protobuf PCL codec — do not edit
// Backend: protobuf | Namespace: pyramid::data_model::base::protobuf_codec
//
// Wraps protoc-generated SerializeToString / ParseFromString
// into the PCL codec API surface (toBinary / fromBinary).
//
// Requires: protoc-generated pyramid/data_model/base.pb.h
// Link with: libprotobuf
#pragma once

#include "pyramid/data_model/base.pb.h"

#include <string>
#include <cstdint>

namespace pyramid::data_model::base::protobuf_codec {

static constexpr const char* kContentType = "application/protobuf";

/// Serialise Angle via protobuf.
inline std::string toBinary(const pyramid::data_model::base::Angle& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise Angle from protobuf wire format.
inline pyramid::data_model::base::Angle fromBinaryAngle(const void* data, size_t size) {
    pyramid::data_model::base::Angle result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::base::Angle fromBinaryAngle(const std::string& s) {
    return fromBinaryAngle(s.data(), s.size());
}

/// Serialise Length via protobuf.
inline std::string toBinary(const pyramid::data_model::base::Length& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise Length from protobuf wire format.
inline pyramid::data_model::base::Length fromBinaryLength(const void* data, size_t size) {
    pyramid::data_model::base::Length result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::base::Length fromBinaryLength(const std::string& s) {
    return fromBinaryLength(s.data(), s.size());
}

/// Serialise Timestamp via protobuf.
inline std::string toBinary(const pyramid::data_model::base::Timestamp& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise Timestamp from protobuf wire format.
inline pyramid::data_model::base::Timestamp fromBinaryTimestamp(const void* data, size_t size) {
    pyramid::data_model::base::Timestamp result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::base::Timestamp fromBinaryTimestamp(const std::string& s) {
    return fromBinaryTimestamp(s.data(), s.size());
}

/// Serialise Identifier via protobuf.
inline std::string toBinary(const pyramid::data_model::base::Identifier& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise Identifier from protobuf wire format.
inline pyramid::data_model::base::Identifier fromBinaryIdentifier(const void* data, size_t size) {
    pyramid::data_model::base::Identifier result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::base::Identifier fromBinaryIdentifier(const std::string& s) {
    return fromBinaryIdentifier(s.data(), s.size());
}

/// Serialise Speed via protobuf.
inline std::string toBinary(const pyramid::data_model::base::Speed& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise Speed from protobuf wire format.
inline pyramid::data_model::base::Speed fromBinarySpeed(const void* data, size_t size) {
    pyramid::data_model::base::Speed result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::base::Speed fromBinarySpeed(const std::string& s) {
    return fromBinarySpeed(s.data(), s.size());
}

/// Serialise Percentage via protobuf.
inline std::string toBinary(const pyramid::data_model::base::Percentage& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise Percentage from protobuf wire format.
inline pyramid::data_model::base::Percentage fromBinaryPercentage(const void* data, size_t size) {
    pyramid::data_model::base::Percentage result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::base::Percentage fromBinaryPercentage(const std::string& s) {
    return fromBinaryPercentage(s.data(), s.size());
}

} // namespace pyramid::data_model::base::protobuf_codec
