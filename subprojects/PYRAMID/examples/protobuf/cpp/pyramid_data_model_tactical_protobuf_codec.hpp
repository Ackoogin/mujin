// Auto-generated Protobuf PCL codec — do not edit
// Backend: protobuf | Namespace: pyramid::data_model::tactical::protobuf_codec
//
// Wraps protoc-generated SerializeToString / ParseFromString
// into the PCL codec API surface (toBinary / fromBinary).
//
// Requires: protoc-generated pyramid/data_model/tactical.pb.h
// Link with: libprotobuf
#pragma once

#include "pyramid/data_model/tactical.pb.h"

#include <string>
#include <cstdint>

namespace pyramid::data_model::tactical::protobuf_codec {

static constexpr const char* kContentType = "application/protobuf";

/// Serialise ObjectDetail via protobuf.
inline std::string toBinary(const pyramid::data_model::tactical::ObjectDetail& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise ObjectDetail from protobuf wire format.
inline pyramid::data_model::tactical::ObjectDetail fromBinaryObjectDetail(const void* data, size_t size) {
    pyramid::data_model::tactical::ObjectDetail result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::tactical::ObjectDetail fromBinaryObjectDetail(const std::string& s) {
    return fromBinaryObjectDetail(s.data(), s.size());
}

/// Serialise ObjectEvidenceRequirement via protobuf.
inline std::string toBinary(const pyramid::data_model::tactical::ObjectEvidenceRequirement& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise ObjectEvidenceRequirement from protobuf wire format.
inline pyramid::data_model::tactical::ObjectEvidenceRequirement fromBinaryObjectEvidenceRequirement(const void* data, size_t size) {
    pyramid::data_model::tactical::ObjectEvidenceRequirement result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::tactical::ObjectEvidenceRequirement fromBinaryObjectEvidenceRequirement(const std::string& s) {
    return fromBinaryObjectEvidenceRequirement(s.data(), s.size());
}

/// Serialise ObjectInterestRequirement via protobuf.
inline std::string toBinary(const pyramid::data_model::tactical::ObjectInterestRequirement& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise ObjectInterestRequirement from protobuf wire format.
inline pyramid::data_model::tactical::ObjectInterestRequirement fromBinaryObjectInterestRequirement(const void* data, size_t size) {
    pyramid::data_model::tactical::ObjectInterestRequirement result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::tactical::ObjectInterestRequirement fromBinaryObjectInterestRequirement(const std::string& s) {
    return fromBinaryObjectInterestRequirement(s.data(), s.size());
}

/// Serialise ObjectMatch via protobuf.
inline std::string toBinary(const pyramid::data_model::tactical::ObjectMatch& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise ObjectMatch from protobuf wire format.
inline pyramid::data_model::tactical::ObjectMatch fromBinaryObjectMatch(const void* data, size_t size) {
    pyramid::data_model::tactical::ObjectMatch result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::tactical::ObjectMatch fromBinaryObjectMatch(const std::string& s) {
    return fromBinaryObjectMatch(s.data(), s.size());
}

} // namespace pyramid::data_model::tactical::protobuf_codec
