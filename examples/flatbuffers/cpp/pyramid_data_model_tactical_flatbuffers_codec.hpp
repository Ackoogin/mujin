// Auto-generated FlatBuffers PCL codec — do not edit
// Backend: flatbuffers | Namespace: pyramid::data_model::tactical::flatbuffers_codec
#pragma once

// Include the flatc-generated header (run flatc on pyramid_data_model_tactical.fbs)
#include "pyramid_data_model_tactical_generated.h"

#include <flatbuffers/flatbuffers.h>
#include <string>
#include <vector>
#include <cstdint>

namespace pyramid::data_model::tactical::flatbuffers_codec {

static constexpr const char* kContentType = "application/flatbuffers";

/// Serialise ObjectDetail to a FlatBuffer binary blob.
inline std::string toBinary(const ObjectDetailT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = ObjectDetail::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise ObjectDetail from a FlatBuffer binary blob.
inline ObjectDetailT fromBinaryObjectDetail(const void* data, size_t size) {
    ObjectDetailT result;
    auto* fb = flatbuffers::GetRoot<ObjectDetail>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ObjectDetailT fromBinaryObjectDetail(const std::string& s) {
    return fromBinaryObjectDetail(s.data(), s.size());
}

/// Serialise ObjectEvidenceRequirement to a FlatBuffer binary blob.
inline std::string toBinary(const ObjectEvidenceRequirementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = ObjectEvidenceRequirement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise ObjectEvidenceRequirement from a FlatBuffer binary blob.
inline ObjectEvidenceRequirementT fromBinaryObjectEvidenceRequirement(const void* data, size_t size) {
    ObjectEvidenceRequirementT result;
    auto* fb = flatbuffers::GetRoot<ObjectEvidenceRequirement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ObjectEvidenceRequirementT fromBinaryObjectEvidenceRequirement(const std::string& s) {
    return fromBinaryObjectEvidenceRequirement(s.data(), s.size());
}

/// Serialise ObjectInterestRequirement to a FlatBuffer binary blob.
inline std::string toBinary(const ObjectInterestRequirementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = ObjectInterestRequirement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise ObjectInterestRequirement from a FlatBuffer binary blob.
inline ObjectInterestRequirementT fromBinaryObjectInterestRequirement(const void* data, size_t size) {
    ObjectInterestRequirementT result;
    auto* fb = flatbuffers::GetRoot<ObjectInterestRequirement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ObjectInterestRequirementT fromBinaryObjectInterestRequirement(const std::string& s) {
    return fromBinaryObjectInterestRequirement(s.data(), s.size());
}

/// Serialise ObjectMatch to a FlatBuffer binary blob.
inline std::string toBinary(const ObjectMatchT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = ObjectMatch::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise ObjectMatch from a FlatBuffer binary blob.
inline ObjectMatchT fromBinaryObjectMatch(const void* data, size_t size) {
    ObjectMatchT result;
    auto* fb = flatbuffers::GetRoot<ObjectMatch>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ObjectMatchT fromBinaryObjectMatch(const std::string& s) {
    return fromBinaryObjectMatch(s.data(), s.size());
}

} // namespace pyramid::data_model::tactical::flatbuffers_codec
