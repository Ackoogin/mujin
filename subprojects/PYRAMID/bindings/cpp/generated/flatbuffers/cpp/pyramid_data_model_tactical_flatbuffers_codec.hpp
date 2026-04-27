// Auto-generated FlatBuffers PCL codec -- do not edit
// Backend: flatbuffers | Namespace: pyramid::data_model::tactical::flatbuffers_codec
#pragma once

// Include the flatc-generated header (run flatc on pyramid_data_model_tactical.fbs)
#include "pyramid_data_model_tactical_generated.h"

#include <flatbuffers/flatbuffers.h>
#include <cstddef>
#include <cstdint>
#include <string>

namespace pyramid::data_model::tactical::flatbuffers_codec {

static constexpr const char* kContentType = "application/flatbuffers";

inline std::string toBinary(const ObjectDetailT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = ObjectDetail::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline ObjectDetailT fromBinaryObjectDetail(const void* data, size_t size) {
    (void) size;
    ObjectDetailT result;
    auto* fb = flatbuffers::GetRoot<ObjectDetail>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ObjectDetailT fromBinaryObjectDetail(const std::string& s) {
    return fromBinaryObjectDetail(s.data(), s.size());
}

inline std::string toBinary(const ObjectEvidenceRequirementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = ObjectEvidenceRequirement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline ObjectEvidenceRequirementT fromBinaryObjectEvidenceRequirement(const void* data, size_t size) {
    (void) size;
    ObjectEvidenceRequirementT result;
    auto* fb = flatbuffers::GetRoot<ObjectEvidenceRequirement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ObjectEvidenceRequirementT fromBinaryObjectEvidenceRequirement(const std::string& s) {
    return fromBinaryObjectEvidenceRequirement(s.data(), s.size());
}

inline std::string toBinary(const ObjectInterestRequirementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = ObjectInterestRequirement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline ObjectInterestRequirementT fromBinaryObjectInterestRequirement(const void* data, size_t size) {
    (void) size;
    ObjectInterestRequirementT result;
    auto* fb = flatbuffers::GetRoot<ObjectInterestRequirement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ObjectInterestRequirementT fromBinaryObjectInterestRequirement(const std::string& s) {
    return fromBinaryObjectInterestRequirement(s.data(), s.size());
}

inline std::string toBinary(const ObjectMatchT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = ObjectMatch::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline ObjectMatchT fromBinaryObjectMatch(const void* data, size_t size) {
    (void) size;
    ObjectMatchT result;
    auto* fb = flatbuffers::GetRoot<ObjectMatch>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ObjectMatchT fromBinaryObjectMatch(const std::string& s) {
    return fromBinaryObjectMatch(s.data(), s.size());
}

} // namespace pyramid::data_model::tactical::flatbuffers_codec
