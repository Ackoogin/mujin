// Auto-generated FlatBuffers PCL codec -- do not edit
// Backend: flatbuffers | Namespace: pyramid::data_model::sensorproducts::flatbuffers_codec
#pragma once

// Include the flatc-generated header (run flatc on pyramid_data_model_sensorproducts.fbs)
#include "pyramid_data_model_sensorproducts_generated.h"

#include <flatbuffers/flatbuffers.h>
#include <cstddef>
#include <cstdint>
#include <string>

namespace pyramid::data_model::sensorproducts::flatbuffers_codec {

static constexpr const char* kContentType = "application/flatbuffers";

inline std::string toBinary(const RadarDisplayProductRequirementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = RadarDisplayProductRequirement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline RadarDisplayProductRequirementT fromBinaryRadarDisplayProductRequirement(const void* data, size_t size) {
    (void) size;
    RadarDisplayProductRequirementT result;
    auto* fb = flatbuffers::GetRoot<RadarDisplayProductRequirement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline RadarDisplayProductRequirementT fromBinaryRadarDisplayProductRequirement(const std::string& s) {
    return fromBinaryRadarDisplayProductRequirement(s.data(), s.size());
}

inline std::string toBinary(const RadarProductRequirementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = RadarProductRequirement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline RadarProductRequirementT fromBinaryRadarProductRequirement(const void* data, size_t size) {
    (void) size;
    RadarProductRequirementT result;
    auto* fb = flatbuffers::GetRoot<RadarProductRequirement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline RadarProductRequirementT fromBinaryRadarProductRequirement(const std::string& s) {
    return fromBinaryRadarProductRequirement(s.data(), s.size());
}

} // namespace pyramid::data_model::sensorproducts::flatbuffers_codec
