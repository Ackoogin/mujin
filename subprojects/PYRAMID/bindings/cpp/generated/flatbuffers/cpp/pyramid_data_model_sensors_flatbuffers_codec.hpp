// Auto-generated FlatBuffers PCL codec -- do not edit
// Backend: flatbuffers | Namespace: pyramid::data_model::sensors::flatbuffers_codec
#pragma once

// Include the flatc-generated header (run flatc on pyramid_data_model_sensors.fbs)
#include "pyramid_data_model_sensors_generated.h"

#include <flatbuffers/flatbuffers.h>
#include <cstddef>
#include <cstdint>
#include <string>

namespace pyramid::data_model::sensors::flatbuffers_codec {

static constexpr const char* kContentType = "application/flatbuffers";

inline std::string toBinary(const InterpretationRequirementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = InterpretationRequirement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline InterpretationRequirementT fromBinaryInterpretationRequirement(const void* data, size_t size) {
    (void) size;
    InterpretationRequirementT result;
    auto* fb = flatbuffers::GetRoot<InterpretationRequirement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline InterpretationRequirementT fromBinaryInterpretationRequirement(const std::string& s) {
    return fromBinaryInterpretationRequirement(s.data(), s.size());
}

inline std::string toBinary(const ManualTrackRequirementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = ManualTrackRequirement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline ManualTrackRequirementT fromBinaryManualTrackRequirement(const void* data, size_t size) {
    (void) size;
    ManualTrackRequirementT result;
    auto* fb = flatbuffers::GetRoot<ManualTrackRequirement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ManualTrackRequirementT fromBinaryManualTrackRequirement(const std::string& s) {
    return fromBinaryManualTrackRequirement(s.data(), s.size());
}

inline std::string toBinary(const ATIRequirementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = ATIRequirement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline ATIRequirementT fromBinaryATIRequirement(const void* data, size_t size) {
    (void) size;
    ATIRequirementT result;
    auto* fb = flatbuffers::GetRoot<ATIRequirement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ATIRequirementT fromBinaryATIRequirement(const std::string& s) {
    return fromBinaryATIRequirement(s.data(), s.size());
}

inline std::string toBinary(const TrackProvisionRequirementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = TrackProvisionRequirement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline TrackProvisionRequirementT fromBinaryTrackProvisionRequirement(const void* data, size_t size) {
    (void) size;
    TrackProvisionRequirementT result;
    auto* fb = flatbuffers::GetRoot<TrackProvisionRequirement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline TrackProvisionRequirementT fromBinaryTrackProvisionRequirement(const std::string& s) {
    return fromBinaryTrackProvisionRequirement(s.data(), s.size());
}

inline std::string toBinary(const ObjectEvidenceProvisionRequirementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = ObjectEvidenceProvisionRequirement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline ObjectEvidenceProvisionRequirementT fromBinaryObjectEvidenceProvisionRequirement(const void* data, size_t size) {
    (void) size;
    ObjectEvidenceProvisionRequirementT result;
    auto* fb = flatbuffers::GetRoot<ObjectEvidenceProvisionRequirement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ObjectEvidenceProvisionRequirementT fromBinaryObjectEvidenceProvisionRequirement(const std::string& s) {
    return fromBinaryObjectEvidenceProvisionRequirement(s.data(), s.size());
}

inline std::string toBinary(const ObjectAquisitionRequirementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = ObjectAquisitionRequirement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline ObjectAquisitionRequirementT fromBinaryObjectAquisitionRequirement(const void* data, size_t size) {
    (void) size;
    ObjectAquisitionRequirementT result;
    auto* fb = flatbuffers::GetRoot<ObjectAquisitionRequirement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ObjectAquisitionRequirementT fromBinaryObjectAquisitionRequirement(const std::string& s) {
    return fromBinaryObjectAquisitionRequirement(s.data(), s.size());
}

inline std::string toBinary(const SensorObjectT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = SensorObject::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline SensorObjectT fromBinarySensorObject(const void* data, size_t size) {
    (void) size;
    SensorObjectT result;
    auto* fb = flatbuffers::GetRoot<SensorObject>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline SensorObjectT fromBinarySensorObject(const std::string& s) {
    return fromBinarySensorObject(s.data(), s.size());
}

inline std::string toBinary(const RadarModeChangeRequirementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = RadarModeChangeRequirement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline RadarModeChangeRequirementT fromBinaryRadarModeChangeRequirement(const void* data, size_t size) {
    (void) size;
    RadarModeChangeRequirementT result;
    auto* fb = flatbuffers::GetRoot<RadarModeChangeRequirement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline RadarModeChangeRequirementT fromBinaryRadarModeChangeRequirement(const std::string& s) {
    return fromBinaryRadarModeChangeRequirement(s.data(), s.size());
}

} // namespace pyramid::data_model::sensors::flatbuffers_codec
