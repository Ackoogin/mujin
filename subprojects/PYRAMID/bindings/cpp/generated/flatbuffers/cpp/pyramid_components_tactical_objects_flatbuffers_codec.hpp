// Auto-generated FlatBuffers PCL codec — do not edit
// Backend: flatbuffers | Namespace: pyramid::components::tactical_objects::flatbuffers_codec
#pragma once

// Include the flatc-generated header (run flatc on pyramid_components_tactical_objects.fbs)
#include "pyramid_components_tactical_objects_generated.h"

#include <flatbuffers/flatbuffers.h>
#include <cstddef>
#include <cstdint>
#include <string>

namespace pyramid::components::tactical_objects::flatbuffers_codec {

static constexpr const char* kContentType = "application/flatbuffers";

inline std::string toBinary(const PositionT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Position::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline PositionT fromBinaryPosition(const void* data, size_t size) {
    (void) size;
    PositionT result;
    auto* fb = flatbuffers::GetRoot<Position>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline PositionT fromBinaryPosition(const std::string& s) {
    return fromBinaryPosition(s.data(), s.size());
}

inline std::string toBinary(const VelocityT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Velocity::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline VelocityT fromBinaryVelocity(const void* data, size_t size) {
    (void) size;
    VelocityT result;
    auto* fb = flatbuffers::GetRoot<Velocity>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline VelocityT fromBinaryVelocity(const std::string& s) {
    return fromBinaryVelocity(s.data(), s.size());
}

inline std::string toBinary(const BoundingBoxT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = BoundingBox::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline BoundingBoxT fromBinaryBoundingBox(const void* data, size_t size) {
    (void) size;
    BoundingBoxT result;
    auto* fb = flatbuffers::GetRoot<BoundingBox>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline BoundingBoxT fromBinaryBoundingBox(const std::string& s) {
    return fromBinaryBoundingBox(s.data(), s.size());
}

inline std::string toBinary(const SourceRefT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = SourceRef::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline SourceRefT fromBinarySourceRef(const void* data, size_t size) {
    (void) size;
    SourceRefT result;
    auto* fb = flatbuffers::GetRoot<SourceRef>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline SourceRefT fromBinarySourceRef(const std::string& s) {
    return fromBinarySourceRef(s.data(), s.size());
}

inline std::string toBinary(const MilClassProfileT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = MilClassProfile::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline MilClassProfileT fromBinaryMilClassProfile(const void* data, size_t size) {
    (void) size;
    MilClassProfileT result;
    auto* fb = flatbuffers::GetRoot<MilClassProfile>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline MilClassProfileT fromBinaryMilClassProfile(const std::string& s) {
    return fromBinaryMilClassProfile(s.data(), s.size());
}

inline std::string toBinary(const ZoneGeometryT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = ZoneGeometry::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline ZoneGeometryT fromBinaryZoneGeometry(const void* data, size_t size) {
    (void) size;
    ZoneGeometryT result;
    auto* fb = flatbuffers::GetRoot<ZoneGeometry>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ZoneGeometryT fromBinaryZoneGeometry(const std::string& s) {
    return fromBinaryZoneGeometry(s.data(), s.size());
}

inline std::string toBinary(const TacticalObjectT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = TacticalObject::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline TacticalObjectT fromBinaryTacticalObject(const void* data, size_t size) {
    (void) size;
    TacticalObjectT result;
    auto* fb = flatbuffers::GetRoot<TacticalObject>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline TacticalObjectT fromBinaryTacticalObject(const std::string& s) {
    return fromBinaryTacticalObject(s.data(), s.size());
}

inline std::string toBinary(const TacticalObjectQueryT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = TacticalObjectQuery::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline TacticalObjectQueryT fromBinaryTacticalObjectQuery(const void* data, size_t size) {
    (void) size;
    TacticalObjectQueryT result;
    auto* fb = flatbuffers::GetRoot<TacticalObjectQuery>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline TacticalObjectQueryT fromBinaryTacticalObjectQuery(const std::string& s) {
    return fromBinaryTacticalObjectQuery(s.data(), s.size());
}

inline std::string toBinary(const ZoneT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Zone::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline ZoneT fromBinaryZone(const void* data, size_t size) {
    (void) size;
    ZoneT result;
    auto* fb = flatbuffers::GetRoot<Zone>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ZoneT fromBinaryZone(const std::string& s) {
    return fromBinaryZone(s.data(), s.size());
}

inline std::string toBinary(const ObservationT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Observation::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline ObservationT fromBinaryObservation(const void* data, size_t size) {
    (void) size;
    ObservationT result;
    auto* fb = flatbuffers::GetRoot<Observation>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ObservationT fromBinaryObservation(const std::string& s) {
    return fromBinaryObservation(s.data(), s.size());
}

inline std::string toBinary(const EntityUpdateFrameT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = EntityUpdateFrame::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline EntityUpdateFrameT fromBinaryEntityUpdateFrame(const void* data, size_t size) {
    (void) size;
    EntityUpdateFrameT result;
    auto* fb = flatbuffers::GetRoot<EntityUpdateFrame>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline EntityUpdateFrameT fromBinaryEntityUpdateFrame(const std::string& s) {
    return fromBinaryEntityUpdateFrame(s.data(), s.size());
}

inline std::string toBinary(const InterestCriteriaT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = InterestCriteria::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline InterestCriteriaT fromBinaryInterestCriteria(const void* data, size_t size) {
    (void) size;
    InterestCriteriaT result;
    auto* fb = flatbuffers::GetRoot<InterestCriteria>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline InterestCriteriaT fromBinaryInterestCriteria(const std::string& s) {
    return fromBinaryInterestCriteria(s.data(), s.size());
}

} // namespace pyramid::components::tactical_objects::flatbuffers_codec
