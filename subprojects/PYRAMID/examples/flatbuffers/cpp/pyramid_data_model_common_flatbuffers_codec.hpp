// Auto-generated FlatBuffers PCL codec — do not edit
// Backend: flatbuffers | Namespace: pyramid::data_model::common::flatbuffers_codec
#pragma once

// Include the flatc-generated header (run flatc on pyramid_data_model_common.fbs)
#include "pyramid_data_model_common_generated.h"

#include <flatbuffers/flatbuffers.h>
#include <string>
#include <vector>
#include <cstdint>

namespace pyramid::data_model::common::flatbuffers_codec {

static constexpr const char* kContentType = "application/flatbuffers";

/// Serialise GeodeticPosition to a FlatBuffer binary blob.
inline std::string toBinary(const GeodeticPositionT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = GeodeticPosition::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise GeodeticPosition from a FlatBuffer binary blob.
inline GeodeticPositionT fromBinaryGeodeticPosition(const void* data, size_t size) {
    GeodeticPositionT result;
    auto* fb = flatbuffers::GetRoot<GeodeticPosition>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline GeodeticPositionT fromBinaryGeodeticPosition(const std::string& s) {
    return fromBinaryGeodeticPosition(s.data(), s.size());
}

/// Serialise PolyArea to a FlatBuffer binary blob.
inline std::string toBinary(const PolyAreaT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = PolyArea::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise PolyArea from a FlatBuffer binary blob.
inline PolyAreaT fromBinaryPolyArea(const void* data, size_t size) {
    PolyAreaT result;
    auto* fb = flatbuffers::GetRoot<PolyArea>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline PolyAreaT fromBinaryPolyArea(const std::string& s) {
    return fromBinaryPolyArea(s.data(), s.size());
}

/// Serialise Achievement to a FlatBuffer binary blob.
inline std::string toBinary(const AchievementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Achievement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise Achievement from a FlatBuffer binary blob.
inline AchievementT fromBinaryAchievement(const void* data, size_t size) {
    AchievementT result;
    auto* fb = flatbuffers::GetRoot<Achievement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline AchievementT fromBinaryAchievement(const std::string& s) {
    return fromBinaryAchievement(s.data(), s.size());
}

/// Serialise Requirement to a FlatBuffer binary blob.
inline std::string toBinary(const RequirementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Requirement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise Requirement from a FlatBuffer binary blob.
inline RequirementT fromBinaryRequirement(const void* data, size_t size) {
    RequirementT result;
    auto* fb = flatbuffers::GetRoot<Requirement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline RequirementT fromBinaryRequirement(const std::string& s) {
    return fromBinaryRequirement(s.data(), s.size());
}

/// Serialise Capability to a FlatBuffer binary blob.
inline std::string toBinary(const CapabilityT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Capability::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise Capability from a FlatBuffer binary blob.
inline CapabilityT fromBinaryCapability(const void* data, size_t size) {
    CapabilityT result;
    auto* fb = flatbuffers::GetRoot<Capability>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline CapabilityT fromBinaryCapability(const std::string& s) {
    return fromBinaryCapability(s.data(), s.size());
}

/// Serialise Entity to a FlatBuffer binary blob.
inline std::string toBinary(const EntityT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Entity::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise Entity from a FlatBuffer binary blob.
inline EntityT fromBinaryEntity(const void* data, size_t size) {
    EntityT result;
    auto* fb = flatbuffers::GetRoot<Entity>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline EntityT fromBinaryEntity(const std::string& s) {
    return fromBinaryEntity(s.data(), s.size());
}

/// Serialise CircleArea to a FlatBuffer binary blob.
inline std::string toBinary(const CircleAreaT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = CircleArea::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise CircleArea from a FlatBuffer binary blob.
inline CircleAreaT fromBinaryCircleArea(const void* data, size_t size) {
    CircleAreaT result;
    auto* fb = flatbuffers::GetRoot<CircleArea>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline CircleAreaT fromBinaryCircleArea(const std::string& s) {
    return fromBinaryCircleArea(s.data(), s.size());
}

/// Serialise Point to a FlatBuffer binary blob.
inline std::string toBinary(const PointT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Point::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise Point from a FlatBuffer binary blob.
inline PointT fromBinaryPoint(const void* data, size_t size) {
    PointT result;
    auto* fb = flatbuffers::GetRoot<Point>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline PointT fromBinaryPoint(const std::string& s) {
    return fromBinaryPoint(s.data(), s.size());
}

/// Serialise Contraint to a FlatBuffer binary blob.
inline std::string toBinary(const ContraintT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Contraint::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise Contraint from a FlatBuffer binary blob.
inline ContraintT fromBinaryContraint(const void* data, size_t size) {
    ContraintT result;
    auto* fb = flatbuffers::GetRoot<Contraint>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ContraintT fromBinaryContraint(const std::string& s) {
    return fromBinaryContraint(s.data(), s.size());
}

/// Serialise Ack to a FlatBuffer binary blob.
inline std::string toBinary(const AckT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Ack::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise Ack from a FlatBuffer binary blob.
inline AckT fromBinaryAck(const void* data, size_t size) {
    AckT result;
    auto* fb = flatbuffers::GetRoot<Ack>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline AckT fromBinaryAck(const std::string& s) {
    return fromBinaryAck(s.data(), s.size());
}

/// Serialise Query to a FlatBuffer binary blob.
inline std::string toBinary(const QueryT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Query::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise Query from a FlatBuffer binary blob.
inline QueryT fromBinaryQuery(const void* data, size_t size) {
    QueryT result;
    auto* fb = flatbuffers::GetRoot<Query>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline QueryT fromBinaryQuery(const std::string& s) {
    return fromBinaryQuery(s.data(), s.size());
}

} // namespace pyramid::data_model::common::flatbuffers_codec
