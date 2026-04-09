// Auto-generated FlatBuffers PCL codec — do not edit
// Backend: flatbuffers | Namespace: pyramid::data_model::common::flatbuffers_codec
#pragma once

// Include the flatc-generated header (run flatc on pyramid_data_model_common.fbs)
#include "pyramid_data_model_common_generated.h"

#include <flatbuffers/flatbuffers.h>
#include <cstddef>
#include <cstdint>
#include <string>

namespace pyramid::data_model::common::flatbuffers_codec {

static constexpr const char* kContentType = "application/flatbuffers";

inline std::string toBinary(const GeodeticPositionT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = GeodeticPosition::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline GeodeticPositionT fromBinaryGeodeticPosition(const void* data, size_t size) {
    (void) size;
    GeodeticPositionT result;
    auto* fb = flatbuffers::GetRoot<GeodeticPosition>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline GeodeticPositionT fromBinaryGeodeticPosition(const std::string& s) {
    return fromBinaryGeodeticPosition(s.data(), s.size());
}

inline std::string toBinary(const PolyAreaT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = PolyArea::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline PolyAreaT fromBinaryPolyArea(const void* data, size_t size) {
    (void) size;
    PolyAreaT result;
    auto* fb = flatbuffers::GetRoot<PolyArea>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline PolyAreaT fromBinaryPolyArea(const std::string& s) {
    return fromBinaryPolyArea(s.data(), s.size());
}

inline std::string toBinary(const AchievementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Achievement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline AchievementT fromBinaryAchievement(const void* data, size_t size) {
    (void) size;
    AchievementT result;
    auto* fb = flatbuffers::GetRoot<Achievement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline AchievementT fromBinaryAchievement(const std::string& s) {
    return fromBinaryAchievement(s.data(), s.size());
}

inline std::string toBinary(const RequirementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Requirement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline RequirementT fromBinaryRequirement(const void* data, size_t size) {
    (void) size;
    RequirementT result;
    auto* fb = flatbuffers::GetRoot<Requirement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline RequirementT fromBinaryRequirement(const std::string& s) {
    return fromBinaryRequirement(s.data(), s.size());
}

inline std::string toBinary(const CapabilityT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Capability::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline CapabilityT fromBinaryCapability(const void* data, size_t size) {
    (void) size;
    CapabilityT result;
    auto* fb = flatbuffers::GetRoot<Capability>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline CapabilityT fromBinaryCapability(const std::string& s) {
    return fromBinaryCapability(s.data(), s.size());
}

inline std::string toBinary(const EntityT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Entity::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline EntityT fromBinaryEntity(const void* data, size_t size) {
    (void) size;
    EntityT result;
    auto* fb = flatbuffers::GetRoot<Entity>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline EntityT fromBinaryEntity(const std::string& s) {
    return fromBinaryEntity(s.data(), s.size());
}

inline std::string toBinary(const CircleAreaT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = CircleArea::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline CircleAreaT fromBinaryCircleArea(const void* data, size_t size) {
    (void) size;
    CircleAreaT result;
    auto* fb = flatbuffers::GetRoot<CircleArea>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline CircleAreaT fromBinaryCircleArea(const std::string& s) {
    return fromBinaryCircleArea(s.data(), s.size());
}

inline std::string toBinary(const PointT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Point::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline PointT fromBinaryPoint(const void* data, size_t size) {
    (void) size;
    PointT result;
    auto* fb = flatbuffers::GetRoot<Point>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline PointT fromBinaryPoint(const std::string& s) {
    return fromBinaryPoint(s.data(), s.size());
}

inline std::string toBinary(const ContraintT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Contraint::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline ContraintT fromBinaryContraint(const void* data, size_t size) {
    (void) size;
    ContraintT result;
    auto* fb = flatbuffers::GetRoot<Contraint>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ContraintT fromBinaryContraint(const std::string& s) {
    return fromBinaryContraint(s.data(), s.size());
}

inline std::string toBinary(const AckT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Ack::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline AckT fromBinaryAck(const void* data, size_t size) {
    (void) size;
    AckT result;
    auto* fb = flatbuffers::GetRoot<Ack>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline AckT fromBinaryAck(const std::string& s) {
    return fromBinaryAck(s.data(), s.size());
}

inline std::string toBinary(const QueryT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Query::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline QueryT fromBinaryQuery(const void* data, size_t size) {
    (void) size;
    QueryT result;
    auto* fb = flatbuffers::GetRoot<Query>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline QueryT fromBinaryQuery(const std::string& s) {
    return fromBinaryQuery(s.data(), s.size());
}

} // namespace pyramid::data_model::common::flatbuffers_codec
