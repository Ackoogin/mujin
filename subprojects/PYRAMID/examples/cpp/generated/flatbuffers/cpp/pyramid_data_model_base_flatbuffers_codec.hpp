// Auto-generated FlatBuffers PCL codec — do not edit
// Backend: flatbuffers | Namespace: pyramid::data_model::base::flatbuffers_codec
#pragma once

// Include the flatc-generated header (run flatc on pyramid_data_model_base.fbs)
#include "pyramid_data_model_base_generated.h"

#include <flatbuffers/flatbuffers.h>
#include <cstddef>
#include <cstdint>
#include <string>

namespace pyramid::data_model::base::flatbuffers_codec {

static constexpr const char* kContentType = "application/flatbuffers";

inline std::string toBinary(const AngleT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Angle::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline AngleT fromBinaryAngle(const void* data, size_t size) {
    (void) size;
    AngleT result;
    auto* fb = flatbuffers::GetRoot<Angle>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline AngleT fromBinaryAngle(const std::string& s) {
    return fromBinaryAngle(s.data(), s.size());
}

inline std::string toBinary(const LengthT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Length::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline LengthT fromBinaryLength(const void* data, size_t size) {
    (void) size;
    LengthT result;
    auto* fb = flatbuffers::GetRoot<Length>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline LengthT fromBinaryLength(const std::string& s) {
    return fromBinaryLength(s.data(), s.size());
}

inline std::string toBinary(const TimestampT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Timestamp::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline TimestampT fromBinaryTimestamp(const void* data, size_t size) {
    (void) size;
    TimestampT result;
    auto* fb = flatbuffers::GetRoot<Timestamp>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline TimestampT fromBinaryTimestamp(const std::string& s) {
    return fromBinaryTimestamp(s.data(), s.size());
}

inline std::string toBinary(const IdentifierT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Identifier::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline IdentifierT fromBinaryIdentifier(const void* data, size_t size) {
    (void) size;
    IdentifierT result;
    auto* fb = flatbuffers::GetRoot<Identifier>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline IdentifierT fromBinaryIdentifier(const std::string& s) {
    return fromBinaryIdentifier(s.data(), s.size());
}

inline std::string toBinary(const SpeedT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Speed::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline SpeedT fromBinarySpeed(const void* data, size_t size) {
    (void) size;
    SpeedT result;
    auto* fb = flatbuffers::GetRoot<Speed>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline SpeedT fromBinarySpeed(const std::string& s) {
    return fromBinarySpeed(s.data(), s.size());
}

inline std::string toBinary(const PercentageT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Percentage::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline PercentageT fromBinaryPercentage(const void* data, size_t size) {
    (void) size;
    PercentageT result;
    auto* fb = flatbuffers::GetRoot<Percentage>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline PercentageT fromBinaryPercentage(const std::string& s) {
    return fromBinaryPercentage(s.data(), s.size());
}

} // namespace pyramid::data_model::base::flatbuffers_codec
