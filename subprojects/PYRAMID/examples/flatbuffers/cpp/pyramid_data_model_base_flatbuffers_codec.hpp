// Auto-generated FlatBuffers PCL codec — do not edit
// Backend: flatbuffers | Namespace: pyramid::data_model::base::flatbuffers_codec
#pragma once

// Include the flatc-generated header (run flatc on pyramid_data_model_base.fbs)
#include "pyramid_data_model_base_generated.h"

#include <flatbuffers/flatbuffers.h>
#include <string>
#include <vector>
#include <cstdint>

namespace pyramid::data_model::base::flatbuffers_codec {

static constexpr const char* kContentType = "application/flatbuffers";

/// Serialise Angle to a FlatBuffer binary blob.
inline std::string toBinary(const AngleT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Angle::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise Angle from a FlatBuffer binary blob.
inline AngleT fromBinaryAngle(const void* data, size_t size) {
    AngleT result;
    auto* fb = flatbuffers::GetRoot<Angle>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline AngleT fromBinaryAngle(const std::string& s) {
    return fromBinaryAngle(s.data(), s.size());
}

/// Serialise Length to a FlatBuffer binary blob.
inline std::string toBinary(const LengthT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Length::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise Length from a FlatBuffer binary blob.
inline LengthT fromBinaryLength(const void* data, size_t size) {
    LengthT result;
    auto* fb = flatbuffers::GetRoot<Length>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline LengthT fromBinaryLength(const std::string& s) {
    return fromBinaryLength(s.data(), s.size());
}

/// Serialise Timestamp to a FlatBuffer binary blob.
inline std::string toBinary(const TimestampT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Timestamp::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise Timestamp from a FlatBuffer binary blob.
inline TimestampT fromBinaryTimestamp(const void* data, size_t size) {
    TimestampT result;
    auto* fb = flatbuffers::GetRoot<Timestamp>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline TimestampT fromBinaryTimestamp(const std::string& s) {
    return fromBinaryTimestamp(s.data(), s.size());
}

/// Serialise Identifier to a FlatBuffer binary blob.
inline std::string toBinary(const IdentifierT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Identifier::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise Identifier from a FlatBuffer binary blob.
inline IdentifierT fromBinaryIdentifier(const void* data, size_t size) {
    IdentifierT result;
    auto* fb = flatbuffers::GetRoot<Identifier>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline IdentifierT fromBinaryIdentifier(const std::string& s) {
    return fromBinaryIdentifier(s.data(), s.size());
}

/// Serialise Speed to a FlatBuffer binary blob.
inline std::string toBinary(const SpeedT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Speed::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise Speed from a FlatBuffer binary blob.
inline SpeedT fromBinarySpeed(const void* data, size_t size) {
    SpeedT result;
    auto* fb = flatbuffers::GetRoot<Speed>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline SpeedT fromBinarySpeed(const std::string& s) {
    return fromBinarySpeed(s.data(), s.size());
}

/// Serialise Percentage to a FlatBuffer binary blob.
inline std::string toBinary(const PercentageT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Percentage::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(reinterpret_cast<const char*>(
        builder.GetBufferPointer()), builder.GetSize());
}

/// Deserialise Percentage from a FlatBuffer binary blob.
inline PercentageT fromBinaryPercentage(const void* data, size_t size) {
    PercentageT result;
    auto* fb = flatbuffers::GetRoot<Percentage>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline PercentageT fromBinaryPercentage(const std::string& s) {
    return fromBinaryPercentage(s.data(), s.size());
}

} // namespace pyramid::data_model::base::flatbuffers_codec
