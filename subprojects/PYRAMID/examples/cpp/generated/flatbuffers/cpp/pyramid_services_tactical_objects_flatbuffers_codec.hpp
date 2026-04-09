// Auto-generated tactical service FlatBuffers codec
// Backend: flatbuffers | Namespace: pyramid::services::tactical_objects::flatbuffers_codec
// Generated from pim/json_schema.py
#pragma once

#include "pyramid_services_tactical_objects_wire_types.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::flatbuffers_codec {

namespace wire_types = pyramid::services::tactical_objects::wire_types;

static constexpr const char* kContentType = "application/flatbuffers";

std::string wrapPayload(const std::string& payload);
std::string unwrapPayload(const void* data, size_t size);
inline std::string unwrapPayload(const std::string& payload) {
    return unwrapPayload(payload.data(), payload.size());
}

std::string toBinary(const wire_types::CreateRequirementRequest& msg);
wire_types::CreateRequirementRequest fromBinaryCreateRequirementRequest(const void* data, size_t size);
inline wire_types::CreateRequirementRequest fromBinaryCreateRequirementRequest(const std::string& s) {
    return fromBinaryCreateRequirementRequest(s.data(), s.size());
}

std::string toBinary(const wire_types::CreateRequirementResponse& msg);
wire_types::CreateRequirementResponse fromBinaryCreateRequirementResponse(const void* data, size_t size);
inline wire_types::CreateRequirementResponse fromBinaryCreateRequirementResponse(const std::string& s) {
    return fromBinaryCreateRequirementResponse(s.data(), s.size());
}

std::string toBinary(const wire_types::EntityMatch& msg);
wire_types::EntityMatch fromBinaryEntityMatch(const void* data, size_t size);
inline wire_types::EntityMatch fromBinaryEntityMatch(const std::string& s) {
    return fromBinaryEntityMatch(s.data(), s.size());
}

std::string toBinary(const wire_types::EntityMatchArray& msg);
wire_types::EntityMatchArray fromBinaryEntityMatchArray(const void* data, size_t size);
inline wire_types::EntityMatchArray fromBinaryEntityMatchArray(const std::string& s) {
    return fromBinaryEntityMatchArray(s.data(), s.size());
}

std::string toBinary(const wire_types::ObjectEvidence& msg);
wire_types::ObjectEvidence fromBinaryObjectEvidence(const void* data, size_t size);
inline wire_types::ObjectEvidence fromBinaryObjectEvidence(const std::string& s) {
    return fromBinaryObjectEvidence(s.data(), s.size());
}

std::string toBinary(const wire_types::EvidenceRequirement& msg);
wire_types::EvidenceRequirement fromBinaryEvidenceRequirement(const void* data, size_t size);
inline wire_types::EvidenceRequirement fromBinaryEvidenceRequirement(const std::string& s) {
    return fromBinaryEvidenceRequirement(s.data(), s.size());
}

} // namespace pyramid::services::tactical_objects::flatbuffers_codec
