// Auto-generated service FlatBuffers codec
// Backend: flatbuffers | Namespace: pyramid::services::tactical_objects::flatbuffers_codec
// Generated from proto service closure for pyramid.components.tactical_objects.services
#pragma once

#include "pyramid_data_model_types.hpp"
#include "pyramid_services_tactical_objects_generated.h"

#include <flatbuffers/flatbuffers.h>
#include <cstddef>
#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::flatbuffers_codec {

namespace data_model = pyramid::domain_model;

static constexpr const char* kContentType = "application/flatbuffers";

std::string toBinary(const pyramid::domain_model::GeodeticPosition& msg);
pyramid::domain_model::GeodeticPosition fromBinaryGeodeticPosition(const void* data, size_t size);
inline pyramid::domain_model::GeodeticPosition fromBinaryGeodeticPosition(const std::string& s) {
    return fromBinaryGeodeticPosition(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::PolyArea& msg);
pyramid::domain_model::PolyArea fromBinaryPolyArea(const void* data, size_t size);
inline pyramid::domain_model::PolyArea fromBinaryPolyArea(const std::string& s) {
    return fromBinaryPolyArea(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::Achievement& msg);
pyramid::domain_model::Achievement fromBinaryAchievement(const void* data, size_t size);
inline pyramid::domain_model::Achievement fromBinaryAchievement(const std::string& s) {
    return fromBinaryAchievement(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::Entity& msg);
pyramid::domain_model::Entity fromBinaryEntity(const void* data, size_t size);
inline pyramid::domain_model::Entity fromBinaryEntity(const std::string& s) {
    return fromBinaryEntity(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::CircleArea& msg);
pyramid::domain_model::CircleArea fromBinaryCircleArea(const void* data, size_t size);
inline pyramid::domain_model::CircleArea fromBinaryCircleArea(const std::string& s) {
    return fromBinaryCircleArea(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::Point& msg);
pyramid::domain_model::Point fromBinaryPoint(const void* data, size_t size);
inline pyramid::domain_model::Point fromBinaryPoint(const std::string& s) {
    return fromBinaryPoint(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::Contraint& msg);
pyramid::domain_model::Contraint fromBinaryContraint(const void* data, size_t size);
inline pyramid::domain_model::Contraint fromBinaryContraint(const std::string& s) {
    return fromBinaryContraint(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::Ack& msg);
pyramid::domain_model::Ack fromBinaryAck(const void* data, size_t size);
inline pyramid::domain_model::Ack fromBinaryAck(const std::string& s) {
    return fromBinaryAck(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::Query& msg);
pyramid::domain_model::Query fromBinaryQuery(const void* data, size_t size);
inline pyramid::domain_model::Query fromBinaryQuery(const std::string& s) {
    return fromBinaryQuery(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::ObjectDetail& msg);
pyramid::domain_model::ObjectDetail fromBinaryObjectDetail(const void* data, size_t size);
inline pyramid::domain_model::ObjectDetail fromBinaryObjectDetail(const std::string& s) {
    return fromBinaryObjectDetail(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::ObjectEvidenceRequirement& msg);
pyramid::domain_model::ObjectEvidenceRequirement fromBinaryObjectEvidenceRequirement(const void* data, size_t size);
inline pyramid::domain_model::ObjectEvidenceRequirement fromBinaryObjectEvidenceRequirement(const std::string& s) {
    return fromBinaryObjectEvidenceRequirement(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::ObjectInterestRequirement& msg);
pyramid::domain_model::ObjectInterestRequirement fromBinaryObjectInterestRequirement(const void* data, size_t size);
inline pyramid::domain_model::ObjectInterestRequirement fromBinaryObjectInterestRequirement(const std::string& s) {
    return fromBinaryObjectInterestRequirement(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::ObjectMatch& msg);
pyramid::domain_model::ObjectMatch fromBinaryObjectMatch(const void* data, size_t size);
inline pyramid::domain_model::ObjectMatch fromBinaryObjectMatch(const std::string& s) {
    return fromBinaryObjectMatch(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::Capability& msg);
pyramid::domain_model::Capability fromBinaryCapability(const void* data, size_t size);
inline pyramid::domain_model::Capability fromBinaryCapability(const std::string& s) {
    return fromBinaryCapability(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::Identifier& msg);
pyramid::domain_model::Identifier fromBinaryIdentifier(const void* data, size_t size);
inline pyramid::domain_model::Identifier fromBinaryIdentifier(const std::string& s) {
    return fromBinaryIdentifier(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::domain_model::ObjectDetail>& msg);
std::vector<pyramid::domain_model::ObjectDetail> fromBinaryObjectDetailArray(const void* data, size_t size);
inline std::vector<pyramid::domain_model::ObjectDetail> fromBinaryObjectDetailArray(const std::string& s) {
    return fromBinaryObjectDetailArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::domain_model::ObjectEvidenceRequirement>& msg);
std::vector<pyramid::domain_model::ObjectEvidenceRequirement> fromBinaryObjectEvidenceRequirementArray(const void* data, size_t size);
inline std::vector<pyramid::domain_model::ObjectEvidenceRequirement> fromBinaryObjectEvidenceRequirementArray(const std::string& s) {
    return fromBinaryObjectEvidenceRequirementArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::domain_model::Capability>& msg);
std::vector<pyramid::domain_model::Capability> fromBinaryCapabilityArray(const void* data, size_t size);
inline std::vector<pyramid::domain_model::Capability> fromBinaryCapabilityArray(const std::string& s) {
    return fromBinaryCapabilityArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::domain_model::ObjectMatch>& msg);
std::vector<pyramid::domain_model::ObjectMatch> fromBinaryObjectMatchArray(const void* data, size_t size);
inline std::vector<pyramid::domain_model::ObjectMatch> fromBinaryObjectMatchArray(const std::string& s) {
    return fromBinaryObjectMatchArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::domain_model::ObjectInterestRequirement>& msg);
std::vector<pyramid::domain_model::ObjectInterestRequirement> fromBinaryObjectInterestRequirementArray(const void* data, size_t size);
inline std::vector<pyramid::domain_model::ObjectInterestRequirement> fromBinaryObjectInterestRequirementArray(const std::string& s) {
    return fromBinaryObjectInterestRequirementArray(s.data(), s.size());
}

} // namespace pyramid::services::tactical_objects::flatbuffers_codec
