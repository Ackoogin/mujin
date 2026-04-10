// Auto-generated service FlatBuffers codec
// Backend: flatbuffers | Namespace: pyramid::services::tactical_objects::flatbuffers_codec
// Generated from proto service closure for pyramid.components.tactical_objects.services
#pragma once

#include "pyramid_data_model_types.hpp"
#include "pyramid_services_tactical_objects_wire_types.hpp"
#include "pyramid_services_tactical_objects_generated.h"

#include <flatbuffers/flatbuffers.h>
#include <cstddef>
#include <string>
#include <vector>

namespace pyramid::services::tactical_objects::flatbuffers_codec {

namespace data_model = pyramid::data_model;
namespace wire_types = pyramid::services::tactical_objects::wire_types;

static constexpr const char* kContentType = "application/flatbuffers";

std::string toBinary(const pyramid::data_model::GeodeticPosition& msg);
pyramid::data_model::GeodeticPosition fromBinaryGeodeticPosition(const void* data, size_t size);
inline pyramid::data_model::GeodeticPosition fromBinaryGeodeticPosition(const std::string& s) {
    return fromBinaryGeodeticPosition(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::PolyArea& msg);
pyramid::data_model::PolyArea fromBinaryPolyArea(const void* data, size_t size);
inline pyramid::data_model::PolyArea fromBinaryPolyArea(const std::string& s) {
    return fromBinaryPolyArea(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::Achievement& msg);
pyramid::data_model::Achievement fromBinaryAchievement(const void* data, size_t size);
inline pyramid::data_model::Achievement fromBinaryAchievement(const std::string& s) {
    return fromBinaryAchievement(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::Entity& msg);
pyramid::data_model::Entity fromBinaryEntity(const void* data, size_t size);
inline pyramid::data_model::Entity fromBinaryEntity(const std::string& s) {
    return fromBinaryEntity(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::CircleArea& msg);
pyramid::data_model::CircleArea fromBinaryCircleArea(const void* data, size_t size);
inline pyramid::data_model::CircleArea fromBinaryCircleArea(const std::string& s) {
    return fromBinaryCircleArea(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::Point& msg);
pyramid::data_model::Point fromBinaryPoint(const void* data, size_t size);
inline pyramid::data_model::Point fromBinaryPoint(const std::string& s) {
    return fromBinaryPoint(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::Contraint& msg);
pyramid::data_model::Contraint fromBinaryContraint(const void* data, size_t size);
inline pyramid::data_model::Contraint fromBinaryContraint(const std::string& s) {
    return fromBinaryContraint(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::Ack& msg);
pyramid::data_model::Ack fromBinaryAck(const void* data, size_t size);
inline pyramid::data_model::Ack fromBinaryAck(const std::string& s) {
    return fromBinaryAck(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::Query& msg);
pyramid::data_model::Query fromBinaryQuery(const void* data, size_t size);
inline pyramid::data_model::Query fromBinaryQuery(const std::string& s) {
    return fromBinaryQuery(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::ObjectDetail& msg);
pyramid::data_model::ObjectDetail fromBinaryObjectDetail(const void* data, size_t size);
inline pyramid::data_model::ObjectDetail fromBinaryObjectDetail(const std::string& s) {
    return fromBinaryObjectDetail(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::ObjectEvidenceRequirement& msg);
pyramid::data_model::ObjectEvidenceRequirement fromBinaryObjectEvidenceRequirement(const void* data, size_t size);
inline pyramid::data_model::ObjectEvidenceRequirement fromBinaryObjectEvidenceRequirement(const std::string& s) {
    return fromBinaryObjectEvidenceRequirement(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::ObjectInterestRequirement& msg);
pyramid::data_model::ObjectInterestRequirement fromBinaryObjectInterestRequirement(const void* data, size_t size);
inline pyramid::data_model::ObjectInterestRequirement fromBinaryObjectInterestRequirement(const std::string& s) {
    return fromBinaryObjectInterestRequirement(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::ObjectMatch& msg);
pyramid::data_model::ObjectMatch fromBinaryObjectMatch(const void* data, size_t size);
inline pyramid::data_model::ObjectMatch fromBinaryObjectMatch(const std::string& s) {
    return fromBinaryObjectMatch(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::Capability& msg);
pyramid::data_model::Capability fromBinaryCapability(const void* data, size_t size);
inline pyramid::data_model::Capability fromBinaryCapability(const std::string& s) {
    return fromBinaryCapability(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::Identifier& msg);
pyramid::data_model::Identifier fromBinaryIdentifier(const void* data, size_t size);
inline pyramid::data_model::Identifier fromBinaryIdentifier(const std::string& s) {
    return fromBinaryIdentifier(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::data_model::ObjectDetail>& msg);
std::vector<pyramid::data_model::ObjectDetail> fromBinaryObjectDetailArray(const void* data, size_t size);
inline std::vector<pyramid::data_model::ObjectDetail> fromBinaryObjectDetailArray(const std::string& s) {
    return fromBinaryObjectDetailArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::data_model::ObjectEvidenceRequirement>& msg);
std::vector<pyramid::data_model::ObjectEvidenceRequirement> fromBinaryObjectEvidenceRequirementArray(const void* data, size_t size);
inline std::vector<pyramid::data_model::ObjectEvidenceRequirement> fromBinaryObjectEvidenceRequirementArray(const std::string& s) {
    return fromBinaryObjectEvidenceRequirementArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::data_model::Capability>& msg);
std::vector<pyramid::data_model::Capability> fromBinaryCapabilityArray(const void* data, size_t size);
inline std::vector<pyramid::data_model::Capability> fromBinaryCapabilityArray(const std::string& s) {
    return fromBinaryCapabilityArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::data_model::ObjectMatch>& msg);
std::vector<pyramid::data_model::ObjectMatch> fromBinaryObjectMatchArray(const void* data, size_t size);
inline std::vector<pyramid::data_model::ObjectMatch> fromBinaryObjectMatchArray(const std::string& s) {
    return fromBinaryObjectMatchArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::data_model::ObjectInterestRequirement>& msg);
std::vector<pyramid::data_model::ObjectInterestRequirement> fromBinaryObjectInterestRequirementArray(const void* data, size_t size);
inline std::vector<pyramid::data_model::ObjectInterestRequirement> fromBinaryObjectInterestRequirementArray(const std::string& s) {
    return fromBinaryObjectInterestRequirementArray(s.data(), s.size());
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
