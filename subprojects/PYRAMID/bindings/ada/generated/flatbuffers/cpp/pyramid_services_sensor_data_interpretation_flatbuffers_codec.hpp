// Auto-generated service FlatBuffers codec
// Backend: flatbuffers | Namespace: pyramid::services::sensor_data_interpretation::flatbuffers_codec
// Generated from proto service closure for pyramid.components.sensor_data_interpretation.services
#pragma once

#include "pyramid_data_model_types.hpp"
#include "pyramid_services_sensor_data_interpretation_generated.h"

#include <flatbuffers/flatbuffers.h>
#include <cstddef>
#include <string>
#include <vector>

namespace pyramid::services::sensor_data_interpretation::flatbuffers_codec {

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

std::string toBinary(const pyramid::domain_model::InterpretationRequirement& msg);
pyramid::domain_model::InterpretationRequirement fromBinaryInterpretationRequirement(const void* data, size_t size);
inline pyramid::domain_model::InterpretationRequirement fromBinaryInterpretationRequirement(const std::string& s) {
    return fromBinaryInterpretationRequirement(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::ObjectEvidenceProvisionRequirement& msg);
pyramid::domain_model::ObjectEvidenceProvisionRequirement fromBinaryObjectEvidenceProvisionRequirement(const void* data, size_t size);
inline pyramid::domain_model::ObjectEvidenceProvisionRequirement fromBinaryObjectEvidenceProvisionRequirement(const std::string& s) {
    return fromBinaryObjectEvidenceProvisionRequirement(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::ObjectAquisitionRequirement& msg);
pyramid::domain_model::ObjectAquisitionRequirement fromBinaryObjectAquisitionRequirement(const void* data, size_t size);
inline pyramid::domain_model::ObjectAquisitionRequirement fromBinaryObjectAquisitionRequirement(const std::string& s) {
    return fromBinaryObjectAquisitionRequirement(s.data(), s.size());
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

std::string toBinary(const std::vector<pyramid::domain_model::ObjectEvidenceProvisionRequirement>& msg);
std::vector<pyramid::domain_model::ObjectEvidenceProvisionRequirement> fromBinaryObjectEvidenceProvisionRequirementArray(const void* data, size_t size);
inline std::vector<pyramid::domain_model::ObjectEvidenceProvisionRequirement> fromBinaryObjectEvidenceProvisionRequirementArray(const std::string& s) {
    return fromBinaryObjectEvidenceProvisionRequirementArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::domain_model::ObjectAquisitionRequirement>& msg);
std::vector<pyramid::domain_model::ObjectAquisitionRequirement> fromBinaryObjectAquisitionRequirementArray(const void* data, size_t size);
inline std::vector<pyramid::domain_model::ObjectAquisitionRequirement> fromBinaryObjectAquisitionRequirementArray(const std::string& s) {
    return fromBinaryObjectAquisitionRequirementArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::domain_model::Capability>& msg);
std::vector<pyramid::domain_model::Capability> fromBinaryCapabilityArray(const void* data, size_t size);
inline std::vector<pyramid::domain_model::Capability> fromBinaryCapabilityArray(const std::string& s) {
    return fromBinaryCapabilityArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::domain_model::InterpretationRequirement>& msg);
std::vector<pyramid::domain_model::InterpretationRequirement> fromBinaryInterpretationRequirementArray(const void* data, size_t size);
inline std::vector<pyramid::domain_model::InterpretationRequirement> fromBinaryInterpretationRequirementArray(const std::string& s) {
    return fromBinaryInterpretationRequirementArray(s.data(), s.size());
}

} // namespace pyramid::services::sensor_data_interpretation::flatbuffers_codec
