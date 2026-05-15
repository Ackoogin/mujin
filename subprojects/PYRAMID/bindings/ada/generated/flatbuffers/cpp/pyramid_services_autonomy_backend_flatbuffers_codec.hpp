// Auto-generated service FlatBuffers codec
// Backend: flatbuffers | Namespace: pyramid::services::autonomy_backend::flatbuffers_codec
// Generated from proto service closure for pyramid.components.autonomy_backend.services
#pragma once

#include "pyramid_data_model_types.hpp"
#include "pyramid_services_autonomy_backend_generated.h"

#include <flatbuffers/flatbuffers.h>
#include <cstddef>
#include <string>
#include <vector>

namespace pyramid::services::autonomy_backend::flatbuffers_codec {

namespace data_model = pyramid::domain_model;

static constexpr const char* kContentType = "application/flatbuffers";

std::string toBinary(const pyramid::domain_model::RequirementReference& msg);
pyramid::domain_model::RequirementReference fromBinaryRequirementReference(const void* data, size_t size);
inline pyramid::domain_model::RequirementReference fromBinaryRequirementReference(const std::string& s) {
    return fromBinaryRequirementReference(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::AgentState& msg);
pyramid::domain_model::AgentState fromBinaryAgentState(const void* data, size_t size);
inline pyramid::domain_model::AgentState fromBinaryAgentState(const std::string& s) {
    return fromBinaryAgentState(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::PlanningPolicy& msg);
pyramid::domain_model::PlanningPolicy fromBinaryPlanningPolicy(const void* data, size_t size);
inline pyramid::domain_model::PlanningPolicy fromBinaryPlanningPolicy(const std::string& s) {
    return fromBinaryPlanningPolicy(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::PlanningGoal& msg);
pyramid::domain_model::PlanningGoal fromBinaryPlanningGoal(const void* data, size_t size);
inline pyramid::domain_model::PlanningGoal fromBinaryPlanningGoal(const std::string& s) {
    return fromBinaryPlanningGoal(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::ExecutionPolicy& msg);
pyramid::domain_model::ExecutionPolicy fromBinaryExecutionPolicy(const void* data, size_t size);
inline pyramid::domain_model::ExecutionPolicy fromBinaryExecutionPolicy(const std::string& s) {
    return fromBinaryExecutionPolicy(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::WorldFactUpdate& msg);
pyramid::domain_model::WorldFactUpdate fromBinaryWorldFactUpdate(const void* data, size_t size);
inline pyramid::domain_model::WorldFactUpdate fromBinaryWorldFactUpdate(const std::string& s) {
    return fromBinaryWorldFactUpdate(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::StateUpdate& msg);
pyramid::domain_model::StateUpdate fromBinaryStateUpdate(const void* data, size_t size);
inline pyramid::domain_model::StateUpdate fromBinaryStateUpdate(const std::string& s) {
    return fromBinaryStateUpdate(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::Capabilities& msg);
pyramid::domain_model::Capabilities fromBinaryCapabilities(const void* data, size_t size);
inline pyramid::domain_model::Capabilities fromBinaryCapabilities(const std::string& s) {
    return fromBinaryCapabilities(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::PlannedComponentInteraction& msg);
pyramid::domain_model::PlannedComponentInteraction fromBinaryPlannedComponentInteraction(const void* data, size_t size);
inline pyramid::domain_model::PlannedComponentInteraction fromBinaryPlannedComponentInteraction(const std::string& s) {
    return fromBinaryPlannedComponentInteraction(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::PlanStep& msg);
pyramid::domain_model::PlanStep fromBinaryPlanStep(const void* data, size_t size);
inline pyramid::domain_model::PlanStep fromBinaryPlanStep(const std::string& s) {
    return fromBinaryPlanStep(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::Plan& msg);
pyramid::domain_model::Plan fromBinaryPlan(const void* data, size_t size);
inline pyramid::domain_model::Plan fromBinaryPlan(const std::string& s) {
    return fromBinaryPlan(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::RequirementPlacement& msg);
pyramid::domain_model::RequirementPlacement fromBinaryRequirementPlacement(const void* data, size_t size);
inline pyramid::domain_model::RequirementPlacement fromBinaryRequirementPlacement(const std::string& s) {
    return fromBinaryRequirementPlacement(s.data(), s.size());
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

std::string toBinary(const pyramid::domain_model::PlanningRequirement& msg);
pyramid::domain_model::PlanningRequirement fromBinaryPlanningRequirement(const void* data, size_t size);
inline pyramid::domain_model::PlanningRequirement fromBinaryPlanningRequirement(const std::string& s) {
    return fromBinaryPlanningRequirement(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::ExecutionRequirement& msg);
pyramid::domain_model::ExecutionRequirement fromBinaryExecutionRequirement(const void* data, size_t size);
inline pyramid::domain_model::ExecutionRequirement fromBinaryExecutionRequirement(const std::string& s) {
    return fromBinaryExecutionRequirement(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::ExecutionRun& msg);
pyramid::domain_model::ExecutionRun fromBinaryExecutionRun(const void* data, size_t size);
inline pyramid::domain_model::ExecutionRun fromBinaryExecutionRun(const std::string& s) {
    return fromBinaryExecutionRun(s.data(), s.size());
}

std::string toBinary(const pyramid::domain_model::Identifier& msg);
pyramid::domain_model::Identifier fromBinaryIdentifier(const void* data, size_t size);
inline pyramid::domain_model::Identifier fromBinaryIdentifier(const std::string& s) {
    return fromBinaryIdentifier(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::domain_model::Capabilities>& msg);
std::vector<pyramid::domain_model::Capabilities> fromBinaryCapabilitiesArray(const void* data, size_t size);
inline std::vector<pyramid::domain_model::Capabilities> fromBinaryCapabilitiesArray(const std::string& s) {
    return fromBinaryCapabilitiesArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::domain_model::PlanningRequirement>& msg);
std::vector<pyramid::domain_model::PlanningRequirement> fromBinaryPlanningRequirementArray(const void* data, size_t size);
inline std::vector<pyramid::domain_model::PlanningRequirement> fromBinaryPlanningRequirementArray(const std::string& s) {
    return fromBinaryPlanningRequirementArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::domain_model::ExecutionRequirement>& msg);
std::vector<pyramid::domain_model::ExecutionRequirement> fromBinaryExecutionRequirementArray(const void* data, size_t size);
inline std::vector<pyramid::domain_model::ExecutionRequirement> fromBinaryExecutionRequirementArray(const std::string& s) {
    return fromBinaryExecutionRequirementArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::domain_model::Plan>& msg);
std::vector<pyramid::domain_model::Plan> fromBinaryPlanArray(const void* data, size_t size);
inline std::vector<pyramid::domain_model::Plan> fromBinaryPlanArray(const std::string& s) {
    return fromBinaryPlanArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::domain_model::ExecutionRun>& msg);
std::vector<pyramid::domain_model::ExecutionRun> fromBinaryExecutionRunArray(const void* data, size_t size);
inline std::vector<pyramid::domain_model::ExecutionRun> fromBinaryExecutionRunArray(const std::string& s) {
    return fromBinaryExecutionRunArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::domain_model::RequirementPlacement>& msg);
std::vector<pyramid::domain_model::RequirementPlacement> fromBinaryRequirementPlacementArray(const void* data, size_t size);
inline std::vector<pyramid::domain_model::RequirementPlacement> fromBinaryRequirementPlacementArray(const std::string& s) {
    return fromBinaryRequirementPlacementArray(s.data(), s.size());
}

} // namespace pyramid::services::autonomy_backend::flatbuffers_codec
