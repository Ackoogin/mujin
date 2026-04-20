// Auto-generated FlatBuffers PCL codec — do not edit
// Backend: flatbuffers | Namespace: pyramid::data_model::autonomy::flatbuffers_codec
#pragma once

// Include the flatc-generated header (run flatc on pyramid_data_model_autonomy.fbs)
#include "pyramid_data_model_autonomy_generated.h"

#include <flatbuffers/flatbuffers.h>
#include <cstddef>
#include <cstdint>
#include <string>

namespace pyramid::data_model::autonomy::flatbuffers_codec {

static constexpr const char* kContentType = "application/flatbuffers";

inline std::string toBinary(const RequirementReferenceT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = RequirementReference::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline RequirementReferenceT fromBinaryRequirementReference(const void* data, size_t size) {
    (void) size;
    RequirementReferenceT result;
    auto* fb = flatbuffers::GetRoot<RequirementReference>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline RequirementReferenceT fromBinaryRequirementReference(const std::string& s) {
    return fromBinaryRequirementReference(s.data(), s.size());
}

inline std::string toBinary(const AgentStateT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = AgentState::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline AgentStateT fromBinaryAgentState(const void* data, size_t size) {
    (void) size;
    AgentStateT result;
    auto* fb = flatbuffers::GetRoot<AgentState>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline AgentStateT fromBinaryAgentState(const std::string& s) {
    return fromBinaryAgentState(s.data(), s.size());
}

inline std::string toBinary(const PlanningPolicyT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = PlanningPolicy::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline PlanningPolicyT fromBinaryPlanningPolicy(const void* data, size_t size) {
    (void) size;
    PlanningPolicyT result;
    auto* fb = flatbuffers::GetRoot<PlanningPolicy>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline PlanningPolicyT fromBinaryPlanningPolicy(const std::string& s) {
    return fromBinaryPlanningPolicy(s.data(), s.size());
}

inline std::string toBinary(const PlanningGoalT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = PlanningGoal::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline PlanningGoalT fromBinaryPlanningGoal(const void* data, size_t size) {
    (void) size;
    PlanningGoalT result;
    auto* fb = flatbuffers::GetRoot<PlanningGoal>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline PlanningGoalT fromBinaryPlanningGoal(const std::string& s) {
    return fromBinaryPlanningGoal(s.data(), s.size());
}

inline std::string toBinary(const PlanningExecutionRequirementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = PlanningExecutionRequirement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline PlanningExecutionRequirementT fromBinaryPlanningExecutionRequirement(const void* data, size_t size) {
    (void) size;
    PlanningExecutionRequirementT result;
    auto* fb = flatbuffers::GetRoot<PlanningExecutionRequirement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline PlanningExecutionRequirementT fromBinaryPlanningExecutionRequirement(const std::string& s) {
    return fromBinaryPlanningExecutionRequirement(s.data(), s.size());
}

inline std::string toBinary(const WorldFactUpdateT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = WorldFactUpdate::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline WorldFactUpdateT fromBinaryWorldFactUpdate(const void* data, size_t size) {
    (void) size;
    WorldFactUpdateT result;
    auto* fb = flatbuffers::GetRoot<WorldFactUpdate>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline WorldFactUpdateT fromBinaryWorldFactUpdate(const std::string& s) {
    return fromBinaryWorldFactUpdate(s.data(), s.size());
}

inline std::string toBinary(const StateUpdateT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = StateUpdate::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline StateUpdateT fromBinaryStateUpdate(const void* data, size_t size) {
    (void) size;
    StateUpdateT result;
    auto* fb = flatbuffers::GetRoot<StateUpdate>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline StateUpdateT fromBinaryStateUpdate(const std::string& s) {
    return fromBinaryStateUpdate(s.data(), s.size());
}

inline std::string toBinary(const CapabilitiesT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Capabilities::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline CapabilitiesT fromBinaryCapabilities(const void* data, size_t size) {
    (void) size;
    CapabilitiesT result;
    auto* fb = flatbuffers::GetRoot<Capabilities>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline CapabilitiesT fromBinaryCapabilities(const std::string& s) {
    return fromBinaryCapabilities(s.data(), s.size());
}

inline std::string toBinary(const PlannedComponentInteractionT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = PlannedComponentInteraction::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline PlannedComponentInteractionT fromBinaryPlannedComponentInteraction(const void* data, size_t size) {
    (void) size;
    PlannedComponentInteractionT result;
    auto* fb = flatbuffers::GetRoot<PlannedComponentInteraction>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline PlannedComponentInteractionT fromBinaryPlannedComponentInteraction(const std::string& s) {
    return fromBinaryPlannedComponentInteraction(s.data(), s.size());
}

inline std::string toBinary(const PlanStepT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = PlanStep::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline PlanStepT fromBinaryPlanStep(const void* data, size_t size) {
    (void) size;
    PlanStepT result;
    auto* fb = flatbuffers::GetRoot<PlanStep>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline PlanStepT fromBinaryPlanStep(const std::string& s) {
    return fromBinaryPlanStep(s.data(), s.size());
}

inline std::string toBinary(const PlanT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Plan::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline PlanT fromBinaryPlan(const void* data, size_t size) {
    (void) size;
    PlanT result;
    auto* fb = flatbuffers::GetRoot<Plan>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline PlanT fromBinaryPlan(const std::string& s) {
    return fromBinaryPlan(s.data(), s.size());
}

inline std::string toBinary(const RequirementPlacementT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = RequirementPlacement::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline RequirementPlacementT fromBinaryRequirementPlacement(const void* data, size_t size) {
    (void) size;
    RequirementPlacementT result;
    auto* fb = flatbuffers::GetRoot<RequirementPlacement>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline RequirementPlacementT fromBinaryRequirementPlacement(const std::string& s) {
    return fromBinaryRequirementPlacement(s.data(), s.size());
}

inline std::string toBinary(const ExecutionRunT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = ExecutionRun::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline ExecutionRunT fromBinaryExecutionRun(const void* data, size_t size) {
    (void) size;
    ExecutionRunT result;
    auto* fb = flatbuffers::GetRoot<ExecutionRun>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline ExecutionRunT fromBinaryExecutionRun(const std::string& s) {
    return fromBinaryExecutionRun(s.data(), s.size());
}

} // namespace pyramid::data_model::autonomy::flatbuffers_codec
