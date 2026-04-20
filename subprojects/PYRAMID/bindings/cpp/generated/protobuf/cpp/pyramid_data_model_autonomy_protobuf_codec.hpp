// Auto-generated Protobuf PCL codec — do not edit
// Backend: protobuf | Namespace: pyramid::data_model::autonomy::protobuf_codec
//
// Wraps protoc-generated SerializeToString / ParseFromString
// into the PCL codec API surface (toBinary / fromBinary).
//
// Requires: protoc-generated pyramid/data_model/autonomy.pb.h
// Link with: libprotobuf
#pragma once

#include "pyramid/data_model/autonomy.pb.h"

#include <string>
#include <cstdint>

namespace pyramid::data_model::autonomy::protobuf_codec {

static constexpr const char* kContentType = "application/protobuf";

/// Serialise RequirementReference via protobuf.
inline std::string toBinary(const pyramid::data_model::autonomy::RequirementReference& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise RequirementReference from protobuf wire format.
inline pyramid::data_model::autonomy::RequirementReference fromBinaryRequirementReference(const void* data, size_t size) {
    pyramid::data_model::autonomy::RequirementReference result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::autonomy::RequirementReference fromBinaryRequirementReference(const std::string& s) {
    return fromBinaryRequirementReference(s.data(), s.size());
}

/// Serialise AgentState via protobuf.
inline std::string toBinary(const pyramid::data_model::autonomy::AgentState& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise AgentState from protobuf wire format.
inline pyramid::data_model::autonomy::AgentState fromBinaryAgentState(const void* data, size_t size) {
    pyramid::data_model::autonomy::AgentState result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::autonomy::AgentState fromBinaryAgentState(const std::string& s) {
    return fromBinaryAgentState(s.data(), s.size());
}

/// Serialise PlanningPolicy via protobuf.
inline std::string toBinary(const pyramid::data_model::autonomy::PlanningPolicy& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise PlanningPolicy from protobuf wire format.
inline pyramid::data_model::autonomy::PlanningPolicy fromBinaryPlanningPolicy(const void* data, size_t size) {
    pyramid::data_model::autonomy::PlanningPolicy result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::autonomy::PlanningPolicy fromBinaryPlanningPolicy(const std::string& s) {
    return fromBinaryPlanningPolicy(s.data(), s.size());
}

/// Serialise PlanningGoal via protobuf.
inline std::string toBinary(const pyramid::data_model::autonomy::PlanningGoal& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise PlanningGoal from protobuf wire format.
inline pyramid::data_model::autonomy::PlanningGoal fromBinaryPlanningGoal(const void* data, size_t size) {
    pyramid::data_model::autonomy::PlanningGoal result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::autonomy::PlanningGoal fromBinaryPlanningGoal(const std::string& s) {
    return fromBinaryPlanningGoal(s.data(), s.size());
}

/// Serialise PlanningExecutionRequirement via protobuf.
inline std::string toBinary(const pyramid::data_model::autonomy::PlanningExecutionRequirement& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise PlanningExecutionRequirement from protobuf wire format.
inline pyramid::data_model::autonomy::PlanningExecutionRequirement fromBinaryPlanningExecutionRequirement(const void* data, size_t size) {
    pyramid::data_model::autonomy::PlanningExecutionRequirement result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::autonomy::PlanningExecutionRequirement fromBinaryPlanningExecutionRequirement(const std::string& s) {
    return fromBinaryPlanningExecutionRequirement(s.data(), s.size());
}

/// Serialise WorldFactUpdate via protobuf.
inline std::string toBinary(const pyramid::data_model::autonomy::WorldFactUpdate& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise WorldFactUpdate from protobuf wire format.
inline pyramid::data_model::autonomy::WorldFactUpdate fromBinaryWorldFactUpdate(const void* data, size_t size) {
    pyramid::data_model::autonomy::WorldFactUpdate result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::autonomy::WorldFactUpdate fromBinaryWorldFactUpdate(const std::string& s) {
    return fromBinaryWorldFactUpdate(s.data(), s.size());
}

/// Serialise StateUpdate via protobuf.
inline std::string toBinary(const pyramid::data_model::autonomy::StateUpdate& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise StateUpdate from protobuf wire format.
inline pyramid::data_model::autonomy::StateUpdate fromBinaryStateUpdate(const void* data, size_t size) {
    pyramid::data_model::autonomy::StateUpdate result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::autonomy::StateUpdate fromBinaryStateUpdate(const std::string& s) {
    return fromBinaryStateUpdate(s.data(), s.size());
}

/// Serialise Capabilities via protobuf.
inline std::string toBinary(const pyramid::data_model::autonomy::Capabilities& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise Capabilities from protobuf wire format.
inline pyramid::data_model::autonomy::Capabilities fromBinaryCapabilities(const void* data, size_t size) {
    pyramid::data_model::autonomy::Capabilities result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::autonomy::Capabilities fromBinaryCapabilities(const std::string& s) {
    return fromBinaryCapabilities(s.data(), s.size());
}

/// Serialise PlannedComponentInteraction via protobuf.
inline std::string toBinary(const pyramid::data_model::autonomy::PlannedComponentInteraction& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise PlannedComponentInteraction from protobuf wire format.
inline pyramid::data_model::autonomy::PlannedComponentInteraction fromBinaryPlannedComponentInteraction(const void* data, size_t size) {
    pyramid::data_model::autonomy::PlannedComponentInteraction result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::autonomy::PlannedComponentInteraction fromBinaryPlannedComponentInteraction(const std::string& s) {
    return fromBinaryPlannedComponentInteraction(s.data(), s.size());
}

/// Serialise PlanStep via protobuf.
inline std::string toBinary(const pyramid::data_model::autonomy::PlanStep& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise PlanStep from protobuf wire format.
inline pyramid::data_model::autonomy::PlanStep fromBinaryPlanStep(const void* data, size_t size) {
    pyramid::data_model::autonomy::PlanStep result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::autonomy::PlanStep fromBinaryPlanStep(const std::string& s) {
    return fromBinaryPlanStep(s.data(), s.size());
}

/// Serialise Plan via protobuf.
inline std::string toBinary(const pyramid::data_model::autonomy::Plan& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise Plan from protobuf wire format.
inline pyramid::data_model::autonomy::Plan fromBinaryPlan(const void* data, size_t size) {
    pyramid::data_model::autonomy::Plan result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::autonomy::Plan fromBinaryPlan(const std::string& s) {
    return fromBinaryPlan(s.data(), s.size());
}

/// Serialise RequirementPlacement via protobuf.
inline std::string toBinary(const pyramid::data_model::autonomy::RequirementPlacement& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise RequirementPlacement from protobuf wire format.
inline pyramid::data_model::autonomy::RequirementPlacement fromBinaryRequirementPlacement(const void* data, size_t size) {
    pyramid::data_model::autonomy::RequirementPlacement result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::autonomy::RequirementPlacement fromBinaryRequirementPlacement(const std::string& s) {
    return fromBinaryRequirementPlacement(s.data(), s.size());
}

/// Serialise ExecutionRun via protobuf.
inline std::string toBinary(const pyramid::data_model::autonomy::ExecutionRun& msg) {
    std::string out;
    msg.SerializeToString(&out);
    return out;
}

/// Deserialise ExecutionRun from protobuf wire format.
inline pyramid::data_model::autonomy::ExecutionRun fromBinaryExecutionRun(const void* data, size_t size) {
    pyramid::data_model::autonomy::ExecutionRun result;
    result.ParseFromArray(data, static_cast<int>(size));
    return result;
}

inline pyramid::data_model::autonomy::ExecutionRun fromBinaryExecutionRun(const std::string& s) {
    return fromBinaryExecutionRun(s.data(), s.size());
}

} // namespace pyramid::data_model::autonomy::protobuf_codec
