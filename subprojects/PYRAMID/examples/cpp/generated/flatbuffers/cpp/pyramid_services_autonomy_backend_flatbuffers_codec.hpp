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

namespace data_model = pyramid::data_model;

static constexpr const char* kContentType = "application/flatbuffers";

std::string toBinary(const pyramid::data_model::FactUpdate& msg);
pyramid::data_model::FactUpdate fromBinaryFactUpdate(const void* data, size_t size);
inline pyramid::data_model::FactUpdate fromBinaryFactUpdate(const std::string& s) {
    return fromBinaryFactUpdate(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::StateUpdate& msg);
pyramid::data_model::StateUpdate fromBinaryStateUpdate(const void* data, size_t size);
inline pyramid::data_model::StateUpdate fromBinaryStateUpdate(const std::string& s) {
    return fromBinaryStateUpdate(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::MissionIntent& msg);
pyramid::data_model::MissionIntent fromBinaryMissionIntent(const void* data, size_t size);
inline pyramid::data_model::MissionIntent fromBinaryMissionIntent(const std::string& s) {
    return fromBinaryMissionIntent(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::AgentState& msg);
pyramid::data_model::AgentState fromBinaryAgentState(const void* data, size_t size);
inline pyramid::data_model::AgentState fromBinaryAgentState(const std::string& s) {
    return fromBinaryAgentState(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::PolicyEnvelope& msg);
pyramid::data_model::PolicyEnvelope fromBinaryPolicyEnvelope(const void* data, size_t size);
inline pyramid::data_model::PolicyEnvelope fromBinaryPolicyEnvelope(const std::string& s) {
    return fromBinaryPolicyEnvelope(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::Session& msg);
pyramid::data_model::Session fromBinarySession(const void* data, size_t size);
inline pyramid::data_model::Session fromBinarySession(const std::string& s) {
    return fromBinarySession(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::Capabilities& msg);
pyramid::data_model::Capabilities fromBinaryCapabilities(const void* data, size_t size);
inline pyramid::data_model::Capabilities fromBinaryCapabilities(const std::string& s) {
    return fromBinaryCapabilities(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::StringKeyValue& msg);
pyramid::data_model::StringKeyValue fromBinaryStringKeyValue(const void* data, size_t size);
inline pyramid::data_model::StringKeyValue fromBinaryStringKeyValue(const std::string& s) {
    return fromBinaryStringKeyValue(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::Command& msg);
pyramid::data_model::Command fromBinaryCommand(const void* data, size_t size);
inline pyramid::data_model::Command fromBinaryCommand(const std::string& s) {
    return fromBinaryCommand(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::GoalDispatch& msg);
pyramid::data_model::GoalDispatch fromBinaryGoalDispatch(const void* data, size_t size);
inline pyramid::data_model::GoalDispatch fromBinaryGoalDispatch(const std::string& s) {
    return fromBinaryGoalDispatch(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::DecisionRecord& msg);
pyramid::data_model::DecisionRecord fromBinaryDecisionRecord(const void* data, size_t size);
inline pyramid::data_model::DecisionRecord fromBinaryDecisionRecord(const std::string& s) {
    return fromBinaryDecisionRecord(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::CommandResult& msg);
pyramid::data_model::CommandResult fromBinaryCommandResult(const void* data, size_t size);
inline pyramid::data_model::CommandResult fromBinaryCommandResult(const std::string& s) {
    return fromBinaryCommandResult(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::DispatchResult& msg);
pyramid::data_model::DispatchResult fromBinaryDispatchResult(const void* data, size_t size);
inline pyramid::data_model::DispatchResult fromBinaryDispatchResult(const std::string& s) {
    return fromBinaryDispatchResult(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::SessionSnapshot& msg);
pyramid::data_model::SessionSnapshot fromBinarySessionSnapshot(const void* data, size_t size);
inline pyramid::data_model::SessionSnapshot fromBinarySessionSnapshot(const std::string& s) {
    return fromBinarySessionSnapshot(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::SessionStepRequest& msg);
pyramid::data_model::SessionStepRequest fromBinarySessionStepRequest(const void* data, size_t size);
inline pyramid::data_model::SessionStepRequest fromBinarySessionStepRequest(const std::string& s) {
    return fromBinarySessionStepRequest(s.data(), s.size());
}

std::string toBinary(const pyramid::data_model::SessionStopRequest& msg);
pyramid::data_model::SessionStopRequest fromBinarySessionStopRequest(const void* data, size_t size);
inline pyramid::data_model::SessionStopRequest fromBinarySessionStopRequest(const std::string& s) {
    return fromBinarySessionStopRequest(s.data(), s.size());
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

std::string toBinary(const pyramid::data_model::Identifier& msg);
pyramid::data_model::Identifier fromBinaryIdentifier(const void* data, size_t size);
inline pyramid::data_model::Identifier fromBinaryIdentifier(const std::string& s) {
    return fromBinaryIdentifier(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::data_model::Capabilities>& msg);
std::vector<pyramid::data_model::Capabilities> fromBinaryCapabilitiesArray(const void* data, size_t size);
inline std::vector<pyramid::data_model::Capabilities> fromBinaryCapabilitiesArray(const std::string& s) {
    return fromBinaryCapabilitiesArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::data_model::SessionSnapshot>& msg);
std::vector<pyramid::data_model::SessionSnapshot> fromBinarySessionSnapshotArray(const void* data, size_t size);
inline std::vector<pyramid::data_model::SessionSnapshot> fromBinarySessionSnapshotArray(const std::string& s) {
    return fromBinarySessionSnapshotArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::data_model::Command>& msg);
std::vector<pyramid::data_model::Command> fromBinaryCommandArray(const void* data, size_t size);
inline std::vector<pyramid::data_model::Command> fromBinaryCommandArray(const std::string& s) {
    return fromBinaryCommandArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::data_model::GoalDispatch>& msg);
std::vector<pyramid::data_model::GoalDispatch> fromBinaryGoalDispatchArray(const void* data, size_t size);
inline std::vector<pyramid::data_model::GoalDispatch> fromBinaryGoalDispatchArray(const std::string& s) {
    return fromBinaryGoalDispatchArray(s.data(), s.size());
}

std::string toBinary(const std::vector<pyramid::data_model::DecisionRecord>& msg);
std::vector<pyramid::data_model::DecisionRecord> fromBinaryDecisionRecordArray(const void* data, size_t size);
inline std::vector<pyramid::data_model::DecisionRecord> fromBinaryDecisionRecordArray(const std::string& s) {
    return fromBinaryDecisionRecordArray(s.data(), s.size());
}

} // namespace pyramid::services::autonomy_backend::flatbuffers_codec
