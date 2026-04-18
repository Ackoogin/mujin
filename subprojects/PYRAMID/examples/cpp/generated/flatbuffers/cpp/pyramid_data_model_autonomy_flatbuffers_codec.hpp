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

inline std::string toBinary(const FactUpdateT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = FactUpdate::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline FactUpdateT fromBinaryFactUpdate(const void* data, size_t size) {
    (void) size;
    FactUpdateT result;
    auto* fb = flatbuffers::GetRoot<FactUpdate>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline FactUpdateT fromBinaryFactUpdate(const std::string& s) {
    return fromBinaryFactUpdate(s.data(), s.size());
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

inline std::string toBinary(const MissionIntentT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = MissionIntent::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline MissionIntentT fromBinaryMissionIntent(const void* data, size_t size) {
    (void) size;
    MissionIntentT result;
    auto* fb = flatbuffers::GetRoot<MissionIntent>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline MissionIntentT fromBinaryMissionIntent(const std::string& s) {
    return fromBinaryMissionIntent(s.data(), s.size());
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

inline std::string toBinary(const PolicyEnvelopeT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = PolicyEnvelope::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline PolicyEnvelopeT fromBinaryPolicyEnvelope(const void* data, size_t size) {
    (void) size;
    PolicyEnvelopeT result;
    auto* fb = flatbuffers::GetRoot<PolicyEnvelope>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline PolicyEnvelopeT fromBinaryPolicyEnvelope(const std::string& s) {
    return fromBinaryPolicyEnvelope(s.data(), s.size());
}

inline std::string toBinary(const SessionT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Session::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline SessionT fromBinarySession(const void* data, size_t size) {
    (void) size;
    SessionT result;
    auto* fb = flatbuffers::GetRoot<Session>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline SessionT fromBinarySession(const std::string& s) {
    return fromBinarySession(s.data(), s.size());
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

inline std::string toBinary(const StringKeyValueT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = StringKeyValue::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline StringKeyValueT fromBinaryStringKeyValue(const void* data, size_t size) {
    (void) size;
    StringKeyValueT result;
    auto* fb = flatbuffers::GetRoot<StringKeyValue>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline StringKeyValueT fromBinaryStringKeyValue(const std::string& s) {
    return fromBinaryStringKeyValue(s.data(), s.size());
}

inline std::string toBinary(const CommandT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = Command::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline CommandT fromBinaryCommand(const void* data, size_t size) {
    (void) size;
    CommandT result;
    auto* fb = flatbuffers::GetRoot<Command>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline CommandT fromBinaryCommand(const std::string& s) {
    return fromBinaryCommand(s.data(), s.size());
}

inline std::string toBinary(const GoalDispatchT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = GoalDispatch::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline GoalDispatchT fromBinaryGoalDispatch(const void* data, size_t size) {
    (void) size;
    GoalDispatchT result;
    auto* fb = flatbuffers::GetRoot<GoalDispatch>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline GoalDispatchT fromBinaryGoalDispatch(const std::string& s) {
    return fromBinaryGoalDispatch(s.data(), s.size());
}

inline std::string toBinary(const DecisionRecordT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = DecisionRecord::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline DecisionRecordT fromBinaryDecisionRecord(const void* data, size_t size) {
    (void) size;
    DecisionRecordT result;
    auto* fb = flatbuffers::GetRoot<DecisionRecord>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline DecisionRecordT fromBinaryDecisionRecord(const std::string& s) {
    return fromBinaryDecisionRecord(s.data(), s.size());
}

inline std::string toBinary(const CommandResultT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = CommandResult::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline CommandResultT fromBinaryCommandResult(const void* data, size_t size) {
    (void) size;
    CommandResultT result;
    auto* fb = flatbuffers::GetRoot<CommandResult>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline CommandResultT fromBinaryCommandResult(const std::string& s) {
    return fromBinaryCommandResult(s.data(), s.size());
}

inline std::string toBinary(const DispatchResultT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = DispatchResult::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline DispatchResultT fromBinaryDispatchResult(const void* data, size_t size) {
    (void) size;
    DispatchResultT result;
    auto* fb = flatbuffers::GetRoot<DispatchResult>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline DispatchResultT fromBinaryDispatchResult(const std::string& s) {
    return fromBinaryDispatchResult(s.data(), s.size());
}

inline std::string toBinary(const SessionSnapshotT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = SessionSnapshot::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline SessionSnapshotT fromBinarySessionSnapshot(const void* data, size_t size) {
    (void) size;
    SessionSnapshotT result;
    auto* fb = flatbuffers::GetRoot<SessionSnapshot>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline SessionSnapshotT fromBinarySessionSnapshot(const std::string& s) {
    return fromBinarySessionSnapshot(s.data(), s.size());
}

inline std::string toBinary(const SessionStepRequestT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = SessionStepRequest::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline SessionStepRequestT fromBinarySessionStepRequest(const void* data, size_t size) {
    (void) size;
    SessionStepRequestT result;
    auto* fb = flatbuffers::GetRoot<SessionStepRequest>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline SessionStepRequestT fromBinarySessionStepRequest(const std::string& s) {
    return fromBinarySessionStepRequest(s.data(), s.size());
}

inline std::string toBinary(const SessionStopRequestT& obj) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto offset = SessionStopRequest::Pack(builder, &obj);
    builder.Finish(offset);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

inline SessionStopRequestT fromBinarySessionStopRequest(const void* data, size_t size) {
    (void) size;
    SessionStopRequestT result;
    auto* fb = flatbuffers::GetRoot<SessionStopRequest>(data);
    if (fb) fb->UnPackTo(&result);
    return result;
}

inline SessionStopRequestT fromBinarySessionStopRequest(const std::string& s) {
    return fromBinarySessionStopRequest(s.data(), s.size());
}

} // namespace pyramid::data_model::autonomy::flatbuffers_codec
