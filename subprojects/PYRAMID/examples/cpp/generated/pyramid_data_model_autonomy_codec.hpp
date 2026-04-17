// Auto-generated data model JSON codec header
// Generated from: autonomy.proto by generate_bindings.py (codec)
// Namespace: pyramid::data_model::autonomy
#pragma once

#include "pyramid_data_model_autonomy_types.hpp"
#include <string>

namespace pyramid::data_model::autonomy {

// Enum string converters
std::string toString(AutonomyBackendState v);
AutonomyBackendState autonomyBackendStateFromString(const std::string& s);
std::string toString(CommandStatus v);
CommandStatus commandStatusFromString(const std::string& s);
std::string toString(StopMode v);
StopMode stopModeFromString(const std::string& s);
std::string toString(FactAuthorityLevel v);
FactAuthorityLevel factAuthorityLevelFromString(const std::string& s);

// JSON codec
std::string toJson(const FactUpdate& msg);
FactUpdate fromJson(const std::string& s, FactUpdate* /*tag*/ = nullptr);
std::string toJson(const StateUpdate& msg);
StateUpdate fromJson(const std::string& s, StateUpdate* /*tag*/ = nullptr);
std::string toJson(const MissionIntent& msg);
MissionIntent fromJson(const std::string& s, MissionIntent* /*tag*/ = nullptr);
std::string toJson(const AgentState& msg);
AgentState fromJson(const std::string& s, AgentState* /*tag*/ = nullptr);
std::string toJson(const PolicyEnvelope& msg);
PolicyEnvelope fromJson(const std::string& s, PolicyEnvelope* /*tag*/ = nullptr);
std::string toJson(const Session& msg);
Session fromJson(const std::string& s, Session* /*tag*/ = nullptr);
std::string toJson(const Capabilities& msg);
Capabilities fromJson(const std::string& s, Capabilities* /*tag*/ = nullptr);
std::string toJson(const StringKeyValue& msg);
StringKeyValue fromJson(const std::string& s, StringKeyValue* /*tag*/ = nullptr);
std::string toJson(const Command& msg);
Command fromJson(const std::string& s, Command* /*tag*/ = nullptr);
std::string toJson(const GoalDispatch& msg);
GoalDispatch fromJson(const std::string& s, GoalDispatch* /*tag*/ = nullptr);
std::string toJson(const DecisionRecord& msg);
DecisionRecord fromJson(const std::string& s, DecisionRecord* /*tag*/ = nullptr);
std::string toJson(const CommandResult& msg);
CommandResult fromJson(const std::string& s, CommandResult* /*tag*/ = nullptr);
std::string toJson(const DispatchResult& msg);
DispatchResult fromJson(const std::string& s, DispatchResult* /*tag*/ = nullptr);
std::string toJson(const SessionSnapshot& msg);
SessionSnapshot fromJson(const std::string& s, SessionSnapshot* /*tag*/ = nullptr);
std::string toJson(const SessionStepRequest& msg);
SessionStepRequest fromJson(const std::string& s, SessionStepRequest* /*tag*/ = nullptr);
std::string toJson(const SessionStopRequest& msg);
SessionStopRequest fromJson(const std::string& s, SessionStopRequest* /*tag*/ = nullptr);

} // namespace pyramid::data_model::autonomy
