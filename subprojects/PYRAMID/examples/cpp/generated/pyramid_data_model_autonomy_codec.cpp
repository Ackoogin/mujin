// Auto-generated data model JSON codec implementation
// Namespace: pyramid::data_model::autonomy

#include "pyramid_data_model_autonomy_codec.hpp"

#include <nlohmann/json.hpp>

#include "pyramid_data_model_base_codec.hpp"
#include "pyramid_data_model_common_codec.hpp"

namespace pyramid::data_model::autonomy {

std::string toString(AutonomyBackendState v) {
    switch (v) {
        case AutonomyBackendState::Unspecified: return "AUTONOMY_BACKEND_STATE_UNSPECIFIED";
        case AutonomyBackendState::Idle: return "AUTONOMY_BACKEND_STATE_IDLE";
        case AutonomyBackendState::Ready: return "AUTONOMY_BACKEND_STATE_READY";
        case AutonomyBackendState::WaitingForResults: return "AUTONOMY_BACKEND_STATE_WAITING_FOR_RESULTS";
        case AutonomyBackendState::Complete: return "AUTONOMY_BACKEND_STATE_COMPLETE";
        case AutonomyBackendState::Failed: return "AUTONOMY_BACKEND_STATE_FAILED";
        case AutonomyBackendState::Stopped: return "AUTONOMY_BACKEND_STATE_STOPPED";
    }
    return "AUTONOMY_BACKEND_STATE_UNSPECIFIED";
}

AutonomyBackendState autonomyBackendStateFromString(const std::string& s) {
    if (s == "AUTONOMY_BACKEND_STATE_UNSPECIFIED") return AutonomyBackendState::Unspecified;
    if (s == "AUTONOMY_BACKEND_STATE_IDLE") return AutonomyBackendState::Idle;
    if (s == "AUTONOMY_BACKEND_STATE_READY") return AutonomyBackendState::Ready;
    if (s == "AUTONOMY_BACKEND_STATE_WAITING_FOR_RESULTS") return AutonomyBackendState::WaitingForResults;
    if (s == "AUTONOMY_BACKEND_STATE_COMPLETE") return AutonomyBackendState::Complete;
    if (s == "AUTONOMY_BACKEND_STATE_FAILED") return AutonomyBackendState::Failed;
    if (s == "AUTONOMY_BACKEND_STATE_STOPPED") return AutonomyBackendState::Stopped;
    return AutonomyBackendState::Unspecified;
}

std::string toString(CommandStatus v) {
    switch (v) {
        case CommandStatus::Unspecified: return "COMMAND_STATUS_UNSPECIFIED";
        case CommandStatus::Pending: return "COMMAND_STATUS_PENDING";
        case CommandStatus::Running: return "COMMAND_STATUS_RUNNING";
        case CommandStatus::Succeeded: return "COMMAND_STATUS_SUCCEEDED";
        case CommandStatus::FailedTransient: return "COMMAND_STATUS_FAILED_TRANSIENT";
        case CommandStatus::FailedPermanent: return "COMMAND_STATUS_FAILED_PERMANENT";
        case CommandStatus::Cancelled: return "COMMAND_STATUS_CANCELLED";
    }
    return "COMMAND_STATUS_UNSPECIFIED";
}

CommandStatus commandStatusFromString(const std::string& s) {
    if (s == "COMMAND_STATUS_UNSPECIFIED") return CommandStatus::Unspecified;
    if (s == "COMMAND_STATUS_PENDING") return CommandStatus::Pending;
    if (s == "COMMAND_STATUS_RUNNING") return CommandStatus::Running;
    if (s == "COMMAND_STATUS_SUCCEEDED") return CommandStatus::Succeeded;
    if (s == "COMMAND_STATUS_FAILED_TRANSIENT") return CommandStatus::FailedTransient;
    if (s == "COMMAND_STATUS_FAILED_PERMANENT") return CommandStatus::FailedPermanent;
    if (s == "COMMAND_STATUS_CANCELLED") return CommandStatus::Cancelled;
    return CommandStatus::Unspecified;
}

std::string toString(StopMode v) {
    switch (v) {
        case StopMode::Unspecified: return "STOP_MODE_UNSPECIFIED";
        case StopMode::Drain: return "STOP_MODE_DRAIN";
        case StopMode::Immediate: return "STOP_MODE_IMMEDIATE";
    }
    return "STOP_MODE_UNSPECIFIED";
}

StopMode stopModeFromString(const std::string& s) {
    if (s == "STOP_MODE_UNSPECIFIED") return StopMode::Unspecified;
    if (s == "STOP_MODE_DRAIN") return StopMode::Drain;
    if (s == "STOP_MODE_IMMEDIATE") return StopMode::Immediate;
    return StopMode::Unspecified;
}

std::string toString(FactAuthorityLevel v) {
    switch (v) {
        case FactAuthorityLevel::Unspecified: return "FACT_AUTHORITY_LEVEL_UNSPECIFIED";
        case FactAuthorityLevel::Believed: return "FACT_AUTHORITY_LEVEL_BELIEVED";
        case FactAuthorityLevel::Confirmed: return "FACT_AUTHORITY_LEVEL_CONFIRMED";
    }
    return "FACT_AUTHORITY_LEVEL_UNSPECIFIED";
}

FactAuthorityLevel factAuthorityLevelFromString(const std::string& s) {
    if (s == "FACT_AUTHORITY_LEVEL_UNSPECIFIED") return FactAuthorityLevel::Unspecified;
    if (s == "FACT_AUTHORITY_LEVEL_BELIEVED") return FactAuthorityLevel::Believed;
    if (s == "FACT_AUTHORITY_LEVEL_CONFIRMED") return FactAuthorityLevel::Confirmed;
    return FactAuthorityLevel::Unspecified;
}

std::string toJson(const FactUpdate& msg) {
    nlohmann::json obj;
    obj["key"] = msg.key;
    obj["value"] = msg.value;
    obj["source"] = msg.source;
    obj["authority"] = toString(msg.authority);
    return obj.dump();
}

FactUpdate fromJson(const std::string& s, FactUpdate* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    FactUpdate msg;
    if (j.contains("key")) msg.key = j["key"].get<std::string>();
    if (j.contains("value")) msg.value = j["value"].get<bool>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("authority")) msg.authority = factAuthorityLevelFromString(j["authority"].get<std::string>());
    return msg;
}

std::string toJson(const StateUpdate& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.fact_updates) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["fact_updates"] = arr;
    }
    return obj.dump();
}

StateUpdate fromJson(const std::string& s, StateUpdate* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    StateUpdate msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("fact_updates")) {
        for (const auto& v : j["fact_updates"]) {
            msg.fact_updates.push_back(fromJson(v.dump(), static_cast<FactUpdate*>(nullptr)));
        }
    }
    return msg;
}

std::string toJson(const MissionIntent& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.goal_fluents) {
            arr.push_back(v);
        }
        obj["goal_fluents"] = arr;
    }
    return obj.dump();
}

MissionIntent fromJson(const std::string& s, MissionIntent* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MissionIntent msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("goal_fluents")) {
        for (const auto& v : j["goal_fluents"]) {
            msg.goal_fluents.push_back(v.get<std::string>());
        }
    }
    return msg;
}

std::string toJson(const AgentState& msg) {
    nlohmann::json obj;
    obj["agent_id"] = msg.agent_id;
    obj["agent_type"] = msg.agent_type;
    obj["available"] = msg.available;
    return obj.dump();
}

AgentState fromJson(const std::string& s, AgentState* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    AgentState msg;
    if (j.contains("agent_id")) msg.agent_id = j["agent_id"].get<std::string>();
    if (j.contains("agent_type")) msg.agent_type = j["agent_type"].get<std::string>();
    if (j.contains("available")) msg.available = j["available"].get<bool>();
    return msg;
}

std::string toJson(const PolicyEnvelope& msg) {
    nlohmann::json obj;
    obj["max_replans"] = msg.max_replans;
    obj["enable_goal_dispatch"] = msg.enable_goal_dispatch;
    return obj.dump();
}

PolicyEnvelope fromJson(const std::string& s, PolicyEnvelope* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    PolicyEnvelope msg;
    if (j.contains("max_replans")) msg.max_replans = j["max_replans"].get<uint32_t>();
    if (j.contains("enable_goal_dispatch")) msg.enable_goal_dispatch = j["enable_goal_dispatch"].get<bool>();
    return msg;
}

std::string toJson(const Session& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    obj["intent"] = nlohmann::json::parse(toJson(msg.intent));
    obj["policy"] = nlohmann::json::parse(toJson(msg.policy));
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.available_agents) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["available_agents"] = arr;
    }
    return obj.dump();
}

Session fromJson(const std::string& s, Session* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    Session msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("intent")) msg.intent = fromJson(j["intent"].dump(), static_cast<MissionIntent*>(nullptr));
    if (j.contains("policy")) msg.policy = fromJson(j["policy"].dump(), static_cast<PolicyEnvelope*>(nullptr));
    if (j.contains("available_agents")) {
        for (const auto& v : j["available_agents"]) {
            msg.available_agents.push_back(fromJson(v.dump(), static_cast<AgentState*>(nullptr)));
        }
    }
    return msg;
}

std::string toJson(const Capabilities& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    obj["backend_id"] = msg.backend_id;
    obj["supports_batch_planning"] = msg.supports_batch_planning;
    obj["supports_external_command_dispatch"] = msg.supports_external_command_dispatch;
    obj["supports_replanning"] = msg.supports_replanning;
    return obj.dump();
}

Capabilities fromJson(const std::string& s, Capabilities* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    Capabilities msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("backend_id")) msg.backend_id = j["backend_id"].get<std::string>();
    if (j.contains("supports_batch_planning")) msg.supports_batch_planning = j["supports_batch_planning"].get<bool>();
    if (j.contains("supports_external_command_dispatch")) msg.supports_external_command_dispatch = j["supports_external_command_dispatch"].get<bool>();
    if (j.contains("supports_replanning")) msg.supports_replanning = j["supports_replanning"].get<bool>();
    return msg;
}

std::string toJson(const StringKeyValue& msg) {
    nlohmann::json obj;
    obj["key"] = msg.key;
    obj["value"] = msg.value;
    return obj.dump();
}

StringKeyValue fromJson(const std::string& s, StringKeyValue* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    StringKeyValue msg;
    if (j.contains("key")) msg.key = j["key"].get<std::string>();
    if (j.contains("value")) msg.value = j["value"].get<std::string>();
    return msg;
}

std::string toJson(const Command& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    obj["command_id"] = msg.command_id;
    obj["action_name"] = msg.action_name;
    obj["signature"] = msg.signature;
    obj["service_name"] = msg.service_name;
    obj["operation"] = msg.operation;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.request_fields) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["request_fields"] = arr;
    }
    return obj.dump();
}

Command fromJson(const std::string& s, Command* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    Command msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("command_id")) msg.command_id = j["command_id"].get<std::string>();
    if (j.contains("action_name")) msg.action_name = j["action_name"].get<std::string>();
    if (j.contains("signature")) msg.signature = j["signature"].get<std::string>();
    if (j.contains("service_name")) msg.service_name = j["service_name"].get<std::string>();
    if (j.contains("operation")) msg.operation = j["operation"].get<std::string>();
    if (j.contains("request_fields")) {
        for (const auto& v : j["request_fields"]) {
            msg.request_fields.push_back(fromJson(v.dump(), static_cast<StringKeyValue*>(nullptr)));
        }
    }
    return msg;
}

std::string toJson(const GoalDispatch& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    obj["dispatch_id"] = msg.dispatch_id;
    obj["agent_id"] = msg.agent_id;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.goals) {
            arr.push_back(v);
        }
        obj["goals"] = arr;
    }
    return obj.dump();
}

GoalDispatch fromJson(const std::string& s, GoalDispatch* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    GoalDispatch msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("dispatch_id")) msg.dispatch_id = j["dispatch_id"].get<std::string>();
    if (j.contains("agent_id")) msg.agent_id = j["agent_id"].get<std::string>();
    if (j.contains("goals")) {
        for (const auto& v : j["goals"]) {
            msg.goals.push_back(v.get<std::string>());
        }
    }
    return msg;
}

std::string toJson(const DecisionRecord& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    obj["session_id"] = msg.session_id;
    obj["backend_id"] = msg.backend_id;
    obj["world_version"] = msg.world_version;
    obj["replan_count"] = msg.replan_count;
    obj["plan_success"] = msg.plan_success;
    obj["solve_time_ms"] = msg.solve_time_ms;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.planned_action_signatures) {
            arr.push_back(v);
        }
        obj["planned_action_signatures"] = arr;
    }
    obj["compiled_bt_xml"] = msg.compiled_bt_xml;
    return obj.dump();
}

DecisionRecord fromJson(const std::string& s, DecisionRecord* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    DecisionRecord msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("session_id")) msg.session_id = j["session_id"].get<std::string>();
    if (j.contains("backend_id")) msg.backend_id = j["backend_id"].get<std::string>();
    if (j.contains("world_version")) msg.world_version = j["world_version"].get<uint64_t>();
    if (j.contains("replan_count")) msg.replan_count = j["replan_count"].get<uint32_t>();
    if (j.contains("plan_success")) msg.plan_success = j["plan_success"].get<bool>();
    if (j.contains("solve_time_ms")) msg.solve_time_ms = j["solve_time_ms"].get<double>();
    if (j.contains("planned_action_signatures")) {
        for (const auto& v : j["planned_action_signatures"]) {
            msg.planned_action_signatures.push_back(v.get<std::string>());
        }
    }
    if (j.contains("compiled_bt_xml")) msg.compiled_bt_xml = j["compiled_bt_xml"].get<std::string>();
    return msg;
}

std::string toJson(const CommandResult& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["entity_source"] = msg.entity_source;
    obj["command_id"] = msg.command_id;
    obj["status"] = toString(msg.status);
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.observed_updates) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["observed_updates"] = arr;
    }
    obj["source"] = msg.source;
    return obj.dump();
}

CommandResult fromJson(const std::string& s, CommandResult* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    CommandResult msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("entity_source")) msg.entity_source = j["entity_source"].get<std::string>();
    if (j.contains("command_id")) msg.command_id = j["command_id"].get<std::string>();
    if (j.contains("status")) msg.status = commandStatusFromString(j["status"].get<std::string>());
    if (j.contains("observed_updates")) {
        for (const auto& v : j["observed_updates"]) {
            msg.observed_updates.push_back(fromJson(v.dump(), static_cast<FactUpdate*>(nullptr)));
        }
    }
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    return msg;
}

std::string toJson(const DispatchResult& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["entity_source"] = msg.entity_source;
    obj["dispatch_id"] = msg.dispatch_id;
    obj["status"] = toString(msg.status);
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.observed_updates) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["observed_updates"] = arr;
    }
    obj["source"] = msg.source;
    return obj.dump();
}

DispatchResult fromJson(const std::string& s, DispatchResult* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    DispatchResult msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("entity_source")) msg.entity_source = j["entity_source"].get<std::string>();
    if (j.contains("dispatch_id")) msg.dispatch_id = j["dispatch_id"].get<std::string>();
    if (j.contains("status")) msg.status = commandStatusFromString(j["status"].get<std::string>());
    if (j.contains("observed_updates")) {
        for (const auto& v : j["observed_updates"]) {
            msg.observed_updates.push_back(fromJson(v.dump(), static_cast<FactUpdate*>(nullptr)));
        }
    }
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    return msg;
}

std::string toJson(const SessionSnapshot& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    obj["session_id"] = msg.session_id;
    obj["state"] = toString(msg.state);
    obj["world_version"] = msg.world_version;
    obj["replan_count"] = msg.replan_count;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.agent_states) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["agent_states"] = arr;
    }
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.outstanding_commands) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["outstanding_commands"] = arr;
    }
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.outstanding_goal_dispatches) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["outstanding_goal_dispatches"] = arr;
    }
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.decision_history) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["decision_history"] = arr;
    }
    return obj.dump();
}

SessionSnapshot fromJson(const std::string& s, SessionSnapshot* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    SessionSnapshot msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("session_id")) msg.session_id = j["session_id"].get<std::string>();
    if (j.contains("state")) msg.state = autonomyBackendStateFromString(j["state"].get<std::string>());
    if (j.contains("world_version")) msg.world_version = j["world_version"].get<uint64_t>();
    if (j.contains("replan_count")) msg.replan_count = j["replan_count"].get<uint32_t>();
    if (j.contains("agent_states")) {
        for (const auto& v : j["agent_states"]) {
            msg.agent_states.push_back(fromJson(v.dump(), static_cast<AgentState*>(nullptr)));
        }
    }
    if (j.contains("outstanding_commands")) {
        for (const auto& v : j["outstanding_commands"]) {
            msg.outstanding_commands.push_back(fromJson(v.dump(), static_cast<Command*>(nullptr)));
        }
    }
    if (j.contains("outstanding_goal_dispatches")) {
        for (const auto& v : j["outstanding_goal_dispatches"]) {
            msg.outstanding_goal_dispatches.push_back(fromJson(v.dump(), static_cast<GoalDispatch*>(nullptr)));
        }
    }
    if (j.contains("decision_history")) {
        for (const auto& v : j["decision_history"]) {
            msg.decision_history.push_back(fromJson(v.dump(), static_cast<DecisionRecord*>(nullptr)));
        }
    }
    return msg;
}

std::string toJson(const SessionStepRequest& msg) {
    nlohmann::json obj;
    obj["session_id"] = msg.session_id;
    return obj.dump();
}

SessionStepRequest fromJson(const std::string& s, SessionStepRequest* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    SessionStepRequest msg;
    if (j.contains("session_id")) msg.session_id = j["session_id"].get<std::string>();
    return msg;
}

std::string toJson(const SessionStopRequest& msg) {
    nlohmann::json obj;
    obj["session_id"] = msg.session_id;
    obj["mode"] = toString(msg.mode);
    return obj.dump();
}

SessionStopRequest fromJson(const std::string& s, SessionStopRequest* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    SessionStopRequest msg;
    if (j.contains("session_id")) msg.session_id = j["session_id"].get<std::string>();
    if (j.contains("mode")) msg.mode = stopModeFromString(j["mode"].get<std::string>());
    return msg;
}

} // namespace pyramid::data_model::autonomy
