// Auto-generated types header
// Generated from: autonomy.proto by generate_bindings.py (types)
// Namespace: pyramid::data_model::autonomy
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include "pyramid_data_model_base_types.hpp"
#include "pyramid_data_model_common_types.hpp"

namespace pyramid::data_model::autonomy {


enum class AutonomyBackendState : int {
    Unspecified = 0,
    Idle = 1,
    Ready = 2,
    WaitingForResults = 3,
    Complete = 4,
    Failed = 5,
    Stopped = 6,
};

enum class CommandStatus : int {
    Unspecified = 0,
    Pending = 1,
    Running = 2,
    Succeeded = 3,
    FailedTransient = 4,
    FailedPermanent = 5,
    Cancelled = 6,
};

enum class StopMode : int {
    Unspecified = 0,
    Drain = 1,
    Immediate = 2,
};

enum class FactAuthorityLevel : int {
    Unspecified = 0,
    Believed = 1,
    Confirmed = 2,
};

struct FactUpdate {
    std::string key = {};
    bool value = false;
    std::string source = {};
    FactAuthorityLevel authority = FactAuthorityLevel::Unspecified;
};

struct StateUpdate {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    std::vector<FactUpdate> fact_updates = {};
};

struct MissionIntent {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    std::vector<std::string> goal_fluents = {};
};

struct AgentState {
    std::string agent_id = {};
    std::string agent_type = {};
    bool available = false;
};

struct PolicyEnvelope {
    uint32_t max_replans = 0;
    bool enable_goal_dispatch = false;
};

struct Session {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    MissionIntent intent = {};
    PolicyEnvelope policy = {};
    std::vector<AgentState> available_agents = {};
};

struct Capabilities {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    std::string backend_id = {};
    bool supports_batch_planning = false;
    bool supports_external_command_dispatch = false;
    bool supports_replanning = false;
};

struct StringKeyValue {
    std::string key = {};
    std::string value = {};
};

struct Command {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    std::string command_id = {};
    std::string action_name = {};
    std::string signature = {};
    std::string service_name = {};
    std::string operation = {};
    std::vector<StringKeyValue> request_fields = {};
};

struct GoalDispatch {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    std::string dispatch_id = {};
    std::string agent_id = {};
    std::vector<std::string> goals = {};
};

struct DecisionRecord {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    std::string session_id = {};
    std::string backend_id = {};
    uint64_t world_version = 0;
    uint32_t replan_count = 0;
    bool plan_success = false;
    double solve_time_ms = 0.0;
    std::vector<std::string> planned_action_signatures = {};
    std::string compiled_bt_xml = {};
};

struct CommandResult {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string entity_source = {};  // from Entity  // optional
    std::string command_id = {};
    CommandStatus status = CommandStatus::Unspecified;
    std::vector<FactUpdate> observed_updates = {};
    std::string source = {};
};

struct DispatchResult {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string entity_source = {};  // from Entity  // optional
    std::string dispatch_id = {};
    CommandStatus status = CommandStatus::Unspecified;
    std::vector<FactUpdate> observed_updates = {};
    std::string source = {};
};

struct SessionSnapshot {
    std::optional<double> update_time;  // from Entity  // optional
    std::string id = {};  // from Entity  // optional
    std::string source = {};  // from Entity  // optional
    std::string session_id = {};
    AutonomyBackendState state = AutonomyBackendState::Unspecified;
    uint64_t world_version = 0;
    uint32_t replan_count = 0;
    std::vector<AgentState> agent_states = {};
    std::vector<Command> outstanding_commands = {};
    std::vector<GoalDispatch> outstanding_goal_dispatches = {};
    std::vector<DecisionRecord> decision_history = {};
};

struct SessionStepRequest {
    std::string session_id = {};
};

struct SessionStopRequest {
    std::string session_id = {};
    StopMode mode = StopMode::Unspecified;
};

} // namespace pyramid::data_model::autonomy
