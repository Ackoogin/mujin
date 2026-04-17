// Auto-generated service FlatBuffers codec
#include "pyramid_services_autonomy_backend_flatbuffers_codec.hpp"

#include "pyramid_data_model_common_codec.hpp"
#include "pyramid_data_model_tactical_codec.hpp"
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <memory>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <utility>

namespace pyramid::services::autonomy_backend::flatbuffers_codec {

namespace fbs = pyramid::services::autonomy_backend;

namespace {

template <typename OffsetT>
std::string finish_buffer(flatbuffers::FlatBufferBuilder& builder, OffsetT root) {
    builder.Finish(root);
    return std::string(
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize());
}

template <typename TableT>
const TableT* verified_root(const void* data, size_t size, const char* type_name) {
    if (data == nullptr || size == 0) {
        throw std::runtime_error(std::string("empty ") + type_name + " flatbuffer");
    }
    auto* bytes = reinterpret_cast<const std::uint8_t*>(data);
    flatbuffers::Verifier verifier(bytes, size);
    if (!verifier.VerifyBuffer<TableT>()) {
        throw std::runtime_error(std::string("invalid ") + type_name + " flatbuffer");
    }
    return flatbuffers::GetRoot<TableT>(bytes);
}

fbs::FactUpdateT to_fb(const pyramid::data_model::FactUpdate& msg) {
    fbs::FactUpdateT out{};
    out.key = msg.key;
    out.value = msg.value;
    out.source = msg.source;
    out.authority = static_cast<fbs::FactAuthorityLevel>(msg.authority);
    return out;
}

pyramid::data_model::FactUpdate from_fb(const fbs::FactUpdateT& msg, pyramid::data_model::FactUpdate* /*tag*/) {
    pyramid::data_model::FactUpdate out{};
    out.key = msg.key;
    out.value = msg.value;
    out.source = msg.source;
    out.authority = static_cast<pyramid::data_model::FactAuthorityLevel>(msg.authority);
    return out;
}

fbs::StateUpdateT to_fb(const pyramid::data_model::StateUpdate& msg) {
    fbs::StateUpdateT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.fact_updates.reserve(msg.fact_updates.size());
    for (const auto& item : msg.fact_updates) {
        out.fact_updates.emplace_back(std::make_unique<fbs::FactUpdateT>(to_fb(item)));
    }
    return out;
}

pyramid::data_model::StateUpdate from_fb(const fbs::StateUpdateT& msg, pyramid::data_model::StateUpdate* /*tag*/) {
    pyramid::data_model::StateUpdate out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    out.fact_updates.reserve(msg.fact_updates.size());
    for (const auto& item : msg.fact_updates) {
        if (item) {
            out.fact_updates.push_back(from_fb(*item, static_cast<pyramid::data_model::FactUpdate*>(nullptr)));
        }
    }
    return out;
}

fbs::MissionIntentT to_fb(const pyramid::data_model::MissionIntent& msg) {
    fbs::MissionIntentT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.goal_fluents = msg.goal_fluents;
    return out;
}

pyramid::data_model::MissionIntent from_fb(const fbs::MissionIntentT& msg, pyramid::data_model::MissionIntent* /*tag*/) {
    pyramid::data_model::MissionIntent out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    out.goal_fluents = msg.goal_fluents;
    return out;
}

fbs::AgentStateT to_fb(const pyramid::data_model::AgentState& msg) {
    fbs::AgentStateT out{};
    out.agent_id = msg.agent_id;
    out.agent_type = msg.agent_type;
    out.available = msg.available;
    return out;
}

pyramid::data_model::AgentState from_fb(const fbs::AgentStateT& msg, pyramid::data_model::AgentState* /*tag*/) {
    pyramid::data_model::AgentState out{};
    out.agent_id = msg.agent_id;
    out.agent_type = msg.agent_type;
    out.available = msg.available;
    return out;
}

fbs::PolicyEnvelopeT to_fb(const pyramid::data_model::PolicyEnvelope& msg) {
    fbs::PolicyEnvelopeT out{};
    out.max_replans = msg.max_replans;
    out.enable_goal_dispatch = msg.enable_goal_dispatch;
    return out;
}

pyramid::data_model::PolicyEnvelope from_fb(const fbs::PolicyEnvelopeT& msg, pyramid::data_model::PolicyEnvelope* /*tag*/) {
    pyramid::data_model::PolicyEnvelope out{};
    out.max_replans = msg.max_replans;
    out.enable_goal_dispatch = msg.enable_goal_dispatch;
    return out;
}

fbs::SessionT to_fb(const pyramid::data_model::Session& msg) {
    fbs::SessionT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.intent = std::make_unique<fbs::MissionIntentT>(to_fb(msg.intent));
    out.policy = std::make_unique<fbs::PolicyEnvelopeT>(to_fb(msg.policy));
    out.available_agents.reserve(msg.available_agents.size());
    for (const auto& item : msg.available_agents) {
        out.available_agents.emplace_back(std::make_unique<fbs::AgentStateT>(to_fb(item)));
    }
    return out;
}

pyramid::data_model::Session from_fb(const fbs::SessionT& msg, pyramid::data_model::Session* /*tag*/) {
    pyramid::data_model::Session out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    if (msg.intent) out.intent = from_fb(*msg.intent, static_cast<pyramid::data_model::MissionIntent*>(nullptr));
    if (msg.policy) out.policy = from_fb(*msg.policy, static_cast<pyramid::data_model::PolicyEnvelope*>(nullptr));
    out.available_agents.reserve(msg.available_agents.size());
    for (const auto& item : msg.available_agents) {
        if (item) {
            out.available_agents.push_back(from_fb(*item, static_cast<pyramid::data_model::AgentState*>(nullptr)));
        }
    }
    return out;
}

fbs::CapabilitiesT to_fb(const pyramid::data_model::Capabilities& msg) {
    fbs::CapabilitiesT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.backend_id = msg.backend_id;
    out.supports_batch_planning = msg.supports_batch_planning;
    out.supports_external_command_dispatch = msg.supports_external_command_dispatch;
    out.supports_replanning = msg.supports_replanning;
    return out;
}

pyramid::data_model::Capabilities from_fb(const fbs::CapabilitiesT& msg, pyramid::data_model::Capabilities* /*tag*/) {
    pyramid::data_model::Capabilities out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    out.backend_id = msg.backend_id;
    out.supports_batch_planning = msg.supports_batch_planning;
    out.supports_external_command_dispatch = msg.supports_external_command_dispatch;
    out.supports_replanning = msg.supports_replanning;
    return out;
}

fbs::StringKeyValueT to_fb(const pyramid::data_model::StringKeyValue& msg) {
    fbs::StringKeyValueT out{};
    out.key = msg.key;
    out.value = msg.value;
    return out;
}

pyramid::data_model::StringKeyValue from_fb(const fbs::StringKeyValueT& msg, pyramid::data_model::StringKeyValue* /*tag*/) {
    pyramid::data_model::StringKeyValue out{};
    out.key = msg.key;
    out.value = msg.value;
    return out;
}

fbs::CommandT to_fb(const pyramid::data_model::Command& msg) {
    fbs::CommandT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.command_id = msg.command_id;
    out.action_name = msg.action_name;
    out.signature = msg.signature;
    out.service_name = msg.service_name;
    out.operation = msg.operation;
    out.request_fields.reserve(msg.request_fields.size());
    for (const auto& item : msg.request_fields) {
        out.request_fields.emplace_back(std::make_unique<fbs::StringKeyValueT>(to_fb(item)));
    }
    return out;
}

pyramid::data_model::Command from_fb(const fbs::CommandT& msg, pyramid::data_model::Command* /*tag*/) {
    pyramid::data_model::Command out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    out.command_id = msg.command_id;
    out.action_name = msg.action_name;
    out.signature = msg.signature;
    out.service_name = msg.service_name;
    out.operation = msg.operation;
    out.request_fields.reserve(msg.request_fields.size());
    for (const auto& item : msg.request_fields) {
        if (item) {
            out.request_fields.push_back(from_fb(*item, static_cast<pyramid::data_model::StringKeyValue*>(nullptr)));
        }
    }
    return out;
}

fbs::GoalDispatchT to_fb(const pyramid::data_model::GoalDispatch& msg) {
    fbs::GoalDispatchT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.dispatch_id = msg.dispatch_id;
    out.agent_id = msg.agent_id;
    out.goals = msg.goals;
    return out;
}

pyramid::data_model::GoalDispatch from_fb(const fbs::GoalDispatchT& msg, pyramid::data_model::GoalDispatch* /*tag*/) {
    pyramid::data_model::GoalDispatch out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    out.dispatch_id = msg.dispatch_id;
    out.agent_id = msg.agent_id;
    out.goals = msg.goals;
    return out;
}

fbs::DecisionRecordT to_fb(const pyramid::data_model::DecisionRecord& msg) {
    fbs::DecisionRecordT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.session_id = msg.session_id;
    out.backend_id = msg.backend_id;
    out.world_version = msg.world_version;
    out.replan_count = msg.replan_count;
    out.plan_success = msg.plan_success;
    out.solve_time_ms = msg.solve_time_ms;
    out.planned_action_signatures = msg.planned_action_signatures;
    out.compiled_bt_xml = msg.compiled_bt_xml;
    return out;
}

pyramid::data_model::DecisionRecord from_fb(const fbs::DecisionRecordT& msg, pyramid::data_model::DecisionRecord* /*tag*/) {
    pyramid::data_model::DecisionRecord out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    out.session_id = msg.session_id;
    out.backend_id = msg.backend_id;
    out.world_version = msg.world_version;
    out.replan_count = msg.replan_count;
    out.plan_success = msg.plan_success;
    out.solve_time_ms = msg.solve_time_ms;
    out.planned_action_signatures = msg.planned_action_signatures;
    out.compiled_bt_xml = msg.compiled_bt_xml;
    return out;
}

fbs::CommandResultT to_fb(const pyramid::data_model::CommandResult& msg) {
    fbs::CommandResultT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.entity_source = msg.entity_source;
    out.command_id = msg.command_id;
    out.status = static_cast<fbs::CommandStatus>(msg.status);
    out.observed_updates.reserve(msg.observed_updates.size());
    for (const auto& item : msg.observed_updates) {
        out.observed_updates.emplace_back(std::make_unique<fbs::FactUpdateT>(to_fb(item)));
    }
    out.source = msg.source;
    return out;
}

pyramid::data_model::CommandResult from_fb(const fbs::CommandResultT& msg, pyramid::data_model::CommandResult* /*tag*/) {
    pyramid::data_model::CommandResult out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.entity_source = msg.entity_source;
    out.command_id = msg.command_id;
    out.status = static_cast<pyramid::data_model::CommandStatus>(msg.status);
    out.observed_updates.reserve(msg.observed_updates.size());
    for (const auto& item : msg.observed_updates) {
        if (item) {
            out.observed_updates.push_back(from_fb(*item, static_cast<pyramid::data_model::FactUpdate*>(nullptr)));
        }
    }
    out.source = msg.source;
    return out;
}

fbs::DispatchResultT to_fb(const pyramid::data_model::DispatchResult& msg) {
    fbs::DispatchResultT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.entity_source = msg.entity_source;
    out.dispatch_id = msg.dispatch_id;
    out.status = static_cast<fbs::CommandStatus>(msg.status);
    out.observed_updates.reserve(msg.observed_updates.size());
    for (const auto& item : msg.observed_updates) {
        out.observed_updates.emplace_back(std::make_unique<fbs::FactUpdateT>(to_fb(item)));
    }
    out.source = msg.source;
    return out;
}

pyramid::data_model::DispatchResult from_fb(const fbs::DispatchResultT& msg, pyramid::data_model::DispatchResult* /*tag*/) {
    pyramid::data_model::DispatchResult out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.entity_source = msg.entity_source;
    out.dispatch_id = msg.dispatch_id;
    out.status = static_cast<pyramid::data_model::CommandStatus>(msg.status);
    out.observed_updates.reserve(msg.observed_updates.size());
    for (const auto& item : msg.observed_updates) {
        if (item) {
            out.observed_updates.push_back(from_fb(*item, static_cast<pyramid::data_model::FactUpdate*>(nullptr)));
        }
    }
    out.source = msg.source;
    return out;
}

fbs::SessionSnapshotT to_fb(const pyramid::data_model::SessionSnapshot& msg) {
    fbs::SessionSnapshotT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.session_id = msg.session_id;
    out.state = static_cast<fbs::AutonomyBackendState>(msg.state);
    out.world_version = msg.world_version;
    out.replan_count = msg.replan_count;
    out.agent_states.reserve(msg.agent_states.size());
    for (const auto& item : msg.agent_states) {
        out.agent_states.emplace_back(std::make_unique<fbs::AgentStateT>(to_fb(item)));
    }
    out.outstanding_commands.reserve(msg.outstanding_commands.size());
    for (const auto& item : msg.outstanding_commands) {
        out.outstanding_commands.emplace_back(std::make_unique<fbs::CommandT>(to_fb(item)));
    }
    out.outstanding_goal_dispatches.reserve(msg.outstanding_goal_dispatches.size());
    for (const auto& item : msg.outstanding_goal_dispatches) {
        out.outstanding_goal_dispatches.emplace_back(std::make_unique<fbs::GoalDispatchT>(to_fb(item)));
    }
    out.decision_history.reserve(msg.decision_history.size());
    for (const auto& item : msg.decision_history) {
        out.decision_history.emplace_back(std::make_unique<fbs::DecisionRecordT>(to_fb(item)));
    }
    return out;
}

pyramid::data_model::SessionSnapshot from_fb(const fbs::SessionSnapshotT& msg, pyramid::data_model::SessionSnapshot* /*tag*/) {
    pyramid::data_model::SessionSnapshot out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    out.session_id = msg.session_id;
    out.state = static_cast<pyramid::data_model::AutonomyBackendState>(msg.state);
    out.world_version = msg.world_version;
    out.replan_count = msg.replan_count;
    out.agent_states.reserve(msg.agent_states.size());
    for (const auto& item : msg.agent_states) {
        if (item) {
            out.agent_states.push_back(from_fb(*item, static_cast<pyramid::data_model::AgentState*>(nullptr)));
        }
    }
    out.outstanding_commands.reserve(msg.outstanding_commands.size());
    for (const auto& item : msg.outstanding_commands) {
        if (item) {
            out.outstanding_commands.push_back(from_fb(*item, static_cast<pyramid::data_model::Command*>(nullptr)));
        }
    }
    out.outstanding_goal_dispatches.reserve(msg.outstanding_goal_dispatches.size());
    for (const auto& item : msg.outstanding_goal_dispatches) {
        if (item) {
            out.outstanding_goal_dispatches.push_back(from_fb(*item, static_cast<pyramid::data_model::GoalDispatch*>(nullptr)));
        }
    }
    out.decision_history.reserve(msg.decision_history.size());
    for (const auto& item : msg.decision_history) {
        if (item) {
            out.decision_history.push_back(from_fb(*item, static_cast<pyramid::data_model::DecisionRecord*>(nullptr)));
        }
    }
    return out;
}

fbs::SessionStepRequestT to_fb(const pyramid::data_model::SessionStepRequest& msg) {
    fbs::SessionStepRequestT out{};
    out.session_id = msg.session_id;
    return out;
}

pyramid::data_model::SessionStepRequest from_fb(const fbs::SessionStepRequestT& msg, pyramid::data_model::SessionStepRequest* /*tag*/) {
    pyramid::data_model::SessionStepRequest out{};
    out.session_id = msg.session_id;
    return out;
}

fbs::SessionStopRequestT to_fb(const pyramid::data_model::SessionStopRequest& msg) {
    fbs::SessionStopRequestT out{};
    out.session_id = msg.session_id;
    out.mode = static_cast<fbs::StopMode>(msg.mode);
    return out;
}

pyramid::data_model::SessionStopRequest from_fb(const fbs::SessionStopRequestT& msg, pyramid::data_model::SessionStopRequest* /*tag*/) {
    pyramid::data_model::SessionStopRequest out{};
    out.session_id = msg.session_id;
    out.mode = static_cast<pyramid::data_model::StopMode>(msg.mode);
    return out;
}

fbs::AckT to_fb(const pyramid::data_model::Ack& msg) {
    fbs::AckT out{};
    out.success = msg.success;
    return out;
}

pyramid::data_model::Ack from_fb(const fbs::AckT& msg, pyramid::data_model::Ack* /*tag*/) {
    pyramid::data_model::Ack out{};
    out.success = msg.success;
    return out;
}

fbs::QueryT to_fb(const pyramid::data_model::Query& msg) {
    fbs::QueryT out{};
    out.id = msg.id;
    out.has_one_shot = msg.one_shot.has_value();
    if (msg.one_shot.has_value()) {
        out.one_shot = msg.one_shot.value();
    }
    return out;
}

pyramid::data_model::Query from_fb(const fbs::QueryT& msg, pyramid::data_model::Query* /*tag*/) {
    pyramid::data_model::Query out{};
    out.id = msg.id;
    if (msg.has_one_shot) {
        out.one_shot = msg.one_shot;
    }
    return out;
}

fbs::IdentifierValueT to_fb(const pyramid::data_model::Identifier& msg) {
    fbs::IdentifierValueT out{};
    out.value = msg;
    return out;
}

pyramid::data_model::Identifier from_fb(const fbs::IdentifierValueT& msg, pyramid::data_model::Identifier* /*tag*/) {
    return msg.value;
}

fbs::CapabilitiesArrayHolderT to_fb(const std::vector<pyramid::data_model::Capabilities>& msg) {
    fbs::CapabilitiesArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::CapabilitiesT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::data_model::Capabilities> from_fb(const fbs::CapabilitiesArrayHolderT& msg) {
    std::vector<pyramid::data_model::Capabilities> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::data_model::Capabilities*>(nullptr)));
        }
    }
    return out;
}

fbs::SessionSnapshotArrayHolderT to_fb(const std::vector<pyramid::data_model::SessionSnapshot>& msg) {
    fbs::SessionSnapshotArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::SessionSnapshotT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::data_model::SessionSnapshot> from_fb(const fbs::SessionSnapshotArrayHolderT& msg) {
    std::vector<pyramid::data_model::SessionSnapshot> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::data_model::SessionSnapshot*>(nullptr)));
        }
    }
    return out;
}

fbs::CommandArrayHolderT to_fb(const std::vector<pyramid::data_model::Command>& msg) {
    fbs::CommandArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::CommandT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::data_model::Command> from_fb(const fbs::CommandArrayHolderT& msg) {
    std::vector<pyramid::data_model::Command> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::data_model::Command*>(nullptr)));
        }
    }
    return out;
}

fbs::GoalDispatchArrayHolderT to_fb(const std::vector<pyramid::data_model::GoalDispatch>& msg) {
    fbs::GoalDispatchArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::GoalDispatchT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::data_model::GoalDispatch> from_fb(const fbs::GoalDispatchArrayHolderT& msg) {
    std::vector<pyramid::data_model::GoalDispatch> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::data_model::GoalDispatch*>(nullptr)));
        }
    }
    return out;
}

fbs::DecisionRecordArrayHolderT to_fb(const std::vector<pyramid::data_model::DecisionRecord>& msg) {
    fbs::DecisionRecordArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::DecisionRecordT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::data_model::DecisionRecord> from_fb(const fbs::DecisionRecordArrayHolderT& msg) {
    std::vector<pyramid::data_model::DecisionRecord> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::data_model::DecisionRecord*>(nullptr)));
        }
    }
    return out;
}

} // namespace

std::string toBinary(const pyramid::data_model::FactUpdate& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::FactUpdate::Pack(builder, &object));
}

pyramid::data_model::FactUpdate fromBinaryFactUpdate(const void* data, size_t size) {
    auto* root = verified_root<fbs::FactUpdate>(data, size, "FactUpdate");
    fbs::FactUpdateT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::FactUpdate*>(nullptr));
}

std::string toBinary(const pyramid::data_model::StateUpdate& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::StateUpdate::Pack(builder, &object));
}

pyramid::data_model::StateUpdate fromBinaryStateUpdate(const void* data, size_t size) {
    auto* root = verified_root<fbs::StateUpdate>(data, size, "StateUpdate");
    fbs::StateUpdateT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::StateUpdate*>(nullptr));
}

std::string toBinary(const pyramid::data_model::MissionIntent& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::MissionIntent::Pack(builder, &object));
}

pyramid::data_model::MissionIntent fromBinaryMissionIntent(const void* data, size_t size) {
    auto* root = verified_root<fbs::MissionIntent>(data, size, "MissionIntent");
    fbs::MissionIntentT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::MissionIntent*>(nullptr));
}

std::string toBinary(const pyramid::data_model::AgentState& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::AgentState::Pack(builder, &object));
}

pyramid::data_model::AgentState fromBinaryAgentState(const void* data, size_t size) {
    auto* root = verified_root<fbs::AgentState>(data, size, "AgentState");
    fbs::AgentStateT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::AgentState*>(nullptr));
}

std::string toBinary(const pyramid::data_model::PolicyEnvelope& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::PolicyEnvelope::Pack(builder, &object));
}

pyramid::data_model::PolicyEnvelope fromBinaryPolicyEnvelope(const void* data, size_t size) {
    auto* root = verified_root<fbs::PolicyEnvelope>(data, size, "PolicyEnvelope");
    fbs::PolicyEnvelopeT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::PolicyEnvelope*>(nullptr));
}

std::string toBinary(const pyramid::data_model::Session& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::Session::Pack(builder, &object));
}

pyramid::data_model::Session fromBinarySession(const void* data, size_t size) {
    auto* root = verified_root<fbs::Session>(data, size, "Session");
    fbs::SessionT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::Session*>(nullptr));
}

std::string toBinary(const pyramid::data_model::Capabilities& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::Capabilities::Pack(builder, &object));
}

pyramid::data_model::Capabilities fromBinaryCapabilities(const void* data, size_t size) {
    auto* root = verified_root<fbs::Capabilities>(data, size, "Capabilities");
    fbs::CapabilitiesT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::Capabilities*>(nullptr));
}

std::string toBinary(const pyramid::data_model::StringKeyValue& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::StringKeyValue::Pack(builder, &object));
}

pyramid::data_model::StringKeyValue fromBinaryStringKeyValue(const void* data, size_t size) {
    auto* root = verified_root<fbs::StringKeyValue>(data, size, "StringKeyValue");
    fbs::StringKeyValueT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::StringKeyValue*>(nullptr));
}

std::string toBinary(const pyramid::data_model::Command& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::Command::Pack(builder, &object));
}

pyramid::data_model::Command fromBinaryCommand(const void* data, size_t size) {
    auto* root = verified_root<fbs::Command>(data, size, "Command");
    fbs::CommandT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::Command*>(nullptr));
}

std::string toBinary(const pyramid::data_model::GoalDispatch& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::GoalDispatch::Pack(builder, &object));
}

pyramid::data_model::GoalDispatch fromBinaryGoalDispatch(const void* data, size_t size) {
    auto* root = verified_root<fbs::GoalDispatch>(data, size, "GoalDispatch");
    fbs::GoalDispatchT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::GoalDispatch*>(nullptr));
}

std::string toBinary(const pyramid::data_model::DecisionRecord& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::DecisionRecord::Pack(builder, &object));
}

pyramid::data_model::DecisionRecord fromBinaryDecisionRecord(const void* data, size_t size) {
    auto* root = verified_root<fbs::DecisionRecord>(data, size, "DecisionRecord");
    fbs::DecisionRecordT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::DecisionRecord*>(nullptr));
}

std::string toBinary(const pyramid::data_model::CommandResult& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::CommandResult::Pack(builder, &object));
}

pyramid::data_model::CommandResult fromBinaryCommandResult(const void* data, size_t size) {
    auto* root = verified_root<fbs::CommandResult>(data, size, "CommandResult");
    fbs::CommandResultT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::CommandResult*>(nullptr));
}

std::string toBinary(const pyramid::data_model::DispatchResult& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::DispatchResult::Pack(builder, &object));
}

pyramid::data_model::DispatchResult fromBinaryDispatchResult(const void* data, size_t size) {
    auto* root = verified_root<fbs::DispatchResult>(data, size, "DispatchResult");
    fbs::DispatchResultT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::DispatchResult*>(nullptr));
}

std::string toBinary(const pyramid::data_model::SessionSnapshot& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::SessionSnapshot::Pack(builder, &object));
}

pyramid::data_model::SessionSnapshot fromBinarySessionSnapshot(const void* data, size_t size) {
    auto* root = verified_root<fbs::SessionSnapshot>(data, size, "SessionSnapshot");
    fbs::SessionSnapshotT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::SessionSnapshot*>(nullptr));
}

std::string toBinary(const pyramid::data_model::SessionStepRequest& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::SessionStepRequest::Pack(builder, &object));
}

pyramid::data_model::SessionStepRequest fromBinarySessionStepRequest(const void* data, size_t size) {
    auto* root = verified_root<fbs::SessionStepRequest>(data, size, "SessionStepRequest");
    fbs::SessionStepRequestT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::SessionStepRequest*>(nullptr));
}

std::string toBinary(const pyramid::data_model::SessionStopRequest& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::SessionStopRequest::Pack(builder, &object));
}

pyramid::data_model::SessionStopRequest fromBinarySessionStopRequest(const void* data, size_t size) {
    auto* root = verified_root<fbs::SessionStopRequest>(data, size, "SessionStopRequest");
    fbs::SessionStopRequestT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::SessionStopRequest*>(nullptr));
}

std::string toBinary(const pyramid::data_model::Ack& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::Ack::Pack(builder, &object));
}

pyramid::data_model::Ack fromBinaryAck(const void* data, size_t size) {
    auto* root = verified_root<fbs::Ack>(data, size, "Ack");
    fbs::AckT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::Ack*>(nullptr));
}

std::string toBinary(const pyramid::data_model::Query& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::Query::Pack(builder, &object));
}

pyramid::data_model::Query fromBinaryQuery(const void* data, size_t size) {
    auto* root = verified_root<fbs::Query>(data, size, "Query");
    fbs::QueryT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::Query*>(nullptr));
}

std::string toBinary(const pyramid::data_model::Identifier& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::IdentifierValue::Pack(builder, &object));
}

pyramid::data_model::Identifier fromBinaryIdentifier(const void* data, size_t size) {
    auto* root = verified_root<fbs::IdentifierValue>(data, size, "Identifier");
    fbs::IdentifierValueT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::Identifier*>(nullptr));
}

std::string toBinary(const std::vector<pyramid::data_model::Capabilities>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::CapabilitiesArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::data_model::Capabilities> fromBinaryCapabilitiesArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::CapabilitiesArrayHolder>(data, size, "CapabilitiesArray");
    fbs::CapabilitiesArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const std::vector<pyramid::data_model::SessionSnapshot>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::SessionSnapshotArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::data_model::SessionSnapshot> fromBinarySessionSnapshotArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::SessionSnapshotArrayHolder>(data, size, "SessionSnapshotArray");
    fbs::SessionSnapshotArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const std::vector<pyramid::data_model::Command>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::CommandArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::data_model::Command> fromBinaryCommandArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::CommandArrayHolder>(data, size, "CommandArray");
    fbs::CommandArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const std::vector<pyramid::data_model::GoalDispatch>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::GoalDispatchArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::data_model::GoalDispatch> fromBinaryGoalDispatchArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::GoalDispatchArrayHolder>(data, size, "GoalDispatchArray");
    fbs::GoalDispatchArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const std::vector<pyramid::data_model::DecisionRecord>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::DecisionRecordArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::data_model::DecisionRecord> fromBinaryDecisionRecordArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::DecisionRecordArrayHolder>(data, size, "DecisionRecordArray");
    fbs::DecisionRecordArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

} // namespace pyramid::services::autonomy_backend::flatbuffers_codec

extern "C" {

void pyramid_services_autonomy_backend_free_buffer(void* data) {
    std::free(data);
}

void* pyramid_services_autonomy_backend_FactUpdate_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::FactUpdate*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_FactUpdate_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryFactUpdate(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_StateUpdate_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::StateUpdate*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_StateUpdate_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryStateUpdate(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_MissionIntent_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::MissionIntent*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_MissionIntent_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryMissionIntent(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_AgentState_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::AgentState*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_AgentState_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryAgentState(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_PolicyEnvelope_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::PolicyEnvelope*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_PolicyEnvelope_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryPolicyEnvelope(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_Session_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::Session*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_Session_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinarySession(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_Capabilities_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::Capabilities*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_Capabilities_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryCapabilities(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_StringKeyValue_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::StringKeyValue*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_StringKeyValue_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryStringKeyValue(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_Command_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::Command*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_Command_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryCommand(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_GoalDispatch_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::GoalDispatch*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_GoalDispatch_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryGoalDispatch(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_DecisionRecord_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::DecisionRecord*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_DecisionRecord_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryDecisionRecord(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_CommandResult_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::CommandResult*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_CommandResult_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryCommandResult(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_DispatchResult_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::DispatchResult*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_DispatchResult_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryDispatchResult(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_SessionSnapshot_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::SessionSnapshot*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_SessionSnapshot_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinarySessionSnapshot(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_SessionStepRequest_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::SessionStepRequest*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_SessionStepRequest_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinarySessionStepRequest(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_SessionStopRequest_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::SessionStopRequest*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_SessionStopRequest_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinarySessionStopRequest(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_Ack_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::Ack*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_Ack_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryAck(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_Query_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::Query*>(nullptr));
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_Query_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryQuery(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_Identifier_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = [&]() { auto j = nlohmann::json::parse(std::string(json ? json : "")); return j.is_string() ? j.get<pyramid::data_model::Identifier>() : pyramid::data_model::Identifier{}; }();
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(value);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_Identifier_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryIdentifier(data, size);
        auto json = nlohmann::json(value).dump();
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_CapabilitiesArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::data_model::Capabilities> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::data_model::common::fromJson(item.dump(), static_cast<pyramid::data_model::Capabilities*>(nullptr)));
            }
        }
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(values);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_CapabilitiesArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryCapabilitiesArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::data_model::common::toJson(item)));
        }
        auto json = arr.dump();
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_SessionSnapshotArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::data_model::SessionSnapshot> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::data_model::common::fromJson(item.dump(), static_cast<pyramid::data_model::SessionSnapshot*>(nullptr)));
            }
        }
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(values);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_SessionSnapshotArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinarySessionSnapshotArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::data_model::common::toJson(item)));
        }
        auto json = arr.dump();
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_CommandArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::data_model::Command> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::data_model::common::fromJson(item.dump(), static_cast<pyramid::data_model::Command*>(nullptr)));
            }
        }
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(values);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_CommandArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryCommandArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::data_model::common::toJson(item)));
        }
        auto json = arr.dump();
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_GoalDispatchArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::data_model::GoalDispatch> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::data_model::common::fromJson(item.dump(), static_cast<pyramid::data_model::GoalDispatch*>(nullptr)));
            }
        }
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(values);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_GoalDispatchArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryGoalDispatchArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::data_model::common::toJson(item)));
        }
        auto json = arr.dump();
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_DecisionRecordArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::data_model::DecisionRecord> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::data_model::common::fromJson(item.dump(), static_cast<pyramid::data_model::DecisionRecord*>(nullptr)));
            }
        }
        auto payload = pyramid::services::autonomy_backend::flatbuffers_codec::toBinary(values);
        if (size_out) *size_out = payload.size();
        if (payload.empty()) return nullptr;
        void* out = std::malloc(payload.size());
        if (!out) return nullptr;
        std::memcpy(out, payload.data(), payload.size());
        return out;
    } catch (...) {
        return nullptr;
    }
}

char* pyramid_services_autonomy_backend_DecisionRecordArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryDecisionRecordArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::data_model::common::toJson(item)));
        }
        auto json = arr.dump();
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

} // extern "C"
