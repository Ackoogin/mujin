// Auto-generated service FlatBuffers codec
#include "pyramid_services_autonomy_backend_flatbuffers_codec.hpp"

#include "pyramid_data_model_autonomy_codec.hpp"
#include "pyramid_data_model_base_codec.hpp"
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

fbs::RequirementReferenceT to_fb(const pyramid::data_model::RequirementReference& msg) {
    fbs::RequirementReferenceT out{};
    out.requirement_id = msg.requirement_id;
    out.component_name = msg.component_name;
    out.service_name = msg.service_name;
    out.type_name = msg.type_name;
    return out;
}

pyramid::data_model::RequirementReference from_fb(const fbs::RequirementReferenceT& msg, pyramid::data_model::RequirementReference* /*tag*/) {
    pyramid::data_model::RequirementReference out{};
    out.requirement_id = msg.requirement_id;
    out.component_name = msg.component_name;
    out.service_name = msg.service_name;
    out.type_name = msg.type_name;
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

fbs::PlanningPolicyT to_fb(const pyramid::data_model::PlanningPolicy& msg) {
    fbs::PlanningPolicyT out{};
    out.max_replans = msg.max_replans;
    out.enable_replanning = msg.enable_replanning;
    return out;
}

pyramid::data_model::PlanningPolicy from_fb(const fbs::PlanningPolicyT& msg, pyramid::data_model::PlanningPolicy* /*tag*/) {
    pyramid::data_model::PlanningPolicy out{};
    out.max_replans = msg.max_replans;
    out.enable_replanning = msg.enable_replanning;
    return out;
}

fbs::PlanningGoalT to_fb(const pyramid::data_model::PlanningGoal& msg) {
    fbs::PlanningGoalT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.name = msg.name;
    if (msg.requirement.has_value()) {
        out.requirement = std::make_unique<fbs::RequirementReferenceT>(to_fb(msg.requirement.value()));
    }
    if (msg.expression.has_value()) {
        out.expression = msg.expression.value();
    }
    return out;
}

pyramid::data_model::PlanningGoal from_fb(const fbs::PlanningGoalT& msg, pyramid::data_model::PlanningGoal* /*tag*/) {
    pyramid::data_model::PlanningGoal out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    out.name = msg.name;
    if (msg.requirement) {
        out.requirement = from_fb(*msg.requirement, static_cast<pyramid::data_model::RequirementReference*>(nullptr));
    }
    if (!msg.expression.empty()) {
        out.expression = msg.expression;
    }
    return out;
}

fbs::ExecutionPolicyT to_fb(const pyramid::data_model::ExecutionPolicy& msg) {
    fbs::ExecutionPolicyT out{};
    out.max_replans = msg.max_replans;
    out.enable_replanning = msg.enable_replanning;
    out.max_concurrent_placements = msg.max_concurrent_placements;
    return out;
}

pyramid::data_model::ExecutionPolicy from_fb(const fbs::ExecutionPolicyT& msg, pyramid::data_model::ExecutionPolicy* /*tag*/) {
    pyramid::data_model::ExecutionPolicy out{};
    out.max_replans = msg.max_replans;
    out.enable_replanning = msg.enable_replanning;
    out.max_concurrent_placements = msg.max_concurrent_placements;
    return out;
}

fbs::WorldFactUpdateT to_fb(const pyramid::data_model::WorldFactUpdate& msg) {
    fbs::WorldFactUpdateT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.entity_source = msg.entity_source;
    out.key = msg.key;
    out.value = msg.value;
    out.source = msg.source;
    out.authority = static_cast<fbs::FactAuthorityLevel>(msg.authority);
    return out;
}

pyramid::data_model::WorldFactUpdate from_fb(const fbs::WorldFactUpdateT& msg, pyramid::data_model::WorldFactUpdate* /*tag*/) {
    pyramid::data_model::WorldFactUpdate out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.entity_source = msg.entity_source;
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
    out.fact_update.reserve(msg.fact_update.size());
    for (const auto& item : msg.fact_update) {
        out.fact_update.emplace_back(std::make_unique<fbs::WorldFactUpdateT>(to_fb(item)));
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
    out.fact_update.reserve(msg.fact_update.size());
    for (const auto& item : msg.fact_update) {
        if (item) {
            out.fact_update.push_back(from_fb(*item, static_cast<pyramid::data_model::WorldFactUpdate*>(nullptr)));
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
    out.supports_planning_requirements = msg.supports_planning_requirements;
    out.supports_execution_requirements = msg.supports_execution_requirements;
    out.supports_approved_plan_execution = msg.supports_approved_plan_execution;
    out.supports_replanning = msg.supports_replanning;
    out.supports_typed_component_requirement_placement = msg.supports_typed_component_requirement_placement;
    out.supports_state_update_ingress = msg.supports_state_update_ingress;
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
    out.supports_planning_requirements = msg.supports_planning_requirements;
    out.supports_execution_requirements = msg.supports_execution_requirements;
    out.supports_approved_plan_execution = msg.supports_approved_plan_execution;
    out.supports_replanning = msg.supports_replanning;
    out.supports_typed_component_requirement_placement = msg.supports_typed_component_requirement_placement;
    out.supports_state_update_ingress = msg.supports_state_update_ingress;
    return out;
}

fbs::PlannedComponentInteractionT to_fb(const pyramid::data_model::PlannedComponentInteraction& msg) {
    fbs::PlannedComponentInteractionT out{};
    out.target_component = msg.target_component;
    out.target_service = msg.target_service;
    out.target_type = msg.target_type;
    out.operation = static_cast<fbs::RequirementPlacementOperation>(msg.operation);
    return out;
}

pyramid::data_model::PlannedComponentInteraction from_fb(const fbs::PlannedComponentInteractionT& msg, pyramid::data_model::PlannedComponentInteraction* /*tag*/) {
    pyramid::data_model::PlannedComponentInteraction out{};
    out.target_component = msg.target_component;
    out.target_service = msg.target_service;
    out.target_type = msg.target_type;
    out.operation = static_cast<pyramid::data_model::RequirementPlacementOperation>(msg.operation);
    return out;
}

fbs::PlanStepT to_fb(const pyramid::data_model::PlanStep& msg) {
    fbs::PlanStepT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.sequence_number = msg.sequence_number;
    out.action_name = msg.action_name;
    out.signature = msg.signature;
    out.interaction.reserve(msg.interaction.size());
    for (const auto& item : msg.interaction) {
        out.interaction.emplace_back(std::make_unique<fbs::PlannedComponentInteractionT>(to_fb(item)));
    }
    return out;
}

pyramid::data_model::PlanStep from_fb(const fbs::PlanStepT& msg, pyramid::data_model::PlanStep* /*tag*/) {
    pyramid::data_model::PlanStep out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    out.sequence_number = msg.sequence_number;
    out.action_name = msg.action_name;
    out.signature = msg.signature;
    out.interaction.reserve(msg.interaction.size());
    for (const auto& item : msg.interaction) {
        if (item) {
            out.interaction.push_back(from_fb(*item, static_cast<pyramid::data_model::PlannedComponentInteraction*>(nullptr)));
        }
    }
    return out;
}

fbs::PlanT to_fb(const pyramid::data_model::Plan& msg) {
    fbs::PlanT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.planning_requirement_id = msg.planning_requirement_id;
    out.backend_id = msg.backend_id;
    out.world_version = msg.world_version;
    out.replan_count = msg.replan_count;
    out.plan_success = msg.plan_success;
    out.solve_time_ms = msg.solve_time_ms;
    out.step.reserve(msg.step.size());
    for (const auto& item : msg.step) {
        out.step.emplace_back(std::make_unique<fbs::PlanStepT>(to_fb(item)));
    }
    out.compiled_bt_xml = msg.compiled_bt_xml;
    out.has_predicted_quality = msg.predicted_quality.has_value();
    if (msg.predicted_quality.has_value()) {
        out.predicted_quality = msg.predicted_quality.value();
    }
    return out;
}

pyramid::data_model::Plan from_fb(const fbs::PlanT& msg, pyramid::data_model::Plan* /*tag*/) {
    pyramid::data_model::Plan out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    out.planning_requirement_id = msg.planning_requirement_id;
    out.backend_id = msg.backend_id;
    out.world_version = msg.world_version;
    out.replan_count = msg.replan_count;
    out.plan_success = msg.plan_success;
    out.solve_time_ms = msg.solve_time_ms;
    out.step.reserve(msg.step.size());
    for (const auto& item : msg.step) {
        if (item) {
            out.step.push_back(from_fb(*item, static_cast<pyramid::data_model::PlanStep*>(nullptr)));
        }
    }
    out.compiled_bt_xml = msg.compiled_bt_xml;
    if (msg.has_predicted_quality) {
        out.predicted_quality = msg.predicted_quality;
    }
    return out;
}

fbs::RequirementPlacementT to_fb(const pyramid::data_model::RequirementPlacement& msg) {
    fbs::RequirementPlacementT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.execution_requirement_id = msg.execution_requirement_id;
    out.planning_requirement_id = msg.planning_requirement_id;
    out.plan_id = msg.plan_id;
    out.plan_step_id = msg.plan_step_id;
    out.target_component = msg.target_component;
    out.target_service = msg.target_service;
    out.target_type = msg.target_type;
    out.operation = static_cast<fbs::RequirementPlacementOperation>(msg.operation);
    out.target_requirement_id = msg.target_requirement_id;
    out.related_entity_id = msg.related_entity_id;
    out.progress = static_cast<fbs::Progress>(msg.progress);
    return out;
}

pyramid::data_model::RequirementPlacement from_fb(const fbs::RequirementPlacementT& msg, pyramid::data_model::RequirementPlacement* /*tag*/) {
    pyramid::data_model::RequirementPlacement out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    out.execution_requirement_id = msg.execution_requirement_id;
    out.planning_requirement_id = msg.planning_requirement_id;
    out.plan_id = msg.plan_id;
    out.plan_step_id = msg.plan_step_id;
    out.target_component = msg.target_component;
    out.target_service = msg.target_service;
    out.target_type = msg.target_type;
    out.operation = static_cast<pyramid::data_model::RequirementPlacementOperation>(msg.operation);
    out.target_requirement_id = msg.target_requirement_id;
    out.related_entity_id = msg.related_entity_id;
    out.progress = static_cast<pyramid::data_model::Progress>(msg.progress);
    return out;
}

fbs::AchievementT to_fb(const pyramid::data_model::Achievement& msg) {
    fbs::AchievementT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.status = static_cast<fbs::Progress>(msg.status);
    out.has_quality = msg.quality.has_value();
    if (msg.quality.has_value()) {
        out.quality = msg.quality.value();
    }
    out.achieveability = static_cast<fbs::Feasibility>(msg.achieveability);
    return out;
}

pyramid::data_model::Achievement from_fb(const fbs::AchievementT& msg, pyramid::data_model::Achievement* /*tag*/) {
    pyramid::data_model::Achievement out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    out.status = static_cast<pyramid::data_model::Progress>(msg.status);
    if (msg.has_quality) {
        out.quality = msg.quality;
    }
    out.achieveability = static_cast<pyramid::data_model::Feasibility>(msg.achieveability);
    return out;
}

fbs::EntityT to_fb(const pyramid::data_model::Entity& msg) {
    fbs::EntityT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    return out;
}

pyramid::data_model::Entity from_fb(const fbs::EntityT& msg, pyramid::data_model::Entity* /*tag*/) {
    pyramid::data_model::Entity out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
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

fbs::PlanningRequirementT to_fb(const pyramid::data_model::PlanningRequirement& msg) {
    fbs::PlanningRequirementT out{};
    out.base = std::make_unique<fbs::EntityT>(to_fb(msg.base));
    out.status = std::make_unique<fbs::AchievementT>(to_fb(msg.status));
    out.upstream_requirement.reserve(msg.upstream_requirement.size());
    for (const auto& item : msg.upstream_requirement) {
        out.upstream_requirement.emplace_back(std::make_unique<fbs::RequirementReferenceT>(to_fb(item)));
    }
    out.goal.reserve(msg.goal.size());
    for (const auto& item : msg.goal) {
        out.goal.emplace_back(std::make_unique<fbs::PlanningGoalT>(to_fb(item)));
    }
    out.policy = std::make_unique<fbs::PlanningPolicyT>(to_fb(msg.policy));
    out.available_agents.reserve(msg.available_agents.size());
    for (const auto& item : msg.available_agents) {
        out.available_agents.emplace_back(std::make_unique<fbs::AgentStateT>(to_fb(item)));
    }
    return out;
}

pyramid::data_model::PlanningRequirement from_fb(const fbs::PlanningRequirementT& msg, pyramid::data_model::PlanningRequirement* /*tag*/) {
    pyramid::data_model::PlanningRequirement out{};
    if (msg.base) out.base = from_fb(*msg.base, static_cast<pyramid::data_model::Entity*>(nullptr));
    if (msg.status) out.status = from_fb(*msg.status, static_cast<pyramid::data_model::Achievement*>(nullptr));
    out.upstream_requirement.reserve(msg.upstream_requirement.size());
    for (const auto& item : msg.upstream_requirement) {
        if (item) {
            out.upstream_requirement.push_back(from_fb(*item, static_cast<pyramid::data_model::RequirementReference*>(nullptr)));
        }
    }
    out.goal.reserve(msg.goal.size());
    for (const auto& item : msg.goal) {
        if (item) {
            out.goal.push_back(from_fb(*item, static_cast<pyramid::data_model::PlanningGoal*>(nullptr)));
        }
    }
    if (msg.policy) out.policy = from_fb(*msg.policy, static_cast<pyramid::data_model::PlanningPolicy*>(nullptr));
    out.available_agents.reserve(msg.available_agents.size());
    for (const auto& item : msg.available_agents) {
        if (item) {
            out.available_agents.push_back(from_fb(*item, static_cast<pyramid::data_model::AgentState*>(nullptr)));
        }
    }
    return out;
}

fbs::ExecutionRequirementT to_fb(const pyramid::data_model::ExecutionRequirement& msg) {
    fbs::ExecutionRequirementT out{};
    out.base = std::make_unique<fbs::EntityT>(to_fb(msg.base));
    out.status = std::make_unique<fbs::AchievementT>(to_fb(msg.status));
    out.upstream_requirement.reserve(msg.upstream_requirement.size());
    for (const auto& item : msg.upstream_requirement) {
        out.upstream_requirement.emplace_back(std::make_unique<fbs::RequirementReferenceT>(to_fb(item)));
    }
    out.plan_id = msg.plan_id;
    out.policy = std::make_unique<fbs::ExecutionPolicyT>(to_fb(msg.policy));
    out.available_agents.reserve(msg.available_agents.size());
    for (const auto& item : msg.available_agents) {
        out.available_agents.emplace_back(std::make_unique<fbs::AgentStateT>(to_fb(item)));
    }
    out.planning_requirement_id = msg.planning_requirement_id;
    return out;
}

pyramid::data_model::ExecutionRequirement from_fb(const fbs::ExecutionRequirementT& msg, pyramid::data_model::ExecutionRequirement* /*tag*/) {
    pyramid::data_model::ExecutionRequirement out{};
    if (msg.base) out.base = from_fb(*msg.base, static_cast<pyramid::data_model::Entity*>(nullptr));
    if (msg.status) out.status = from_fb(*msg.status, static_cast<pyramid::data_model::Achievement*>(nullptr));
    out.upstream_requirement.reserve(msg.upstream_requirement.size());
    for (const auto& item : msg.upstream_requirement) {
        if (item) {
            out.upstream_requirement.push_back(from_fb(*item, static_cast<pyramid::data_model::RequirementReference*>(nullptr)));
        }
    }
    out.plan_id = msg.plan_id;
    if (msg.policy) out.policy = from_fb(*msg.policy, static_cast<pyramid::data_model::ExecutionPolicy*>(nullptr));
    out.available_agents.reserve(msg.available_agents.size());
    for (const auto& item : msg.available_agents) {
        if (item) {
            out.available_agents.push_back(from_fb(*item, static_cast<pyramid::data_model::AgentState*>(nullptr)));
        }
    }
    out.planning_requirement_id = msg.planning_requirement_id;
    return out;
}

fbs::ExecutionRunT to_fb(const pyramid::data_model::ExecutionRun& msg) {
    fbs::ExecutionRunT out{};
    out.has_update_time = msg.update_time.has_value();
    if (msg.update_time.has_value()) {
        out.update_time = msg.update_time.value();
    }
    out.id = msg.id;
    out.source = msg.source;
    out.execution_requirement_id = msg.execution_requirement_id;
    out.planning_requirement_id = msg.planning_requirement_id;
    out.plan_id = msg.plan_id;
    out.state = static_cast<fbs::ExecutionState>(msg.state);
    out.achievement = std::make_unique<fbs::AchievementT>(to_fb(msg.achievement));
    out.replan_count = msg.replan_count;
    out.outstanding_placement.reserve(msg.outstanding_placement.size());
    for (const auto& item : msg.outstanding_placement) {
        out.outstanding_placement.emplace_back(std::make_unique<fbs::RequirementPlacementT>(to_fb(item)));
    }
    return out;
}

pyramid::data_model::ExecutionRun from_fb(const fbs::ExecutionRunT& msg, pyramid::data_model::ExecutionRun* /*tag*/) {
    pyramid::data_model::ExecutionRun out{};
    if (msg.has_update_time) {
        out.update_time = msg.update_time;
    }
    out.id = msg.id;
    out.source = msg.source;
    out.execution_requirement_id = msg.execution_requirement_id;
    out.planning_requirement_id = msg.planning_requirement_id;
    out.plan_id = msg.plan_id;
    out.state = static_cast<pyramid::data_model::ExecutionState>(msg.state);
    if (msg.achievement) out.achievement = from_fb(*msg.achievement, static_cast<pyramid::data_model::Achievement*>(nullptr));
    out.replan_count = msg.replan_count;
    out.outstanding_placement.reserve(msg.outstanding_placement.size());
    for (const auto& item : msg.outstanding_placement) {
        if (item) {
            out.outstanding_placement.push_back(from_fb(*item, static_cast<pyramid::data_model::RequirementPlacement*>(nullptr)));
        }
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

fbs::PlanningRequirementArrayHolderT to_fb(const std::vector<pyramid::data_model::PlanningRequirement>& msg) {
    fbs::PlanningRequirementArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::PlanningRequirementT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::data_model::PlanningRequirement> from_fb(const fbs::PlanningRequirementArrayHolderT& msg) {
    std::vector<pyramid::data_model::PlanningRequirement> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::data_model::PlanningRequirement*>(nullptr)));
        }
    }
    return out;
}

fbs::ExecutionRequirementArrayHolderT to_fb(const std::vector<pyramid::data_model::ExecutionRequirement>& msg) {
    fbs::ExecutionRequirementArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::ExecutionRequirementT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::data_model::ExecutionRequirement> from_fb(const fbs::ExecutionRequirementArrayHolderT& msg) {
    std::vector<pyramid::data_model::ExecutionRequirement> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::data_model::ExecutionRequirement*>(nullptr)));
        }
    }
    return out;
}

fbs::PlanArrayHolderT to_fb(const std::vector<pyramid::data_model::Plan>& msg) {
    fbs::PlanArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::PlanT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::data_model::Plan> from_fb(const fbs::PlanArrayHolderT& msg) {
    std::vector<pyramid::data_model::Plan> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::data_model::Plan*>(nullptr)));
        }
    }
    return out;
}

fbs::ExecutionRunArrayHolderT to_fb(const std::vector<pyramid::data_model::ExecutionRun>& msg) {
    fbs::ExecutionRunArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::ExecutionRunT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::data_model::ExecutionRun> from_fb(const fbs::ExecutionRunArrayHolderT& msg) {
    std::vector<pyramid::data_model::ExecutionRun> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::data_model::ExecutionRun*>(nullptr)));
        }
    }
    return out;
}

fbs::RequirementPlacementArrayHolderT to_fb(const std::vector<pyramid::data_model::RequirementPlacement>& msg) {
    fbs::RequirementPlacementArrayHolderT out{};
    out.items.reserve(msg.size());
    for (const auto& item : msg) {
        out.items.emplace_back(std::make_unique<fbs::RequirementPlacementT>(to_fb(item)));
    }
    return out;
}

std::vector<pyramid::data_model::RequirementPlacement> from_fb(const fbs::RequirementPlacementArrayHolderT& msg) {
    std::vector<pyramid::data_model::RequirementPlacement> out{};
    out.reserve(msg.items.size());
    for (const auto& item : msg.items) {
        if (item) {
            out.push_back(from_fb(*item, static_cast<pyramid::data_model::RequirementPlacement*>(nullptr)));
        }
    }
    return out;
}

} // namespace

std::string toBinary(const pyramid::data_model::RequirementReference& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::RequirementReference::Pack(builder, &object));
}

pyramid::data_model::RequirementReference fromBinaryRequirementReference(const void* data, size_t size) {
    auto* root = verified_root<fbs::RequirementReference>(data, size, "RequirementReference");
    fbs::RequirementReferenceT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::RequirementReference*>(nullptr));
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

std::string toBinary(const pyramid::data_model::PlanningPolicy& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::PlanningPolicy::Pack(builder, &object));
}

pyramid::data_model::PlanningPolicy fromBinaryPlanningPolicy(const void* data, size_t size) {
    auto* root = verified_root<fbs::PlanningPolicy>(data, size, "PlanningPolicy");
    fbs::PlanningPolicyT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::PlanningPolicy*>(nullptr));
}

std::string toBinary(const pyramid::data_model::PlanningGoal& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::PlanningGoal::Pack(builder, &object));
}

pyramid::data_model::PlanningGoal fromBinaryPlanningGoal(const void* data, size_t size) {
    auto* root = verified_root<fbs::PlanningGoal>(data, size, "PlanningGoal");
    fbs::PlanningGoalT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::PlanningGoal*>(nullptr));
}

std::string toBinary(const pyramid::data_model::ExecutionPolicy& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ExecutionPolicy::Pack(builder, &object));
}

pyramid::data_model::ExecutionPolicy fromBinaryExecutionPolicy(const void* data, size_t size) {
    auto* root = verified_root<fbs::ExecutionPolicy>(data, size, "ExecutionPolicy");
    fbs::ExecutionPolicyT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::ExecutionPolicy*>(nullptr));
}

std::string toBinary(const pyramid::data_model::WorldFactUpdate& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::WorldFactUpdate::Pack(builder, &object));
}

pyramid::data_model::WorldFactUpdate fromBinaryWorldFactUpdate(const void* data, size_t size) {
    auto* root = verified_root<fbs::WorldFactUpdate>(data, size, "WorldFactUpdate");
    fbs::WorldFactUpdateT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::WorldFactUpdate*>(nullptr));
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

std::string toBinary(const pyramid::data_model::PlannedComponentInteraction& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::PlannedComponentInteraction::Pack(builder, &object));
}

pyramid::data_model::PlannedComponentInteraction fromBinaryPlannedComponentInteraction(const void* data, size_t size) {
    auto* root = verified_root<fbs::PlannedComponentInteraction>(data, size, "PlannedComponentInteraction");
    fbs::PlannedComponentInteractionT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::PlannedComponentInteraction*>(nullptr));
}

std::string toBinary(const pyramid::data_model::PlanStep& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::PlanStep::Pack(builder, &object));
}

pyramid::data_model::PlanStep fromBinaryPlanStep(const void* data, size_t size) {
    auto* root = verified_root<fbs::PlanStep>(data, size, "PlanStep");
    fbs::PlanStepT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::PlanStep*>(nullptr));
}

std::string toBinary(const pyramid::data_model::Plan& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::Plan::Pack(builder, &object));
}

pyramid::data_model::Plan fromBinaryPlan(const void* data, size_t size) {
    auto* root = verified_root<fbs::Plan>(data, size, "Plan");
    fbs::PlanT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::Plan*>(nullptr));
}

std::string toBinary(const pyramid::data_model::RequirementPlacement& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::RequirementPlacement::Pack(builder, &object));
}

pyramid::data_model::RequirementPlacement fromBinaryRequirementPlacement(const void* data, size_t size) {
    auto* root = verified_root<fbs::RequirementPlacement>(data, size, "RequirementPlacement");
    fbs::RequirementPlacementT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::RequirementPlacement*>(nullptr));
}

std::string toBinary(const pyramid::data_model::Achievement& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::Achievement::Pack(builder, &object));
}

pyramid::data_model::Achievement fromBinaryAchievement(const void* data, size_t size) {
    auto* root = verified_root<fbs::Achievement>(data, size, "Achievement");
    fbs::AchievementT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::Achievement*>(nullptr));
}

std::string toBinary(const pyramid::data_model::Entity& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::Entity::Pack(builder, &object));
}

pyramid::data_model::Entity fromBinaryEntity(const void* data, size_t size) {
    auto* root = verified_root<fbs::Entity>(data, size, "Entity");
    fbs::EntityT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::Entity*>(nullptr));
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

std::string toBinary(const pyramid::data_model::PlanningRequirement& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::PlanningRequirement::Pack(builder, &object));
}

pyramid::data_model::PlanningRequirement fromBinaryPlanningRequirement(const void* data, size_t size) {
    auto* root = verified_root<fbs::PlanningRequirement>(data, size, "PlanningRequirement");
    fbs::PlanningRequirementT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::PlanningRequirement*>(nullptr));
}

std::string toBinary(const pyramid::data_model::ExecutionRequirement& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ExecutionRequirement::Pack(builder, &object));
}

pyramid::data_model::ExecutionRequirement fromBinaryExecutionRequirement(const void* data, size_t size) {
    auto* root = verified_root<fbs::ExecutionRequirement>(data, size, "ExecutionRequirement");
    fbs::ExecutionRequirementT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::ExecutionRequirement*>(nullptr));
}

std::string toBinary(const pyramid::data_model::ExecutionRun& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ExecutionRun::Pack(builder, &object));
}

pyramid::data_model::ExecutionRun fromBinaryExecutionRun(const void* data, size_t size) {
    auto* root = verified_root<fbs::ExecutionRun>(data, size, "ExecutionRun");
    fbs::ExecutionRunT object{};
    root->UnPackTo(&object);
    return from_fb(object, static_cast<pyramid::data_model::ExecutionRun*>(nullptr));
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

std::string toBinary(const std::vector<pyramid::data_model::PlanningRequirement>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::PlanningRequirementArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::data_model::PlanningRequirement> fromBinaryPlanningRequirementArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::PlanningRequirementArrayHolder>(data, size, "PlanningRequirementArray");
    fbs::PlanningRequirementArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const std::vector<pyramid::data_model::ExecutionRequirement>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ExecutionRequirementArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::data_model::ExecutionRequirement> fromBinaryExecutionRequirementArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::ExecutionRequirementArrayHolder>(data, size, "ExecutionRequirementArray");
    fbs::ExecutionRequirementArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const std::vector<pyramid::data_model::Plan>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::PlanArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::data_model::Plan> fromBinaryPlanArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::PlanArrayHolder>(data, size, "PlanArray");
    fbs::PlanArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const std::vector<pyramid::data_model::ExecutionRun>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::ExecutionRunArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::data_model::ExecutionRun> fromBinaryExecutionRunArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::ExecutionRunArrayHolder>(data, size, "ExecutionRunArray");
    fbs::ExecutionRunArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

std::string toBinary(const std::vector<pyramid::data_model::RequirementPlacement>& msg) {
    flatbuffers::FlatBufferBuilder builder(256);
    auto object = to_fb(msg);
    return finish_buffer(builder, fbs::RequirementPlacementArrayHolder::Pack(builder, &object));
}

std::vector<pyramid::data_model::RequirementPlacement> fromBinaryRequirementPlacementArray(const void* data, size_t size) {
    auto* root = verified_root<fbs::RequirementPlacementArrayHolder>(data, size, "RequirementPlacementArray");
    fbs::RequirementPlacementArrayHolderT object{};
    root->UnPackTo(&object);
    return from_fb(object);
}

} // namespace pyramid::services::autonomy_backend::flatbuffers_codec

extern "C" {

void pyramid_services_autonomy_backend_free_buffer(void* data) {
    std::free(data);
}

void* pyramid_services_autonomy_backend_RequirementReference_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::autonomy::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::RequirementReference*>(nullptr));
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

char* pyramid_services_autonomy_backend_RequirementReference_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryRequirementReference(data, size);
        auto json = pyramid::data_model::autonomy::toJson(value);
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
        auto value = pyramid::data_model::autonomy::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::AgentState*>(nullptr));
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
        auto json = pyramid::data_model::autonomy::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_PlanningPolicy_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::autonomy::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::PlanningPolicy*>(nullptr));
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

char* pyramid_services_autonomy_backend_PlanningPolicy_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryPlanningPolicy(data, size);
        auto json = pyramid::data_model::autonomy::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_PlanningGoal_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::autonomy::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::PlanningGoal*>(nullptr));
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

char* pyramid_services_autonomy_backend_PlanningGoal_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryPlanningGoal(data, size);
        auto json = pyramid::data_model::autonomy::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_ExecutionPolicy_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::autonomy::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::ExecutionPolicy*>(nullptr));
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

char* pyramid_services_autonomy_backend_ExecutionPolicy_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryExecutionPolicy(data, size);
        auto json = pyramid::data_model::autonomy::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_WorldFactUpdate_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::autonomy::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::WorldFactUpdate*>(nullptr));
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

char* pyramid_services_autonomy_backend_WorldFactUpdate_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryWorldFactUpdate(data, size);
        auto json = pyramid::data_model::autonomy::toJson(value);
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
        auto value = pyramid::data_model::autonomy::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::StateUpdate*>(nullptr));
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
        auto json = pyramid::data_model::autonomy::toJson(value);
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
        auto value = pyramid::data_model::autonomy::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::Capabilities*>(nullptr));
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
        auto json = pyramid::data_model::autonomy::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_PlannedComponentInteraction_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::autonomy::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::PlannedComponentInteraction*>(nullptr));
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

char* pyramid_services_autonomy_backend_PlannedComponentInteraction_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryPlannedComponentInteraction(data, size);
        auto json = pyramid::data_model::autonomy::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_PlanStep_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::autonomy::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::PlanStep*>(nullptr));
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

char* pyramid_services_autonomy_backend_PlanStep_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryPlanStep(data, size);
        auto json = pyramid::data_model::autonomy::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_Plan_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::autonomy::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::Plan*>(nullptr));
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

char* pyramid_services_autonomy_backend_Plan_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryPlan(data, size);
        auto json = pyramid::data_model::autonomy::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_RequirementPlacement_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::autonomy::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::RequirementPlacement*>(nullptr));
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

char* pyramid_services_autonomy_backend_RequirementPlacement_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryRequirementPlacement(data, size);
        auto json = pyramid::data_model::autonomy::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_Achievement_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::Achievement*>(nullptr));
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

char* pyramid_services_autonomy_backend_Achievement_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryAchievement(data, size);
        auto json = pyramid::data_model::common::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_Entity_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::common::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::Entity*>(nullptr));
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

char* pyramid_services_autonomy_backend_Entity_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryEntity(data, size);
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

void* pyramid_services_autonomy_backend_PlanningRequirement_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::autonomy::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::PlanningRequirement*>(nullptr));
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

char* pyramid_services_autonomy_backend_PlanningRequirement_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryPlanningRequirement(data, size);
        auto json = pyramid::data_model::autonomy::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_ExecutionRequirement_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::autonomy::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::ExecutionRequirement*>(nullptr));
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

char* pyramid_services_autonomy_backend_ExecutionRequirement_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryExecutionRequirement(data, size);
        auto json = pyramid::data_model::autonomy::toJson(value);
        char* out = static_cast<char*>(std::malloc(json.size() + 1));
        if (!out) return nullptr;
        std::memcpy(out, json.c_str(), json.size() + 1);
        return out;
    } catch (...) {
        return nullptr;
    }
}

void* pyramid_services_autonomy_backend_ExecutionRun_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto value = pyramid::data_model::autonomy::fromJson(std::string(json ? json : ""), static_cast<pyramid::data_model::ExecutionRun*>(nullptr));
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

char* pyramid_services_autonomy_backend_ExecutionRun_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto value = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryExecutionRun(data, size);
        auto json = pyramid::data_model::autonomy::toJson(value);
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
                values.push_back(pyramid::data_model::autonomy::fromJson(item.dump(), static_cast<pyramid::data_model::Capabilities*>(nullptr)));
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
            arr.push_back(nlohmann::json::parse(pyramid::data_model::autonomy::toJson(item)));
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

void* pyramid_services_autonomy_backend_PlanningRequirementArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::data_model::PlanningRequirement> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::data_model::autonomy::fromJson(item.dump(), static_cast<pyramid::data_model::PlanningRequirement*>(nullptr)));
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

char* pyramid_services_autonomy_backend_PlanningRequirementArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryPlanningRequirementArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::data_model::autonomy::toJson(item)));
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

void* pyramid_services_autonomy_backend_ExecutionRequirementArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::data_model::ExecutionRequirement> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::data_model::autonomy::fromJson(item.dump(), static_cast<pyramid::data_model::ExecutionRequirement*>(nullptr)));
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

char* pyramid_services_autonomy_backend_ExecutionRequirementArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryExecutionRequirementArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::data_model::autonomy::toJson(item)));
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

void* pyramid_services_autonomy_backend_PlanArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::data_model::Plan> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::data_model::autonomy::fromJson(item.dump(), static_cast<pyramid::data_model::Plan*>(nullptr)));
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

char* pyramid_services_autonomy_backend_PlanArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryPlanArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::data_model::autonomy::toJson(item)));
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

void* pyramid_services_autonomy_backend_ExecutionRunArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::data_model::ExecutionRun> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::data_model::autonomy::fromJson(item.dump(), static_cast<pyramid::data_model::ExecutionRun*>(nullptr)));
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

char* pyramid_services_autonomy_backend_ExecutionRunArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryExecutionRunArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::data_model::autonomy::toJson(item)));
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

void* pyramid_services_autonomy_backend_RequirementPlacementArray_to_flatbuffer_json(const char* json, size_t* size_out) {
    if (size_out) *size_out = 0;
    try {
        auto arr = nlohmann::json::parse(std::string(json ? json : "[]"));
        std::vector<pyramid::data_model::RequirementPlacement> values;
        if (arr.is_array()) {
            values.reserve(arr.size());
            for (const auto& item : arr) {
                values.push_back(pyramid::data_model::autonomy::fromJson(item.dump(), static_cast<pyramid::data_model::RequirementPlacement*>(nullptr)));
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

char* pyramid_services_autonomy_backend_RequirementPlacementArray_from_flatbuffer_json(const void* data, size_t size) {
    try {
        auto values = pyramid::services::autonomy_backend::flatbuffers_codec::fromBinaryRequirementPlacementArray(data, size);
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& item : values) {
            arr.push_back(nlohmann::json::parse(pyramid::data_model::autonomy::toJson(item)));
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
