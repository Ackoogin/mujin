// Auto-generated data model JSON codec implementation
// Namespace: pyramid::data_model::autonomy

#include "pyramid_data_model_autonomy_codec.hpp"

#include <nlohmann/json.hpp>

#include "pyramid_data_model_base_codec.hpp"
#include "pyramid_data_model_common_codec.hpp"

namespace pyramid::data_model::autonomy {

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

std::string toString(PlanningExecutionMode v) {
    switch (v) {
        case PlanningExecutionMode::Unspecified: return "PLANNING_EXECUTION_MODE_UNSPECIFIED";
        case PlanningExecutionMode::PlanAndExecute: return "PLANNING_EXECUTION_MODE_PLAN_AND_EXECUTE";
        case PlanningExecutionMode::PlanOnly: return "PLANNING_EXECUTION_MODE_PLAN_ONLY";
        case PlanningExecutionMode::ExecuteApprovedPlan: return "PLANNING_EXECUTION_MODE_EXECUTE_APPROVED_PLAN";
    }
    return "PLANNING_EXECUTION_MODE_UNSPECIFIED";
}

PlanningExecutionMode planningExecutionModeFromString(const std::string& s) {
    if (s == "PLANNING_EXECUTION_MODE_UNSPECIFIED") return PlanningExecutionMode::Unspecified;
    if (s == "PLANNING_EXECUTION_MODE_PLAN_AND_EXECUTE") return PlanningExecutionMode::PlanAndExecute;
    if (s == "PLANNING_EXECUTION_MODE_PLAN_ONLY") return PlanningExecutionMode::PlanOnly;
    if (s == "PLANNING_EXECUTION_MODE_EXECUTE_APPROVED_PLAN") return PlanningExecutionMode::ExecuteApprovedPlan;
    return PlanningExecutionMode::Unspecified;
}

std::string toString(PlanningExecutionState v) {
    switch (v) {
        case PlanningExecutionState::Unspecified: return "PLANNING_EXECUTION_STATE_UNSPECIFIED";
        case PlanningExecutionState::Accepted: return "PLANNING_EXECUTION_STATE_ACCEPTED";
        case PlanningExecutionState::Planning: return "PLANNING_EXECUTION_STATE_PLANNING";
        case PlanningExecutionState::Executing: return "PLANNING_EXECUTION_STATE_EXECUTING";
        case PlanningExecutionState::WaitingForComponents: return "PLANNING_EXECUTION_STATE_WAITING_FOR_COMPONENTS";
        case PlanningExecutionState::Achieved: return "PLANNING_EXECUTION_STATE_ACHIEVED";
        case PlanningExecutionState::Failed: return "PLANNING_EXECUTION_STATE_FAILED";
        case PlanningExecutionState::Cancelled: return "PLANNING_EXECUTION_STATE_CANCELLED";
    }
    return "PLANNING_EXECUTION_STATE_UNSPECIFIED";
}

PlanningExecutionState planningExecutionStateFromString(const std::string& s) {
    if (s == "PLANNING_EXECUTION_STATE_UNSPECIFIED") return PlanningExecutionState::Unspecified;
    if (s == "PLANNING_EXECUTION_STATE_ACCEPTED") return PlanningExecutionState::Accepted;
    if (s == "PLANNING_EXECUTION_STATE_PLANNING") return PlanningExecutionState::Planning;
    if (s == "PLANNING_EXECUTION_STATE_EXECUTING") return PlanningExecutionState::Executing;
    if (s == "PLANNING_EXECUTION_STATE_WAITING_FOR_COMPONENTS") return PlanningExecutionState::WaitingForComponents;
    if (s == "PLANNING_EXECUTION_STATE_ACHIEVED") return PlanningExecutionState::Achieved;
    if (s == "PLANNING_EXECUTION_STATE_FAILED") return PlanningExecutionState::Failed;
    if (s == "PLANNING_EXECUTION_STATE_CANCELLED") return PlanningExecutionState::Cancelled;
    return PlanningExecutionState::Unspecified;
}

std::string toString(RequirementPlacementOperation v) {
    switch (v) {
        case RequirementPlacementOperation::Unspecified: return "REQUIREMENT_PLACEMENT_OPERATION_UNSPECIFIED";
        case RequirementPlacementOperation::CreateRequirement: return "REQUIREMENT_PLACEMENT_OPERATION_CREATE_REQUIREMENT";
        case RequirementPlacementOperation::ReadRequirement: return "REQUIREMENT_PLACEMENT_OPERATION_READ_REQUIREMENT";
        case RequirementPlacementOperation::UpdateRequirement: return "REQUIREMENT_PLACEMENT_OPERATION_UPDATE_REQUIREMENT";
        case RequirementPlacementOperation::DeleteRequirement: return "REQUIREMENT_PLACEMENT_OPERATION_DELETE_REQUIREMENT";
        case RequirementPlacementOperation::ReadProduct: return "REQUIREMENT_PLACEMENT_OPERATION_READ_PRODUCT";
        case RequirementPlacementOperation::ReadCapability: return "REQUIREMENT_PLACEMENT_OPERATION_READ_CAPABILITY";
    }
    return "REQUIREMENT_PLACEMENT_OPERATION_UNSPECIFIED";
}

RequirementPlacementOperation requirementPlacementOperationFromString(const std::string& s) {
    if (s == "REQUIREMENT_PLACEMENT_OPERATION_UNSPECIFIED") return RequirementPlacementOperation::Unspecified;
    if (s == "REQUIREMENT_PLACEMENT_OPERATION_CREATE_REQUIREMENT") return RequirementPlacementOperation::CreateRequirement;
    if (s == "REQUIREMENT_PLACEMENT_OPERATION_READ_REQUIREMENT") return RequirementPlacementOperation::ReadRequirement;
    if (s == "REQUIREMENT_PLACEMENT_OPERATION_UPDATE_REQUIREMENT") return RequirementPlacementOperation::UpdateRequirement;
    if (s == "REQUIREMENT_PLACEMENT_OPERATION_DELETE_REQUIREMENT") return RequirementPlacementOperation::DeleteRequirement;
    if (s == "REQUIREMENT_PLACEMENT_OPERATION_READ_PRODUCT") return RequirementPlacementOperation::ReadProduct;
    if (s == "REQUIREMENT_PLACEMENT_OPERATION_READ_CAPABILITY") return RequirementPlacementOperation::ReadCapability;
    return RequirementPlacementOperation::Unspecified;
}

std::string toJson(const RequirementReference& msg) {
    nlohmann::json obj;
    obj["requirement_id"] = msg.requirement_id;
    obj["component_name"] = msg.component_name;
    obj["service_name"] = msg.service_name;
    obj["type_name"] = msg.type_name;
    return obj.dump();
}

RequirementReference fromJson(const std::string& s, RequirementReference* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    RequirementReference msg;
    if (j.contains("requirement_id")) msg.requirement_id = j["requirement_id"].get<std::string>();
    if (j.contains("component_name")) msg.component_name = j["component_name"].get<std::string>();
    if (j.contains("service_name")) msg.service_name = j["service_name"].get<std::string>();
    if (j.contains("type_name")) msg.type_name = j["type_name"].get<std::string>();
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

std::string toJson(const PlanningPolicy& msg) {
    nlohmann::json obj;
    obj["max_replans"] = msg.max_replans;
    obj["enable_replanning"] = msg.enable_replanning;
    obj["max_concurrent_placements"] = msg.max_concurrent_placements;
    return obj.dump();
}

PlanningPolicy fromJson(const std::string& s, PlanningPolicy* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    PlanningPolicy msg;
    if (j.contains("max_replans")) msg.max_replans = j["max_replans"].get<uint32_t>();
    if (j.contains("enable_replanning")) msg.enable_replanning = j["enable_replanning"].get<bool>();
    if (j.contains("max_concurrent_placements")) msg.max_concurrent_placements = j["max_concurrent_placements"].get<uint32_t>();
    return msg;
}

std::string toJson(const PlanningGoal& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    obj["name"] = msg.name;
    if (msg.requirement.has_value()) {
        obj["requirement"] = nlohmann::json::parse(toJson(msg.requirement.value()));
    }
    if (msg.expression.has_value()) {
        obj["expression"] = msg.expression.value();
    }
    return obj.dump();
}

PlanningGoal fromJson(const std::string& s, PlanningGoal* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    PlanningGoal msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("name")) msg.name = j["name"].get<std::string>();
    if (j.contains("requirement")) {
        msg.requirement = fromJson(j["requirement"].dump(), static_cast<RequirementReference*>(nullptr));
    }
    if (j.contains("expression")) {
        msg.expression = j["expression"].get<std::string>();
    }
    return msg;
}

std::string toJson(const PlanningExecutionRequirement& msg) {
    nlohmann::json obj;
    obj["base"] = nlohmann::json::parse(toJson(msg.base));
    obj["status"] = nlohmann::json::parse(toJson(msg.status));
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.upstream_requirement) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["upstream_requirement"] = arr;
    }
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.goal) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["goal"] = arr;
    }
    obj["policy"] = nlohmann::json::parse(toJson(msg.policy));
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.available_agents) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["available_agents"] = arr;
    }
    obj["mode"] = toString(msg.mode);
    obj["approved_plan_id"] = msg.approved_plan_id;
    return obj.dump();
}

PlanningExecutionRequirement fromJson(const std::string& s, PlanningExecutionRequirement* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    PlanningExecutionRequirement msg;
    if (j.contains("base")) msg.base = fromJson(j["base"].dump(), static_cast<pyramid::data_model::common::Entity*>(nullptr));
    if (j.contains("status")) msg.status = fromJson(j["status"].dump(), static_cast<pyramid::data_model::common::Achievement*>(nullptr));
    if (j.contains("upstream_requirement")) {
        for (const auto& v : j["upstream_requirement"]) {
            msg.upstream_requirement.push_back(fromJson(v.dump(), static_cast<RequirementReference*>(nullptr)));
        }
    }
    if (j.contains("goal")) {
        for (const auto& v : j["goal"]) {
            msg.goal.push_back(fromJson(v.dump(), static_cast<PlanningGoal*>(nullptr)));
        }
    }
    if (j.contains("policy")) msg.policy = fromJson(j["policy"].dump(), static_cast<PlanningPolicy*>(nullptr));
    if (j.contains("available_agents")) {
        for (const auto& v : j["available_agents"]) {
            msg.available_agents.push_back(fromJson(v.dump(), static_cast<AgentState*>(nullptr)));
        }
    }
    if (j.contains("mode")) msg.mode = planningExecutionModeFromString(j["mode"].get<std::string>());
    if (j.contains("approved_plan_id")) msg.approved_plan_id = j["approved_plan_id"].get<std::string>();
    return msg;
}

std::string toJson(const WorldFactUpdate& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["entity_source"] = msg.entity_source;
    obj["key"] = msg.key;
    obj["value"] = msg.value;
    obj["source"] = msg.source;
    obj["authority"] = toString(msg.authority);
    return obj.dump();
}

WorldFactUpdate fromJson(const std::string& s, WorldFactUpdate* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    WorldFactUpdate msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("entity_source")) msg.entity_source = j["entity_source"].get<std::string>();
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
        for (const auto& v : msg.fact_update) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["fact_update"] = arr;
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
    if (j.contains("fact_update")) {
        for (const auto& v : j["fact_update"]) {
            msg.fact_update.push_back(fromJson(v.dump(), static_cast<WorldFactUpdate*>(nullptr)));
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
    obj["supports_plan_only"] = msg.supports_plan_only;
    obj["supports_plan_and_execute"] = msg.supports_plan_and_execute;
    obj["supports_execute_approved_plan"] = msg.supports_execute_approved_plan;
    obj["supports_replanning"] = msg.supports_replanning;
    obj["supports_typed_component_requirement_placement"] = msg.supports_typed_component_requirement_placement;
    obj["supports_state_update_ingress"] = msg.supports_state_update_ingress;
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
    if (j.contains("supports_plan_only")) msg.supports_plan_only = j["supports_plan_only"].get<bool>();
    if (j.contains("supports_plan_and_execute")) msg.supports_plan_and_execute = j["supports_plan_and_execute"].get<bool>();
    if (j.contains("supports_execute_approved_plan")) msg.supports_execute_approved_plan = j["supports_execute_approved_plan"].get<bool>();
    if (j.contains("supports_replanning")) msg.supports_replanning = j["supports_replanning"].get<bool>();
    if (j.contains("supports_typed_component_requirement_placement")) msg.supports_typed_component_requirement_placement = j["supports_typed_component_requirement_placement"].get<bool>();
    if (j.contains("supports_state_update_ingress")) msg.supports_state_update_ingress = j["supports_state_update_ingress"].get<bool>();
    return msg;
}

std::string toJson(const PlannedComponentInteraction& msg) {
    nlohmann::json obj;
    obj["target_component"] = msg.target_component;
    obj["target_service"] = msg.target_service;
    obj["target_type"] = msg.target_type;
    obj["operation"] = toString(msg.operation);
    return obj.dump();
}

PlannedComponentInteraction fromJson(const std::string& s, PlannedComponentInteraction* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    PlannedComponentInteraction msg;
    if (j.contains("target_component")) msg.target_component = j["target_component"].get<std::string>();
    if (j.contains("target_service")) msg.target_service = j["target_service"].get<std::string>();
    if (j.contains("target_type")) msg.target_type = j["target_type"].get<std::string>();
    if (j.contains("operation")) msg.operation = requirementPlacementOperationFromString(j["operation"].get<std::string>());
    return msg;
}

std::string toJson(const PlanStep& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    obj["sequence_number"] = msg.sequence_number;
    obj["action_name"] = msg.action_name;
    obj["signature"] = msg.signature;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.interaction) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["interaction"] = arr;
    }
    return obj.dump();
}

PlanStep fromJson(const std::string& s, PlanStep* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    PlanStep msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("sequence_number")) msg.sequence_number = j["sequence_number"].get<uint32_t>();
    if (j.contains("action_name")) msg.action_name = j["action_name"].get<std::string>();
    if (j.contains("signature")) msg.signature = j["signature"].get<std::string>();
    if (j.contains("interaction")) {
        for (const auto& v : j["interaction"]) {
            msg.interaction.push_back(fromJson(v.dump(), static_cast<PlannedComponentInteraction*>(nullptr)));
        }
    }
    return msg;
}

std::string toJson(const Plan& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    obj["planning_execution_requirement_id"] = msg.planning_execution_requirement_id;
    obj["backend_id"] = msg.backend_id;
    obj["world_version"] = msg.world_version;
    obj["replan_count"] = msg.replan_count;
    obj["plan_success"] = msg.plan_success;
    obj["solve_time_ms"] = msg.solve_time_ms;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.step) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["step"] = arr;
    }
    obj["compiled_bt_xml"] = msg.compiled_bt_xml;
    if (msg.predicted_quality.has_value()) {
        obj["predicted_quality"] = msg.predicted_quality.value();
    }
    return obj.dump();
}

Plan fromJson(const std::string& s, Plan* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    Plan msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("planning_execution_requirement_id")) msg.planning_execution_requirement_id = j["planning_execution_requirement_id"].get<std::string>();
    if (j.contains("backend_id")) msg.backend_id = j["backend_id"].get<std::string>();
    if (j.contains("world_version")) msg.world_version = j["world_version"].get<uint64_t>();
    if (j.contains("replan_count")) msg.replan_count = j["replan_count"].get<uint32_t>();
    if (j.contains("plan_success")) msg.plan_success = j["plan_success"].get<bool>();
    if (j.contains("solve_time_ms")) msg.solve_time_ms = j["solve_time_ms"].get<double>();
    if (j.contains("step")) {
        for (const auto& v : j["step"]) {
            msg.step.push_back(fromJson(v.dump(), static_cast<PlanStep*>(nullptr)));
        }
    }
    if (j.contains("compiled_bt_xml")) msg.compiled_bt_xml = j["compiled_bt_xml"].get<std::string>();
    if (j.contains("predicted_quality")) {
        msg.predicted_quality = j["predicted_quality"].get<double>();
    }
    return msg;
}

std::string toJson(const RequirementPlacement& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    obj["planning_execution_requirement_id"] = msg.planning_execution_requirement_id;
    obj["plan_id"] = msg.plan_id;
    obj["plan_step_id"] = msg.plan_step_id;
    obj["target_component"] = msg.target_component;
    obj["target_service"] = msg.target_service;
    obj["target_type"] = msg.target_type;
    obj["operation"] = toString(msg.operation);
    obj["target_requirement_id"] = msg.target_requirement_id;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.related_entity_id) {
            arr.push_back(v);
        }
        obj["related_entity_id"] = arr;
    }
    obj["progress"] = toString(msg.progress);
    return obj.dump();
}

RequirementPlacement fromJson(const std::string& s, RequirementPlacement* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    RequirementPlacement msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("planning_execution_requirement_id")) msg.planning_execution_requirement_id = j["planning_execution_requirement_id"].get<std::string>();
    if (j.contains("plan_id")) msg.plan_id = j["plan_id"].get<std::string>();
    if (j.contains("plan_step_id")) msg.plan_step_id = j["plan_step_id"].get<std::string>();
    if (j.contains("target_component")) msg.target_component = j["target_component"].get<std::string>();
    if (j.contains("target_service")) msg.target_service = j["target_service"].get<std::string>();
    if (j.contains("target_type")) msg.target_type = j["target_type"].get<std::string>();
    if (j.contains("operation")) msg.operation = requirementPlacementOperationFromString(j["operation"].get<std::string>());
    if (j.contains("target_requirement_id")) msg.target_requirement_id = j["target_requirement_id"].get<std::string>();
    if (j.contains("related_entity_id")) {
        for (const auto& v : j["related_entity_id"]) {
            msg.related_entity_id.push_back(v.get<std::string>());
        }
    }
    if (j.contains("progress")) msg.progress = pyramid::data_model::common::progressFromString(j["progress"].get<std::string>());
    return msg;
}

std::string toJson(const ExecutionRun& msg) {
    nlohmann::json obj;
    if (msg.update_time.has_value()) {
        obj["update_time"] = msg.update_time.value();
    }
    obj["id"] = msg.id;
    obj["source"] = msg.source;
    obj["planning_execution_requirement_id"] = msg.planning_execution_requirement_id;
    obj["plan_id"] = msg.plan_id;
    obj["state"] = toString(msg.state);
    obj["achievement"] = nlohmann::json::parse(toJson(msg.achievement));
    obj["replan_count"] = msg.replan_count;
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& v : msg.outstanding_placement) {
            arr.push_back(nlohmann::json::parse(toJson(v)));
        }
        obj["outstanding_placement"] = arr;
    }
    return obj.dump();
}

ExecutionRun fromJson(const std::string& s, ExecutionRun* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    ExecutionRun msg;
    if (j.contains("update_time")) {
        msg.update_time = j["update_time"].get<double>();
    }
    if (j.contains("id")) msg.id = j["id"].get<std::string>();
    if (j.contains("source")) msg.source = j["source"].get<std::string>();
    if (j.contains("planning_execution_requirement_id")) msg.planning_execution_requirement_id = j["planning_execution_requirement_id"].get<std::string>();
    if (j.contains("plan_id")) msg.plan_id = j["plan_id"].get<std::string>();
    if (j.contains("state")) msg.state = planningExecutionStateFromString(j["state"].get<std::string>());
    if (j.contains("achievement")) msg.achievement = fromJson(j["achievement"].dump(), static_cast<pyramid::data_model::common::Achievement*>(nullptr));
    if (j.contains("replan_count")) msg.replan_count = j["replan_count"].get<uint32_t>();
    if (j.contains("outstanding_placement")) {
        for (const auto& v : j["outstanding_placement"]) {
            msg.outstanding_placement.push_back(fromJson(v.dump(), static_cast<RequirementPlacement*>(nullptr)));
        }
    }
    return msg;
}

} // namespace pyramid::data_model::autonomy
