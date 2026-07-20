// Auto-generated data model JSON codec implementation
// Namespace: pyramid::components::agra::mission_autonomy::services::provided

#include "pyramid_components_agra_mission_autonomy_services_provided_codec.hpp"

#include <nlohmann/json.hpp>

#include "pyramid_data_model_agra_codec.hpp"
#include "pyramid_data_model_agra_port_grammar_codec.hpp"

namespace pyramid::components::agra::mission_autonomy::services::provided {

std::string toJson(const MA_MissionPlanCommand_Service_Request& msg) {
    nlohmann::json obj;
    if (msg.ma_mission_plan_command.has_value()) {
        obj["ma_mission_plan_command"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_mission_plan_command.value()));
    }
    if (msg.update.has_value()) {
        obj["update"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.update.value()));
    }
    return obj.dump();
}

MA_MissionPlanCommand_Service_Request fromJson(const std::string& s, MA_MissionPlanCommand_Service_Request* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_MissionPlanCommand_Service_Request msg;
    if (j.contains("ma_mission_plan_command")) {
        msg.ma_mission_plan_command = pyramid::domain_model::agra::fromJson(j["ma_mission_plan_command"].dump(), static_cast<pyramid::domain_model::agra::MA_MissionPlanCommandMT*>(nullptr));
    }
    if (j.contains("update")) {
        msg.update = pyramid::domain_model::agra::fromJson(j["update"].dump(), static_cast<pyramid::domain_model::agra::MA_MissionPlanCommandStatusMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_MissionPlanCommand_Service_Entity& msg) {
    nlohmann::json obj;
    if (msg.ma_mission_plan_command_status.has_value()) {
        obj["ma_mission_plan_command_status"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_mission_plan_command_status.value()));
    }
    return obj.dump();
}

MA_MissionPlanCommand_Service_Entity fromJson(const std::string& s, MA_MissionPlanCommand_Service_Entity* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_MissionPlanCommand_Service_Entity msg;
    if (j.contains("ma_mission_plan_command_status")) {
        msg.ma_mission_plan_command_status = pyramid::domain_model::agra::fromJson(j["ma_mission_plan_command_status"].dump(), static_cast<pyramid::domain_model::agra::MA_MissionPlanCommandStatusMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_MissionPlanActivationCommand_Service_Request& msg) {
    nlohmann::json obj;
    if (msg.ma_mission_plan_activation_command.has_value()) {
        obj["ma_mission_plan_activation_command"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_mission_plan_activation_command.value()));
    }
    if (msg.update.has_value()) {
        obj["update"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.update.value()));
    }
    return obj.dump();
}

MA_MissionPlanActivationCommand_Service_Request fromJson(const std::string& s, MA_MissionPlanActivationCommand_Service_Request* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_MissionPlanActivationCommand_Service_Request msg;
    if (j.contains("ma_mission_plan_activation_command")) {
        msg.ma_mission_plan_activation_command = pyramid::domain_model::agra::fromJson(j["ma_mission_plan_activation_command"].dump(), static_cast<pyramid::domain_model::agra::MA_MissionPlanActivationCommandMT*>(nullptr));
    }
    if (j.contains("update")) {
        msg.update = pyramid::domain_model::agra::fromJson(j["update"].dump(), static_cast<pyramid::domain_model::agra::MA_MissionPlanActivationCommandStatusMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_MissionPlanActivationCommand_Service_Entity& msg) {
    nlohmann::json obj;
    if (msg.ma_mission_plan_activation_command_status.has_value()) {
        obj["ma_mission_plan_activation_command_status"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_mission_plan_activation_command_status.value()));
    }
    return obj.dump();
}

MA_MissionPlanActivationCommand_Service_Entity fromJson(const std::string& s, MA_MissionPlanActivationCommand_Service_Entity* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_MissionPlanActivationCommand_Service_Entity msg;
    if (j.contains("ma_mission_plan_activation_command_status")) {
        msg.ma_mission_plan_activation_command_status = pyramid::domain_model::agra::fromJson(j["ma_mission_plan_activation_command_status"].dump(), static_cast<pyramid::domain_model::agra::MA_MissionPlanActivationCommandStatusMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_PlanningFunctionSettingsCommand_Service_Request& msg) {
    nlohmann::json obj;
    if (msg.ma_planning_function_settings_command.has_value()) {
        obj["ma_planning_function_settings_command"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_planning_function_settings_command.value()));
    }
    if (msg.update.has_value()) {
        obj["update"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.update.value()));
    }
    return obj.dump();
}

MA_PlanningFunctionSettingsCommand_Service_Request fromJson(const std::string& s, MA_PlanningFunctionSettingsCommand_Service_Request* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_PlanningFunctionSettingsCommand_Service_Request msg;
    if (j.contains("ma_planning_function_settings_command")) {
        msg.ma_planning_function_settings_command = pyramid::domain_model::agra::fromJson(j["ma_planning_function_settings_command"].dump(), static_cast<pyramid::domain_model::agra::MA_PlanningFunctionSettingsCommandMT*>(nullptr));
    }
    if (j.contains("update")) {
        msg.update = pyramid::domain_model::agra::fromJson(j["update"].dump(), static_cast<pyramid::domain_model::agra::MA_PlanningFunctionSettingsCommandStatusMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_PlanningFunctionSettingsCommand_Service_Entity& msg) {
    nlohmann::json obj;
    if (msg.ma_planning_function_settings_command_status.has_value()) {
        obj["ma_planning_function_settings_command_status"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_planning_function_settings_command_status.value()));
    }
    return obj.dump();
}

MA_PlanningFunctionSettingsCommand_Service_Entity fromJson(const std::string& s, MA_PlanningFunctionSettingsCommand_Service_Entity* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_PlanningFunctionSettingsCommand_Service_Entity msg;
    if (j.contains("ma_planning_function_settings_command_status")) {
        msg.ma_planning_function_settings_command_status = pyramid::domain_model::agra::fromJson(j["ma_planning_function_settings_command_status"].dump(), static_cast<pyramid::domain_model::agra::MA_PlanningFunctionSettingsCommandStatusMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_ApprovalRequest_Service_Request& msg) {
    nlohmann::json obj;
    if (msg.ma_approval_request.has_value()) {
        obj["ma_approval_request"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_approval_request.value()));
    }
    if (msg.update.has_value()) {
        obj["update"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.update.value()));
    }
    return obj.dump();
}

MA_ApprovalRequest_Service_Request fromJson(const std::string& s, MA_ApprovalRequest_Service_Request* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_ApprovalRequest_Service_Request msg;
    if (j.contains("ma_approval_request")) {
        msg.ma_approval_request = pyramid::domain_model::agra::fromJson(j["ma_approval_request"].dump(), static_cast<pyramid::domain_model::agra::MA_ApprovalRequestMT*>(nullptr));
    }
    if (j.contains("update")) {
        msg.update = pyramid::domain_model::agra::fromJson(j["update"].dump(), static_cast<pyramid::domain_model::agra::MA_ApprovalRequestStatusMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_ApprovalRequest_Service_Entity& msg) {
    nlohmann::json obj;
    if (msg.ma_approval_request_status.has_value()) {
        obj["ma_approval_request_status"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_approval_request_status.value()));
    }
    return obj.dump();
}

MA_ApprovalRequest_Service_Entity fromJson(const std::string& s, MA_ApprovalRequest_Service_Entity* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_ApprovalRequest_Service_Entity msg;
    if (j.contains("ma_approval_request_status")) {
        msg.ma_approval_request_status = pyramid::domain_model::agra::fromJson(j["ma_approval_request_status"].dump(), static_cast<pyramid::domain_model::agra::MA_ApprovalRequestStatusMT*>(nullptr));
    }
    return msg;
}

} // namespace pyramid::components::agra::mission_autonomy::services::provided
