// Auto-generated types header
// Generated from: pyramid.components.agra.mission_autonomy.services.provided.proto by generate_bindings.py (types)
// Namespace: pyramid::components::agra::mission_autonomy::services::provided
#pragma once

#include <cstdint>
#include <tl/optional.hpp>
#include <string>
#include <vector>
#include "pyramid_data_model_agra_types.hpp"
#include "pyramid_data_model_agra_port_grammar_types.hpp"

namespace pyramid::components::agra::mission_autonomy::services::provided {


struct MA_MissionPlanCommand_Service_Request {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_MissionPlanCommandMT> ma_mission_plan_command;
    tl::optional<pyramid::domain_model::agra::MA_MissionPlanCommandStatusMT> update;
};

struct MA_MissionPlanCommand_Service_Entity {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_MissionPlanCommandStatusMT> ma_mission_plan_command_status;
};

struct MA_MissionPlanActivationCommand_Service_Request {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_MissionPlanActivationCommandMT> ma_mission_plan_activation_command;
    tl::optional<pyramid::domain_model::agra::MA_MissionPlanActivationCommandStatusMT> update;
};

struct MA_MissionPlanActivationCommand_Service_Entity {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_MissionPlanActivationCommandStatusMT> ma_mission_plan_activation_command_status;
};

struct MA_PlanningFunctionSettingsCommand_Service_Request {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_PlanningFunctionSettingsCommandMT> ma_planning_function_settings_command;
    tl::optional<pyramid::domain_model::agra::MA_PlanningFunctionSettingsCommandStatusMT> update;
};

struct MA_PlanningFunctionSettingsCommand_Service_Entity {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_PlanningFunctionSettingsCommandStatusMT> ma_planning_function_settings_command_status;
};

struct MA_ApprovalRequest_Service_Request {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_ApprovalRequestMT> ma_approval_request;
    tl::optional<pyramid::domain_model::agra::MA_ApprovalRequestStatusMT> update;
};

struct MA_ApprovalRequest_Service_Entity {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_ApprovalRequestStatusMT> ma_approval_request_status;
};

} // namespace pyramid::components::agra::mission_autonomy::services::provided
