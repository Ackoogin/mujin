#pragma once

#include "pyramid_data_model_types.hpp"
#include "pyramid_components_agra_mission_autonomy_services_provided_types.hpp"
#include "pyramid_components_agra_mission_autonomy_services_provided_cabi.h"
#include "pyramid_data_model_agra_cabi_marshal.hpp"
#include "pyramid_data_model_agra_port_grammar_cabi_marshal.hpp"

namespace pyramid::cabi {

void to_c(const pyramid::components::agra::mission_autonomy::services::provided::MA_MissionPlanCommand_Service_Request& in, pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanCommand_Service_Request_c* out);
void from_c(const pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanCommand_Service_Request_c* in, pyramid::components::agra::mission_autonomy::services::provided::MA_MissionPlanCommand_Service_Request& out);

void to_c(const pyramid::components::agra::mission_autonomy::services::provided::MA_MissionPlanCommand_Service_Entity& in, pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanCommand_Service_Entity_c* out);
void from_c(const pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanCommand_Service_Entity_c* in, pyramid::components::agra::mission_autonomy::services::provided::MA_MissionPlanCommand_Service_Entity& out);

void to_c(const pyramid::components::agra::mission_autonomy::services::provided::MA_MissionPlanActivationCommand_Service_Request& in, pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanActivationCommand_Service_Request_c* out);
void from_c(const pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanActivationCommand_Service_Request_c* in, pyramid::components::agra::mission_autonomy::services::provided::MA_MissionPlanActivationCommand_Service_Request& out);

void to_c(const pyramid::components::agra::mission_autonomy::services::provided::MA_MissionPlanActivationCommand_Service_Entity& in, pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanActivationCommand_Service_Entity_c* out);
void from_c(const pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanActivationCommand_Service_Entity_c* in, pyramid::components::agra::mission_autonomy::services::provided::MA_MissionPlanActivationCommand_Service_Entity& out);

void to_c(const pyramid::components::agra::mission_autonomy::services::provided::MA_PlanningFunctionSettingsCommand_Service_Request& in, pyramid_components_agra_mission_autonomy_services_provided_MA_PlanningFunctionSettingsCommand_Service_Request_c* out);
void from_c(const pyramid_components_agra_mission_autonomy_services_provided_MA_PlanningFunctionSettingsCommand_Service_Request_c* in, pyramid::components::agra::mission_autonomy::services::provided::MA_PlanningFunctionSettingsCommand_Service_Request& out);

void to_c(const pyramid::components::agra::mission_autonomy::services::provided::MA_PlanningFunctionSettingsCommand_Service_Entity& in, pyramid_components_agra_mission_autonomy_services_provided_MA_PlanningFunctionSettingsCommand_Service_Entity_c* out);
void from_c(const pyramid_components_agra_mission_autonomy_services_provided_MA_PlanningFunctionSettingsCommand_Service_Entity_c* in, pyramid::components::agra::mission_autonomy::services::provided::MA_PlanningFunctionSettingsCommand_Service_Entity& out);

void to_c(const pyramid::components::agra::mission_autonomy::services::provided::MA_ApprovalRequest_Service_Request& in, pyramid_components_agra_mission_autonomy_services_provided_MA_ApprovalRequest_Service_Request_c* out);
void from_c(const pyramid_components_agra_mission_autonomy_services_provided_MA_ApprovalRequest_Service_Request_c* in, pyramid::components::agra::mission_autonomy::services::provided::MA_ApprovalRequest_Service_Request& out);

void to_c(const pyramid::components::agra::mission_autonomy::services::provided::MA_ApprovalRequest_Service_Entity& in, pyramid_components_agra_mission_autonomy_services_provided_MA_ApprovalRequest_Service_Entity_c* out);
void from_c(const pyramid_components_agra_mission_autonomy_services_provided_MA_ApprovalRequest_Service_Entity_c* in, pyramid::components::agra::mission_autonomy::services::provided::MA_ApprovalRequest_Service_Entity& out);

} // namespace pyramid::cabi
