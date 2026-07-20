// Auto-generated data model JSON codec header
// Generated from: pyramid.components.agra.mission_autonomy.services.provided.proto by generate_bindings.py (codec)
// Namespace: pyramid::components::agra::mission_autonomy::services::provided
#pragma once

#include "pyramid_components_agra_mission_autonomy_services_provided_types.hpp"
#include <string>

namespace pyramid::components::agra::mission_autonomy::services::provided {

// JSON codec
std::string toJson(const MA_MissionPlanCommand_Service_Request& msg);
MA_MissionPlanCommand_Service_Request fromJson(const std::string& s, MA_MissionPlanCommand_Service_Request* /*tag*/ = nullptr);
std::string toJson(const MA_MissionPlanCommand_Service_Entity& msg);
MA_MissionPlanCommand_Service_Entity fromJson(const std::string& s, MA_MissionPlanCommand_Service_Entity* /*tag*/ = nullptr);
std::string toJson(const MA_MissionPlanActivationCommand_Service_Request& msg);
MA_MissionPlanActivationCommand_Service_Request fromJson(const std::string& s, MA_MissionPlanActivationCommand_Service_Request* /*tag*/ = nullptr);
std::string toJson(const MA_MissionPlanActivationCommand_Service_Entity& msg);
MA_MissionPlanActivationCommand_Service_Entity fromJson(const std::string& s, MA_MissionPlanActivationCommand_Service_Entity* /*tag*/ = nullptr);
std::string toJson(const MA_PlanningFunctionSettingsCommand_Service_Request& msg);
MA_PlanningFunctionSettingsCommand_Service_Request fromJson(const std::string& s, MA_PlanningFunctionSettingsCommand_Service_Request* /*tag*/ = nullptr);
std::string toJson(const MA_PlanningFunctionSettingsCommand_Service_Entity& msg);
MA_PlanningFunctionSettingsCommand_Service_Entity fromJson(const std::string& s, MA_PlanningFunctionSettingsCommand_Service_Entity* /*tag*/ = nullptr);
std::string toJson(const MA_ApprovalRequest_Service_Request& msg);
MA_ApprovalRequest_Service_Request fromJson(const std::string& s, MA_ApprovalRequest_Service_Request* /*tag*/ = nullptr);
std::string toJson(const MA_ApprovalRequest_Service_Entity& msg);
MA_ApprovalRequest_Service_Entity fromJson(const std::string& s, MA_ApprovalRequest_Service_Entity* /*tag*/ = nullptr);

} // namespace pyramid::components::agra::mission_autonomy::services::provided
