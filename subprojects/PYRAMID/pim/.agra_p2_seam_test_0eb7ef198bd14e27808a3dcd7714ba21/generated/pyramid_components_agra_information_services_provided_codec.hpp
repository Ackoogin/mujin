// Auto-generated data model JSON codec header
// Generated from: pyramid.components.agra.information.services.provided.proto by generate_bindings.py (codec)
// Namespace: pyramid::components::agra::information::services::provided
#pragma once

#include "pyramid_components_agra_information_services_provided_types.hpp"
#include <string>

namespace pyramid::components::agra::information::services::provided {

// JSON codec
std::string toJson(const MA_Action_Service_Information& msg);
MA_Action_Service_Information fromJson(const std::string& s, MA_Action_Service_Information* /*tag*/ = nullptr);
std::string toJson(const MA_MissionPlan_Service_Information& msg);
MA_MissionPlan_Service_Information fromJson(const std::string& s, MA_MissionPlan_Service_Information* /*tag*/ = nullptr);
std::string toJson(const MA_PlanningFunction_Service_Information& msg);
MA_PlanningFunction_Service_Information fromJson(const std::string& s, MA_PlanningFunction_Service_Information* /*tag*/ = nullptr);
std::string toJson(const MA_Response_Service_Information& msg);
MA_Response_Service_Information fromJson(const std::string& s, MA_Response_Service_Information* /*tag*/ = nullptr);
std::string toJson(const MA_Task_Service_Information& msg);
MA_Task_Service_Information fromJson(const std::string& s, MA_Task_Service_Information* /*tag*/ = nullptr);
std::string toJson(const MissionContingencyAlert_Service_Information& msg);
MissionContingencyAlert_Service_Information fromJson(const std::string& s, MissionContingencyAlert_Service_Information* /*tag*/ = nullptr);
std::string toJson(const MA_ActionStatus_Service_Information& msg);
MA_ActionStatus_Service_Information fromJson(const std::string& s, MA_ActionStatus_Service_Information* /*tag*/ = nullptr);
std::string toJson(const MA_MissionPlanActivationStatus_Service_Information& msg);
MA_MissionPlanActivationStatus_Service_Information fromJson(const std::string& s, MA_MissionPlanActivationStatus_Service_Information* /*tag*/ = nullptr);
std::string toJson(const MA_MissionPlanExecutionStatus_Service_Information& msg);
MA_MissionPlanExecutionStatus_Service_Information fromJson(const std::string& s, MA_MissionPlanExecutionStatus_Service_Information* /*tag*/ = nullptr);
std::string toJson(const MA_PlanningFunctionStatus_Service_Information& msg);
MA_PlanningFunctionStatus_Service_Information fromJson(const std::string& s, MA_PlanningFunctionStatus_Service_Information* /*tag*/ = nullptr);
std::string toJson(const MA_TaskStatus_Service_Information& msg);
MA_TaskStatus_Service_Information fromJson(const std::string& s, MA_TaskStatus_Service_Information* /*tag*/ = nullptr);
std::string toJson(const MA_ApprovalPolicy_Service_Information& msg);
MA_ApprovalPolicy_Service_Information fromJson(const std::string& s, MA_ApprovalPolicy_Service_Information* /*tag*/ = nullptr);
std::string toJson(const Empty& msg);
Empty fromJson(const std::string& s, Empty* /*tag*/ = nullptr);

} // namespace pyramid::components::agra::information::services::provided
