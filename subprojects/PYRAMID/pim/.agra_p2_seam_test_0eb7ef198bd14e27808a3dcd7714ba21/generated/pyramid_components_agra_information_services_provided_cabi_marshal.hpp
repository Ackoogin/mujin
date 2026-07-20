#pragma once

#include "pyramid_data_model_types.hpp"
#include "pyramid_components_agra_information_services_provided_types.hpp"
#include "pyramid_components_agra_information_services_provided_cabi.h"
#include "pyramid_data_model_agra_cabi_marshal.hpp"

namespace pyramid::cabi {

void to_c(const pyramid::components::agra::information::services::provided::MA_Action_Service_Information& in, pyramid_components_agra_information_services_provided_MA_Action_Service_Information_c* out);
void from_c(const pyramid_components_agra_information_services_provided_MA_Action_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_Action_Service_Information& out);

void to_c(const pyramid::components::agra::information::services::provided::MA_MissionPlan_Service_Information& in, pyramid_components_agra_information_services_provided_MA_MissionPlan_Service_Information_c* out);
void from_c(const pyramid_components_agra_information_services_provided_MA_MissionPlan_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_MissionPlan_Service_Information& out);

void to_c(const pyramid::components::agra::information::services::provided::MA_PlanningFunction_Service_Information& in, pyramid_components_agra_information_services_provided_MA_PlanningFunction_Service_Information_c* out);
void from_c(const pyramid_components_agra_information_services_provided_MA_PlanningFunction_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_PlanningFunction_Service_Information& out);

void to_c(const pyramid::components::agra::information::services::provided::MA_Response_Service_Information& in, pyramid_components_agra_information_services_provided_MA_Response_Service_Information_c* out);
void from_c(const pyramid_components_agra_information_services_provided_MA_Response_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_Response_Service_Information& out);

void to_c(const pyramid::components::agra::information::services::provided::MA_Task_Service_Information& in, pyramid_components_agra_information_services_provided_MA_Task_Service_Information_c* out);
void from_c(const pyramid_components_agra_information_services_provided_MA_Task_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_Task_Service_Information& out);

void to_c(const pyramid::components::agra::information::services::provided::MissionContingencyAlert_Service_Information& in, pyramid_components_agra_information_services_provided_MissionContingencyAlert_Service_Information_c* out);
void from_c(const pyramid_components_agra_information_services_provided_MissionContingencyAlert_Service_Information_c* in, pyramid::components::agra::information::services::provided::MissionContingencyAlert_Service_Information& out);

void to_c(const pyramid::components::agra::information::services::provided::MA_ActionStatus_Service_Information& in, pyramid_components_agra_information_services_provided_MA_ActionStatus_Service_Information_c* out);
void from_c(const pyramid_components_agra_information_services_provided_MA_ActionStatus_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_ActionStatus_Service_Information& out);

void to_c(const pyramid::components::agra::information::services::provided::MA_MissionPlanActivationStatus_Service_Information& in, pyramid_components_agra_information_services_provided_MA_MissionPlanActivationStatus_Service_Information_c* out);
void from_c(const pyramid_components_agra_information_services_provided_MA_MissionPlanActivationStatus_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_MissionPlanActivationStatus_Service_Information& out);

void to_c(const pyramid::components::agra::information::services::provided::MA_MissionPlanExecutionStatus_Service_Information& in, pyramid_components_agra_information_services_provided_MA_MissionPlanExecutionStatus_Service_Information_c* out);
void from_c(const pyramid_components_agra_information_services_provided_MA_MissionPlanExecutionStatus_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_MissionPlanExecutionStatus_Service_Information& out);

void to_c(const pyramid::components::agra::information::services::provided::MA_PlanningFunctionStatus_Service_Information& in, pyramid_components_agra_information_services_provided_MA_PlanningFunctionStatus_Service_Information_c* out);
void from_c(const pyramid_components_agra_information_services_provided_MA_PlanningFunctionStatus_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_PlanningFunctionStatus_Service_Information& out);

void to_c(const pyramid::components::agra::information::services::provided::MA_TaskStatus_Service_Information& in, pyramid_components_agra_information_services_provided_MA_TaskStatus_Service_Information_c* out);
void from_c(const pyramid_components_agra_information_services_provided_MA_TaskStatus_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_TaskStatus_Service_Information& out);

void to_c(const pyramid::components::agra::information::services::provided::MA_ApprovalPolicy_Service_Information& in, pyramid_components_agra_information_services_provided_MA_ApprovalPolicy_Service_Information_c* out);
void from_c(const pyramid_components_agra_information_services_provided_MA_ApprovalPolicy_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_ApprovalPolicy_Service_Information& out);

void to_c(const pyramid::components::agra::information::services::provided::Empty& in, pyramid_components_agra_information_services_provided_Empty_c* out);
void from_c(const pyramid_components_agra_information_services_provided_Empty_c* in, pyramid::components::agra::information::services::provided::Empty& out);

} // namespace pyramid::cabi
