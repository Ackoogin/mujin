// Auto-generated types header
// Generated from: pyramid.components.agra.information.services.provided.proto by generate_bindings.py (types)
// Namespace: pyramid::components::agra::information::services::provided
#pragma once

#include <cstdint>
#include <tl/optional.hpp>
#include <string>
#include <vector>
#include "pyramid_data_model_agra_types.hpp"

namespace pyramid::components::agra::information::services::provided {


struct MA_Action_Service_Information {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_ActionMT> ma_action;
};

struct MA_MissionPlan_Service_Information {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_MissionPlanMT> ma_mission_plan;
};

struct MA_PlanningFunction_Service_Information {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_PlanningFunctionMT> ma_planning_function;
};

struct MA_Response_Service_Information {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_ResponseMT> ma_response;
};

struct MA_Task_Service_Information {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_TaskMT> ma_task;
};

struct MissionContingencyAlert_Service_Information {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MissionContingencyAlertMT> mission_contingency_alert;
};

struct MA_ActionStatus_Service_Information {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_ActionStatusMT> ma_action_status;
};

struct MA_MissionPlanActivationStatus_Service_Information {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_MissionPlanActivationStatusMT> ma_mission_plan_activation_status;
};

struct MA_MissionPlanExecutionStatus_Service_Information {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_MissionPlanExecutionStatusMT> ma_mission_plan_execution_status;
};

struct MA_PlanningFunctionStatus_Service_Information {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_PlanningFunctionStatusMT> ma_planning_function_status;
};

struct MA_TaskStatus_Service_Information {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_TaskStatusMT> ma_task_status;
};

struct MA_ApprovalPolicy_Service_Information {
    // oneof payload
    tl::optional<pyramid::domain_model::agra::MA_ApprovalPolicyMT> ma_approval_policy;
};

struct Empty {
};

} // namespace pyramid::components::agra::information::services::provided
