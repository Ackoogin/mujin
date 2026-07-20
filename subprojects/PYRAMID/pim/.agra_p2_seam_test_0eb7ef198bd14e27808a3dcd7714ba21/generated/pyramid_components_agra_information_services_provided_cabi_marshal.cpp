// Auto-generated C-ABI marshalling
// Ownership: to_c malloc-deep-copies all variable-length data into
// the C struct; from_c deep-copies into native values; _free releases
// everything allocated for the C struct. One full copy per direction.

#include "pyramid_components_agra_information_services_provided_cabi_marshal.hpp"
#include <pcl/pcl_alloc.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

void dup_str(pyramid_str_t& out, const std::string& in) {
  if (in.empty()) {
    out.ptr = nullptr;
    out.len = 0;
    return;
  }
  out.len = static_cast<uint32_t>(in.size());
  out.ptr = static_cast<const char*>(pcl_alloc(out.len));
  std::memcpy(const_cast<char*>(out.ptr), in.data(), out.len);
}

void free_str(pyramid_str_t& s) {
  if (s.ptr) {
    pcl_free(const_cast<char*>(s.ptr));
    s.ptr = nullptr;
    s.len = 0;
  }
}

} // namespace

namespace pyramid::cabi {

void to_c(const pyramid::components::agra::information::services::provided::MA_Action_Service_Information& in, pyramid_components_agra_information_services_provided_MA_Action_Service_Information_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_action = in.ma_action.has_value() ? 1u : 0u;
  if (in.ma_action) {
    to_c(*in.ma_action, &out->ma_action);
  }
}

void from_c(const pyramid_components_agra_information_services_provided_MA_Action_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_Action_Service_Information& out) {
  if (in->has_ma_action) {
    out.ma_action.emplace();
    from_c(&in->ma_action, *out.ma_action);
  } else {
    out.ma_action = tl::nullopt;
  }
}

void _free_MA_Action_Service_Information(pyramid_components_agra_information_services_provided_MA_Action_Service_Information_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_action) {
    pyramid_data_model_agra_MA_ActionMT_c_free(&value->ma_action);
    value->has_ma_action = 0;
  }
}

void to_c(const pyramid::components::agra::information::services::provided::MA_MissionPlan_Service_Information& in, pyramid_components_agra_information_services_provided_MA_MissionPlan_Service_Information_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_mission_plan = in.ma_mission_plan.has_value() ? 1u : 0u;
  if (in.ma_mission_plan) {
    to_c(*in.ma_mission_plan, &out->ma_mission_plan);
  }
}

void from_c(const pyramid_components_agra_information_services_provided_MA_MissionPlan_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_MissionPlan_Service_Information& out) {
  if (in->has_ma_mission_plan) {
    out.ma_mission_plan.emplace();
    from_c(&in->ma_mission_plan, *out.ma_mission_plan);
  } else {
    out.ma_mission_plan = tl::nullopt;
  }
}

void _free_MA_MissionPlan_Service_Information(pyramid_components_agra_information_services_provided_MA_MissionPlan_Service_Information_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_mission_plan) {
    pyramid_data_model_agra_MA_MissionPlanMT_c_free(&value->ma_mission_plan);
    value->has_ma_mission_plan = 0;
  }
}

void to_c(const pyramid::components::agra::information::services::provided::MA_PlanningFunction_Service_Information& in, pyramid_components_agra_information_services_provided_MA_PlanningFunction_Service_Information_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_planning_function = in.ma_planning_function.has_value() ? 1u : 0u;
  if (in.ma_planning_function) {
    to_c(*in.ma_planning_function, &out->ma_planning_function);
  }
}

void from_c(const pyramid_components_agra_information_services_provided_MA_PlanningFunction_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_PlanningFunction_Service_Information& out) {
  if (in->has_ma_planning_function) {
    out.ma_planning_function.emplace();
    from_c(&in->ma_planning_function, *out.ma_planning_function);
  } else {
    out.ma_planning_function = tl::nullopt;
  }
}

void _free_MA_PlanningFunction_Service_Information(pyramid_components_agra_information_services_provided_MA_PlanningFunction_Service_Information_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_planning_function) {
    pyramid_data_model_agra_MA_PlanningFunctionMT_c_free(&value->ma_planning_function);
    value->has_ma_planning_function = 0;
  }
}

void to_c(const pyramid::components::agra::information::services::provided::MA_Response_Service_Information& in, pyramid_components_agra_information_services_provided_MA_Response_Service_Information_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_response = in.ma_response.has_value() ? 1u : 0u;
  if (in.ma_response) {
    to_c(*in.ma_response, &out->ma_response);
  }
}

void from_c(const pyramid_components_agra_information_services_provided_MA_Response_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_Response_Service_Information& out) {
  if (in->has_ma_response) {
    out.ma_response.emplace();
    from_c(&in->ma_response, *out.ma_response);
  } else {
    out.ma_response = tl::nullopt;
  }
}

void _free_MA_Response_Service_Information(pyramid_components_agra_information_services_provided_MA_Response_Service_Information_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_response) {
    pyramid_data_model_agra_MA_ResponseMT_c_free(&value->ma_response);
    value->has_ma_response = 0;
  }
}

void to_c(const pyramid::components::agra::information::services::provided::MA_Task_Service_Information& in, pyramid_components_agra_information_services_provided_MA_Task_Service_Information_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_task = in.ma_task.has_value() ? 1u : 0u;
  if (in.ma_task) {
    to_c(*in.ma_task, &out->ma_task);
  }
}

void from_c(const pyramid_components_agra_information_services_provided_MA_Task_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_Task_Service_Information& out) {
  if (in->has_ma_task) {
    out.ma_task.emplace();
    from_c(&in->ma_task, *out.ma_task);
  } else {
    out.ma_task = tl::nullopt;
  }
}

void _free_MA_Task_Service_Information(pyramid_components_agra_information_services_provided_MA_Task_Service_Information_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_task) {
    pyramid_data_model_agra_MA_TaskMT_c_free(&value->ma_task);
    value->has_ma_task = 0;
  }
}

void to_c(const pyramid::components::agra::information::services::provided::MissionContingencyAlert_Service_Information& in, pyramid_components_agra_information_services_provided_MissionContingencyAlert_Service_Information_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_mission_contingency_alert = in.mission_contingency_alert.has_value() ? 1u : 0u;
  if (in.mission_contingency_alert) {
    to_c(*in.mission_contingency_alert, &out->mission_contingency_alert);
  }
}

void from_c(const pyramid_components_agra_information_services_provided_MissionContingencyAlert_Service_Information_c* in, pyramid::components::agra::information::services::provided::MissionContingencyAlert_Service_Information& out) {
  if (in->has_mission_contingency_alert) {
    out.mission_contingency_alert.emplace();
    from_c(&in->mission_contingency_alert, *out.mission_contingency_alert);
  } else {
    out.mission_contingency_alert = tl::nullopt;
  }
}

void _free_MissionContingencyAlert_Service_Information(pyramid_components_agra_information_services_provided_MissionContingencyAlert_Service_Information_c* value) {
  if (!value) {
    return;
  }
  if (value->has_mission_contingency_alert) {
    pyramid_data_model_agra_MissionContingencyAlertMT_c_free(&value->mission_contingency_alert);
    value->has_mission_contingency_alert = 0;
  }
}

void to_c(const pyramid::components::agra::information::services::provided::MA_ActionStatus_Service_Information& in, pyramid_components_agra_information_services_provided_MA_ActionStatus_Service_Information_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_action_status = in.ma_action_status.has_value() ? 1u : 0u;
  if (in.ma_action_status) {
    to_c(*in.ma_action_status, &out->ma_action_status);
  }
}

void from_c(const pyramid_components_agra_information_services_provided_MA_ActionStatus_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_ActionStatus_Service_Information& out) {
  if (in->has_ma_action_status) {
    out.ma_action_status.emplace();
    from_c(&in->ma_action_status, *out.ma_action_status);
  } else {
    out.ma_action_status = tl::nullopt;
  }
}

void _free_MA_ActionStatus_Service_Information(pyramid_components_agra_information_services_provided_MA_ActionStatus_Service_Information_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_action_status) {
    pyramid_data_model_agra_MA_ActionStatusMT_c_free(&value->ma_action_status);
    value->has_ma_action_status = 0;
  }
}

void to_c(const pyramid::components::agra::information::services::provided::MA_MissionPlanActivationStatus_Service_Information& in, pyramid_components_agra_information_services_provided_MA_MissionPlanActivationStatus_Service_Information_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_mission_plan_activation_status = in.ma_mission_plan_activation_status.has_value() ? 1u : 0u;
  if (in.ma_mission_plan_activation_status) {
    to_c(*in.ma_mission_plan_activation_status, &out->ma_mission_plan_activation_status);
  }
}

void from_c(const pyramid_components_agra_information_services_provided_MA_MissionPlanActivationStatus_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_MissionPlanActivationStatus_Service_Information& out) {
  if (in->has_ma_mission_plan_activation_status) {
    out.ma_mission_plan_activation_status.emplace();
    from_c(&in->ma_mission_plan_activation_status, *out.ma_mission_plan_activation_status);
  } else {
    out.ma_mission_plan_activation_status = tl::nullopt;
  }
}

void _free_MA_MissionPlanActivationStatus_Service_Information(pyramid_components_agra_information_services_provided_MA_MissionPlanActivationStatus_Service_Information_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_mission_plan_activation_status) {
    pyramid_data_model_agra_MA_MissionPlanActivationStatusMT_c_free(&value->ma_mission_plan_activation_status);
    value->has_ma_mission_plan_activation_status = 0;
  }
}

void to_c(const pyramid::components::agra::information::services::provided::MA_MissionPlanExecutionStatus_Service_Information& in, pyramid_components_agra_information_services_provided_MA_MissionPlanExecutionStatus_Service_Information_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_mission_plan_execution_status = in.ma_mission_plan_execution_status.has_value() ? 1u : 0u;
  if (in.ma_mission_plan_execution_status) {
    to_c(*in.ma_mission_plan_execution_status, &out->ma_mission_plan_execution_status);
  }
}

void from_c(const pyramid_components_agra_information_services_provided_MA_MissionPlanExecutionStatus_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_MissionPlanExecutionStatus_Service_Information& out) {
  if (in->has_ma_mission_plan_execution_status) {
    out.ma_mission_plan_execution_status.emplace();
    from_c(&in->ma_mission_plan_execution_status, *out.ma_mission_plan_execution_status);
  } else {
    out.ma_mission_plan_execution_status = tl::nullopt;
  }
}

void _free_MA_MissionPlanExecutionStatus_Service_Information(pyramid_components_agra_information_services_provided_MA_MissionPlanExecutionStatus_Service_Information_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_mission_plan_execution_status) {
    pyramid_data_model_agra_MA_MissionPlanExecutionStatusMT_c_free(&value->ma_mission_plan_execution_status);
    value->has_ma_mission_plan_execution_status = 0;
  }
}

void to_c(const pyramid::components::agra::information::services::provided::MA_PlanningFunctionStatus_Service_Information& in, pyramid_components_agra_information_services_provided_MA_PlanningFunctionStatus_Service_Information_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_planning_function_status = in.ma_planning_function_status.has_value() ? 1u : 0u;
  if (in.ma_planning_function_status) {
    to_c(*in.ma_planning_function_status, &out->ma_planning_function_status);
  }
}

void from_c(const pyramid_components_agra_information_services_provided_MA_PlanningFunctionStatus_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_PlanningFunctionStatus_Service_Information& out) {
  if (in->has_ma_planning_function_status) {
    out.ma_planning_function_status.emplace();
    from_c(&in->ma_planning_function_status, *out.ma_planning_function_status);
  } else {
    out.ma_planning_function_status = tl::nullopt;
  }
}

void _free_MA_PlanningFunctionStatus_Service_Information(pyramid_components_agra_information_services_provided_MA_PlanningFunctionStatus_Service_Information_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_planning_function_status) {
    pyramid_data_model_agra_MA_PlanningFunctionStatusMT_c_free(&value->ma_planning_function_status);
    value->has_ma_planning_function_status = 0;
  }
}

void to_c(const pyramid::components::agra::information::services::provided::MA_TaskStatus_Service_Information& in, pyramid_components_agra_information_services_provided_MA_TaskStatus_Service_Information_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_task_status = in.ma_task_status.has_value() ? 1u : 0u;
  if (in.ma_task_status) {
    to_c(*in.ma_task_status, &out->ma_task_status);
  }
}

void from_c(const pyramid_components_agra_information_services_provided_MA_TaskStatus_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_TaskStatus_Service_Information& out) {
  if (in->has_ma_task_status) {
    out.ma_task_status.emplace();
    from_c(&in->ma_task_status, *out.ma_task_status);
  } else {
    out.ma_task_status = tl::nullopt;
  }
}

void _free_MA_TaskStatus_Service_Information(pyramid_components_agra_information_services_provided_MA_TaskStatus_Service_Information_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_task_status) {
    pyramid_data_model_agra_MA_TaskStatusMT_c_free(&value->ma_task_status);
    value->has_ma_task_status = 0;
  }
}

void to_c(const pyramid::components::agra::information::services::provided::MA_ApprovalPolicy_Service_Information& in, pyramid_components_agra_information_services_provided_MA_ApprovalPolicy_Service_Information_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_approval_policy = in.ma_approval_policy.has_value() ? 1u : 0u;
  if (in.ma_approval_policy) {
    to_c(*in.ma_approval_policy, &out->ma_approval_policy);
  }
}

void from_c(const pyramid_components_agra_information_services_provided_MA_ApprovalPolicy_Service_Information_c* in, pyramid::components::agra::information::services::provided::MA_ApprovalPolicy_Service_Information& out) {
  if (in->has_ma_approval_policy) {
    out.ma_approval_policy.emplace();
    from_c(&in->ma_approval_policy, *out.ma_approval_policy);
  } else {
    out.ma_approval_policy = tl::nullopt;
  }
}

void _free_MA_ApprovalPolicy_Service_Information(pyramid_components_agra_information_services_provided_MA_ApprovalPolicy_Service_Information_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_approval_policy) {
    pyramid_data_model_agra_MA_ApprovalPolicyMT_c_free(&value->ma_approval_policy);
    value->has_ma_approval_policy = 0;
  }
}

void to_c(const pyramid::components::agra::information::services::provided::Empty& in, pyramid_components_agra_information_services_provided_Empty_c* out) {
  std::memset(out, 0, sizeof(*out));
}

void from_c(const pyramid_components_agra_information_services_provided_Empty_c* in, pyramid::components::agra::information::services::provided::Empty& out) {
}

void _free_Empty(pyramid_components_agra_information_services_provided_Empty_c* value) {
  if (!value) {
    return;
  }
}

} // namespace pyramid::cabi

extern "C" {

void pyramid_components_agra_information_services_provided_MA_Action_Service_Information_c_free(pyramid_components_agra_information_services_provided_MA_Action_Service_Information_c* value) {
  pyramid::cabi::_free_MA_Action_Service_Information(value);
}

void pyramid_components_agra_information_services_provided_MA_MissionPlan_Service_Information_c_free(pyramid_components_agra_information_services_provided_MA_MissionPlan_Service_Information_c* value) {
  pyramid::cabi::_free_MA_MissionPlan_Service_Information(value);
}

void pyramid_components_agra_information_services_provided_MA_PlanningFunction_Service_Information_c_free(pyramid_components_agra_information_services_provided_MA_PlanningFunction_Service_Information_c* value) {
  pyramid::cabi::_free_MA_PlanningFunction_Service_Information(value);
}

void pyramid_components_agra_information_services_provided_MA_Response_Service_Information_c_free(pyramid_components_agra_information_services_provided_MA_Response_Service_Information_c* value) {
  pyramid::cabi::_free_MA_Response_Service_Information(value);
}

void pyramid_components_agra_information_services_provided_MA_Task_Service_Information_c_free(pyramid_components_agra_information_services_provided_MA_Task_Service_Information_c* value) {
  pyramid::cabi::_free_MA_Task_Service_Information(value);
}

void pyramid_components_agra_information_services_provided_MissionContingencyAlert_Service_Information_c_free(pyramid_components_agra_information_services_provided_MissionContingencyAlert_Service_Information_c* value) {
  pyramid::cabi::_free_MissionContingencyAlert_Service_Information(value);
}

void pyramid_components_agra_information_services_provided_MA_ActionStatus_Service_Information_c_free(pyramid_components_agra_information_services_provided_MA_ActionStatus_Service_Information_c* value) {
  pyramid::cabi::_free_MA_ActionStatus_Service_Information(value);
}

void pyramid_components_agra_information_services_provided_MA_MissionPlanActivationStatus_Service_Information_c_free(pyramid_components_agra_information_services_provided_MA_MissionPlanActivationStatus_Service_Information_c* value) {
  pyramid::cabi::_free_MA_MissionPlanActivationStatus_Service_Information(value);
}

void pyramid_components_agra_information_services_provided_MA_MissionPlanExecutionStatus_Service_Information_c_free(pyramid_components_agra_information_services_provided_MA_MissionPlanExecutionStatus_Service_Information_c* value) {
  pyramid::cabi::_free_MA_MissionPlanExecutionStatus_Service_Information(value);
}

void pyramid_components_agra_information_services_provided_MA_PlanningFunctionStatus_Service_Information_c_free(pyramid_components_agra_information_services_provided_MA_PlanningFunctionStatus_Service_Information_c* value) {
  pyramid::cabi::_free_MA_PlanningFunctionStatus_Service_Information(value);
}

void pyramid_components_agra_information_services_provided_MA_TaskStatus_Service_Information_c_free(pyramid_components_agra_information_services_provided_MA_TaskStatus_Service_Information_c* value) {
  pyramid::cabi::_free_MA_TaskStatus_Service_Information(value);
}

void pyramid_components_agra_information_services_provided_MA_ApprovalPolicy_Service_Information_c_free(pyramid_components_agra_information_services_provided_MA_ApprovalPolicy_Service_Information_c* value) {
  pyramid::cabi::_free_MA_ApprovalPolicy_Service_Information(value);
}

void pyramid_components_agra_information_services_provided_Empty_c_free(pyramid_components_agra_information_services_provided_Empty_c* value) {
  pyramid::cabi::_free_Empty(value);
}

} // extern "C"
