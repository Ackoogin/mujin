// Auto-generated C-ABI marshalling
// Ownership: to_c malloc-deep-copies all variable-length data into
// the C struct; from_c deep-copies into native values; _free releases
// everything allocated for the C struct. One full copy per direction.

#include "pyramid_components_agra_mission_autonomy_services_provided_cabi_marshal.hpp"
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

void to_c(const pyramid::components::agra::mission_autonomy::services::provided::MA_MissionPlanCommand_Service_Request& in, pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanCommand_Service_Request_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_mission_plan_command = in.ma_mission_plan_command.has_value() ? 1u : 0u;
  if (in.ma_mission_plan_command) {
    to_c(*in.ma_mission_plan_command, &out->ma_mission_plan_command);
  }
  out->has_update = in.update.has_value() ? 1u : 0u;
  if (in.update) {
    to_c(*in.update, &out->update);
  }
}

void from_c(const pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanCommand_Service_Request_c* in, pyramid::components::agra::mission_autonomy::services::provided::MA_MissionPlanCommand_Service_Request& out) {
  if (in->has_ma_mission_plan_command) {
    out.ma_mission_plan_command.emplace();
    from_c(&in->ma_mission_plan_command, *out.ma_mission_plan_command);
  } else {
    out.ma_mission_plan_command = tl::nullopt;
  }
  if (in->has_update) {
    out.update.emplace();
    from_c(&in->update, *out.update);
  } else {
    out.update = tl::nullopt;
  }
}

void _free_MA_MissionPlanCommand_Service_Request(pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanCommand_Service_Request_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_mission_plan_command) {
    pyramid_data_model_agra_MA_MissionPlanCommandMT_c_free(&value->ma_mission_plan_command);
    value->has_ma_mission_plan_command = 0;
  }
  if (value->has_update) {
    pyramid_data_model_agra_MA_MissionPlanCommandStatusMT_c_free(&value->update);
    value->has_update = 0;
  }
}

void to_c(const pyramid::components::agra::mission_autonomy::services::provided::MA_MissionPlanCommand_Service_Entity& in, pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanCommand_Service_Entity_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_mission_plan_command_status = in.ma_mission_plan_command_status.has_value() ? 1u : 0u;
  if (in.ma_mission_plan_command_status) {
    to_c(*in.ma_mission_plan_command_status, &out->ma_mission_plan_command_status);
  }
}

void from_c(const pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanCommand_Service_Entity_c* in, pyramid::components::agra::mission_autonomy::services::provided::MA_MissionPlanCommand_Service_Entity& out) {
  if (in->has_ma_mission_plan_command_status) {
    out.ma_mission_plan_command_status.emplace();
    from_c(&in->ma_mission_plan_command_status, *out.ma_mission_plan_command_status);
  } else {
    out.ma_mission_plan_command_status = tl::nullopt;
  }
}

void _free_MA_MissionPlanCommand_Service_Entity(pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanCommand_Service_Entity_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_mission_plan_command_status) {
    pyramid_data_model_agra_MA_MissionPlanCommandStatusMT_c_free(&value->ma_mission_plan_command_status);
    value->has_ma_mission_plan_command_status = 0;
  }
}

void to_c(const pyramid::components::agra::mission_autonomy::services::provided::MA_MissionPlanActivationCommand_Service_Request& in, pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanActivationCommand_Service_Request_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_mission_plan_activation_command = in.ma_mission_plan_activation_command.has_value() ? 1u : 0u;
  if (in.ma_mission_plan_activation_command) {
    to_c(*in.ma_mission_plan_activation_command, &out->ma_mission_plan_activation_command);
  }
  out->has_update = in.update.has_value() ? 1u : 0u;
  if (in.update) {
    to_c(*in.update, &out->update);
  }
}

void from_c(const pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanActivationCommand_Service_Request_c* in, pyramid::components::agra::mission_autonomy::services::provided::MA_MissionPlanActivationCommand_Service_Request& out) {
  if (in->has_ma_mission_plan_activation_command) {
    out.ma_mission_plan_activation_command.emplace();
    from_c(&in->ma_mission_plan_activation_command, *out.ma_mission_plan_activation_command);
  } else {
    out.ma_mission_plan_activation_command = tl::nullopt;
  }
  if (in->has_update) {
    out.update.emplace();
    from_c(&in->update, *out.update);
  } else {
    out.update = tl::nullopt;
  }
}

void _free_MA_MissionPlanActivationCommand_Service_Request(pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanActivationCommand_Service_Request_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_mission_plan_activation_command) {
    pyramid_data_model_agra_MA_MissionPlanActivationCommandMT_c_free(&value->ma_mission_plan_activation_command);
    value->has_ma_mission_plan_activation_command = 0;
  }
  if (value->has_update) {
    pyramid_data_model_agra_MA_MissionPlanActivationCommandStatusMT_c_free(&value->update);
    value->has_update = 0;
  }
}

void to_c(const pyramid::components::agra::mission_autonomy::services::provided::MA_MissionPlanActivationCommand_Service_Entity& in, pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanActivationCommand_Service_Entity_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_mission_plan_activation_command_status = in.ma_mission_plan_activation_command_status.has_value() ? 1u : 0u;
  if (in.ma_mission_plan_activation_command_status) {
    to_c(*in.ma_mission_plan_activation_command_status, &out->ma_mission_plan_activation_command_status);
  }
}

void from_c(const pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanActivationCommand_Service_Entity_c* in, pyramid::components::agra::mission_autonomy::services::provided::MA_MissionPlanActivationCommand_Service_Entity& out) {
  if (in->has_ma_mission_plan_activation_command_status) {
    out.ma_mission_plan_activation_command_status.emplace();
    from_c(&in->ma_mission_plan_activation_command_status, *out.ma_mission_plan_activation_command_status);
  } else {
    out.ma_mission_plan_activation_command_status = tl::nullopt;
  }
}

void _free_MA_MissionPlanActivationCommand_Service_Entity(pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanActivationCommand_Service_Entity_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_mission_plan_activation_command_status) {
    pyramid_data_model_agra_MA_MissionPlanActivationCommandStatusMT_c_free(&value->ma_mission_plan_activation_command_status);
    value->has_ma_mission_plan_activation_command_status = 0;
  }
}

void to_c(const pyramid::components::agra::mission_autonomy::services::provided::MA_PlanningFunctionSettingsCommand_Service_Request& in, pyramid_components_agra_mission_autonomy_services_provided_MA_PlanningFunctionSettingsCommand_Service_Request_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_planning_function_settings_command = in.ma_planning_function_settings_command.has_value() ? 1u : 0u;
  if (in.ma_planning_function_settings_command) {
    to_c(*in.ma_planning_function_settings_command, &out->ma_planning_function_settings_command);
  }
  out->has_update = in.update.has_value() ? 1u : 0u;
  if (in.update) {
    to_c(*in.update, &out->update);
  }
}

void from_c(const pyramid_components_agra_mission_autonomy_services_provided_MA_PlanningFunctionSettingsCommand_Service_Request_c* in, pyramid::components::agra::mission_autonomy::services::provided::MA_PlanningFunctionSettingsCommand_Service_Request& out) {
  if (in->has_ma_planning_function_settings_command) {
    out.ma_planning_function_settings_command.emplace();
    from_c(&in->ma_planning_function_settings_command, *out.ma_planning_function_settings_command);
  } else {
    out.ma_planning_function_settings_command = tl::nullopt;
  }
  if (in->has_update) {
    out.update.emplace();
    from_c(&in->update, *out.update);
  } else {
    out.update = tl::nullopt;
  }
}

void _free_MA_PlanningFunctionSettingsCommand_Service_Request(pyramid_components_agra_mission_autonomy_services_provided_MA_PlanningFunctionSettingsCommand_Service_Request_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_planning_function_settings_command) {
    pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandMT_c_free(&value->ma_planning_function_settings_command);
    value->has_ma_planning_function_settings_command = 0;
  }
  if (value->has_update) {
    pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandStatusMT_c_free(&value->update);
    value->has_update = 0;
  }
}

void to_c(const pyramid::components::agra::mission_autonomy::services::provided::MA_PlanningFunctionSettingsCommand_Service_Entity& in, pyramid_components_agra_mission_autonomy_services_provided_MA_PlanningFunctionSettingsCommand_Service_Entity_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_planning_function_settings_command_status = in.ma_planning_function_settings_command_status.has_value() ? 1u : 0u;
  if (in.ma_planning_function_settings_command_status) {
    to_c(*in.ma_planning_function_settings_command_status, &out->ma_planning_function_settings_command_status);
  }
}

void from_c(const pyramid_components_agra_mission_autonomy_services_provided_MA_PlanningFunctionSettingsCommand_Service_Entity_c* in, pyramid::components::agra::mission_autonomy::services::provided::MA_PlanningFunctionSettingsCommand_Service_Entity& out) {
  if (in->has_ma_planning_function_settings_command_status) {
    out.ma_planning_function_settings_command_status.emplace();
    from_c(&in->ma_planning_function_settings_command_status, *out.ma_planning_function_settings_command_status);
  } else {
    out.ma_planning_function_settings_command_status = tl::nullopt;
  }
}

void _free_MA_PlanningFunctionSettingsCommand_Service_Entity(pyramid_components_agra_mission_autonomy_services_provided_MA_PlanningFunctionSettingsCommand_Service_Entity_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_planning_function_settings_command_status) {
    pyramid_data_model_agra_MA_PlanningFunctionSettingsCommandStatusMT_c_free(&value->ma_planning_function_settings_command_status);
    value->has_ma_planning_function_settings_command_status = 0;
  }
}

void to_c(const pyramid::components::agra::mission_autonomy::services::provided::MA_ApprovalRequest_Service_Request& in, pyramid_components_agra_mission_autonomy_services_provided_MA_ApprovalRequest_Service_Request_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_approval_request = in.ma_approval_request.has_value() ? 1u : 0u;
  if (in.ma_approval_request) {
    to_c(*in.ma_approval_request, &out->ma_approval_request);
  }
  out->has_update = in.update.has_value() ? 1u : 0u;
  if (in.update) {
    to_c(*in.update, &out->update);
  }
}

void from_c(const pyramid_components_agra_mission_autonomy_services_provided_MA_ApprovalRequest_Service_Request_c* in, pyramid::components::agra::mission_autonomy::services::provided::MA_ApprovalRequest_Service_Request& out) {
  if (in->has_ma_approval_request) {
    out.ma_approval_request.emplace();
    from_c(&in->ma_approval_request, *out.ma_approval_request);
  } else {
    out.ma_approval_request = tl::nullopt;
  }
  if (in->has_update) {
    out.update.emplace();
    from_c(&in->update, *out.update);
  } else {
    out.update = tl::nullopt;
  }
}

void _free_MA_ApprovalRequest_Service_Request(pyramid_components_agra_mission_autonomy_services_provided_MA_ApprovalRequest_Service_Request_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_approval_request) {
    pyramid_data_model_agra_MA_ApprovalRequestMT_c_free(&value->ma_approval_request);
    value->has_ma_approval_request = 0;
  }
  if (value->has_update) {
    pyramid_data_model_agra_MA_ApprovalRequestStatusMT_c_free(&value->update);
    value->has_update = 0;
  }
}

void to_c(const pyramid::components::agra::mission_autonomy::services::provided::MA_ApprovalRequest_Service_Entity& in, pyramid_components_agra_mission_autonomy_services_provided_MA_ApprovalRequest_Service_Entity_c* out) {
  std::memset(out, 0, sizeof(*out));
  out->has_ma_approval_request_status = in.ma_approval_request_status.has_value() ? 1u : 0u;
  if (in.ma_approval_request_status) {
    to_c(*in.ma_approval_request_status, &out->ma_approval_request_status);
  }
}

void from_c(const pyramid_components_agra_mission_autonomy_services_provided_MA_ApprovalRequest_Service_Entity_c* in, pyramid::components::agra::mission_autonomy::services::provided::MA_ApprovalRequest_Service_Entity& out) {
  if (in->has_ma_approval_request_status) {
    out.ma_approval_request_status.emplace();
    from_c(&in->ma_approval_request_status, *out.ma_approval_request_status);
  } else {
    out.ma_approval_request_status = tl::nullopt;
  }
}

void _free_MA_ApprovalRequest_Service_Entity(pyramid_components_agra_mission_autonomy_services_provided_MA_ApprovalRequest_Service_Entity_c* value) {
  if (!value) {
    return;
  }
  if (value->has_ma_approval_request_status) {
    pyramid_data_model_agra_MA_ApprovalRequestStatusMT_c_free(&value->ma_approval_request_status);
    value->has_ma_approval_request_status = 0;
  }
}

} // namespace pyramid::cabi

extern "C" {

void pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanCommand_Service_Request_c_free(pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanCommand_Service_Request_c* value) {
  pyramid::cabi::_free_MA_MissionPlanCommand_Service_Request(value);
}

void pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanCommand_Service_Entity_c_free(pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanCommand_Service_Entity_c* value) {
  pyramid::cabi::_free_MA_MissionPlanCommand_Service_Entity(value);
}

void pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanActivationCommand_Service_Request_c_free(pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanActivationCommand_Service_Request_c* value) {
  pyramid::cabi::_free_MA_MissionPlanActivationCommand_Service_Request(value);
}

void pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanActivationCommand_Service_Entity_c_free(pyramid_components_agra_mission_autonomy_services_provided_MA_MissionPlanActivationCommand_Service_Entity_c* value) {
  pyramid::cabi::_free_MA_MissionPlanActivationCommand_Service_Entity(value);
}

void pyramid_components_agra_mission_autonomy_services_provided_MA_PlanningFunctionSettingsCommand_Service_Request_c_free(pyramid_components_agra_mission_autonomy_services_provided_MA_PlanningFunctionSettingsCommand_Service_Request_c* value) {
  pyramid::cabi::_free_MA_PlanningFunctionSettingsCommand_Service_Request(value);
}

void pyramid_components_agra_mission_autonomy_services_provided_MA_PlanningFunctionSettingsCommand_Service_Entity_c_free(pyramid_components_agra_mission_autonomy_services_provided_MA_PlanningFunctionSettingsCommand_Service_Entity_c* value) {
  pyramid::cabi::_free_MA_PlanningFunctionSettingsCommand_Service_Entity(value);
}

void pyramid_components_agra_mission_autonomy_services_provided_MA_ApprovalRequest_Service_Request_c_free(pyramid_components_agra_mission_autonomy_services_provided_MA_ApprovalRequest_Service_Request_c* value) {
  pyramid::cabi::_free_MA_ApprovalRequest_Service_Request(value);
}

void pyramid_components_agra_mission_autonomy_services_provided_MA_ApprovalRequest_Service_Entity_c_free(pyramid_components_agra_mission_autonomy_services_provided_MA_ApprovalRequest_Service_Entity_c* value) {
  pyramid::cabi::_free_MA_ApprovalRequest_Service_Entity(value);
}

} // extern "C"
