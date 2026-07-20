// Auto-generated data model JSON codec implementation
// Namespace: pyramid::components::agra::information::services::provided

#include "pyramid_components_agra_information_services_provided_codec.hpp"

#include <nlohmann/json.hpp>

#include "pyramid_data_model_agra_codec.hpp"

namespace pyramid::components::agra::information::services::provided {

std::string toJson(const MA_Action_Service_Information& msg) {
    nlohmann::json obj;
    if (msg.ma_action.has_value()) {
        obj["ma_action"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_action.value()));
    }
    return obj.dump();
}

MA_Action_Service_Information fromJson(const std::string& s, MA_Action_Service_Information* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_Action_Service_Information msg;
    if (j.contains("ma_action")) {
        msg.ma_action = pyramid::domain_model::agra::fromJson(j["ma_action"].dump(), static_cast<pyramid::domain_model::agra::MA_ActionMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_MissionPlan_Service_Information& msg) {
    nlohmann::json obj;
    if (msg.ma_mission_plan.has_value()) {
        obj["ma_mission_plan"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_mission_plan.value()));
    }
    return obj.dump();
}

MA_MissionPlan_Service_Information fromJson(const std::string& s, MA_MissionPlan_Service_Information* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_MissionPlan_Service_Information msg;
    if (j.contains("ma_mission_plan")) {
        msg.ma_mission_plan = pyramid::domain_model::agra::fromJson(j["ma_mission_plan"].dump(), static_cast<pyramid::domain_model::agra::MA_MissionPlanMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_PlanningFunction_Service_Information& msg) {
    nlohmann::json obj;
    if (msg.ma_planning_function.has_value()) {
        obj["ma_planning_function"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_planning_function.value()));
    }
    return obj.dump();
}

MA_PlanningFunction_Service_Information fromJson(const std::string& s, MA_PlanningFunction_Service_Information* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_PlanningFunction_Service_Information msg;
    if (j.contains("ma_planning_function")) {
        msg.ma_planning_function = pyramid::domain_model::agra::fromJson(j["ma_planning_function"].dump(), static_cast<pyramid::domain_model::agra::MA_PlanningFunctionMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_Response_Service_Information& msg) {
    nlohmann::json obj;
    if (msg.ma_response.has_value()) {
        obj["ma_response"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_response.value()));
    }
    return obj.dump();
}

MA_Response_Service_Information fromJson(const std::string& s, MA_Response_Service_Information* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_Response_Service_Information msg;
    if (j.contains("ma_response")) {
        msg.ma_response = pyramid::domain_model::agra::fromJson(j["ma_response"].dump(), static_cast<pyramid::domain_model::agra::MA_ResponseMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_Task_Service_Information& msg) {
    nlohmann::json obj;
    if (msg.ma_task.has_value()) {
        obj["ma_task"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_task.value()));
    }
    return obj.dump();
}

MA_Task_Service_Information fromJson(const std::string& s, MA_Task_Service_Information* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_Task_Service_Information msg;
    if (j.contains("ma_task")) {
        msg.ma_task = pyramid::domain_model::agra::fromJson(j["ma_task"].dump(), static_cast<pyramid::domain_model::agra::MA_TaskMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MissionContingencyAlert_Service_Information& msg) {
    nlohmann::json obj;
    if (msg.mission_contingency_alert.has_value()) {
        obj["mission_contingency_alert"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.mission_contingency_alert.value()));
    }
    return obj.dump();
}

MissionContingencyAlert_Service_Information fromJson(const std::string& s, MissionContingencyAlert_Service_Information* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MissionContingencyAlert_Service_Information msg;
    if (j.contains("mission_contingency_alert")) {
        msg.mission_contingency_alert = pyramid::domain_model::agra::fromJson(j["mission_contingency_alert"].dump(), static_cast<pyramid::domain_model::agra::MissionContingencyAlertMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_ActionStatus_Service_Information& msg) {
    nlohmann::json obj;
    if (msg.ma_action_status.has_value()) {
        obj["ma_action_status"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_action_status.value()));
    }
    return obj.dump();
}

MA_ActionStatus_Service_Information fromJson(const std::string& s, MA_ActionStatus_Service_Information* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_ActionStatus_Service_Information msg;
    if (j.contains("ma_action_status")) {
        msg.ma_action_status = pyramid::domain_model::agra::fromJson(j["ma_action_status"].dump(), static_cast<pyramid::domain_model::agra::MA_ActionStatusMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_MissionPlanActivationStatus_Service_Information& msg) {
    nlohmann::json obj;
    if (msg.ma_mission_plan_activation_status.has_value()) {
        obj["ma_mission_plan_activation_status"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_mission_plan_activation_status.value()));
    }
    return obj.dump();
}

MA_MissionPlanActivationStatus_Service_Information fromJson(const std::string& s, MA_MissionPlanActivationStatus_Service_Information* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_MissionPlanActivationStatus_Service_Information msg;
    if (j.contains("ma_mission_plan_activation_status")) {
        msg.ma_mission_plan_activation_status = pyramid::domain_model::agra::fromJson(j["ma_mission_plan_activation_status"].dump(), static_cast<pyramid::domain_model::agra::MA_MissionPlanActivationStatusMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_MissionPlanExecutionStatus_Service_Information& msg) {
    nlohmann::json obj;
    if (msg.ma_mission_plan_execution_status.has_value()) {
        obj["ma_mission_plan_execution_status"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_mission_plan_execution_status.value()));
    }
    return obj.dump();
}

MA_MissionPlanExecutionStatus_Service_Information fromJson(const std::string& s, MA_MissionPlanExecutionStatus_Service_Information* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_MissionPlanExecutionStatus_Service_Information msg;
    if (j.contains("ma_mission_plan_execution_status")) {
        msg.ma_mission_plan_execution_status = pyramid::domain_model::agra::fromJson(j["ma_mission_plan_execution_status"].dump(), static_cast<pyramid::domain_model::agra::MA_MissionPlanExecutionStatusMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_PlanningFunctionStatus_Service_Information& msg) {
    nlohmann::json obj;
    if (msg.ma_planning_function_status.has_value()) {
        obj["ma_planning_function_status"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_planning_function_status.value()));
    }
    return obj.dump();
}

MA_PlanningFunctionStatus_Service_Information fromJson(const std::string& s, MA_PlanningFunctionStatus_Service_Information* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_PlanningFunctionStatus_Service_Information msg;
    if (j.contains("ma_planning_function_status")) {
        msg.ma_planning_function_status = pyramid::domain_model::agra::fromJson(j["ma_planning_function_status"].dump(), static_cast<pyramid::domain_model::agra::MA_PlanningFunctionStatusMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_TaskStatus_Service_Information& msg) {
    nlohmann::json obj;
    if (msg.ma_task_status.has_value()) {
        obj["ma_task_status"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_task_status.value()));
    }
    return obj.dump();
}

MA_TaskStatus_Service_Information fromJson(const std::string& s, MA_TaskStatus_Service_Information* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_TaskStatus_Service_Information msg;
    if (j.contains("ma_task_status")) {
        msg.ma_task_status = pyramid::domain_model::agra::fromJson(j["ma_task_status"].dump(), static_cast<pyramid::domain_model::agra::MA_TaskStatusMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const MA_ApprovalPolicy_Service_Information& msg) {
    nlohmann::json obj;
    if (msg.ma_approval_policy.has_value()) {
        obj["ma_approval_policy"] = nlohmann::json::parse(pyramid::domain_model::agra::toJson(msg.ma_approval_policy.value()));
    }
    return obj.dump();
}

MA_ApprovalPolicy_Service_Information fromJson(const std::string& s, MA_ApprovalPolicy_Service_Information* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    MA_ApprovalPolicy_Service_Information msg;
    if (j.contains("ma_approval_policy")) {
        msg.ma_approval_policy = pyramid::domain_model::agra::fromJson(j["ma_approval_policy"].dump(), static_cast<pyramid::domain_model::agra::MA_ApprovalPolicyMT*>(nullptr));
    }
    return msg;
}

std::string toJson(const Empty& msg) {
    nlohmann::json obj;
    return obj.dump();
}

Empty fromJson(const std::string& s, Empty* /*tag*/) {
    auto j = nlohmann::json::parse(s);
    Empty msg;
    return msg;
}

} // namespace pyramid::components::agra::information::services::provided
