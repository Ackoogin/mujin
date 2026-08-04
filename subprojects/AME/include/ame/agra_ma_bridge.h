#pragma once

#include "ame/autonomy_backend.h"

#include "pyramid_services_agra_c2_consumed_components.hpp"
#include "pyramid_services_agra_c2_provided_components.hpp"

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace ame {

namespace agra = pyramid::domain_model::agra;
namespace agra_c2_consumed =
    pyramid::components::agra::c2::services::consumed;
namespace agra_c2_provided =
    pyramid::components::agra::c2::services::provided;

/// \brief Explicit policy and A-GRA envelope settings for the MA bridge.
struct AgraMaBridgeOptions {
  std::string system_uuid;
  std::string approval_authority_system_uuid;
  agra::MessageModeEnum message_mode = agra::MessageModeEnum::Unspecified;
  unsigned approval_timeout_seconds = 0;
  PolicyEnvelope backend_policy;
};

/// \brief One externally grounded waypoint for a symbolic plan element.
struct AgraWaypointGrounding {
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  double altitude_m = 0.0;
};

/// \brief AutoMTK-provided grounding for one AME action signature.
struct AgraPlanElementGrounding {
  std::string action_signature;
  agra::MA_ActionMT action;
  bool requires_kinematics = false;
  std::string route_plan_id;
  std::vector<AgraWaypointGrounding> waypoints;
  std::unordered_map<std::string, std::string> command_parameters;
};

/// \brief A-GRA P3 C2 planning boundary over one hardened autonomy backend.
class AgraMaBridge final
    : public agra_c2_provided::MaMissionplancommandStatusPortHandler,
      public agra_c2_provided::MaTaskcommandStatusPortHandler,
      public agra_c2_provided::
          MaMissionplanactivationcommandStatusPortHandler {
public:
  AgraMaBridge(IAutonomyBackend& backend, AgraMaBridgeOptions options);

  static std::string deterministicUuid(const std::string& value);
  static agra::CommandProcessingStateEnum commandProcessingState(
      CommandStatus status);
  static CommandStatus commandStatus(
      agra::CommandProcessingStateEnum state);

  void registerTaskGrounding(const agra::MA_TaskMT& task,
                             const std::vector<std::string>& goal_fluents);

  void supplyPlanGrounding(
      const std::string& mission_plan_command_uuid,
      const std::vector<AgraPlanElementGrounding>& grounding);

  std::vector<agra_c2_consumed::MA_ApprovalRequest_Service_Information>
  pullApprovalRequests();

  std::vector<ActionCommand> pullCommands();
  void pushCommandResult(const CommandResult& result);
  std::vector<
      agra_c2_provided::MA_ActionCommand_Service_Information>
  pullActionCommandMessages();
  void pushActionCommandStatus(
      const agra_c2_provided::
          MA_ActionCommandStatus_Service_Information& status);
  void step();

  void ingestApprovalStatus(
      const agra_c2_consumed::
          MA_ApprovalRequestStatus_Service_Information& status);

  void onCommand(
      const agra_c2_provided::
          MA_MissionPlanCommand_Service_Information& command) override;
  void onCommand(
      const agra_c2_provided::MA_TaskCommand_Service_Information& command)
      override;
  void onCommand(
      const agra_c2_provided::
          MA_MissionPlanActivationCommand_Service_Information& command)
      override;

  std::vector<
      agra_c2_provided::MA_MissionPlanCommandStatus_Service_Information>
  handleMaMissionplancommandstatusRead(
      const agra_c2_provided::Empty& request);
  std::vector<agra_c2_provided::MA_TaskCommandStatus_Service_Information>
  handleMaTaskcommandstatusRead(const agra_c2_provided::Empty& request);
  std::vector<
      agra_c2_provided::
          MA_MissionPlanActivationCommandStatus_Service_Information>
  handleMaMissionplanactivationcommandstatusRead(
      const agra_c2_provided::Empty& request);
  std::vector<agra_c2_provided::MA_Action_Service_Information>
  handleMaActionRead(const agra_c2_provided::Empty& request);
  std::vector<agra_c2_provided::MA_ActionPlan_Service_Information>
  handleMaActionplanRead(const agra_c2_provided::Empty& request);
  std::vector<agra_c2_provided::MA_RoutePlan_Service_Information>
  handleMaRouteplanRead(const agra_c2_provided::Empty& request);
  std::vector<agra_c2_provided::MA_MissionPlan_Service_Information>
  handleMaMissionplanRead(const agra_c2_provided::Empty& request);
  std::vector<agra_c2_provided::MA_PlanningFunction_Service_Information>
  handleMaPlanningfunctionRead(const agra_c2_provided::Empty& request);
  std::vector<agra_c2_provided::MA_Task_Service_Information>
  handleMaTaskRead(const agra_c2_provided::Empty& request);
  std::vector<
      agra_c2_provided::
          MA_MissionPlanActivationStatus_Service_Information>
  handleMaMissionplanactivationstatusRead(
      const agra_c2_provided::Empty& request);
  std::vector<
      agra_c2_provided::
          MA_MissionPlanExecutionStatus_Service_Information>
  handleMaMissionplanexecutionstatusRead(
      const agra_c2_provided::Empty& request);
  std::vector<agra_c2_provided::MA_TaskStatus_Service_Information>
  handleMaTaskstatusRead(const agra_c2_provided::Empty& request);
  std::vector<
      agra_c2_provided::MissionContingencyAlert_Service_Information>
  handleMissioncontingencyalertRead(
      const agra_c2_provided::Empty& request);

private:
  struct TaskGrounding {
    agra::MA_TaskMT task;
    std::vector<std::string> goal_fluents;
  };

  static std::string requireUuid(const std::string& uuid,
                                 const std::string& field_name);
  static std::string uuidHex(const std::string& uuid);
  static agra::CannotComplyType cannotComply(
      agra::CannotComplyEnum reason,
      const std::string& description);
  static agra::RequirementExecutionStateEnum requirementState(
      AutonomyBackendState state);
  static agra::PlanExecutionStateEnum planExecutionState(
      AutonomyBackendState state);
  static agra_c2_provided::MA_ActionCommand_Service_Information
  actionCommandMessage(
      const ActionCommand& command,
      const agra::SecurityInformationType& security_information,
      const agra::HeaderType& header);

  agra::HeaderType makeHeader() const;
  agra::HeaderType makeHeader(
      const agra::MissionID_Type& mission_id) const;
  void validateHeader(const agra::HeaderType& header,
                      const std::string& message_name) const;
  std::string taskKey(const agra::TaskID_Type& task_id) const;
  std::string planKey(const agra::MissionPlanID_Type& plan_id) const;
  agra::MissionPlanID_Type missionPlanId(
      const std::string& backend_plan_id) const;
  void collectDecisionRecords(const std::string& command_uuid);
  void projectPlan(const DecisionRecord& record,
                   const std::string& command_uuid);
  void queueMissionPlanCommandStatus(
      const std::string& command_uuid,
      agra::CommandProcessingStateEnum processing_state,
      agra::ProcessingStatusEnum status,
      const std::string& plan_id = {},
      const std::string& error = {});
  void queueTaskCommandStatus(
      const agra::CommandID_Type& command_id,
      const agra::SecurityInformationType& security_information,
      agra::CommandProcessingStateEnum processing_state,
      const std::string& error = {});
  void queueActivationCommandStatus(
      const std::string& command_uuid,
      const agra::SecurityInformationType& security_information,
      agra::CommandProcessingStateEnum processing_state,
      agra::ProcessingStatusEnum status,
      const std::string& error = {});
  const AgraPlanElementGrounding& groundingForCommand(
      const ActionCommand& command) const;
  void refreshExecutionStatus();

  IAutonomyBackend& backend_;
  AgraMaBridgeOptions options_;
  std::unordered_map<std::string, TaskGrounding> task_groundings_;
  std::unordered_map<std::string, std::vector<AgraPlanElementGrounding>>
      plan_groundings_;
  std::unordered_map<std::string, std::string> approval_plan_by_request_;
  std::unordered_map<std::string, std::string> backend_plan_by_agra_plan_;
  std::unordered_map<std::string, std::string> backend_action_command_by_agra_;
  std::vector<
      agra_c2_consumed::MA_ApprovalRequest_Service_Information>
      approval_requests_;
  std::vector<
      agra_c2_provided::MA_MissionPlanCommandStatus_Service_Information>
      mission_plan_command_statuses_;
  std::vector<agra_c2_provided::MA_TaskCommandStatus_Service_Information>
      task_command_statuses_;
  std::vector<
      agra_c2_provided::
          MA_MissionPlanActivationCommandStatus_Service_Information>
      activation_command_statuses_;
  std::vector<agra_c2_provided::MA_Action_Service_Information> actions_;
  std::vector<agra_c2_provided::MA_ActionPlan_Service_Information>
      action_plans_;
  std::vector<agra_c2_provided::MA_RoutePlan_Service_Information>
      route_plans_;
  std::vector<agra_c2_provided::MA_MissionPlan_Service_Information>
      mission_plans_;
  std::vector<agra_c2_provided::MA_Task_Service_Information> tasks_;
  std::vector<
      agra_c2_provided::
          MA_MissionPlanActivationStatus_Service_Information>
      activation_statuses_;
  std::vector<
      agra_c2_provided::
          MA_MissionPlanExecutionStatus_Service_Information>
      execution_statuses_;
  std::vector<agra_c2_provided::MA_TaskStatus_Service_Information>
      task_statuses_;
  std::vector<
      agra_c2_provided::MissionContingencyAlert_Service_Information>
      contingency_alerts_;
  std::string active_command_uuid_;
  std::string active_task_key_;
  agra::SecurityInformationType active_command_security_;
  std::string current_backend_plan_id_;
  std::string current_agra_plan_uuid_;
  bool activation_received_ = false;
  size_t projected_replan_count_ = 0;
};

}  // namespace ame
