#pragma once

/// \file agra_ma_component.hpp
/// \brief Host AgraMaBridge on PYRAMID's generated Port layer.
///
/// AgraMaBridge implements the generated `…PortHandler` interfaces; this owns
/// the matching `…PortProvider` objects and drives them, so callers need no
/// containers, codec handling or port type-name strings of their own. The
/// generated provider supplies the port type name and the configured content
/// type, which is why neither appears here.
///
/// Transport is chosen by deployment, not by this file: the endpoints named in
/// `deploymentDescriptor()` -- `MA_TaskCommand` and `MA_TaskCommandStatus` --
/// are routed to a peer by the routing manifest or ports file, so the same
/// component runs over shared memory or LA-CAL unchanged.
///
/// The generated Port layer marshals C++ structs to and from OMS-JSON, but
/// the wire and the bridge disagree about what a UUID is: OMS-JSON requires
/// 32 hexadecimal characters, and `AgraMaBridge` requires 16 raw bytes
/// (`ame/agra_oms_uuid.h`). Converting at this seam is this component's job,
/// so `AgraMaBridge` keeps working in native form regardless of which
/// transport carries it.

#include "ame/agra_ma_bridge.h"
#include "ame/agra_oms_uuid.h"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>

#include <cstdio>
#include <exception>
#include <string>
#include <utility>

namespace ame {

namespace detail {

inline void agraHeaderWireToNative(agra::HeaderType& header) {
  header.system_id.uuid = nativeUuidFromOms(header.system_id.uuid);
  if (header.mission_id.has_value()) {
    header.mission_id->base.uuid =
        nativeUuidFromOms(header.mission_id->base.uuid);
  }
}

inline void agraHeaderNativeToWire(agra::HeaderType& header) {
  header.system_id.uuid = omsUuidFromNative(header.system_id.uuid);
  if (header.mission_id.has_value()) {
    header.mission_id->base.uuid =
        omsUuidFromNative(header.mission_id->base.uuid);
  }
}

/// \brief Convert a received MA_TaskCommand from OMS-JSON wire form to the
///        native form `AgraMaBridge` requires.
inline agra_c2_provided::MA_TaskCommand_Service_Information
agraTaskCommandWireToNative(
    agra_c2_provided::MA_TaskCommand_Service_Information command) {
  if (!command.ma_task_command.has_value()) {
    return command;
  }
  auto& message = command.ma_task_command.value();
  agraHeaderWireToNative(message.message_header);
  for (auto& item : message.message_data.command) {
    item.capability.base.command_id.uuid =
        nativeUuidFromOms(item.capability.base.command_id.uuid);
    item.capability.capability_id.uuid =
        nativeUuidFromOms(item.capability.capability_id.uuid);
    item.capability.task_id.base.uuid =
        nativeUuidFromOms(item.capability.task_id.base.uuid);
  }
  return command;
}

inline agra_c2_provided::MA_TaskCommand_Service_Information
agraTaskCommandNativeToWire(
    agra_c2_provided::MA_TaskCommand_Service_Information command) {
  if (!command.ma_task_command.has_value()) {
    return command;
  }
  auto& message = command.ma_task_command.value();
  agraHeaderNativeToWire(message.message_header);
  for (auto& item : message.message_data.command) {
    item.capability.base.command_id.uuid =
        omsUuidFromNative(item.capability.base.command_id.uuid);
    item.capability.capability_id.uuid =
        omsUuidFromNative(item.capability.capability_id.uuid);
    item.capability.task_id.base.uuid =
        omsUuidFromNative(item.capability.task_id.base.uuid);
  }
  return command;
}

/// \brief Convert a bridge-produced MA_TaskCommandStatus from native form to
///        the OMS-JSON wire form the codec requires.
inline agra_c2_provided::MA_TaskCommandStatus_Service_Information
agraTaskCommandStatusNativeToWire(
    agra_c2_provided::MA_TaskCommandStatus_Service_Information status) {
  if (!status.ma_task_command_status.has_value()) {
    return status;
  }
  auto& message = status.ma_task_command_status.value();
  agraHeaderNativeToWire(message.message_header);
  message.message_data.base.command_id.uuid =
      omsUuidFromNative(message.message_data.base.command_id.uuid);
  if (message.message_data.base.command_processing_state_reason
          .has_value() &&
      message.message_data.base.command_processing_state_reason
          ->associated_id.has_value()) {
    message.message_data.base.command_processing_state_reason->associated_id
        ->uuid = omsUuidFromNative(message.message_data.base
                                        .command_processing_state_reason
                                        ->associated_id->uuid);
  }
  for (auto& activity : message.message_data.activity) {
    activity.activity_id.uuid = omsUuidFromNative(activity.activity_id.uuid);
  }
  return status;
}

inline agra_c2_provided::MA_TaskCommandStatus_Service_Information
agraTaskCommandStatusWireToNative(
    agra_c2_provided::MA_TaskCommandStatus_Service_Information status) {
  if (!status.ma_task_command_status.has_value()) {
    return status;
  }
  auto& message = status.ma_task_command_status.value();
  agraHeaderWireToNative(message.message_header);
  message.message_data.base.command_id.uuid =
      nativeUuidFromOms(message.message_data.base.command_id.uuid);
  if (message.message_data.base.command_processing_state_reason
          .has_value() &&
      message.message_data.base.command_processing_state_reason
          ->associated_id.has_value()) {
    message.message_data.base.command_processing_state_reason->associated_id
        ->uuid = nativeUuidFromOms(message.message_data.base
                                       .command_processing_state_reason
                                       ->associated_id->uuid);
  }
  for (auto& activity : message.message_data.activity) {
    activity.activity_id.uuid =
        nativeUuidFromOms(activity.activity_id.uuid);
  }
  return status;
}

template <typename VersionedId>
inline void agraVersionedIdWireToNative(VersionedId& id) {
  id.base.uuid = nativeUuidFromOms(id.base.uuid);
}

template <typename VersionedId>
inline void agraVersionedIdNativeToWire(VersionedId& id) {
  id.base.uuid = omsUuidFromNative(id.base.uuid);
}

inline void agraCannotComplyNativeToWire(
    tl::optional<agra::CannotComplyType>& reason) {
  if (reason.has_value() && reason->associated_id.has_value()) {
    reason->associated_id->uuid =
        omsUuidFromNative(reason->associated_id->uuid);
  }
}

inline agra_c2_provided::
    MA_MissionPlanCommand_Service_Information
agraMissionPlanCommandWireToNative(
    agra_c2_provided::
        MA_MissionPlanCommand_Service_Information command) {
  if (!command.ma_mission_plan_command.has_value()) {
    return command;
  }
  auto& message = command.ma_mission_plan_command.value();
  agraHeaderWireToNative(message.message_header);
  message.message_data.command_id.uuid =
      nativeUuidFromOms(message.message_data.command_id.uuid);
  message.message_data.inputs.planning_process_id.uuid =
      nativeUuidFromOms(
          message.message_data.inputs.planning_process_id.uuid);
  if (message.message_data.inputs.proposed_requirements.has_value()) {
    auto& requirements =
        message.message_data.inputs.proposed_requirements.value();
    for (auto& effect : requirements.proposed_effect) {
      agraVersionedIdWireToNative(effect.effect_id);
    }
    for (auto& action : requirements.proposed_action) {
      agraVersionedIdWireToNative(action.action_id);
    }
    for (auto& task : requirements.proposed_task) {
      agraVersionedIdWireToNative(task.task_id);
    }
    for (auto& response : requirements.proposed_response) {
      agraVersionedIdWireToNative(response.response_id);
    }
  }
  return command;
}

inline agra_c2_provided::
    MA_MissionPlanCommand_Service_Information
agraMissionPlanCommandNativeToWire(
    agra_c2_provided::
        MA_MissionPlanCommand_Service_Information command) {
  if (!command.ma_mission_plan_command.has_value()) {
    return command;
  }
  auto& message = command.ma_mission_plan_command.value();
  agraHeaderNativeToWire(message.message_header);
  message.message_data.command_id.uuid =
      omsUuidFromNative(message.message_data.command_id.uuid);
  message.message_data.inputs.planning_process_id.uuid =
      omsUuidFromNative(
          message.message_data.inputs.planning_process_id.uuid);
  if (message.message_data.inputs.proposed_requirements.has_value()) {
    auto& requirements =
        message.message_data.inputs.proposed_requirements.value();
    for (auto& effect : requirements.proposed_effect) {
      agraVersionedIdNativeToWire(effect.effect_id);
    }
    for (auto& action : requirements.proposed_action) {
      agraVersionedIdNativeToWire(action.action_id);
    }
    for (auto& task : requirements.proposed_task) {
      agraVersionedIdNativeToWire(task.task_id);
    }
    for (auto& response : requirements.proposed_response) {
      agraVersionedIdNativeToWire(response.response_id);
    }
  }
  return command;
}

inline agra_c2_provided::
    MA_MissionPlanCommandStatus_Service_Information
agraMissionPlanCommandStatusNativeToWire(
    agra_c2_provided::
        MA_MissionPlanCommandStatus_Service_Information status) {
  if (!status.ma_mission_plan_command_status.has_value()) {
    return status;
  }
  auto& message = status.ma_mission_plan_command_status.value();
  agraHeaderNativeToWire(message.message_header);
  message.message_data.command_id.uuid =
      omsUuidFromNative(message.message_data.command_id.uuid);
  agraCannotComplyNativeToWire(
      message.message_data.planning_status
          .command_processing_state_reason);
  if (message.message_data.resulting_plan_identifier.has_value()) {
    auto& plans =
        message.message_data.resulting_plan_identifier.value();
    for (auto& plan : plans.mission_plan_id) {
      agraVersionedIdNativeToWire(plan);
    }
  }
  return status;
}

inline agra_c2_provided::
    MA_MissionPlanCommandStatus_Service_Information
agraMissionPlanCommandStatusWireToNative(
    agra_c2_provided::
        MA_MissionPlanCommandStatus_Service_Information status) {
  if (!status.ma_mission_plan_command_status.has_value()) {
    return status;
  }
  auto& message = status.ma_mission_plan_command_status.value();
  agraHeaderWireToNative(message.message_header);
  message.message_data.command_id.uuid =
      nativeUuidFromOms(message.message_data.command_id.uuid);
  if (message.message_data.planning_status
          .command_processing_state_reason.has_value() &&
      message.message_data.planning_status
          .command_processing_state_reason->associated_id.has_value()) {
    message.message_data.planning_status
        .command_processing_state_reason->associated_id->uuid =
        nativeUuidFromOms(
            message.message_data.planning_status
                .command_processing_state_reason->associated_id->uuid);
  }
  if (message.message_data.resulting_plan_identifier.has_value()) {
    auto& plans =
        message.message_data.resulting_plan_identifier.value();
    for (auto& plan : plans.mission_plan_id) {
      agraVersionedIdWireToNative(plan);
    }
  }
  return status;
}

inline agra_c2_provided::
    MA_MissionPlanActivationCommand_Service_Information
agraMissionPlanActivationCommandWireToNative(
    agra_c2_provided::
        MA_MissionPlanActivationCommand_Service_Information command) {
  if (!command.ma_mission_plan_activation_command.has_value()) {
    return command;
  }
  auto& message =
      command.ma_mission_plan_activation_command.value();
  agraHeaderWireToNative(message.message_header);
  message.message_data.command_id.uuid =
      nativeUuidFromOms(message.message_data.command_id.uuid);
  for (auto& approval_id :
       message.message_data.approval_management_command_id) {
    approval_id.uuid = nativeUuidFromOms(approval_id.uuid);
  }
  for (auto& activation : message.message_data.command) {
    agraVersionedIdWireToNative(activation.mission_plan_id);
  }
  return command;
}

inline agra_c2_provided::
    MA_MissionPlanActivationCommand_Service_Information
agraMissionPlanActivationCommandNativeToWire(
    agra_c2_provided::
        MA_MissionPlanActivationCommand_Service_Information command) {
  if (!command.ma_mission_plan_activation_command.has_value()) {
    return command;
  }
  auto& message =
      command.ma_mission_plan_activation_command.value();
  agraHeaderNativeToWire(message.message_header);
  message.message_data.command_id.uuid =
      omsUuidFromNative(message.message_data.command_id.uuid);
  for (auto& approval_id :
       message.message_data.approval_management_command_id) {
    approval_id.uuid = omsUuidFromNative(approval_id.uuid);
  }
  for (auto& activation : message.message_data.command) {
    agraVersionedIdNativeToWire(activation.mission_plan_id);
  }
  return command;
}

inline agra_c2_provided::
    MA_MissionPlanActivationCommandStatus_Service_Information
agraMissionPlanActivationCommandStatusNativeToWire(
    agra_c2_provided::
        MA_MissionPlanActivationCommandStatus_Service_Information
            status) {
  if (!status.ma_mission_plan_activation_command_status.has_value()) {
    return status;
  }
  auto& message =
      status.ma_mission_plan_activation_command_status.value();
  agraHeaderNativeToWire(message.message_header);
  message.message_data.command_id.uuid =
      omsUuidFromNative(message.message_data.command_id.uuid);
  agraCannotComplyNativeToWire(
      message.message_data.activation_status
          .command_processing_state_reason);
  for (auto& activation :
       message.message_data.activation_command_by_state) {
    for (auto& plan : activation.plans.mission_plan_id) {
      agraVersionedIdNativeToWire(plan);
    }
  }
  return status;
}

inline agra_c2_provided::
    MA_MissionPlanActivationCommandStatus_Service_Information
agraMissionPlanActivationCommandStatusWireToNative(
    agra_c2_provided::
        MA_MissionPlanActivationCommandStatus_Service_Information
            status) {
  if (!status.ma_mission_plan_activation_command_status.has_value()) {
    return status;
  }
  auto& message =
      status.ma_mission_plan_activation_command_status.value();
  agraHeaderWireToNative(message.message_header);
  message.message_data.command_id.uuid =
      nativeUuidFromOms(message.message_data.command_id.uuid);
  if (message.message_data.activation_status
          .command_processing_state_reason.has_value() &&
      message.message_data.activation_status
          .command_processing_state_reason->associated_id.has_value()) {
    message.message_data.activation_status
        .command_processing_state_reason->associated_id->uuid =
        nativeUuidFromOms(
            message.message_data.activation_status
                .command_processing_state_reason->associated_id->uuid);
  }
  for (auto& activation :
       message.message_data.activation_command_by_state) {
    for (auto& plan : activation.plans.mission_plan_id) {
      agraVersionedIdWireToNative(plan);
    }
  }
  return status;
}

inline void agraSystemNativeToWire(agra::SystemID_Type& system) {
  system.uuid = omsUuidFromNative(system.uuid);
}

inline void agraSystemWireToNative(agra::SystemID_Type& system) {
  system.uuid = nativeUuidFromOms(system.uuid);
}

inline agra_c2_provided::MA_MissionPlan_Service_Information
agraMissionPlanNativeToWire(
    agra_c2_provided::MA_MissionPlan_Service_Information information) {
  if (!information.ma_mission_plan.has_value()) {
    return information;
  }
  auto& message = information.ma_mission_plan.value();
  agraHeaderNativeToWire(message.message_header);
  agraVersionedIdNativeToWire(
      message.message_data.mission_plan_id);
  if (message.message_data.mission_plan_command_id.has_value() &&
      message.message_data.mission_plan_command_id
          ->mission_plan_command_id.has_value()) {
    auto& command_id =
        message.message_data.mission_plan_command_id
            ->mission_plan_command_id.value();
    command_id.uuid = omsUuidFromNative(command_id.uuid);
  }
  if (message.message_data.applicability.planned_for_id.has_value()) {
    agraSystemNativeToWire(
        message.message_data.applicability.planned_for_id.value());
  }
  if (message.message_data.applicability.applicable_to_i_ds
          .system_id.has_value()) {
    agraSystemNativeToWire(
        message.message_data.applicability.applicable_to_i_ds
            .system_id.value());
  }
  if (message.message_data.sub_plans.has_value()) {
    for (auto& action_plan_id :
         message.message_data.sub_plans->action_plan_id) {
      agraVersionedIdNativeToWire(action_plan_id);
    }
    for (auto& route_plan_id :
         message.message_data.sub_plans->route_plan_id) {
      agraVersionedIdNativeToWire(route_plan_id);
    }
  }
  if (message.message_data.execution_sequence.has_value()) {
    auto& sequence =
        message.message_data.execution_sequence.value();
    sequence.initial_execution_plan_set_id.uuid =
        omsUuidFromNative(
            sequence.initial_execution_plan_set_id.uuid);
    for (auto& plan_set : sequence.route_execution_plan_set) {
      plan_set.execution_plan_set_id.uuid =
          omsUuidFromNative(plan_set.execution_plan_set_id.uuid);
      agraVersionedIdNativeToWire(plan_set.route_plan_id);
      if (plan_set.next_execution_plan_set_id.has_value()) {
        plan_set.next_execution_plan_set_id->uuid =
            omsUuidFromNative(
                plan_set.next_execution_plan_set_id->uuid);
      }
    }
  }
  return information;
}

inline agra_c2_provided::MA_MissionPlan_Service_Information
agraMissionPlanWireToNative(
    agra_c2_provided::MA_MissionPlan_Service_Information information) {
  if (!information.ma_mission_plan.has_value()) {
    return information;
  }
  auto& message = information.ma_mission_plan.value();
  agraHeaderWireToNative(message.message_header);
  agraVersionedIdWireToNative(
      message.message_data.mission_plan_id);
  if (message.message_data.mission_plan_command_id.has_value() &&
      message.message_data.mission_plan_command_id
          ->mission_plan_command_id.has_value()) {
    auto& command_id =
        message.message_data.mission_plan_command_id
            ->mission_plan_command_id.value();
    command_id.uuid = nativeUuidFromOms(command_id.uuid);
  }
  if (message.message_data.applicability.planned_for_id.has_value()) {
    agraSystemWireToNative(
        message.message_data.applicability.planned_for_id.value());
  }
  if (message.message_data.applicability.applicable_to_i_ds
          .system_id.has_value()) {
    agraSystemWireToNative(
        message.message_data.applicability.applicable_to_i_ds
            .system_id.value());
  }
  if (message.message_data.sub_plans.has_value()) {
    for (auto& action_plan_id :
         message.message_data.sub_plans->action_plan_id) {
      agraVersionedIdWireToNative(action_plan_id);
    }
    for (auto& route_plan_id :
         message.message_data.sub_plans->route_plan_id) {
      agraVersionedIdWireToNative(route_plan_id);
    }
  }
  if (message.message_data.execution_sequence.has_value()) {
    auto& sequence =
        message.message_data.execution_sequence.value();
    sequence.initial_execution_plan_set_id.uuid =
        nativeUuidFromOms(
            sequence.initial_execution_plan_set_id.uuid);
    for (auto& plan_set : sequence.route_execution_plan_set) {
      plan_set.execution_plan_set_id.uuid =
          nativeUuidFromOms(plan_set.execution_plan_set_id.uuid);
      agraVersionedIdWireToNative(plan_set.route_plan_id);
      if (plan_set.next_execution_plan_set_id.has_value()) {
        plan_set.next_execution_plan_set_id->uuid =
            nativeUuidFromOms(
                plan_set.next_execution_plan_set_id->uuid);
      }
    }
  }
  return information;
}

inline agra_c2_consumed::MA_ApprovalRequest_Service_Information
agraApprovalRequestNativeToWire(
    agra_c2_consumed::MA_ApprovalRequest_Service_Information
        information) {
  if (!information.ma_approval_request.has_value()) {
    return information;
  }
  auto& message = information.ma_approval_request.value();
  agraHeaderNativeToWire(message.message_header);
  message.message_data.request_id.uuid =
      omsUuidFromNative(message.message_data.request_id.uuid);
  if (message.message_data.approver.non_operator_identifier
          .has_value()) {
    agraSystemNativeToWire(
        message.message_data.approver.non_operator_identifier
            ->system_id);
  }
  message.message_data.approval_references.approval_policy_id.uuid =
      omsUuidFromNative(
          message.message_data.approval_references
              .approval_policy_id.uuid);
  auto& approval_item =
      message.message_data.approval_references.approval_item;
  if (approval_item.plan_approval.has_value() &&
      approval_item.plan_approval->mission_plan_id.has_value()) {
    agraVersionedIdNativeToWire(
        approval_item.plan_approval->mission_plan_id.value());
  }
  return information;
}

inline agra_c2_consumed::MA_ApprovalRequest_Service_Information
agraApprovalRequestWireToNative(
    agra_c2_consumed::MA_ApprovalRequest_Service_Information
        information) {
  if (!information.ma_approval_request.has_value()) {
    return information;
  }
  auto& message = information.ma_approval_request.value();
  agraHeaderWireToNative(message.message_header);
  message.message_data.request_id.uuid =
      nativeUuidFromOms(message.message_data.request_id.uuid);
  if (message.message_data.approver.non_operator_identifier
          .has_value()) {
    agraSystemWireToNative(
        message.message_data.approver.non_operator_identifier
            ->system_id);
  }
  message.message_data.approval_references.approval_policy_id.uuid =
      nativeUuidFromOms(
          message.message_data.approval_references
              .approval_policy_id.uuid);
  auto& approval_item =
      message.message_data.approval_references.approval_item;
  if (approval_item.plan_approval.has_value() &&
      approval_item.plan_approval->mission_plan_id.has_value()) {
    agraVersionedIdWireToNative(
        approval_item.plan_approval->mission_plan_id.value());
  }
  return information;
}

inline agra_c2_consumed::MA_ApprovalRequestStatus_Service_Information
agraApprovalStatusNativeToWire(
    agra_c2_consumed::
        MA_ApprovalRequestStatus_Service_Information information) {
  if (!information.ma_approval_request_status.has_value()) {
    return information;
  }
  auto& message = information.ma_approval_request_status.value();
  agraHeaderNativeToWire(message.message_header);
  message.message_data.request_id.uuid =
      omsUuidFromNative(message.message_data.request_id.uuid);
  return information;
}

inline agra_c2_consumed::MA_ApprovalRequestStatus_Service_Information
agraApprovalStatusWireToNative(
    agra_c2_consumed::
        MA_ApprovalRequestStatus_Service_Information information) {
  if (!information.ma_approval_request_status.has_value()) {
    return information;
  }
  auto& message = information.ma_approval_request_status.value();
  agraHeaderWireToNative(message.message_header);
  message.message_data.request_id.uuid =
      nativeUuidFromOms(message.message_data.request_id.uuid);
  return information;
}

inline agra_c2_provided::
    MA_MissionPlanExecutionStatus_Service_Information
agraExecutionStatusNativeToWire(
    agra_c2_provided::
        MA_MissionPlanExecutionStatus_Service_Information information) {
  if (!information.ma_mission_plan_execution_status.has_value()) {
    return information;
  }
  auto& message =
      information.ma_mission_plan_execution_status.value();
  agraHeaderNativeToWire(message.message_header);
  agraSystemNativeToWire(message.message_data.system_id);
  for (auto& plan_status :
       message.message_data.plan_execution_status) {
    agraVersionedIdNativeToWire(plan_status.mission_plan_id);
    for (auto& plan_set :
         plan_status.execution_plan_set_status) {
      plan_set.execution_plan_set_id.uuid =
          omsUuidFromNative(plan_set.execution_plan_set_id.uuid);
    }
  }
  return information;
}

inline agra_c2_provided::
    MA_MissionPlanExecutionStatus_Service_Information
agraExecutionStatusWireToNative(
    agra_c2_provided::
        MA_MissionPlanExecutionStatus_Service_Information information) {
  if (!information.ma_mission_plan_execution_status.has_value()) {
    return information;
  }
  auto& message =
      information.ma_mission_plan_execution_status.value();
  agraHeaderWireToNative(message.message_header);
  agraSystemWireToNative(message.message_data.system_id);
  for (auto& plan_status :
       message.message_data.plan_execution_status) {
    agraVersionedIdWireToNative(plan_status.mission_plan_id);
    for (auto& plan_set :
         plan_status.execution_plan_set_status) {
      plan_set.execution_plan_set_id.uuid =
          nativeUuidFromOms(plan_set.execution_plan_set_id.uuid);
    }
  }
  return information;
}

inline agra_c2_provided::MissionContingencyAlert_Service_Information
agraContingencyAlertNativeToWire(
    agra_c2_provided::MissionContingencyAlert_Service_Information
        information) {
  if (!information.mission_contingency_alert.has_value()) {
    return information;
  }
  auto& message = information.mission_contingency_alert.value();
  agraHeaderNativeToWire(message.message_header);
  message.message_data.mission_contingency_alert_id.uuid =
      omsUuidFromNative(
          message.message_data.mission_contingency_alert_id.uuid);
  agraSystemNativeToWire(message.message_data.source_system_id);
  for (auto& condition :
       message.message_data.contingency_condition) {
    agraSystemNativeToWire(condition.conflicted_system_id);
  }
  return information;
}

inline agra_c2_provided::MissionContingencyAlert_Service_Information
agraContingencyAlertWireToNative(
    agra_c2_provided::MissionContingencyAlert_Service_Information
        information) {
  if (!information.mission_contingency_alert.has_value()) {
    return information;
  }
  auto& message = information.mission_contingency_alert.value();
  agraHeaderWireToNative(message.message_header);
  message.message_data.mission_contingency_alert_id.uuid =
      nativeUuidFromOms(
          message.message_data.mission_contingency_alert_id.uuid);
  agraSystemWireToNative(message.message_data.source_system_id);
  for (auto& condition :
       message.message_data.contingency_condition) {
    agraSystemWireToNative(condition.conflicted_system_id);
  }
  return information;
}

}  // namespace detail

/// \brief The Mission Autonomy side of the A-GRA C2 boundary as a component.
class AgraMaComponent final : public pcl::Component,
                              private agra_c2_provided::MaTaskcommandStatusPortHandler {
public:
  /// \param bridge The planning boundary. Its lifetime must outlive this
  ///        component; it is the port handler, not a copy of one.
  /// \param content_type Codec selection. Defaults to OMS-JSON because that is
  ///        what the LA-CAL transport accepts and what AutoMTK's contract
  ///        generates; a shared-memory deployment may use any codec the
  ///        bindings provide.
  AgraMaComponent(pcl::Executor& executor, AgraMaBridge& bridge,
                  std::string content_type = "application/oms-json",
                  double tick_rate_hz = 20.0)
      : pcl::Component("automtk_agra_ma"),
        bridge_(bridge),
        task_command_port_(*this, executor, *this, std::move(content_type)) {
    setTickRateHz(tick_rate_hz);
  }

  /// \brief Statuses drained on the last tick. Zero is normal and not an error.
  std::size_t published_status_count() const { return published_; }

  /// Commands the port delivered, whatever the bridge then decided.
  std::size_t received_command_count() const { return received_; }

protected:
  pcl_status_t on_configure() override { return task_command_port_.bind(); }

  /// Received commands are the bridge's to judge; this only counts them, so a
  /// silent boundary can be told apart from one that is deciding to refuse.
  ///
  /// `AgraMaBridge::onCommand` queues the correlated status -- Accepted or
  /// Rejected -- before it decides whether to also throw; throwing is how it
  /// tells a direct, in-process caller why, but the status is already queued
  /// either way (see `agra_ma_bridge.cpp`). A rejection must not bring this
  /// component down and strand the status it just queued, so the throw is
  /// caught and logged, not propagated.
  void onCommand(
      const agra_c2_provided::MA_TaskCommand_Service_Information& command)
      override {
    ++received_;
    try {
      bridge_.onCommand(detail::agraTaskCommandWireToNative(command));
    } catch (const std::exception& error) {
      std::fprintf(stderr, "[automtk_agra_ma] MA_TaskCommand refused: %s\n",
                   error.what());
    }
  }

  /// Drain whatever the bridge decided since the last tick and publish it.
  ///
  /// The bridge produces statuses as a side effect of `onCommand`, which the
  /// port calls; polling its read accessor here is what turns those into wire
  /// traffic. A publish failure is returned rather than swallowed: a status
  /// that cannot reach the commanding side leaves it waiting on a decision
  /// that has already been made.
  pcl_status_t on_tick(double) override {
    for (const auto& status :
         bridge_.handleMaTaskcommandstatusRead(
             agra_c2_provided::Empty{})) {
      const pcl_status_t published = task_command_port_.publishStatus(
          detail::agraTaskCommandStatusNativeToWire(status));
      if (published != PCL_OK) {
        return published;
      }
      ++published_;
    }
    return PCL_OK;
  }

private:
  AgraMaBridge& bridge_;
  agra_c2_provided::MaTaskcommandStatusPortProvider task_command_port_;
  std::size_t published_ = 0;
  std::size_t received_ = 0;
};

/// \brief Mission Autonomy's MA_MissionPlanCommand/Status port adapter.
class AgraMaMissionPlanComponent final
    : public pcl::Component,
      private agra_c2_provided::
          MaMissionplancommandStatusPortHandler {
public:
  AgraMaMissionPlanComponent(
      pcl::Executor& executor, AgraMaBridge& bridge,
      std::string content_type = "application/oms-json",
      double tick_rate_hz = 20.0)
      : pcl::Component("automtk_agra_ma_mission_plan"),
        bridge_(bridge),
        mission_plan_port_(
            *this, executor, *this, std::move(content_type)) {
    setTickRateHz(tick_rate_hz);
  }

  std::size_t published_status_count() const { return published_; }
  std::size_t received_command_count() const { return received_; }

protected:
  pcl_status_t on_configure() override {
    return mission_plan_port_.bind();
  }

  void onCommand(
      const agra_c2_provided::
          MA_MissionPlanCommand_Service_Information& command)
      override {
    ++received_;
    try {
      bridge_.onCommand(
          detail::agraMissionPlanCommandWireToNative(command));
    } catch (const std::exception& error) {
      std::fprintf(
          stderr,
          "[automtk_agra_ma_mission_plan] command refused: %s\n",
          error.what());
    }
  }

  pcl_status_t on_tick(double) override {
    for (auto& status :
         bridge_.handleMaMissionplancommandstatusRead(
             agra_c2_provided::Empty{})) {
      const pcl_status_t published =
          mission_plan_port_.publishStatus(
              detail::agraMissionPlanCommandStatusNativeToWire(
                  std::move(status)));
      if (published != PCL_OK) {
        return published;
      }
      ++published_;
    }
    return PCL_OK;
  }

private:
  AgraMaBridge& bridge_;
  agra_c2_provided::MaMissionplancommandStatusPortProvider
      mission_plan_port_;
  std::size_t published_ = 0;
  std::size_t received_ = 0;
};

/// \brief Mission Autonomy's activation-command/status port adapter.
class AgraMaMissionPlanActivationComponent final
    : public pcl::Component,
      private agra_c2_provided::
          MaMissionplanactivationcommandStatusPortHandler {
public:
  AgraMaMissionPlanActivationComponent(
      pcl::Executor& executor, AgraMaBridge& bridge,
      std::string content_type = "application/oms-json",
      double tick_rate_hz = 20.0)
      : pcl::Component("automtk_agra_ma_activation"),
        bridge_(bridge),
        activation_port_(
            *this, executor, *this, std::move(content_type)) {
    setTickRateHz(tick_rate_hz);
  }

  std::size_t published_status_count() const { return published_; }
  std::size_t received_command_count() const { return received_; }

protected:
  pcl_status_t on_configure() override {
    return activation_port_.bind();
  }

  void onCommand(
      const agra_c2_provided::
          MA_MissionPlanActivationCommand_Service_Information&
              command) override {
    ++received_;
    try {
      bridge_.onCommand(
          detail::
              agraMissionPlanActivationCommandWireToNative(
                  command));
    } catch (const std::exception& error) {
      std::fprintf(
          stderr,
          "[automtk_agra_ma_activation] command refused: %s\n",
          error.what());
    }
  }

  pcl_status_t on_tick(double) override {
    for (auto& status :
         bridge_.
             handleMaMissionplanactivationcommandstatusRead(
                 agra_c2_provided::Empty{})) {
      const pcl_status_t published =
          activation_port_.publishStatus(
              detail::
                  agraMissionPlanActivationCommandStatusNativeToWire(
                      std::move(status)));
      if (published != PCL_OK) {
        return published;
      }
      ++published_;
    }
    return PCL_OK;
  }

private:
  AgraMaBridge& bridge_;
  agra_c2_provided::
      MaMissionplanactivationcommandStatusPortProvider
          activation_port_;
  std::size_t published_ = 0;
  std::size_t received_ = 0;
};

}  // namespace ame
