#pragma once

/// \file agra_ma_grounding_component.hpp
/// \brief Host `Grounding_Service` on PYRAMID's generated Port layer and
///        forward it into `AgraMaBridge`.
///
/// This is the receiving side of the grounding-over-the-wire boundary. A
/// Python host runs `AmeProblemBuilder`/the planner and hands the exact domain,
/// problem, confirmed authority facts, declarative action bindings, task goal
/// fluents, and per-action symbolic/kinematic grounding to the standalone
/// process that owns the real `AgraMaBridge`. `GroundingServiceHandler`
/// installs those planning inputs without deriving or changing the plan, then
/// forwards task/plan grounding into
/// `AgraMaBridge::registerTaskGrounding`/`supplyPlanGrounding`.
///
/// `TaskGrounding.task_payload` and `ActionGrounding.action_payload` carry
/// "native JSON" -- `agra_codec::fromJson`'s own pivot dialect, with
/// **base64** UUIDs, not the 32-hex-character OMS-JSON wire form
/// (`ame/agra_oms_uuid.h`) real A-GRA peers use. This is deliberate, not an
/// oversight: `_agra_protobuf_to_native_json` (`ame_py.cpp`) already
/// produces exactly this dialect host-side with no new Python code, and
/// `ame::nativeUuidFromNativeJson` (`ame/agra_native_json_uuid.h`) is the
/// matching pure-C++ decode, mirroring `ame_py.cpp`'s own
/// `decodeTask`/`decodeAction`. Confusing the two UUID dialects would
/// silently corrupt every UUID rather than fail loudly -- both are
/// plausible-looking strings.
///
/// `PlanGrounding.mission_plan_command_uuid` carries the same "native
/// JSON" base64 dialect, not raw bytes: `AgraMaComposite.supply_plan_
/// grounding` passes it straight through with no decode step *in
/// process*, but the generated JSON codec (`pyramid_components_agra_ma_
/// grounding_services_provided_codec.cpp`) copies a `bytes` field
/// straight into an `nlohmann::json` string with no base64 step of its
/// own -- raw 16-byte UUIDs are not guaranteed valid UTF-8, so sending
/// them unconverted silently breaks JSON serialization the moment a real
/// Port is involved. Caught by the Port round-trip test
/// (`test_agra_ma_grounding_port_round_trip.cpp`), not the direct-call
/// unit test, which never serializes to JSON at all.

#include "ame/agra_ma_bridge.h"
#include "ame/agra_native_json_uuid.h"
#include "ame/action_registry.h"
#include "ame/pddl_parser.h"
#include "ame/world_model.h"

#include <pyramid_data_model_agra_codec.hpp>
// pyramid_services_agra_ma_grounding_provided(_components).hpp reference
// TaskGrounding/PlanGrounding/ActionGrounding/Waypoint/CommandParameter
// without including the header that declares them -- every other
// component in this tree only carries pyramid.data_model.* types in its
// service, already covered by the umbrella pyramid_data_model_types.hpp,
// so this gap in the generator was never exercised before. Included
// explicitly here rather than patched in the generator.
#include <pyramid_components_agra_ma_grounding_services_provided_types.hpp>
#include <pyramid_services_agra_ma_grounding_provided_components.hpp>

#include <string>
#include <utility>
#include <vector>

namespace ame {

namespace grounding_service =
    pyramid::components::agra_ma_grounding::services::provided;
namespace agra_codec = pyramid::domain_model::agra;

namespace detail {

inline void decodeGroundingHeader(agra::HeaderType& header) {
  header.system_id.uuid = nativeUuidFromNativeJson(header.system_id.uuid);
  if (header.mission_id.has_value()) {
    header.mission_id->base.uuid =
        nativeUuidFromNativeJson(header.mission_id->base.uuid);
  }
}

inline void encodeGroundingHeader(agra::HeaderType& header) {
  header.system_id.uuid =
      nativeJsonUuidFromNative(header.system_id.uuid);
  if (header.mission_id.has_value()) {
    header.mission_id->base.uuid =
        nativeJsonUuidFromNative(header.mission_id->base.uuid);
  }
}

template <typename VersionedId>
inline void encodeGroundingVersionedId(VersionedId& id) {
  id.base.uuid = nativeJsonUuidFromNative(id.base.uuid);
}

/// \brief Decode one `TaskGrounding.task_payload` into the native
///        `MA_TaskMT` `AgraMaBridge::registerTaskGrounding` requires.
///        Mirrors `ame_py.cpp`'s `decodeTask()` field-for-field.
inline agra::MA_TaskMT decodeGroundingTaskPayload(const std::string& payload) {
  auto task =
      agra_codec::fromJson(payload, static_cast<agra::MA_TaskMT*>(nullptr));
  decodeGroundingHeader(task.message_header);
  task.message_data.task_id.base.uuid =
      nativeUuidFromNativeJson(task.message_data.task_id.base.uuid);
  return task;
}

/// \brief Decode one `ActionGrounding.action_payload` into the native
///        `MA_ActionMT` `AgraPlanElementGrounding::action` requires.
///        Mirrors `ame_py.cpp`'s `decodeAction()` field-for-field.
inline agra::MA_ActionMT decodeGroundingActionPayload(
    const std::string& payload) {
  auto action = agra_codec::fromJson(payload,
                                     static_cast<agra::MA_ActionMT*>(nullptr));
  decodeGroundingHeader(action.message_header);
  action.message_data.action_id.base.uuid =
      nativeUuidFromNativeJson(action.message_data.action_id.base.uuid);
  return action;
}

inline std::string encodeGroundingApprovalRequestPayload(
    agra::MA_ApprovalRequestMT request) {
  encodeGroundingHeader(request.message_header);
  request.message_data.request_id.uuid =
      nativeJsonUuidFromNative(
          request.message_data.request_id.uuid);
  if (request.message_data.approver.non_operator_identifier
          .has_value()) {
    auto& system =
        request.message_data.approver.non_operator_identifier
            ->system_id;
    system.uuid = nativeJsonUuidFromNative(system.uuid);
  }
  request.message_data.approval_references
      .approval_policy_id.uuid =
      nativeJsonUuidFromNative(
          request.message_data.approval_references
              .approval_policy_id.uuid);
  auto& approval_item =
      request.message_data.approval_references.approval_item;
  if (approval_item.plan_approval.has_value() &&
      approval_item.plan_approval->mission_plan_id.has_value()) {
    encodeGroundingVersionedId(
        approval_item.plan_approval->mission_plan_id.value());
  }
  return agra_codec::toJson(request);
}

inline agra::MA_ApprovalRequestStatusMT
decodeGroundingApprovalStatusPayload(const std::string& payload) {
  auto status = agra_codec::fromJson(
      payload,
      static_cast<agra::MA_ApprovalRequestStatusMT*>(nullptr));
  decodeGroundingHeader(status.message_header);
  status.message_data.request_id.uuid =
      nativeUuidFromNativeJson(
          status.message_data.request_id.uuid);
  return status;
}

inline AgraPlanElementGrounding decodeGroundingActionElement(
    const grounding_service::ActionGrounding& wire) {
  AgraPlanElementGrounding element;
  element.action_signature = wire.action_signature;
  element.action = decodeGroundingActionPayload(wire.action_payload);
  element.requires_kinematics = wire.requires_kinematics;
  element.route_plan_id = wire.route_plan_id;
  element.waypoints.reserve(wire.waypoints.size());
  for (const auto& waypoint : wire.waypoints) {
    element.waypoints.push_back(
        {waypoint.latitude_deg, waypoint.longitude_deg, waypoint.altitude_m});
  }
  for (const auto& parameter : wire.command_parameters) {
    element.command_parameters[parameter.key] = parameter.value;
  }
  return element;
}

}  // namespace detail

/// \brief `Grounding_Service` provided-side handler: decodes the wire form
///        and forwards straight into `AgraMaBridge`. No state of its own.
class GroundingServiceHandler : public grounding_service::ProvidedHandler {
public:
  explicit GroundingServiceHandler(AgraMaBridge& bridge) : bridge_(&bridge) {}

  GroundingServiceHandler(
      AgraMaBridge& bridge, WorldModel& world_model,
      ActionRegistry& action_registry)
      : bridge_(&bridge),
        world_model_(&world_model),
        action_registry_(&action_registry) {}

  pyramid::domain_model::base::Identifier onGroundingCreateTaskGrounding(
      const grounding_service::TaskGrounding& request) override {
    configurePlanningInputs(request);
    bridge_->registerTaskGrounding(
        detail::decodeGroundingTaskPayload(request.task_payload),
        request.goal_fluents);
    return "grounding-task-accepted";
  }

  pyramid::domain_model::base::Identifier onGroundingCreatePlanGrounding(
      const grounding_service::PlanGrounding& request) override {
    std::vector<AgraPlanElementGrounding> elements;
    elements.reserve(request.grounding.size());
    for (const auto& item : request.grounding) {
      elements.push_back(detail::decodeGroundingActionElement(item));
    }
    bridge_->supplyPlanGrounding(
        nativeUuidFromNativeJson(request.mission_plan_command_uuid),
        std::move(elements));
    return "grounding-plan-accepted";
  }

  grounding_service::NativePayload
  onGroundingReadApprovalRequests(
      const grounding_service::Empty&) override {
    auto requests = bridge_->pullApprovalRequests();
    if (requests.size() > 1u) {
      throw std::runtime_error(
          "AgraMaBridge produced multiple undelivered approval requests");
    }
    if (requests.empty()) {
      return {};
    }
    auto& information = requests.front();
    if (!information.ma_approval_request.has_value()) {
      throw std::runtime_error(
          "AgraMaBridge produced an approval request without a payload");
    }
    grounding_service::NativePayload response;
    response.payload =
        detail::encodeGroundingApprovalRequestPayload(
            std::move(
                information.ma_approval_request.value()));
    return response;
  }

  pyramid::domain_model::base::Identifier
  onGroundingCreateApprovalStatus(
      const grounding_service::NativePayload& request) override {
    agra_c2_consumed::
        MA_ApprovalRequestStatus_Service_Information information;
    information.ma_approval_request_status =
        detail::decodeGroundingApprovalStatusPayload(
            request.payload);
    bridge_->ingestApprovalStatus(information);
    return "approval-status-accepted";
  }

private:
  void configurePlanningInputs(
      const grounding_service::TaskGrounding& request) {
    const bool supplied =
        !request.domain_pddl.empty() ||
        !request.problem_pddl.empty() ||
        !request.action_bindings.empty() ||
        !request.confirmed_fluents.empty();
    if (world_model_ == nullptr || action_registry_ == nullptr) {
      if (supplied) {
        throw std::invalid_argument(
            "TaskGrounding supplied dynamic planning inputs to a "
            "preconfigured grounding provider");
      }
      return;
    }
    if (request.domain_pddl.empty() ||
        request.problem_pddl.empty() ||
        request.action_bindings.empty()) {
      throw std::invalid_argument(
          "TaskGrounding for a dynamic composite requires domain_pddl, "
          "problem_pddl, and action_bindings");
    }

    *world_model_ = WorldModel{};
    PddlParser::parseFromString(
        request.domain_pddl, request.problem_pddl, *world_model_);
    for (const auto& fact : request.confirmed_fluents) {
      if (fact.empty()) {
        throw std::invalid_argument(
            "TaskGrounding contains an empty confirmed fluent");
      }
      if (!world_model_->getFact(fact)) {
        throw std::invalid_argument(
            "TaskGrounding confirmed fluent is not true in problem: " +
            fact);
      }
      world_model_->setFact(
          fact, true, "automtk_wire_grounding",
          FactAuthority::CONFIRMED);
    }
    for (const auto& binding : request.action_bindings) {
      if (binding.pddl_name.empty() ||
          binding.implementation.empty()) {
        throw std::invalid_argument(
            "TaskGrounding contains an incomplete action binding");
      }
      if (binding.binding_kind == "action") {
        action_registry_->registerAction(
            binding.pddl_name, binding.implementation,
            binding.reactive);
      } else if (binding.binding_kind == "subtree") {
        action_registry_->registerActionSubTree(
            binding.pddl_name, binding.implementation,
            binding.reactive);
      } else {
        throw std::invalid_argument(
            "TaskGrounding action binding '" +
            binding.pddl_name +
            "' has unsupported binding_kind '" +
            binding.binding_kind + "'");
      }
    }
  }

  AgraMaBridge* bridge_;
  WorldModel* world_model_ = nullptr;
  ActionRegistry* action_registry_ = nullptr;
};

}  // namespace ame
