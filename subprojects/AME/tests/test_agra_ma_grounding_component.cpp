#include <gtest/gtest.h>

#include "ame/action_registry.h"
#include "ame/agra_ma_bridge.h"
#include "ame/agra_ma_grounding_component.hpp"
#include "ame/agra_native_json_uuid.h"
#include "ame/current_ame_backend_adapter.h"
#include "ame/world_model.h"

#include <pyramid_data_model_agra_codec.hpp>

#include <string>
#include <vector>

namespace {

namespace grounding_service =
    pyramid::components::agra_ma_grounding::services::provided;

constexpr const char* kSchemaVersion =
    "005.0a.ASK-20260423-f1380e7";

ame::WorldModel buildDomain() {
  ame::WorldModel wm;
  auto& types = wm.typeSystem();
  types.addType("object");
  types.addType("location", "object");
  types.addType("sector", "location");
  types.addType("robot", "object");
  wm.addObject("uav1", "robot");
  wm.addObject("sector_a", "sector");
  wm.registerPredicate("at", {"robot", "location"});
  wm.registerPredicate("searched", {"sector"});
  wm.registerAction("search",
                    {"?r", "?s"},
                    {"robot", "sector"},
                    {"(at ?r ?s)"},
                    {"(searched ?s)"},
                    {});
  wm.setFact("(at uav1 sector_a)",
             true,
             "test",
             ame::FactAuthority::CONFIRMED);
  return wm;
}

ame::ActionRegistry buildRegistry() {
  ame::ActionRegistry registry;
  registry.registerActionSubTree(
      "search",
      "<InvokeService service_name=\"imaging\" operation=\"search\" "
      "param_names=\"?robot;?sector\" "
      "param_values=\"{param0};{param1}\" timeout_ms=\"0\"/>");
  return registry;
}

ame::agra::HeaderType header(const std::string& sender = "stub-c2") {
  ame::agra::HeaderType value;
  value.system_id.uuid = ame::AgraMaBridge::deterministicUuid(sender);
  value.system_id.descriptive_label = sender;
  value.timestamp = "2026-07-25T12:00:00Z";
  value.schema_version = kSchemaVersion;
  value.mode = ame::agra::MessageModeEnum::Simulation;
  return value;
}

ame::AgraMaBridgeOptions bridgeOptions() {
  ame::AgraMaBridgeOptions options;
  options.system_uuid =
      ame::AgraMaBridge::deterministicUuid("ame.current_stack");
  options.approval_authority_system_uuid =
      ame::AgraMaBridge::deterministicUuid("stub-c2");
  options.message_mode = ame::agra::MessageModeEnum::Simulation;
  options.approval_timeout_seconds = 60;
  options.backend_policy.max_replans = 3;
  options.backend_policy.require_plan_approval = false;
  return options;
}

ame::agra::MA_TaskMT task(const std::string& task_uuid) {
  ame::agra::MA_TaskMT value;
  value.message_header = header();
  ame::agra::MissionID_Type mission_id;
  mission_id.base.uuid = ame::AgraMaBridge::deterministicUuid("mission-1");
  mission_id.base.descriptive_label = "mission-1";
  value.message_header.mission_id = mission_id;
  value.object_state = ame::agra::ObjectStateEnum::New;
  value.message_data.task_id.base.uuid = task_uuid;
  value.message_data.task_id.base.descriptive_label = "search-task";
  value.message_data.task_id.version = 1;
  value.message_data.task_type.po = ame::agra::PO_TaskType{};
  value.message_data.task_plurality =
      ame::agra::MA_TaskPluralityEnum::SingleEntity;
  return value;
}

ame::agra::MA_ActionMT searchAction() {
  ame::agra::MA_ActionMT value;
  value.message_header = header();
  value.object_state = ame::agra::ObjectStateEnum::New;
  value.message_data.action_id.base.uuid =
      ame::AgraMaBridge::deterministicUuid("search-action");
  value.message_data.action_id.base.descriptive_label =
      "search(uav1,sector_a)";
  value.message_data.action_id.version = 1;
  value.message_data.action_type = ame::agra::ActionTypeEnum::FindSearch;
  return value;
}

// Encode-side test doubles for the host's own native->native-JSON
// conversion (ame_py.cpp's `_agra_protobuf_to_native_json`, embedded
// Python this test cannot call). What the wire actually needs to match is
// the *decode* side (agra_native_json_uuid.h, exercised for real by
// GroundingServiceHandler below) -- these only need to produce a payload
// that decodes back to the same message, not byte-identical JSON.

void encodeGroundingHeader(ame::agra::HeaderType& header_value) {
  header_value.system_id.uuid =
      ame::nativeJsonUuidFromNative(header_value.system_id.uuid);
  if (header_value.mission_id.has_value()) {
    header_value.mission_id->base.uuid =
        ame::nativeJsonUuidFromNative(header_value.mission_id->base.uuid);
  }
}

std::string encodeGroundingTaskPayload(ame::agra::MA_TaskMT task_value) {
  encodeGroundingHeader(task_value.message_header);
  task_value.message_data.task_id.base.uuid =
      ame::nativeJsonUuidFromNative(task_value.message_data.task_id.base.uuid);
  return ame::agra_codec::toJson(task_value);
}

std::string encodeGroundingActionPayload(ame::agra::MA_ActionMT action) {
  encodeGroundingHeader(action.message_header);
  action.message_data.action_id.base.uuid = ame::nativeJsonUuidFromNative(
      action.message_data.action_id.base.uuid);
  return ame::agra_codec::toJson(action);
}

grounding_service::TaskGrounding wireTaskGrounding(
    const ame::agra::MA_TaskMT& task_value,
    const std::vector<std::string>& goal_fluents) {
  grounding_service::TaskGrounding wire;
  wire.task_payload = encodeGroundingTaskPayload(task_value);
  wire.goal_fluents = goal_fluents;
  return wire;
}

grounding_service::PlanGrounding wireSearchPlanGrounding(
    const std::string& plan_command_uuid) {
  grounding_service::PlanGrounding wire;
  wire.mission_plan_command_uuid =
      ame::nativeJsonUuidFromNative(plan_command_uuid);

  grounding_service::ActionGrounding action_wire;
  action_wire.action_signature = "search(uav1,sector_a)";
  action_wire.action_payload = encodeGroundingActionPayload(searchAction());
  action_wire.requires_kinematics = false;
  wire.grounding.push_back(std::move(action_wire));
  return wire;
}

ame::agra_c2_provided::MA_MissionPlanCommand_Service_Information planCommand(
    const std::string& command_uuid, const ame::agra::TaskID_Type& task_id) {
  ame::agra::MA_MissionPlanCommandMT command;
  command.message_header = header();
  command.message_data.command_id.uuid = command_uuid;
  command.message_data.command_id.descriptive_label = "stub-plan-command";
  command.message_data.command_state = ame::agra::CommandStateEnum::New;
  command.message_data.inputs.planning_process_id.uuid =
      ame::AgraMaBridge::deterministicUuid("ame-planning-process");
  command.message_data.inputs.plan_initiation =
      ame::agra::PlanInitiationSourceEnum::PlannerServiceOperatorInitiated;
  command.message_data.inputs.planning_data_source =
      ame::agra::PlanningDataSourceEnum::Live;
  command.message_data.inputs.results_in_mission_plan = true;
  ame::agra::MA_RequirementAllocationCommandType requirements;
  ame::agra::MA_TaskAllocationType allocation;
  allocation.task_id = task_id;
  requirements.proposed_task.push_back(std::move(allocation));
  command.message_data.inputs.proposed_requirements = std::move(requirements);

  ame::agra_c2_provided::MA_MissionPlanCommand_Service_Information request;
  request.ma_mission_plan_command = std::move(command);
  return request;
}

void completeSearch(ame::AgraMaBridge& bridge) {
  auto commands = bridge.pullCommands();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands.front().operation, "search");
  ame::CommandResult result;
  result.command_id = commands.front().command_id;
  result.status = ame::CommandStatus::SUCCEEDED;
  result.source = "stub-vehicle";
  bridge.pushCommandResult(result);
}

}  // namespace

// Proves the real seam GroundingServiceHandler exists for: grounding
// delivered as wire-shaped structs (TaskGrounding/PlanGrounding, exactly
// what a generated Grounding_Service Port hands a provided-side handler)
// produces the same accepted-plan-and-execute outcome as the existing
// direct bridge.registerTaskGrounding()/supplyPlanGrounding() calls do in
// test_agra_ma_bridge.cpp's AutoApprovePreservesImmediateExecutionPath --
// this is that same scenario, driven through the handler instead.
TEST(AgraMaGroundingComponent, WireGroundingProducesTheSamePlanAndExecutesIt) {
  auto wm = buildDomain();
  auto registry = buildRegistry();
  ame::CurrentAmeBackendAdapter backend(wm, registry);
  ame::AgraMaBridge bridge(backend, bridgeOptions());
  ame::GroundingServiceHandler handler(bridge);

  const auto task_uuid = ame::AgraMaBridge::deterministicUuid("task-wire");
  const auto tasking = task(task_uuid);

  const auto task_id = handler.onGroundingCreateTaskGrounding(
      wireTaskGrounding(tasking, {"(searched sector_a)"}));
  EXPECT_FALSE(task_id.empty());

  const auto plan_command_uuid =
      ame::AgraMaBridge::deterministicUuid("plan-command-wire");
  const auto plan_id = handler.onGroundingCreatePlanGrounding(
      wireSearchPlanGrounding(plan_command_uuid));
  EXPECT_FALSE(plan_id.empty());

  bridge.onCommand(planCommand(plan_command_uuid, tasking.message_data.task_id));

  EXPECT_TRUE(bridge.pullApprovalRequests().empty());
  EXPECT_EQ(backend.readSnapshot().state,
            ame::AutonomyBackendState::WAITING_FOR_RESULTS);
  auto plans = bridge.handleMaMissionplanRead({});
  ASSERT_EQ(plans.size(), 1u);
  EXPECT_TRUE(plans.front().ma_mission_plan.has_value());

  completeSearch(bridge);
  EXPECT_EQ(backend.readSnapshot().state,
            ame::AutonomyBackendState::COMPLETE);
}

TEST(AgraMaGroundingComponent,
     RejectsAPayloadThatDoesNotDecodeAsNativeJson) {
  auto wm = buildDomain();
  auto registry = buildRegistry();
  ame::CurrentAmeBackendAdapter backend(wm, registry);
  ame::AgraMaBridge bridge(backend, bridgeOptions());
  ame::GroundingServiceHandler handler(bridge);

  grounding_service::TaskGrounding malformed;
  malformed.task_payload = "not json at all";
  EXPECT_THROW(handler.onGroundingCreateTaskGrounding(malformed),
               std::exception);
}
