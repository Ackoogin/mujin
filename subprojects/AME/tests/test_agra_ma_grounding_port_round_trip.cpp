/// \file test_agra_ma_grounding_port_round_trip.cpp
/// \brief Proves GroundingServiceHandler over a real generated Port, not
///        just a direct method call.
///
/// test_agra_ma_grounding_component.cpp calls GroundingServiceHandler's
/// methods directly -- it proves the decode-and-forward logic but never
/// exercises the generated ProvidedService/ConsumedService dispatch
/// machinery itself (the wire-shaped TaskGrounding/PlanGrounding never
/// actually crossed a Port). This file does: a real ProvidedService-hosting
/// component and a real ConsumedService client, wired with local
/// transport, issuing genuine async RPCs -- mirroring the working pattern
/// in test_pcl_generated_component_stream_handle.cpp's
/// LocalJsonConfigRoutesServiceInProcess (the closest existing precedent
/// for this exact ProvidedService/ConsumedService shape; no
/// ProvidedService/ConsumedService-based EntityActions service had a real
/// transport test anywhere in this tree before this file -- see
/// docs/plans/phase3-4-progress-status.md, open item 10).
///
/// "Local" transport is in-process dispatch, not wire serialization -- it
/// does not prove OMS-JSON codec correctness (agra_codec::fromJson/toJson
/// already has its own coverage) or a real shared-memory/LA-CAL crossing.
/// What it proves is that GroundingServiceHandler is wired correctly as a
/// pcl::Component underneath a real generated Port, which the direct-call
/// test cannot.

#include <gtest/gtest.h>

#include "ame/action_registry.h"
#include "ame/agra_ma_bridge.h"
#include "ame/agra_ma_grounding_component.hpp"
#include "ame/agra_native_json_uuid.h"
#include "ame/current_ame_backend_adapter.h"
#include "ame/world_model.h"

#include <pcl/component.hpp>
#include <pcl/executor.hpp>
#include <pcl/pcl_plugin_loader.h>

#include <pyramid_data_model_agra_codec.hpp>

#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <string>
#include <vector>

namespace {

namespace grounding_service =
    pyramid::components::agra_ma_grounding::services::provided;

// Grounding_Service's ProvidedService::bind() refuses to configure unless
// its content type's codec is already registered -- load the plugin once,
// process-wide, exactly like test_ame_autonomy_contract.cpp's
// CodecLoaderEnvironment does for the autonomy_backend service.
class CodecLoaderEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    pcl_plugin_handle_t* handle = nullptr;
    const auto status = pcl_plugin_load_codec(
        AGRA_MA_GROUNDING_JSON_CODEC_PLUGIN_PATH,
        nullptr,
        pcl_codec_registry_default(),
        &handle);
    if (status != PCL_OK) {
      std::fprintf(stderr,
                   "Failed to load agra_ma_grounding JSON codec plugin: %d\n",
                   static_cast<int>(status));
      std::abort();
    }
  }
};

::testing::Environment* const codec_loader_environment =
    ::testing::AddGlobalTestEnvironment(new CodecLoaderEnvironment);

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

/// \brief Hosts GroundingServiceHandler behind a real generated
///        ProvidedService, exactly as agra_ma_composite_process.cpp would
///        need to for the real host cutover -- see this class's
///        configureLocalTransport(), which mirrors
///        test_pcl_generated_component_stream_handle.cpp's
///        RequirementComponent.
class GroundingProviderComponent final : public pcl::Component {
public:
  GroundingProviderComponent(pcl::Executor& executor, ame::AgraMaBridge& bridge)
      : pcl::Component("grounding_provider"),
        handler_(bridge),
        provided_(*this, executor, handler_) {}

  ame::GroundingServiceHandler& handler() { return handler_; }

  pcl_status_t configureLocalTransport() {
    return provided_.configureTransport(R"({"transport":"local"})");
  }

protected:
  pcl_status_t on_configure() override { return provided_.bind(); }

private:
  ame::GroundingServiceHandler handler_;
  grounding_service::ProvidedService provided_;
};

}  // namespace

TEST(AgraMaGroundingPortRoundTrip,
     TaskAndPlanGroundingCrossARealPortAndProduceAWorkingPlan) {
  auto wm = buildDomain();
  auto registry = buildRegistry();
  ame::CurrentAmeBackendAdapter backend(wm, registry);
  ame::AgraMaBridge bridge(backend, bridgeOptions());

  pcl::Executor executor;
  GroundingProviderComponent server{executor, bridge};
  pcl::Component client{"grounding_consumer"};
  grounding_service::ConsumedService consumed{client, executor};

  ASSERT_EQ(server.configure(), PCL_OK);
  ASSERT_EQ(server.activate(), PCL_OK);
  ASSERT_EQ(executor.add(server), PCL_OK);
  ASSERT_EQ(server.configureLocalTransport(), PCL_OK);
  ASSERT_EQ(consumed.configureTransport(R"({"transport":"local"})"), PCL_OK);

  const auto task_uuid = ame::AgraMaBridge::deterministicUuid("task-port");
  const auto tasking = task(task_uuid);

  auto task_future = consumed.groundingCreateTaskGroundingAsync(
      wireTaskGrounding(tasking, {"(searched sector_a)"}));
  ASSERT_EQ(task_future.wait_for(std::chrono::milliseconds(0)),
            std::future_status::ready);
  const auto task_result = task_future.get();
  EXPECT_TRUE(task_result.ok());
  EXPECT_FALSE(task_result.value.empty());

  const auto plan_command_uuid =
      ame::AgraMaBridge::deterministicUuid("plan-command-port");
  auto plan_future = consumed.groundingCreatePlanGroundingAsync(
      wireSearchPlanGrounding(plan_command_uuid));
  ASSERT_EQ(plan_future.wait_for(std::chrono::milliseconds(0)),
            std::future_status::ready);
  const auto plan_result = plan_future.get();
  EXPECT_TRUE(plan_result.ok());
  EXPECT_FALSE(plan_result.value.empty());

  // The grounding really landed in the real AgraMaBridge, not just in the
  // handler's own state: driving a real MA_MissionPlanCommand through it
  // produces a plan and executes, exactly like the direct-call test and
  // test_agra_ma_bridge.cpp's AutoApprovePreservesImmediateExecutionPath.
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
