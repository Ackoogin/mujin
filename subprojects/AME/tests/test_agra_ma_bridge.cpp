#include <gtest/gtest.h>

#include "ame/action_registry.h"
#include "ame/agra_ma_bridge.h"
#include "ame/current_ame_backend_adapter.h"
#include "ame/world_model.h"

#include <stdexcept>
#include <string>
#include <vector>

namespace {

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

ame::AgraMaBridgeOptions bridgeOptions(bool require_approval) {
  ame::AgraMaBridgeOptions options;
  options.system_uuid =
      ame::AgraMaBridge::deterministicUuid("ame.current_stack");
  options.approval_authority_system_uuid =
      ame::AgraMaBridge::deterministicUuid("stub-c2");
  options.message_mode = ame::agra::MessageModeEnum::Simulation;
  options.approval_timeout_seconds = 60;
  options.backend_policy.max_replans = 3;
  options.backend_policy.require_plan_approval = require_approval;
  return options;
}

ame::agra::MA_TaskMT task(const std::string& task_uuid) {
  ame::agra::MA_TaskMT value;
  value.message_header = header();
  ame::agra::MissionID_Type mission_id;
  mission_id.base.uuid =
      ame::AgraMaBridge::deterministicUuid("mission-1");
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

ame::AgraPlanElementGrounding searchGrounding() {
  ame::AgraPlanElementGrounding value;
  value.action_signature = "search(uav1,sector_a)";
  value.action.message_header = header();
  value.action.object_state = ame::agra::ObjectStateEnum::New;
  value.action.message_data.action_id.base.uuid =
      ame::AgraMaBridge::deterministicUuid("search-action");
  value.action.message_data.action_id.base.descriptive_label =
      "search(uav1,sector_a)";
  value.action.message_data.action_id.version = 1;
  value.action.message_data.action_type =
      ame::agra::ActionTypeEnum::FindSearch;
  return value;
}

ame::agra_c2_provided::MA_MissionPlanCommand_Service_Information
planCommand(
    const std::string& command_uuid,
    const ame::agra::TaskID_Type& task_id) {
  ame::agra::MA_MissionPlanCommandMT command;
  command.message_header = header();
  command.message_data.command_id.uuid = command_uuid;
  command.message_data.command_id.descriptive_label =
      "stub-plan-command";
  command.message_data.command_state =
      ame::agra::CommandStateEnum::New;
  command.message_data.inputs.planning_process_id.uuid =
      ame::AgraMaBridge::deterministicUuid("ame-planning-process");
  command.message_data.inputs.plan_initiation =
      ame::agra::PlanInitiationSourceEnum::
          PlannerServiceOperatorInitiated;
  command.message_data.inputs.planning_data_source =
      ame::agra::PlanningDataSourceEnum::Live;
  command.message_data.inputs.results_in_mission_plan = true;
  ame::agra::MA_RequirementAllocationCommandType requirements;
  ame::agra::MA_TaskAllocationType allocation;
  allocation.task_id = task_id;
  requirements.proposed_task.push_back(std::move(allocation));
  command.message_data.inputs.proposed_requirements =
      std::move(requirements);

  ame::agra_c2_provided::
      MA_MissionPlanCommand_Service_Information request;
  request.ma_mission_plan_command = std::move(command);
  return request;
}

ame::agra_c2_provided::
    MA_MissionPlanActivationCommand_Service_Information
activationCommand(
    const std::string& command_uuid,
    const ame::agra::MissionPlanID_Type& plan_id) {
  ame::agra::MA_MissionPlanActivationCommandMT command;
  command.message_header = header();
  command.message_data.command_id.uuid = command_uuid;
  command.message_data.command_id.descriptive_label =
      "stub-activation-command";
  command.message_data.command_state =
      ame::agra::CommandStateEnum::New;
  ame::agra::MA_MissionPlanActivationCommandType activation;
  activation.mission_plan_id = plan_id;
  ame::agra::MissionPlanActivationType details;
  details.activation_command =
      ame::agra::PlanActivationCommandEnum::Activate;
  activation.activation_details.by_mission_plan = details;
  command.message_data.command.push_back(std::move(activation));

  ame::agra_c2_provided::
      MA_MissionPlanActivationCommand_Service_Information request;
  request.ma_mission_plan_activation_command = std::move(command);
  return request;
}

ame::agra_c2_consumed::MA_ApprovalRequestStatus_Service_Information
approvalStatus(
    const ame::agra::MA_ApprovalRequestMT& request,
    ame::agra::ApprovalStatusEnum status,
    const std::string& reason = {}) {
  ame::agra::MA_ApprovalRequestStatusMT result;
  result.message_header = header();
  result.message_data.request_id = request.message_data.request_id;
  result.message_data.request_processing_state =
      ame::agra::RequestProcessingStateEnum::Completed;
  result.message_data.approval_request_processing_state = status;
  if (!reason.empty()) {
    ame::agra::CannotComplyType cannot_comply;
    cannot_comply.reason = ame::agra::CannotComplyEnum::Other;
    cannot_comply.description = reason;
    result.message_data.approval_request_processing_state_reason =
        std::move(cannot_comply);
  }
  ame::agra_c2_consumed::
      MA_ApprovalRequestStatus_Service_Information information;
  information.ma_approval_request_status = std::move(result);
  return information;
}

ame::agra_c2_provided::MA_TaskCommand_Service_Information taskCommand(
    const std::string& command_uuid,
    const ame::agra::TaskID_Type& task_id) {
  ame::agra::MA_TaskCommandMT message;
  message.security_information.classification =
      ame::agra::ClassificationEnum::U;
  ame::agra::OwnerProducerChoiceType owner;
  owner.government_identifier = ame::agra::OwnerProducerEnum::Usa;
  message.security_information.owner_producer.push_back(owner);
  message.message_header = header();
  ame::agra::MA_TaskCommandType command;
  command.capability.base.command_id.uuid = command_uuid;
  command.capability.base.command_id.descriptive_label =
      "stub-task-command";
  command.capability.base.command_state =
      ame::agra::CommandStateEnum::New;
  command.capability.capability_id.uuid =
      ame::AgraMaBridge::deterministicUuid("task-capability");
  command.capability.task_id = task_id;
  message.message_data.command.push_back(std::move(command));
  ame::agra_c2_provided::MA_TaskCommand_Service_Information
      information;
  information.ma_task_command = std::move(message);
  return information;
}

ame::agra::MissionPlanID_Type currentMissionPlan(
    ame::AgraMaBridge& bridge) {
  auto plans = bridge.handleMaMissionplanRead({});
  EXPECT_EQ(plans.size(), 1u);
  EXPECT_TRUE(plans.front().ma_mission_plan.has_value());
  return plans.front()
      .ma_mission_plan->message_data.mission_plan_id;
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

TEST(AgraMaBridge, MissingTaskMissionIdentityFailsClosedWithElementNamed) {
  auto wm = buildDomain();
  auto registry = buildRegistry();
  ame::CurrentAmeBackendAdapter backend(wm, registry);
  ame::AgraMaBridge bridge(backend, bridgeOptions(true));

  auto tasking = task(
      ame::AgraMaBridge::deterministicUuid("task-missing-mission"));
  tasking.message_header.mission_id.reset();

  try {
    bridge.registerTaskGrounding(tasking, {"(searched sector_a)"});
    FAIL() << "Expected missing mission identity to be refused";
  } catch (const std::invalid_argument& error) {
    EXPECT_EQ(
        std::string(error.what()),
        "MA_Task.message_header.mission_id is missing");
  }
}

TEST(AgraMaBridge, ApprovalPlanActivationAndExecutionRoundTrip) {
  auto wm = buildDomain();
  auto registry = buildRegistry();
  ame::CurrentAmeBackendAdapter backend(wm, registry);
  ame::AgraMaBridge bridge(backend, bridgeOptions(true));

  const auto task_uuid =
      ame::AgraMaBridge::deterministicUuid("task-approval");
  const auto tasking = task(task_uuid);
  bridge.registerTaskGrounding(
      tasking, {"(searched sector_a)"});
  const auto task_command_uuid =
      ame::AgraMaBridge::deterministicUuid("task-command-approval");
  bridge.onCommand(
      taskCommand(task_command_uuid, tasking.message_data.task_id));
  const auto task_statuses = bridge.handleMaTaskcommandstatusRead({});
  ASSERT_EQ(task_statuses.size(), 1u);
  ASSERT_TRUE(task_statuses.front().ma_task_command_status.has_value());
  EXPECT_EQ(
      task_statuses.front()
          .ma_task_command_status->message_data.base.command_id.uuid,
      task_command_uuid);
  EXPECT_EQ(
      task_statuses.front()
          .ma_task_command_status->message_data.base
          .command_processing_state,
      ame::agra::CommandProcessingStateEnum::Accepted);
  EXPECT_EQ(
      task_statuses.front()
          .ma_task_command_status->security_information.classification,
      ame::agra::ClassificationEnum::U);
  ASSERT_EQ(
      task_statuses.front()
          .ma_task_command_status->security_information.owner_producer
          .size(),
      1u);
  EXPECT_EQ(
      task_statuses.front()
          .ma_task_command_status->security_information.owner_producer
          .front()
          .government_identifier,
      ame::agra::OwnerProducerEnum::Usa);
  const auto plan_command_uuid =
      ame::AgraMaBridge::deterministicUuid("plan-command-approval");
  bridge.supplyPlanGrounding(plan_command_uuid, {searchGrounding()});

  bridge.onCommand(
      planCommand(plan_command_uuid, tasking.message_data.task_id));
  EXPECT_EQ(backend.readSnapshot().state,
            ame::AutonomyBackendState::PENDING_APPROVAL);

  const auto statuses =
      bridge.handleMaMissionplancommandstatusRead({});
  ASSERT_EQ(statuses.size(), 2u);
  for (const auto& status : statuses) {
    ASSERT_TRUE(status.ma_mission_plan_command_status.has_value());
    EXPECT_EQ(
        status.ma_mission_plan_command_status->message_data.command_id.uuid,
        plan_command_uuid);
  }
  const auto plan_id = currentMissionPlan(bridge);
  ASSERT_FALSE(plan_id.base.descriptive_label.empty());
  EXPECT_EQ(
      statuses.back()
          .ma_mission_plan_command_status->message_data
          .resulting_plan_identifier->mission_plan_id.front().base.uuid,
      plan_id.base.uuid);
  EXPECT_EQ(
      statuses.back()
          .ma_mission_plan_command_status->message_data.planning_status
          .command_status,
      ame::agra::ProcessingStatusEnum::Completed);

  auto approvals = bridge.pullApprovalRequests();
  ASSERT_EQ(approvals.size(), 1u);
  ASSERT_TRUE(approvals.front().ma_approval_request.has_value());
  ASSERT_TRUE(
      approvals.front().ma_approval_request->message_header.mission_id
          .has_value());
  EXPECT_EQ(
      approvals.front()
          .ma_approval_request->message_header.mission_id
          ->base.descriptive_label,
      "mission-1");
  EXPECT_EQ(approvals.front()
                .ma_approval_request->message_data.request_id
                .descriptive_label,
            plan_id.base.descriptive_label);
  ASSERT_TRUE(
      approvals.front()
          .ma_approval_request->message_data.approval_references
          .approval_item.plan_approval.has_value());
  ASSERT_TRUE(
      approvals.front()
          .ma_approval_request->message_data.approval_references
          .approval_item.plan_approval->mission_plan_id.has_value());
  EXPECT_EQ(
      approvals.front()
          .ma_approval_request->message_data.approval_references
          .approval_item.plan_approval->mission_plan_id->base.uuid,
      plan_id.base.uuid);
  bridge.ingestApprovalStatus(
      approvalStatus(approvals.front().ma_approval_request.value(),
                     ame::agra::ApprovalStatusEnum::Approved));
  EXPECT_EQ(backend.readSnapshot().state,
            ame::AutonomyBackendState::EXECUTING);

  const auto activation_uuid =
      ame::AgraMaBridge::deterministicUuid("activation-approval");
  bridge.onCommand(activationCommand(activation_uuid, plan_id));
  const auto activation_statuses =
      bridge.handleMaMissionplanactivationcommandstatusRead({});
  ASSERT_EQ(activation_statuses.size(), 1u);
  EXPECT_EQ(
      activation_statuses.front()
          .ma_mission_plan_activation_command_status->message_data
          .command_id.uuid,
      activation_uuid);
  EXPECT_EQ(
      activation_statuses.front()
          .ma_mission_plan_activation_command_status->message_data
          .activation_status.command_status,
      ame::agra::ProcessingStatusEnum::Completed);

  completeSearch(bridge);
  EXPECT_EQ(backend.readSnapshot().state,
            ame::AutonomyBackendState::COMPLETE);
  const auto execution_statuses =
      bridge.handleMaMissionplanexecutionstatusRead({});
  ASSERT_FALSE(execution_statuses.empty());
  const auto& final_status =
      execution_statuses.back()
          .ma_mission_plan_execution_status->message_data
          .plan_execution_status.front();
  EXPECT_EQ(final_status.mission_plan_id.base.uuid,
            plan_id.base.uuid);
  EXPECT_EQ(final_status.plan_execution_state,
            ame::agra::PlanExecutionStateEnum::Complete);
}

TEST(AgraMaBridge, AutoApprovePreservesImmediateExecutionPath) {
  auto wm = buildDomain();
  auto registry = buildRegistry();
  ame::CurrentAmeBackendAdapter backend(wm, registry);
  ame::AgraMaBridge bridge(backend, bridgeOptions(false));

  const auto task_uuid =
      ame::AgraMaBridge::deterministicUuid("task-auto");
  const auto tasking = task(task_uuid);
  bridge.registerTaskGrounding(
      tasking, {"(searched sector_a)"});
  const auto plan_command_uuid =
      ame::AgraMaBridge::deterministicUuid("plan-command-auto");
  bridge.supplyPlanGrounding(plan_command_uuid, {searchGrounding()});
  bridge.onCommand(
      planCommand(plan_command_uuid, tasking.message_data.task_id));

  EXPECT_TRUE(bridge.pullApprovalRequests().empty());
  EXPECT_EQ(backend.readSnapshot().state,
            ame::AutonomyBackendState::WAITING_FOR_RESULTS);
  const auto plan_id = currentMissionPlan(bridge);
  const auto activation_uuid =
      ame::AgraMaBridge::deterministicUuid("activation-auto");
  bridge.onCommand(activationCommand(activation_uuid, plan_id));
  completeSearch(bridge);
  EXPECT_EQ(backend.readSnapshot().state,
            ame::AutonomyBackendState::COMPLETE);
}

TEST(AgraMaBridge, AdvertisesOnlyImplementedPlanningInterfaces) {
  auto wm = buildDomain();
  auto registry = buildRegistry();
  ame::CurrentAmeBackendAdapter backend(wm, registry);
  ame::AgraMaBridge bridge(backend, bridgeOptions(false));

  const auto functions = bridge.handleMaPlanningfunctionRead({});
  ASSERT_EQ(functions.size(), 1u);
  ASSERT_TRUE(functions.front().ma_planning_function.has_value());
  const auto& interfaces =
      functions.front()
          .ma_planning_function->message_data.planning_interfaces;
  ASSERT_EQ(interfaces.size(), 1u);
  EXPECT_EQ(
      interfaces.front().plan_interface,
      (std::vector<ame::agra::PlanningInterfaceEnum>{
          ame::agra::PlanningInterfaceEnum::PlanCommand,
          ame::agra::PlanningInterfaceEnum::RequirementManagement,
          ame::agra::PlanningInterfaceEnum::PlanValidation,
      }));
}

TEST(AgraMaBridge, PublishesExternallyGroundedP3RouteAndActionPlans) {
  auto wm = buildDomain();
  auto registry = buildRegistry();
  ame::CurrentAmeBackendAdapter backend(wm, registry);
  ame::AgraMaBridge bridge(backend, bridgeOptions(false));

  const auto task_uuid =
      ame::AgraMaBridge::deterministicUuid("task-grounded-route");
  const auto tasking = task(task_uuid);
  bridge.registerTaskGrounding(
      tasking, {"(searched sector_a)"});
  const auto plan_command_uuid =
      ame::AgraMaBridge::deterministicUuid(
          "plan-command-grounded-route");
  auto grounding = searchGrounding();
  grounding.requires_kinematics = true;
  grounding.route_plan_id = "automtk-route-42";
  grounding.waypoints = {
      {51.0, -1.0, 100.0},
      {51.1, -1.1, 120.0},
  };
  grounding.command_parameters = {
      {"sensor_mode", "wide-area-search"},
  };
  bridge.supplyPlanGrounding(plan_command_uuid, {grounding});

  bridge.onCommand(
      planCommand(plan_command_uuid, tasking.message_data.task_id));

  const auto routes = bridge.handleMaRouteplanRead({});
  ASSERT_EQ(routes.size(), 1u);
  ASSERT_TRUE(routes.front().ma_route_plan.has_value());
  const auto& route = routes.front().ma_route_plan.value();
  EXPECT_EQ(route.message_data.route_plan_id.base.descriptive_label,
            "automtk-route-42");
  ASSERT_EQ(route.message_data.plan.route.path.size(), 1u);
  ASSERT_EQ(
      route.message_data.plan.route.path.front().path_segment.size(),
      2u);
  const auto& segments =
      route.message_data.plan.route.path.front().path_segment;
  ASSERT_TRUE(
      segments[0].end_point.way_point->point_choice.point2_d.has_value());
  ASSERT_TRUE(
      segments[1].end_point.way_point->point_choice.point2_d.has_value());
  constexpr double kDegreesToRadians =
      3.14159265358979323846 / 180.0;
  const auto& first_point =
      segments[0].end_point.way_point->point_choice.point2_d.value();
  const auto& second_point =
      segments[1].end_point.way_point->point_choice.point2_d.value();
  ASSERT_TRUE(first_point.altitude.has_value());
  ASSERT_TRUE(second_point.altitude.has_value());
  EXPECT_DOUBLE_EQ(first_point.latitude, 51.0 * kDegreesToRadians);
  EXPECT_DOUBLE_EQ(first_point.longitude, -1.0 * kDegreesToRadians);
  EXPECT_DOUBLE_EQ(first_point.altitude.value(), 100.0);
  EXPECT_DOUBLE_EQ(second_point.latitude, 51.1 * kDegreesToRadians);
  EXPECT_DOUBLE_EQ(second_point.longitude, -1.1 * kDegreesToRadians);
  EXPECT_DOUBLE_EQ(second_point.altitude.value(), 120.0);

  const auto action_plans = bridge.handleMaActionplanRead({});
  ASSERT_EQ(action_plans.size(), 1u);
  ASSERT_TRUE(action_plans.front().ma_action_plan.has_value());
  EXPECT_EQ(
      action_plans.front()
          .ma_action_plan->message_data.plan.allocated_action.size(),
      1u);

  const auto plans = bridge.handleMaMissionplanRead({});
  ASSERT_EQ(plans.size(), 1u);
  ASSERT_TRUE(plans.front().ma_mission_plan.has_value());
  ASSERT_TRUE(
      plans.front().ma_mission_plan->message_data.sub_plans.has_value());
  EXPECT_EQ(
      plans.front()
          .ma_mission_plan->message_data.sub_plans->route_plan_id.size(),
      1u);
  EXPECT_EQ(
      plans.front()
          .ma_mission_plan->message_data.sub_plans->action_plan_id.size(),
      1u);
  const auto commands = bridge.pullCommands();
  ASSERT_EQ(commands.size(), 1u);
  const auto sensor_mode =
      commands.front().request_fields.find("sensor_mode");
  ASSERT_NE(sensor_mode, commands.front().request_fields.end());
  EXPECT_EQ(sensor_mode->second, "wide-area-search");
}

TEST(AgraMaBridge, RefusesActivationAndStaleApprovalWhilePending) {
  auto wm = buildDomain();
  auto registry = buildRegistry();
  ame::CurrentAmeBackendAdapter backend(wm, registry);
  ame::AgraMaBridge bridge(backend, bridgeOptions(true));

  const auto task_uuid =
      ame::AgraMaBridge::deterministicUuid("task-negative");
  const auto tasking = task(task_uuid);
  bridge.registerTaskGrounding(
      tasking, {"(searched sector_a)"});
  const auto plan_command_uuid =
      ame::AgraMaBridge::deterministicUuid("plan-command-negative");
  bridge.supplyPlanGrounding(plan_command_uuid, {searchGrounding()});
  bridge.onCommand(
      planCommand(plan_command_uuid, tasking.message_data.task_id));
  const auto plan_id = currentMissionPlan(bridge);

  try {
    bridge.onCommand(
        activationCommand(
            ame::AgraMaBridge::deterministicUuid("activation-too-early"),
            plan_id));
    FAIL() << "activation should fail while approval is pending";
  } catch (const std::logic_error& error) {
    EXPECT_NE(std::string(error.what()).find(
                  plan_id.base.descriptive_label),
              std::string::npos);
  }
  const auto refused_activation =
      bridge.handleMaMissionplanactivationcommandstatusRead({});
  ASSERT_EQ(refused_activation.size(), 1u);
  ASSERT_TRUE(
      refused_activation.front()
          .ma_mission_plan_activation_command_status.has_value());
  EXPECT_EQ(
      refused_activation.front()
          .ma_mission_plan_activation_command_status->message_data
          .activation_status.command_processing_state,
      ame::agra::CommandProcessingStateEnum::Rejected);
  EXPECT_TRUE(backend.pullCommands().empty());

  const std::string stale_plan_id = "stale-plan-id";
  try {
    backend.approvePlan(stale_plan_id);
    FAIL() << "stale plan approval should fail";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find(stale_plan_id),
              std::string::npos);
  }

  auto approvals = bridge.pullApprovalRequests();
  ASSERT_EQ(approvals.size(), 1u);
  ASSERT_TRUE(approvals.front().ma_approval_request.has_value());
  auto stale_status = approvalStatus(
      approvals.front().ma_approval_request.value(),
      ame::agra::ApprovalStatusEnum::Approved);
  stale_status.ma_approval_request_status->message_data.request_id
      .descriptive_label =
      stale_plan_id;
  try {
    bridge.ingestApprovalStatus(stale_status);
    FAIL() << "stale correlated approval should fail";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find(stale_plan_id),
              std::string::npos);
  }
  EXPECT_EQ(backend.readSnapshot().state,
            ame::AutonomyBackendState::PENDING_APPROVAL);
}

TEST(AgraMaBridge, MissingKinematicGroundingFailsClosedAndLoud) {
  auto wm = buildDomain();
  auto registry = buildRegistry();
  ame::CurrentAmeBackendAdapter backend(wm, registry);
  ame::AgraMaBridge bridge(backend, bridgeOptions(true));

  const auto task_uuid =
      ame::AgraMaBridge::deterministicUuid("task-kinematic");
  const auto tasking = task(task_uuid);
  bridge.registerTaskGrounding(
      tasking, {"(searched sector_a)"});
  const auto plan_command_uuid =
      ame::AgraMaBridge::deterministicUuid("plan-command-kinematic");
  auto grounding = searchGrounding();
  grounding.requires_kinematics = true;
  grounding.route_plan_id = "route-from-automtk";
  bridge.supplyPlanGrounding(plan_command_uuid, {grounding});

  try {
    bridge.onCommand(
        planCommand(plan_command_uuid, tasking.message_data.task_id));
    FAIL() << "missing route coordinates must fail closed";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("fewer than two waypoints"),
              std::string::npos);
    EXPECT_NE(std::string(error.what()).find(
                  "search(uav1,sector_a)"),
              std::string::npos);
  }
  const auto statuses =
      bridge.handleMaMissionplancommandstatusRead({});
  ASSERT_EQ(statuses.size(), 2u);
  ASSERT_TRUE(
      statuses.back().ma_mission_plan_command_status.has_value());
  EXPECT_EQ(
      statuses.back()
          .ma_mission_plan_command_status->message_data.planning_status
          .command_processing_state,
      ame::agra::CommandProcessingStateEnum::Rejected);
  EXPECT_EQ(backend.readSnapshot().state,
            ame::AutonomyBackendState::STOPPED);
  EXPECT_TRUE(backend.pullCommands().empty());
}

TEST(AutonomyBackendApproval, RejectReturnsToPlanningAndStalesPlanId) {
  auto wm = buildDomain();
  auto registry = buildRegistry();
  ame::CurrentAmeBackendAdapter backend(wm, registry);
  ame::SessionRequest request;
  request.session_id = "reject-plan";
  request.intent.goal_fluents = {"(searched sector_a)"};
  request.policy.require_plan_approval = true;
  backend.start(request);
  backend.step();

  auto decisions = backend.pullDecisionRecords();
  ASSERT_EQ(decisions.size(), 1u);
  const auto rejected_plan_id = decisions.front().plan_id;
  ASSERT_FALSE(rejected_plan_id.empty());
  backend.rejectPlan(rejected_plan_id, "operator rejected route");
  EXPECT_EQ(backend.readSnapshot().state,
            ame::AutonomyBackendState::READY);
  EXPECT_EQ(backend.readSnapshot().replan_count, 1u);

  try {
    backend.approvePlan(rejected_plan_id);
    FAIL() << "rejected plan identity must become stale";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find(rejected_plan_id),
              std::string::npos);
  }
  EXPECT_TRUE(backend.pullCommands().empty());
}

TEST(AutonomyBackendApproval, DrainRetainsLateCommandResultAndRunHistory) {
  auto wm = buildDomain();
  auto registry = buildRegistry();
  ame::CurrentAmeBackendAdapter backend(wm, registry);
  ame::SessionRequest request;
  request.session_id = "drain-race";
  request.intent.goal_fluents = {"(searched sector_a)"};
  backend.start(request);
  backend.step();
  auto commands = backend.pullCommands();
  ASSERT_EQ(commands.size(), 1u);
  ASSERT_EQ(backend.readSnapshot().decision_history.size(), 1u);

  backend.requestStop(ame::StopMode::DRAIN);
  ame::CommandResult late_result;
  late_result.command_id = commands.front().command_id;
  late_result.status = ame::CommandStatus::SUCCEEDED;
  late_result.source = "late-stub-result";
  late_result.observed_updates = {
      {"(searched sector_a)",
       true,
       "late-stub-result",
       ame::FactAuthorityLevel::CONFIRMED},
  };
  backend.pushCommandResult(late_result);

  const auto snapshot = backend.readSnapshot();
  EXPECT_EQ(snapshot.state, ame::AutonomyBackendState::STOPPED);
  EXPECT_EQ(snapshot.decision_history.size(), 1u);
  ASSERT_EQ(snapshot.command_result_history.size(), 1u);
  EXPECT_EQ(snapshot.command_result_history.front().command_id,
            commands.front().command_id);
  EXPECT_EQ(snapshot.command_result_history.front().source,
            "late-stub-result");
  EXPECT_TRUE(wm.getFact("(searched sector_a)"));
}
