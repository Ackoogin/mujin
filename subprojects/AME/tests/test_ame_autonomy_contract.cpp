#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "ame/action_registry.h"
#include "ame/pyramid_autonomy_bridge.h"
#include "ame/world_model.h"

#include "pyramid_data_model_autonomy_codec.hpp"
#include "pyramid_data_model_common_codec.hpp"
#include "pyramid_services_autonomy_backend_provided.hpp"

#include <cstdlib>
#include <string>
#include <vector>

namespace {

namespace autonomy_codec = pyramid::domain_model::autonomy;
namespace common = pyramid::domain_model::common;
namespace common_codec = pyramid::domain_model::common;
namespace model = pyramid::domain_model;
namespace provided = pyramid::components::autonomy_backend::services::provided;

ame::WorldModel buildDomain() {
  ame::WorldModel wm;
  auto& ts = wm.typeSystem();
  ts.addType("object");
  ts.addType("location", "object");
  ts.addType("sector", "location");
  ts.addType("robot", "object");

  wm.addObject("uav1", "robot");
  wm.addObject("base", "location");
  wm.addObject("sector_a", "sector");

  wm.registerPredicate("at", {"robot", "location"});
  wm.registerPredicate("searched", {"sector"});

  wm.registerAction("move",
                    {"?r", "?from", "?to"},
                    {"robot", "location", "location"},
                    {"(at ?r ?from)"},
                    {"(at ?r ?to)"},
                    {"(at ?r ?from)"});
  wm.registerAction("search",
                    {"?r", "?s"},
                    {"robot", "sector"},
                    {"(at ?r ?s)"},
                    {"(searched ?s)"},
                    {});

  return wm;
}

ame::ActionRegistry buildRegistry() {
  ame::ActionRegistry registry;
  registry.registerActionSubTree(
      "move",
      "<InvokeService service_name=\"mobility\" operation=\"move\" "
      "param_names=\"?robot;?from;?to\" "
      "param_values=\"{param0};{param1};{param2}\" timeout_ms=\"0\"/>");
  registry.registerActionSubTree(
      "search",
      "<InvokeService service_name=\"imaging\" operation=\"search\" "
      "param_names=\"?robot;?sector\" "
      "param_values=\"{param0};{param1}\" timeout_ms=\"0\"/>");
  return registry;
}

std::string dispatchJson(provided::ServiceHandler& handler,
                         provided::ServiceChannel channel,
                         const std::string& payload) {
  void* response_buf = nullptr;
  size_t response_size = 0;
  provided::dispatch(handler,
                     channel,
                     payload.data(),
                     payload.size(),
                     provided::kJsonContentType,
                     &response_buf,
                     &response_size);

  std::string response;
  if (response_buf != nullptr && response_size > 0) {
    response.assign(static_cast<const char*>(response_buf), response_size);
  }
  std::free(response_buf);
  return response;
}

template <typename MessageT, typename DecodeFn>
std::vector<MessageT> decodeJsonArray(const std::string& payload,
                                      DecodeFn decode) {
  std::vector<MessageT> result;
  const auto values = nlohmann::json::parse(payload);
  for (const auto& value : values) {
    result.push_back(decode(value.dump()));
  }
  return result;
}

model::Query queryAll() {
  return {};
}

model::Query queryId(const std::string& id) {
  model::Query query;
  query.id.push_back(id);
  return query;
}

model::StateUpdate initialAtBaseState() {
  model::StateUpdate state;
  model::WorldFactUpdate initial_location;
  initial_location.key = "(at uav1 base)";
  initial_location.value = true;
  initial_location.source = "test-client";
  initial_location.authority = model::FactAuthorityLevel::Confirmed;
  state.fact_update.push_back(initial_location);
  return state;
}

model::PlanningRequirement planningRequirement() {
  model::PlanningRequirement requirement;
  requirement.policy.max_replans = 3;
  requirement.policy.enable_replanning = true;

  model::PlanningGoal goal;
  goal.expression = "(searched sector_a)";
  requirement.goal.push_back(goal);

  model::AgentState agent;
  agent.agent_id = "uav1";
  agent.agent_type = "robot";
  agent.available = true;
  requirement.available_agents.push_back(agent);
  return requirement;
}

model::ExecutionRequirement executionRequirement(
    const std::string& plan_id,
    const std::string& planning_requirement_id) {
  model::ExecutionRequirement requirement;
  requirement.plan_id = plan_id;
  requirement.planning_requirement_id = planning_requirement_id;
  requirement.policy.max_replans = 3;
  requirement.policy.enable_replanning = true;
  requirement.policy.max_concurrent_placements = 2;
  return requirement;
}

ame::ExecutionBinding customMoveBinding() {
  ame::ExecutionBinding binding;
  binding.action_name = "move";
  binding.target_component = "host_mobility";
  binding.target_service = "Host_Move_Service";
  binding.target_type = "host.data_model.MoveRequirement";
  return binding;
}

ame::ExecutionBinding customSearchBinding() {
  ame::ExecutionBinding binding;
  binding.action_name = "search";
  binding.target_component = "host_sensor";
  binding.target_service = "Host_Search_Service";
  binding.target_type = "host.data_model.SearchRequirement";
  return binding;
}

}  // namespace

TEST(AmeAutonomyContract, PlansThroughGeneratedJsonServiceContract) {
  auto wm = buildDomain();
  auto registry = buildRegistry();
  ame::PyramidAutonomyBridge bridge(wm, registry);

  model::StateUpdate state;
  model::WorldFactUpdate initial_location;
  initial_location.key = "(at uav1 base)";
  initial_location.value = true;
  initial_location.source = "test-client";
  initial_location.authority = model::FactAuthorityLevel::Confirmed;
  state.fact_update.push_back(initial_location);

  const auto state_id_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::StateCreateState,
                   autonomy_codec::toJson(state));
  const auto state_id =
      nlohmann::json::parse(state_id_payload).get<std::string>();
  ASSERT_FALSE(state_id.empty());
  EXPECT_TRUE(wm.getFact("(at uav1 base)"));

  const auto requirement = planningRequirement();

  const auto requirement_id_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::PlanningRequirementCreatePlanningRequirement,
                   autonomy_codec::toJson(requirement));
  const auto requirement_id =
      nlohmann::json::parse(requirement_id_payload).get<std::string>();
  ASSERT_FALSE(requirement_id.empty());

  const auto read_requirement_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::PlanningRequirementReadPlanningRequirement,
                   common_codec::toJson(queryId(requirement_id)));
  const auto requirements =
      decodeJsonArray<model::PlanningRequirement>(
          read_requirement_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::PlanningRequirement*>(nullptr));
          });
  ASSERT_EQ(requirements.size(), 1u);
  EXPECT_EQ(requirements.front().base.id, requirement_id);
  EXPECT_EQ(requirements.front().status.status, common::Progress::Completed);

  const auto read_plan_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::PlanReadPlan,
                   common_codec::toJson(queryAll()));
  const auto plans =
      decodeJsonArray<model::Plan>(
          read_plan_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::Plan*>(nullptr));
          });
  ASSERT_EQ(plans.size(), 1u);
  EXPECT_EQ(plans.front().planning_requirement_id, requirement_id);
  EXPECT_TRUE(plans.front().plan_success);
  EXPECT_GE(plans.front().step.size(), 2u);
  EXPECT_FALSE(plans.front().compiled_bt_xml.empty());

  const auto second_plan_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::PlanReadPlan,
                   common_codec::toJson(queryAll()));
  const auto second_plans =
      decodeJsonArray<model::Plan>(
          second_plan_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::Plan*>(nullptr));
          });
  ASSERT_EQ(second_plans.size(), 1u);
  EXPECT_EQ(second_plans.front().id, plans.front().id);

  model::Plan imported_plan;
  imported_plan.id = "external-plan-1";
  imported_plan.backend_id = "external.coordinator";
  imported_plan.plan_success = true;
  imported_plan.compiled_bt_xml = "<root/>";
  model::PlanStep imported_step;
  imported_step.id = "external-plan-1/step/1";
  imported_step.sequence_number = 1;
  imported_step.action_name = "search";
  imported_step.signature = "search(uav1,sector_a)";
  imported_plan.step.push_back(imported_step);

  const auto imported_plan_id =
      nlohmann::json::parse(
          dispatchJson(bridge,
                       provided::ServiceChannel::PlanCreatePlan,
                       autonomy_codec::toJson(imported_plan)))
          .get<std::string>();
  EXPECT_EQ(imported_plan_id, imported_plan.id);

  imported_plan.backend_id = "external.coordinator.updated";
  const auto update_plan_ack =
      common_codec::fromJson(
          dispatchJson(bridge,
                       provided::ServiceChannel::PlanUpdatePlan,
                       autonomy_codec::toJson(imported_plan)),
          static_cast<model::Ack*>(nullptr));
  EXPECT_TRUE(update_plan_ack.success);

  const auto read_imported_plan_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::PlanReadPlan,
                   common_codec::toJson(queryId(imported_plan_id)));
  const auto imported_plans =
      decodeJsonArray<model::Plan>(
          read_imported_plan_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::Plan*>(nullptr));
          });
  ASSERT_EQ(imported_plans.size(), 1u);
  EXPECT_EQ(imported_plans.front().backend_id,
            "external.coordinator.updated");

  const auto delete_plan_ack =
      common_codec::fromJson(
          dispatchJson(bridge,
                       provided::ServiceChannel::PlanDeletePlan,
                       nlohmann::json(imported_plan_id).dump()),
          static_cast<model::Ack*>(nullptr));
  EXPECT_TRUE(delete_plan_ack.success);

  const auto read_run_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::ExecutionRunReadRun,
                   common_codec::toJson(queryAll()));
  const auto runs =
      decodeJsonArray<model::ExecutionRun>(
          read_run_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::ExecutionRun*>(nullptr));
          });
  EXPECT_TRUE(runs.empty());

  const auto read_placement_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::RequirementPlacementReadPlacement,
                   common_codec::toJson(queryAll()));
  const auto placements =
      decodeJsonArray<model::RequirementPlacement>(
          read_placement_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::RequirementPlacement*>(nullptr));
          });
  EXPECT_TRUE(placements.empty());
}

TEST(AmeAutonomyContract, PlanAndExecuteCompletesFromStateFeedback) {
  auto wm = buildDomain();
  auto registry = buildRegistry();
  ame::PyramidAutonomyBridge bridge(wm, registry);

  const auto capabilities_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::CapabilitiesReadCapabilities,
                   common_codec::toJson(queryAll()));
  const auto capabilities =
      decodeJsonArray<model::Capabilities>(
          capabilities_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::Capabilities*>(nullptr));
          });
  ASSERT_EQ(capabilities.size(), 1u);
  EXPECT_TRUE(capabilities.front().supports_planning_requirements);
  EXPECT_TRUE(capabilities.front().supports_execution_requirements);
  EXPECT_TRUE(capabilities.front().supports_approved_plan_execution);

  model::StateUpdate initial_state;
  model::WorldFactUpdate initial_location;
  initial_location.key = "(at uav1 base)";
  initial_location.value = true;
  initial_location.source = "test-client";
  initial_location.authority = model::FactAuthorityLevel::Confirmed;
  initial_state.fact_update.push_back(initial_location);
  const auto initial_state_id =
      nlohmann::json::parse(
          dispatchJson(bridge,
                       provided::ServiceChannel::StateCreateState,
                       autonomy_codec::toJson(initial_state)))
          .get<std::string>();
  ASSERT_FALSE(initial_state_id.empty());

  const auto planning_requirement_id =
      nlohmann::json::parse(
          dispatchJson(bridge,
                       provided::ServiceChannel::PlanningRequirementCreatePlanningRequirement,
                       autonomy_codec::toJson(planningRequirement())))
          .get<std::string>();
  ASSERT_FALSE(planning_requirement_id.empty());

  const auto plan_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::PlanReadPlan,
                   common_codec::toJson(queryAll()));
  const auto plans =
      decodeJsonArray<model::Plan>(
          plan_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::Plan*>(nullptr));
          });
  ASSERT_EQ(plans.size(), 1u);
  ASSERT_TRUE(plans.front().plan_success);

  const auto execution_requirement_id =
      nlohmann::json::parse(
          dispatchJson(bridge,
                       provided::ServiceChannel::ExecutionRequirementCreateExecutionRequirement,
                       autonomy_codec::toJson(executionRequirement(
                           plans.front().id, planning_requirement_id))))
          .get<std::string>();
  ASSERT_FALSE(execution_requirement_id.empty());

  const auto initial_run_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::ExecutionRunReadRun,
                   common_codec::toJson(queryAll()));
  const auto initial_runs =
      decodeJsonArray<model::ExecutionRun>(
          initial_run_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::ExecutionRun*>(nullptr));
          });
  ASSERT_EQ(initial_runs.size(), 1u);
  EXPECT_EQ(initial_runs.front().execution_requirement_id,
            execution_requirement_id);
  EXPECT_EQ(initial_runs.front().planning_requirement_id,
            planning_requirement_id);
  EXPECT_EQ(initial_runs.front().state,
            model::ExecutionState::WaitingForComponents);
  EXPECT_EQ(initial_runs.front().achievement.status,
            common::Progress::InProgress);
  ASSERT_GE(initial_runs.front().outstanding_placement.size(), 2u);

  const auto initial_placement_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::RequirementPlacementReadPlacement,
                   common_codec::toJson(queryAll()));
  const auto initial_placements =
      decodeJsonArray<model::RequirementPlacement>(
          initial_placement_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::RequirementPlacement*>(nullptr));
          });
  ASSERT_GE(initial_placements.size(), 2u);
  bool found_open_mobility_placement = false;
  bool found_open_tactical_placement = false;
  for (const auto& placement : initial_placements) {
    EXPECT_EQ(placement.execution_requirement_id, execution_requirement_id);
    EXPECT_EQ(placement.planning_requirement_id, planning_requirement_id);
    if (placement.target_component == "mobility") {
      found_open_mobility_placement = true;
      EXPECT_EQ(placement.target_service, "Move_Service");
      EXPECT_EQ(placement.target_type,
                "pyramid.data_model.mobility.MoveRequirement");
      EXPECT_EQ(placement.progress, common::Progress::InProgress);
      EXPECT_FALSE(placement.target_requirement_id.empty());
    }
    if (placement.target_component == "tactical_objects") {
      found_open_tactical_placement = true;
      EXPECT_EQ(placement.progress, common::Progress::InProgress);
      EXPECT_FALSE(placement.target_requirement_id.empty());
    }
  }
  EXPECT_TRUE(found_open_mobility_placement);
  EXPECT_TRUE(found_open_tactical_placement);

  model::StateUpdate completion_state;
  model::WorldFactUpdate searched;
  searched.key = "(searched sector_a)";
  searched.value = true;
  searched.source = "test-client:tactical_objects";
  searched.authority = model::FactAuthorityLevel::Confirmed;
  completion_state.fact_update.push_back(searched);

  const auto update_ack_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::StateUpdateState,
                   autonomy_codec::toJson(completion_state));
  const auto update_ack =
      common_codec::fromJson(update_ack_payload,
                             static_cast<model::Ack*>(nullptr));
  ASSERT_TRUE(update_ack.success);

  const auto completed_run_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::ExecutionRunReadRun,
                   common_codec::toJson(queryAll()));
  const auto completed_runs =
      decodeJsonArray<model::ExecutionRun>(
          completed_run_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::ExecutionRun*>(nullptr));
          });
  ASSERT_EQ(completed_runs.size(), 1u);
  EXPECT_EQ(completed_runs.front().state,
            model::ExecutionState::Achieved);
  EXPECT_EQ(completed_runs.front().achievement.status,
            common::Progress::Completed);
  EXPECT_TRUE(completed_runs.front().outstanding_placement.empty());

  const auto completed_requirement_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::ExecutionRequirementReadExecutionRequirement,
                   common_codec::toJson(queryId(execution_requirement_id)));
  const auto completed_requirements =
      decodeJsonArray<model::ExecutionRequirement>(
          completed_requirement_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::ExecutionRequirement*>(nullptr));
          });
  ASSERT_EQ(completed_requirements.size(), 1u);
  EXPECT_EQ(completed_requirements.front().status.status,
            common::Progress::Completed);

  const auto completed_placement_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::RequirementPlacementReadPlacement,
                   common_codec::toJson(queryAll()));
  const auto completed_placements =
      decodeJsonArray<model::RequirementPlacement>(
          completed_placement_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::RequirementPlacement*>(nullptr));
          });
  ASSERT_GE(completed_placements.size(), 2u);
  for (const auto& placement : completed_placements) {
    EXPECT_EQ(placement.progress, common::Progress::Completed);
  }
}

TEST(AmeAutonomyContract, HostCanRegisterExecutionBindingsForProjection) {
  auto wm = buildDomain();
  auto registry = buildRegistry();

  ame::PyramidAutonomyBridgeOptions options;
  options.include_default_execution_bindings = false;
  options.execution_policy = ame::ExecutionBindingPolicy::RequireTypedPlacement;
  ame::PyramidAutonomyBridge bridge(wm, registry, options);
  EXPECT_TRUE(bridge.registerExecutionBinding(customMoveBinding()));
  EXPECT_TRUE(bridge.registerExecutionBinding(customSearchBinding()));

  const auto state_id =
      nlohmann::json::parse(
          dispatchJson(bridge,
                       provided::ServiceChannel::StateCreateState,
                       autonomy_codec::toJson(initialAtBaseState())))
          .get<std::string>();
  ASSERT_FALSE(state_id.empty());

  const auto planning_requirement_id =
      nlohmann::json::parse(
          dispatchJson(bridge,
                       provided::ServiceChannel::PlanningRequirementCreatePlanningRequirement,
                       autonomy_codec::toJson(planningRequirement())))
          .get<std::string>();
  ASSERT_FALSE(planning_requirement_id.empty());

  const auto plan_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::PlanReadPlan,
                   common_codec::toJson(queryAll()));
  const auto plans =
      decodeJsonArray<model::Plan>(
          plan_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::Plan*>(nullptr));
          });
  ASSERT_EQ(plans.size(), 1u);

  const auto execution_requirement_id =
      nlohmann::json::parse(
          dispatchJson(bridge,
                       provided::ServiceChannel::ExecutionRequirementCreateExecutionRequirement,
                       autonomy_codec::toJson(executionRequirement(
                           plans.front().id, planning_requirement_id))))
          .get<std::string>();
  ASSERT_FALSE(execution_requirement_id.empty());

  const auto placement_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::RequirementPlacementReadPlacement,
                   common_codec::toJson(queryAll()));
  const auto placements =
      decodeJsonArray<model::RequirementPlacement>(
          placement_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::RequirementPlacement*>(nullptr));
          });

  ASSERT_EQ(placements.size(), 2u);
  bool found_host_mobility = false;
  bool found_host_sensor = false;
  for (const auto& placement : placements) {
    EXPECT_EQ(placement.progress, common::Progress::InProgress);
    EXPECT_EQ(placement.operation,
              model::RequirementPlacementOperation::CreateRequirement);
    EXPECT_EQ(placement.execution_requirement_id, execution_requirement_id);
    if (placement.target_component == "host_mobility") {
      found_host_mobility = true;
      EXPECT_EQ(placement.target_service, "Host_Move_Service");
      EXPECT_EQ(placement.target_type, "host.data_model.MoveRequirement");
    }
    if (placement.target_component == "host_sensor") {
      found_host_sensor = true;
      EXPECT_EQ(placement.target_service, "Host_Search_Service");
      EXPECT_EQ(placement.target_type, "host.data_model.SearchRequirement");
    }
  }
  EXPECT_TRUE(found_host_mobility);
  EXPECT_TRUE(found_host_sensor);
}

TEST(AmeAutonomyContract, StrictPolicyFailsRunWhenNoBindingExists) {
  auto wm = buildDomain();
  auto registry = buildRegistry();

  ame::PyramidAutonomyBridgeOptions options;
  options.include_default_execution_bindings = false;
  options.execution_policy = ame::ExecutionBindingPolicy::RequireTypedPlacement;
  ame::PyramidAutonomyBridge bridge(wm, registry, options);

  const auto state_id =
      nlohmann::json::parse(
          dispatchJson(bridge,
                       provided::ServiceChannel::StateCreateState,
                       autonomy_codec::toJson(initialAtBaseState())))
          .get<std::string>();
  ASSERT_FALSE(state_id.empty());

  const auto planning_requirement_id =
      nlohmann::json::parse(
          dispatchJson(bridge,
                       provided::ServiceChannel::PlanningRequirementCreatePlanningRequirement,
                       autonomy_codec::toJson(planningRequirement())))
          .get<std::string>();
  ASSERT_FALSE(planning_requirement_id.empty());

  const auto plan_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::PlanReadPlan,
                   common_codec::toJson(queryAll()));
  const auto plans =
      decodeJsonArray<model::Plan>(
          plan_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::Plan*>(nullptr));
          });
  ASSERT_EQ(plans.size(), 1u);

  const auto execution_requirement_id =
      nlohmann::json::parse(
          dispatchJson(bridge,
                       provided::ServiceChannel::ExecutionRequirementCreateExecutionRequirement,
                       autonomy_codec::toJson(executionRequirement(
                           plans.front().id, planning_requirement_id))))
          .get<std::string>();
  ASSERT_FALSE(execution_requirement_id.empty());

  const auto run_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::ExecutionRunReadRun,
                   common_codec::toJson(queryAll()));
  const auto runs =
      decodeJsonArray<model::ExecutionRun>(
          run_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::ExecutionRun*>(nullptr));
          });
  ASSERT_EQ(runs.size(), 1u);
  EXPECT_EQ(runs.front().state, model::ExecutionState::Failed);
  EXPECT_EQ(runs.front().achievement.status, common::Progress::Failed);

  const auto requirement_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::ExecutionRequirementReadExecutionRequirement,
                   common_codec::toJson(queryId(execution_requirement_id)));
  const auto requirements =
      decodeJsonArray<model::ExecutionRequirement>(
          requirement_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::ExecutionRequirement*>(nullptr));
          });
  ASSERT_EQ(requirements.size(), 1u);
  EXPECT_EQ(requirements.front().status.status, common::Progress::Failed);

  const auto placement_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::RequirementPlacementReadPlacement,
                   common_codec::toJson(queryAll()));
  const auto placements =
      decodeJsonArray<model::RequirementPlacement>(
          placement_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::RequirementPlacement*>(nullptr));
          });
  ASSERT_EQ(placements.size(), 2u);
  for (const auto& placement : placements) {
    EXPECT_EQ(placement.progress, common::Progress::Failed);
  }
}
