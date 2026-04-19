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

namespace autonomy_codec = pyramid::data_model::autonomy;
namespace common = pyramid::data_model::common;
namespace common_codec = pyramid::data_model::common;
namespace model = pyramid::data_model;
namespace provided = pyramid::services::autonomy_backend::provided;

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
                   provided::ServiceChannel::CreateState,
                   autonomy_codec::toJson(state));
  const auto state_id =
      nlohmann::json::parse(state_id_payload).get<std::string>();
  ASSERT_FALSE(state_id.empty());
  EXPECT_TRUE(wm.getFact("(at uav1 base)"));

  model::PlanningExecutionRequirement requirement;
  requirement.mode = model::PlanningExecutionMode::PlanOnly;
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

  const auto requirement_id_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::CreateRequirement,
                   autonomy_codec::toJson(requirement));
  const auto requirement_id =
      nlohmann::json::parse(requirement_id_payload).get<std::string>();
  ASSERT_FALSE(requirement_id.empty());

  const auto read_requirement_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::ReadRequirement,
                   common_codec::toJson(queryId(requirement_id)));
  const auto requirements =
      decodeJsonArray<model::PlanningExecutionRequirement>(
          read_requirement_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::PlanningExecutionRequirement*>(nullptr));
          });
  ASSERT_EQ(requirements.size(), 1u);
  EXPECT_EQ(requirements.front().base.id, requirement_id);
  EXPECT_EQ(requirements.front().status.status, common::Progress::InProgress);

  const auto read_plan_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::ReadPlan,
                   common_codec::toJson(queryAll()));
  const auto plans =
      decodeJsonArray<model::Plan>(
          read_plan_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::Plan*>(nullptr));
          });
  ASSERT_EQ(plans.size(), 1u);
  EXPECT_EQ(plans.front().planning_execution_requirement_id, requirement_id);
  EXPECT_TRUE(plans.front().plan_success);
  EXPECT_GE(plans.front().step.size(), 2u);
  EXPECT_FALSE(plans.front().compiled_bt_xml.empty());

  const auto second_plan_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::ReadPlan,
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

  const auto read_run_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::ReadRun,
                   common_codec::toJson(queryAll()));
  const auto runs =
      decodeJsonArray<model::ExecutionRun>(
          read_run_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::ExecutionRun*>(nullptr));
          });
  ASSERT_EQ(runs.size(), 1u);
  EXPECT_EQ(runs.front().planning_execution_requirement_id, requirement_id);
  EXPECT_EQ(runs.front().plan_id, plans.front().id);
  EXPECT_EQ(runs.front().state, model::PlanningExecutionState::Achieved);
  EXPECT_EQ(runs.front().achievement.status, common::Progress::Completed);

  const auto read_placement_payload =
      dispatchJson(bridge,
                   provided::ServiceChannel::ReadPlacement,
                   common_codec::toJson(queryAll()));
  const auto placements =
      decodeJsonArray<model::RequirementPlacement>(
          read_placement_payload,
          [](const std::string& item) {
            return autonomy_codec::fromJson(
                item, static_cast<model::RequirementPlacement*>(nullptr));
          });
  ASSERT_GE(placements.size(), 2u);

  bool found_tactical_object_requirement = false;
  for (const auto& placement : placements) {
    if (placement.target_component == "tactical_objects") {
      found_tactical_object_requirement = true;
      EXPECT_EQ(placement.planning_execution_requirement_id, requirement_id);
      EXPECT_EQ(placement.plan_id, plans.front().id);
      EXPECT_EQ(placement.target_service, "Object_Of_Interest_Service");
      EXPECT_EQ(placement.target_type,
                "pyramid.data_model.tactical.ObjectInterestRequirement");
      EXPECT_EQ(placement.operation,
                model::RequirementPlacementOperation::CreateRequirement);
      EXPECT_EQ(placement.progress, common::Progress::NotStarted);
    }
  }
  EXPECT_TRUE(found_tactical_object_requirement);
}
