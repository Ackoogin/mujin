#include <gtest/gtest.h>

#include <ame/bt_nodes/check_world_predicate.h>
#include <ame/bt_nodes/set_world_predicate.h>
#include <ame/executor_component.h>
#include <ame/planner_component.h>

#include <test_component_utils.hpp>

///< REQ_ENGINE_003: Executor component shall run compiled planner output against the shared world model.
TEST(ExecutorComponent, ExecutesPlannerOutputAgainstSharedWorldModel) {
  auto wm = buildUavWorldModel();

  ame::PlannerComponent planner_component;
  planner_component.setParam("plan_audit.enabled", false);
  planner_component.setInProcessWorldModel(&wm);
  registerUavActions(planner_component.actionRegistry());
  ASSERT_EQ(planner_component.configure(), PCL_OK);
  ASSERT_EQ(planner_component.activate(), PCL_OK);

  const auto plan_result =
      planner_component.solveGoal({"(searched sector_a)", "(classified sector_a)"});
  ASSERT_TRUE(plan_result.success);

  ame::ExecutorComponent executor_component;
  executor_component.setParam("bt_log.enabled", false);
  executor_component.setInProcessWorldModel(&wm);
  executor_component.factory().registerNodeType<ame::CheckWorldPredicate>(
      "CheckWorldPredicate");
  executor_component.factory().registerNodeType<ame::SetWorldPredicate>(
      "SetWorldPredicate");
  registerUavStubNodes(executor_component.factory());

  std::vector<std::string> events;
  executor_component.setEventSink(
      [&events](const std::string& json_line) { events.push_back(json_line); });

  ASSERT_EQ(executor_component.configure(), PCL_OK);
  ASSERT_EQ(executor_component.activate(), PCL_OK);
  executor_component.loadAndExecute(plan_result.bt_xml);

  for (int i = 0; i < 50 && executor_component.isExecuting(); ++i) {
    executor_component.tickOnce();
  }

  EXPECT_EQ(executor_component.lastStatus(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(wm.getFact("(searched sector_a)"));
  EXPECT_TRUE(wm.getFact("(classified sector_a)"));
  EXPECT_FALSE(events.empty());

  EXPECT_EQ(executor_component.deactivate(), PCL_OK);
  EXPECT_EQ(executor_component.cleanup(), PCL_OK);
  EXPECT_EQ(executor_component.shutdown(), PCL_OK);
  EXPECT_EQ(planner_component.deactivate(), PCL_OK);
  EXPECT_EQ(planner_component.cleanup(), PCL_OK);
  EXPECT_EQ(planner_component.shutdown(), PCL_OK);
}
