#include <gtest/gtest.h>

#include <mujin/planner_component.h>

#include <test_component_utils.hpp>

///< REQ_ENGINE_002: Planner component shall solve goals from an in-process world model.
TEST(PlannerComponent, InProcessPlanningDoesNotRequirePddlFiles) {
  auto wm = buildUavWorldModel();

  mujin::PlannerComponent component;
  component.setParam("plan_audit.enabled", false);
  component.setParam("compiler.parallel", false);
  component.setInProcessWorldModel(&wm);
  registerUavActions(component.actionRegistry());

  EXPECT_EQ(component.configure(), PCL_OK);
  EXPECT_EQ(component.activate(), PCL_OK);

  const auto result =
      component.solveGoal({"(searched sector_a)", "(classified sector_a)"});
  EXPECT_TRUE(result.success);
  EXPECT_FALSE(result.bt_xml.empty());
  EXPECT_GE(result.plan_actions.size(), 3u);
  EXPECT_EQ(result.plan_actions.size(), result.action_indices.size());

  EXPECT_EQ(component.deactivate(), PCL_OK);
  EXPECT_EQ(component.cleanup(), PCL_OK);
  EXPECT_EQ(component.shutdown(), PCL_OK);
}
