#include <gtest/gtest.h>

#include <mujin/world_model_component.h>

///< REQ_ENGINE_001: World model component shall expose lifecycle and state query behavior.
TEST(WorldModelComponent, LifecycleAndQueries) {
  mujin::WorldModelComponent component;
  component.setParam("audit_log.enabled", false);

  EXPECT_EQ(component.configure(), PCL_OK);
  EXPECT_EQ(component.activate(), PCL_OK);
  EXPECT_TRUE(component.consumeStateDirty());

  auto& wm = component.worldModel();
  wm.typeSystem().addType("object");
  wm.typeSystem().addType("robot", "object");
  wm.typeSystem().addType("location", "object");
  wm.addObject("uav1", "robot");
  wm.addObject("base", "location");
  wm.registerPredicate("at", {"robot", "location"});
  wm.setGoal({"(at uav1 base)"});

  const auto set_result = component.setFact("(at uav1 base)", true, "test");
  EXPECT_TRUE(set_result.success);
  EXPECT_TRUE(component.consumeStateDirty());

  const auto get_result = component.getFact("(at uav1 base)");
  EXPECT_TRUE(get_result.found);
  EXPECT_TRUE(get_result.value);

  const auto snapshot = component.queryState({});
  ASSERT_EQ(snapshot.facts.size(), 1u);
  EXPECT_EQ(snapshot.facts.front().key, "(at uav1 base)");
  ASSERT_EQ(snapshot.goal_fluents.size(), 1u);
  EXPECT_EQ(snapshot.goal_fluents.front(), "(at uav1 base)");

  EXPECT_EQ(component.deactivate(), PCL_OK);
  EXPECT_EQ(component.cleanup(), PCL_OK);
  EXPECT_EQ(component.shutdown(), PCL_OK);
}
