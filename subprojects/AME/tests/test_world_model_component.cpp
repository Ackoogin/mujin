#include <gtest/gtest.h>

#include <algorithm>

#include <ame/world_model.h>
#include <ame/world_model_component.h>

///< REQ_ENGINE_001: World model component shall expose lifecycle and state query behavior.
TEST(WorldModelComponent, LifecycleAndQueries) {
  ame::WorldModelComponent component;
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

namespace {
const char* kNavDomainWithScanned =
    "(define (domain nav)\n"
    "  (:requirements :strips :typing)\n"
    "  (:types robot location)\n"
    "  (:predicates (at ?r - robot ?l - location) (scanned ?l - location))\n"
    ")\n";
const char* kNavDomainNoScanned =
    "(define (domain nav)\n"
    "  (:requirements :strips :typing)\n"
    "  (:types robot location)\n"
    "  (:predicates (at ?r - robot ?l - location))\n"
    ")\n";
const char* kNavProblem =
    "(define (problem p) (:domain nav)\n"
    "  (:objects uav1 - robot base area1 - location)\n"
    "  (:init (at uav1 base))\n"
    "  (:goal (at uav1 area1)))\n";
}  // namespace

///< F-20/D9: a CONFIRMED fact must keep its authority + timestamp across a
///< domain reload, and a fact whose fluent is absent in the new domain must be
///< surfaced (LoadDomainResult::dropped_facts), not silently swallowed.
TEST(WorldModelComponent, DomainReloadPreservesProvenanceAndReportsDrops) {
  ame::WorldModelComponent component;
  component.setParam("audit_log.enabled", false);
  ASSERT_EQ(component.configure(), PCL_OK);
  ASSERT_EQ(component.activate(), PCL_OK);

  auto load1 = component.loadDomainFromStrings(kNavDomainWithScanned, kNavProblem);
  ASSERT_TRUE(load1.success) << load1.error_msg;

  // Operator-confirmed fact set directly on the world model.
  auto& wm = component.worldModel();
  wm.setFact("(scanned area1)", true, "operator", ame::FactAuthority::CONFIRMED);
  const auto before = wm.getFactMetadata("(scanned area1)");
  ASSERT_EQ(before.authority, ame::FactAuthority::CONFIRMED);

  // Reload the SAME domain — the confirmed fact must survive with authority
  // and timestamp intact (no silent demotion to BELIEVED).
  auto load2 = component.loadDomainFromStrings(kNavDomainWithScanned, kNavProblem);
  ASSERT_TRUE(load2.success) << load2.error_msg;
  EXPECT_TRUE(load2.dropped_facts.empty());

  const auto after = component.worldModel().getFactMetadata("(scanned area1)");
  EXPECT_EQ(after.authority, ame::FactAuthority::CONFIRMED);
  EXPECT_EQ(after.source, "operator");
  EXPECT_EQ(after.timestamp_us, before.timestamp_us);

  // Reload a domain WITHOUT the scanned predicate — the preserved fact can no
  // longer be grounded, so it must be reported as dropped, not lost silently.
  auto load3 = component.loadDomainFromStrings(kNavDomainNoScanned, kNavProblem);
  ASSERT_TRUE(load3.success) << load3.error_msg;
  EXPECT_NE(std::find(load3.dropped_facts.begin(), load3.dropped_facts.end(),
                      "(scanned area1)"),
            load3.dropped_facts.end());

  EXPECT_EQ(component.deactivate(), PCL_OK);
  EXPECT_EQ(component.cleanup(), PCL_OK);
  EXPECT_EQ(component.shutdown(), PCL_OK);
}
