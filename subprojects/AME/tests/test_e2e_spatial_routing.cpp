#include <gtest/gtest.h>
#include "ame/world_model.h"
#include "ame/action_registry.h"
#include "ame/planner.h"
#include "ame/plan_compiler.h"
#include "ame/spatial_oracle.h"
#include "ame/pyramid_service.h"
#include "ame/bt_nodes/check_world_predicate.h"
#include "ame/bt_nodes/set_world_predicate.h"
#include "ame/bt_nodes/invoke_service.h"
#include "ame/bt_nodes/follow_route.h"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

// =============================================================================
// Stub path-planner service: returns a fixed waypoint list for any route request
// =============================================================================

class StubPathPlannerService : public ame::IPyramidService {
public:
  bool call(const std::string&, const std::string&,
            const ame::ServiceMessage&, ame::ServiceMessage& response) override {
    response.set("waypoints", "51.0,1.0,100|51.1,1.1,100|51.2,1.2,100");
    response.set("status", "ok");
    return true;
  }

  uint64_t callAsync(const std::string&, const std::string&,
                     const ame::ServiceMessage&) override {
    return ++next_id_;
  }

  ame::AsyncCallStatus pollResult(uint64_t, ame::ServiceMessage& response) override {
    response.set("waypoints", "51.0,1.0,100|51.1,1.1,100|51.2,1.2,100");
    response.set("status", "ok");
    ++call_count_;
    return ame::AsyncCallStatus::SUCCESS;
  }

  void cancelCall(uint64_t) override {}

  unsigned callCount() const { return call_count_; }

private:
  uint64_t next_id_ = 0;
  unsigned call_count_ = 0;
};

// Stub search action
class StubSearchAction : public BT::SyncActionNode {
public:
  StubSearchAction(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {}
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("param0"),
        BT::InputPort<std::string>("param1"),
    };
  }
};

// =============================================================================
// Domain builder: UAV search domain with spatial predicates
// =============================================================================

static ame::WorldModel buildSpatialDomain() {
  ame::WorldModel wm;
  auto& ts = wm.typeSystem();
  ts.addType("object");
  ts.addType("location", "object");
  ts.addType("sector", "location");
  ts.addType("robot", "object");

  wm.addObject("uav1", "robot");
  wm.addObject("base", "location");
  wm.addObject("sector_a", "sector");
  wm.addObject("sector_b", "sector");
  wm.addObject("sector_c", "sector");

  // Core predicates
  wm.registerPredicate("at", {"robot", "location"});
  wm.registerPredicate("searched", {"sector"});

  // Spatial predicates (Option E: Hybrid)
  wm.registerPredicate("reachable", {"robot", "location"});
  wm.registerPredicate("nearest", {"robot", "location"});
  wm.registerPredicate("route-planned", {"robot", "location", "location"});

  // Move action now requires (reachable ?r ?to)
  wm.registerAction("move",
      {"?r", "?from", "?to"}, {"robot", "location", "location"},
      {"(at ?r ?from)", "(reachable ?r ?to)"},
      {"(at ?r ?to)"},
      {"(at ?r ?from)"});

  wm.registerAction("search",
      {"?r", "?s"}, {"robot", "sector"},
      {"(at ?r ?s)"},
      {"(searched ?s)"},
      {});

  return wm;
}

static ame::ActionRegistry buildSpatialRegistry() {
  ame::ActionRegistry reg;

  // move action uses a subtree: InvokeService → FollowRoute → SetWorldPredicate
  reg.registerActionSubTree("move", R"xml(
    <Sequence>
      <InvokeService service_name="path_planner" operation="compute_route"
                     param_names="?robot;?from;?to"
                     param_values="{param0};{param1};{param2}"
                     timeout_ms="3000"
                     response_json="{route_response}"/>
      <FollowRoute agent="{param0}" route="{route_response}"/>
    </Sequence>
  )xml");

  reg.registerAction("search", "StubSearchAction");
  return reg;
}

static BT::BehaviorTreeFactory buildSpatialFactory() {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ame::CheckWorldPredicate>("CheckWorldPredicate");
  factory.registerNodeType<ame::SetWorldPredicate>("SetWorldPredicate");
  factory.registerNodeType<ame::InvokeService>("InvokeService");
  factory.registerNodeType<ame::FollowRoute>("FollowRoute");
  factory.registerNodeType<StubSearchAction>("StubSearchAction");
  return factory;
}

static BT::Tree createSpatialTree(BT::BehaviorTreeFactory& factory,
                                  const std::string& xml,
                                  ame::WorldModel& wm,
                                  ame::IPyramidService& service) {
  auto blackboard = BT::Blackboard::create();
  blackboard->set("world_model", &wm);
  blackboard->set<ame::IPyramidService*>("pyramid_service", &service);
  return factory.createTreeFromText(xml, blackboard);
}

// =============================================================================
// ISpatialOracle unit tests
// =============================================================================

TEST(SpatialOracle, StubSetsAllReachable) {
  auto wm = buildSpatialDomain();
  wm.setFact("(at uav1 base)", true);

  ame::StubSpatialOracle oracle;
  oracle.updateReachability(wm);

  // All (reachable uav1 ...) facts should be CONFIRMED true
  EXPECT_TRUE(wm.getFact("(reachable uav1 base)"));
  EXPECT_TRUE(wm.getFact("(reachable uav1 sector_a)"));
  EXPECT_TRUE(wm.getFact("(reachable uav1 sector_b)"));
  EXPECT_TRUE(wm.getFact("(reachable uav1 sector_c)"));

  auto meta = wm.getFactMetadata("(reachable uav1 sector_a)");
  EXPECT_EQ(meta.authority, ame::FactAuthority::CONFIRMED);
  EXPECT_EQ(meta.source, "stub_spatial_oracle");
}

TEST(SpatialOracle, StubSetsNearestAlphabetically) {
  auto wm = buildSpatialDomain();
  wm.setFact("(at uav1 base)", true);

  ame::StubSpatialOracle oracle;
  oracle.updateReachability(wm);
  oracle.updateNearest(wm);

  // Nearest should be sector_a (first alphabetically, excluding "base" where
  // uav1 currently is)
  EXPECT_TRUE(wm.getFact("(nearest uav1 sector_a)"));
  EXPECT_FALSE(wm.getFact("(nearest uav1 sector_b)"));
  EXPECT_FALSE(wm.getFact("(nearest uav1 sector_c)"));
  // base is current location, so should not be nearest
  EXPECT_FALSE(wm.getFact("(nearest uav1 base)"));
}

TEST(SpatialOracle, NearestUpdatesAfterMove) {
  auto wm = buildSpatialDomain();
  wm.setFact("(at uav1 sector_a)", true);

  ame::StubSpatialOracle oracle;
  oracle.updateReachability(wm);
  oracle.updateNearest(wm);

  // Now at sector_a, nearest should be base (alphabetically: base < sector_b)
  EXPECT_TRUE(wm.getFact("(nearest uav1 base)"));
  EXPECT_FALSE(wm.getFact("(nearest uav1 sector_a)"));
}

// =============================================================================
// FollowRoute BT node tests
// =============================================================================

TEST(FollowRoute, SucceedsWithEmptyRoute) {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ame::FollowRoute>("FollowRoute");

  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="main">
        <FollowRoute agent="uav1" route=""/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml);
  auto status = tree.tickWhileRunning();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(FollowRoute, SucceedsWithWaypoints) {
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ame::FollowRoute>("FollowRoute");

  std::string xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="main">
        <FollowRoute agent="uav1" route="51.0,1.0,100|51.1,1.1,100|51.2,1.2,100"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml);
  auto status = tree.tickWhileRunning();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

// =============================================================================
// E2E Hybrid Spatial Routing: oracle → plan → compile → execute → verify
// =============================================================================

TEST(E2ESpatialRouting, OracleThenPlanThenExecute) {
  auto wm = buildSpatialDomain();
  wm.setFact("(at uav1 base)", true, "init", ame::FactAuthority::CONFIRMED);
  wm.setGoal({"(searched sector_a)"});

  // Phase 1: Spatial oracle populates reachability facts before planning
  ame::StubSpatialOracle oracle;
  oracle.updateReachability(wm);
  oracle.updateNearest(wm);

  // Verify oracle ran
  ASSERT_TRUE(wm.getFact("(reachable uav1 sector_a)"));

  // Phase 2: Plan — planner should produce move(uav1,base,sector_a) + search
  ame::Planner planner;
  auto plan_result = planner.solve(wm);
  ASSERT_TRUE(plan_result.success);
  ASSERT_GE(plan_result.steps.size(), 2u);

  // Phase 3: Compile with spatial registry (move → InvokeService+FollowRoute)
  auto reg = buildSpatialRegistry();
  ame::PlanCompiler compiler;
  auto xml = compiler.compileSequential(plan_result.steps, wm, reg);
  ASSERT_FALSE(xml.empty());

  // Verify compiled XML contains InvokeService and FollowRoute nodes
  EXPECT_NE(xml.find("InvokeService"), std::string::npos);
  EXPECT_NE(xml.find("FollowRoute"), std::string::npos);
  EXPECT_NE(xml.find("path_planner"), std::string::npos);

  // Phase 4: Execute BT with stub path planner service
  StubPathPlannerService path_planner;
  auto factory = buildSpatialFactory();
  auto tree = createSpatialTree(factory, xml, wm, path_planner);

  auto status = tree.tickWhileRunning();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // Phase 5: Verify goal met and path planner was called
  EXPECT_TRUE(wm.getFact("(searched sector_a)"));
  EXPECT_FALSE(wm.getFact("(at uav1 base)"));
  EXPECT_TRUE(wm.getFact("(at uav1 sector_a)"));
  EXPECT_GE(path_planner.callCount(), 1u);
}

TEST(E2ESpatialRouting, MultiSectorSearchWithOracle) {
  auto wm = buildSpatialDomain();
  wm.setFact("(at uav1 base)", true, "init", ame::FactAuthority::CONFIRMED);
  wm.setGoal({"(searched sector_a)", "(searched sector_b)"});

  // Oracle populates spatial facts
  ame::StubSpatialOracle oracle;
  oracle.updateReachability(wm);
  oracle.updateNearest(wm);

  // Plan
  ame::Planner planner;
  auto plan_result = planner.solve(wm);
  ASSERT_TRUE(plan_result.success);
  // Should have at least 4 steps: move+search for two sectors
  ASSERT_GE(plan_result.steps.size(), 4u);

  // Compile and execute
  auto reg = buildSpatialRegistry();
  ame::PlanCompiler compiler;
  auto xml = compiler.compileSequential(plan_result.steps, wm, reg);

  StubPathPlannerService path_planner;
  auto factory = buildSpatialFactory();
  auto tree = createSpatialTree(factory, xml, wm, path_planner);

  auto status = tree.tickWhileRunning();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  EXPECT_TRUE(wm.getFact("(searched sector_a)"));
  EXPECT_TRUE(wm.getFact("(searched sector_b)"));
  // Path planner called at least twice (one per move)
  EXPECT_GE(path_planner.callCount(), 2u);
}

TEST(E2ESpatialRouting, UnreachableLocationBlocksPlan) {
  auto wm = buildSpatialDomain();
  wm.setFact("(at uav1 base)", true, "init", ame::FactAuthority::CONFIRMED);
  wm.setGoal({"(searched sector_c)"});

  // Only set sector_a and sector_b reachable, NOT sector_c
  wm.setFact("(reachable uav1 base)", true, "oracle", ame::FactAuthority::CONFIRMED);
  wm.setFact("(reachable uav1 sector_a)", true, "oracle", ame::FactAuthority::CONFIRMED);
  wm.setFact("(reachable uav1 sector_b)", true, "oracle", ame::FactAuthority::CONFIRMED);
  // sector_c is NOT reachable

  // Plan should fail — cannot reach sector_c
  ame::Planner planner;
  auto plan_result = planner.solve(wm);
  EXPECT_FALSE(plan_result.success);
}

TEST(E2ESpatialRouting, ReplanAfterOracleRefresh) {
  auto wm = buildSpatialDomain();
  wm.setFact("(at uav1 base)", true, "init", ame::FactAuthority::CONFIRMED);
  wm.setGoal({"(searched sector_a)"});

  auto reg = buildSpatialRegistry();
  ame::Planner planner;
  ame::PlanCompiler compiler;
  ame::StubSpatialOracle oracle;

  // First plan: oracle → plan → execute
  oracle.updateReachability(wm);
  oracle.updateNearest(wm);

  auto result1 = planner.solve(wm);
  ASSERT_TRUE(result1.success);

  auto xml1 = compiler.compileSequential(result1.steps, wm, reg);
  StubPathPlannerService pp1;
  auto factory1 = buildSpatialFactory();
  auto tree1 = createSpatialTree(factory1, xml1, wm, pp1);
  tree1.tickWhileRunning();

  EXPECT_TRUE(wm.getFact("(searched sector_a)"));
  EXPECT_TRUE(wm.getFact("(at uav1 sector_a)"));

  // Now add a new goal and re-run oracle (simulates oracle refresh after move)
  wm.setGoal({"(searched sector_a)", "(searched sector_b)"});
  oracle.updateReachability(wm);
  oracle.updateNearest(wm);

  // Replan from current state
  auto result2 = planner.solve(wm);
  ASSERT_TRUE(result2.success);
  // Only need move+search for sector_b now (same step count as first plan,
  // but sector_a is already searched so it's a different plan)
  EXPECT_LE(result2.steps.size(), result1.steps.size());

  auto xml2 = compiler.compileSequential(result2.steps, wm, reg);
  StubPathPlannerService pp2;
  auto factory2 = buildSpatialFactory();
  auto tree2 = createSpatialTree(factory2, xml2, wm, pp2);
  tree2.tickWhileRunning();

  EXPECT_TRUE(wm.getFact("(searched sector_b)"));
}

TEST(E2ESpatialRouting, FactAuthorityOnSpatialFacts) {
  auto wm = buildSpatialDomain();
  wm.setFact("(at uav1 base)", true, "init", ame::FactAuthority::CONFIRMED);

  ame::StubSpatialOracle oracle;
  oracle.updateReachability(wm);

  // Oracle facts should be CONFIRMED authority
  auto meta = wm.getFactMetadata("(reachable uav1 sector_a)");
  EXPECT_EQ(meta.authority, ame::FactAuthority::CONFIRMED);

  // After plan execution, move effects are BELIEVED
  wm.setGoal({"(searched sector_a)"});

  ame::Planner planner;
  auto plan_result = planner.solve(wm);
  ASSERT_TRUE(plan_result.success);

  auto reg = buildSpatialRegistry();
  ame::PlanCompiler compiler;
  auto xml = compiler.compileSequential(plan_result.steps, wm, reg);

  StubPathPlannerService path_planner;
  auto factory = buildSpatialFactory();
  auto tree = createSpatialTree(factory, xml, wm, path_planner);
  tree.tickWhileRunning();

  // BT-applied location facts should be BELIEVED (SetWorldPredicate default)
  auto at_meta = wm.getFactMetadata("(at uav1 sector_a)");
  EXPECT_EQ(at_meta.authority, ame::FactAuthority::BELIEVED);

  // Reachability should still be CONFIRMED (set by oracle, not touched by BT)
  auto reach_meta = wm.getFactMetadata("(reachable uav1 sector_a)");
  EXPECT_EQ(reach_meta.authority, ame::FactAuthority::CONFIRMED);
}

TEST(E2ESpatialRouting, RouteAuditTrail) {
  auto wm = buildSpatialDomain();
  wm.setFact("(at uav1 base)", true, "init", ame::FactAuthority::CONFIRMED);
  wm.setGoal({"(searched sector_a)"});

  // Collect audit events
  std::vector<std::string> audit_log;
  wm.setAuditCallback(
      [&](uint64_t, uint64_t, const std::string& fact, bool value,
          const std::string& source) {
        audit_log.push_back(fact + "=" + (value ? "T" : "F") + " [" + source + "]");
      });

  ame::StubSpatialOracle oracle;
  oracle.updateReachability(wm);
  oracle.updateNearest(wm);

  ame::Planner planner;
  auto plan_result = planner.solve(wm);
  ASSERT_TRUE(plan_result.success);

  auto reg = buildSpatialRegistry();
  ame::PlanCompiler compiler;
  auto xml = compiler.compileSequential(plan_result.steps, wm, reg);

  StubPathPlannerService path_planner;
  auto factory = buildSpatialFactory();
  auto tree = createSpatialTree(factory, xml, wm, path_planner);
  tree.tickWhileRunning();

  // Audit log should contain oracle-sourced facts and BT-applied effects
  bool has_oracle_fact = false;
  bool has_bt_effect = false;
  for (const auto& entry : audit_log) {
    if (entry.find("stub_spatial_oracle") != std::string::npos) {
      has_oracle_fact = true;
    }
    if (entry.find("SetWorldPredicate") != std::string::npos) {
      has_bt_effect = true;
    }
  }
  EXPECT_TRUE(has_oracle_fact);
  EXPECT_TRUE(has_bt_effect);
}
