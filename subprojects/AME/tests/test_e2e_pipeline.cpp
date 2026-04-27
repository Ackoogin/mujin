#include <gtest/gtest.h>
#include "ame/world_model.h"
#include "ame/action_registry.h"
#include "ame/planner.h"
#include "ame/plan_compiler.h"
#include "ame/bt_nodes/check_world_predicate.h"
#include "ame/bt_nodes/set_world_predicate.h"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

// Stub action that succeeds and accepts any ports
class StubAction : public BT::SyncActionNode {
public:
    StubAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("param0"),
            BT::InputPort<std::string>("param1"),
            BT::InputPort<std::string>("param2"),
        };
    }
};

static ame::WorldModel buildUAVDomain() {
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

    wm.registerPredicate("at", {"robot", "location"});
    wm.registerPredicate("searched", {"sector"});
    wm.registerPredicate("classified", {"sector"});

    wm.registerAction("move",
        {"?r", "?from", "?to"}, {"robot", "location", "location"},
        {"(at ?r ?from)"}, {"(at ?r ?to)"}, {"(at ?r ?from)"});
    wm.registerAction("search",
        {"?r", "?s"}, {"robot", "sector"},
        {"(at ?r ?s)"}, {"(searched ?s)"}, {});
    wm.registerAction("classify",
        {"?r", "?s"}, {"robot", "sector"},
        {"(at ?r ?s)", "(searched ?s)"}, {"(classified ?s)"}, {});

    return wm;
}

static ame::ActionRegistry buildRegistry() {
    ame::ActionRegistry reg;
    reg.registerAction("move", "StubMoveAction");
    reg.registerAction("search", "StubSearchAction");
    reg.registerAction("classify", "StubClassifyAction");
    return reg;
}

static BT::BehaviorTreeFactory buildFactory() {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ame::CheckWorldPredicate>("CheckWorldPredicate");
    factory.registerNodeType<ame::SetWorldPredicate>("SetWorldPredicate");
    factory.registerNodeType<StubAction>("StubMoveAction");
    factory.registerNodeType<StubAction>("StubSearchAction");
    factory.registerNodeType<StubAction>("StubClassifyAction");
    return factory;
}

// Full pipeline: plan -> compile -> execute -> verify goal
TEST(E2EPipeline, UAVSearchAndClassify) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true);
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    auto reg = buildRegistry();

    // Plan
    ame::Planner planner;
    auto plan_result = planner.solve(wm);
    ASSERT_TRUE(plan_result.success);
    ASSERT_GE(plan_result.steps.size(), 3u);

    // Compile to BT
    ame::PlanCompiler compiler;
    auto xml = compiler.compileSequential(plan_result.steps, wm, reg);
    ASSERT_FALSE(xml.empty());

    // Execute BT
    auto factory = buildFactory();
    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

    auto status = tree.tickWhileRunning();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

    // Verify goal state
    EXPECT_TRUE(wm.getFact("(searched sector_a)"));
    EXPECT_TRUE(wm.getFact("(classified sector_a)"));
    EXPECT_FALSE(wm.getFact("(at uav1 base)"));
}

// Test replanning: change world state, plan again, execute new plan
TEST(E2EPipeline, ReplanAfterStateChange) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true);
    wm.setGoal({"(searched sector_a)"});

    auto reg = buildRegistry();
    ame::Planner planner;
    ame::PlanCompiler compiler;

    // First plan: move + search
    auto result1 = planner.solve(wm);
    ASSERT_TRUE(result1.success);
    auto plan1_size = result1.steps.size();

    // Execute first plan
    auto xml1 = compiler.compileSequential(result1.steps, wm, reg);
    auto factory1 = buildFactory();
    auto tree1 = factory1.createTreeFromText(xml1);
    tree1.rootBlackboard()->set("world_model", &wm);
    auto status1 = tree1.tickWhileRunning();
    EXPECT_EQ(status1, BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(wm.getFact("(searched sector_a)"));

    // Now change goal: also classify sector_a
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    // Replan from current state (uav1 is now at sector_a, sector_a is searched)
    auto result2 = planner.solve(wm);
    ASSERT_TRUE(result2.success);

    // Should be shorter -- just classify (already at sector_a, already searched)
    EXPECT_LT(result2.steps.size(), plan1_size);

    // Execute second plan
    auto xml2 = compiler.compileSequential(result2.steps, wm, reg);
    auto factory2 = buildFactory();
    auto tree2 = factory2.createTreeFromText(xml2);
    tree2.rootBlackboard()->set("world_model", &wm);
    auto status2 = tree2.tickWhileRunning();
    EXPECT_EQ(status2, BT::NodeStatus::SUCCESS);

    // Verify both goals met
    EXPECT_TRUE(wm.getFact("(searched sector_a)"));
    EXPECT_TRUE(wm.getFact("(classified sector_a)"));
}

// Test that the causal-analysis compiler also works end-to-end
TEST(E2EPipeline, CausalAnalysisCompilation) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true);
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    auto reg = buildRegistry();

    ame::Planner planner;
    auto plan_result = planner.solve(wm);
    ASSERT_TRUE(plan_result.success);

    // Use compile() which does causal analysis
    ame::PlanCompiler compiler;
    auto xml = compiler.compile(plan_result.steps, wm, reg);
    ASSERT_FALSE(xml.empty());

    // Execute
    auto factory = buildFactory();
    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

    auto status = tree.tickWhileRunning();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

    // Verify
    EXPECT_TRUE(wm.getFact("(searched sector_a)"));
    EXPECT_TRUE(wm.getFact("(classified sector_a)"));
}

// Test pipeline with PDDL-parsed domain (programmatic construction should match)
TEST(E2EPipeline, PlannerStatistics) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true);
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    ame::Planner planner;
    auto result = planner.solve(wm);

    ASSERT_TRUE(result.success);
    EXPECT_GT(result.expanded, 0u);
    EXPECT_GT(result.generated, 0u);
    EXPECT_GT(result.cost, 0.0f);
}

// =============================================================================
// Extensions 3 & 5: Perception & Thread Safety E2E Tests
// =============================================================================

// Test State Authority: plan effects are BELIEVED, can be overridden by CONFIRMED
TEST(E2EPipeline, StateAuthoritySemantics) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true, "init", ame::FactAuthority::BELIEVED);
    wm.setGoal({"(searched sector_a)"});

    auto reg = buildRegistry();
    ame::Planner planner;
    ame::PlanCompiler compiler;

    // Plan and execute
    auto result = planner.solve(wm);
    ASSERT_TRUE(result.success);

    auto xml = compiler.compileSequential(result.steps, wm, reg);
    auto factory = buildFactory();
    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);
    tree.tickWhileRunning();

    // After execution, facts set by BT should be BELIEVED
    auto at_meta = wm.getFactMetadata("(at uav1 sector_a)");
    EXPECT_EQ(at_meta.authority, ame::FactAuthority::BELIEVED);

    // Simulate perception confirming the location
    wm.setFact("(at uav1 sector_a)", true, "perception:gps", ame::FactAuthority::CONFIRMED);
    auto confirmed_meta = wm.getFactMetadata("(at uav1 sector_a)");
    EXPECT_EQ(confirmed_meta.authority, ame::FactAuthority::CONFIRMED);
    EXPECT_EQ(confirmed_meta.source, "perception:gps");
}

// Test snapshot isolation: BT reads consistent state while perception updates
TEST(E2EPipeline, SnapshotIsolationDuringExecution) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true);

    // Capture snapshot before any changes
    auto snapshot1 = wm.captureSnapshot();
    unsigned base_id = wm.fluentIndex("(at uav1 base)");
    unsigned sector_id = wm.fluentIndex("(at uav1 sector_a)");

    EXPECT_TRUE(snapshot1->getFact(base_id));
    EXPECT_FALSE(snapshot1->getFact(sector_id));

    // Simulate perception updating live state
    wm.setFact("(at uav1 base)", false, "perception:tracker", ame::FactAuthority::CONFIRMED);
    wm.setFact("(at uav1 sector_a)", true, "perception:tracker", ame::FactAuthority::CONFIRMED);

    // Snapshot1 should be unchanged (immutable)
    EXPECT_TRUE(snapshot1->getFact(base_id));
    EXPECT_FALSE(snapshot1->getFact(sector_id));

    // New snapshot reflects updated state
    auto snapshot2 = wm.captureSnapshot();
    EXPECT_FALSE(snapshot2->getFact(base_id));
    EXPECT_TRUE(snapshot2->getFact(sector_id));
    EXPECT_GT(snapshot2->version, snapshot1->version);
}

// Test mutation queue: batch perception updates between BT ticks
TEST(E2EPipeline, MutationQueueBatchedPerception) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true);
    wm.setGoal({"(searched sector_a)", "(searched sector_b)"});

    unsigned searched_a = wm.fluentIndex("(searched sector_a)");
    unsigned searched_b = wm.fluentIndex("(searched sector_b)");

    // Simulate perception thread queueing multiple updates
    EXPECT_FALSE(wm.hasPendingMutations());
    wm.enqueueMutation(searched_a, true, "perception:camera", ame::FactAuthority::CONFIRMED);
    wm.enqueueMutation(searched_b, true, "perception:camera", ame::FactAuthority::CONFIRMED);
    EXPECT_TRUE(wm.hasPendingMutations());

    // State unchanged until applied
    EXPECT_FALSE(wm.getFact(searched_a));
    EXPECT_FALSE(wm.getFact(searched_b));

    // Apply all queued mutations atomically (between BT ticks)
    size_t applied = wm.applyQueuedMutations();
    EXPECT_EQ(applied, 2u);
    EXPECT_FALSE(wm.hasPendingMutations());

    // Now state reflects perception
    EXPECT_TRUE(wm.getFact(searched_a));
    EXPECT_TRUE(wm.getFact(searched_b));

    // Goal now satisfied -- replanning should find empty plan
    ame::Planner planner;
    auto result = planner.solve(wm);
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.steps.size(), 0u);  // already at goal
}

// Test authority conflict detection during replan
TEST(E2EPipeline, AuthorityConflictTriggersReplan) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true, "init", ame::FactAuthority::BELIEVED);
    wm.setGoal({"(searched sector_a)"});

    auto reg = buildRegistry();
    ame::Planner planner;
    ame::PlanCompiler compiler;

    // Plan expects: move(base->sector_a), search(sector_a)
    auto result1 = planner.solve(wm);
    ASSERT_TRUE(result1.success);
    EXPECT_GE(result1.steps.size(), 2u);

    // Simulate: plan predicts UAV will be at sector_a after move
    wm.setFact("(at uav1 base)", false, "plan:move", ame::FactAuthority::BELIEVED);
    wm.setFact("(at uav1 sector_a)", true, "plan:move", ame::FactAuthority::BELIEVED);

    // But perception says UAV is actually at sector_b (conflict!)
    unsigned sector_a_id = wm.fluentIndex("(at uav1 sector_a)");
    EXPECT_TRUE(wm.hasAuthorityConflict(sector_a_id, false));  // perception would say false

    // Perception overrides with ground truth
    wm.setFact("(at uav1 sector_a)", false, "perception:gps", ame::FactAuthority::CONFIRMED);
    wm.setFact("(at uav1 sector_b)", true, "perception:gps", ame::FactAuthority::CONFIRMED);

    // Replan from actual state
    auto result2 = planner.solve(wm);
    ASSERT_TRUE(result2.success);

    // New plan should start from sector_b
    auto xml = compiler.compileSequential(result2.steps, wm, reg);
    auto factory = buildFactory();
    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);
    tree.tickWhileRunning();

    EXPECT_TRUE(wm.getFact("(searched sector_a)"));
}

// Test snapshot metadata access
TEST(E2EPipeline, SnapshotMetadataAccess) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true, "init", ame::FactAuthority::BELIEVED);
    wm.setFact("(searched sector_a)", true, "perception:camera", ame::FactAuthority::CONFIRMED);

    auto snapshot = wm.captureSnapshot();

    unsigned base_id = wm.fluentIndex("(at uav1 base)");
    unsigned searched_id = wm.fluentIndex("(searched sector_a)");

    // Verify metadata in snapshot
    auto& base_meta = snapshot->getMetadata(base_id);
    EXPECT_EQ(base_meta.authority, ame::FactAuthority::BELIEVED);
    EXPECT_EQ(base_meta.source, "init");

    auto& searched_meta = snapshot->getMetadata(searched_id);
    EXPECT_EQ(searched_meta.authority, ame::FactAuthority::CONFIRMED);
    EXPECT_EQ(searched_meta.source, "perception:camera");
    EXPECT_GT(searched_meta.timestamp_us, 0u);
}
