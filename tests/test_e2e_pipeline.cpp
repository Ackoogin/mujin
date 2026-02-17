#include <gtest/gtest.h>
#include "mujin/world_model.h"
#include "mujin/action_registry.h"
#include "mujin/planner.h"
#include "mujin/plan_compiler.h"
#include "mujin/bt_nodes/check_world_predicate.h"
#include "mujin/bt_nodes/set_world_predicate.h"

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

static mujin::WorldModel buildUAVDomain() {
    mujin::WorldModel wm;
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

static mujin::ActionRegistry buildRegistry() {
    mujin::ActionRegistry reg;
    reg.registerAction("move", "StubMoveAction");
    reg.registerAction("search", "StubSearchAction");
    reg.registerAction("classify", "StubClassifyAction");
    return reg;
}

static BT::BehaviorTreeFactory buildFactory() {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<mujin::CheckWorldPredicate>("CheckWorldPredicate");
    factory.registerNodeType<mujin::SetWorldPredicate>("SetWorldPredicate");
    factory.registerNodeType<StubAction>("StubMoveAction");
    factory.registerNodeType<StubAction>("StubSearchAction");
    factory.registerNodeType<StubAction>("StubClassifyAction");
    return factory;
}

// Full pipeline: plan → compile → execute → verify goal
TEST(E2EPipeline, UAVSearchAndClassify) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true);
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    auto reg = buildRegistry();

    // Plan
    mujin::Planner planner;
    auto plan_result = planner.solve(wm);
    ASSERT_TRUE(plan_result.success);
    ASSERT_GE(plan_result.steps.size(), 3u);

    // Compile to BT
    mujin::PlanCompiler compiler;
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
    mujin::Planner planner;
    mujin::PlanCompiler compiler;

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

    // Should be shorter — just classify (already at sector_a, already searched)
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

    mujin::Planner planner;
    auto plan_result = planner.solve(wm);
    ASSERT_TRUE(plan_result.success);

    // Use compile() which does causal analysis
    mujin::PlanCompiler compiler;
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

    mujin::Planner planner;
    auto result = planner.solve(wm);

    ASSERT_TRUE(result.success);
    EXPECT_GT(result.expanded, 0u);
    EXPECT_GT(result.generated, 0u);
    EXPECT_GT(result.cost, 0.0f);
}
