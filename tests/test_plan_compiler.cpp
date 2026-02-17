#include <gtest/gtest.h>
#include "mujin/plan_compiler.h"
#include "mujin/action_registry.h"
#include "mujin/world_model.h"
#include "mujin/bt_nodes/check_world_predicate.h"
#include "mujin/bt_nodes/set_world_predicate.h"

#include <behaviortree_cpp/bt_factory.h>

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

static unsigned findAction(const mujin::WorldModel& wm, const std::string& sig) {
    for (unsigned i = 0; i < wm.numGroundActions(); ++i) {
        if (wm.groundActions()[i].signature == sig) return i;
    }
    throw std::runtime_error("Action not found: " + sig);
}

TEST(PlanCompiler, CompileEmptyPlan) {
    auto wm = buildUAVDomain();
    auto reg = buildRegistry();
    mujin::PlanCompiler compiler;

    auto xml = compiler.compile({}, wm, reg);
    EXPECT_NE(xml.find("Sequence"), std::string::npos);
    EXPECT_NE(xml.find("BTCPP_format"), std::string::npos);
}

TEST(PlanCompiler, CompileSequentialTwoActions) {
    auto wm = buildUAVDomain();
    auto reg = buildRegistry();
    mujin::PlanCompiler compiler;

    std::vector<mujin::PlanStep> plan = {
        {findAction(wm, "move(uav1,base,sector_a)")},
        {findAction(wm, "search(uav1,sector_a)")},
    };

    auto xml = compiler.compileSequential(plan, wm, reg);

    EXPECT_NE(xml.find("<Sequence>"), std::string::npos);
    EXPECT_NE(xml.find("CheckWorldPredicate"), std::string::npos);
    EXPECT_NE(xml.find("SetWorldPredicate"), std::string::npos);
    EXPECT_NE(xml.find("StubMoveAction"), std::string::npos);
    EXPECT_NE(xml.find("StubSearchAction"), std::string::npos);
}

TEST(PlanCompiler, CausallyLinkedIsSequential) {
    auto wm = buildUAVDomain();
    auto reg = buildRegistry();
    mujin::PlanCompiler compiler;

    // move adds at(uav1,sector_a) which is a precondition of search
    std::vector<mujin::PlanStep> plan = {
        {findAction(wm, "move(uav1,base,sector_a)")},
        {findAction(wm, "search(uav1,sector_a)")},
    };

    auto xml = compiler.compile(plan, wm, reg);
    EXPECT_EQ(xml.find("<Parallel"), std::string::npos);
    EXPECT_NE(xml.find("<Sequence>"), std::string::npos);
}

TEST(PlanCompiler, IndependentActionsAreParallel) {
    mujin::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("item");
    wm.addObject("a", "item");
    wm.addObject("b", "item");
    wm.registerPredicate("done", {"item"});

    wm.registerAction("do_a", {"?x"}, {"item"}, {}, {"(done ?x)"}, {});
    wm.registerAction("do_b", {"?x"}, {"item"}, {}, {"(done ?x)"}, {});

    mujin::ActionRegistry reg;
    reg.registerAction("do_a", "DoANode");
    reg.registerAction("do_b", "DoBNode");

    mujin::PlanCompiler compiler;

    std::vector<mujin::PlanStep> plan = {
        {findAction(wm, "do_a(a)")},
        {findAction(wm, "do_b(b)")},
    };

    auto xml = compiler.compile(plan, wm, reg);
    EXPECT_NE(xml.find("<Parallel"), std::string::npos);
}

TEST(PlanCompiler, ReactiveSequenceUsed) {
    auto wm = buildUAVDomain();
    mujin::ActionRegistry reg;
    reg.registerAction("move", "StubMoveAction", true);
    reg.registerAction("search", "StubSearchAction", false);

    mujin::PlanCompiler compiler;

    std::vector<mujin::PlanStep> plan = {
        {findAction(wm, "move(uav1,base,sector_a)")},
    };

    auto xml = compiler.compileSequential(plan, wm, reg);
    EXPECT_NE(xml.find("ReactiveSequence"), std::string::npos);
}

TEST(PlanCompiler, CompiledXMLIsLoadable) {
    auto wm = buildUAVDomain();
    auto reg = buildRegistry();
    mujin::PlanCompiler compiler;

    std::vector<mujin::PlanStep> plan = {
        {findAction(wm, "move(uav1,base,sector_a)")},
        {findAction(wm, "search(uav1,sector_a)")},
        {findAction(wm, "classify(uav1,sector_a)")},
    };

    auto xml = compiler.compileSequential(plan, wm, reg);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<mujin::CheckWorldPredicate>("CheckWorldPredicate");
    factory.registerNodeType<mujin::SetWorldPredicate>("SetWorldPredicate");
    factory.registerNodeType<StubAction>("StubMoveAction");
    factory.registerNodeType<StubAction>("StubSearchAction");
    factory.registerNodeType<StubAction>("StubClassifyAction");

    EXPECT_NO_THROW({
        auto tree = factory.createTreeFromText(xml);
    });
}

TEST(PlanCompiler, FullPlanExecution) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true);

    auto reg = buildRegistry();
    mujin::PlanCompiler compiler;

    std::vector<mujin::PlanStep> plan = {
        {findAction(wm, "move(uav1,base,sector_a)")},
        {findAction(wm, "search(uav1,sector_a)")},
        {findAction(wm, "classify(uav1,sector_a)")},
    };

    auto xml = compiler.compileSequential(plan, wm, reg);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<mujin::CheckWorldPredicate>("CheckWorldPredicate");
    factory.registerNodeType<mujin::SetWorldPredicate>("SetWorldPredicate");
    factory.registerNodeType<StubAction>("StubMoveAction");
    factory.registerNodeType<StubAction>("StubSearchAction");
    factory.registerNodeType<StubAction>("StubClassifyAction");

    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

    auto status = tree.tickWhileRunning();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

    EXPECT_FALSE(wm.getFact("(at uav1 base)"));
    EXPECT_TRUE(wm.getFact("(at uav1 sector_a)"));
    EXPECT_TRUE(wm.getFact("(searched sector_a)"));
    EXPECT_TRUE(wm.getFact("(classified sector_a)"));
}
