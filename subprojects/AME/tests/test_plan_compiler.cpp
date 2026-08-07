#include <gtest/gtest.h>
#include "ame/plan_compiler.h"
#include "ame/action_registry.h"
#include "ame/world_model.h"
#include "ame/bt_nodes/goal_reached.h"
#include "ame/bt_nodes/planned_action.h"
#include "ame/bt_nodes/planned_action_node.h"
#include "ame/bt_nodes/simulated_action.h"

#include <behaviortree_cpp/bt_factory.h>

// Stub action that succeeds and accepts any ports
class StubAction : public ame::PlannedActionNode {
public:
    StubAction(const std::string& name, const BT::NodeConfiguration& config)
        : PlannedActionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return withBasePorts({
            BT::InputPort<std::string>("param0"),
            BT::InputPort<std::string>("param1"),
            BT::InputPort<std::string>("param2"),
        });
    }
protected:
    BT::NodeStatus onActionStart() override { return BT::NodeStatus::SUCCESS; }
};

static void registerCoreNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<ame::GoalReached>("GoalReached");
    factory.registerNodeType<ame::PlannedAction>("PlannedAction");
    factory.registerNodeType<ame::SimulatedAction>("SimulatedAction");
}

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

static unsigned findAction(const ame::WorldModel& wm, const std::string& sig) {
    for (unsigned i = 0; i < wm.numGroundActions(); ++i) {
        if (wm.groundActions()[i].signature == sig) return i;
    }
    throw std::runtime_error("Action not found: " + sig);
}

static size_t countOccurrences(const std::string& text, const std::string& needle) {
    size_t count = 0;
    size_t pos = 0;
    while ((pos = text.find(needle, pos)) != std::string::npos) {
        ++count;
        pos += needle.size();
    }
    return count;
}

TEST(PlanCompiler, CompileEmptyPlan) {
    auto wm = buildUAVDomain();
    auto reg = buildRegistry();
    ame::PlanCompiler compiler;

    auto xml = compiler.compile({}, wm, reg);
    EXPECT_NE(xml.find("Sequence"), std::string::npos);
    EXPECT_NE(xml.find("BTCPP_format"), std::string::npos);
}

TEST(PlanCompiler, CompileSequentialTwoActions) {
    auto wm = buildUAVDomain();
    auto reg = buildRegistry();
    ame::PlanCompiler compiler;

    std::vector<ame::PlanStep> plan = {
        {findAction(wm, "move(uav1,base,sector_a)")},
        {findAction(wm, "search(uav1,sector_a)")},
    };

    auto xml = compiler.compileSequential(plan, wm, reg);

    EXPECT_NE(xml.find("<Sequence>"), std::string::npos);
    EXPECT_EQ(xml.find("CheckWorldPredicate"), std::string::npos);
    EXPECT_EQ(xml.find("<Set"), std::string::npos);
    EXPECT_NE(xml.find("StubMoveAction"), std::string::npos);
    EXPECT_NE(xml.find("StubSearchAction"), std::string::npos);
    EXPECT_NE(xml.find("name=\"move(uav1,base,sector_a)\""), std::string::npos);
    EXPECT_NE(xml.find("ame_preconditions=\"(at uav1 base)\""), std::string::npos);
    EXPECT_NE(xml.find("ame_add_effects=\"(at uav1 sector_a)\""), std::string::npos);
    EXPECT_NE(xml.find("ame_del_effects=\"(at uav1 base)\""), std::string::npos);
}

TEST(PlanCompiler, CausallyLinkedIsSequential) {
    auto wm = buildUAVDomain();
    auto reg = buildRegistry();
    ame::PlanCompiler compiler;

    // move adds at(uav1,sector_a) which is a precondition of search
    std::vector<ame::PlanStep> plan = {
        {findAction(wm, "move(uav1,base,sector_a)")},
        {findAction(wm, "search(uav1,sector_a)")},
    };

    auto xml = compiler.compile(plan, wm, reg);
    EXPECT_EQ(xml.find("<Parallel"), std::string::npos);
    EXPECT_NE(xml.find("<Sequence>"), std::string::npos);
}

TEST(PlanCompiler, IndependentActionsAreParallel) {
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("item");
    wm.addObject("a", "item");
    wm.addObject("b", "item");
    wm.registerPredicate("done", {"item"});

    wm.registerAction("do_a", {"?x"}, {"item"}, {}, {"(done ?x)"}, {});
    wm.registerAction("do_b", {"?x"}, {"item"}, {}, {"(done ?x)"}, {});

    ame::ActionRegistry reg;
    reg.registerAction("do_a", "DoANode");
    reg.registerAction("do_b", "DoBNode");

    ame::PlanCompiler compiler;

    std::vector<ame::PlanStep> plan = {
        {findAction(wm, "do_a(a)")},
        {findAction(wm, "do_b(b)")},
    };

    auto xml = compiler.compile(plan, wm, reg);
    EXPECT_NE(xml.find("<Parallel"), std::string::npos);
}

TEST(PlanCompiler, ReactiveFlagBecomesActionPort) {
    auto wm = buildUAVDomain();
    ame::ActionRegistry reg;
    reg.registerAction("move", "StubMoveAction", true);
    reg.registerAction("search", "StubSearchAction", false);

    ame::PlanCompiler compiler;

    std::vector<ame::PlanStep> plan = {
        {findAction(wm, "move(uav1,base,sector_a)")},
    };

    auto xml = compiler.compileSequential(plan, wm, reg);
    EXPECT_EQ(xml.find("ReactiveSequence"), std::string::npos);
    EXPECT_NE(xml.find("ame_reactive=\"true\""), std::string::npos);
}

TEST(PlanCompiler, CompiledXMLIsLoadable) {
    auto wm = buildUAVDomain();
    auto reg = buildRegistry();
    ame::PlanCompiler compiler;

    std::vector<ame::PlanStep> plan = {
        {findAction(wm, "move(uav1,base,sector_a)")},
        {findAction(wm, "search(uav1,sector_a)")},
        {findAction(wm, "classify(uav1,sector_a)")},
    };

    auto xml = compiler.compileSequential(plan, wm, reg);

    BT::BehaviorTreeFactory factory;
    registerCoreNodes(factory);
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
    ame::PlanCompiler compiler;

    std::vector<ame::PlanStep> plan = {
        {findAction(wm, "move(uav1,base,sector_a)")},
        {findAction(wm, "search(uav1,sector_a)")},
        {findAction(wm, "classify(uav1,sector_a)")},
    };

    auto xml = compiler.compileSequential(plan, wm, reg);

    BT::BehaviorTreeFactory factory;
    registerCoreNodes(factory);
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


// A planned action defaults to accepting a precondition whichever way it was
// established. Asking for confirmed state makes it wait for an observation.
TEST(PlannedActionAuthority, ConfirmedRequirementRejectsBelievedPrecondition) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true, "planner_init", ame::FactAuthority::BELIEVED);

    BT::BehaviorTreeFactory factory;
    registerCoreNodes(factory);
    factory.registerNodeType<StubAction>("StubMoveAction");

    const std::string xml =
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\">"
        "<StubMoveAction name=\"move\" ame_preconditions=\"(at uav1 base)\""
        " ame_add_effects=\"(at uav1 sector_a)\""
        " ame_required_authority=\"confirmed\"/>"
        "</BehaviorTree></root>";

    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);
    EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
    EXPECT_FALSE(wm.getFact("(at uav1 sector_a)"));

    // The same fact, once observed rather than predicted, lets the action run.
    wm.setFact("(at uav1 base)", true, "perception", ame::FactAuthority::CONFIRMED);
    auto confirmed_tree = factory.createTreeFromText(xml);
    confirmed_tree.rootBlackboard()->set("world_model", &wm);
    EXPECT_EQ(confirmed_tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(wm.getFact("(at uav1 sector_a)"));
}

TEST(PlannedActionAuthority, DefaultAcceptsBelievedPrecondition) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true, "planner_init", ame::FactAuthority::BELIEVED);

    BT::BehaviorTreeFactory factory;
    registerCoreNodes(factory);
    factory.registerNodeType<StubAction>("StubMoveAction");

    auto tree = factory.createTreeFromText(
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\">"
        "<StubMoveAction name=\"move\" ame_preconditions=\"(at uav1 base)\"/>"
        "</BehaviorTree></root>");
    tree.rootBlackboard()->set("world_model", &wm);
    EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
}

// The goal guard is held to the same standard, so a mission is not declared
// complete on predicted facts when the run asked for observed ones.
TEST(PlannedActionAuthority, GoalReachedHonoursConfirmedRequirement) {
    auto wm = buildUAVDomain();
    wm.setFact("(searched sector_a)", true, "planner_init",
               ame::FactAuthority::BELIEVED);

    BT::BehaviorTreeFactory factory;
    registerCoreNodes(factory);

    const std::string xml =
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\">"
        "<GoalReached goals=\"(searched sector_a)\""
        " ame_required_authority=\"confirmed\"/>"
        "</BehaviorTree></root>";

    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);
    EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::FAILURE);

    wm.setFact("(searched sector_a)", true, "perception",
               ame::FactAuthority::CONFIRMED);
    auto confirmed_tree = factory.createTreeFromText(xml);
    confirmed_tree.rootBlackboard()->set("world_model", &wm);
    EXPECT_EQ(confirmed_tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
}

// The decorator applies the same rule to a registered subtree.
TEST(PlannedActionAuthority, DecoratorHonoursConfirmedRequirement) {
    auto wm = buildUAVDomain();
    wm.setFact("(at uav1 base)", true, "planner_init", ame::FactAuthority::BELIEVED);

    BT::BehaviorTreeFactory factory;
    registerCoreNodes(factory);

    auto tree = factory.createTreeFromText(
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\">"
        "<PlannedAction name=\"move\" ame_preconditions=\"(at uav1 base)\""
        " ame_required_authority=\"confirmed\">"
        "<AlwaysSuccess/></PlannedAction>"
        "</BehaviorTree></root>");
    tree.rootBlackboard()->set("world_model", &wm);
    EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
}

TEST(PlanCompiler, SubtreeIsWrappedInPlannedAction) {
    auto wm = buildUAVDomain();
    ame::ActionRegistry registry;
    registry.registerActionSubTree(
        "move", "<Sequence><AlwaysSuccess/><AlwaysSuccess/></Sequence>", true);
    ame::PlanCompiler compiler;
    std::vector<ame::PlanStep> plan = {
        {findAction(wm, "move(uav1,base,sector_a)")},
    };

    const auto xml = compiler.compileSequential(plan, wm, registry);
    EXPECT_NE(xml.find("<PlannedAction name=\"move(uav1,base,sector_a)\""),
              std::string::npos);
    EXPECT_NE(xml.find("ame_reactive=\"true\""), std::string::npos);
    EXPECT_NE(xml.find("<Sequence><AlwaysSuccess/><AlwaysSuccess/></Sequence>"),
              std::string::npos);
}

TEST(PlanCompiler, ThreeStepTreeHasThreeActionElementsAndOneGoalCondition) {
    auto wm = buildUAVDomain();
    wm.setGoal({"(classified sector_a)"});
    auto registry = buildRegistry();
    ame::PlanCompiler compiler;
    std::vector<ame::PlanStep> plan = {
        {findAction(wm, "move(uav1,base,sector_a)")},
        {findAction(wm, "search(uav1,sector_a)")},
        {findAction(wm, "classify(uav1,sector_a)")},
    };

    const auto xml = compiler.compileSequential(plan, wm, registry);
    auto count = [&xml](const std::string& token) {
        size_t result = 0;
        for (size_t pos = 0; (pos = xml.find(token, pos)) != std::string::npos;
             pos += token.size()) {
            ++result;
        }
        return result;
    };
    EXPECT_EQ(count("<StubMoveAction"), 1u);
    EXPECT_EQ(count("<StubSearchAction"), 1u);
    EXPECT_EQ(count("<StubClassifyAction"), 1u);
    EXPECT_EQ(count("<GoalReached"), 1u);
    EXPECT_EQ(xml.find("CheckWorldPredicate"), std::string::npos);
    EXPECT_EQ(xml.find("<Set"), std::string::npos);
}

#include "ame/pddl_parser.h"
TEST(PlanCompiler, UnregisteredActionThrows) {
    auto wm = buildUAVDomain();
    ame::ActionRegistry reg;
    reg.registerAction("search", "StubSearchAction");
    ame::PlanCompiler compiler;

    std::vector<ame::PlanStep> plan = {
        {findAction(wm, "move(uav1,base,sector_a)")},
    };

    EXPECT_THROW(compiler.compileSequential(plan, wm, reg), std::runtime_error);
    EXPECT_THROW(compiler.compile(plan, wm, reg), std::runtime_error);
}

TEST(PlanCompiler, ParallelPhaseIsOrderedByStepIndex) {
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("item");
    wm.addObject("a", "item");
    wm.addObject("b", "item");
    wm.registerPredicate("started", {"item"});
    wm.registerPredicate("done", {"item"});

    wm.registerAction("start", {"?x"}, {"item"}, {}, {"(started ?x)"}, {});
    wm.registerAction("finish", {"?x"}, {"item"}, {"(started ?x)"}, {"(done ?x)"}, {});

    ame::ActionRegistry reg;
    reg.registerAction("start", "StartNode");
    reg.registerAction("finish", "FinishNode");

    ame::PlanCompiler compiler;

    std::vector<ame::PlanStep> plan = {
        {findAction(wm, "start(a)")},
        {findAction(wm, "start(b)")},
        {findAction(wm, "finish(a)")},
        {findAction(wm, "finish(b)")},
    };

    auto xml = compiler.compile(plan, wm, reg);
    ASSERT_NE(xml.find("<Parallel"), std::string::npos);

    const auto phase_a = xml.find("name=\"start(a)\"");
    const auto phase_b = xml.find("name=\"start(b)\"");
    ASSERT_NE(phase_a, std::string::npos);
    ASSERT_NE(phase_b, std::string::npos);
    EXPECT_LT(phase_a, phase_b);
}

TEST(PlanCompiler, ParallelPhaseCanBeFollowedByDependentStep) {
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("item");
    wm.addObject("a", "item");
    wm.addObject("b", "item");
    wm.registerPredicate("started", {"item"});
    wm.registerPredicate("done", {"item"});

    wm.registerAction("start", {"?x"}, {"item"}, {}, {"(started ?x)"}, {});
    wm.registerAction("finish", {"?x"}, {"item"}, {"(started ?x)"}, {"(done ?x)"}, {});

    ame::ActionRegistry reg;
    reg.registerAction("start", "StartNode");
    reg.registerAction("finish", "FinishNode");

    ame::PlanCompiler compiler;

    std::vector<ame::PlanStep> plan = {
        {findAction(wm, "start(a)")},
        {findAction(wm, "start(b)")},
        {findAction(wm, "finish(a)")},
    };

    auto xml = compiler.compile(plan, wm, reg);
    const auto parallel = xml.find("<Parallel success_count=\"2\" failure_count=\"1\">");
    ASSERT_NE(parallel, std::string::npos);

    const auto start_a = xml.find("name=\"start(a)\"");
    const auto start_b = xml.find("name=\"start(b)\"");
    const auto parallel_close = xml.find("</Parallel>", parallel);
    const auto finish_a = xml.find("name=\"finish(a)\"");
    ASSERT_NE(start_a, std::string::npos);
    ASSERT_NE(start_b, std::string::npos);
    ASSERT_NE(parallel_close, std::string::npos);
    ASSERT_NE(finish_a, std::string::npos);

    EXPECT_LT(parallel, start_a);
    EXPECT_LT(start_a, start_b);
    EXPECT_LT(start_b, parallel_close);
    EXPECT_LT(parallel_close, finish_a);
}

TEST(PlanCompiler, IndependentChainsEmitParallelExecutionPhases) {
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("item");
    wm.addObject("a", "item");
    wm.addObject("b", "item");
    wm.registerPredicate("started", {"item"});
    wm.registerPredicate("done", {"item"});

    wm.registerAction("start", {"?x"}, {"item"}, {}, {"(started ?x)"}, {});
    wm.registerAction("finish", {"?x"}, {"item"}, {"(started ?x)"}, {"(done ?x)"}, {});

    ame::ActionRegistry reg;
    reg.registerAction("start", "StartNode");
    reg.registerAction("finish", "FinishNode");

    ame::PlanCompiler compiler;

    std::vector<ame::PlanStep> plan = {
        {findAction(wm, "start(a)")},
        {findAction(wm, "start(b)")},
        {findAction(wm, "finish(a)")},
        {findAction(wm, "finish(b)")},
    };

    auto xml = compiler.compile(plan, wm, reg);
    EXPECT_EQ(countOccurrences(xml, "<Parallel success_count=\"2\" failure_count=\"1\">"), 2u);

    const auto start_a = xml.find("name=\"start(a)\"");
    const auto start_b = xml.find("name=\"start(b)\"");
    const auto finish_a = xml.find("name=\"finish(a)\"");
    const auto finish_b = xml.find("name=\"finish(b)\"");
    ASSERT_NE(start_a, std::string::npos);
    ASSERT_NE(start_b, std::string::npos);
    ASSERT_NE(finish_a, std::string::npos);
    ASSERT_NE(finish_b, std::string::npos);

    EXPECT_LT(start_a, start_b);
    EXPECT_LT(start_b, finish_a);
    EXPECT_LT(finish_a, finish_b);
}

TEST(PlanCompilerNegPre, AddVsLaterNegPreStaysSequential) {
    // Step 0 adds (armed x); step 1 needs (not armed x). The later add threatens
    // the negative precondition, so the two must not be parallelised.
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("object");
    ts.addType("robot", "object");
    wm.addObject("uav1", "robot");
    wm.registerPredicate("armed", {"robot"});
    wm.registerPredicate("ready", {"robot"});

    wm.registerAction("arm", {"?r"}, {"robot"},
        /*pre*/{}, /*neg_pre*/{}, /*add*/{"(armed ?r)"}, /*del*/{});
    wm.registerAction("prep", {"?r"}, {"robot"},
        /*pre*/{}, /*neg_pre*/{"(armed ?r)"}, /*add*/{"(ready ?r)"}, /*del*/{});

    ame::ActionRegistry reg;
    reg.registerAction("arm", "ArmNode");
    reg.registerAction("prep", "PrepNode");

    ame::PlanCompiler compiler;
    // prep before arm: a later add(armed) conflicts with prep's earlier not-armed.
    std::vector<ame::PlanStep> plan = {
        {findAction(wm, "prep(uav1)")},
        {findAction(wm, "arm(uav1)")},
    };
    auto xml = compiler.compile(plan, wm, reg);
    EXPECT_EQ(xml.find("<Parallel"), std::string::npos);
}

TEST(PlanCompilerNegPre, DelSupportsLaterNegPreStaysSequential) {
    // Step 0 deletes (safe x); step 1 needs (not safe x). Causal support links
    // them, keeping a single ordered flow.
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("object");
    ts.addType("robot", "object");
    wm.addObject("uav1", "robot");
    wm.registerPredicate("safe", {"robot"});
    wm.registerPredicate("launched", {"robot"});

    wm.registerAction("make_unsafe", {"?r"}, {"robot"},
        /*pre*/{"(safe ?r)"}, /*neg_pre*/{}, /*add*/{}, /*del*/{"(safe ?r)"});
    wm.registerAction("launch", {"?r"}, {"robot"},
        /*pre*/{}, /*neg_pre*/{"(safe ?r)"}, /*add*/{"(launched ?r)"}, /*del*/{});

    ame::ActionRegistry reg;
    reg.registerAction("make_unsafe", "UnsafeNode");
    reg.registerAction("launch", "LaunchNode");

    ame::PlanCompiler compiler;
    std::vector<ame::PlanStep> plan = {
        {findAction(wm, "make_unsafe(uav1)")},
        {findAction(wm, "launch(uav1)")},
    };
    auto xml = compiler.compile(plan, wm, reg);
    EXPECT_EQ(xml.find("<Parallel"), std::string::npos);
    EXPECT_NE(xml.find("<Sequence>"), std::string::npos);
}

TEST(PlanCompilerExtensions, DisjunctTaggedActionResolvesBaseImplementation) {
    const char* domain = R"(
(define (domain disj)
  (:requirements :strips :typing)
  (:types robot - object)
  (:predicates (keyed ?r - robot) (carded ?r - robot) (in ?r - robot))
  (:action enter
    :parameters (?r - robot)
    :precondition (or (keyed ?r) (carded ?r))
    :effect (in ?r)
  )
)
)";
    const char* problem = R"(
(define (problem disj-1)
  (:domain disj)
  (:objects uav1 - robot)
  (:init (carded uav1))
  (:goal (in uav1))
)
)";
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(domain, problem, wm);

    ame::ActionRegistry reg;
    reg.registerAction("enter", "EnterNode");  // base name only

    // Find a disjunct-tagged ground action (e.g. enter#0(uav1) / enter#1(uav1)).
    unsigned idx = 0;
    bool found = false;
    for (unsigned i = 0; i < wm.numGroundActions(); ++i) {
        if (wm.groundActions()[i].signature.rfind("enter#", 0) == 0) {
            idx = i;
            found = true;
            break;
        }
    }
    ASSERT_TRUE(found);

    ame::PlanCompiler compiler;
    std::vector<ame::PlanStep> plan = {{idx}};
    // actionName strips the "#k" tag, so the base impl resolves and compile
    // does not throw "Unregistered action".
    std::string xml;
    ASSERT_NO_THROW(xml = compiler.compileSequential(plan, wm, reg));
    EXPECT_NE(xml.find("EnterNode"), std::string::npos);
}

// Stub mode is the authoring/devenv preview: an action with no BT binding
// compiles to a SimulatedAction that still carries the state contract, so the
// preview shows the plan's real precondition and effect structure.
TEST(PlanCompiler, StubModeCompilesUnregisteredAction) {
    auto wm = buildUAVDomain();
    ame::ActionRegistry reg;
    reg.registerAction("search", "StubSearchAction");
    ame::PlanCompiler compiler;
    compiler.setStubUnregisteredActions(true);

    std::vector<ame::PlanStep> plan = {
        {findAction(wm, "move(uav1,base,sector_a)")},
    };

    const std::string seq = compiler.compileSequential(plan, wm, reg);
    EXPECT_NE(seq.find("<SimulatedAction"), std::string::npos);
    EXPECT_EQ(seq.find("StubMoveAction"), std::string::npos);
    // Effects still travel with the stub so the preview advances the plan.
    EXPECT_NE(seq.find("ame_add_effects=\"(at uav1 sector_a)\""), std::string::npos);
    EXPECT_NE(seq.find("ame_del_effects=\"(at uav1 base)\""), std::string::npos);

    EXPECT_NO_THROW(compiler.compile(plan, wm, reg));
}

TEST(PlanCompiler, EscapesDynamicXmlAttributeValues) {
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("item");
    const std::string object_name = "a&<>\"'";
    wm.addObject(object_name, "item");
    wm.registerPredicate("done", {"item"});
    wm.registerAction("inspect", {"?x"}, {"item"}, {}, {"(done ?x)"}, {});

    ame::ActionRegistry reg;
    reg.registerAction("inspect", "InspectNode");

    ame::PlanCompiler compiler;
    std::vector<ame::PlanStep> plan = {
        {findAction(wm, "inspect(" + object_name + ")")},
    };

    auto xml = compiler.compileSequential(plan, wm, reg);
    EXPECT_NE(xml.find("name=\"inspect(a&amp;&lt;&gt;&quot;&apos;)\""), std::string::npos);
    EXPECT_NE(xml.find("ame_add_effects=\"(done a&amp;&lt;&gt;&quot;&apos;)\""),
              std::string::npos);
    EXPECT_NE(xml.find("param0=\"a&amp;&lt;&gt;&quot;&apos;\""), std::string::npos);
}

// A negative precondition travels as its own contract attribute, so the action
// node can require it to be false without a separate check node.
TEST(PlanCompilerNegPre, EmitsNegPreconditionAttribute) {
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("object");
    ts.addType("robot", "object");
    wm.addObject("uav1", "robot");
    wm.registerPredicate("armed", {"robot"});
    wm.registerPredicate("safe", {"robot"});
    wm.registerAction("arm", {"?r"}, {"robot"},
        /*pre*/{"(safe ?r)"}, /*neg_pre*/{"(armed ?r)"},
        /*add*/{"(armed ?r)"}, /*del*/{});

    ame::ActionRegistry reg;
    reg.registerAction("arm", "ArmNode");

    ame::PlanCompiler compiler;
    std::vector<ame::PlanStep> plan = {{findAction(wm, "arm(uav1)")}};
    auto xml = compiler.compileSequential(plan, wm, reg);

    EXPECT_NE(xml.find("ame_preconditions=\"(safe uav1)\""), std::string::npos);
    EXPECT_NE(xml.find("ame_neg_preconditions=\"(armed uav1)\""), std::string::npos);
    EXPECT_EQ(xml.find("CheckWorldPredicate"), std::string::npos);
}

TEST(PlanCompilerExtensions, DisjunctiveGoalEmitsFallbackGuard) {
    const char* domain = R"(
(define (domain dgoal)
  (:requirements :strips :typing)
  (:types loc - object)
  (:predicates (at ?x - loc) (linked ?a - loc ?b - loc))
  (:action move
    :parameters (?from - loc ?to - loc)
    :precondition (and (at ?from) (linked ?from ?to))
    :effect (and (at ?to) (not (at ?from)))
  )
)
)";
    const char* problem = R"(
(define (problem dgoal-1)
  (:domain dgoal)
  (:objects l1 l2 l3 - loc)
  (:init (at l1) (linked l1 l2))
  (:goal (or (at l2) (at l3)))
)
)";
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(domain, problem, wm);
    ASSERT_EQ(wm.goalAlternatives().size(), 2u);

    ame::ActionRegistry reg;
    reg.registerAction("move", "MoveNode");

    unsigned mv = findAction(wm, "move(l1,l2)");
    ame::PlanCompiler compiler;
    std::vector<ame::PlanStep> plan = {{mv}};
    std::string xml = compiler.compile(plan, wm, reg);
    EXPECT_NE(xml.find("<ReactiveFallback>"), std::string::npos);
    EXPECT_NE(xml.find("GoalAltCheck"), std::string::npos);
    // One GoalReached per alternative, each carrying that alternative's conjunction.
    EXPECT_EQ(countOccurrences(xml, "<GoalReached"), 2u);
    EXPECT_NE(xml.find("goals=\"(at l2)\""), std::string::npos);
    EXPECT_NE(xml.find("goals=\"(at l3)\""), std::string::npos);
}

// =========================================================================
// Domain-declared confirmed predicates
// =========================================================================

// A gate predicate is declared evidence-bearing in the domain, so the compiler
// puts it in its own list and the action node holds it to observed state --
// while the action's ordinary predicted preconditions still pass.
TEST(ConfirmedPredicates, SplitsPreconditionsByDomainDeclaration) {
    const char* domain = R"(
(define (domain gated)
  (:requirements :strips :typing)
  (:types robot gate - object)
  (:predicates (airborne ?r - robot) (authorised ?g - gate) (struck ?r - robot))
  (:confirmed-predicates authorised)
  (:action strike
    :parameters (?r - robot ?g - gate)
    :precondition (and (airborne ?r) (authorised ?g))
    :effect (struck ?r)
  )
)
)";
    const char* problem = R"(
(define (problem gated-1)
  (:domain gated)
  (:objects uav1 - robot g1 - gate)
  (:init (airborne uav1) (authorised g1))
  (:goal (struck uav1))
)
)";
    ame::WorldModel wm;
    ame::PddlParser::parseFromString(domain, problem, wm);
    EXPECT_TRUE(wm.isConfirmedPredicate("authorised"));
    EXPECT_FALSE(wm.isConfirmedPredicate("airborne"));
    EXPECT_TRUE(wm.isConfirmedFact("(authorised g1)"));
    EXPECT_FALSE(wm.isConfirmedFact("(airborne uav1)"));

    ame::ActionRegistry reg;
    reg.registerAction("strike", "StubMoveAction");
    ame::PlanCompiler compiler;
    std::vector<ame::PlanStep> plan = {{findAction(wm, "strike(uav1,g1)")}};
    const auto xml = compiler.compileSequential(plan, wm, reg);

    EXPECT_NE(xml.find("ame_preconditions=\"(airborne uav1)\""), std::string::npos);
    EXPECT_NE(xml.find("ame_confirmed_preconditions=\"(authorised g1)\""),
              std::string::npos);
}

// The gate is fail-closed: a believed (authorised) does not let the action run,
// even though the action's other precondition is only believed too.
TEST(ConfirmedPredicates, GateRejectsBelievedButAcceptsObserved) {
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("object");
    ts.addType("robot", "object");
    ts.addType("gate", "object");
    wm.addObject("uav1", "robot");
    wm.addObject("g1", "gate");
    wm.registerPredicate("airborne", {"robot"});
    wm.registerPredicate("authorised", {"gate"});
    wm.registerPredicate("struck", {"robot"});

    // Airborne is predicted by an earlier takeoff; the gate is operator state.
    wm.setFact("(airborne uav1)", true, "planner_init", ame::FactAuthority::BELIEVED);
    wm.setFact("(authorised g1)", true, "planner_init", ame::FactAuthority::BELIEVED);

    BT::BehaviorTreeFactory factory;
    registerCoreNodes(factory);
    factory.registerNodeType<StubAction>("StrikeNode");

    const std::string xml =
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\">"
        "<StrikeNode name=\"strike\""
        " ame_preconditions=\"(airborne uav1)\""
        " ame_confirmed_preconditions=\"(authorised g1)\""
        " ame_add_effects=\"(struck uav1)\"/>"
        "</BehaviorTree></root>";

    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);
    EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
    EXPECT_FALSE(wm.getFact("(struck uav1)"));

    // Operator authorisation observed: the gate opens, airborne stays believed.
    wm.setFact("(authorised g1)", true, "operator", ame::FactAuthority::CONFIRMED);
    auto authorised_tree = factory.createTreeFromText(xml);
    authorised_tree.rootBlackboard()->set("world_model", &wm);
    EXPECT_EQ(authorised_tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(wm.getFact("(struck uav1)"));
}

// A negative precondition is enforced from its own attribute.
TEST(ConfirmedPredicates, NegativePreconditionBlocksWhenFactHolds) {
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("object");
    ts.addType("robot", "object");
    wm.addObject("uav1", "robot");
    wm.registerPredicate("armed", {"robot"});
    wm.registerPredicate("safe", {"robot"});

    wm.setFact("(armed uav1)", true);

    BT::BehaviorTreeFactory factory;
    registerCoreNodes(factory);
    factory.registerNodeType<StubAction>("ArmNode");

    const std::string xml =
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\">"
        "<ArmNode name=\"arm\" ame_neg_preconditions=\"(armed uav1)\""
        " ame_add_effects=\"(safe uav1)\"/>"
        "</BehaviorTree></root>";

    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);
    EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::FAILURE);

    wm.setFact("(armed uav1)", false);
    auto disarmed_tree = factory.createTreeFromText(xml);
    disarmed_tree.rootBlackboard()->set("world_model", &wm);
    EXPECT_EQ(disarmed_tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(wm.getFact("(safe uav1)"));
}
