#include <gtest/gtest.h>
#include "ame/world_model.h"
#include "ame/bt_nodes/check_world_predicate.h"
#include "ame/bt_nodes/goal_reached.h"
#include "ame/bt_nodes/planned_action.h"
#include "ame/bt_nodes/simulated_action.h"
#include "ame/world_state_access.h"

#include <behaviortree_cpp/bt_factory.h>

#include <unordered_map>

// Helper: set up a WorldModel with a simple domain and register BT nodes
static ame::WorldModel makeSimpleWM() {
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("location");
    ts.addType("robot");
    wm.addObject("uav1", "robot");
    wm.addObject("base", "location");
    wm.registerPredicate("at", {"robot", "location"});
    wm.registerPredicate("searched", {"location"});
    return wm;
}

static BT::BehaviorTreeFactory makeFactory() {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ame::CheckWorldPredicate>("CheckWorldPredicate");
    factory.registerNodeType<ame::GoalReached>("GoalReached");
    factory.registerNodeType<ame::PlannedAction>("PlannedAction");
    factory.registerNodeType<ame::SimulatedAction>("SimulatedAction");
    return factory;
}

static BT::Tree createTreeWithWorldModel(BT::BehaviorTreeFactory& factory,
                                         const char* xml,
                                         ame::WorldModel& wm) {
    auto blackboard = BT::Blackboard::create();
    blackboard->set("world_model", &wm);
    return factory.createTreeFromText(xml, blackboard);
}

TEST(BTNodes, CheckWorldPredicate_SuccessWhenTrue) {
    auto wm = makeSimpleWM();
    wm.setFact("(at uav1 base)", true);

    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <CheckWorldPredicate predicate="(at uav1 base)"/>
            </BehaviorTree>
        </root>
    )xml";
    auto tree = createTreeWithWorldModel(factory, xml, wm);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(BTNodes, CheckWorldPredicate_FailureWhenFalse) {
    auto wm = makeSimpleWM();

    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <CheckWorldPredicate predicate="(at uav1 base)"/>
            </BehaviorTree>
        </root>
    )xml";
    auto tree = createTreeWithWorldModel(factory, xml, wm);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(BTNodes, CheckWorldPredicate_ExpectedFalse) {
    auto wm = makeSimpleWM();

    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <CheckWorldPredicate predicate="(at uav1 base)" expected="false"/>
            </BehaviorTree>
        </root>
    )xml";
    auto tree = createTreeWithWorldModel(factory, xml, wm);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(BTNodes, CheckWorldPredicate_RequiredAuthorityConfirmed_FailsOnBelieved) {
    auto wm = makeSimpleWM();
    // Set fact with default BELIEVED authority
    wm.setFact("(at uav1 base)", true, "plan", ame::FactAuthority::BELIEVED);

    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <CheckWorldPredicate predicate="(at uav1 base)" required_authority="confirmed"/>
            </BehaviorTree>
        </root>
    )xml";
    auto tree = createTreeWithWorldModel(factory, xml, wm);

    // Fact is true but only BELIEVED -- should fail
    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(BTNodes, CheckWorldPredicate_RequiredAuthorityConfirmed_SucceedsOnConfirmed) {
    auto wm = makeSimpleWM();
    wm.setFact("(at uav1 base)", true, "perception", ame::FactAuthority::CONFIRMED);

    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <CheckWorldPredicate predicate="(at uav1 base)" required_authority="confirmed"/>
            </BehaviorTree>
        </root>
    )xml";
    auto tree = createTreeWithWorldModel(factory, xml, wm);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(BTNodes, CheckWorldPredicate_RequiredAuthorityAny_AcceptsBelieved) {
    auto wm = makeSimpleWM();
    wm.setFact("(at uav1 base)", true, "plan", ame::FactAuthority::BELIEVED);

    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <CheckWorldPredicate predicate="(at uav1 base)" required_authority="any"/>
            </BehaviorTree>
        </root>
    )xml";
    auto tree = createTreeWithWorldModel(factory, xml, wm);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(BTNodes, PlannedActionAppliesAddEffect) {
    auto wm = makeSimpleWM();
    EXPECT_FALSE(wm.getFact("(searched base)"));

    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <SimulatedAction name="search(uav1,base)"
                    ame_add_effects="(searched base)"/>
            </BehaviorTree>
        </root>
    )xml";
    auto tree = createTreeWithWorldModel(factory, xml, wm);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(wm.getFact("(searched base)"));
}

TEST(BTNodes, PlannedActionAppliesDeleteEffect) {
    auto wm = makeSimpleWM();
    wm.setFact("(at uav1 base)", true);

    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <SimulatedAction name="leave(uav1,base)"
                    ame_del_effects="(at uav1 base)"/>
            </BehaviorTree>
        </root>
    )xml";
    auto tree = createTreeWithWorldModel(factory, xml, wm);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_FALSE(wm.getFact("(at uav1 base)"));
}

TEST(BTNodes, PlannedActionChecksPreconditionThenAppliesEffect) {
    auto wm = makeSimpleWM();
    wm.setFact("(at uav1 base)", true);

    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <SimulatedAction name="search(uav1,base)"
                    ame_preconditions="(at uav1 base)"
                    ame_add_effects="(searched base)"/>
            </BehaviorTree>
        </root>
    )xml";
    auto tree = createTreeWithWorldModel(factory, xml, wm);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(wm.getFact("(searched base)"));
}

TEST(BTNodes, PlannedActionFailsOnPrecondition) {
    auto wm = makeSimpleWM();
    EXPECT_FALSE(wm.getFact("(searched base)"));

    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <SimulatedAction name="search(uav1,base)"
                    ame_preconditions="(at uav1 base)"
                    ame_add_effects="(searched base)"/>
            </BehaviorTree>
        </root>
    )xml";
    auto tree = createTreeWithWorldModel(factory, xml, wm);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
    EXPECT_FALSE(wm.getFact("(searched base)"));
}

TEST(BTNodes, PlannedActionAppliesAddAndDeleteEffects) {
    auto wm = makeSimpleWM();
    wm.setFact("(at uav1 base)", true);

    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <SimulatedAction name="search-and-leave(uav1,base)"
                    ame_preconditions="(at uav1 base)"
                    ame_add_effects="(searched base)"
                    ame_del_effects="(at uav1 base)"/>
            </BehaviorTree>
        </root>
    )xml";
    auto tree = createTreeWithWorldModel(factory, xml, wm);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(wm.getFact("(searched base)"));
    EXPECT_FALSE(wm.getFact("(at uav1 base)"));
}

class RecordingWorldState : public ame::IWorldStateAccess {
public:
    bool getFact(const std::string& key) override { return facts[key]; }

    bool setFact(const std::string& key,
                 bool value,
                 const std::string& source) override {
        facts[key] = value;
        last_source = source;
        return writes_succeed;
    }

    std::unordered_map<std::string, bool> facts;
    std::string last_source;
    bool writes_succeed = true;
};

TEST(BTNodes, PlannedActionUsesWorldStateSeam) {
    RecordingWorldState world_state;
    world_state.facts["ready(uav1)"] = true;
    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <SimulatedAction name="work(uav1)"
                    ame_preconditions="ready(uav1)"
                    ame_add_effects="done(uav1)"
                    ame_del_effects="ready(uav1)"/>
            </BehaviorTree>
        </root>
    )xml";
    auto blackboard = BT::Blackboard::create();
    blackboard->set<ame::IWorldStateAccess*>("world_state", &world_state);
    auto tree = factory.createTreeFromText(xml, blackboard);

    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(world_state.facts["done(uav1)"]);
    EXPECT_FALSE(world_state.facts["ready(uav1)"]);
    EXPECT_EQ(world_state.last_source, "PlannedAction:work(uav1)");
}

TEST(BTNodes, ReactiveActionRechecksPreconditionsWhileRunning) {
    RecordingWorldState world_state;
    world_state.facts["ready(uav1)"] = true;
    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <SimulatedAction name="work(uav1)" ticks="2"
                    ame_reactive="true"
                    ame_preconditions="ready(uav1)"
                    ame_add_effects="done(uav1)"/>
            </BehaviorTree>
        </root>
    )xml";
    auto blackboard = BT::Blackboard::create();
    blackboard->set<ame::IWorldStateAccess*>("world_state", &world_state);
    auto tree = factory.createTreeFromText(xml, blackboard);

    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    world_state.facts["ready(uav1)"] = false;
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
    EXPECT_FALSE(world_state.facts["done(uav1)"]);
}

TEST(BTNodes, FailedSimulatedActionDoesNotCommitEffects) {
    RecordingWorldState world_state;
    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <SimulatedAction name="work(uav1)" success="false"
                    ame_add_effects="done(uav1)"/>
            </BehaviorTree>
        </root>
    )xml";
    auto blackboard = BT::Blackboard::create();
    blackboard->set<ame::IWorldStateAccess*>("world_state", &world_state);
    auto tree = factory.createTreeFromText(xml, blackboard);

    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
    EXPECT_FALSE(world_state.facts["done(uav1)"]);
    EXPECT_TRUE(world_state.last_source.empty());
}

TEST(BTNodes, PlannedActionDecoratorOwnsSubtreeContract) {
    RecordingWorldState world_state;
    world_state.facts["ready(uav1)"] = true;
    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <PlannedAction name="work(uav1)"
                    ame_preconditions="ready(uav1)"
                    ame_add_effects="done(uav1)">
                    <AlwaysSuccess/>
                </PlannedAction>
            </BehaviorTree>
        </root>
    )xml";
    auto blackboard = BT::Blackboard::create();
    blackboard->set<ame::IWorldStateAccess*>("world_state", &world_state);
    auto tree = factory.createTreeFromText(xml, blackboard);

    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(world_state.facts["done(uav1)"]);
}

TEST(BTNodes, PlannedActionDecoratorDoesNotCommitFailedSubtree) {
    RecordingWorldState world_state;
    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <PlannedAction name="work(uav1)" ame_add_effects="done(uav1)">
                    <AlwaysFailure/>
                </PlannedAction>
            </BehaviorTree>
        </root>
    )xml";
    auto blackboard = BT::Blackboard::create();
    blackboard->set<ame::IWorldStateAccess*>("world_state", &world_state);
    auto tree = factory.createTreeFromText(xml, blackboard);

    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
    EXPECT_FALSE(world_state.facts["done(uav1)"]);
}

TEST(BTNodes, GoalReachedUsesWorldStateSeam) {
    RecordingWorldState world_state;
    world_state.facts["done(a)"] = true;
    world_state.facts["done(b)"] = true;
    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <GoalReached goals="done(a);done(b)"/>
            </BehaviorTree>
        </root>
    )xml";
    auto blackboard = BT::Blackboard::create();
    blackboard->set<ame::IWorldStateAccess*>("world_state", &world_state);
    auto tree = factory.createTreeFromText(xml, blackboard);

    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    world_state.facts["done(b)"] = false;
    tree.haltTree();
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}
