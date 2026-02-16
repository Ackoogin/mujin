#include <gtest/gtest.h>
#include "mujin/world_model.h"
#include "mujin/bt_nodes/check_world_predicate.h"
#include "mujin/bt_nodes/set_world_predicate.h"

#include <behaviortree_cpp/bt_factory.h>

// Helper: set up a WorldModel with a simple domain and register BT nodes
static mujin::WorldModel makeSimpleWM() {
    mujin::WorldModel wm;
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
    factory.registerNodeType<mujin::CheckWorldPredicate>("CheckWorldPredicate");
    factory.registerNodeType<mujin::SetWorldPredicate>("SetWorldPredicate");
    return factory;
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
    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

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
    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

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
    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(BTNodes, SetWorldPredicate_SetsTrue) {
    auto wm = makeSimpleWM();
    EXPECT_FALSE(wm.getFact("(searched base)"));

    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <SetWorldPredicate predicate="(searched base)" value="true"/>
            </BehaviorTree>
        </root>
    )xml";
    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(wm.getFact("(searched base)"));
}

TEST(BTNodes, SetWorldPredicate_SetsFalse) {
    auto wm = makeSimpleWM();
    wm.setFact("(at uav1 base)", true);

    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <SetWorldPredicate predicate="(at uav1 base)" value="false"/>
            </BehaviorTree>
        </root>
    )xml";
    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_FALSE(wm.getFact("(at uav1 base)"));
}

TEST(BTNodes, SequenceCheckThenSet) {
    auto wm = makeSimpleWM();
    wm.setFact("(at uav1 base)", true);

    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <Sequence>
                    <CheckWorldPredicate predicate="(at uav1 base)"/>
                    <SetWorldPredicate predicate="(searched base)" value="true"/>
                </Sequence>
            </BehaviorTree>
        </root>
    )xml";
    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(wm.getFact("(searched base)"));
}

TEST(BTNodes, SequenceFailsOnPrecondition) {
    auto wm = makeSimpleWM();
    EXPECT_FALSE(wm.getFact("(searched base)"));

    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <Sequence>
                    <CheckWorldPredicate predicate="(at uav1 base)"/>
                    <SetWorldPredicate predicate="(searched base)" value="true"/>
                </Sequence>
            </BehaviorTree>
        </root>
    )xml";
    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
    EXPECT_FALSE(wm.getFact("(searched base)"));
}

TEST(BTNodes, MultipleActionsInSequence) {
    auto wm = makeSimpleWM();
    wm.setFact("(at uav1 base)", true);

    auto factory = makeFactory();
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <Sequence>
                    <CheckWorldPredicate predicate="(at uav1 base)"/>
                    <SetWorldPredicate predicate="(searched base)" value="true"/>
                    <SetWorldPredicate predicate="(at uav1 base)" value="false"/>
                </Sequence>
            </BehaviorTree>
        </root>
    )xml";
    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(wm.getFact("(searched base)"));
    EXPECT_FALSE(wm.getFact("(at uav1 base)"));
}
