#include <gtest/gtest.h>
#include "ame/action_registry.h"

TEST(ActionRegistry, RegisterSimpleAction) {
    ame::ActionRegistry reg;
    reg.registerAction("move", "MoveAction");

    EXPECT_TRUE(reg.hasAction("move"));
    EXPECT_FALSE(reg.hasAction("fly"));
}

TEST(ActionRegistry, ResolveSimpleAction) {
    ame::ActionRegistry reg;
    reg.registerAction("move", "MoveAction");

    auto impl = reg.resolve("move", {"uav1", "base", "sector_a"});
    EXPECT_EQ(impl.xml, R"(<MoveAction param0="uav1" param1="base" param2="sector_a"/>)");
    EXPECT_FALSE(impl.reactive);
    EXPECT_EQ(impl.param_bindings.size(), 3u);
    EXPECT_EQ(impl.param_bindings[0], "uav1");
}

TEST(ActionRegistry, ResolveSimpleActionNoParams) {
    ame::ActionRegistry reg;
    reg.registerAction("noop", "NoOpAction");

    auto impl = reg.resolve("noop", {});
    EXPECT_EQ(impl.xml, "<NoOpAction/>");
}

TEST(ActionRegistry, RegisterSubTreeTemplate) {
    ame::ActionRegistry reg;
    reg.registerActionSubTree("search",
        R"(<Sequence><FlyTo target="{param1}"/><RunSensor area="{param1}"/></Sequence>)");

    auto impl = reg.resolve("search", {"uav1", "sector_a"});
    EXPECT_EQ(impl.xml,
        R"(<Sequence><FlyTo target="sector_a"/><RunSensor area="sector_a"/></Sequence>)");
}

TEST(ActionRegistry, ReactiveFlag) {
    ame::ActionRegistry reg;
    reg.registerAction("search", "SearchAction", true);
    reg.registerAction("move", "MoveAction", false);

    EXPECT_TRUE(reg.isReactive("search"));
    EXPECT_FALSE(reg.isReactive("move"));

    auto impl = reg.resolve("search", {"uav1", "sector_a"});
    EXPECT_TRUE(impl.reactive);
}

TEST(ActionRegistry, UnknownActionThrows) {
    ame::ActionRegistry reg;
    EXPECT_THROW(reg.resolve("nonexistent", {}), std::runtime_error);
    EXPECT_THROW(reg.isReactive("nonexistent"), std::runtime_error);
}

TEST(ActionRegistry, OverwriteRegistration) {
    ame::ActionRegistry reg;
    reg.registerAction("move", "MoveActionV1");
    reg.registerAction("move", "MoveActionV2");

    auto impl = reg.resolve("move", {});
    EXPECT_EQ(impl.xml, "<MoveActionV2/>");
}
