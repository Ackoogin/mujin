#include <gtest/gtest.h>
#include "ame/action_registry.h"
#include "ame/detail/escape.h"

#include <filesystem>
#include <fstream>

namespace {

std::filesystem::path writeTempActionXml(const std::string& filename,
                                         const std::string& contents) {
    const auto path = std::filesystem::temp_directory_path() / filename;
    std::ofstream out(path, std::ios::out | std::ios::binary | std::ios::trunc);
    out << contents;
    return path;
}

} // namespace

TEST(ActionRegistry, RegisterSimpleAction) {
    ame::ActionRegistry reg;
    reg.registerAction("move", "MoveAction");

    EXPECT_TRUE(reg.hasAction("move"));
    EXPECT_FALSE(reg.hasAction("fly"));
}

TEST(ActionRegistry, RegisteredNamesReturnsSortedActionNames) {
    ame::ActionRegistry reg;
    reg.registerAction("search", "SearchAction");
    reg.registerAction("move", "MoveAction");

    EXPECT_EQ(reg.registeredNames(),
              std::vector<std::string>({"move", "search"}));
}

TEST(ActionRegistry, ResolveSimpleAction) {
    ame::ActionRegistry reg;
    reg.registerAction("move", "MoveAction");

    auto impl = reg.resolve("move", {"uav1", "base", "sector_a"});
    EXPECT_EQ(impl.xml, R"(<MoveAction param0="uav1" param1="base" param2="sector_a"/>)");
    EXPECT_EQ(impl.node_type, "MoveAction");
    EXPECT_FALSE(impl.is_subtree);
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

TEST(EscapeUtilities, XmlAttrEscapeCoversReservedAttributeCharacters) {
    EXPECT_EQ(ame::detail::xmlAttrEscape("&<>\"'"),
              "&amp;&lt;&gt;&quot;&apos;");
}

TEST(ActionRegistry, ResolveEscapesSimpleActionParams) {
    ame::ActionRegistry reg;
    reg.registerAction("inspect", "InspectAction");

    auto impl = reg.resolve("inspect", {"a&<>\"'"});
    EXPECT_EQ(impl.xml,
              R"(<InspectAction param0="a&amp;&lt;&gt;&quot;&apos;"/>)");
}

TEST(ActionRegistry, RegisterSubTreeTemplate) {
    ame::ActionRegistry reg;
    reg.registerActionSubTree("search",
        R"(<Sequence><FlyTo target="{param1}"/><RunSensor area="{param1}"/></Sequence>)");

    auto impl = reg.resolve("search", {"uav1", "sector_a"});
    EXPECT_EQ(impl.xml,
        R"(<Sequence><FlyTo target="sector_a"/><RunSensor area="sector_a"/></Sequence>)");
    EXPECT_TRUE(impl.is_subtree);
    EXPECT_TRUE(impl.node_type.empty());
}

TEST(ActionRegistry, RegisterSubTreeTemplateEscapesParams) {
    ame::ActionRegistry reg;
    reg.registerActionSubTree("inspect",
        R"(<Sequence><Inspect target="{param0}"/></Sequence>)");

    auto impl = reg.resolve("inspect", {"a&<>\"'"});
    EXPECT_EQ(impl.xml,
        R"(<Sequence><Inspect target="a&amp;&lt;&gt;&quot;&apos;"/></Sequence>)");
}

TEST(ActionRegistry, RegisterActionFileResolvesWithParamSubstitution) {
    const auto path = writeTempActionXml("ame_action_registry_valid.xml",
        R"(<Sequence><FlyTo target="{param1}"/><RunSensor agent="{param0}" area="{param1}"/></Sequence>)");

    ame::ActionRegistry reg;
    reg.registerActionFile("search", path.string(), true);

    auto impl = reg.resolve("search", {"uav1", "sector_a"});
    EXPECT_EQ(impl.xml,
        R"(<Sequence><FlyTo target="sector_a"/><RunSensor agent="uav1" area="sector_a"/></Sequence>)");
    EXPECT_TRUE(impl.reactive);

    std::filesystem::remove(path);
}

TEST(ActionRegistry, RegisterActionFileMissingFileThrowsAtRegistration) {
    const auto path = std::filesystem::temp_directory_path() /
        "ame_action_registry_missing.xml";
    std::filesystem::remove(path);

    ame::ActionRegistry reg;
    EXPECT_THROW(reg.registerActionFile("search", path.string()), std::runtime_error);
}

TEST(ActionRegistry, RegisterActionFileMalformedXmlThrowsAtRegistration) {
    const auto path = writeTempActionXml("ame_action_registry_malformed.xml",
        R"(<Sequence><FlyTo target="{param1}"></Sequence>)");

    ame::ActionRegistry reg;
    EXPECT_THROW(reg.registerActionFile("search", path.string()), std::runtime_error);

    std::filesystem::remove(path);
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
