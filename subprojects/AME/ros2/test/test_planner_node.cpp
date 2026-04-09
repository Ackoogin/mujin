#include <gtest/gtest.h>
#include "ame_ros2/planner_node.hpp"
#include "ame_ros2/world_model_node.hpp"
#include <ame/action_registry.h>
#include <ame/planner_component.h>
#include <ame/world_model.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

class PlannerNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        wm_node_ = std::make_shared<ame_ros2::WorldModelNode>();
        pl_node_ = std::make_shared<ame_ros2::PlannerNode>();

        pl_node_->setInProcessWorldModel(&wm_node_->worldModel());

        pl_node_->actionRegistry().registerAction("move",     "StubMoveAction");
        pl_node_->actionRegistry().registerAction("search",   "StubSearchAction");
        pl_node_->actionRegistry().registerAction("classify", "StubClassifyAction");

        ASSERT_EQ(
            wm_node_->on_configure(rclcpp_lifecycle::State{}),
            ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);
        ASSERT_EQ(
            wm_node_->on_activate(rclcpp_lifecycle::State{}),
            ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);
        ASSERT_EQ(
            pl_node_->on_configure(rclcpp_lifecycle::State{}),
            ame_ros2::PlannerNode::CallbackReturn::SUCCESS);
        ASSERT_EQ(
            pl_node_->on_activate(rclcpp_lifecycle::State{}),
            ame_ros2::PlannerNode::CallbackReturn::SUCCESS);

        auto& wm = wm_node_->worldModel();
        wm.typeSystem().addType("object");
        wm.typeSystem().addType("location", "object");
        wm.typeSystem().addType("sector", "location");
        wm.typeSystem().addType("robot", "object");
        wm.addObject("uav1", "robot");
        wm.addObject("base", "location");
        wm.addObject("sector_a", "sector");
        wm.registerPredicate("at", {"robot", "location"});
        wm.registerPredicate("searched", {"sector"});
        wm.registerPredicate("classified", {"sector"});
        wm.registerAction("move",
            {"?r","?from","?to"}, {"robot","location","location"},
            {"(at ?r ?from)"}, {"(at ?r ?to)"}, {"(at ?r ?from)"});
        wm.registerAction("search",
            {"?r","?s"}, {"robot","sector"},
            {"(at ?r ?s)"}, {"(searched ?s)"}, {});
        wm.registerAction("classify",
            {"?r","?s"}, {"robot","sector"},
            {"(at ?r ?s)","(searched ?s)"}, {"(classified ?s)"}, {});
        wm.setFact("(at uav1 base)", true, "planner_init");
    }

    void TearDown() override {
        wm_node_.reset();
        pl_node_.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<ame_ros2::WorldModelNode> wm_node_;
    std::shared_ptr<ame_ros2::PlannerNode>    pl_node_;
};

TEST_F(PlannerNodeTest, PlanActionSucceeds) {
    // Planning is now a direct synchronous PCL service call (action server removed).
    auto result = pl_node_->plannerComponent().solveGoal({
        "(searched sector_a)",
        "(classified sector_a)",
    });

    ASSERT_TRUE(result.success) << result.error_msg;
    EXPECT_FALSE(result.bt_xml.empty());
    EXPECT_FALSE(result.plan_actions.empty());
    EXPECT_GE(result.plan_actions.size(), 3u);  // at least: move, search, classify
}
