#include <gtest/gtest.h>
#include "ame_ros2/planner_node.hpp"
#include "ame_ros2/world_model_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>

class PlannerNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        wm_node_ = std::make_shared<ame_ros2::WorldModelNode>();
        pl_node_ = std::make_shared<ame_ros2::PlannerNode>();

        // In-process mode: inject canonical WorldModel pointer
        pl_node_->setInProcessWorldModel(&wm_node_->worldModel());

        // Register action mappings
        pl_node_->actionRegistry().registerAction("move",     "StubMoveAction");
        pl_node_->actionRegistry().registerAction("search",   "StubSearchAction");
        pl_node_->actionRegistry().registerAction("classify", "StubClassifyAction");

        // Lifecycle: configure + activate both nodes
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

        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(wm_node_->get_node_base_interface());
        executor_->add_node(pl_node_->get_node_base_interface());

        // Build the domain in WorldModelNode's WorldModel
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
        executor_.reset();
        wm_node_.reset();
        pl_node_.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<ame_ros2::WorldModelNode> wm_node_;
    std::shared_ptr<ame_ros2::PlannerNode>    pl_node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(PlannerNodeTest, PlanActionSucceeds) {
    auto action_client = rclcpp_action::create_client<ame_ros2::action::Plan>(
        pl_node_, "/planner_node/plan");

    ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(3)));

    auto goal = ame_ros2::action::Plan::Goal();
    goal.goal_fluents = {"(searched sector_a)", "(classified sector_a)"};
    goal.replan = false;

    std::shared_ptr<ame_ros2::action::Plan::Result const> result_ptr;
    bool done = false;

    auto send_goal_opts = rclcpp_action::Client<ame_ros2::action::Plan>::SendGoalOptions();
    send_goal_opts.result_callback = [&](auto wrapped) {
        result_ptr = wrapped.result;
        done = true;
    };

    action_client->async_send_goal(goal, send_goal_opts);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (!done && std::chrono::steady_clock::now() < deadline) {
        executor_->spin_some(std::chrono::milliseconds(50));
    }

    ASSERT_TRUE(done) << "Plan action did not complete";
    ASSERT_NE(result_ptr, nullptr);
    EXPECT_TRUE(result_ptr->success);
    EXPECT_FALSE(result_ptr->bt_xml.empty());
    EXPECT_FALSE(result_ptr->plan_actions.empty());
}
