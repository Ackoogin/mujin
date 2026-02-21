/**
 * Full in-process pipeline test:
 *   WorldModel + Planner + Executor — no IPC, direct pointer wiring.
 *   Mirrors the existing tests/test_e2e_pipeline.cpp but via ROS2 nodes.
 */
#include <gtest/gtest.h>

#include "mujin_ros2/world_model_node.hpp"
#include "mujin_ros2/planner_node.hpp"
#include "mujin_ros2/executor_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <behaviortree_cpp/action_node.h>

#include <chrono>

// Stub action nodes (same signatures as main.cpp)
class StubMoveAction : public BT::SyncActionNode {
public:
    StubMoveAction(const std::string& n, const BT::NodeConfiguration& c) : BT::SyncActionNode(n, c) {}
    BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("param0"),
                 BT::InputPort<std::string>("param1"),
                 BT::InputPort<std::string>("param2") };
    }
};
class StubSearchAction : public BT::SyncActionNode {
public:
    StubSearchAction(const std::string& n, const BT::NodeConfiguration& c) : BT::SyncActionNode(n, c) {}
    BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("param0"),
                 BT::InputPort<std::string>("param1") };
    }
};
class StubClassifyAction : public BT::SyncActionNode {
public:
    StubClassifyAction(const std::string& n, const BT::NodeConfiguration& c) : BT::SyncActionNode(n, c) {}
    BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("param0"),
                 BT::InputPort<std::string>("param1") };
    }
};

class FullPipelineTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);

        wm_node_ = std::make_shared<mujin_ros2::WorldModelNode>();
        pl_node_ = std::make_shared<mujin_ros2::PlannerNode>();
        ex_node_ = std::make_shared<mujin_ros2::ExecutorNode>();

        pl_node_->setInProcessWorldModel(&wm_node_->worldModel());
        ex_node_->setInProcessWorldModel(&wm_node_->worldModel());

        pl_node_->actionRegistry().registerAction("move",     "StubMoveAction");
        pl_node_->actionRegistry().registerAction("search",   "StubSearchAction");
        pl_node_->actionRegistry().registerAction("classify", "StubClassifyAction");

        ex_node_->factory().registerNodeType<StubMoveAction>("StubMoveAction");
        ex_node_->factory().registerNodeType<StubSearchAction>("StubSearchAction");
        ex_node_->factory().registerNodeType<StubClassifyAction>("StubClassifyAction");

        for (auto* n : {
                static_cast<rclcpp_lifecycle::LifecycleNode*>(wm_node_.get()),
                static_cast<rclcpp_lifecycle::LifecycleNode*>(pl_node_.get()),
                static_cast<rclcpp_lifecycle::LifecycleNode*>(ex_node_.get()) })
        {
            n->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
            n->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        }

        executor_.add_node(wm_node_->get_node_base_interface());
        executor_.add_node(pl_node_->get_node_base_interface());
        executor_.add_node(ex_node_->get_node_base_interface());

        // Populate WorldModel with UAV search domain
        auto& wm = wm_node_->worldModel();
        wm.typeSystem().addType("object");
        wm.typeSystem().addType("location", "object");
        wm.typeSystem().addType("sector", "location");
        wm.typeSystem().addType("robot", "object");
        wm.addObject("uav1", "robot");
        wm.addObject("base", "location");
        wm.addObject("sector_a", "sector");
        wm.registerPredicate("at",         {"robot", "location"});
        wm.registerPredicate("searched",   {"sector"});
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
        ex_node_.reset();
        pl_node_.reset();
        wm_node_.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<mujin_ros2::WorldModelNode> wm_node_;
    std::shared_ptr<mujin_ros2::PlannerNode>    pl_node_;
    std::shared_ptr<mujin_ros2::ExecutorNode>   ex_node_;
    rclcpp::executors::SingleThreadedExecutor   executor_;
};

TEST_F(FullPipelineTest, PlanAndExecuteReachesGoal) {
    const std::vector<std::string> goal_fluents = {
        "(searched sector_a)", "(classified sector_a)"
    };

    // 1. Send Plan action
    auto action_client = rclcpp_action::create_client<mujin_ros2::action::Plan>(
        pl_node_, "/mujin/plan");
    ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(3)));

    auto goal_msg = mujin_ros2::action::Plan::Goal();
    goal_msg.goal_fluents = goal_fluents;

    std::shared_ptr<mujin_ros2::action::Plan::Result const> plan_result;
    bool plan_done = false;

    auto opts = rclcpp_action::Client<mujin_ros2::action::Plan>::SendGoalOptions();
    opts.result_callback = [&](auto wrapped) {
        plan_result = wrapped.result;
        plan_done   = true;
    };
    action_client->async_send_goal(goal_msg, opts);

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (!plan_done && std::chrono::steady_clock::now() < deadline) {
        executor_.spin_some(std::chrono::milliseconds(50));
    }
    ASSERT_TRUE(plan_done) << "Plan action timed out";
    ASSERT_NE(plan_result, nullptr);
    ASSERT_TRUE(plan_result->success) << plan_result->error_msg;

    // 2. Load BT and execute
    ex_node_->loadAndExecute(plan_result->bt_xml);

    deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (ex_node_->lastStatus() == BT::NodeStatus::RUNNING
           && std::chrono::steady_clock::now() < deadline)
    {
        executor_.spin_some(std::chrono::milliseconds(20));
    }
    EXPECT_EQ(ex_node_->lastStatus(), BT::NodeStatus::SUCCESS);

    // 3. Verify goal state in WorldModel
    const auto& wm = wm_node_->worldModel();
    EXPECT_TRUE(wm.getFact("(searched sector_a)"));
    EXPECT_TRUE(wm.getFact("(classified sector_a)"));
}
