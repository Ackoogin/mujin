// Full in-process pipeline test:
//   WorldModel + Planner + Executor with direct pointer wiring.
//   Mirrors tests/test_e2e_pipeline.cpp through the ROS2 wrappers.
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ame_ros2/executor_node.hpp>
#include <ame_ros2/planner_node.hpp>
#include <ame_ros2/world_model_node.hpp>
#include <ame/action_registry.h>
#include <ame/executor_component.h>
#include <ame/planner_component.h>
#include <ame/world_model.h>
#include <pcl/executor.hpp>

#include <chrono>

class StubMoveAction : public BT::SyncActionNode {
public:
  StubMoveAction(const std::string& n, const BT::NodeConfiguration& c)
      : BT::SyncActionNode(n, c) {}
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("param0"),
             BT::InputPort<std::string>("param1"),
             BT::InputPort<std::string>("param2") };
  }
};

class StubSearchAction : public BT::SyncActionNode {
public:
  StubSearchAction(const std::string& n, const BT::NodeConfiguration& c)
      : BT::SyncActionNode(n, c) {}
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("param0"),
             BT::InputPort<std::string>("param1") };
  }
};

class StubClassifyAction : public BT::SyncActionNode {
public:
  StubClassifyAction(const std::string& n, const BT::NodeConfiguration& c)
      : BT::SyncActionNode(n, c) {}
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

    wm_node_ = std::make_shared<ame_ros2::WorldModelNode>();
    pl_node_ = std::make_shared<ame_ros2::PlannerNode>();
    auto ex_opts = rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("tick_rate_hz", 1000.0)});
    ex_node_ = std::make_shared<ame_ros2::ExecutorNode>(ex_opts);

    pl_node_->setInProcessWorldModel(&wm_node_->worldModel());
    ex_node_->setInProcessWorldModel(&wm_node_->worldModel());
    ex_node_->setPlanner(&pl_node_->planner());
    ex_node_->setPlanCompiler(&pl_node_->compiler());
    ex_node_->setActionRegistry(&pl_node_->actionRegistry());

    pl_node_->actionRegistry().registerAction("move", "StubMoveAction");
    pl_node_->actionRegistry().registerAction("search", "StubSearchAction");
    pl_node_->actionRegistry().registerAction("classify", "StubClassifyAction");

    ex_node_->factory().registerNodeType<StubMoveAction>("StubMoveAction");
    ex_node_->factory().registerNodeType<StubSearchAction>("StubSearchAction");
    ex_node_->factory().registerNodeType<StubClassifyAction>("StubClassifyAction");

    pcl_exec_ = std::make_unique<pcl::Executor>();
    pcl_exec_->add(ex_node_->component());

    ASSERT_EQ(wm_node_->on_configure(rclcpp_lifecycle::State{}),
              ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);
    ASSERT_EQ(wm_node_->on_activate(rclcpp_lifecycle::State{}),
              ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);
    ASSERT_EQ(pl_node_->on_configure(rclcpp_lifecycle::State{}),
              ame_ros2::PlannerNode::CallbackReturn::SUCCESS);
    ASSERT_EQ(pl_node_->on_activate(rclcpp_lifecycle::State{}),
              ame_ros2::PlannerNode::CallbackReturn::SUCCESS);
    ASSERT_EQ(ex_node_->on_configure(rclcpp_lifecycle::State{}),
              ame_ros2::ExecutorNode::CallbackReturn::SUCCESS);
    ASSERT_EQ(ex_node_->on_activate(rclcpp_lifecycle::State{}),
              ame_ros2::ExecutorNode::CallbackReturn::SUCCESS);

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(wm_node_->get_node_base_interface());
    executor_->add_node(pl_node_->get_node_base_interface());
    executor_->add_node(ex_node_->get_node_base_interface());

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
    pcl_exec_.reset();
    executor_.reset();
    ex_node_.reset();
    pl_node_.reset();
    wm_node_.reset();
    rclcpp::shutdown();
  }

  void spinUntilExecutionDone(std::chrono::seconds timeout = std::chrono::seconds(10)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (ex_node_->component().lastStatus() != BT::NodeStatus::SUCCESS &&
           ex_node_->component().lastStatus() != BT::NodeStatus::FAILURE &&
           std::chrono::steady_clock::now() < deadline) {
      pcl_exec_->spinOnce(0);
    }
  }

  std::shared_ptr<ame_ros2::WorldModelNode> wm_node_;
  std::shared_ptr<ame_ros2::PlannerNode>    pl_node_;
  std::shared_ptr<ame_ros2::ExecutorNode>   ex_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<pcl::Executor>            pcl_exec_;
};

/// REQ_ENGINE_004: The ROS2 wrappers shall plan and execute through the BT XML handoff.
TEST_F(FullPipelineTest, PlanAndExecuteReachesGoal) {
  // Plan in-process (action server replaced by direct PCL component call)
  auto plan_result = pl_node_->plannerComponent().solveGoal({
      "(searched sector_a)",
      "(classified sector_a)",
  });
  ASSERT_TRUE(plan_result.success) << plan_result.error_msg;
  ASSERT_FALSE(plan_result.bt_xml.empty());

  ex_node_->component().loadAndExecute(plan_result.bt_xml);
  spinUntilExecutionDone();

  EXPECT_EQ(ex_node_->component().lastStatus(), BT::NodeStatus::SUCCESS);

  const auto& wm = wm_node_->worldModel();
  EXPECT_TRUE(wm.getFact("(searched sector_a)"));
  EXPECT_TRUE(wm.getFact("(classified sector_a)"));
}
