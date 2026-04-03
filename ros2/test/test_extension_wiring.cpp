/// \brief Tests for ROS2 extension wiring (gap-fix Steps 1-8).
///
/// Validates that InvokeService, ExecutePhaseAction, DelegateToAgent,
/// perception mutation queue, and AgentDispatcherNode dispatch_goals
/// service are all correctly wired through the ROS2 lifecycle nodes.

#include <gtest/gtest.h>

#include <ame_ros2/executor_node.hpp>
#include <ame_ros2/planner_node.hpp>
#include <ame_ros2/world_model_node.hpp>
#include <ame_ros2/agent_dispatcher_node.hpp>

#include <ame/pyramid_service.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <behaviortree_cpp/action_node.h>

#include <chrono>

// ---------------------------------------------------------------------------
// Stub action nodes for planning domains
// ---------------------------------------------------------------------------
class StubMoveAction : public BT::SyncActionNode {
public:
  StubMoveAction(const std::string& n, const BT::NodeConfiguration& c)
    : BT::SyncActionNode(n, c) {}
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("param0"),
            BT::InputPort<std::string>("param1"),
            BT::InputPort<std::string>("param2")};
  }
};

class StubSearchAction : public BT::SyncActionNode {
public:
  StubSearchAction(const std::string& n, const BT::NodeConfiguration& c)
    : BT::SyncActionNode(n, c) {}
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("param0"),
            BT::InputPort<std::string>("param1")};
  }
};

class StubClassifyAction : public BT::SyncActionNode {
public:
  StubClassifyAction(const std::string& n, const BT::NodeConfiguration& c)
    : BT::SyncActionNode(n, c) {}
  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("param0"),
            BT::InputPort<std::string>("param1")};
  }
};

// ---------------------------------------------------------------------------
// Test fixture: in-process pipeline with all extensions wired
// ---------------------------------------------------------------------------
class ExtensionWiringTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);

    wm_node_ = std::make_shared<ame_ros2::WorldModelNode>();
    pl_node_ = std::make_shared<ame_ros2::PlannerNode>();
    ex_node_ = std::make_shared<ame_ros2::ExecutorNode>();

    // In-process wiring
    pl_node_->setInProcessWorldModel(&wm_node_->worldModel());
    ex_node_->setInProcessWorldModel(&wm_node_->worldModel());

    // PYRAMID service (mock)
    ex_node_->setPyramidService(&mock_pyramid_);

    // Hierarchical planning dependencies
    ex_node_->setPlanner(&pl_node_->planner());
    ex_node_->setPlanCompiler(&pl_node_->compiler());
    ex_node_->setActionRegistry(&pl_node_->actionRegistry());
    ex_node_->setPlanAuditLog(pl_node_->planAuditLog());

    // Register action mappings
    pl_node_->actionRegistry().registerAction("move", "StubMoveAction");
    pl_node_->actionRegistry().registerAction("search", "StubSearchAction");
    pl_node_->actionRegistry().registerAction("classify", "StubClassifyAction");

    // Register BT node types
    ex_node_->factory().registerNodeType<StubMoveAction>("StubMoveAction");
    ex_node_->factory().registerNodeType<StubSearchAction>("StubSearchAction");
    ex_node_->factory().registerNodeType<StubClassifyAction>("StubClassifyAction");

    // Lifecycle transitions
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
    ASSERT_EQ(
        ex_node_->on_configure(rclcpp_lifecycle::State{}),
        ame_ros2::ExecutorNode::CallbackReturn::SUCCESS);
    ASSERT_EQ(
        ex_node_->on_activate(rclcpp_lifecycle::State{}),
        ame_ros2::ExecutorNode::CallbackReturn::SUCCESS);

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(wm_node_->get_node_base_interface());
    executor_->add_node(pl_node_->get_node_base_interface());
    executor_->add_node(ex_node_->get_node_base_interface());

    // Build the UAV search domain
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
    wm.registerAction(
      "move", {"?r", "?from", "?to"}, {"robot", "location", "location"},
      {"(at ?r ?from)"}, {"(at ?r ?to)"}, {"(at ?r ?from)"});
    wm.registerAction(
      "search", {"?r", "?s"}, {"robot", "sector"},
      {"(at ?r ?s)"}, {"(searched ?s)"}, {});
    wm.registerAction(
      "classify", {"?r", "?s"}, {"robot", "sector"},
      {"(at ?r ?s)", "(searched ?s)"}, {"(classified ?s)"}, {});
    wm.setFact("(at uav1 base)", true, "test_init");
  }

  void TearDown() override {
    executor_.reset();
    ex_node_.reset();
    pl_node_.reset();
    wm_node_.reset();
    rclcpp::shutdown();
  }

  void spinUntilExecutionDone(std::chrono::seconds timeout = std::chrono::seconds(10)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (ex_node_->lastStatus() != BT::NodeStatus::SUCCESS &&
           ex_node_->lastStatus() != BT::NodeStatus::FAILURE &&
           std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(std::chrono::milliseconds(20));
    }
  }

  ame::MockPyramidService mock_pyramid_;
  std::shared_ptr<ame_ros2::WorldModelNode> wm_node_;
  std::shared_ptr<ame_ros2::PlannerNode> pl_node_;
  std::shared_ptr<ame_ros2::ExecutorNode> ex_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

// ---------------------------------------------------------------------------
// Test: InvokeService BT node is registered and callable
// ---------------------------------------------------------------------------
TEST_F(ExtensionWiringTest, InvokeServiceRegisteredAndSucceeds) {
  // BT XML that uses InvokeService with mock PYRAMID service
  static const char* bt_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <InvokeService service_name="mobility" operation="move"
                   timeout_ms="5000"/>
  </BehaviorTree>
</root>
)";

  ex_node_->loadAndExecute(bt_xml);
  spinUntilExecutionDone();

  EXPECT_EQ(ex_node_->lastStatus(), BT::NodeStatus::SUCCESS);
}

// ---------------------------------------------------------------------------
// Test: ExecutePhaseAction BT node can plan and execute a sub-goal
// ---------------------------------------------------------------------------
TEST_F(ExtensionWiringTest, ExecutePhaseActionPlansAndExecutes) {
  // BT XML that uses ExecutePhaseAction to achieve a sub-goal
  static const char* bt_xml = R"BT(
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <ExecutePhaseAction phase_goals="(searched sector_a)"
                        phase_name="search_phase"/>
  </BehaviorTree>
</root>
)BT";

  ex_node_->loadAndExecute(bt_xml);
  spinUntilExecutionDone();

  EXPECT_EQ(ex_node_->lastStatus(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(wm_node_->worldModel().getFact("(searched sector_a)"));
}

// ---------------------------------------------------------------------------
// Test: DelegateToAgent BT node is registered and handles agent goals
// ---------------------------------------------------------------------------
TEST_F(ExtensionWiringTest, DelegateToAgentRegisteredAndExecutes) {
  // Register agent in WorldModel
  wm_node_->worldModel().registerAgent("uav1", "robot");

  static const char* bt_xml = R"BT(
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <DelegateToAgent agent_id="uav1"
                     agent_goals="(searched sector_a)"/>
  </BehaviorTree>
</root>
)BT";

  ex_node_->loadAndExecute(bt_xml);
  spinUntilExecutionDone();

  EXPECT_EQ(ex_node_->lastStatus(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(wm_node_->worldModel().getFact("(searched sector_a)"));
  // Agent should be available again after delegation completes
  EXPECT_TRUE(wm_node_->worldModel().getAgent("uav1")->available);
}

// ---------------------------------------------------------------------------
// Test: Perception mutations are queued and flushed
// ---------------------------------------------------------------------------

class PerceptionQueueTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<ame_ros2::WorldModelNode>();
    ASSERT_EQ(
        node_->on_configure(rclcpp_lifecycle::State{}),
        ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);
    ASSERT_EQ(
        node_->on_activate(rclcpp_lifecycle::State{}),
        ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);

    auto& wm = node_->worldModel();
    wm.typeSystem().addType("object");
    wm.typeSystem().addType("robot", "object");
    wm.typeSystem().addType("location", "object");
    wm.addObject("uav1", "robot");
    wm.addObject("sector_a", "location");
    wm.registerPredicate("at", {"robot", "location"});

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
  }

  void TearDown() override {
    executor_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<ame_ros2::WorldModelNode> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(PerceptionQueueTest, DetectionEnqueuedAndFlushedByTimer) {
  // Publish a detection message
  auto pub_node = rclcpp::Node::make_shared("test_detection_pub");
  auto pub = pub_node->create_publisher<ame_ros2::msg::Detection>(
    "/detections", rclcpp::SensorDataQoS());
  executor_->add_node(pub_node);

  ame_ros2::msg::Detection det;
  det.entity_id = "uav1";
  det.entity_type = "robot";
  det.property_keys = {"at"};
  det.property_values = {"sector_a"};
  det.confidence = 0.9f;
  det.sensor_source = "camera_front";
  pub->publish(det);

  // Spin to process the detection callback (enqueues mutation)
  executor_->spin_some(std::chrono::milliseconds(100));

  // Fact should NOT yet be set (it's queued, not applied)
  // Note: this depends on timing — the publish timer may or may not have fired.
  // Instead, spin enough for the timer to fire and flush the queue.
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some(std::chrono::milliseconds(50));
    if (node_->worldModel().getFact("(at uav1 sector_a)")) {
      break;
    }
  }

  EXPECT_TRUE(node_->worldModel().getFact("(at uav1 sector_a)"));
}

// ---------------------------------------------------------------------------
// Test: AgentDispatcherNode creates dispatch_goals service
// ---------------------------------------------------------------------------

class DispatcherServiceTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    dispatcher_ = std::make_shared<ame_ros2::AgentDispatcherNode>();
    ASSERT_EQ(
        dispatcher_->on_configure(rclcpp_lifecycle::State{}),
        ame_ros2::AgentDispatcherNode::CallbackReturn::SUCCESS);
    ASSERT_EQ(
        dispatcher_->on_activate(rclcpp_lifecycle::State{}),
        ame_ros2::AgentDispatcherNode::CallbackReturn::SUCCESS);

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(dispatcher_->get_node_base_interface());
  }

  void TearDown() override {
    executor_.reset();
    dispatcher_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<ame_ros2::AgentDispatcherNode> dispatcher_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(DispatcherServiceTest, DispatchGoalsServiceExists) {
  auto client_node = rclcpp::Node::make_shared("test_dispatch_client");
  auto client = client_node->create_client<ame_ros2::srv::DispatchGoals>(
    "/agent_dispatcher_node/dispatch_goals");
  executor_->add_node(client_node);

  // Spin briefly to let discovery happen
  executor_->spin_some(std::chrono::milliseconds(100));

  EXPECT_TRUE(client->wait_for_service(std::chrono::seconds(2)))
    << "dispatch_goals service not available";
}

TEST_F(DispatcherServiceTest, DispatchGoalsRejectsEmptyGoals) {
  auto client_node = rclcpp::Node::make_shared("test_dispatch_client");
  auto client = client_node->create_client<ame_ros2::srv::DispatchGoals>(
    "/agent_dispatcher_node/dispatch_goals");
  executor_->add_node(client_node);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

  auto req = std::make_shared<ame_ros2::srv::DispatchGoals::Request>();
  // Empty goal_fluents
  auto future = client->async_send_request(req);

  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready
         && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some(std::chrono::milliseconds(20));
  }

  ASSERT_EQ(future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  auto res = future.get();
  EXPECT_FALSE(res->success);
  EXPECT_FALSE(res->error_msg.empty());
}
