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

#include <ame/agent_dispatcher.h>
#include <ame/action_registry.h>
#include <ame/executor_component.h>
#include <ame/pcl_msg_json.h>
#include <ame/pyramid_service.h>

#include <pcl/executor.hpp>
#include <pcl/pcl_executor.h>

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
    auto ex_opts = rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("tick_rate_hz", 1000.0)});
    ex_node_ = std::make_shared<ame_ros2::ExecutorNode>(ex_opts);

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

    pcl_exec_ = std::make_unique<pcl::Executor>();
    pcl_exec_->add(ex_node_->component());

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

  ame::MockPyramidService mock_pyramid_;
  std::shared_ptr<ame_ros2::WorldModelNode> wm_node_;
  std::shared_ptr<ame_ros2::PlannerNode> pl_node_;
  std::shared_ptr<ame_ros2::ExecutorNode> ex_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<pcl::Executor>           pcl_exec_;
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

  ex_node_->component().loadAndExecute(bt_xml);
  spinUntilExecutionDone();

  EXPECT_EQ(ex_node_->component().lastStatus(), BT::NodeStatus::SUCCESS);
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

  ex_node_->component().loadAndExecute(bt_xml);
  spinUntilExecutionDone();

  EXPECT_EQ(ex_node_->component().lastStatus(), BT::NodeStatus::SUCCESS);
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

  ex_node_->component().loadAndExecute(bt_xml);
  spinUntilExecutionDone();

  EXPECT_EQ(ex_node_->component().lastStatus(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(wm_node_->worldModel().getFact("(searched sector_a)"));
  // Agent should be available again after delegation completes
  EXPECT_TRUE(wm_node_->worldModel().getAgent("uav1")->available);
}

// ---------------------------------------------------------------------------
// Test: Perception mutations are queued and flushed
// ---------------------------------------------------------------------------

class PerceptionQueueTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(PerceptionQueueTest, DetectionEnqueuedAndFlushedByTick) {
  // Detections now arrive via PCL subscriber (not ROS2 pub/sub).
  // Create a PCL executor, add the component before configure, then verify
  // that postIncoming + spinOnce causes the mutation to be applied.
  // NodeOptions injects params into ROS2 param system -- on_configure reads them,
  // so these values survive the configure step (unlike setParam before configure).
  auto node_opts = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("perception.enabled", true),
      rclcpp::Parameter("audit_log.enabled",  false),
      rclcpp::Parameter("publish_rate_hz",    1000.0),
  });
  auto node = std::make_shared<ame_ros2::WorldModelNode>(node_opts);

  pcl::Executor pcl_exec;
  pcl_exec.add(node->component());

  ASSERT_EQ(node->on_configure(rclcpp_lifecycle::State{}),
            ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);

  auto& wm = node->worldModel();
  wm.typeSystem().addType("object");
  wm.typeSystem().addType("robot", "object");
  wm.typeSystem().addType("location", "object");
  wm.addObject("uav1", "robot");
  wm.addObject("sector_a", "location");
  wm.registerPredicate("at", {"robot", "location"});

  ASSERT_EQ(node->on_activate(rclcpp_lifecycle::State{}),
            ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);

  ame::Detection det;
  det.confidence      = 0.9f;
  det.sensor_source   = "camera_front";
  det.entity_id       = "uav1";
  det.property_keys   = {"at"};
  det.property_values = {"sector_a"};

  std::string det_json = ame::ame_pack_detection(det);
  pcl_msg_t msg{};
  msg.data      = det_json.c_str();
  msg.size      = static_cast<uint32_t>(det_json.size());
  msg.type_name = "ame/Detection";
  ASSERT_EQ(pcl_exec.postIncoming("detections", &msg), PCL_OK);

  // spinOnce: dispatches subscriber callback (enqueueMutation), then calls on_tick
  // (applyQueuedMutations) -- tick fires because 1000 Hz period <= initial dt=0.001s.
  pcl_exec.spinOnce(0);

  EXPECT_TRUE(wm.getFact("(at uav1 sector_a)"));
}

// ---------------------------------------------------------------------------
// Test: AgentDispatcherNode exposes dispatch_goals via PCL service
// ---------------------------------------------------------------------------

TEST(DispatcherServiceTest, DispatchGoalsServiceExistsInPcl) {
  rclcpp::init(0, nullptr);

  auto dispatcher = std::make_shared<ame_ros2::AgentDispatcherNode>();
  pcl::Executor pcl_exec;
  pcl_exec.add(dispatcher->dispatcher());

  ASSERT_EQ(dispatcher->on_configure(rclcpp_lifecycle::State{}),
            ame_ros2::AgentDispatcherNode::CallbackReturn::SUCCESS);
  ASSERT_EQ(dispatcher->on_activate(rclcpp_lifecycle::State{}),
            ame_ros2::AgentDispatcherNode::CallbackReturn::SUCCESS);

  // Verify the PCL dispatch_goals service is reachable (empty goals -> failure response)
  std::string req_json = ame::ame_pack_dispatch_goals_request({});
  pcl_msg_t req_msg{};
  req_msg.data      = req_json.c_str();
  req_msg.size      = static_cast<uint32_t>(req_json.size());
  req_msg.type_name = "ame/DispatchGoals_Request";
  pcl_msg_t resp_msg{};

  pcl_status_t rc = pcl_executor_invoke_service(
      pcl_exec.handle(), "dispatch_goals", &req_msg, &resp_msg);
  EXPECT_EQ(rc, PCL_OK);

  auto resp = ame::ame_unpack_dispatch_goals_response(&resp_msg);
  EXPECT_FALSE(resp.success);
  EXPECT_FALSE(resp.error_msg.empty());

  rclcpp::shutdown();
}
