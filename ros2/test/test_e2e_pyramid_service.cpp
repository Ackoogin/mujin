/// \brief E2E test: PDDL plan generation → BT execution → pyramid service call.
///
/// Builds a domain where a robot must move to a build site and construct a
/// pyramid.  The "build_pyramid" action is mapped to an InvokeService subtree
/// that calls a stub PYRAMID service node.  The test verifies the full
/// pipeline: WorldModel → Planner → PlanCompiler → ExecutorNode → InvokeService
/// → stub pyramid service, and asserts that the service received the expected
/// calls with correct parameters.

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/action_node.h>

#include <ame_ros2/executor_node.hpp>
#include <ame_ros2/planner_node.hpp>
#include <ame_ros2/world_model_node.hpp>
#include <ame/pyramid_service.h>

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Tracking pyramid service stub — records every async call for verification
// ---------------------------------------------------------------------------
struct PyramidCallRecord {
  std::string service_name;
  std::string operation;
  ame::ServiceMessage request;
};

class TrackingPyramidService : public ame::IPyramidService {
public:
  bool call(const std::string& service_name,
            const std::string& operation,
            const ame::ServiceMessage& request,
            ame::ServiceMessage& /*response*/) override {
    std::lock_guard<std::mutex> lk(mu_);
    calls_.push_back({service_name, operation, request});
    return true;
  }

  uint64_t callAsync(const std::string& service_name,
                     const std::string& operation,
                     const ame::ServiceMessage& request) override {
    std::lock_guard<std::mutex> lk(mu_);
    calls_.push_back({service_name, operation, request});
    return ++next_id_;
  }

  ame::AsyncCallStatus pollResult(uint64_t /*request_id*/,
                                  ame::ServiceMessage& /*response*/) override {
    return ame::AsyncCallStatus::SUCCESS;
  }

  void cancelCall(uint64_t /*request_id*/) override {}

  std::vector<PyramidCallRecord> calls() const {
    std::lock_guard<std::mutex> lk(mu_);
    return calls_;
  }

  size_t callCount() const {
    std::lock_guard<std::mutex> lk(mu_);
    return calls_.size();
  }

private:
  mutable std::mutex mu_;
  std::vector<PyramidCallRecord> calls_;
  uint64_t next_id_ = 0;
};

// ---------------------------------------------------------------------------
// Stub BT action for "move" (simple, no pyramid service needed)
// ---------------------------------------------------------------------------
class StubMoveAction : public BT::SyncActionNode {
public:
  StubMoveAction(const std::string& n, const BT::NodeConfiguration& c)
      : BT::SyncActionNode(n, c) {}

  BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("param0"),
        BT::InputPort<std::string>("param1"),
        BT::InputPort<std::string>("param2"),
    };
  }
};

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------
class E2EPyramidServiceTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);

    wm_node_ = std::make_shared<ame_ros2::WorldModelNode>();
    pl_node_ = std::make_shared<ame_ros2::PlannerNode>();
    ex_node_ = std::make_shared<ame_ros2::ExecutorNode>();

    // In-process wiring (skip ROS2 IPC for fast testing)
    pl_node_->setInProcessWorldModel(&wm_node_->worldModel());
    ex_node_->setInProcessWorldModel(&wm_node_->worldModel());

    // Wire the tracking pyramid service stub
    ex_node_->setPyramidService(&pyramid_stub_);

    // Hierarchical planning dependencies (for ExecutePhaseAction if needed)
    ex_node_->setPlanner(&pl_node_->planner());
    ex_node_->setPlanCompiler(&pl_node_->compiler());
    ex_node_->setActionRegistry(&pl_node_->actionRegistry());

    // --- Action mappings ---
    // "move" uses a simple stub BT node
    pl_node_->actionRegistry().registerAction("move", "StubMoveAction");

    // "build_pyramid" is mapped to an InvokeService subtree that calls the
    // pyramid service with the robot and site parameters forwarded.
    pl_node_->actionRegistry().registerActionSubTree(
        "build_pyramid",
        R"(<InvokeService service_name="construction")"
        R"( operation="build_pyramid")"
        R"( timeout_ms="5000")"
        R"( param_names="?robot;?site")"
        R"( param_values="{param0};{param1}"/>)");

    // Register BT node types on executor factory
    ex_node_->factory().registerNodeType<StubMoveAction>("StubMoveAction");
    // InvokeService is registered automatically by ExecutorNode

    // --- Lifecycle transitions ---
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

    // Single-threaded executor for all nodes
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(wm_node_->get_node_base_interface());
    executor_->add_node(pl_node_->get_node_base_interface());
    executor_->add_node(ex_node_->get_node_base_interface());

    // --- Build PDDL domain ---
    auto& wm = wm_node_->worldModel();

    // Types
    wm.typeSystem().addType("object");
    wm.typeSystem().addType("location", "object");
    wm.typeSystem().addType("build_site", "location");
    wm.typeSystem().addType("robot", "object");

    // Objects
    wm.addObject("builder1", "robot");
    wm.addObject("base", "location");
    wm.addObject("pyramid_site", "build_site");

    // Predicates
    wm.registerPredicate("at", {"robot", "location"});
    wm.registerPredicate("pyramid_built", {"build_site"});

    // Actions
    wm.registerAction(
        "move",
        {"?r", "?from", "?to"},
        {"robot", "location", "location"},
        {"(at ?r ?from)"},          // preconditions
        {"(at ?r ?to)"},            // add effects
        {"(at ?r ?from)"});         // delete effects

    wm.registerAction(
        "build_pyramid",
        {"?r", "?s"},
        {"robot", "build_site"},
        {"(at ?r ?s)"},             // preconditions: robot must be at site
        {"(pyramid_built ?s)"},     // add effects
        {});                        // delete effects

    // Initial state
    wm.setFact("(at builder1 base)", true, "test_init");
  }

  void TearDown() override {
    executor_.reset();
    ex_node_.reset();
    pl_node_.reset();
    wm_node_.reset();
    rclcpp::shutdown();
  }

  void spinUntilPlanDone(
      std::shared_ptr<const ame_ros2::action::Plan::Result>& result,
      bool& done,
      std::chrono::seconds timeout = std::chrono::seconds(30)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!done && std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(std::chrono::milliseconds(50));
    }
  }

  void spinUntilExecutionDone(
      std::chrono::seconds timeout = std::chrono::seconds(10)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (ex_node_->lastStatus() != BT::NodeStatus::SUCCESS &&
           ex_node_->lastStatus() != BT::NodeStatus::FAILURE &&
           std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(std::chrono::milliseconds(20));
    }
  }

  TrackingPyramidService pyramid_stub_;
  std::shared_ptr<ame_ros2::WorldModelNode> wm_node_;
  std::shared_ptr<ame_ros2::PlannerNode> pl_node_;
  std::shared_ptr<ame_ros2::ExecutorNode> ex_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

// ---------------------------------------------------------------------------
// Test: Plan generates move + build_pyramid, execution calls pyramid service
// ---------------------------------------------------------------------------
TEST_F(E2EPyramidServiceTest, PlanAndExecuteCallsPyramidService) {
  // Goal: pyramid is built at the site
  auto action_client =
      rclcpp_action::create_client<ame_ros2::action::Plan>(
          pl_node_, "/planner_node/plan");
  ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(3)));

  auto goal_msg = ame_ros2::action::Plan::Goal();
  goal_msg.goal_fluents = {"(pyramid_built pyramid_site)"};

  std::shared_ptr<const ame_ros2::action::Plan::Result> plan_result;
  bool plan_done = false;

  auto options = rclcpp_action::Client<ame_ros2::action::Plan>::SendGoalOptions();
  options.result_callback = [&](auto wrapped) {
    plan_result = wrapped.result;
    plan_done = true;
  };
  action_client->async_send_goal(goal_msg, options);

  // Wait for planning to complete
  spinUntilPlanDone(plan_result, plan_done);
  ASSERT_TRUE(plan_done) << "Plan action timed out";
  ASSERT_NE(plan_result, nullptr);
  ASSERT_TRUE(plan_result->success) << plan_result->error_msg;

  // Plan should contain at least move + build_pyramid
  EXPECT_GE(plan_result->plan_actions.size(), 2u);

  // Wait for BT execution to finish
  spinUntilExecutionDone();
  EXPECT_EQ(ex_node_->lastStatus(), BT::NodeStatus::SUCCESS);

  // Verify world model goal state
  const auto& wm = wm_node_->worldModel();
  EXPECT_TRUE(wm.getFact("(pyramid_built pyramid_site)"));
  EXPECT_TRUE(wm.getFact("(at builder1 pyramid_site)"));

  // Verify the pyramid service stub was called
  ASSERT_GE(pyramid_stub_.callCount(), 1u);

  auto calls = pyramid_stub_.calls();
  // Find the build_pyramid call
  bool found_build_call = false;
  for (const auto& c : calls) {
    if (c.service_name == "construction" && c.operation == "build_pyramid") {
      found_build_call = true;
      // Verify PDDL parameters were forwarded
      EXPECT_EQ(c.request.get("?robot"), "builder1");
      EXPECT_EQ(c.request.get("?site"), "pyramid_site");
    }
  }
  EXPECT_TRUE(found_build_call)
      << "Expected a call to construction/build_pyramid";
}

// ---------------------------------------------------------------------------
// Test: Pyramid service failure causes BT execution to fail
// ---------------------------------------------------------------------------

class FailingPyramidService : public ame::IPyramidService {
public:
  bool call(const std::string&, const std::string&,
            const ame::ServiceMessage&, ame::ServiceMessage&) override {
    return false;
  }

  uint64_t callAsync(const std::string&, const std::string&,
                     const ame::ServiceMessage&) override {
    return ++next_id_;
  }

  ame::AsyncCallStatus pollResult(uint64_t, ame::ServiceMessage&) override {
    return ame::AsyncCallStatus::FAILURE;
  }

  void cancelCall(uint64_t) override {}

private:
  uint64_t next_id_ = 0;
};

TEST_F(E2EPyramidServiceTest, PyramidServiceFailurePropagates) {
  // Replace the stub with a failing service
  FailingPyramidService failing_service;
  ex_node_->setPyramidService(&failing_service);

  auto action_client =
      rclcpp_action::create_client<ame_ros2::action::Plan>(
          pl_node_, "/planner_node/plan");
  ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(3)));

  auto goal_msg = ame_ros2::action::Plan::Goal();
  goal_msg.goal_fluents = {"(pyramid_built pyramid_site)"};

  std::shared_ptr<const ame_ros2::action::Plan::Result> plan_result;
  bool plan_done = false;

  auto options = rclcpp_action::Client<ame_ros2::action::Plan>::SendGoalOptions();
  options.result_callback = [&](auto wrapped) {
    plan_result = wrapped.result;
    plan_done = true;
  };
  action_client->async_send_goal(goal_msg, options);

  spinUntilPlanDone(plan_result, plan_done);
  ASSERT_TRUE(plan_done) << "Plan action timed out";
  ASSERT_TRUE(plan_result->success) << "Planning should still succeed";

  // BT execution should fail because the pyramid service returns FAILURE
  spinUntilExecutionDone();
  EXPECT_EQ(ex_node_->lastStatus(), BT::NodeStatus::FAILURE);

  // Goal should NOT be achieved
  EXPECT_FALSE(wm_node_->worldModel().getFact("(pyramid_built pyramid_site)"));
}
