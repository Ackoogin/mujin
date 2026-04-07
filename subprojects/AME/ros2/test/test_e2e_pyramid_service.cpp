/// \brief E2E test: PDDL plan → BT execution → PYRAMID service calls via InvokeService.
///
/// Uses the standard UAV search-and-classify domain with all PDDL actions
/// mapped to InvokeService subtree templates (the PYRAMID integration pattern
/// from subprojects/AME/docs/guides/pyramid_service_integration_guide.md).  A tracking
/// IPyramidService stub records every async call so the test can verify that
/// the correct PYRAMID services were invoked with the right parameters.
///
/// Pipeline under test:
///   WorldModel → Planner (LAPKT) → PlanCompiler → ExecutorNode
///     → InvokeService BT nodes → IPyramidService stub

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
// Tracking PYRAMID service stub — records every async call for verification
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

  std::vector<PyramidCallRecord> callsByOperation(const std::string& op) const {
    std::lock_guard<std::mutex> lk(mu_);
    std::vector<PyramidCallRecord> result;
    for (const auto& c : calls_) {
      if (c.operation == op) {
        result.push_back(c);
      }
    }
    return result;
  }

private:
  mutable std::mutex mu_;
  std::vector<PyramidCallRecord> calls_;
  uint64_t next_id_ = 0;
};

// ---------------------------------------------------------------------------
// Failing PYRAMID service stub — all async calls return FAILURE
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

// ---------------------------------------------------------------------------
// Test fixture — UAV search domain with all actions via PYRAMID InvokeService
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

    // Wire the tracking PYRAMID service stub
    ex_node_->setPyramidService(&pyramid_stub_);

    // Hierarchical planning dependencies
    ex_node_->setPlanner(&pl_node_->planner());
    ex_node_->setPlanCompiler(&pl_node_->compiler());
    ex_node_->setActionRegistry(&pl_node_->actionRegistry());

    // --- Map all PDDL actions to PYRAMID InvokeService templates ---
    // Following the pattern from pyramid_service_integration_guide.md

    // (move ?robot ?from ?to) → mobility/move
    pl_node_->actionRegistry().registerActionSubTree("move",
        R"(<InvokeService service_name="mobility" operation="move")"
        R"( timeout_ms="5000")"
        R"( param_names="?robot;?from;?to")"
        R"( param_values="{param0};{param1};{param2}"/>)");

    // (search ?robot ?sector) → sensors/area_search
    pl_node_->actionRegistry().registerActionSubTree("search",
        R"(<InvokeService service_name="sensors" operation="area_search")"
        R"( timeout_ms="10000")"
        R"( param_names="?robot;?sector")"
        R"( param_values="{param0};{param1}"/>)");

    // (classify ?robot ?sector) → imaging/classify with extra request fields
    pl_node_->actionRegistry().registerActionSubTree("classify",
        R"(<InvokeService service_name="imaging" operation="classify")"
        R"( timeout_ms="10000")"
        R"( request_json="confidence_threshold=0.8")"
        R"( param_names="?robot;?sector")"
        R"( param_values="{param0};{param1}"/>)");

    // InvokeService is registered automatically by ExecutorNode's registerCoreNodes

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

    // --- Build PDDL domain (standard UAV search) ---
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
        "move",
        {"?r", "?from", "?to"},
        {"robot", "location", "location"},
        {"(at ?r ?from)"},
        {"(at ?r ?to)"},
        {"(at ?r ?from)"});

    wm.registerAction(
        "search",
        {"?r", "?s"},
        {"robot", "sector"},
        {"(at ?r ?s)"},
        {"(searched ?s)"},
        {});

    wm.registerAction(
        "classify",
        {"?r", "?s"},
        {"robot", "sector"},
        {"(at ?r ?s)", "(searched ?s)"},
        {"(classified ?s)"},
        {});

    wm.setFact("(at uav1 base)", true, "test_init");
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
// Test: Full pipeline — plan, execute, verify all PYRAMID service calls
// ---------------------------------------------------------------------------
TEST_F(E2EPyramidServiceTest, PlanAndExecuteInvokesPyramidServices) {
  // Goal: search and classify sector_a
  auto action_client =
      rclcpp_action::create_client<ame_ros2::action::Plan>(
          pl_node_, "/planner_node/plan");
  ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(3)));

  auto goal_msg = ame_ros2::action::Plan::Goal();
  goal_msg.goal_fluents = {
      "(searched sector_a)",
      "(classified sector_a)",
  };

  std::shared_ptr<const ame_ros2::action::Plan::Result> plan_result;
  bool plan_done = false;

  auto options = rclcpp_action::Client<ame_ros2::action::Plan>::SendGoalOptions();
  options.result_callback = [&](auto wrapped) {
    plan_result = wrapped.result;
    plan_done = true;
  };
  action_client->async_send_goal(goal_msg, options);

  // Wait for planning
  spinUntilPlanDone(plan_result, plan_done);
  ASSERT_TRUE(plan_done) << "Plan action timed out";
  ASSERT_NE(plan_result, nullptr);
  ASSERT_TRUE(plan_result->success) << plan_result->error_msg;

  // Plan should contain move + search + classify (at least 3 steps)
  EXPECT_GE(plan_result->plan_actions.size(), 3u);

  // In-process mode: explicitly hand the compiled BT XML to the executor
  ASSERT_FALSE(plan_result->bt_xml.empty());
  ex_node_->loadAndExecute(plan_result->bt_xml);

  // Wait for BT execution
  spinUntilExecutionDone();
  EXPECT_EQ(ex_node_->lastStatus(), BT::NodeStatus::SUCCESS);

  // Verify world model goal state
  const auto& wm = wm_node_->worldModel();
  EXPECT_TRUE(wm.getFact("(searched sector_a)"));
  EXPECT_TRUE(wm.getFact("(classified sector_a)"));
  EXPECT_TRUE(wm.getFact("(at uav1 sector_a)"));

  // Verify PYRAMID service calls were made with correct parameters
  // At least 3 calls: mobility/move, sensors/area_search, imaging/classify
  ASSERT_GE(pyramid_stub_.callCount(), 3u);

  // Check mobility/move call
  auto move_calls = pyramid_stub_.callsByOperation("move");
  ASSERT_GE(move_calls.size(), 1u);
  EXPECT_EQ(move_calls[0].service_name, "mobility");
  EXPECT_EQ(move_calls[0].request.get("robot"), "uav1");
  EXPECT_EQ(move_calls[0].request.get("from"), "base");
  EXPECT_EQ(move_calls[0].request.get("to"), "sector_a");

  // Check sensors/area_search call
  auto search_calls = pyramid_stub_.callsByOperation("area_search");
  ASSERT_EQ(search_calls.size(), 1u);
  EXPECT_EQ(search_calls[0].service_name, "sensors");
  EXPECT_EQ(search_calls[0].request.get("robot"), "uav1");
  EXPECT_EQ(search_calls[0].request.get("sector"), "sector_a");

  // Check imaging/classify call (should include extra request_json fields)
  auto classify_calls = pyramid_stub_.callsByOperation("classify");
  ASSERT_EQ(classify_calls.size(), 1u);
  EXPECT_EQ(classify_calls[0].service_name, "imaging");
  EXPECT_EQ(classify_calls[0].request.get("robot"), "uav1");
  EXPECT_EQ(classify_calls[0].request.get("sector"), "sector_a");
  EXPECT_EQ(classify_calls[0].request.get("confidence_threshold"), "0.8");
}

// ---------------------------------------------------------------------------
// Test: PYRAMID service failure propagates through the BT to execution failure
// ---------------------------------------------------------------------------
TEST_F(E2EPyramidServiceTest, PyramidServiceFailurePropagates) {
  // Replace the stub with a failing PYRAMID service
  FailingPyramidService failing_service;
  ex_node_->setPyramidService(&failing_service);

  auto action_client =
      rclcpp_action::create_client<ame_ros2::action::Plan>(
          pl_node_, "/planner_node/plan");
  ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(3)));

  auto goal_msg = ame_ros2::action::Plan::Goal();
  goal_msg.goal_fluents = {
      "(searched sector_a)",
      "(classified sector_a)",
  };

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

  // In-process mode: explicitly hand the compiled BT XML to the executor
  ASSERT_FALSE(plan_result->bt_xml.empty());
  ex_node_->loadAndExecute(plan_result->bt_xml);

  // BT execution should fail because the PYRAMID service returns FAILURE
  spinUntilExecutionDone();
  EXPECT_EQ(ex_node_->lastStatus(), BT::NodeStatus::FAILURE);

  // Goals should NOT be achieved
  EXPECT_FALSE(wm_node_->worldModel().getFact("(searched sector_a)"));
  EXPECT_FALSE(wm_node_->worldModel().getFact("(classified sector_a)"));
}
