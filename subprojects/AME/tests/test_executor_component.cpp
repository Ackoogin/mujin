#include <gtest/gtest.h>

#include <ame/bt_nodes/check_world_predicate.h>
#include <ame/action_registry.h>
#include <ame/execution_sink.h>
#include <ame/executor_component.h>
#include <ame/planner_component.h>

#include <test_component_utils.hpp>

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

class RecordingSink : public ame::IExecutionSink {
public:
  void reset(const std::string& session_id) override {
    session_id_ = session_id;
    commands_.clear();
    results_.clear();
    pending_.clear();
  }

  ame::ExecutionSubmission submit(const ame::ActionCommand& command) override {
    commands_.push_back(command);
    pending_[command.command_id] = true;

    ame::ExecutionSubmission submission;
    submission.accepted = true;
    submission.command_egress_visible = true;
    return submission;
  }

  std::vector<ame::ActionCommand> pullCommands() override {
    auto commands = commands_;
    commands_.clear();
    return commands;
  }

  void pushResult(const ame::CommandResult& result) override {
    results_[result.command_id] = result;
    pending_[result.command_id] = false;
  }

  void cancel(const std::string& command_id) override {
    pending_[command_id] = false;
  }

  std::optional<ame::CommandResult> resultFor(
      const std::string& command_id) const override {
    const auto it = results_.find(command_id);
    if (it == results_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  bool isPending(const std::string& command_id) const override {
    const auto it = pending_.find(command_id);
    return it != pending_.end() && it->second;
  }

  std::vector<ame::RequirementPlacementRecord> readPlacements() const override {
    return {};
  }

private:
  std::string session_id_;
  std::vector<ame::ActionCommand> commands_;
  std::unordered_map<std::string, ame::CommandResult> results_;
  std::unordered_map<std::string, bool> pending_;
};

}  // namespace

///< REQ_ENGINE_003: Executor component shall run compiled planner output against the shared world model.
TEST(ExecutorComponent, ExecutesPlannerOutputAgainstSharedWorldModel) {
  auto wm = buildUavWorldModel();

  ame::PlannerComponent planner_component;
  planner_component.setParam("plan_audit.enabled", false);
  planner_component.setInProcessWorldModel(&wm);
  registerUavActions(planner_component.actionRegistry());
  ASSERT_EQ(planner_component.configure(), PCL_OK);
  ASSERT_EQ(planner_component.activate(), PCL_OK);

  const auto plan_result =
      planner_component.solveGoal({"(searched sector_a)", "(classified sector_a)"});
  ASSERT_TRUE(plan_result.success);

  ame::ExecutorComponent executor_component;
  executor_component.setParam("bt_log.enabled", false);
  executor_component.setInProcessWorldModel(&wm);
  registerUavStubNodes(executor_component.factory());

  std::vector<std::string> events;
  executor_component.setEventSink(
      [&events](const std::string& json_line) { events.push_back(json_line); });

  ASSERT_EQ(executor_component.configure(), PCL_OK);
  ASSERT_EQ(executor_component.activate(), PCL_OK);
  executor_component.loadAndExecute(plan_result.bt_xml);

  for (int i = 0; i < 50; ++i) {
    executor_component.tickOnce();
    if (executor_component.lastStatus() == BT::NodeStatus::SUCCESS) {
      break;
    }
  }

  EXPECT_EQ(executor_component.lastStatus(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(wm.getFact("(searched sector_a)"));
  EXPECT_TRUE(wm.getFact("(classified sector_a)"));
  EXPECT_FALSE(events.empty());

  EXPECT_EQ(executor_component.deactivate(), PCL_OK);
  EXPECT_EQ(executor_component.cleanup(), PCL_OK);
  EXPECT_EQ(executor_component.shutdown(), PCL_OK);
  EXPECT_EQ(planner_component.deactivate(), PCL_OK);
  EXPECT_EQ(planner_component.cleanup(), PCL_OK);
  EXPECT_EQ(planner_component.shutdown(), PCL_OK);
}

TEST(ExecutorComponent, RegisteredVerbDispatchesThroughActionSink) {
  ame::ActionRegistry registry;
  registry.registerAction("inspect", "inspect");

  RecordingSink sink;
  sink.reset("session");

  ame::ExecutorComponent executor_component;
  executor_component.setParam("bt_log.enabled", false);
  executor_component.setActionRegistry(&registry);
  executor_component.setActionSink(&sink);

  ASSERT_EQ(executor_component.configure(), PCL_OK);
  ASSERT_EQ(executor_component.activate(), PCL_OK);

  const std::string bt_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <inspect param0="uav1" param1="sector_a"/>
  </BehaviorTree>
</root>
)";

  ASSERT_NO_THROW(executor_component.loadAndExecute(bt_xml));
  executor_component.tickOnce();

  auto commands = sink.pullCommands();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(commands[0].action_name, "inspect");
  EXPECT_EQ(commands[0].signature, "inspect(uav1,sector_a)");
  EXPECT_EQ(executor_component.lastStatus(), BT::NodeStatus::RUNNING);

  ame::CommandResult result;
  result.command_id = commands[0].command_id;
  result.status = ame::CommandStatus::SUCCEEDED;
  result.source = "test";
  sink.pushResult(result);

  executor_component.tickOnce();
  EXPECT_EQ(executor_component.lastStatus(), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(executor_component.deactivate(), PCL_OK);
  EXPECT_EQ(executor_component.cleanup(), PCL_OK);
  EXPECT_EQ(executor_component.shutdown(), PCL_OK);
}

TEST(ExecutorComponent, RegisteredVerbFailsWhenDispatchResultIsLost) {
  ame::ActionRegistry registry;
  registry.registerAction("inspect", "inspect");

  RecordingSink sink;
  sink.reset("session");

  ame::ExecutorComponent executor_component;
  executor_component.setParam("bt_log.enabled", false);
  executor_component.setActionRegistry(&registry);
  executor_component.setActionSink(&sink);

  ASSERT_EQ(executor_component.configure(), PCL_OK);
  ASSERT_EQ(executor_component.activate(), PCL_OK);

  const std::string bt_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <inspect param0="uav1" param1="sector_a"/>
  </BehaviorTree>
</root>
)";

  ASSERT_NO_THROW(executor_component.loadAndExecute(bt_xml));
  executor_component.tickOnce();

  auto commands = sink.pullCommands();
  ASSERT_EQ(commands.size(), 1u);
  EXPECT_EQ(executor_component.lastStatus(), BT::NodeStatus::RUNNING);

  sink.reset("new-session");
  executor_component.tickOnce();
  EXPECT_EQ(executor_component.lastStatus(), BT::NodeStatus::FAILURE);

  EXPECT_EQ(executor_component.deactivate(), PCL_OK);
  EXPECT_EQ(executor_component.cleanup(), PCL_OK);
  EXPECT_EQ(executor_component.shutdown(), PCL_OK);
}

TEST(ExecutorComponent, UnregisteredVerbFailsTreeLoad) {
  ame::ActionRegistry registry;
  registry.registerAction("inspect", "inspect");

  ame::ExecutorComponent executor_component;
  executor_component.setParam("bt_log.enabled", false);
  executor_component.setActionRegistry(&registry);

  ASSERT_EQ(executor_component.configure(), PCL_OK);
  ASSERT_EQ(executor_component.activate(), PCL_OK);

  const std::string bt_xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <survey param0="uav1"/>
  </BehaviorTree>
</root>
)";

  EXPECT_THROW(executor_component.loadAndExecute(bt_xml), std::exception);

  EXPECT_EQ(executor_component.deactivate(), PCL_OK);
  EXPECT_EQ(executor_component.cleanup(), PCL_OK);
  EXPECT_EQ(executor_component.shutdown(), PCL_OK);
}
