#include <gtest/gtest.h>
#include "mujin_ros2/executor_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <behaviortree_cpp/action_node.h>
#include <chrono>

// Minimal stub action for the test BT
class StubTestAction : public BT::SyncActionNode {
public:
    StubTestAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
    static BT::PortsList providedPorts() { return {}; }
};

static const char* SIMPLE_BT_XML = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <StubTestAction name="step1"/>
      <StubTestAction name="step2"/>
    </Sequence>
  </BehaviorTree>
</root>
)";

class ExecutorNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        wm_ = std::make_unique<mujin::WorldModel>();
        ex_node_ = std::make_shared<mujin_ros2::ExecutorNode>();
        ex_node_->setInProcessWorldModel(wm_.get());

        // Register custom action type before configure
        ex_node_->factory().registerNodeType<StubTestAction>("StubTestAction");

        ex_node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        ex_node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

        executor_.add_node(ex_node_->get_node_base_interface());
    }

    void TearDown() override {
        ex_node_.reset();
        wm_.reset();
        rclcpp::shutdown();
    }

    std::unique_ptr<mujin::WorldModel>           wm_;
    std::shared_ptr<mujin_ros2::ExecutorNode>    ex_node_;
    rclcpp::executors::SingleThreadedExecutor    executor_;
};

TEST_F(ExecutorNodeTest, LoadAndExecuteSimpleBT) {
    ex_node_->loadAndExecute(SIMPLE_BT_XML);

    // Spin until the BT completes (should be immediate for stubs)
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (ex_node_->lastStatus() == BT::NodeStatus::RUNNING
           && std::chrono::steady_clock::now() < deadline)
    {
        executor_.spin_some(std::chrono::milliseconds(20));
    }

    EXPECT_EQ(ex_node_->lastStatus(), BT::NodeStatus::SUCCESS);
}
