#include <gtest/gtest.h>
#include "ame_ros2/executor_node.hpp"
#include <ame/executor_component.h>
#include <ame/world_model.h>
#include <pcl/executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>

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
        wm_ = std::make_unique<ame::WorldModel>();

        // Pass tick_rate_hz=1000 via NodeOptions so on_configure doesn't overwrite with default
        auto opts = rclcpp::NodeOptions().parameter_overrides(
            {rclcpp::Parameter("tick_rate_hz", 1000.0)});
        ex_node_ = std::make_shared<ame_ros2::ExecutorNode>(opts);
        ex_node_->setInProcessWorldModel(wm_.get());
        ex_node_->factory().registerNodeType<StubTestAction>("StubTestAction");

        // PCL executor drives the BT tick -- must be added before configure
        pcl_exec_ = std::make_unique<pcl::Executor>();
        pcl_exec_->add(ex_node_->component());

        ASSERT_EQ(
            ex_node_->on_configure(rclcpp_lifecycle::State{}),
            ame_ros2::ExecutorNode::CallbackReturn::SUCCESS);
        ASSERT_EQ(
            ex_node_->on_activate(rclcpp_lifecycle::State{}),
            ame_ros2::ExecutorNode::CallbackReturn::SUCCESS);
    }

    void TearDown() override {
        pcl_exec_.reset();
        ex_node_.reset();
        wm_.reset();
        rclcpp::shutdown();
    }

    std::unique_ptr<ame::WorldModel>           wm_;
    std::shared_ptr<ame_ros2::ExecutorNode>    ex_node_;
    std::unique_ptr<pcl::Executor>             pcl_exec_;
};

TEST_F(ExecutorNodeTest, LoadAndExecuteSimpleBT) {
    ex_node_->component().loadAndExecute(SIMPLE_BT_XML);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (ex_node_->component().lastStatus() != BT::NodeStatus::SUCCESS
           && ex_node_->component().lastStatus() != BT::NodeStatus::FAILURE
           && std::chrono::steady_clock::now() < deadline)
    {
        pcl_exec_->spinOnce(0);
    }

    EXPECT_EQ(ex_node_->component().lastStatus(), BT::NodeStatus::SUCCESS);
}
