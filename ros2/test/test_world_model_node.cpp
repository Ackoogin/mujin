#include <gtest/gtest.h>
#include "mujin_ros2/world_model_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <chrono>
#include <thread>

class WorldModelNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<mujin_ros2::WorldModelNode>();
    }
    void TearDown() override {
        node_.reset();
        rclcpp::shutdown();
    }

    // Manually drive lifecycle transitions
    void configure() {
        node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        rclcpp::spin_some(node_->get_node_base_interface());
    }
    void activate() {
        node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        rclcpp::spin_some(node_->get_node_base_interface());
    }

    std::shared_ptr<mujin_ros2::WorldModelNode> node_;
};

TEST_F(WorldModelNodeTest, ConfiguresWithoutPddlFile) {
    // No PDDL files — should warn but not fail
    ASSERT_NO_THROW(configure());
}

TEST_F(WorldModelNodeTest, DirectWorldModelAccess) {
    configure();
    // Can access the WorldModel directly (in-process mode)
    auto& wm = node_->worldModel();
    EXPECT_EQ(wm.version(), 0u);
}

TEST_F(WorldModelNodeTest, SetAndGetFactViaWorldModel) {
    configure();
    auto& wm = node_->worldModel();

    // Build minimal domain so the fluent exists
    wm.typeSystem().addType("object");
    wm.typeSystem().addType("robot", "object");
    wm.typeSystem().addType("location", "object");
    wm.addObject("uav1", "robot");
    wm.addObject("base", "location");
    wm.registerPredicate("at", {"robot", "location"});

    wm.setFact("(at uav1 base)", true, "test");
    EXPECT_TRUE(wm.getFact("(at uav1 base)"));
    EXPECT_GE(wm.version(), 1u);
}

TEST_F(WorldModelNodeTest, GetFactServiceCall) {
    configure();
    activate();

    auto& wm = node_->worldModel();
    wm.typeSystem().addType("object");
    wm.typeSystem().addType("robot", "object");
    wm.typeSystem().addType("location", "object");
    wm.addObject("uav1", "robot");
    wm.addObject("base", "location");
    wm.registerPredicate("at", {"robot", "location"});
    wm.setFact("(at uav1 base)", true, "test");

    // Create a client node and call the service
    auto client_node = rclcpp::Node::make_shared("test_client");
    auto client = client_node->create_client<mujin_ros2::srv::GetFact>(
        "/world_model_node/get_fact");

    // Wait for service
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

    auto req  = std::make_shared<mujin_ros2::srv::GetFact::Request>();
    req->key  = "(at uav1 base)";

    auto future = client->async_send_request(req);

    // Spin both nodes until the future is ready
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_->get_node_base_interface());
    exec.add_node(client_node);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready
           && std::chrono::steady_clock::now() < deadline)
    {
        exec.spin_some();
    }

    ASSERT_EQ(future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
    auto res = future.get();
    EXPECT_TRUE(res->found);
    EXPECT_TRUE(res->value);
}

TEST_F(WorldModelNodeTest, SetFactServiceCall) {
    configure();
    activate();

    auto& wm = node_->worldModel();
    wm.typeSystem().addType("object");
    wm.typeSystem().addType("robot", "object");
    wm.typeSystem().addType("location", "object");
    wm.addObject("uav1", "robot");
    wm.addObject("base", "location");
    wm.registerPredicate("at", {"robot", "location"});

    auto client_node = rclcpp::Node::make_shared("test_client");
    auto client = client_node->create_client<mujin_ros2::srv::SetFact>(
        "/world_model_node/set_fact");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

    auto req    = std::make_shared<mujin_ros2::srv::SetFact::Request>();
    req->key    = "(at uav1 base)";
    req->value  = true;
    req->source = "test";

    auto future = client->async_send_request(req);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_->get_node_base_interface());
    exec.add_node(client_node);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready
           && std::chrono::steady_clock::now() < deadline)
    {
        exec.spin_some();
    }

    ASSERT_EQ(future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
    auto res = future.get();
    EXPECT_TRUE(res->success);
    EXPECT_TRUE(wm.getFact("(at uav1 base)"));
}
