#include <gtest/gtest.h>
#include "ame_ros2/world_model_node.hpp"
#include <ame/world_model.h>
#include <ame/world_model_component.h>
#include <ame/pcl_msg_json.h>
#include <pcl/executor.hpp>
#include <pcl/pcl_executor.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

class WorldModelNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<ame_ros2::WorldModelNode>();
    }
    void TearDown() override {
        node_.reset();
        rclcpp::shutdown();
    }

    auto configure() {
        return node_->on_configure(rclcpp_lifecycle::State{});
    }
    auto activate() {
        return node_->on_activate(rclcpp_lifecycle::State{});
    }

    std::shared_ptr<ame_ros2::WorldModelNode> node_;
};

TEST_F(WorldModelNodeTest, ConfiguresWithoutPddlFile) {
    ASSERT_EQ(configure(), ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);
}

TEST_F(WorldModelNodeTest, DirectWorldModelAccess) {
    ASSERT_EQ(configure(), ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);
    auto& wm = node_->worldModel();
    EXPECT_EQ(wm.version(), 0u);
}

TEST_F(WorldModelNodeTest, SetAndGetFactViaWorldModel) {
    ASSERT_EQ(configure(), ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);
    auto& wm = node_->worldModel();

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

// ---------------------------------------------------------------------------
// PCL service tests -- call get_fact / set_fact / query_state via PCL executor
// ---------------------------------------------------------------------------

TEST_F(WorldModelNodeTest, GetFactServiceCall) {
    // Register component with PCL executor before configure
    pcl::Executor pcl_exec;
    node_->component().setParam("publish_rate_hz", 1000.0);
    pcl_exec.add(node_->component());

    ASSERT_EQ(configure(), ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);
    ASSERT_EQ(activate(),  ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);

    auto& wm = node_->worldModel();
    wm.typeSystem().addType("object");
    wm.typeSystem().addType("robot", "object");
    wm.typeSystem().addType("location", "object");
    wm.addObject("uav1", "robot");
    wm.addObject("base", "location");
    wm.registerPredicate("at", {"robot", "location"});
    wm.setFact("(at uav1 base)", true, "test");

    std::string req_json = ame::ame_pack_get_fact_request("(at uav1 base)");
    pcl_msg_t req_msg{};
    req_msg.data      = req_json.c_str();
    req_msg.size      = static_cast<uint32_t>(req_json.size());
    req_msg.type_name = "ame/GetFact_Request";
    pcl_msg_t resp_msg{};

    pcl_status_t rc = pcl_executor_invoke_service(
        pcl_exec.handle(), "get_fact", &req_msg, &resp_msg);
    ASSERT_EQ(rc, PCL_OK);

    auto result = ame::ame_unpack_get_fact_response(&resp_msg);
    EXPECT_TRUE(result.found);
    EXPECT_TRUE(result.value);
}

TEST_F(WorldModelNodeTest, SetFactServiceCall) {
    pcl::Executor pcl_exec;
    node_->component().setParam("publish_rate_hz", 1000.0);
    pcl_exec.add(node_->component());

    ASSERT_EQ(configure(), ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);
    ASSERT_EQ(activate(),  ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);

    auto& wm = node_->worldModel();
    wm.typeSystem().addType("object");
    wm.typeSystem().addType("robot", "object");
    wm.typeSystem().addType("location", "object");
    wm.addObject("uav1", "robot");
    wm.addObject("base", "location");
    wm.registerPredicate("at", {"robot", "location"});

    ame::SetFactRequest req;
    req.key    = "(at uav1 base)";
    req.value  = true;
    req.source = "test";

    std::string req_json = ame::ame_pack_set_fact_request(req);
    pcl_msg_t req_msg{};
    req_msg.data      = req_json.c_str();
    req_msg.size      = static_cast<uint32_t>(req_json.size());
    req_msg.type_name = "ame/SetFact_Request";
    pcl_msg_t resp_msg{};

    pcl_status_t rc = pcl_executor_invoke_service(
        pcl_exec.handle(), "set_fact", &req_msg, &resp_msg);
    ASSERT_EQ(rc, PCL_OK);

    auto result = ame::ame_unpack_set_fact_response(&resp_msg);
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(wm.getFact("(at uav1 base)"));
}

TEST_F(WorldModelNodeTest, QueryStateReturnsGoalFluents) {
    pcl::Executor pcl_exec;
    node_->component().setParam("publish_rate_hz", 1000.0);
    pcl_exec.add(node_->component());

    ASSERT_EQ(configure(), ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);
    ASSERT_EQ(activate(),  ame_ros2::WorldModelNode::CallbackReturn::SUCCESS);

    auto& wm = node_->worldModel();
    wm.typeSystem().addType("object");
    wm.typeSystem().addType("sector", "object");
    wm.addObject("sector_a", "sector");
    wm.registerPredicate("searched", {"sector"});
    wm.setGoal({"(searched sector_a)"});

    std::string req_json = ame::ame_pack_query_state_request({});
    pcl_msg_t req_msg{};
    req_msg.data      = req_json.c_str();
    req_msg.size      = static_cast<uint32_t>(req_json.size());
    req_msg.type_name = "ame/QueryState_Request";
    pcl_msg_t resp_msg{};

    pcl_status_t rc = pcl_executor_invoke_service(
        pcl_exec.handle(), "query_state", &req_msg, &resp_msg);
    ASSERT_EQ(rc, PCL_OK);

    auto snap = ame::ame_unpack_query_state_response(&resp_msg);
    ASSERT_TRUE(snap.success);
    ASSERT_EQ(snap.goal_fluents.size(), 1u);
    EXPECT_EQ(snap.goal_fluents.front(), "(searched sector_a)");
}
