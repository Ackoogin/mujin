/**
 * mujin_combined — in-process single-executor mode.
 *
 * All three nodes share one SingleThreadedExecutor and the canonical WorldModel
 * is accessed directly (no service IPC for BT nodes or planner snapshots).
 *
 * Usage:
 *   ros2 run mujin_ros2 mujin_combined \
 *     --ros-args \
 *     -p domain.pddl_file:=domains/uav_search/domain.pddl \
 *     -p domain.problem_file:=domains/uav_search/problem.pddl
 *
 * After the lifecycle manager brings all nodes up, send a Plan action goal:
 *   ros2 action send_goal /mujin/plan mujin_ros2/action/Plan \
 *     "{goal_fluents: ['(searched sector_a)', '(classified sector_a)'], replan: false}"
 *
 * The ExecutorNode will then tick the compiled BT automatically.
 */

#include "mujin_ros2/world_model_node.hpp"
#include "mujin_ros2/planner_node.hpp"
#include "mujin_ros2/executor_node.hpp"
#include "mujin_ros2/lifecycle_manager.hpp"

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <memory>
#include <thread>

// ---------------------------------------------------------------------------
// Stub action nodes (same as main.cpp) for the UAV search demo domain.
// In a real deployment, replace these with implementations that call hardware
// or PYRAMID services (Extension 4 from extensions.md).
// ---------------------------------------------------------------------------
class StubMoveAction : public BT::SyncActionNode {
public:
    StubMoveAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("param0"),
                 BT::InputPort<std::string>("param1"),
                 BT::InputPort<std::string>("param2") };
    }
};

class StubSearchAction : public BT::SyncActionNode {
public:
    StubSearchAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("param0"),
                 BT::InputPort<std::string>("param1") };
    }
};

class StubClassifyAction : public BT::SyncActionNode {
public:
    StubClassifyAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("param0"),
                 BT::InputPort<std::string>("param1") };
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create nodes
    auto wm_node = std::make_shared<mujin_ros2::WorldModelNode>();
    auto pl_node = std::make_shared<mujin_ros2::PlannerNode>();
    auto ex_node = std::make_shared<mujin_ros2::ExecutorNode>();
    auto lm_node = std::make_shared<mujin_ros2::MujinLifecycleManager>();

    // Wire in-process mode: direct WorldModel pointer skips service calls
    pl_node->setInProcessWorldModel(&wm_node->worldModel());
    ex_node->setInProcessWorldModel(&wm_node->worldModel());

    // Register action node types on the ExecutorNode's factory.
    // These must be registered before loadAndExecute() is called.
    // Replace or extend with real action implementations as needed.
    ex_node->factory().registerNodeType<StubMoveAction>("StubMoveAction");
    ex_node->factory().registerNodeType<StubSearchAction>("StubSearchAction");
    ex_node->factory().registerNodeType<StubClassifyAction>("StubClassifyAction");

    // Also register action mappings in the PlannerNode's ActionRegistry so that
    // the compiled BT XML uses the correct node type names.
    pl_node->actionRegistry().registerAction("move",     "StubMoveAction");
    pl_node->actionRegistry().registerAction("search",   "StubSearchAction");
    pl_node->actionRegistry().registerAction("classify", "StubClassifyAction");

    // Single-threaded executor: all ROS2 callbacks run sequentially.
    // This is required because WorldModel has no mutex.
    // The Planner's std::thread uses a local copy of WorldModel, so no race.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(wm_node->get_node_base_interface());
    executor.add_node(pl_node->get_node_base_interface());
    executor.add_node(ex_node->get_node_base_interface());
    executor.add_node(lm_node->get_node_base_interface());

    // Bring all nodes through configure → activate in a background thread
    // (so we don't block spin())
    std::thread startup_thread([&lm_node]() {
        // Give the executor a moment to start
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (!lm_node->startup()) {
            RCLCPP_ERROR(rclcpp::get_logger("mujin_combined"),
                "Lifecycle startup failed — nodes may not be fully active");
        }
    });

    RCLCPP_INFO(rclcpp::get_logger("mujin_combined"),
        "mujin_combined running. Send a Plan action to /mujin/plan to start.");

    executor.spin();

    startup_thread.join();
    rclcpp::shutdown();
    return 0;
}
