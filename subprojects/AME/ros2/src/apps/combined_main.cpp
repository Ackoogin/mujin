/// \brief `ame_combined` — in-process single-executor deployment.
///
/// Architecture (post-PCL migration):
///   - A PCL Executor drives all component ticking and message dispatch
///     in a dedicated background thread.
///   - A ROS2 SingleThreadedExecutor runs on the main thread and handles
///     only ROS2 lifecycle transitions driven by AmeLifecycleManager.
///   - The thin ROS2 lifecycle nodes bridge ROS2 lifecycle transitions to
///     PCL component configure/activate/deactivate/cleanup/shutdown calls.
///   - All WorldModel reads/writes, BT ticking, and planning happen on the
///     PCL executor thread — no cross-thread WorldModel access.
///
/// Usage:
///   ros2 run ame_ros2 ame_combined \
///     --ros-args \
///     -p domain.pddl_file:=subprojects/AME/domains/uav_search/domain.pddl \
///     -p domain.problem_file:=subprojects/AME/domains/uav_search/problem.pddl
///
/// After lifecycle startup, send a plan via the PCL "plan" service
/// (or extend with a ROS2 transport adapter for ros2 action send_goal).

#include "ame_ros2/world_model_node.hpp"
#include "ame_ros2/planner_node.hpp"
#include "ame_ros2/executor_node.hpp"
#include "ame_ros2/lifecycle_manager.hpp"

#include <ame/action_registry.h>
#include <ame/executor_component.h>
#include <ame/planner_component.h>
#include <ame/pyramid_service.h>
#include <ame/world_model_component.h>
#include <pcl/executor.hpp>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <atomic>
#include <memory>
#include <thread>

// ---------------------------------------------------------------------------
// Stub action nodes for the UAV search demo domain
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

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // -- Create thin ROS2 lifecycle nodes -----------------------------------
  auto wm_node = std::make_shared<ame_ros2::WorldModelNode>();
  auto pl_node = std::make_shared<ame_ros2::PlannerNode>();
  auto ex_node = std::make_shared<ame_ros2::ExecutorNode>();
  auto lm_node = std::make_shared<ame_ros2::AmeLifecycleManager>();

  // -- Wire in-process dependencies (before configure) --------------------
  // The thin nodes delegate these to their components immediately.
  pl_node->setInProcessWorldModel(&wm_node->worldModel());
  pl_node->setWorldModelMutex(&wm_node->component().worldModelMutex());
  ex_node->setInProcessWorldModel(&wm_node->worldModel());

  // PYRAMID service (mock for demo)
  ame::MockPyramidService mock_pyramid;
  ex_node->setPyramidService(&mock_pyramid);

  // Hierarchical planning dependencies
  ex_node->setPlanner(&pl_node->planner());
  ex_node->setPlanCompiler(&pl_node->compiler());
  ex_node->setActionRegistry(&pl_node->actionRegistry());
  ex_node->setPlanAuditLog(pl_node->planAuditLog());

  // Register action node types on the ExecutorNode's BT factory
  ex_node->factory().registerNodeType<StubMoveAction>("StubMoveAction");
  ex_node->factory().registerNodeType<StubSearchAction>("StubSearchAction");
  ex_node->factory().registerNodeType<StubClassifyAction>("StubClassifyAction");

  // Register PDDL-to-BT action mappings in the PlannerNode's ActionRegistry
  pl_node->actionRegistry().registerAction("move",     "StubMoveAction");
  pl_node->actionRegistry().registerAction("search",   "StubSearchAction");
  pl_node->actionRegistry().registerAction("classify", "StubClassifyAction");

  // -- Create PCL executor and register component handles ----------------
  // The PCL executor drives ticking; components are added here so the
  // executor knows about them before on_configure is called.
  pcl::Executor pcl_executor;
  pcl_executor.add(wm_node->component());
  pcl_executor.add(pl_node->plannerComponent());
  pcl_executor.add(ex_node->component());

  // -- Build the ROS2 executor --------------------------------------------
  rclcpp::executors::SingleThreadedExecutor ros_executor;
  ros_executor.add_node(wm_node->get_node_base_interface());
  ros_executor.add_node(pl_node->get_node_base_interface());
  ros_executor.add_node(ex_node->get_node_base_interface());
  ros_executor.add_node(lm_node->get_node_base_interface());

  // -- Start PCL executor in a background thread -------------------------
  // The PCL executor will idle until components are activated via
  // ROS2 lifecycle transitions.
  std::atomic<bool> pcl_running{true};
  std::thread pcl_thread([&] {
    pcl_executor.spin();
    pcl_running.store(false);
  });

  // -- Bring all nodes through configure → activate via lifecycle manager --
  std::thread startup_thread([&lm_node]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (!lm_node->startup()) {
      RCLCPP_ERROR(rclcpp::get_logger("ame_combined"),
                   "Lifecycle startup failed — nodes may not be fully active");
    }
  });

  RCLCPP_INFO(rclcpp::get_logger("ame_combined"),
              "ame_combined running (PCL executor + ROS2 lifecycle).");

  ros_executor.spin();

  // -- Shutdown -----------------------------------------------------------
  pcl_executor.requestShutdown();
  pcl_thread.join();
  startup_thread.join();
  rclcpp::shutdown();
  return 0;
}
