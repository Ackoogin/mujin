/// \brief `ame_pcl_main` -- pure PCL executor entry point (no ROS2).
///
/// Runs the full AME pipeline using the PCL executor for all component
/// ticking and message dispatch.  No ROS2 dependency.
///
/// Usage (all parameters are PCL component params set before configure):
///   Set PDDL files via component_.setParam() before configure, or pass
///   them via the socket transport at runtime using the load_domain service.
///
/// Example (in-process demo with UAV search domain):
///   Modify domain_file / problem_file below and rebuild.

#include <pcl/executor.hpp>

#include <ame/world_model_component.h>
#include <ame/executor_component.h>
#include <ame/planner_component.h>
#include <ame/agent_dispatcher.h>
#include <ame/action_registry.h>
#include <ame/pyramid_service.h>

#include <behaviortree_cpp/action_node.h>

#include <csignal>
#include <cstdio>
#include <string>

// ---------------------------------------------------------------------------
// Stub BT action nodes for the UAV search demo domain
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
// Signal handler
// ---------------------------------------------------------------------------

static pcl::Executor* g_executor = nullptr;

static void signalHandler(int /*sig*/) {
  if (g_executor) g_executor->requestShutdown();
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
  (void)argc; (void)argv;

  // Default domain paths (override by passing arguments or using load_domain)
  const std::string domain_file  = "subprojects/AME/domains/uav_search/domain.pddl";
  const std::string problem_file = "subprojects/AME/domains/uav_search/problem.pddl";

  // -- Create components --------------------------------------------------
  ame::WorldModelComponent wm_comp;
  ame::PlannerComponent    pl_comp;
  ame::ExecutorComponent   ex_comp;
  ame::AgentDispatcher     ad_comp;

  // -- Set parameters (before configure) ----------------------------------
  wm_comp.setParam("domain.pddl_file",   domain_file.c_str());
  wm_comp.setParam("domain.problem_file", problem_file.c_str());
  wm_comp.setParam("audit_log.enabled",  true);
  wm_comp.setParam("audit_log.path",     "ame_wm_audit.jsonl");
  wm_comp.setParam("publish_rate_hz",    10.0);

  pl_comp.setParam("plan_audit.enabled", true);
  pl_comp.setParam("plan_audit.path",    "ame_plan_audit.jsonl");
  pl_comp.setParam("compiler.parallel",  false);

  ex_comp.setParam("tick_rate_hz",       50.0);
  ex_comp.setParam("bt_log.enabled",     true);
  ex_comp.setParam("bt_log.path",        "ame_bt_events.jsonl");
  ex_comp.setParam("bt_log.tree_id",     "MissionPlan");

  // -- Wire in-process pointers -------------------------------------------
  pl_comp.setInProcessWorldModel(&wm_comp.worldModel());
  ex_comp.setInProcessWorldModel(&wm_comp.worldModel());

  ad_comp.setWorldModel(&wm_comp.worldModel());
  ad_comp.setPlanner(&pl_comp.planner());
  ad_comp.setPlanCompiler(&pl_comp.compiler());
  ad_comp.setActionRegistry(&pl_comp.actionRegistry());

  // -- Register UAV domain action node types ------------------------------
  pl_comp.actionRegistry().registerAction("move",     "StubMoveAction");
  pl_comp.actionRegistry().registerAction("search",   "StubSearchAction");
  pl_comp.actionRegistry().registerAction("classify", "StubClassifyAction");

  ex_comp.factory().registerNodeType<StubMoveAction>("StubMoveAction");
  ex_comp.factory().registerNodeType<StubSearchAction>("StubSearchAction");
  ex_comp.factory().registerNodeType<StubClassifyAction>("StubClassifyAction");

  // Wire hierarchical planning onto executor blackboard
  ex_comp.setBlackboardInitializer([&](const BT::Blackboard::Ptr& bb) {
    bb->set<ame::WorldModel*>("world_model",   &wm_comp.worldModel());
    bb->set<ame::Planner*>("planner",          &pl_comp.planner());
    bb->set<ame::PlanCompiler*>("plan_compiler", &pl_comp.compiler());
    bb->set<ame::ActionRegistry*>("action_registry", &pl_comp.actionRegistry());
    bb->set<BT::BehaviorTreeFactory*>("bt_factory", &ex_comp.factory());
  });

  // -- Create PCL executor and add all components -------------------------
  pcl::Executor executor;
  g_executor = &executor;

  executor.add(wm_comp);
  executor.add(pl_comp);
  executor.add(ex_comp);
  executor.add(ad_comp);

  // -- Configure and activate all components ------------------------------
  if (wm_comp.configure() != PCL_OK) { std::fputs("WorldModel configure failed\n", stderr); return 1; }
  if (pl_comp.configure() != PCL_OK) { std::fputs("Planner configure failed\n",    stderr); return 1; }
  if (ex_comp.configure() != PCL_OK) { std::fputs("Executor configure failed\n",   stderr); return 1; }
  if (ad_comp.configure() != PCL_OK) { std::fputs("Dispatcher configure failed\n", stderr); return 1; }

  if (wm_comp.activate() != PCL_OK) { std::fputs("WorldModel activate failed\n", stderr); return 1; }
  if (pl_comp.activate() != PCL_OK) { std::fputs("Planner activate failed\n",    stderr); return 1; }
  if (ex_comp.activate() != PCL_OK) { std::fputs("Executor activate failed\n",   stderr); return 1; }
  if (ad_comp.activate() != PCL_OK) { std::fputs("Dispatcher activate failed\n", stderr); return 1; }

  // -- Register signal handler for clean shutdown -------------------------
  std::signal(SIGINT,  signalHandler);
  std::signal(SIGTERM, signalHandler);

  std::fputs("ame_pcl_main running. Send a 'plan' service request to start.\n",
             stdout);
  std::fputs("Press Ctrl+C to shut down.\n", stdout);

  // -- Deterministic single-threaded spin ---------------------------------
  executor.spin();

  // -- Graceful shutdown --------------------------------------------------
  ex_comp.deactivate();  ex_comp.cleanup();  ex_comp.shutdown();
  pl_comp.deactivate();  pl_comp.cleanup();  pl_comp.shutdown();
  ad_comp.deactivate();  ad_comp.cleanup();  ad_comp.shutdown();
  wm_comp.deactivate();  wm_comp.cleanup();  wm_comp.shutdown();

  g_executor = nullptr;
  return 0;
}
