/**
 * mujin_test_app — End-to-end vertical slice
 *
 * Demonstrates the complete pipeline:
 *   1. Parse PDDL domain + problem → WorldModel
 *   2. Register BT action mappings (stub implementations)
 *   3. Plan with LAPKT BRFS
 *   4. Compile plan to BT XML
 *   5. Execute BT to completion
 *   6. Verify goal state
 */

#include "mujin/pddl_parser.h"
#include "mujin/world_model.h"
#include "mujin/action_registry.h"
#include "mujin/planner.h"
#include "mujin/plan_compiler.h"
#include "mujin/bt_nodes/check_world_predicate.h"
#include "mujin/bt_nodes/set_world_predicate.h"
#include "mujin/bt_logger.h"
#include "mujin/wm_audit_log.h"
#include "mujin/plan_audit_log.h"

#if defined(MUJIN_FOXGLOVE)
#include "mujin/foxglove_bridge.h"
#endif

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/loggers/bt_observer.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>

// Stub action nodes that just log and succeed
class StubMoveAction : public BT::SyncActionNode {
public:
    StubMoveAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override {
        std::cout << "  [BT] " << name() << " executed\n";
        return BT::NodeStatus::SUCCESS;
    }
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("param0"),
            BT::InputPort<std::string>("param1"),
            BT::InputPort<std::string>("param2"),
        };
    }
};

class StubSearchAction : public BT::SyncActionNode {
public:
    StubSearchAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override {
        std::cout << "  [BT] " << name() << " executed\n";
        return BT::NodeStatus::SUCCESS;
    }
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("param0"),
            BT::InputPort<std::string>("param1"),
        };
    }
};

class StubClassifyAction : public BT::SyncActionNode {
public:
    StubClassifyAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override {
        std::cout << "  [BT] " << name() << " executed\n";
        return BT::NodeStatus::SUCCESS;
    }
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("param0"),
            BT::InputPort<std::string>("param1"),
        };
    }
};

int main() {
    std::cout << "============================================\n";
    std::cout << " MUJIN End-to-End Vertical Slice\n";
    std::cout << " PDDL → LAPKT → BT.CPP Pipeline\n";
    std::cout << "============================================\n";

    // ---- Step 1: Build world model from PDDL ----
    std::cout << "\n--- Step 1: Build WorldModel ---\n";
    mujin::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("object");
    ts.addType("location", "object");
    ts.addType("sector", "location");
    ts.addType("robot", "object");

    wm.addObject("uav1", "robot");
    wm.addObject("base", "location");
    wm.addObject("sector_a", "sector");
    wm.addObject("sector_b", "sector");

    wm.registerPredicate("at", {"robot", "location"});
    wm.registerPredicate("searched", {"sector"});
    wm.registerPredicate("classified", {"sector"});

    wm.registerAction("move",
        {"?r", "?from", "?to"}, {"robot", "location", "location"},
        {"(at ?r ?from)"}, {"(at ?r ?to)"}, {"(at ?r ?from)"});
    wm.registerAction("search",
        {"?r", "?s"}, {"robot", "sector"},
        {"(at ?r ?s)"}, {"(searched ?s)"}, {});
    wm.registerAction("classify",
        {"?r", "?s"}, {"robot", "sector"},
        {"(at ?r ?s)", "(searched ?s)"}, {"(classified ?s)"}, {});

    // ---- Observability: attach WM audit log (Layer 3) ----
    mujin::WmAuditLog wm_audit("mujin_wm_audit.jsonl");

#if defined(MUJIN_FOXGLOVE)
    // ---- Observability: start Foxglove bridge (Layer 4) ----
    mujin::FoxgloveBridge foxglove({/*.port=*/8765, /*.server_name=*/"mujin"});
    foxglove.start();
    auto foxglove_wm_sink = foxglove.wmEventSink();
#endif

    wm.setAuditCallback([&wm_audit
#if defined(MUJIN_FOXGLOVE)
                          , &foxglove_wm_sink
#endif
                         ](uint64_t ver, uint64_t ts,
                           const std::string& fact, bool val,
                           const std::string& src) {
        wm_audit.onFactChange(ver, ts, fact, val, src);
#if defined(MUJIN_FOXGLOVE)
        // Forward to Foxglove as a formatted JSON line
        std::string json = std::string("{\"wm_version\":") + std::to_string(ver)
            + ",\"ts_us\":" + std::to_string(ts)
            + ",\"fact\":\"" + fact + "\""
            + ",\"value\":" + (val ? "true" : "false")
            + ",\"source\":\"" + src + "\"}";
        foxglove_wm_sink(json);
#endif
    });

    // Set initial state
    wm.setFact("(at uav1 base)", true, "planner_init");

    // Set goal
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    std::cout << "  Fluents: " << wm.numFluents() << "\n";
    std::cout << "  Ground actions: " << wm.numGroundActions() << "\n";
    std::cout << "  Initial: at(uav1, base) = " << wm.getFact("(at uav1 base)") << "\n";

    // ---- Step 2: Register BT action mappings ----
    std::cout << "\n--- Step 2: Register Action Mappings ---\n";
    mujin::ActionRegistry registry;
    registry.registerAction("move", "StubMoveAction");
    registry.registerAction("search", "StubSearchAction");
    registry.registerAction("classify", "StubClassifyAction");
    std::cout << "  Registered: move, search, classify\n";

    // ---- Step 3: Plan with LAPKT BRFS ----
    std::cout << "\n--- Step 3: Plan with LAPKT BRFS ---\n";
    mujin::Planner planner;
    auto plan_result = planner.solve(wm);

    if (!plan_result.success) {
        std::cerr << "  FAIL: No plan found!\n";
        return EXIT_FAILURE;
    }

    std::cout << "  Plan found! " << plan_result.steps.size() << " steps, cost=" << plan_result.cost << "\n";
    std::cout << "  Expanded: " << plan_result.expanded << ", Generated: " << plan_result.generated
              << ", Solve time: " << plan_result.solve_time_ms << " ms\n";
    for (unsigned i = 0; i < plan_result.steps.size(); ++i) {
        auto& ga = wm.groundActions()[plan_result.steps[i].action_index];
        std::cout << "  " << (i + 1) << ". " << ga.signature << "\n";
    }

    // ---- Step 4: Compile plan to BT XML ----
    std::cout << "\n--- Step 4: Compile Plan to BT ---\n";
    mujin::PlanCompiler compiler;
    auto xml = compiler.compileSequential(plan_result.steps, wm, registry);
    std::cout << "  Generated BT XML (" << xml.size() << " bytes)\n";

    // ---- Observability: record plan audit trail (Layer 5) ----
    mujin::PlanAuditLog plan_audit("mujin_plan_audit.jsonl");
    {
        mujin::PlanAuditLog::Episode ep;
        ep.ts_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        // Snapshot init state: all true fluents
        for (unsigned i = 0; i < wm.numFluents(); ++i) {
            if (wm.getFact(i)) {
                ep.init_facts.push_back(wm.fluentName(i));
            }
        }
        // Goal fluents
        for (auto gid : wm.goalFluentIds()) {
            ep.goal_fluents.push_back(wm.fluentName(gid));
        }
        ep.solver = "BRFS";
        ep.solve_time_ms = plan_result.solve_time_ms;
        ep.success = plan_result.success;
        ep.expanded = plan_result.expanded;
        ep.generated = plan_result.generated;
        ep.cost = plan_result.cost;
        // Plan action signatures
        for (auto& step : plan_result.steps) {
            ep.plan_actions.push_back(wm.groundActions()[step.action_index].signature);
        }
        ep.bt_xml = xml;
        plan_audit.recordEpisode(ep);
        std::cout << "  Plan audit trail recorded\n";
    }

    // ---- Step 5: Execute BT ----
    std::cout << "\n--- Step 5: Execute BT ---\n";
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<mujin::CheckWorldPredicate>("CheckWorldPredicate");
    factory.registerNodeType<mujin::SetWorldPredicate>("SetWorldPredicate");
    factory.registerNodeType<StubMoveAction>("StubMoveAction");
    factory.registerNodeType<StubSearchAction>("StubSearchAction");
    factory.registerNodeType<StubClassifyAction>("StubClassifyAction");

    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

    // ---- Observability: attach BT loggers (Layers 1 + 2) ----
    BT::TreeObserver observer(tree);

    mujin::MujinBTLogger bt_logger(tree, "MissionPlan", &wm);
    bt_logger.addFileSink("mujin_bt_events.jsonl");
#if defined(MUJIN_FOXGLOVE)
    bt_logger.addCallbackSink(foxglove.btEventSink());
#endif

    auto status = tree.tickWhileRunning();
    std::cout << "  Tree status: " << BT::toStr(status) << "\n";

    // ---- Observability: report statistics ----
    std::cout << "\n--- Observability Report ---\n";
    std::cout << "  BT events logged:   " << bt_logger.events().size() << "\n";
    std::cout << "  WM audit entries:    " << wm_audit.size() << "\n";
    std::cout << "  Plan audit episodes: " << plan_audit.size() << "\n";
    bt_logger.flush();
    wm_audit.flush();
    plan_audit.flush();
    std::cout << "  Files: mujin_bt_events.jsonl, mujin_wm_audit.jsonl, mujin_plan_audit.jsonl\n";
#if defined(MUJIN_FOXGLOVE)
    std::cout << "  Foxglove bridge: ws://localhost:8765\n";
    foxglove.stop();
#endif

    // ---- Step 6: Verify goal state ----
    std::cout << "\n--- Step 6: Verify Goal State ---\n";
    bool goal_reached = true;
    bool searched = wm.getFact("(searched sector_a)");
    bool classified = wm.getFact("(classified sector_a)");
    bool at_base = wm.getFact("(at uav1 base)");
    bool at_sector = wm.getFact("(at uav1 sector_a)");

    std::cout << "  searched(sector_a)    = " << searched << "\n";
    std::cout << "  classified(sector_a)  = " << classified << "\n";
    std::cout << "  at(uav1, base)        = " << at_base << "\n";
    std::cout << "  at(uav1, sector_a)    = " << at_sector << "\n";

    goal_reached = searched && classified;

    std::cout << "\n============================================\n";
    if (goal_reached && status == BT::NodeStatus::SUCCESS) {
        std::cout << " SUCCESS: Goal state reached!\n";
    } else {
        std::cout << " FAILURE: Goal state NOT reached.\n";
    }
    std::cout << "============================================\n";

    return (goal_reached && status == BT::NodeStatus::SUCCESS) ? EXIT_SUCCESS : EXIT_FAILURE;
}
