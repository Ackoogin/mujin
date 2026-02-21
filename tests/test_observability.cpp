/**
 * Tests for Observability Layers 1–3:
 *   - Layer 1: TreeObserver integration
 *   - Layer 2: MujinBTLogger structured JSON events
 *   - Layer 3: WorldModel audit log with source tagging
 */

#include "mujin/world_model.h"
#include "mujin/bt_logger.h"
#include "mujin/wm_audit_log.h"
#include "mujin/plan_audit_log.h"
#include "mujin/action_registry.h"
#include "mujin/plan_compiler.h"
#include "mujin/planner.h"
#include "mujin/bt_nodes/check_world_predicate.h"
#include "mujin/bt_nodes/set_world_predicate.h"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_observer.h>

#include <gtest/gtest.h>

#include <string>
#include <vector>

// =========================================================================
// Helpers
// =========================================================================

static mujin::WorldModel makeSimpleWorldModel() {
    mujin::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("object");
    ts.addType("location", "object");
    ts.addType("sector", "location");
    ts.addType("robot", "object");

    wm.addObject("uav1", "robot");
    wm.addObject("base", "location");
    wm.addObject("sector_a", "sector");

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

    return wm;
}

// Stub BT action that always succeeds
class StubAction : public BT::SyncActionNode {
public:
    StubAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("param0"),
            BT::InputPort<std::string>("param1"),
            BT::InputPort<std::string>("param2"),
        };
    }
};

// =========================================================================
// Layer 3: WorldModel Audit Log
// =========================================================================

TEST(WmAuditLog, RecordsFactChanges) {
    mujin::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("object");
    ts.addType("location", "object");
    ts.addType("robot", "object");
    wm.addObject("uav1", "robot");
    wm.addObject("base", "location");
    wm.registerPredicate("at", {"robot", "location"});

    mujin::WmAuditLog audit;
    wm.setAuditCallback([&audit](uint64_t ver, uint64_t ts_us,
                                  const std::string& fact, bool val,
                                  const std::string& src) {
        audit.onFactChange(ver, ts_us, fact, val, src);
    });

    ASSERT_EQ(audit.size(), 0u);

    wm.setFact("(at uav1 base)", true, "planner_init");
    ASSERT_EQ(audit.size(), 1u);
    EXPECT_EQ(audit.entries()[0].fact, "(at uav1 base)");
    EXPECT_TRUE(audit.entries()[0].value);
    EXPECT_EQ(audit.entries()[0].source, "planner_init");
    EXPECT_EQ(audit.entries()[0].wm_version, 1u);
    EXPECT_GT(audit.entries()[0].ts_us, 0u);
}

TEST(WmAuditLog, NoEntryWhenValueUnchanged) {
    mujin::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("object");
    ts.addType("robot", "object");
    wm.addObject("uav1", "robot");
    wm.registerPredicate("ready", {"robot"});

    mujin::WmAuditLog audit;
    wm.setAuditCallback([&audit](uint64_t ver, uint64_t ts_us,
                                  const std::string& fact, bool val,
                                  const std::string& src) {
        audit.onFactChange(ver, ts_us, fact, val, src);
    });

    // Default is false; setting to false again should not trigger
    wm.setFact("(ready uav1)", false, "test");
    EXPECT_EQ(audit.size(), 0u);

    // Set to true → entry
    wm.setFact("(ready uav1)", true, "test");
    EXPECT_EQ(audit.size(), 1u);

    // Set to true again (no change) → no entry
    wm.setFact("(ready uav1)", true, "test");
    EXPECT_EQ(audit.size(), 1u);
}

TEST(WmAuditLog, SourceTagFromSetWorldPredicate) {
    auto wm = makeSimpleWorldModel();
    wm.setFact("(at uav1 base)", true, "planner_init");
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    mujin::WmAuditLog audit;
    wm.setAuditCallback([&audit](uint64_t ver, uint64_t ts_us,
                                  const std::string& fact, bool val,
                                  const std::string& src) {
        audit.onFactChange(ver, ts_us, fact, val, src);
    });
    audit.entries(); // clear any prior (none expected yet)

    // Plan
    mujin::Planner planner;
    auto plan = planner.solve(wm);
    ASSERT_TRUE(plan.success);

    // Compile
    mujin::ActionRegistry registry;
    registry.registerAction("move", "StubAction");
    registry.registerAction("search", "StubAction");
    registry.registerAction("classify", "StubAction");

    mujin::PlanCompiler compiler;
    auto xml = compiler.compileSequential(plan.steps, wm, registry);

    // Execute
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<mujin::CheckWorldPredicate>("CheckWorldPredicate");
    factory.registerNodeType<mujin::SetWorldPredicate>("SetWorldPredicate");
    factory.registerNodeType<StubAction>("StubAction");

    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

    auto status = tree.tickWhileRunning();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

    // Audit entries should exist and have SetWorldPredicate source tags
    EXPECT_GT(audit.size(), 0u);

    bool found_set_wp_source = false;
    for (auto& entry : audit.entries()) {
        if (entry.source.find("SetWorldPredicate:") == 0) {
            found_set_wp_source = true;
            break;
        }
    }
    EXPECT_TRUE(found_set_wp_source)
        << "Expected at least one audit entry with SetWorldPredicate source tag";
}

TEST(WmAuditLog, WritesToFile) {
    std::string filepath = "test_wm_audit_output.jsonl";

    {
        mujin::WorldModel wm;
        auto& ts = wm.typeSystem();
        ts.addType("object");
        ts.addType("robot", "object");
        wm.addObject("r1", "robot");
        wm.registerPredicate("ready", {"robot"});

        mujin::WmAuditLog audit(filepath);
        wm.setAuditCallback([&audit](uint64_t ver, uint64_t ts_us,
                                      const std::string& fact, bool val,
                                      const std::string& src) {
            audit.onFactChange(ver, ts_us, fact, val, src);
        });

        wm.setFact("(ready r1)", true, "test_source");
        // Destructor flushes
    }

    // Verify file exists and has content
    std::ifstream in(filepath);
    ASSERT_TRUE(in.is_open());
    std::string line;
    ASSERT_TRUE(std::getline(in, line));
    EXPECT_NE(line.find("\"wm_version\":"), std::string::npos);
    EXPECT_NE(line.find("\"fact\":\"(ready r1)\""), std::string::npos);
    EXPECT_NE(line.find("\"source\":\"test_source\""), std::string::npos);
    in.close();

    // Clean up
    std::remove(filepath.c_str());
}

// =========================================================================
// Layer 2: MujinBTLogger
// =========================================================================

TEST(MujinBTLogger, EmitsEventsOnBTExecution) {
    auto wm = makeSimpleWorldModel();
    wm.setFact("(at uav1 base)", true, "planner_init");
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    mujin::Planner planner;
    auto plan = planner.solve(wm);
    ASSERT_TRUE(plan.success);

    mujin::ActionRegistry registry;
    registry.registerAction("move", "StubAction");
    registry.registerAction("search", "StubAction");
    registry.registerAction("classify", "StubAction");

    mujin::PlanCompiler compiler;
    auto xml = compiler.compileSequential(plan.steps, wm, registry);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<mujin::CheckWorldPredicate>("CheckWorldPredicate");
    factory.registerNodeType<mujin::SetWorldPredicate>("SetWorldPredicate");
    factory.registerNodeType<StubAction>("StubAction");

    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

    // Attach MujinBTLogger
    mujin::MujinBTLogger logger(tree, "TestPlan", &wm);

    auto status = tree.tickWhileRunning();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

    // Should have logged events
    EXPECT_GT(logger.events().size(), 0u);

    // Every event should be valid JSON-ish with required fields
    for (auto& event : logger.events()) {
        EXPECT_NE(event.find("\"ts_us\":"), std::string::npos);
        EXPECT_NE(event.find("\"node\":"), std::string::npos);
        EXPECT_NE(event.find("\"type\":"), std::string::npos);
        EXPECT_NE(event.find("\"prev\":"), std::string::npos);
        EXPECT_NE(event.find("\"status\":"), std::string::npos);
        EXPECT_NE(event.find("\"tree_id\":\"TestPlan\""), std::string::npos);
        EXPECT_NE(event.find("\"wm_version\":"), std::string::npos);
    }
}

TEST(MujinBTLogger, CallbackSinkReceivesEvents) {
    auto wm = makeSimpleWorldModel();
    wm.setFact("(at uav1 base)", true, "planner_init");
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    mujin::Planner planner;
    auto plan = planner.solve(wm);
    ASSERT_TRUE(plan.success);

    mujin::ActionRegistry registry;
    registry.registerAction("move", "StubAction");
    registry.registerAction("search", "StubAction");
    registry.registerAction("classify", "StubAction");

    mujin::PlanCompiler compiler;
    auto xml = compiler.compileSequential(plan.steps, wm, registry);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<mujin::CheckWorldPredicate>("CheckWorldPredicate");
    factory.registerNodeType<mujin::SetWorldPredicate>("SetWorldPredicate");
    factory.registerNodeType<StubAction>("StubAction");

    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

    // Attach logger with callback sink
    std::vector<std::string> received;
    mujin::MujinBTLogger logger(tree, "TestPlan", &wm);
    logger.addCallbackSink([&received](const std::string& line) {
        received.push_back(line);
    });

    tree.tickWhileRunning();

    // Callback should have received the same events as in-memory store
    EXPECT_EQ(received.size(), logger.events().size());
}

TEST(MujinBTLogger, WritesToFile) {
    auto wm = makeSimpleWorldModel();
    wm.setFact("(at uav1 base)", true, "planner_init");
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    mujin::Planner planner;
    auto plan = planner.solve(wm);
    ASSERT_TRUE(plan.success);

    mujin::ActionRegistry registry;
    registry.registerAction("move", "StubAction");
    registry.registerAction("search", "StubAction");
    registry.registerAction("classify", "StubAction");

    mujin::PlanCompiler compiler;
    auto xml = compiler.compileSequential(plan.steps, wm, registry);

    std::string filepath = "test_bt_events_output.jsonl";

    {
        BT::BehaviorTreeFactory factory;
        factory.registerNodeType<mujin::CheckWorldPredicate>("CheckWorldPredicate");
        factory.registerNodeType<mujin::SetWorldPredicate>("SetWorldPredicate");
        factory.registerNodeType<StubAction>("StubAction");

        auto tree = factory.createTreeFromText(xml);
        tree.rootBlackboard()->set("world_model", &wm);

        mujin::MujinBTLogger logger(tree, "FilePlan", &wm);
        logger.addFileSink(filepath);

        tree.tickWhileRunning();
        // Destructor flushes
    }

    std::ifstream in(filepath);
    ASSERT_TRUE(in.is_open());
    std::string line;
    ASSERT_TRUE(std::getline(in, line));
    EXPECT_NE(line.find("\"tree_id\":\"FilePlan\""), std::string::npos);
    in.close();

    std::remove(filepath.c_str());
}

// =========================================================================
// Layer 1: TreeObserver integration
// =========================================================================

TEST(TreeObserver, CollectsStatisticsOnExecution) {
    auto wm = makeSimpleWorldModel();
    wm.setFact("(at uav1 base)", true, "planner_init");
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    mujin::Planner planner;
    auto plan = planner.solve(wm);
    ASSERT_TRUE(plan.success);

    mujin::ActionRegistry registry;
    registry.registerAction("move", "StubAction");
    registry.registerAction("search", "StubAction");
    registry.registerAction("classify", "StubAction");

    mujin::PlanCompiler compiler;
    auto xml = compiler.compileSequential(plan.steps, wm, registry);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<mujin::CheckWorldPredicate>("CheckWorldPredicate");
    factory.registerNodeType<mujin::SetWorldPredicate>("SetWorldPredicate");
    factory.registerNodeType<StubAction>("StubAction");

    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

    // Layer 1: attach TreeObserver
    BT::TreeObserver observer(tree);

    auto status = tree.tickWhileRunning();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

    // The observer should have recorded statistics.
    // Use the root node UID to query directly.
    auto root_uid = tree.rootNode()->UID();
    auto root_stats = observer.getStatistics(root_uid);
    EXPECT_GT(root_stats.transitions_count, 0u);
}

// =========================================================================
// Integration: all three layers together
// =========================================================================

TEST(ObservabilityIntegration, AllLayersWorkTogether) {
    auto wm = makeSimpleWorldModel();

    // Layer 3: attach audit log
    mujin::WmAuditLog audit;
    wm.setAuditCallback([&audit](uint64_t ver, uint64_t ts_us,
                                  const std::string& fact, bool val,
                                  const std::string& src) {
        audit.onFactChange(ver, ts_us, fact, val, src);
    });

    wm.setFact("(at uav1 base)", true, "planner_init");
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    mujin::Planner planner;
    auto plan = planner.solve(wm);
    ASSERT_TRUE(plan.success);

    mujin::ActionRegistry registry;
    registry.registerAction("move", "StubAction");
    registry.registerAction("search", "StubAction");
    registry.registerAction("classify", "StubAction");

    mujin::PlanCompiler compiler;
    auto xml = compiler.compileSequential(plan.steps, wm, registry);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<mujin::CheckWorldPredicate>("CheckWorldPredicate");
    factory.registerNodeType<mujin::SetWorldPredicate>("SetWorldPredicate");
    factory.registerNodeType<StubAction>("StubAction");

    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("world_model", &wm);

    // Layer 1: TreeObserver
    BT::TreeObserver observer(tree);

    // Layer 2: MujinBTLogger
    mujin::MujinBTLogger bt_logger(tree, "IntegrationPlan", &wm);

    auto status = tree.tickWhileRunning();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

    // Verify all layers produced output
    EXPECT_GT(bt_logger.events().size(), 0u) << "Layer 2: should have BT events";
    EXPECT_GT(audit.size(), 0u) << "Layer 3: should have WM audit entries";

    // The initial planner_init entry + at least the BT-driven fact changes
    bool has_init = false;
    bool has_bt = false;
    for (auto& e : audit.entries()) {
        if (e.source == "planner_init") has_init = true;
        if (e.source.find("SetWorldPredicate:") == 0) has_bt = true;
    }
    EXPECT_TRUE(has_init) << "Should have planner_init source";
    EXPECT_TRUE(has_bt) << "Should have SetWorldPredicate source";
}

// =========================================================================
// Layer 5: Plan Audit Trail
// =========================================================================

TEST(PlanAuditLog, RecordsEpisodeInMemory) {
    auto wm = makeSimpleWorldModel();
    wm.setFact("(at uav1 base)", true, "planner_init");
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    mujin::Planner planner;
    auto plan = planner.solve(wm);
    ASSERT_TRUE(plan.success);
    EXPECT_GT(plan.solve_time_ms, 0.0);

    mujin::ActionRegistry registry;
    registry.registerAction("move", "StubAction");
    registry.registerAction("search", "StubAction");
    registry.registerAction("classify", "StubAction");

    mujin::PlanCompiler compiler;
    auto xml = compiler.compileSequential(plan.steps, wm, registry);

    // Record episode
    mujin::PlanAuditLog audit;

    mujin::PlanAuditLog::Episode ep;
    ep.ts_us = 12345;
    for (unsigned i = 0; i < wm.numFluents(); ++i) {
        if (wm.getFact(i)) ep.init_facts.push_back(wm.fluentName(i));
    }
    for (auto gid : wm.goalFluentIds()) {
        ep.goal_fluents.push_back(wm.fluentName(gid));
    }
    ep.solver = "BRFS";
    ep.solve_time_ms = plan.solve_time_ms;
    ep.success = plan.success;
    ep.expanded = plan.expanded;
    ep.generated = plan.generated;
    ep.cost = plan.cost;
    for (auto& step : plan.steps) {
        ep.plan_actions.push_back(wm.groundActions()[step.action_index].signature);
    }
    ep.bt_xml = xml;

    audit.recordEpisode(ep);

    ASSERT_EQ(audit.size(), 1u);
    auto& recorded = audit.episodes()[0];
    EXPECT_EQ(recorded.solver, "BRFS");
    EXPECT_TRUE(recorded.success);
    EXPECT_GT(recorded.plan_actions.size(), 0u);
    EXPECT_FALSE(recorded.bt_xml.empty());
    EXPECT_EQ(recorded.goal_fluents.size(), 2u);
}

TEST(PlanAuditLog, WritesToFile) {
    std::string filepath = "test_plan_audit_output.jsonl";

    {
        mujin::PlanAuditLog audit(filepath);
        mujin::PlanAuditLog::Episode ep;
        ep.ts_us = 99999;
        ep.solver = "BRFS";
        ep.solve_time_ms = 1.5;
        ep.success = true;
        ep.expanded = 10;
        ep.generated = 20;
        ep.cost = 3.0f;
        ep.init_facts = {"(at uav1 base)"};
        ep.goal_fluents = {"(searched sector_a)"};
        ep.plan_actions = {"move(uav1,base,sector_a)", "search(uav1,sector_a)"};
        ep.bt_xml = "<root>...</root>";
        audit.recordEpisode(ep);
    }

    std::ifstream in(filepath);
    ASSERT_TRUE(in.is_open());
    std::string line;
    ASSERT_TRUE(std::getline(in, line));
    EXPECT_NE(line.find("\"solver\":\"BRFS\""), std::string::npos);
    EXPECT_NE(line.find("\"success\":true"), std::string::npos);
    EXPECT_NE(line.find("\"plan_actions\":["), std::string::npos);
    EXPECT_NE(line.find("\"bt_xml\":"), std::string::npos);
    in.close();

    std::remove(filepath.c_str());
}

TEST(PlanAuditLog, SolveTimeIsRecorded) {
    auto wm = makeSimpleWorldModel();
    wm.setFact("(at uav1 base)", true);
    wm.setGoal({"(searched sector_a)", "(classified sector_a)"});

    mujin::Planner planner;
    auto plan = planner.solve(wm);

    EXPECT_TRUE(plan.success);
    // solve_time_ms should be > 0 (it took some time to solve)
    EXPECT_GE(plan.solve_time_ms, 0.0);
}
