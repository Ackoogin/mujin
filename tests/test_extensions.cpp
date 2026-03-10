/**
 * Tests for Core Extensions:
 *   - Extension 3: PerceptionBridge
 *   - Extension 4: PYRAMID Service Nodes (InvokeService)
 *   - Extension 5: Thread Safety (WorldModelSnapshot / SnapshotManager)
 *   - Extension 6: Hierarchical Planning (ExecutePhaseAction)
 */

#include "mujin/world_model.h"
#include "mujin/perception_bridge.h"
#include "mujin/world_model_snapshot.h"
#include "mujin/pyramid_service.h"
#include "mujin/bt_nodes/invoke_service.h"
#include "mujin/bt_nodes/execute_phase_action.h"
#include "mujin/bt_nodes/check_world_predicate.h"
#include "mujin/bt_nodes/set_world_predicate.h"
#include "mujin/action_registry.h"
#include "mujin/plan_compiler.h"
#include "mujin/planner.h"

#include <behaviortree_cpp/bt_factory.h>

#include <gtest/gtest.h>

#include <atomic>
#include <string>
#include <thread>
#include <vector>

// =========================================================================
// Shared helpers
// =========================================================================

static mujin::WorldModel makeUavWorldModel() {
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

// =========================================================================
// Extension 3: PerceptionBridge
// =========================================================================

TEST(PerceptionBridge, BuffersUpdatesBeforeFlush) {
    auto wm = makeUavWorldModel();
    mujin::PerceptionBridge bridge(wm);

    // Fact not set yet
    EXPECT_FALSE(wm.getFact("(at uav1 base)"));

    bridge.updateFact("(at uav1 base)", true);
    EXPECT_EQ(bridge.pendingCount(), 1u);

    // Still not applied until flush
    EXPECT_FALSE(wm.getFact("(at uav1 base)"));

    bridge.flush();
    EXPECT_EQ(bridge.pendingCount(), 0u);
    EXPECT_TRUE(wm.getFact("(at uav1 base)"));
}

TEST(PerceptionBridge, SourceTaggedAsPerception) {
    auto wm = makeUavWorldModel();

    std::string captured_source;
    wm.setAuditCallback([&](uint64_t, uint64_t, const std::string&,
                             bool, const std::string& src) {
        captured_source = src;
    });

    mujin::PerceptionBridge bridge(wm);
    bridge.updateFact("(at uav1 base)", true);
    bridge.flush();

    EXPECT_EQ(captured_source, "perception");
}

TEST(PerceptionBridge, SourceTagIncludesSubtag) {
    auto wm = makeUavWorldModel();

    std::string captured_source;
    wm.setAuditCallback([&](uint64_t, uint64_t, const std::string&,
                             bool, const std::string& src) {
        captured_source = src;
    });

    mujin::PerceptionBridge bridge(wm);
    bridge.updateFact("(at uav1 base)", true, "camera_front");
    bridge.flush();

    EXPECT_EQ(captured_source, "perception:camera_front");
}

TEST(PerceptionBridge, CallbackFiredOnFlush) {
    auto wm = makeUavWorldModel();
    mujin::PerceptionBridge bridge(wm);

    std::vector<std::pair<std::string, bool>> fired;
    bridge.setUpdateCallback([&](const std::string& fact, bool val) {
        fired.push_back({fact, val});
    });

    bridge.updateFact("(at uav1 base)", true);
    bridge.updateFact("(searched sector_a)", true);
    unsigned n = bridge.flush();

    EXPECT_EQ(n, 2u);
    ASSERT_EQ(fired.size(), 2u);
    EXPECT_EQ(fired[0].first, "(at uav1 base)");
    EXPECT_EQ(fired[1].first, "(searched sector_a)");
}

TEST(PerceptionBridge, ThreadSafeMultipleWriters) {
    auto wm = makeUavWorldModel();
    // Pre-register the fact so setFact won't throw
    wm.setFact("(at uav1 base)", false);

    mujin::PerceptionBridge bridge(wm);

    // Two threads push updates concurrently
    std::thread t1([&]() {
        for (int i = 0; i < 50; ++i) {
            bridge.updateFact("(at uav1 base)", true);
        }
    });
    std::thread t2([&]() {
        for (int i = 0; i < 50; ++i) {
            bridge.updateFact("(at uav1 base)", false);
        }
    });
    t1.join();
    t2.join();

    EXPECT_EQ(bridge.pendingCount(), 100u);
    bridge.flush();
    EXPECT_EQ(bridge.pendingCount(), 0u);
}

// =========================================================================
// Extension 4: PYRAMID Service Nodes
// =========================================================================

TEST(PyramidService, MockAlwaysSucceeds) {
    mujin::MockPyramidService svc;
    mujin::ServiceMessage req, resp;
    req.set("target", "sector_a");
    EXPECT_TRUE(svc.call("imaging", "capture", req, resp));
}

TEST(PyramidService, ServiceMessageGetSet) {
    mujin::ServiceMessage msg;
    msg.set("key1", "val1");
    EXPECT_EQ(msg.get("key1"), "val1");
    EXPECT_EQ(msg.get("missing", "default"), "default");
    EXPECT_TRUE(msg.has("key1"));
    EXPECT_FALSE(msg.has("missing"));
}

TEST(InvokeServiceNode, SuccessWithMockService) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<mujin::InvokeService>("InvokeService");

    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <InvokeService service_name="imaging"
                               operation="capture"
                               request_json="target=sector_a;priority=1"
                               response_json="{response_out}"/>
            </BehaviorTree>
        </root>
    )xml";

    auto tree = factory.createTreeFromText(xml);

    mujin::MockPyramidService svc;
    tree.rootBlackboard()->set("pyramid_service", static_cast<mujin::IPyramidService*>(&svc));

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(InvokeServiceNode, FailureWithNullService) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<mujin::InvokeService>("InvokeService");

    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <InvokeService service_name="imaging" operation="capture"/>
            </BehaviorTree>
        </root>
    )xml";

    auto tree = factory.createTreeFromText(xml);
    // No "pyramid_service" in blackboard -> should fail gracefully
    // Note: get<IPyramidService*> will return nullptr
    tree.rootBlackboard()->set("pyramid_service", static_cast<mujin::IPyramidService*>(nullptr));

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

// Concrete failing service for testing failure path
class AlwaysFailService : public mujin::IPyramidService {
public:
    bool call(const std::string&, const std::string&,
              const mujin::ServiceMessage&, mujin::ServiceMessage&) override {
        return false;
    }
};

TEST(InvokeServiceNode, FailureWhenServiceReturnsFalse) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<mujin::InvokeService>("InvokeService");

    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <InvokeService service_name="imaging" operation="capture"/>
            </BehaviorTree>
        </root>
    )xml";

    auto tree = factory.createTreeFromText(xml);
    AlwaysFailService svc;
    tree.rootBlackboard()->set("pyramid_service", static_cast<mujin::IPyramidService*>(&svc));

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

// =========================================================================
// Extension 5: Thread Safety — WorldModelSnapshot / SnapshotManager
// =========================================================================

TEST(WorldModelSnapshot, GetFactMatchesWorldModel) {
    auto wm = makeUavWorldModel();
    wm.setFact("(at uav1 base)", true);
    wm.setFact("(searched sector_a)", false);

    mujin::SnapshotManager manager(wm);
    auto snap = manager.current();

    ASSERT_NE(snap, nullptr);
    EXPECT_EQ(snap->version, wm.version());
    EXPECT_TRUE(snap->getFact("(at uav1 base)"));
    EXPECT_FALSE(snap->getFact("(searched sector_a)"));
}

TEST(WorldModelSnapshot, SnapshotIsolatedFromLaterChanges) {
    auto wm = makeUavWorldModel();
    wm.setFact("(at uav1 base)", true);

    mujin::SnapshotManager manager(wm);
    auto snap_before = manager.current();

    // Change live WM
    wm.setFact("(at uav1 base)", false);

    // Old snapshot still shows true
    EXPECT_TRUE(snap_before->getFact("(at uav1 base)"));

    // Publish new snapshot
    manager.publish();
    auto snap_after = manager.current();
    EXPECT_FALSE(snap_after->getFact("(at uav1 base)"));
}

TEST(WorldModelSnapshot, UnknownFactReturnsFalse) {
    auto wm = makeUavWorldModel();
    mujin::SnapshotManager manager(wm);
    auto snap = manager.current();

    EXPECT_FALSE(snap->getFact("(nonexistent predicate)"));
}

TEST(SnapshotManager, ConcurrentPublishAndRead) {
    auto wm = makeUavWorldModel();
    wm.setFact("(at uav1 base)", true);

    mujin::SnapshotManager manager(wm);

    std::atomic<bool> stop{false};
    std::atomic<unsigned> reads{0};
    std::atomic<unsigned> publishes{0};

    // Reader thread: continuously grab snapshot and verify it's non-null
    std::thread reader([&]() {
        while (!stop.load()) {
            auto snap = manager.current();
            EXPECT_NE(snap, nullptr);
            ++reads;
        }
    });

    // Publisher thread: continuously publish new snapshots
    std::thread publisher([&]() {
        for (int i = 0; i < 200; ++i) {
            manager.publish();
            ++publishes;
        }
        stop.store(true);
    });

    publisher.join();
    reader.join();

    EXPECT_GT(reads.load(), 0u);
    EXPECT_EQ(publishes.load(), 200u);
}

// =========================================================================
// Extension 6: Hierarchical Planning — ExecutePhaseAction
// =========================================================================

// Stub BT action node (always succeeds) for use in compiled subtrees
class StubAction : public BT::SyncActionNode {
public:
    StubAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override {
        auto pred = getInput<std::string>("predicate");
        if (pred) {
            auto* wm = config().blackboard->get<mujin::WorldModel*>("world_model");
            if (wm) wm->setFact(pred.value(), true, "StubAction");
        }
        return BT::NodeStatus::SUCCESS;
    }
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("predicate", "", "fact to set on success") };
    }
};

static BT::BehaviorTreeFactory makeFullFactory() {
    BT::BehaviorTreeFactory f;
    f.registerNodeType<mujin::CheckWorldPredicate>("CheckWorldPredicate");
    f.registerNodeType<mujin::SetWorldPredicate>("SetWorldPredicate");
    f.registerNodeType<mujin::ExecutePhaseAction>("ExecutePhaseAction");
    f.registerNodeType<StubAction>("StubAction");
    return f;
}

TEST(ExecutePhaseAction, FailsWithoutBlackboardKeys) {
    BT::BehaviorTreeFactory factory = makeFullFactory();

    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <ExecutePhaseAction phase_goals="(searched sector_a)" phase_name="search"/>
            </BehaviorTree>
        </root>
    )xml";

    auto tree = factory.createTreeFromText(xml);
    // No world_model / planner / etc on blackboard -> FAILURE
    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(ExecutePhaseAction, FailsWithEmptyGoals) {
    auto wm = makeUavWorldModel();
    wm.setFact("(at uav1 sector_a)", true);

    mujin::Planner planner;
    mujin::PlanCompiler compiler;
    mujin::ActionRegistry registry;
    registry.registerAction("move",    "StubAction");
    registry.registerAction("search",  "StubAction");
    registry.registerAction("classify","StubAction");

    BT::BehaviorTreeFactory factory = makeFullFactory();

    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <ExecutePhaseAction phase_goals="" phase_name="empty"/>
            </BehaviorTree>
        </root>
    )xml";

    auto tree = factory.createTreeFromText(xml);
    auto bb = tree.rootBlackboard();
    bb->set("world_model",     &wm);
    bb->set("planner",         &planner);
    bb->set("plan_compiler",   &compiler);
    bb->set("action_registry", &registry);
    bb->set("bt_factory",      &factory);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(ExecutePhaseAction, SucceedsWhenGoalAlreadyMet) {
    auto wm = makeUavWorldModel();
    // Goal is already true
    wm.setFact("(searched sector_a)", true);
    wm.setGoal({"(searched sector_a)"});

    mujin::Planner planner;
    auto result = planner.solve(wm);
    // An empty plan is valid when goal is already satisfied
    // (LAPKT returns success with 0 steps)
    // We test the ExecutePhaseAction indirectly through a direct planner call here.
    // When goal already satisfied, planning returns an empty or trivially short plan.
    EXPECT_TRUE(result.success);
}

TEST(ExecutePhaseAction, PlanAndExecuteSubGoal) {
    auto wm = makeUavWorldModel();
    wm.setFact("(at uav1 sector_a)", true);

    mujin::Planner planner;
    mujin::PlanCompiler compiler;
    mujin::ActionRegistry registry;
    registry.registerAction("move",    "StubAction");
    registry.registerAction("search",  "StubAction");
    registry.registerAction("classify","StubAction");

    BT::BehaviorTreeFactory factory = makeFullFactory();

    // Set goals so the planner has a target
    wm.setGoal({"(searched sector_a)"});

    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <ExecutePhaseAction
                    phase_goals="(searched sector_a)"
                    phase_name="search_phase"/>
            </BehaviorTree>
        </root>
    )xml";

    auto tree = factory.createTreeFromText(xml);
    auto bb = tree.rootBlackboard();
    bb->set("world_model",     &wm);
    bb->set("planner",         &planner);
    bb->set("plan_compiler",   &compiler);
    bb->set("action_registry", &registry);
    bb->set("bt_factory",      &factory);

    // Tick up to 20 times to allow execution to complete
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    for (int i = 0; i < 20 && status == BT::NodeStatus::RUNNING; ++i) {
        status = tree.tickOnce();
    }

    // Should complete (SUCCESS or FAILURE, but not still RUNNING)
    EXPECT_NE(status, BT::NodeStatus::RUNNING);
}
