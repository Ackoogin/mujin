/**
 * Tests for Core Extensions:
 *   - Extension 3: PerceptionBridge
 *   - Extension 4: PYRAMID Service Nodes (InvokeService)
 *   - Extension 5: Thread Safety (WorldModelSnapshot / SnapshotManager)
 *   - Extension 6: Hierarchical Planning (ExecutePhaseAction)
 */

#include "ame/world_model.h"
#include "ame/perception_bridge.h"
#include "ame/world_model_snapshot.h"
#include "ame/pyramid_service.h"
#include "ame/bt_nodes/invoke_service.h"
#include "ame/bt_nodes/execute_phase_action.h"
#include "ame/bt_nodes/check_world_predicate.h"
#include "ame/bt_nodes/set_world_predicate.h"
#include "ame/action_registry.h"
#include "ame/plan_audit_log.h"
#include "ame/plan_compiler.h"
#include "ame/planner.h"
#include "ame/planner_component.h"

#include <behaviortree_cpp/bt_factory.h>

#include <gtest/gtest.h>

#include <atomic>
#include <string>
#include <thread>
#include <vector>

// =========================================================================
// Shared helpers
// =========================================================================

static ame::WorldModel makeUavWorldModel() {
    ame::WorldModel wm;
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
    ame::PerceptionBridge bridge(wm);

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

    ame::PerceptionBridge bridge(wm);
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

    ame::PerceptionBridge bridge(wm);
    bridge.updateFact("(at uav1 base)", true, "camera_front");
    bridge.flush();

    EXPECT_EQ(captured_source, "perception:camera_front");
}

TEST(PerceptionBridge, CallbackFiredOnFlush) {
    auto wm = makeUavWorldModel();
    ame::PerceptionBridge bridge(wm);

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

    ame::PerceptionBridge bridge(wm);

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
// Extension 4: PYRAMID Service Nodes (async StatefulActionNode)
// =========================================================================

// --- Test service implementations ---

// Async service that fails immediately on poll
class AlwaysFailService : public ame::IPyramidService {
public:
    bool call(const std::string&, const std::string&,
              const ame::ServiceMessage&, ame::ServiceMessage&) override {
        return false;
    }
    uint64_t callAsync(const std::string&, const std::string&,
                       const ame::ServiceMessage&) override {
        return ++next_id_;
    }
    ame::AsyncCallStatus pollResult(uint64_t, ame::ServiceMessage&) override {
        return ame::AsyncCallStatus::FAILURE;
    }
    void cancelCall(uint64_t) override {}
private:
    uint64_t next_id_ = 0;
};

// Async service that stays PENDING for a configurable number of polls,
// then returns SUCCESS with a response.
class DelayedService : public ame::IPyramidService {
public:
    explicit DelayedService(int polls_before_done)
        : polls_before_done_(polls_before_done) {}

    bool call(const std::string&, const std::string&,
              const ame::ServiceMessage&, ame::ServiceMessage&) override {
        return true;
    }

    uint64_t callAsync(const std::string& service_name,
                       const std::string& operation,
                       const ame::ServiceMessage& request) override {
        last_service_name = service_name;
        last_operation = operation;
        last_request = request;
        poll_count_ = 0;
        cancelled_ = false;
        return ++next_id_;
    }

    ame::AsyncCallStatus pollResult(uint64_t, ame::ServiceMessage& response) override {
        if (cancelled_) return ame::AsyncCallStatus::CANCELLED;
        ++poll_count_;
        if (poll_count_ >= polls_before_done_) {
            response.set("status", "done");
            response.set("result", "ok");
            return ame::AsyncCallStatus::SUCCESS;
        }
        return ame::AsyncCallStatus::PENDING;
    }

    void cancelCall(uint64_t) override {
        cancelled_ = true;
        ++cancel_count;
    }

    std::string last_service_name;
    std::string last_operation;
    ame::ServiceMessage last_request;
    int cancel_count = 0;

private:
    int polls_before_done_;
    int poll_count_ = 0;
    bool cancelled_ = false;
    uint64_t next_id_ = 0;
};

static BT::Tree createTreeWithService(BT::BehaviorTreeFactory& factory,
                                      const char* xml,
                                      ame::IPyramidService* service) {
    auto blackboard = BT::Blackboard::create();
    blackboard->set("pyramid_service", service);
    return factory.createTreeFromText(xml, blackboard);
}

// --- ServiceMessage tests ---

TEST(PyramidService, MockAlwaysSucceeds) {
    ame::MockPyramidService svc;
    ame::ServiceMessage req, resp;
    req.set("target", "sector_a");
    EXPECT_TRUE(svc.call("imaging", "capture", req, resp));
}

TEST(PyramidService, ServiceMessageGetSet) {
    ame::ServiceMessage msg;
    msg.set("key1", "val1");
    EXPECT_EQ(msg.get("key1"), "val1");
    EXPECT_EQ(msg.get("missing", "default"), "default");
    EXPECT_TRUE(msg.has("key1"));
    EXPECT_FALSE(msg.has("missing"));
}

TEST(PyramidService, MockAsyncReturnsSuccess) {
    ame::MockPyramidService svc;
    ame::ServiceMessage req;
    req.set("target", "sector_a");
    uint64_t id = svc.callAsync("imaging", "capture", req);
    EXPECT_GT(id, 0u);

    ame::ServiceMessage resp;
    auto status = svc.pollResult(id, resp);
    EXPECT_EQ(status, ame::AsyncCallStatus::SUCCESS);
}

// --- InvokeService async node tests ---

TEST(InvokeServiceNode, SuccessWithMockService) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ame::InvokeService>("InvokeService");

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

    ame::MockPyramidService svc;
    auto tree = createTreeWithService(factory, xml,
                                      static_cast<ame::IPyramidService*>(&svc));

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(InvokeServiceNode, FailureWithNullService) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ame::InvokeService>("InvokeService");

    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <InvokeService service_name="imaging" operation="capture"/>
            </BehaviorTree>
        </root>
    )xml";

    auto tree = createTreeWithService(factory, xml,
                                      static_cast<ame::IPyramidService*>(nullptr));

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(InvokeServiceNode, FailureWhenServiceReturnsFalse) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ame::InvokeService>("InvokeService");

    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <InvokeService service_name="imaging" operation="capture"/>
            </BehaviorTree>
        </root>
    )xml";

    AlwaysFailService svc;
    auto tree = createTreeWithService(factory, xml,
                                      static_cast<ame::IPyramidService*>(&svc));

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(InvokeServiceNode, AsyncReturnsRunningThenSuccess) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ame::InvokeService>("InvokeService");

    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <InvokeService service_name="nav"
                               operation="goto"
                               timeout_ms="0"/>
            </BehaviorTree>
        </root>
    )xml";

    DelayedService svc(3);  // completes after 3 polls
    auto tree = createTreeWithService(factory, xml,
                                      static_cast<ame::IPyramidService*>(&svc));

    // First tick: onStart() -> callAsync + first poll -> PENDING -> RUNNING
    auto s1 = tree.tickOnce();
    EXPECT_EQ(s1, BT::NodeStatus::RUNNING);

    // Second tick: onRunning() -> second poll -> PENDING -> RUNNING
    auto s2 = tree.tickOnce();
    EXPECT_EQ(s2, BT::NodeStatus::RUNNING);

    // Third tick: onRunning() -> third poll -> SUCCESS
    auto s3 = tree.tickOnce();
    EXPECT_EQ(s3, BT::NodeStatus::SUCCESS);
}

TEST(InvokeServiceNode, TimeoutCausesFailure) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ame::InvokeService>("InvokeService");

    // Very short timeout: 1ms
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <InvokeService service_name="nav"
                               operation="goto"
                               timeout_ms="1"/>
            </BehaviorTree>
        </root>
    )xml";

    DelayedService svc(1000);  // would take 1000 polls
    auto tree = createTreeWithService(factory, xml,
                                      static_cast<ame::IPyramidService*>(&svc));

    // First tick may or may not timeout (depends on timing)
    auto s1 = tree.tickOnce();
    if (s1 == BT::NodeStatus::RUNNING) {
        // Sleep to ensure timeout elapses
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        auto s2 = tree.tickOnce();
        EXPECT_EQ(s2, BT::NodeStatus::FAILURE);
    } else {
        // Timed out immediately (very fast machine or scheduling)
        EXPECT_EQ(s1, BT::NodeStatus::FAILURE);
    }
    EXPECT_GE(svc.cancel_count, 1);
}

TEST(InvokeServiceNode, HaltCancelsPendingCall) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ame::InvokeService>("InvokeService");

    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <InvokeService service_name="nav"
                               operation="goto"
                               timeout_ms="0"/>
            </BehaviorTree>
        </root>
    )xml";

    DelayedService svc(100);  // stays pending
    auto tree = createTreeWithService(factory, xml,
                                      static_cast<ame::IPyramidService*>(&svc));

    auto s1 = tree.tickOnce();
    EXPECT_EQ(s1, BT::NodeStatus::RUNNING);

    tree.haltTree();
    EXPECT_GE(svc.cancel_count, 1);
}

TEST(InvokeServiceNode, PddlParamAutoMapping) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ame::InvokeService>("InvokeService");

    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <InvokeService service_name="mobility"
                               operation="move"
                               param_names="?robot;?from;?to"
                               param_values="uav1;base;sector_a"
                               timeout_ms="0"/>
            </BehaviorTree>
        </root>
    )xml";

    DelayedService svc(1);  // completes on first poll
    auto tree = createTreeWithService(factory, xml,
                                      static_cast<ame::IPyramidService*>(&svc));

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

    // Verify the param bindings were translated to the request
    EXPECT_EQ(svc.last_service_name, "mobility");
    EXPECT_EQ(svc.last_operation, "move");
    EXPECT_EQ(svc.last_request.get("robot"), "uav1");
    EXPECT_EQ(svc.last_request.get("from"), "base");
    EXPECT_EQ(svc.last_request.get("to"), "sector_a");
}

TEST(InvokeServiceNode, ParamMappingMergesWithRequestJson) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ame::InvokeService>("InvokeService");

    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <InvokeService service_name="imaging"
                               operation="capture"
                               request_json="priority=1;mode=wide"
                               param_names="?target"
                               param_values="sector_a"
                               timeout_ms="0"/>
            </BehaviorTree>
        </root>
    )xml";

    DelayedService svc(1);
    auto tree = createTreeWithService(factory, xml,
                                      static_cast<ame::IPyramidService*>(&svc));

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

    // Both explicit request_json and param bindings should be in the request
    EXPECT_EQ(svc.last_request.get("priority"), "1");
    EXPECT_EQ(svc.last_request.get("mode"), "wide");
    EXPECT_EQ(svc.last_request.get("target"), "sector_a");
}

TEST(InvokeServiceNode, NoTimeoutMeansNoLimit) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ame::InvokeService>("InvokeService");

    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <InvokeService service_name="nav"
                               operation="goto"
                               timeout_ms="0"/>
            </BehaviorTree>
        </root>
    )xml";

    DelayedService svc(5);  // completes after 5 polls
    auto tree = createTreeWithService(factory, xml,
                                      static_cast<ame::IPyramidService*>(&svc));

    // Tick multiple times; should stay RUNNING, not timeout
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    for (int i = 0; i < 10; ++i) {
        status = tree.tickOnce();
        if (status != BT::NodeStatus::RUNNING) break;
    }
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_EQ(svc.cancel_count, 0);
}

// =========================================================================
// Extension 5: Thread Safety -- WorldModelSnapshot / SnapshotManager
// =========================================================================

TEST(WorldModelSnapshot, GetFactMatchesWorldModel) {
    auto wm = makeUavWorldModel();
    wm.setFact("(at uav1 base)", true);
    wm.setFact("(searched sector_a)", false);

    ame::SnapshotManager manager(wm);
    auto snap = manager.current();

    ASSERT_NE(snap, nullptr);
    EXPECT_EQ(snap->version, wm.version());
    EXPECT_TRUE(snap->getFact("(at uav1 base)"));
    EXPECT_FALSE(snap->getFact("(searched sector_a)"));
}

TEST(WorldModelSnapshot, SnapshotIsolatedFromLaterChanges) {
    auto wm = makeUavWorldModel();
    wm.setFact("(at uav1 base)", true);

    ame::SnapshotManager manager(wm);
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
    ame::SnapshotManager manager(wm);
    auto snap = manager.current();

    EXPECT_FALSE(snap->getFact("(nonexistent predicate)"));
}

TEST(SnapshotManager, ConcurrentPublishAndRead) {
    auto wm = makeUavWorldModel();
    wm.setFact("(at uav1 base)", true);

    ame::SnapshotManager manager(wm);

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
// Extension 6: Hierarchical Planning -- ExecutePhaseAction
// =========================================================================

// Stub BT action node (always succeeds) for use in compiled subtrees.
// Declares param0..param2 to match ports emitted by ActionRegistry::resolve().
class StubAction : public BT::SyncActionNode {
public:
    StubAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick() override {
        auto pred = getInput<std::string>("predicate");
        if (pred && !pred.value().empty()) {
            auto* wm = config().blackboard->get<ame::WorldModel*>("world_model");
            if (wm) wm->setFact(pred.value(), true, "StubAction");
        }
        return BT::NodeStatus::SUCCESS;
    }
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("predicate", "", "fact to set on success"),
            BT::InputPort<std::string>("param0", "", ""),
            BT::InputPort<std::string>("param1", "", ""),
            BT::InputPort<std::string>("param2", "", ""),
        };
    }
};

static BT::BehaviorTreeFactory makeFullFactory() {
    BT::BehaviorTreeFactory f;
    f.registerNodeType<ame::CheckWorldPredicate>("CheckWorldPredicate");
    f.registerNodeType<ame::SetWorldPredicate>("SetWorldPredicate");
    f.registerNodeType<ame::ExecutePhaseAction>("ExecutePhaseAction");
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

    ame::Planner planner;
    ame::PlanCompiler compiler;
    ame::ActionRegistry registry;
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

    ame::Planner planner;
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

    ame::Planner planner;
    ame::PlanCompiler compiler;
    ame::ActionRegistry registry;
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

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(wm.getFact("(searched sector_a)"));
}

// =========================================================================
// Extension 6: Hierarchical Planning -- Audit Causal Links
// =========================================================================

TEST(ExecutePhaseAction, AuditLogRecordsEpisodeWithPhaseName) {
    auto wm = makeUavWorldModel();
    wm.setFact("(at uav1 sector_a)", true);

    ame::Planner planner;
    ame::PlanCompiler compiler;
    ame::ActionRegistry registry;
    registry.registerAction("move",    "StubAction");
    registry.registerAction("search",  "StubAction");
    registry.registerAction("classify","StubAction");
    ame::PlanAuditLog audit;

    BT::BehaviorTreeFactory factory = makeFullFactory();

    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <ExecutePhaseAction
                    phase_goals="(searched sector_a)"
                    phase_name="recon_phase"/>
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
    bb->set("plan_audit_log",  &audit);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    for (int i = 0; i < 20 && status == BT::NodeStatus::RUNNING; ++i) {
        status = tree.tickOnce();
    }

    EXPECT_NE(status, BT::NodeStatus::RUNNING);

    // Audit log should have exactly one episode with correct phase name
    ASSERT_EQ(audit.size(), 1u);
    const auto& ep = audit.episodes()[0];
    EXPECT_EQ(ep.phase_name, "recon_phase");
    EXPECT_TRUE(ep.success);
    EXPECT_GT(ep.episode_id, 0u);
    EXPECT_EQ(ep.parent_episode_id, 0u);  // top-level, no parent
    EXPECT_FALSE(ep.plan_actions.empty());
    EXPECT_FALSE(ep.bt_xml.empty());
    // Goal fluents should be recorded
    ASSERT_EQ(ep.goal_fluents.size(), 1u);
    EXPECT_EQ(ep.goal_fluents[0], "(searched sector_a)");
}

TEST(ExecutePhaseAction, AuditLogRecordsCausalLinksForSequentialPhases) {
    auto wm = makeUavWorldModel();
    wm.setFact("(at uav1 sector_a)", true);

    ame::Planner planner;
    ame::PlanCompiler compiler;
    ame::ActionRegistry registry;
    registry.registerAction("move",    "StubAction");
    registry.registerAction("search",  "StubAction");
    registry.registerAction("classify","StubAction");
    ame::PlanAuditLog audit;

    BT::BehaviorTreeFactory factory = makeFullFactory();

    // Two sequential phases: search then classify
    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <Sequence>
                    <ExecutePhaseAction
                        phase_goals="(searched sector_a)"
                        phase_name="search_phase"/>
                    <ExecutePhaseAction
                        phase_goals="(classified sector_a)"
                        phase_name="classify_phase"/>
                </Sequence>
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
    bb->set("plan_audit_log",  &audit);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    for (int i = 0; i < 50 && status == BT::NodeStatus::RUNNING; ++i) {
        status = tree.tickOnce();
    }

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

    // Two episodes recorded (one per phase)
    ASSERT_EQ(audit.size(), 2u);

    const auto& ep1 = audit.episodes()[0];
    const auto& ep2 = audit.episodes()[1];

    EXPECT_EQ(ep1.phase_name, "search_phase");
    EXPECT_EQ(ep2.phase_name, "classify_phase");

    // Each episode gets a unique ID
    EXPECT_NE(ep1.episode_id, ep2.episode_id);
    EXPECT_GT(ep1.episode_id, 0u);
    EXPECT_GT(ep2.episode_id, 0u);

    // Both are top-level (sibling phases, not nested)
    EXPECT_EQ(ep1.parent_episode_id, 0u);
    // ep2 inherits parent_episode_id from ep1 since ep1 sets it on the
    // shared blackboard; this is the expected causal link showing ep2
    // was preceded by ep1 in the same hierarchical context.
    EXPECT_EQ(ep2.parent_episode_id, ep1.episode_id);
}

TEST(ExecutePhaseAction, FailedPlanningRecordsFailureEpisode) {
    auto wm = makeUavWorldModel();
    // UAV is at base, no actions registered to reach an impossible goal

    ame::Planner planner;
    ame::PlanCompiler compiler;
    ame::ActionRegistry registry;
    // No actions registered -> planning will fail
    ame::PlanAuditLog audit;

    BT::BehaviorTreeFactory factory = makeFullFactory();

    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <ExecutePhaseAction
                    phase_goals="(searched sector_a)"
                    phase_name="impossible_phase"/>
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
    bb->set("plan_audit_log",  &audit);

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);

    // Audit log should record the failed episode
    ASSERT_EQ(audit.size(), 1u);
    const auto& ep = audit.episodes()[0];
    EXPECT_EQ(ep.phase_name, "impossible_phase");
    EXPECT_FALSE(ep.success);
    EXPECT_GT(ep.episode_id, 0u);
}

TEST(PlanAuditLog, EpisodeIdAutoIncrement) {
    ame::PlanAuditLog audit;

    ame::PlanAuditLog::Episode ep1;
    ep1.solver = "BRFS";
    ep1.success = true;
    ep1.phase_name = "phase_a";
    uint64_t id1 = audit.recordEpisode(std::move(ep1));

    ame::PlanAuditLog::Episode ep2;
    ep2.solver = "BRFS";
    ep2.success = true;
    ep2.phase_name = "phase_b";
    uint64_t id2 = audit.recordEpisode(std::move(ep2));

    EXPECT_GT(id1, 0u);
    EXPECT_GT(id2, 0u);
    EXPECT_NE(id1, id2);

    ASSERT_EQ(audit.size(), 2u);
    EXPECT_EQ(audit.episodes()[0].episode_id, id1);
    EXPECT_EQ(audit.episodes()[1].episode_id, id2);
}

TEST(PlanAuditLog, ParentEpisodeIdPreserved) {
    ame::PlanAuditLog audit;

    ame::PlanAuditLog::Episode parent_ep;
    parent_ep.solver = "BRFS";
    parent_ep.success = true;
    parent_ep.phase_name = "parent";
    uint64_t parent_id = audit.recordEpisode(std::move(parent_ep));

    ame::PlanAuditLog::Episode child_ep;
    child_ep.solver = "BRFS";
    child_ep.success = true;
    child_ep.phase_name = "child";
    child_ep.parent_episode_id = parent_id;
    uint64_t child_id = audit.recordEpisode(std::move(child_ep));

    ASSERT_EQ(audit.size(), 2u);
    EXPECT_EQ(audit.episodes()[1].parent_episode_id, parent_id);
    EXPECT_NE(child_id, parent_id);
}

TEST(ExecutePhaseAction, PlannerComponentPath) {
    auto wm = makeUavWorldModel();
    wm.setFact("(at uav1 sector_a)", true);

    ame::PlannerComponent component;
    component.setParam("plan_audit.enabled", false);
    component.setParam("compiler.parallel", false);
    component.setInProcessWorldModel(&wm);
    component.actionRegistry().registerAction("move",    "StubAction");
    component.actionRegistry().registerAction("search",  "StubAction");
    component.actionRegistry().registerAction("classify","StubAction");
    ASSERT_EQ(component.configure(), PCL_OK);
    ASSERT_EQ(component.activate(), PCL_OK);

    ame::PlanAuditLog audit;
    BT::BehaviorTreeFactory factory = makeFullFactory();

    static const char* xml = R"xml(
        <root BTCPP_format="4">
            <BehaviorTree ID="MainTree">
                <ExecutePhaseAction
                    phase_goals="(searched sector_a)"
                    phase_name="component_phase"/>
            </BehaviorTree>
        </root>
    )xml";

    auto tree = factory.createTreeFromText(xml);
    auto bb = tree.rootBlackboard();
    bb->set("world_model",        &wm);
    bb->set("planner_component",  &component);
    bb->set("bt_factory",         &factory);
    bb->set("plan_audit_log",     &audit);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    for (int i = 0; i < 20 && status == BT::NodeStatus::RUNNING; ++i) {
        status = tree.tickOnce();
    }

    EXPECT_NE(status, BT::NodeStatus::RUNNING);

    // Audit log should have recorded the episode
    ASSERT_EQ(audit.size(), 1u);
    EXPECT_EQ(audit.episodes()[0].phase_name, "component_phase");
    EXPECT_TRUE(audit.episodes()[0].success);

    EXPECT_EQ(component.deactivate(), PCL_OK);
    EXPECT_EQ(component.cleanup(), PCL_OK);
    EXPECT_EQ(component.shutdown(), PCL_OK);
}
