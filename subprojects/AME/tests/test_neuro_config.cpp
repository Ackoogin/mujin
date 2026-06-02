// Phase 4 tests: NeuroConfig, integration seams
// WI-4.1, WI-4.2

#include "ame/neuro/neuro_config.h"
#include "ame/neuro/advisor.h"
#include "ame/neuro/backend_registry.h"
#include "ame/neuro/mock_backend.h"
#include "ame/neuro/verifier.h"
#include "ame/planner.h"
#include "ame/world_model.h"

#if defined(AME_NEURO)
#include "ame/executor_component.h"
#include "ame/planner_component.h"
#endif

#include <gtest/gtest.h>

using namespace ame::neuro;

// ---------------------------------------------------------------------------
// NeuroConfig (WI-4.1)
// ---------------------------------------------------------------------------

TEST(NeuroConfig, AllDisabledByDefault) {
    auto cfg = NeuroConfig::all_disabled();
    EXPECT_FALSE(cfg.enabled);
    EXPECT_TRUE(cfg.integrations.empty());
}

TEST(NeuroConfig, PolicyForDisabledReturnsDisabled) {
    NeuroConfig cfg = NeuroConfig::all_disabled();
    auto policy = cfg.policy_for("goal_interpreter");
    EXPECT_FALSE(policy.enabled);
}

TEST(NeuroConfig, ParseJsonBasic) {
    const std::string json = R"({
        "enabled": true,
        "integrations": [
            {
                "integration_kind": "goal_interpreter",
                "enabled": true,
                "latency_budget_ms": 300.0,
                "max_retries": 1,
                "backend_id": "my_llm",
                "model_id": "v1.2"
            }
        ]
    })";
    auto cfg = NeuroConfig::from_json(json);
    EXPECT_TRUE(cfg.enabled);
    ASSERT_EQ(cfg.integrations.size(), 1u);
    EXPECT_EQ(cfg.integrations[0].integration_kind, "goal_interpreter");
    EXPECT_TRUE(cfg.integrations[0].policy.enabled);
    EXPECT_DOUBLE_EQ(cfg.integrations[0].policy.latency_budget_ms, 300.0);
    EXPECT_EQ(cfg.integrations[0].policy.max_retries, 1u);
    EXPECT_EQ(cfg.integrations[0].policy.backend_id, "my_llm");
    EXPECT_EQ(cfg.integrations[0].policy.model_id, "v1.2");
}

TEST(NeuroConfig, GlobalKillSwitchDisablesAll) {
    const std::string json = R"({
        "enabled": false,
        "integrations": [
            {
                "integration_kind": "plan_repair",
                "enabled": true,
                "latency_budget_ms": 200.0
            }
        ]
    })";
    auto cfg = NeuroConfig::from_json(json);
    auto policy = cfg.policy_for("plan_repair");
    EXPECT_FALSE(policy.enabled); // global kill-switch overrides
}

TEST(NeuroConfig, FindIntegration) {
    NeuroConfig cfg;
    cfg.enabled = true;
    IntegrationConfig ic;
    ic.integration_kind = "heuristic_guide";
    ic.policy.enabled = true;
    cfg.integrations.push_back(ic);

    const auto* found = cfg.find("heuristic_guide");
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->integration_kind, "heuristic_guide");

    EXPECT_EQ(cfg.find("nonexistent"), nullptr);
}

// ---------------------------------------------------------------------------
// WI-4.2: Integration seams — symbolic baseline identical when unused
// ---------------------------------------------------------------------------

TEST(PlannerSeam, BaselineIdenticalWithoutHook) {
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("object");
    ts.addType("location", "object");
    ts.addType("robot", "object");

    wm.addObject("bot", "robot");
    wm.addObject("a", "location");
    wm.addObject("b", "location");

    wm.registerPredicate("at", {"robot", "location"});
    wm.registerAction("move",
        {"?r", "?from", "?to"},
        {"robot", "location", "location"},
        {"(at ?r ?from)"},
        {"(at ?r ?to)"},
        {"(at ?r ?from)"});

    wm.setFact("(at bot a)", true, "test");
    wm.setGoal({"(at bot b)"});

    // Baseline: no hook
    ame::Planner p1;
    auto r1 = p1.solve(wm);
    ASSERT_TRUE(r1.success);

#if defined(AME_NEURO)
    // With hook that returns empty scores — must produce identical result
    ame::Planner p2;
    bool hook_called = false;
    p2.setHeuristicHook([&](const ame::WorldModel&, const std::vector<unsigned>&)
                            -> std::vector<ame::ActionScore> {
        hook_called = true;
        return {}; // empty = default ordering
    });
    auto r2 = p2.solve(wm);
    EXPECT_TRUE(r2.success);
    EXPECT_EQ(r1.steps.size(), r2.steps.size());
    EXPECT_TRUE(hook_called);
    EXPECT_EQ(r2.heuristic_source, "neural_hook");
#endif
}

#if defined(AME_NEURO)
TEST(PlannerSeam, HookIsCalledWhenAttached) {
    ame::WorldModel wm;
    wm.typeSystem().addType("object");
    wm.addObject("x", "object");
    wm.registerPredicate("done", {"object"});
    wm.registerAction("finish",
        {"?o"}, {"object"}, {}, {"(done ?o)"}, {});
    wm.setGoal({"(done x)"});

    std::vector<unsigned> received_goals;
    ame::Planner p;
    p.setHeuristicHook([&](const ame::WorldModel& m,
                            const std::vector<unsigned>& g)
                            -> std::vector<ame::ActionScore> {
        received_goals = g;
        return {};
    });

    auto r = p.solve(wm);
    EXPECT_TRUE(r.success);
    EXPECT_FALSE(received_goals.empty());
}

TEST(PlannerSeam, ClearHookRestoresBaseline) {
    ame::WorldModel wm;
    wm.typeSystem().addType("object");
    wm.addObject("x", "object");
    wm.registerPredicate("done", {"object"});
    wm.registerAction("finish", {"?o"}, {"object"}, {}, {"(done ?o)"}, {});
    wm.setGoal({"(done x)"});

    ame::Planner p;
    bool called = false;
    p.setHeuristicHook([&](const ame::WorldModel&, const std::vector<unsigned>&)
                           -> std::vector<ame::ActionScore> {
        called = true; return {};
    });
    p.clearHeuristicHook();
    auto r = p.solve(wm);
    EXPECT_TRUE(r.success);
    EXPECT_FALSE(called);
    EXPECT_EQ(r.heuristic_source, "symbolic");
}
#endif

// ---------------------------------------------------------------------------
// Thread 17: parser accepts optional whitespace before colons (pretty-printed JSON)
// ---------------------------------------------------------------------------

TEST(NeuroConfig, ParseJsonWithSpacesBeforeColons) {
    // Common pretty-printer output: "key" : value  (space before colon)
    const std::string json = R"({
        "enabled" : true,
        "integrations" : [
            {
                "integration_kind" : "heuristic_guide",
                "enabled" : true,
                "latency_budget_ms" : 150.0,
                "backend_id" : "fast_model"
            }
        ]
    })";
    auto cfg = NeuroConfig::from_json(json);
    EXPECT_TRUE(cfg.enabled);
    ASSERT_EQ(cfg.integrations.size(), 1u);
    EXPECT_EQ(cfg.integrations[0].integration_kind, "heuristic_guide");
    EXPECT_TRUE(cfg.integrations[0].policy.enabled);
    EXPECT_DOUBLE_EQ(cfg.integrations[0].policy.latency_budget_ms, 150.0);
    EXPECT_EQ(cfg.integrations[0].policy.backend_id, "fast_model");
}

// ---------------------------------------------------------------------------
// Thread 25: repair hook loading failure preserves FAILURE in lastStatus()
// ---------------------------------------------------------------------------

#if defined(AME_NEURO)

namespace {

// Expose on_tick() for white-box testing.
class TestableExecutorComponent : public ame::ExecutorComponent {
public:
    pcl_status_t test_tick() { return on_tick(0.0); }
};

// Minimal BT that always fails via CheckWorldPredicate on an absent fact.
const char* k_fail_bt = R"xml(
<root BTCPP_format="4">
  <BehaviorTree ID="FailPlan">
    <CheckWorldPredicate predicate="(absent_fact x)" />
  </BehaviorTree>
</root>)xml";

} // namespace

TEST(ExecutorSeam, RepairLoadFailurePreservesFailureStatus) {
    ame::WorldModel wm;
    wm.typeSystem().addType("object");
    wm.addObject("x", "object");
    wm.registerPredicate("absent_fact", {"object"});
    // (absent_fact x) is not set → CheckWorldPredicate returns FAILURE.

    TestableExecutorComponent exec;
    exec.setParam("bt_log.enabled", false);
    exec.setInProcessWorldModel(&wm);
    ASSERT_EQ(exec.configure(), PCL_OK);
    ASSERT_EQ(exec.activate(), PCL_OK);

    exec.loadAndExecute(k_fail_bt);

    // Repair hook: return syntactically invalid BT XML so loadAndExecute throws.
    exec.setRepairHook([](unsigned, const ame::WorldModel&) -> std::string {
        return "<invalid_bt_xml_that_will_throw/>";
    });

    // test_tick() drives the BT to FAILURE and invokes the repair path.
    EXPECT_EQ(exec.test_tick(), PCL_OK);

    // Even though loadAndExecute reset last_status_ to IDLE before throwing,
    // the fix must restore it to FAILURE so callers observe the correct status.
    EXPECT_EQ(exec.lastStatus(), BT::NodeStatus::FAILURE);

    exec.deactivate();
    exec.cleanup();
    exec.shutdown();
}

#endif
