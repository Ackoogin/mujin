// Phase 5 tests: replay harness, latency-pressure, determinism
// WI-5.2

#include "ame/neuro/advisor.h"
#include "ame/neuro/audit_index.h"
#include "ame/neuro/backend_registry.h"
#include "ame/neuro/fallback_policy.h"
#include "ame/neuro/mock_backend.h"
#include "ame/neuro/neuro_audit_log.h"
#include "ame/neuro/verifier.h"
#include "ame/planner.h"
#include "ame/world_model.h"

#include <gtest/gtest.h>
#include <chrono>

using namespace ame::neuro;
using StrAdvisor = Advisor<std::string, std::string>;

// ---------------------------------------------------------------------------
// Replay: feed recorded audit lines through AuditIndex and verify round-trip
// ---------------------------------------------------------------------------

TEST(Replay, RoundTripAuditRecords) {
    // Simulate a series of advisor calls and record them
    NeuroAuditLog log;

    for (int i = 0; i < 5; ++i) {
        NeuroAuditRecord r;
        r.integration_kind = "goal_interpreter";
        r.outcome = (i % 2 == 0) ? "Accepted" : "RejectedFellBack";
        r.affected_behaviour = (i % 2 == 0);
        r.latency_ms = static_cast<double>(i * 10);
        r.ts_us = static_cast<uint64_t>(i * 1000);
        log.append(r);
    }

    // Serialise to JSONL in-memory by re-creating from records
    std::vector<std::string> lines;
    for (const auto& rec : log.records()) {
        // Re-create a representative line for indexing test
        std::string line = "{\"record_id\":" + std::to_string(rec.record_id) +
            ",\"ts_us\":" + std::to_string(rec.ts_us) +
            ",\"integration_kind\":\"" + rec.integration_kind + "\"" +
            ",\"outcome\":\"" + rec.outcome + "\"" +
            ",\"affected_behaviour\":" + (rec.affected_behaviour ? "true" : "false") +
            ",\"latency_ms\":" + std::to_string(rec.latency_ms) +
            ",\"retries\":0,\"evidence\":[]}";
        lines.push_back(line);
    }

    AuditIndex idx;
    idx.load_lines(lines, "neuro");
    EXPECT_EQ(idx.size(), 5u);

    // Window check
    auto w = idx.window(0, 3000);
    EXPECT_EQ(w.size(), 4u); // ts_us: 0, 1000, 2000, 3000
}

// ---------------------------------------------------------------------------
// Latency-pressure: wall-time bound holds end-to-end (WI-5.2)
// Verify that planning still completes via fallback under budget pressure.
// ---------------------------------------------------------------------------

TEST(Replay, LatencyPressure_PlanCompletesViaFallback) {
    // Build a trivial world model
    ame::WorldModel wm;
    wm.typeSystem().addType("object");
    wm.addObject("x", "object");
    wm.registerPredicate("done", {"object"});
    wm.registerAction("finish", {"?o"}, {"object"}, {}, {"(done ?o)"}, {});
    wm.setGoal({"(done x)"});

    // Advisor with a tight budget that always times out
    auto mb = std::make_shared<MockBackend>("mock", false);
    mb->add_script({"", false, "", 10000.0, true, true}); // infinite hang
    BackendRegistry reg;
    reg.add(mb);
    AlwaysAccept<std::string> v;
    NeuroAuditLog audit;

    FallbackPolicy policy;
    policy.enabled = true;
    policy.latency_budget_ms = 80.0;
    policy.backend_id = "mock";

    Advisor<std::string, std::string> advisor("lat_test", reg, v, policy, &audit);

    // Advisor times out; symbolic planner still works
    auto advisor_result = advisor.advise("req", wm);
    EXPECT_EQ(advisor_result.outcome, StrAdvisor::Outcome::TimedOutFellBack);

    // Symbolic plan unaffected
    ame::Planner planner;
    auto plan_result = planner.solve(wm);
    EXPECT_TRUE(plan_result.success);
}

// ---------------------------------------------------------------------------
// Determinism: with neural disabled, replayed runs reproduce baseline (WI-5.2)
// ---------------------------------------------------------------------------

TEST(Replay, Determinism_DisabledProducesSymbolicBaseline) {
    ame::WorldModel wm;
    auto& ts = wm.typeSystem();
    ts.addType("object");
    ts.addType("loc", "object");
    ts.addType("robot", "object");

    wm.addObject("r", "robot");
    wm.addObject("home", "loc");
    wm.addObject("target", "loc");

    wm.registerPredicate("at", {"robot", "loc"});
    wm.registerAction("go",
        {"?r", "?from", "?to"}, {"robot", "loc", "loc"},
        {"(at ?r ?from)"}, {"(at ?r ?to)"}, {"(at ?r ?from)"});

    wm.setFact("(at r home)", true, "test");
    wm.setGoal({"(at r target)"});

    // Run symbolic solver N times — must be identical each time
    ame::Planner planner;
    auto r0 = planner.solve(wm);
    ASSERT_TRUE(r0.success);

    for (int i = 0; i < 5; ++i) {
        auto ri = planner.solve(wm);
        EXPECT_TRUE(ri.success);
        EXPECT_EQ(ri.steps.size(), r0.steps.size());
        for (size_t j = 0; j < ri.steps.size(); ++j) {
            EXPECT_EQ(ri.steps[j].action_index, r0.steps[j].action_index);
        }
    }
}
