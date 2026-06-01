// Phase 5 tests: contract tests, adversarial responses, codec fuzz
// WI-5.1 — adversarial responses never produce an accepted-yet-invalid proposal

#include "ame/neuro/advisor.h"
#include "ame/neuro/backend_registry.h"
#include "ame/neuro/fallback_policy.h"
#include "ame/neuro/forward_sim_verifier.h"
#include "ame/neuro/grounded_fluent_verifier.h"
#include "ame/neuro/mock_backend.h"
#include "ame/neuro/neuro_audit_log.h"
#include "ame/neuro/verifier.h"
#include "ame/world_model.h"

#include <gtest/gtest.h>

using namespace ame::neuro;
using StrAdvisor = Advisor<std::string, std::string>;

static ame::WorldModel make_simple_wm() {
    ame::WorldModel wm;
    wm.typeSystem().addType("object");
    wm.addObject("a", "object");
    wm.registerPredicate("done", {"object"});
    return wm;
}

static BackendRegistry make_reg(std::shared_ptr<MockBackend> mb) {
    BackendRegistry r;
    r.add(mb);
    return r;
}

// ---------------------------------------------------------------------------
// Contract: invalid proposals are ALWAYS rejected (WI-5.1)
// ---------------------------------------------------------------------------

TEST(Contract, AdversarialPayloadNeverAccepted) {
    // Verifier rejects everything
    AlwaysReject<std::string> v;
    auto wm = make_simple_wm();
    NeuroAuditLog audit;

    std::vector<std::string> adversarial_payloads = {
        "",
        "{}",
        R"({"drop_table":"users"})",
        "<script>alert(1)</script>",
        std::string(1024, 'A'), // oversized
        "\x00\x01\x02",
        "null",
        "true",
        std::string{"{\"fluent_keys\":[\"(nonexistent key)\"]}"},
    };

    for (const auto& payload : adversarial_payloads) {
        auto mb = std::make_shared<MockBackend>("adv");
        mb->add_script({payload, true, "", 0.0});
        auto reg = make_reg(mb);

        FallbackPolicy policy = FallbackPolicy::hot_path("adv");
        Advisor<std::string, std::string> advisor("test", reg, v, policy, &audit);
        auto result = advisor.advise("req", wm);

        EXPECT_NE(result.outcome, StrAdvisor::Outcome::Accepted)
            << "adversarial payload should not be accepted: " << payload;
    }
}

// ---------------------------------------------------------------------------
// Contract: symbolic output unchanged when disabled (WI-4.2, WI-5.1)
// ---------------------------------------------------------------------------

TEST(Contract, DisabledNeverCallsBackend) {
    auto mb = std::make_shared<MockBackend>("mock");
    mb->add_script({"should_not_see_this", true});
    auto reg = make_reg(mb);
    auto wm = make_simple_wm();
    AlwaysAccept<std::string> v;

    FallbackPolicy policy = FallbackPolicy::disabled();
    Advisor<std::string, std::string> advisor("test", reg, v, policy);

    for (int i = 0; i < 10; ++i) {
        auto r = advisor.advise("req", wm);
        EXPECT_EQ(r.outcome, StrAdvisor::Outcome::Disabled);
    }
    EXPECT_EQ(mb->call_count(), 0u);
}

// ---------------------------------------------------------------------------
// Contract: fallback always reachable within budget (WI-1.3)
// Tests all four WI-1.3 acceptance criteria:
// (a) cooperative backend cancelled  (b) non-cooperative hang abandoned
// (c) immediate error  (d) late result discarded
// ---------------------------------------------------------------------------

TEST(Contract, FallbackReachable_CooperativeHang) {
    auto mb = std::make_shared<MockBackend>("mock", true);
    mb->add_script({"", false, "", 2000.0, /*hang*/true, /*noncoop*/false});
    auto reg = make_reg(mb);
    auto wm = make_simple_wm();
    AlwaysAccept<std::string> v;

    FallbackPolicy p;
    p.enabled = true;
    p.latency_budget_ms = 100.0;
    p.backend_id = "mock";

    Advisor<std::string, std::string> advisor("t", reg, v, p);
    auto t0 = std::chrono::steady_clock::now();
    auto r = advisor.advise("req", wm);
    double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

    EXPECT_NE(r.outcome, StrAdvisor::Outcome::Accepted);
    EXPECT_LT(elapsed, 500.0); // within budget + generous tolerance
}

TEST(Contract, FallbackReachable_NonCoopHang) {
    auto mb = std::make_shared<MockBackend>("mock", false);
    mb->add_script({"", false, "", 10000.0, true, true});
    auto reg = make_reg(mb);
    auto wm = make_simple_wm();
    AlwaysAccept<std::string> v;

    FallbackPolicy p;
    p.enabled = true;
    p.latency_budget_ms = 100.0;
    p.backend_id = "mock";

    Advisor<std::string, std::string> advisor("t", reg, v, p);
    auto t0 = std::chrono::steady_clock::now();
    auto r = advisor.advise("req", wm);
    double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

    EXPECT_EQ(r.outcome, StrAdvisor::Outcome::TimedOutFellBack);
    EXPECT_LT(elapsed, 500.0);
}

TEST(Contract, FallbackReachable_ImmediateError) {
    auto mb = std::make_shared<MockBackend>("mock");
    mb->add_script({"", false, "err", 0.0});
    auto reg = make_reg(mb);
    auto wm = make_simple_wm();
    AlwaysAccept<std::string> v;

    FallbackPolicy p;
    p.enabled = true;
    p.latency_budget_ms = 500.0;
    p.backend_id = "mock";

    Advisor<std::string, std::string> advisor("t", reg, v, p);
    auto t0 = std::chrono::steady_clock::now();
    auto r = advisor.advise("req", wm);
    double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

    EXPECT_EQ(r.outcome, StrAdvisor::Outcome::ErroredFellBack);
    EXPECT_LT(elapsed, 200.0);
}

TEST(Contract, FallbackReachable_LateResultDiscarded) {
    auto mb = std::make_shared<MockBackend>("mock");
    mb->add_script({"result", true, "", 300.0}); // arrives after 300ms
    auto reg = make_reg(mb);
    auto wm = make_simple_wm();
    AlwaysAccept<std::string> v;

    FallbackPolicy p;
    p.enabled = true;
    p.latency_budget_ms = 50.0; // budget expires before result
    p.backend_id = "mock";

    Advisor<std::string, std::string> advisor("t", reg, v, p);
    auto r = advisor.advise("req", wm);
    EXPECT_EQ(r.outcome, StrAdvisor::Outcome::TimedOutFellBack);
    EXPECT_FALSE(r.proposal.has_value());
}

// ---------------------------------------------------------------------------
// Contract: codec robustness — garbage in, clean rejection (WI-5.1)
// The std::string ProposalCodec always returns a valid optional, so we test
// the verifier-rejection path to ensure no crash on adversarial content.
// ---------------------------------------------------------------------------

TEST(Contract, MalformedResponseDoesNotCrash) {
    auto wm = make_simple_wm();
    AlwaysReject<std::string> v;
    NeuroAuditLog audit;

    std::vector<std::string> garbage = {
        "", "\x00", std::string(65536, '\xff'), "}{invalid json}",
    };

    for (const auto& g : garbage) {
        auto mb = std::make_shared<MockBackend>("gc");
        mb->add_script({g, true, "", 0.0});
        BackendRegistry reg;
        reg.add(mb);
        FallbackPolicy p = FallbackPolicy::hot_path("gc");
        Advisor<std::string, std::string> advisor("t", reg, v, p, &audit);
        EXPECT_NO_THROW({
            auto r = advisor.advise("req", wm);
            EXPECT_NE(r.outcome, StrAdvisor::Outcome::Accepted);
        });
    }
}

// ---------------------------------------------------------------------------
// Contract: GroundedFluentVerifier never accepts unknown keys (WI-2.1)
// ---------------------------------------------------------------------------

TEST(Contract, GroundedFluentVerifier_NoFalsePositives) {
    auto wm = make_simple_wm();
    GroundedFluentVerifier v;

    std::vector<std::string> adversarial_keys = {
        "(nonexistent a)",
        "(done x)", // x not in world
        "(done)",   // wrong arity
        "",
        "(done a) OR (done a)", // injection attempt
    };

    for (const auto& key : adversarial_keys) {
        FluentProposal p;
        p.fluent_keys = {key};
        auto verdict = v.verify(p, wm);
        // Valid key "(done a)" should be accepted; others should not
        if (key == "(done a)") {
            EXPECT_TRUE(verdict.accepted) << "Valid key should be accepted";
        } else {
            EXPECT_FALSE(verdict.accepted) << "Invalid key should be rejected: " << key;
        }
    }
}
