// Phase 1 tests: IVerifier, FallbackPolicy, Advisor envelope
// WI-1.1, WI-1.2, WI-1.3

#include "ame/neuro/advisor.h"
#include "ame/neuro/backend_registry.h"
#include "ame/neuro/fallback_policy.h"
#include "ame/neuro/mock_backend.h"
#include "ame/neuro/neuro_audit_log.h"
#include "ame/neuro/verifier.h"
#include "ame/world_model.h"

#include <gtest/gtest.h>
#include <atomic>
#include <chrono>

using namespace ame::neuro;
using namespace std::chrono_literals;

// Helper: build a minimal non-empty WorldModel
static ame::WorldModel make_wm() {
    ame::WorldModel wm;
    wm.typeSystem().addType("object");
    wm.addObject("a", "object");
    wm.registerPredicate("done", {"object"});
    return wm;
}

// Helper: make a registry with one MockBackend named "mock"
static BackendRegistry make_registry(std::shared_ptr<MockBackend> mb) {
    BackendRegistry reg;
    reg.add(mb);
    return reg;
}

using StrAdvisor = Advisor<std::string, std::string>;

// ---------------------------------------------------------------------------
// IVerifier reference implementations
// ---------------------------------------------------------------------------

TEST(Verifier, AlwaysAccept) {
    AlwaysAccept<std::string> v;
    ame::WorldModel wm = make_wm();
    auto verdict = v.verify("anything", wm);
    EXPECT_TRUE(verdict.accepted);
}

TEST(Verifier, AlwaysReject) {
    AlwaysReject<std::string> v("my_reason");
    ame::WorldModel wm = make_wm();
    auto verdict = v.verify("anything", wm);
    EXPECT_FALSE(verdict.accepted);
    EXPECT_EQ(verdict.reason, "my_reason");
}

// ---------------------------------------------------------------------------
// Advisor outcomes (WI-1.2)
// ---------------------------------------------------------------------------

TEST(Advisor, Disabled) {
    auto mb = std::make_shared<MockBackend>("mock");
    auto reg = make_registry(mb);
    ame::WorldModel wm = make_wm();
    AlwaysAccept<std::string> v;
    NeuroAuditLog audit;

    FallbackPolicy policy = FallbackPolicy::disabled(); // enabled=false
    StrAdvisor advisor("test_kind", reg, v, policy, &audit);

    auto result = advisor.advise("req", wm);
    EXPECT_EQ(result.outcome, StrAdvisor::Outcome::Disabled);
    EXPECT_EQ(audit.size(), 1u);
    EXPECT_EQ(audit.records()[0].outcome, "Disabled");
    EXPECT_FALSE(audit.records()[0].affected_behaviour);
}

TEST(Advisor, Accepted) {
    auto mb = std::make_shared<MockBackend>("mock");
    mb->add_script({"the_proposal", true, "", 0.0});
    auto reg = make_registry(mb);
    ame::WorldModel wm = make_wm();
    AlwaysAccept<std::string> v;
    NeuroAuditLog audit;

    FallbackPolicy policy = FallbackPolicy::hot_path("mock");
    StrAdvisor advisor("test_kind", reg, v, policy, &audit);

    auto result = advisor.advise("req", wm);
    ASSERT_EQ(result.outcome, StrAdvisor::Outcome::Accepted);
    ASSERT_TRUE(result.proposal.has_value());
    EXPECT_EQ(*result.proposal, "the_proposal");
    EXPECT_EQ(audit.size(), 1u);
    EXPECT_TRUE(audit.records()[0].affected_behaviour);
}

TEST(Advisor, RejectedFellBack) {
    auto mb = std::make_shared<MockBackend>("mock");
    mb->add_script({"bad_proposal", true, "", 0.0});
    auto reg = make_registry(mb);
    ame::WorldModel wm = make_wm();
    AlwaysReject<std::string> v("rejected_by_test");
    NeuroAuditLog audit;

    FallbackPolicy policy = FallbackPolicy::hot_path("mock");
    StrAdvisor advisor("test_kind", reg, v, policy, &audit);

    auto result = advisor.advise("req", wm);
    EXPECT_EQ(result.outcome, StrAdvisor::Outcome::RejectedFellBack);
    EXPECT_EQ(audit.size(), 1u);
    EXPECT_FALSE(audit.records()[0].affected_behaviour);
    EXPECT_EQ(audit.records()[0].verdict_reason, "rejected_by_test");
}

TEST(Advisor, ErroredFellBack) {
    auto mb = std::make_shared<MockBackend>("mock");
    mb->add_script({"", false, "backend_error", 0.0});
    auto reg = make_registry(mb);
    ame::WorldModel wm = make_wm();
    AlwaysAccept<std::string> v;
    NeuroAuditLog audit;

    FallbackPolicy policy = FallbackPolicy::hot_path("mock");
    StrAdvisor advisor("test_kind", reg, v, policy, &audit);

    auto result = advisor.advise("req", wm);
    EXPECT_EQ(result.outcome, StrAdvisor::Outcome::ErroredFellBack);
    EXPECT_EQ(audit.size(), 1u);
}

TEST(Advisor, UnavailableFellBack) {
    BackendRegistry empty_reg;
    ame::WorldModel wm = make_wm();
    AlwaysAccept<std::string> v;
    NeuroAuditLog audit;

    FallbackPolicy policy;
    policy.enabled = true;
    policy.backend_id = "nonexistent";
    StrAdvisor advisor("test_kind", empty_reg, v, policy, &audit);

    auto result = advisor.advise("req", wm);
    EXPECT_EQ(result.outcome, StrAdvisor::Outcome::UnavailableFellBack);
    EXPECT_EQ(audit.size(), 1u);
    EXPECT_EQ(audit.records()[0].outcome, "UnavailableFellBack");
}

// ---------------------------------------------------------------------------
// Timeout + abandonment (WI-1.3 criterion b: non-cooperative hang)
// ---------------------------------------------------------------------------

TEST(Advisor, TimedOutFellBack_NonCoopHang) {
    auto mb = std::make_shared<MockBackend>("mock", false);
    mb->add_script({"", false, "", 10000.0, /*hang=*/true, /*non_cooperative=*/true});
    BackendExecutorConfig bcfg;
    bcfg.max_in_flight = 4;
    BackendRegistry reg;
    reg.add(mb, BackendTier::Warm, bcfg);
    ame::WorldModel wm = make_wm();
    AlwaysAccept<std::string> v;
    NeuroAuditLog audit;

    FallbackPolicy policy;
    policy.enabled = true;
    policy.latency_budget_ms = 150.0;
    policy.max_retries = 0;
    policy.backend_id = "mock";

    StrAdvisor advisor("test_kind", reg, v, policy, &audit);

    auto t0 = std::chrono::steady_clock::now();
    auto result = advisor.advise("req", wm);
    double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

    EXPECT_EQ(result.outcome, StrAdvisor::Outcome::TimedOutFellBack);
    // Must return within budget + tolerance
    EXPECT_LT(elapsed_ms, 450.0); // generous tolerance for CI
    EXPECT_EQ(audit.size(), 1u);
    EXPECT_EQ(audit.records()[0].outcome, "TimedOutFellBack");
}

// ---------------------------------------------------------------------------
// Retry: errored then accepted on retry (WI-1.3)
// ---------------------------------------------------------------------------

TEST(Advisor, RetryAfterError) {
    auto mb = std::make_shared<MockBackend>("mock");
    mb->add_script({"", false, "first_err", 0.0}); // first attempt errors
    mb->add_script({"good_proposal", true, "", 0.0}); // second attempt ok
    auto reg = make_registry(mb);
    ame::WorldModel wm = make_wm();
    AlwaysAccept<std::string> v;
    NeuroAuditLog audit;

    FallbackPolicy policy = FallbackPolicy::warm_path("mock");
    policy.max_retries = 1;
    policy.retry_backoff_ms = 0.0; // no backoff for test speed
    StrAdvisor advisor("test_kind", reg, v, policy, &audit);

    auto result = advisor.advise("req", wm);
    EXPECT_EQ(result.outcome, StrAdvisor::Outcome::Accepted);
    ASSERT_TRUE(result.proposal.has_value());
    EXPECT_EQ(*result.proposal, "good_proposal");
}

// ---------------------------------------------------------------------------
// Exactly one audit record per advise() call (WI-1.2, WI-3.1)
// ---------------------------------------------------------------------------

TEST(Advisor, ExactlyOneAuditRecordPerCall) {
    auto mb = std::make_shared<MockBackend>("mock");
    mb->add_script({"ok", true});
    auto reg = make_registry(mb);
    ame::WorldModel wm = make_wm();
    AlwaysAccept<std::string> v;
    NeuroAuditLog audit;

    FallbackPolicy policy = FallbackPolicy::hot_path("mock");
    StrAdvisor advisor("test_kind", reg, v, policy, &audit);

    for (int i = 0; i < 5; ++i) {
        advisor.advise("r" + std::to_string(i), wm);
    }
    EXPECT_EQ(audit.size(), 5u);
}

// ---------------------------------------------------------------------------
// Symbolic output unchanged when disabled (WI-4.2, WI-5.3)
// ---------------------------------------------------------------------------

TEST(Advisor, DisabledNeverAffectsBehaviour) {
    auto mb = std::make_shared<MockBackend>("mock");
    mb->add_script({"irrelevant", true});
    auto reg = make_registry(mb);
    ame::WorldModel wm = make_wm();
    AlwaysAccept<std::string> v;
    NeuroAuditLog audit;

    FallbackPolicy policy = FallbackPolicy::disabled();
    StrAdvisor advisor("test_kind", reg, v, policy, &audit);

    for (int i = 0; i < 3; ++i) {
        auto r = advisor.advise("req", wm);
        EXPECT_EQ(r.outcome, StrAdvisor::Outcome::Disabled);
        EXPECT_FALSE(r.proposal.has_value());
    }
    EXPECT_EQ(mb->call_count(), 0u); // backend never called
}

// ---------------------------------------------------------------------------
// WI-1.3: wall-time bound holds even when backend errors immediately
// ---------------------------------------------------------------------------

TEST(Advisor, ImmediateErrorWithinBudget) {
    auto mb = std::make_shared<MockBackend>("mock");
    mb->add_script({"", false, "err", 0.0});
    auto reg = make_registry(mb);
    ame::WorldModel wm = make_wm();
    AlwaysAccept<std::string> v;

    FallbackPolicy policy = FallbackPolicy::hot_path("mock");
    policy.latency_budget_ms = 500.0;
    StrAdvisor advisor("test_kind", reg, v, policy);

    auto t0 = std::chrono::steady_clock::now();
    auto result = advisor.advise("req", wm);
    double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

    EXPECT_EQ(result.outcome, StrAdvisor::Outcome::ErroredFellBack);
    EXPECT_LT(elapsed, 200.0);
}

// ---------------------------------------------------------------------------
// WI-1.3: late result discarded (result after budget is TimedOut, not Accepted)
// ---------------------------------------------------------------------------

TEST(Advisor, LateResultDiscarded) {
    auto mb = std::make_shared<MockBackend>("mock");
    // Latency longer than the budget
    mb->add_script({"late_proposal", true, "", 200.0});
    auto reg = make_registry(mb);
    ame::WorldModel wm = make_wm();
    AlwaysAccept<std::string> v;
    NeuroAuditLog audit;

    FallbackPolicy policy;
    policy.enabled = true;
    policy.latency_budget_ms = 50.0; // budget < backend latency
    policy.max_retries = 0;
    policy.backend_id = "mock";
    StrAdvisor advisor("test_kind", reg, v, policy, &audit);

    auto result = advisor.advise("req", wm);
    EXPECT_EQ(result.outcome, StrAdvisor::Outcome::TimedOutFellBack);
    EXPECT_FALSE(result.proposal.has_value());
}

// ---------------------------------------------------------------------------
// Thread 10: ts_us uses system_clock so it aligns with plan/BT/WM audit streams
// ---------------------------------------------------------------------------

TEST(Advisor, AuditTimestamp_UsesSystemClock) {
    auto mb = std::make_shared<MockBackend>("mock");
    mb->add_script({"proposal", true, "", 0.0});
    auto reg = make_registry(mb);
    ame::WorldModel wm = make_wm();
    AlwaysAccept<std::string> v;
    NeuroAuditLog audit;

    FallbackPolicy policy = FallbackPolicy::hot_path("mock");
    StrAdvisor advisor("test_kind", reg, v, policy, &audit);
    advisor.advise("req", wm);

    ASSERT_EQ(audit.size(), 1u);
    // system_clock epoch in microseconds is ~1.7e18 for year 2024+.
    // steady_clock epoch (time since boot) is typically <1e13 (< ~115 days).
    // A threshold of 1e15 us (year ~2001) safely separates the two.
    const uint64_t epoch_threshold_us = 1'000'000'000'000'000ULL; // 1e15
    EXPECT_GT(audit.records()[0].ts_us, epoch_threshold_us)
        << "ts_us looks like a monotonic (steady_clock) value, not an epoch timestamp";
}

// ---------------------------------------------------------------------------
// Thread 11: verbosity >= 1 stores raw backend payload in raw_proposal, not digest
// ---------------------------------------------------------------------------

TEST(Advisor, VerboseAudit_StoresRawPayloadNotDigest) {
    const std::string raw_backend_payload = "full_proposal_content_from_backend";

    auto mb = std::make_shared<MockBackend>("mock");
    mb->add_script({raw_backend_payload, true, "", 0.0});
    auto reg = make_registry(mb);
    ame::WorldModel wm = make_wm();
    AlwaysAccept<std::string> v;
    NeuroAuditLog audit;

    FallbackPolicy policy = FallbackPolicy::hot_path("mock");
    policy.verbosity = 1;
    StrAdvisor advisor("test_kind", reg, v, policy, &audit);
    auto result = advisor.advise("req", wm);

    ASSERT_EQ(result.outcome, StrAdvisor::Outcome::Accepted);
    ASSERT_EQ(audit.size(), 1u);
    const auto& rec = audit.records()[0];
    // raw_proposal must be the full backend payload, not a truncated digest.
    EXPECT_EQ(rec.raw_proposal, raw_backend_payload);
    // proposal_digest is still the short summary (unchanged).
    EXPECT_NE(rec.proposal_digest, raw_backend_payload);
    EXPECT_FALSE(rec.proposal_digest.empty());
}

// ---------------------------------------------------------------------------
// Thread 20: throwing ProposalCodec::decode() produces ErroredFellBack + audit
// ---------------------------------------------------------------------------

struct ThrowingProposal {};

namespace ame::neuro {
template <>
struct ProposalCodec<ThrowingProposal> {
    static std::optional<ThrowingProposal> decode(const std::string&, std::string&) {
        throw std::runtime_error("decoder exploded");
    }
    static std::string digest(const ThrowingProposal&) { return "throwing"; }
};
} // namespace ame::neuro

TEST(Advisor, ThrowingDecoder_ProducesErroredFellBack) {
    using ThrowAdvisor = Advisor<std::string, ThrowingProposal>;

    auto mb = std::make_shared<MockBackend>("mock");
    mb->add_script({"some_payload", true, "", 0.0});
    BackendRegistry reg;
    reg.add(mb);
    ame::WorldModel wm = make_wm();
    AlwaysAccept<ThrowingProposal> v;
    NeuroAuditLog audit;

    FallbackPolicy policy = FallbackPolicy::hot_path("mock");
    ThrowAdvisor advisor("test_kind", reg, v, policy, &audit);

    ThrowAdvisor::Result result;
    EXPECT_NO_THROW(result = advisor.advise("req", wm));
    EXPECT_EQ(result.outcome, ThrowAdvisor::Outcome::ErroredFellBack);
    EXPECT_EQ(audit.size(), 1u);
    EXPECT_EQ(audit.records()[0].outcome, "ErroredFellBack");
}

// ---------------------------------------------------------------------------
// Thread 21: throwing RequestCodec::encode() produces ErroredFellBack + audit
// ---------------------------------------------------------------------------

struct ThrowingRequest {};

namespace ame::neuro {
template <>
struct RequestCodec<ThrowingRequest> {
    static NeuralRequest encode(const ThrowingRequest&, const std::string&) {
        throw std::runtime_error("encoder exploded");
    }
    static std::string digest(const ThrowingRequest&) { return "throw_req"; }
};
} // namespace ame::neuro

TEST(Advisor, ThrowingEncoder_ProducesErroredFellBack) {
    using ThrowEncAdvisor = Advisor<ThrowingRequest, std::string>;

    auto mb = std::make_shared<MockBackend>("mock");
    mb->add_script({"ok", true, "", 0.0});
    BackendRegistry reg;
    reg.add(mb);
    ame::WorldModel wm = make_wm();
    AlwaysAccept<std::string> v;
    NeuroAuditLog audit;

    FallbackPolicy policy = FallbackPolicy::hot_path("mock");
    ThrowEncAdvisor advisor("test_kind", reg, v, policy, &audit);

    ThrowEncAdvisor::Result result;
    EXPECT_NO_THROW(result = advisor.advise(ThrowingRequest{}, wm));
    EXPECT_EQ(result.outcome, ThrowEncAdvisor::Outcome::ErroredFellBack);
    // Contract: exactly one audit record per advise() call, even on encode exception.
    EXPECT_EQ(audit.size(), 1u);
    EXPECT_EQ(audit.records()[0].outcome, "ErroredFellBack");
}

// ---------------------------------------------------------------------------
// Thread 16: throwing verifier produces ErroredFellBack + exactly one audit record
// ---------------------------------------------------------------------------

struct ThrowingVerifier : public IVerifier<std::string> {
    Verdict verify(const std::string&, const ame::WorldModel&) const override {
        throw std::runtime_error("verifier exploded");
    }
};

TEST(Advisor, ThrowingVerifier_ProducesErroredFellBack) {
    auto mb = std::make_shared<MockBackend>("mock");
    mb->add_script({"proposal", true, "", 0.0});
    auto reg = make_registry(mb);
    ame::WorldModel wm = make_wm();
    ThrowingVerifier v;
    NeuroAuditLog audit;

    FallbackPolicy policy = FallbackPolicy::hot_path("mock");
    StrAdvisor advisor("test_kind", reg, v, policy, &audit);

    StrAdvisor::Result result;
    EXPECT_NO_THROW(result = advisor.advise("req", wm));
    EXPECT_EQ(result.outcome, StrAdvisor::Outcome::ErroredFellBack);
    // Contract: exactly one audit record per advise() call, even on verifier exception.
    EXPECT_EQ(audit.size(), 1u);
    EXPECT_EQ(audit.records()[0].outcome, "ErroredFellBack");
}

// ---------------------------------------------------------------------------
// Thread 23 / Thread GaV86: full remaining budget used per attempt — an
// in-budget response must not be cancelled by a pre-split retry share.
//
// Backend responds after 50 ms — within the 80 ms total budget, but more
// than half.  With max_retries=1 the old split (80/2 = 40 ms each) would
// cancel the first attempt after 40 ms and then have insufficient remaining
// budget for the retry.  With the correct full-remaining-ms per attempt the
// first attempt waits up to 80 ms, the backend replies at 50 ms, and the
// result is Accepted with zero retries.
// ---------------------------------------------------------------------------

TEST(Advisor, FullBudgetUsedPerAttempt) {
    auto mb = std::make_shared<MockBackend>("mock_delayed", /*cooperative=*/false);
    // Non-hanging backend that sleeps 50 ms then returns success.
    mb->add_script({"ok_response", true, "", 50.0, /*hang=*/false});

    BackendRegistry reg;
    reg.add(mb);
    ame::WorldModel wm = make_wm();
    AlwaysAccept<std::string> v;

    FallbackPolicy policy;
    policy.enabled = true;
    policy.backend_id = "mock_delayed";
    policy.latency_budget_ms = 80.0;  // 50 ms < 80 ms → must succeed first try
    policy.max_retries = 1;           // would split to 40 ms with the old approach
    policy.retry_backoff_ms = 0.0;

    StrAdvisor advisor("test_kind", reg, v, policy);
    auto result = advisor.advise("req", wm);
    EXPECT_EQ(result.outcome, StrAdvisor::Outcome::Accepted);
    EXPECT_EQ(result.retries, 0u); // no retry needed: first attempt captured the response
}

// ---------------------------------------------------------------------------
// Thread 24: verbose audit with throwing encoder does not propagate the throw
// ---------------------------------------------------------------------------

TEST(Advisor, VerboseAuditWithThrowingEncoderDoesNotThrow) {
    using ThrowEncAdvisor = Advisor<ThrowingRequest, std::string>;

    auto mb = std::make_shared<MockBackend>("mock");
    mb->add_script({"ok", true, "", 0.0});
    BackendRegistry reg;
    reg.add(mb);
    ame::WorldModel wm = make_wm();
    AlwaysAccept<std::string> v;
    NeuroAuditLog audit;

    FallbackPolicy policy = FallbackPolicy::hot_path("mock");
    policy.verbosity = 1; // triggers raw_request encoding in emit_audit
    ThrowEncAdvisor advisor("test_kind", reg, v, policy, &audit);

    ThrowEncAdvisor::Result result;
    EXPECT_NO_THROW(result = advisor.advise(ThrowingRequest{}, wm));
    // Encoder throws in advise() → ErroredFellBack + one audit record
    EXPECT_EQ(result.outcome, ThrowEncAdvisor::Outcome::ErroredFellBack);
    EXPECT_EQ(audit.size(), 1u);
    EXPECT_EQ(audit.records()[0].outcome, "ErroredFellBack");
}

// ---------------------------------------------------------------------------
// Thread GagZQ: unpinned policy (empty backend_id) resolves actual backend in audit
// ---------------------------------------------------------------------------

TEST(Advisor, UnpinnedPolicyRecordsActualBackendId) {
    auto mb = std::make_shared<MockBackend>("my_backend");
    mb->add_script({"result", true, "", 0.0});
    BackendRegistry reg;
    reg.add(mb);
    ame::WorldModel wm = make_wm();
    AlwaysAccept<std::string> v;
    NeuroAuditLog audit;

    // Empty backend_id: BackendRegistry::find("") returns the first executor.
    FallbackPolicy policy;
    policy.enabled = true;
    policy.latency_budget_ms = 1000.0;
    // policy.backend_id left empty (simulates warm_path()-style unpinned config)

    StrAdvisor advisor("test_kind", reg, v, policy, &audit);
    auto result = advisor.advise("req", wm);

    EXPECT_EQ(result.outcome, StrAdvisor::Outcome::Accepted);
    ASSERT_EQ(audit.size(), 1u);
    // Audit record must reflect the actual backend used, not leave backend_id empty.
    EXPECT_EQ(audit.records()[0].backend_id, "my_backend");
}
