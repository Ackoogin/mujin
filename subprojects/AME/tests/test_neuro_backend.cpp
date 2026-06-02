// Phase 0 tests: INeuralBackend, CancelToken, BackendExecutor, BackendRegistry
// WI-0.1 (builds in CI), WI-0.2 (async + cancel + circuit breaker)

#include "ame/neuro/backend_executor.h"
#include "ame/neuro/backend_registry.h"
#include "ame/neuro/cancel_token.h"
#include "ame/neuro/mock_backend.h"
#include "ame/neuro/neural_backend.h"

#include <gtest/gtest.h>
#include <chrono>
#include <thread>

using namespace ame::neuro;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// CancelToken
// ---------------------------------------------------------------------------

TEST(CancelToken, DefaultNotCancelled) {
    CancelToken tok;
    EXPECT_FALSE(tok.cancelled());
}

TEST(CancelToken, CooperativeCancel) {
    CancelSource src;
    CancelToken tok = src.token();
    EXPECT_FALSE(tok.cancelled());
    src.request_cancel();
    EXPECT_TRUE(tok.cancelled());
}

TEST(CancelToken, MultipleTokensSharedFlag) {
    CancelSource src;
    CancelToken t1 = src.token();
    CancelToken t2 = src.token();
    src.request_cancel();
    EXPECT_TRUE(t1.cancelled());
    EXPECT_TRUE(t2.cancelled());
}

// ---------------------------------------------------------------------------
// NullBackend
// ---------------------------------------------------------------------------

TEST(NullBackend, AlwaysUnavailable) {
    NullBackend nb("null_test");
    EXPECT_EQ(nb.info().id, "null_test");

    CancelSource cs;
    auto fut = nb.submit({}, cs.token());
    auto resp = fut.get();
    EXPECT_FALSE(resp.ok);
    EXPECT_FALSE(resp.error.empty());
}

// ---------------------------------------------------------------------------
// MockBackend — basic scripted responses
// ---------------------------------------------------------------------------

TEST(MockBackend, ScriptedOkResponse) {
    auto mb = std::make_shared<MockBackend>("mock_ok");
    mb->add_script({"hello", true, "", 0.0});

    CancelSource cs;
    auto fut = mb->submit({}, cs.token());
    auto resp = fut.get();
    EXPECT_TRUE(resp.ok);
    EXPECT_EQ(resp.payload, "hello");
    EXPECT_EQ(mb->call_count(), 1u);
}

TEST(MockBackend, ScriptedErrorResponse) {
    auto mb = std::make_shared<MockBackend>("mock_err");
    mb->add_script({"", false, "test_error", 0.0});

    CancelSource cs;
    auto resp = mb->submit({}, cs.token()).get();
    EXPECT_FALSE(resp.ok);
    EXPECT_EQ(resp.error, "test_error");
}

TEST(MockBackend, LastScriptRepeats) {
    auto mb = std::make_shared<MockBackend>("mock_repeat");
    mb->add_script({"first", true, "", 0.0});
    mb->add_script({"second", true, "", 0.0});

    CancelSource cs;
    auto r1 = mb->submit({}, cs.token()).get();
    auto r2 = mb->submit({}, cs.token()).get();
    auto r3 = mb->submit({}, cs.token()).get();
    EXPECT_EQ(r1.payload, "first");
    EXPECT_EQ(r2.payload, "second");
    EXPECT_EQ(r3.payload, "second"); // last repeats
}

// ---------------------------------------------------------------------------
// MockBackend — cooperative cancellation (WI-0.2 criterion a)
// ---------------------------------------------------------------------------

TEST(MockBackend, CooperativeCancelMidHang) {
    auto mb = std::make_shared<MockBackend>("mock_coop", /*cooperative=*/true);
    mb->add_script({"", false, "", 500.0, /*hang=*/true, /*non_cooperative=*/false});

    CancelSource cs;
    auto fut = mb->submit({}, cs.token());

    // Give the thread a moment to start, then cancel
    std::this_thread::sleep_for(20ms);
    cs.request_cancel();

    auto status = fut.wait_for(200ms);
    EXPECT_EQ(status, std::future_status::ready);
    auto resp = fut.get();
    EXPECT_FALSE(resp.ok);
}

// ---------------------------------------------------------------------------
// BackendExecutor — non-cooperative hang (WI-0.2 criterion b)
// Caller abandons the future; must return within budget.
// ---------------------------------------------------------------------------

TEST(BackendExecutor, NonCoopHangAbandonedWithinBudget) {
    auto mb = std::make_shared<MockBackend>("mock_noncoop", false);
    // Non-cooperative hang for 10 seconds
    mb->add_script({"", false, "", 10000.0, /*hang=*/true, /*non_cooperative=*/true});

    BackendExecutorConfig cfg;
    cfg.max_in_flight = 4;
    BackendExecutor exec(mb, cfg);

    CancelSource cs;
    auto fut = exec.submit({}, cs.token());

    auto t0 = std::chrono::steady_clock::now();
    constexpr double budget_ms = 150.0;
    auto status = fut.wait_for(std::chrono::milliseconds(static_cast<long long>(budget_ms)));

    cs.request_cancel(); // signal (ignored by backend, but required by Advisor contract)
    exec.on_abandoned(); // notify executor

    double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

    // Caller must have returned within budget (+ small tolerance)
    EXPECT_EQ(status, std::future_status::timeout);
    EXPECT_LT(elapsed_ms, budget_ms + 100.0);
}

// ---------------------------------------------------------------------------
// BackendExecutor — circuit breaker (WI-0.2 criterion c)
// ---------------------------------------------------------------------------

TEST(BackendExecutor, CircuitBreakerOpensAfterAbandonments) {
    auto mb = std::make_shared<MockBackend>("mock_cb", false);
    // Always hang — will all be abandoned
    for (int i = 0; i < 6; ++i) {
        mb->add_script({"", false, "", 10000.0, true, true});
    }

    BackendExecutorConfig cfg;
    cfg.max_in_flight = 4;
    cfg.failure_threshold = 3;
    BackendExecutor exec(mb, cfg);

    // Submit and abandon until circuit trips
    for (int i = 0; i < (int)cfg.failure_threshold; ++i) {
        ASSERT_TRUE(exec.available()) << "should be available at iteration " << i;
        CancelSource cs;
        auto fut = exec.submit({}, cs.token());
        // Abandon immediately
        cs.request_cancel();
        fut.wait_for(1ms); // don't wait
        exec.on_abandoned();
    }

    // Circuit should now be open
    EXPECT_FALSE(exec.available());

    // After reset, circuit should close
    exec.reset_circuit();
    EXPECT_TRUE(exec.available());
}

// ---------------------------------------------------------------------------
// BackendExecutor — pool saturation (WI-0.2 criterion c, pool exhaustion)
// ---------------------------------------------------------------------------

TEST(BackendExecutor, PoolSaturationReturnsUnavailable) {
    auto mb = std::make_shared<MockBackend>("mock_pool", false);
    for (int i = 0; i < 10; ++i) {
        mb->add_script({"", false, "", 10000.0, true, true});
    }

    BackendExecutorConfig cfg;
    cfg.max_in_flight = 2;
    cfg.failure_threshold = 100; // don't trip circuit
    BackendExecutor exec(mb, cfg);

    // Fill the pool
    CancelSource cs1, cs2;
    auto f1 = exec.submit({}, cs1.token());
    auto f2 = exec.submit({}, cs2.token());

    // Next submit should be unavailable immediately
    CancelSource cs3;
    auto f3 = exec.submit({}, cs3.token());
    auto r3 = f3.get();
    EXPECT_FALSE(r3.ok);
    EXPECT_FALSE(r3.error.empty()); // "pool_saturated"

    // Cleanup
    cs1.request_cancel(); cs2.request_cancel();
    f1.wait_for(50ms); f2.wait_for(50ms);
}

// ---------------------------------------------------------------------------
// BackendExecutor — abandoned late success does not reset consecutive_failures
// Regression for PR review Thread 12: std::promise::set_value() does not throw
// when the future is dropped, so delivered==true for all completions.  The fix
// uses a per-call abandoned_flag so the circuit breaker still trips when the
// backend is consistently slower than the latency budget.
// ---------------------------------------------------------------------------

TEST(BackendExecutor, LateSuccessAfterAbandonDoesNotResetCircuit) {
    // Backend returns ok=true but only after 300 ms (beyond any budget we use).
    auto mb = std::make_shared<MockBackend>("mock_slow", false);
    for (int i = 0; i < 4; ++i)
        mb->add_script({"ok", true, "", 300.0, /*hang=*/true, /*non_cooperative=*/true});

    BackendExecutorConfig cfg;
    cfg.max_in_flight = 8;
    cfg.failure_threshold = 3;
    cfg.recovery_window_ms = 60000.0; // don't let recovery window interfere
    BackendExecutor exec(mb, cfg);

    // Abandon three calls (threshold=3).  Each one signals the flag before calling
    // on_abandoned() so the late ok=true result doesn't reset consecutive_failures.
    for (int i = 0; i < (int)cfg.failure_threshold; ++i) {
        ASSERT_TRUE(exec.available()) << "iteration " << i;
        auto abandoned = std::make_shared<std::atomic<bool>>(false);
        CancelSource cs;
        auto fut = exec.submit({}, cs.token(), abandoned);
        cs.request_cancel();
        fut.wait_for(5ms); // well short of 300 ms backend latency
        abandoned->store(true);  // mark BEFORE on_abandoned
        exec.on_abandoned();
    }

    // Circuit must be open now despite the backends eventually returning ok=true.
    EXPECT_FALSE(exec.available())
        << "circuit should open after " << cfg.failure_threshold
        << " abandoned calls even when the backend eventually returns ok=true";
}

// ---------------------------------------------------------------------------
// BackendRegistry — named lookup
// ---------------------------------------------------------------------------

TEST(BackendRegistry, LookupByName) {
    BackendRegistry reg;
    auto mb = std::make_shared<MockBackend>("my_backend");
    mb->add_script({"ok", true});
    reg.add(mb);

    auto* exec = reg.find("my_backend");
    ASSERT_NE(exec, nullptr);
    EXPECT_EQ(exec->info().id, "my_backend");
}

TEST(BackendRegistry, EmptyIdReturnsFirst) {
    BackendRegistry reg;
    reg.add(std::make_shared<MockBackend>("first"));
    reg.add(std::make_shared<MockBackend>("second"));

    auto* exec = reg.find("");
    ASSERT_NE(exec, nullptr);
    EXPECT_EQ(exec->info().id, "first");
}

TEST(BackendRegistry, UnknownIdReturnsNull) {
    BackendRegistry reg;
    EXPECT_EQ(reg.find("does_not_exist"), nullptr);
}
