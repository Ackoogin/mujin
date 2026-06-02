# Neuro-Symbolic Reasoning Infrastructure

> **Status:** Core infrastructure implemented (`ame_neuro` library, `AME_NEURO=ON`).
> Use-case-specific integrations (Options A–G from the research doc) plug into the
> seams defined here. See [`07-extensions.md`](07-extensions.md) for the adoption roadmap.

---

## 1. Design Principles

### The Propose-Verify-Fallback Envelope

Every neural integration must follow this mandatory envelope:

```
Neural backend
      |
      | NeuralRequest
      v
 INeuralBackend::submit()       ← async, cancellable, bounded
      |
      | NeuralResponse (payload string)
      v
 ProposalCodec<P>::decode()     ← typed proposal
      |
      v
 IVerifier<P>::verify()         ← deterministic symbolic gate
      |
      +--[accepted]--> use proposal
      |
      +--[rejected / timed-out / error]--> symbolic fallback (mandatory)
```

**Key invariants:**
- The symbolic solver is always the authority. Neural components are advisory only.
- Fallback is automatic and unconditional — a disabled, crashed, or slow backend
  has zero impact on safety or correctness.
- Every call through the envelope emits exactly one `NeuroAuditRecord` (Layer 6).
- All verifiers are deterministic and side-effect-free; they operate on WorldModel
  snapshots, never on live state.

### Null-Object Defaults

Seams in `ame_core` default to `nullptr`. When no hook is attached, `ame_core`
behaves identically to a non-`AME_NEURO` build, byte-for-byte. Tests can verify
this by comparing symbolic-only runs with and without `AME_NEURO=ON`.

---

## 2. Library Layout

```
ame_core  (always built; AME_NEURO=1 propagated when ame_neuro is used)
  |- Planner           HeuristicHook seam   (#if AME_NEURO)
  \- ExecutorComponent RepairHook seam      (#if AME_NEURO)

ame_neuro  (built when -DAME_NEURO=ON; depends only on ame_core)
  |- Backend layer
  |    cancel_token.h / neural_backend.h / backend_executor.h
  |    backend_registry.h / mock_backend.h
  |    src/lib/neuro/backend_executor.cpp
  |    src/lib/neuro/backend_registry.cpp
  |    src/lib/neuro/mock_backend.cpp
  |
  |- Advisor + Policy
  |    advisor.h / verifier.h / fallback_policy.h / codec.h
  |    src/lib/neuro/fallback_policy.cpp
  |
  |- Domain Verifiers
  |    authority_view.h
  |    grounded_fluent_verifier.h / forward_sim_verifier.h
  |    src/lib/neuro/grounded_fluent_verifier.cpp
  |    src/lib/neuro/forward_sim_verifier.cpp
  |
  |- Observability (Layer 6)
  |    neuro_audit_log.h / audit_index.h
  |    src/lib/neuro/neuro_audit_log.cpp
  |    src/lib/neuro/audit_index.cpp
  |
  \- Configuration
       neuro_config.h
       src/lib/neuro/neuro_config.cpp
```

No ROS2, ONNX, or libcurl dependency enters `ame_neuro`. Concrete backends
(ONNX, HTTP/LLM) are compiled in separately and registered at runtime.

---

## 3. Backend Layer

### CancelToken / CancelSource

C++17-compatible cooperative cancellation (no `std::stop_token`):

```cpp
// Producer (Advisor)
CancelSource cs;
auto fut = exec->submit(req, cs.token());

// ... after budget expires:
cs.request_cancel();   // signal; does not block

// Consumer (backend thread)
while (still_working) {
    if (cancel.cancelled()) break;   // polls atomically
    // ...
}
```

Internals: `shared_ptr<atomic<bool>>`. The token holds a copy of the shared
pointer — cancellation is safe even if the `CancelSource` is destroyed first.
Backends that do not poll are "non-cooperative"; the Advisor still enforces the
wall-clock budget via `fut.wait_for()` regardless.

### INeuralBackend

```cpp
struct BackendInfo {
    std::string id;            // registry key, e.g. "onnx_heuristic"
    std::string model_version; // for provenance pinning
};

struct NeuralRequest {
    std::string payload;  // opaque; codec encodes domain-specific content
};

struct NeuralResponse {
    bool        ok = false;
    std::string payload;
    std::string error;
    double      latency_ms = 0.0;
    std::string backend_id;
    std::string model_id;
};

class INeuralBackend {
public:
    virtual BackendInfo info() const = 0;
    // Non-blocking: returns a future immediately and does work asynchronously.
    virtual std::future<NeuralResponse> submit(const NeuralRequest&,
                                               CancelToken) = 0;
    virtual ~INeuralBackend() = default;
};
```

**Contract:**
- `submit()` must return promptly (never block the caller).
- Future destruction must be non-blocking (use `promise`/`future`, not `async`).
- Concrete backends may launch detached threads, a thread pool, or an I/O loop.

### BackendExecutor

Wraps any `INeuralBackend` with two safety mechanisms:

| Mechanism | Purpose |
|-----------|---------|
| Bounded pool (`max_in_flight`) | Prevents runaway detached threads when the backend hangs |
| Circuit breaker | Stops calling a pathological backend after `failure_threshold` consecutive failures; auto-resets after `recovery_window_ms` |

```
submit() called
  |
  +--[pool saturated]---------> return NeuralResponse{ok=false, "pool_saturated"}
  |
  +--[circuit open]-----------> return NeuralResponse{ok=false, "circuit_open"}
  |
  +--> ++in_flight, launch detached thread
              |
              +--> backend->submit(req, cancel)
              +--> --in_flight, update circuit state
              +--> promise->set_value(resp)   ← may throw if future abandoned
```

**Thread safety:** All mutable state lives in `shared_ptr<State>` so detached
threads can safely update `in_flight` and `consecutive_failures` even if the
`BackendExecutor` has been destroyed (e.g., after an Advisor timeout discarded
the future and the registry was torn down).

**Backend ownership:** `BackendExecutor` holds a `shared_ptr<INeuralBackend>`,
keeping the backend alive for the full lifetime of any in-flight threads.

### BackendRegistry

Named lookup + ownership store:

```cpp
BackendRegistry reg;
reg.add(std::make_shared<MyOnnxBackend>("onnx_heuristic"), BackendTier::Hot);
reg.add(std::make_shared<MyLlmBackend>("llm_repair"),      BackendTier::Warm);

BackendExecutor* exec = reg.find("onnx_heuristic");  // nullptr if absent
```

Tiers:

| Tier | Typical backend | Latency target |
|------|-----------------|---------------|
| Hot  | Local ONNX / fast model | < 10 ms |
| Warm | Local LLM / fast cloud API | < 500 ms |
| Cold | Full cloud API | < 5 s |

### MockBackend

Deterministic scripted backend for testing. Each call consumes one script entry
(the last entry repeats). Supports:

```cpp
MockBackend mb("mock", /*cooperative=*/true);
mb.add_script({"response_payload", /*ok=*/true, /*error=*/"", /*latency_ms=*/5.0});
mb.add_script({"", false, "test_error", 0.0, /*hang=*/true, /*non_cooperative=*/false});
```

Non-cooperative hangs simulate the real case where a backend ignores the cancel
token. The Advisor enforces the wall-clock budget regardless.

---

## 4. Advisor Orchestrator

`Advisor<Request, Proposal>` is the main entry point for all integrations. It
wires together backend, codec, verifier, fallback policy, and audit log.

### Template Parameters

```cpp
template <typename Request, typename Proposal>
class Advisor {
    // Requires:
    //   RequestCodec<Request>::encode(req) -> NeuralRequest
    //   ProposalCodec<Proposal>::decode(payload) -> optional<Proposal>
    //   IVerifier<Proposal>
    //   FallbackPolicy
    //   NeuroAuditLog*  (nullable; skip audit if null)
};
```

### Outcome Enum

```
Accepted          -- neural proposal verified; symbolic solver sees the result
RejectedFellBack  -- verifier rejected; symbolic fallback used
TimedOutFellBack  -- budget expired; symbolic fallback used
UnavailableFellBack -- circuit open or pool full; symbolic fallback used
ErroredFellBack   -- decode/backend error; symbolic fallback used
Disabled          -- policy.enabled == false; backend never called
```

### advise() Lifecycle

```
advise(req, wm)
  |
  +--[policy.enabled == false]--> return {Disabled, fallback_proposal}
  |
  +--[!exec->available()]-------> return {UnavailableFellBack, fallback_proposal}
  |
  +--> encode request
  +--> CancelSource cs; fut = exec->submit(encoded, cs.token())
  |
  +--> [retry loop, up to policy.max_retries]
  |       |
  |       +--> fut.wait_for(remaining_budget)
  |       |
  |       +--[ready]--> decode response
  |       |               +--[decode fails]--> ErroredFellBack
  |       |               |
  |       |               +--> verifier.verify(proposal, wm)
  |       |                     +--[rejected]--> RejectedFellBack
  |       |                     +--[accepted]--> Accepted ✓
  |       |
  |       +--[timeout]---> cs.request_cancel()
  |                        exec->on_abandoned()  ← circuit breaker tracking
  |                        [if retries remain: backoff, resubmit]
  |                        [if no retries remain: TimedOutFellBack]
  |
  +--> emit NeuroAuditRecord (always, exactly once)
  +--> return AdvisorResult{outcome, proposal_or_fallback, evidence}
```

### Usage Pattern

```cpp
// 1. Create advisor (once, at setup time)
auto advisor = std::make_shared<Advisor<MyRequest, MyProposal>>(
    registry.find("my_backend"),          // BackendExecutor*
    std::make_shared<MyVerifier>(),       // IVerifier<MyProposal>
    FallbackPolicy::warm_path("my_backend"),
    audit_log.get(),                      // NeuroAuditLog* (nullable)
    "my_integration"                      // integration_id for audit
);

// 2. Call per planning/execution event
auto result = advisor->advise(request, world_model);

switch (result.outcome) {
    case Outcome::Accepted:
        use(result.proposal);
        break;
    default:
        use(symbolic_fallback());         // always available
        break;
}
```

### FallbackPolicy

```cpp
struct FallbackPolicy {
    bool     enabled            = true;
    double   latency_budget_ms  = 200.0;
    unsigned max_retries        = 1;
    double   retry_backoff_ms   = 50.0;
    std::string backend_id;      // empty = first registered
    std::string model_id;

    static FallbackPolicy hot_path(const std::string& backend_id = "");
    static FallbackPolicy warm_path(const std::string& backend_id = "");
    static FallbackPolicy cold_path(const std::string& backend_id = "");
    static FallbackPolicy disabled();
};
```

| Factory | Budget | Retries | Backoff | Use case |
|---------|--------|---------|---------|---------|
| `hot_path` | 50 ms | 1 | 10 ms | Online heuristic (BRFS tie-breaking) |
| `warm_path` | 500 ms | 2 | 50 ms | Pre-solve goal interpretation |
| `cold_path` | 5 000 ms | 3 | 500 ms | Offline analysis |
| `disabled` | — | — | — | Compile out without removing code |

---

## 5. Verifiers

### IVerifier<Proposal>

```cpp
template <typename Proposal>
class IVerifier {
public:
    struct Verdict {
        bool                     accepted = false;
        std::string              reason;
        std::vector<std::string> evidence;  // fact names, step labels, etc.
    };

    virtual Verdict verify(const Proposal&, const WorldModel&) const = 0;
    virtual ~IVerifier() = default;
};
```

Verifiers must be **pure** (no side effects, deterministic given the same inputs).
They receive the WorldModel by const reference and must not modify it.

Reference implementations: `AlwaysAccept<P>` and `AlwaysReject<P>` (useful for
testing policy and circuit-breaker behaviour without a real verifier).

### AuthorityView

Controls which facts verifiers consider trustworthy:

| Mode | Behaviour |
|------|-----------|
| `All` | Accept any true fact regardless of its `FactAuthority` |
| `ConfirmedThenBelieved` | Accept both CONFIRMED and BELIEVED; record believed facts in `evidence` |
| `ConfirmedOnly` | Reject any fact below `CONFIRMED` authority |

`CONFIRMED` facts come from perception (sensor data). `BELIEVED` facts come from
plan effect predictions. Use `ConfirmedOnly` for safety-critical checks.

### GroundedFluentVerifier

Checks that every fluent key in a `FluentProposal` is registered in the WorldModel
and (optionally) on an allow-list:

```cpp
FluentProposal proposal;
proposal.fluent_keys = {"(at uav1 base)", "(armed uav1)"};

GroundedFluentVerifier v(/*allow_list=*/nullptr);
auto verdict = v.verify(proposal, wm);
// verdict.accepted == false if "(armed uav1)" is not a registered fluent
```

Allow-list hook (`AllowListFn = std::function<bool(const std::string&)>`) lets
operators restrict which fluents neural components are permitted to name, providing
a governance layer independent of grounding correctness.

### ForwardSimVerifier

Simulates a `PlanProposal` (ordered list of `PlanStep` + goal fluent keys) against
a **copy** of the WorldModel's state bits, never touching live state:

```
for each step:
    check all preconditions (respecting AuthorityView)
    apply add_effects (mark BELIEVED)
    apply del_effects

check goal fluents are satisfied in sim state
```

A precondition failure or unachieved goal returns `Verdict{accepted=false}` with
the failing step index and fluent name in `evidence`. All effects are tagged
`BELIEVED` in the simulation (plan predictions, not perception).

---

## 6. Codec Traits

Codecs decouple domain-specific serialisation from the generic Advisor template:

```cpp
// Request side: encode a domain-specific request to a NeuralRequest
template <typename T>
struct RequestCodec {
    static NeuralRequest encode(const T&);
};

// Proposal side: decode a NeuralResponse payload to a typed proposal
template <typename T>
struct ProposalCodec {
    static std::optional<T> decode(const std::string& payload);
};
```

Specialize these for each integration type:

```cpp
// Example: heuristic scores
template<>
struct RequestCodec<HeuristicRequest> {
    static NeuralRequest encode(const HeuristicRequest& r) {
        return NeuralRequest{to_json(r)};
    }
};

template<>
struct ProposalCodec<std::vector<ActionScore>> {
    static std::optional<std::vector<ActionScore>> decode(const std::string& s) {
        return parse_action_scores(s);  // return nullopt on parse failure
    }
};
```

`std::string` specializations ship with `ame_neuro` for infrastructure tests
(identity encode/decode).

---

## 7. Observability: Layer 6

`NeuroAuditLog` extends the 5-layer stack with a neuro-specific record per
`Advisor::advise()` call.

### NeuroAuditRecord

```cpp
struct NeuroAuditRecord {
    uint64_t    id;              // monotonically assigned, globally unique per log
    uint64_t    ts_us;           // wall clock, microseconds since epoch
    std::string integration_id;  // e.g. "heuristic_hook", "goal_interpreter"
    std::string backend_id;
    std::string outcome;         // "Accepted", "TimedOutFellBack", etc.
    double      latency_ms;
    unsigned    retries;
    bool        verify_accepted;
    std::string reason;          // verifier reason or error message
    std::vector<std::string> evidence;
    std::string request_digest;  // first 64 chars of encoded request
    std::string proposal_digest; // first 64 chars of decoded proposal
};
```

JSONL output line:
```json
{
  "id": 1,
  "ts_us": 1702345679000000,
  "integration_id": "heuristic_hook",
  "backend_id": "onnx_heuristic",
  "outcome": "Accepted",
  "latency_ms": 12.4,
  "retries": 0,
  "verify_accepted": true,
  "reason": "plan_verified",
  "evidence": ["believed:42"],
  "request_digest": "{\"fluents\":[\"(at uav1 base)\"]",
  "proposal_digest": "[{\"ground_action_id\":3,\"score\":0.91}]"
}
```

### AuditIndex

Loads multiple JSONL streams (bt/wm/plan/neuro) and builds time + entity indices
for cross-stream queries:

```cpp
AuditIndex idx;
idx.load("bt_events.jsonl",  "bt");
idx.load("wm_audit.jsonl",   "wm");
idx.load("plan_audit.jsonl", "plan");
idx.load("neuro_audit.jsonl","neuro");

// Time window query
auto records = idx.window(t0_us, t1_us);

// Context window around a specific entity
auto context = idx.around("episode_42", /*k=*/10, /*max_bytes=*/65536);

// Export training samples (plan-stream records)
auto samples = idx.training_export();

// Human-readable provenance citation
auto ids = idx.cite("uav1_sectorA");
```

This gives a unified view of what the BT was doing, what WM facts were changing,
what plan was active, and what neural calls were made — all on the same timeline.

---

## 8. NeuroConfig

Runtime configuration without recompiling. Loaded from JSON:

```json
{
  "all_disabled": false,
  "integrations": [
    {
      "id": "heuristic_hook",
      "enabled": true,
      "backend_id": "onnx_heuristic",
      "model_id": "heuristic_v2",
      "model_version_pin": "2.1.0",
      "latency_budget_ms": 50.0,
      "max_retries": 1,
      "retry_backoff_ms": 10.0
    },
    {
      "id": "repair_hook",
      "enabled": false,
      "backend_id": "llm_repair",
      "latency_budget_ms": 500.0
    }
  ]
}
```

```cpp
auto cfg = NeuroConfig::from_file("neuro.json");

// Kill-switch: disables all integrations
if (cfg.all_disabled) { /* neural path not taken */ }

// Per-integration config
const IntegrationConfig* ic = cfg.find("heuristic_hook");
FallbackPolicy policy = cfg.policy_for("heuristic_hook"); // disabled() if absent

// Model version pinning: reject if backend reports a different version
if (ic && !ic->model_version_pin.empty()) {
    auto info = exec->info();
    if (info.model_version != ic->model_version_pin)
        throw std::runtime_error("model version mismatch");
}
```

`NeuroConfig` is immutable after construction. Hot-reload is the caller's
responsibility (swap the `shared_ptr`).

---

## 9. Integration Seams in ame_core

Two thin seams in `ame_core` let `ame_neuro` components attach without modifying
the core planning/execution loop. Both compile to nothing when `AME_NEURO` is not
defined.

### HeuristicHook (Planner)

```cpp
// planner.h
#if defined(AME_NEURO)
struct ActionScore {
    unsigned ground_action_id;
    float    score;             // higher = prefer earlier in LAPKT traversal
};

using HeuristicHook =
    std::function<std::vector<ActionScore>(const WorldModel&,
                                           const std::vector<unsigned>& goal_ids)>;

class Planner {
    void setHeuristicHook(HeuristicHook h);
    void clearHeuristicHook();
    bool hasHeuristicHook() const;
};
#endif
```

**How it works in `Planner::solve()`:**

1. Hook fires before LAPKT; returns `vector<ActionScore>`.
2. Scores are sorted descending to produce a `action_order` permutation.
3. `WorldModel::projectToSTRIPS(strips, action_order)` adds actions to LAPKT
   in score order — higher-scored actions are tried first by BRFS at each level.
4. `lapkt_to_wm[i] = action_order[i]` maps LAPKT indices back to WM ground action
   indices in the plan result.
5. `PlanResult::heuristic_source` is set to `"neural_hook"` whenever the hook fires
   (even if it returns empty scores). This value is propagated to `PlanAuditLog`.

**Null-object guarantee:** When no hook is attached, `action_order` is the identity
permutation and `projectToSTRIPS(strips, action_order)` produces the same result
as the no-order overload.

### RepairHook (ExecutorComponent)

```cpp
// executor_component.h
#if defined(AME_NEURO)
// Returns non-empty BT XML to immediately re-execute a repair plan,
// or empty to fall through to the baseline FAILURE path.
// The closure captures PlanCompiler/ActionRegistry from its own scope.
using RepairHook = std::function<std::string(unsigned failed_step,
                                              const WorldModel& wm)>;

class ExecutorComponent {
    void setRepairHook(RepairHook h);
    void clearRepairHook();
    bool hasRepairHook() const;
};
#endif
```

**How it works in `ExecutorComponent::on_tick()`:**

1. BT tick returns `FAILURE`.
2. If `repair_hook_` is set and `inprocess_wm_` is non-null:
   - Hook is called with `(failed_step=0, *inprocess_wm_)`.
   - Non-empty BT XML → `loadAndExecute(xml)` restarts execution; publish RUNNING.
   - Empty or exception → fall through to baseline: halt tree, publish FAILURE.
3. External `PlannerComponent` handles full replanning after FAILURE.

The hook closure typically captures `PlanCompiler&`, `ActionRegistry&`, and
`Planner&` from the surrounding scope, runs a fast replan, and compiles the result
to BT XML:

```cpp
executor.setRepairHook([&planner, &compiler, &registry, &wm]
                        (unsigned /*step*/, const WorldModel& current_wm) -> std::string {
    auto result = planner.solve(current_wm);
    if (!result.success || result.steps.empty()) return {};
    return compiler.compile(result.steps, current_wm, registry);
});
```

---

## 10. PlanAuditLog Provenance (Layer 5 Extension)

`PlanAuditLog::Episode` gained four additive fields for neural provenance:

| Field | Default | Set when |
|-------|---------|---------|
| `heuristic_source` | `"symbolic"` | `Planner::solve()` fires HeuristicHook → copied by `PlannerComponent::recordAuditEpisode()` |
| `goal_source` | `"symbolic"` | Goal-interpretation integrations set before planning |
| `repair_source` | `"symbolic"` | Repair integrations set on success |
| `neuro_record_ids` | `[]` | Integrations append Layer 6 record IDs for cross-referencing |

These fields are serialised alongside existing episode fields in JSONL and are
transparent to consumers that don't read them.

---

## 11. Adding a New Integration

Follow these steps to build an integration using the infrastructure:

### Step 1: Define types

```cpp
struct MyRequest  { /* domain-specific content */ };
struct MyProposal { /* verified output type */ };
```

### Step 2: Implement codecs

```cpp
template<> struct RequestCodec<MyRequest> {
    static NeuralRequest encode(const MyRequest& r) { return {to_json(r)}; }
};
template<> struct ProposalCodec<MyProposal> {
    static std::optional<MyProposal> decode(const std::string& s) {
        return parse_my_proposal(s); // nullopt on bad JSON
    }
};
```

### Step 3: Implement verifier

```cpp
class MyVerifier : public IVerifier<MyProposal> {
    Verdict verify(const MyProposal& p, const WorldModel& wm) const override {
        // Pure, side-effect-free check against wm snapshot
        // return {true, "ok", {}} or {false, "reason", {"evidence..."}}
    }
};
```

### Step 4: Create backend and advisor

```cpp
// At startup
BackendRegistry reg;
reg.add(std::make_shared<MyBackend>("my_backend"));

auto advisor = std::make_shared<Advisor<MyRequest, MyProposal>>(
    reg.find("my_backend"),
    std::make_shared<MyVerifier>(),
    cfg.policy_for("my_integration"),
    &neuro_audit_log,
    "my_integration"
);
```

### Step 5: Wire the hook (if online planning path)

```cpp
// For heuristic guidance:
planner.setHeuristicHook([advisor, &wm]
    (const WorldModel& wm, const std::vector<unsigned>& goals)
    -> std::vector<ActionScore> {
    auto result = advisor->advise(build_request(wm, goals), wm);
    if (result.outcome == Outcome::Accepted)
        return result.proposal; // vector<ActionScore>
    return {};                  // symbolic fallback: identity ordering
});

// For repair proposals:
executor.setRepairHook([advisor, &planner, &compiler, &registry]
    (unsigned step, const WorldModel& wm) -> std::string {
    auto result = advisor->advise(build_repair_request(step, wm), wm);
    if (result.outcome != Outcome::Accepted) return {};
    auto plan = planner.solve_with_hint(result.proposal, wm);
    return plan.success ? compiler.compile(plan.steps, wm, registry) : "";
});
```

### Step 6: Add to NeuroConfig

```json
{
  "id": "my_integration",
  "enabled": true,
  "backend_id": "my_backend",
  "latency_budget_ms": 200.0
}
```

---

## 12. Testing

All neuro test binaries require `AME_NEURO=1` and link against `ame_neuro`:

| Binary | Coverage |
|--------|---------|
| `test_neuro_backend` | CancelToken, MockBackend scripts/hang/cancel, BackendExecutor circuit breaker + pool, BackendRegistry lookup |
| `test_neuro_advisor` | All 6 outcomes, retry logic, audit record count, disabled-path, error timing |
| `test_neuro_verifiers` | GroundedFluentVerifier known/unknown/allow-list, ForwardSimVerifier valid/precondition-fail/goal-fail, AuthorityView three modes |
| `test_neuro_audit` | NeuroAuditLog ID assignment, file sink, PlanAuditLog provenance defaults/fields, AuditIndex window/around/export/cite |
| `test_neuro_config` | NeuroConfig all-disabled/kill-switch/JSON-parse/find/policy-for, PlannerSeam baseline/hook-called/clear-hook |
| `test_neuro_contract` | Adversarial payloads, disabled-never-calls-backend, WI-1.3 criteria, malformed-no-crash |
| `test_neuro_replay` | Round-trip audit records, latency-pressure fallback, symbolic determinism |

### Build Presets

```bash
# Neuro-only, fast cycle (no Foxglove/gRPC)
cmake --preset neuro-dev
cmake --build --preset neuro-dev-release --parallel

# Run neuro tests only
ctest --test-dir build-neuro -R "test_neuro" -C Release
```

### Key Test Patterns

**Verify symbolic baseline is unchanged:**
```cpp
// Planner with no hook attached must produce identical results to AME_NEURO=OFF
Planner p;
auto r1 = p.solve(wm);  // no hook
p.setHeuristicHook(/*returns empty*/);
auto r2 = p.solve(wm);  // hook fires but returns {}
EXPECT_EQ(r1.steps, r2.steps);
```

**Verify timeout is enforced regardless of backend cooperation:**
```cpp
auto mb = std::make_shared<MockBackend>("hang", false);
mb->add_script({"", false, "", 10000.0, /*hang=*/true, /*non_coop=*/true});
BackendExecutor exec(mb, {});
// ...
FallbackPolicy policy = FallbackPolicy::hot_path();
policy.latency_budget_ms = 50.0;
auto result = advisor.advise(req, wm);
EXPECT_EQ(result.outcome, Outcome::TimedOutFellBack);
// elapsed must be near 50ms, not 10000ms
```
