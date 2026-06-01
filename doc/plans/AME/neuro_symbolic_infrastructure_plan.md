# Neuro-Symbolic Reasoning — Core Infrastructure Work Plan

**Scope:** The shared, use-case-agnostic substrate that *every* neuro-symbolic
option (A–G) plugs into. This plan deliberately does **not** implement any
single integration (goal interpreter, heuristic guide, plan repair, mission
analyst, etc.). It builds the common plumbing — abstractions, the
propose-verify-fallback envelope, verification utilities, telemetry, policy,
and the test/assurance harness — so that each option later becomes a thin
adapter rather than a bespoke pillar.

**References:**
- `doc/research/AME/neuro_symbolic_reasoning.md` (options A–G, Extension 8)
- `doc/reviews/AME/neuro_symbolic_reasoning_review.md` (constraints, pre-requisites)
- `subprojects/AME/doc/architecture/07-extensions.md` (roadmap placement)
- `doc/plans/AME/autonomy_assurance_plan.md` (assurance trigger points)

---

## Design Principles (the invariants every neural component inherits)

These are derived directly from the review's "Architecture Constraints" and the
research doc's "Soundness and Safety" section. They are stated once here and
enforced by the infrastructure so individual options cannot violate them.

1. **Symbolic authority is absolute.** Neural components only ever *propose*.
   The symbolic stack (`WorldModel`, `Planner`/LAPKT, forward simulation) is the
   sole arbiter of correctness. Every proposal crosses a verification gate
   before it can affect mission state or the executed plan.
2. **Fallback is mandatory and automatic.** Any neural component that is
   disabled, unavailable, times out, errors, or whose proposal fails
   verification must transparently fall through to the existing deterministic
   path. No neural component is ever on the critical path.
3. **Bounded by budget.** Every neural call runs under an explicit latency
   budget and is cancellable. Exceeding the budget is a normal, logged outcome
   that triggers fallback — not an error.
4. **Fully auditable.** Every proposal, its verification verdict, and whether it
   affected behaviour is logged with enough provenance to replay it offline.
5. **`ame_core` stays neural-free.** Neural dependencies live in separate,
   optional libraries. With all neural flags `OFF`, the build, binaries, and
   behaviour are byte-for-byte the symbolic baseline.

---

## Architecture Overview

```
                +-------------------------------------------------------+
                |                   ame_core (unchanged)                |
                |   WorldModel  Planner  PlanCompiler  Executor  Logs   |
                +---------+--------------------+-------------------+-----+
                          | seam (hook ifaces) | seam              | seam
                          v                    v                   v
        +----------------------------------------------------------------+
        |                ame_neuro  (NEW, optional, AME_NEURO)           |
        |                                                                |
        |   IAdvisor<Req,Proposal>   ---- the propose-verify-fallback    |
        |        |          ^               envelope (templated)         |
        |        v          |                                            |
        |   INeuralBackend  |   IVerifier<Proposal>   FallbackPolicy     |
        |   (transport)     |   (symbolic gate)       (budget+retry)     |
        |        |          |          |                                 |
        |        |          +----------+--- NeuroAuditTrail (Layer 6)    |
        +--------+-----------------------------------------------------+-+
                 |                                                     
        +--------+---------+        +-------------------+              
        |  ame_neuro_llm   |        |  ame_neuro_onnx   |   (NEW, optional)
        |  (AME_LLM:       |        |  (AME_NEURAL:     |              
        |   libcurl HTTP)  |        |   ONNX Runtime)   |              
        +------------------+        +-------------------+              
```

- **`ame_neuro`** — backend-agnostic core: the advisor template, verifier
  interfaces, fallback policy, configuration, and the neuro audit trail.
  Depends only on `ame_core`. No transport, no ML runtime.
- **`ame_neuro_llm`** — `INeuralBackend` implementations over HTTP LLMs
  (libcurl), shared prompt/response plumbing, JSON contract enforcement.
- **`ame_neuro_onnx`** — `INeuralBackend` implementations over ONNX Runtime for
  sub-millisecond local models.

Options A–G become adapters: each defines a request type, a proposal type, a
verifier, and a prompt/feature mapping — and reuses everything else.

---

## Phase 0: Foundation & Build Integration

### WI-0.1: Optional library targets and build flags

**Description:** Add the three optional libraries and their CMake options,
following the established `AME_FOXGLOVE` pattern. Guard all neural code behind
`#if defined(AME_NEURO)` etc. so `ame_core` and the default build are untouched.

**Deliverables:**
- [ ] `option(AME_NEURO  "Neuro-symbolic core infrastructure" OFF)`
- [ ] `option(AME_LLM    "LLM backends (requires libcurl)"     OFF)` (implies `AME_NEURO`)
- [ ] `option(AME_NEURAL "ONNX Runtime backends"               OFF)` (implies `AME_NEURO`)
- [ ] `ame_neuro` static lib (`PUBLIC ame_core`, no transport deps)
- [ ] `ame_neuro_llm` lib gated on `AME_LLM` (`find_package(CURL)`)
- [ ] `ame_neuro_onnx` lib gated on `AME_NEURAL` (FetchContent ONNX Runtime, ~5 MB)
- [ ] New configure preset additions / `all-on` includes the neural flags
- [ ] CTest preset wiring for neural unit tests

**Acceptance criteria:** `cmake --preset all-off` is bit-identical in target set
to today. `cmake --preset default -DAME_NEURO=ON` builds `ame_neuro` and its
tests with no `ame_core` source changes. All 73 existing tests still pass.

**Dependencies:** None
**Effort:** Small

---

### WI-0.2: Backend transport abstraction (`INeuralBackend`)

**Description:** Define the single seam through which all proposals are
requested, independent of whether the backend is a cloud LLM, a local LLM, or an
ONNX model. Calls are asynchronous. The budget guarantee (WI-1.3) is enforced
**caller-side by abandonment**, not by trusting the backend: the `Advisor`
waits on the result only up to the budget and then proceeds to fallback
regardless of whether the backend has returned. `std::stop_token` is a
*best-effort, cooperative* signal that lets well-behaved backends release
resources early — it is explicitly **not** the mechanism that bounds wall-time.

This separation matters because `std::stop_token` cannot stop a backend blocked
inside HTTP/ONNX work, and `std::future` offers no kill/abort. The contract
therefore defines *isolation and abandonment* semantics so a non-cooperative or
stuck backend can never hang the advisor path.

**Deliverables:**
- [ ] `INeuralBackend` interface in `ame_neuro`:
  ```cpp
  struct NeuralRequest  { std::string kind; std::string payload; /* prompt or feature blob */ };
  struct NeuralResponse { bool ok; std::string payload; std::string error;
                          double latency_ms; std::string backend_id; std::string model_id; };

  class INeuralBackend {
  public:
      virtual ~INeuralBackend() = default;
      virtual BackendInfo info() const = 0;   // id, modality, nominal latency, cooperative?

      // Async. Implementations SHOULD observe `stop` to abort early and MUST
      // set a transport-level deadline <= the advertised max so the call
      // terminates on its own even if `stop` is ignored. The returned future
      // may be abandoned by the caller (see contract below); implementations
      // MUST tolerate their result being discarded.
      virtual std::future<NeuralResponse> submit(const NeuralRequest&,
                                                 std::stop_token stop) = 0;
  };
  ```
- [ ] **Contract clauses (documented in the header):**
  1. *Cooperation expected, not assumed.* A backend that ignores `stop` is
     still legal; correctness of the envelope must not depend on cooperation.
  2. *Self-terminating deadline.* Every backend MUST impose its own transport
     deadline/timeout so an abandoned call eventually completes and frees its
     resources, rather than blocking forever.
  3. *Abandonment is normal.* If the budget elapses first, the `Advisor`
     requests stop, stops waiting on the future, records `TimedOutFellBack`,
     and continues. The in-flight call is run on an isolated worker (below);
     its eventual result is discarded.
  4. `info().cooperative` advertises whether the backend honours `stop`,
     letting the policy/registry treat non-cooperative backends more
     conservatively (smaller pool slice, eager circuit-breaking).
- [ ] **Isolation:** backend calls execute on a bounded worker pool
      (`BackendExecutor`) with a max-in-flight cap and a circuit breaker.
      Abandoned (timed-out) slots are reclaimed only when the underlying call
      returns or hits its deadline; once outstanding abandoned calls saturate
      the pool, the breaker opens and further `submit`s report unavailable
      immediately — bounding resource leakage from a misbehaving backend.
- [ ] `NullBackend` (always reports unavailable) and `MockBackend` (scripted
      responses, injectable latency/error, *and a non-cooperative/hang mode that
      ignores `stop`*) for tests — both in `ame_neuro`.
- [ ] `BackendRegistry` for named lookup and hot-path/warm-path/cold-path tiering.

**Acceptance criteria:** A unit test drives `MockBackend` through the registry
and verifies: (a) cooperative cancellation via `stop_token` mid-flight; (b) a
**non-cooperative hanging** backend that ignores `stop` is abandoned and the
caller still returns within budget; (c) sustained abandonment opens the circuit
breaker so the pool cannot be exhausted; (d) latency is reported. No real network
or model required.

**Dependencies:** WI-0.1
**Effort:** Medium

---

## Phase 1: The Propose-Verify-Fallback Envelope

### WI-1.1: `IVerifier` — the symbolic gate contract

**Description:** Generalise "the symbolic component verifies" into one interface.
Every proposal type ships a verifier; the envelope refuses to apply any proposal
that does not pass. Verifiers are pure functions of symbolic state and proposal —
no neural dependency — so they are cheap to unit-test exhaustively.

**Deliverables:**
- [ ] Templated interface:
  ```cpp
  template <class Proposal>
  class IVerifier {
  public:
      struct Verdict { bool accepted; std::string reason; std::vector<std::string> evidence; };
      virtual ~IVerifier() = default;
      virtual Verdict verify(const Proposal&, const WorldModel&) const = 0;
  };
  ```
- [ ] `AlwaysReject` / `AlwaysAccept` reference verifiers for tests.
- [ ] Documented contract: verifiers MUST be deterministic, side-effect-free,
      and operate on a `WorldModel` snapshot (no mutation of authoritative state).

**Acceptance criteria:** Verdict carries a machine-usable rejection reason and
cited evidence keys, exercised by unit tests.

**Dependencies:** WI-0.1
**Effort:** Small

---

### WI-1.2: `Advisor` — the envelope orchestrator

**Description:** The reusable engine that ties backend + verifier + policy + audit
into the propose-verify-fallback loop. This is the heart of the infrastructure;
options A–G instantiate it with their own request/proposal/verifier types and
write almost no orchestration code of their own.

**Deliverables:**
- [ ] Templated orchestrator:
  ```cpp
  template <class Request, class Proposal>
  class Advisor {
  public:
      struct Result {
          enum class Outcome { Accepted, RejectedFellBack, TimedOutFellBack,
                               UnavailableFellBack, ErroredFellBack, Disabled };
          Outcome outcome;
          std::optional<Proposal> proposal;   // present iff Accepted
          IVerifier<Proposal>::Verdict verdict;
          double total_latency_ms;
      };
      // Returns Accepted proposal, or signals caller to use its symbolic fallback.
      Result advise(const Request&, const WorldModel&);
  };
  ```
- [ ] Wiring: serialise `Request` → `NeuralRequest`, submit under budget, parse
      `NeuralResponse` → `Proposal`, run `IVerifier`, apply `FallbackPolicy`,
      emit one `NeuroAuditTrail` record per call.
- [ ] Per-proposal-type traits seam: `RequestCodec<Request>` (to payload) and
      `ProposalCodec<Proposal>` (from payload) — the only thing an option must
      supply besides its verifier.

**Acceptance criteria:** A table-driven test using `MockBackend` exercises every
`Outcome`: accepted, verifier-rejected, timeout, unavailable, malformed-response,
disabled — each lands on the correct outcome and emits exactly one audit record.

**Dependencies:** WI-0.2, WI-1.1, WI-3.1
**Effort:** Large

---

### WI-1.3: `FallbackPolicy` — budget, retry, and degradation

**Description:** Centralise the timing and retry rules the review demands
("strict timeout budgets", "bounded retry policy before falling back"). Policy is
data, not code, so it is configurable per integration and per deployment tier.

**Deliverables:**
- [ ] `FallbackPolicy` struct: `latency_budget_ms`, `max_retries`,
      `retry_backoff_ms`, `on_reject` (FallBack | RetryWithFeedback),
      `enabled` flag, `tier` (Hot/Warm/Cold).
- [ ] Deterministic clock injection so budget behaviour is testable without
      real waits.
- [ ] Hard guarantee: total wall-time across retries never exceeds the budget;
      the symbolic fallback is reachable within a bounded delay regardless of
      backend behaviour. This is enforced by **abandonment** (WI-0.2): the
      advisor `wait_for`s each attempt's future up to the remaining budget,
      requests cooperative stop, and — whether or not the backend honours it —
      stops waiting and falls back. The guarantee never depends on the backend
      cancelling or returning.

**Acceptance criteria:** Unit tests prove the wall-time bound holds under
(a) a *cooperative* backend that hangs until `stop`, (b) a *non-cooperative*
backend that ignores `stop` and blocks past the budget (must be abandoned),
(c) a backend that errors immediately, and (d) a backend that returns just after
the budget expires (late result is discarded, outcome is `TimedOutFellBack`).

**Dependencies:** WI-1.2
**Effort:** Medium

---

## Phase 2: Reusable Symbolic Verification Utilities

These are the concrete verifiers/utilities that multiple options share. They live
in `ame_neuro` and depend only on `ame_core`, so they are reusable and trivially
testable. (They are *building blocks*, not the options themselves.)

### WI-2.1: Grounded-fluent validator

**Description:** Validate that a set of fluent/predicate names resolve to real
grounded entries in a `WorldModel`. Shared by any option that proposes
goals, facts, or fluent references.

**Deliverables:**
- [ ] `GroundedFluentVerifier`: each proposed fluent key exists in the
      `WorldModel` fluent index; unknown keys rejected with the offending key
      cited as evidence.
- [ ] Optional allow-list hook (`std::function<bool(string)>`) for the review's
      "allowed-goal filtering / goal authorization" governance concern.

**Acceptance criteria:** Accepts only keys present in the grounded model;
rejection evidence names every invalid key.

**Dependencies:** WI-1.1
**Effort:** Small

---

### WI-2.2: Forward-simulation plan verifier

**Description:** Apply a proposed sequence of `PlanStep`s to a *copy* of the
`WorldModel`, checking each step's preconditions and effects, and confirm the
goal is reached. This is the shared gate for any option that proposes plan
fragments (full plans, suffixes, repairs). Reuses existing grounding and the
same precondition/effect semantics LAPKT relies on.

**Deliverables:**
- [ ] `ForwardSimVerifier`: takes proposed `std::vector<PlanStep>` + goal fluents;
      returns accept iff every precondition holds in sequence and the goal set is
      satisfied at the end. On reject, cites the first failing step + unmet
      precondition.
- [ ] Operates on a `WorldStateData` snapshot clone — never mutates authoritative
      state. Respects `FactAuthority` (see WI-2.3).
- [ ] Reuses `WorldModel` grounding utilities; no duplicate STRIPS logic.

**Acceptance criteria:** Hand-built valid plan accepted; plan with a violated
precondition at step *k* rejected with *k* and the predicate named. Verified
equivalent to a LAPKT-produced plan for a shared fixture.

**Dependencies:** WI-1.1
**Effort:** Large

---

### WI-2.3: State-authority resolution for verification inputs

**Description:** Resolve the review's open question (gap #4) at the
infrastructure level: when a verifier simulates from "current state", which facts
count? The codebase already tags facts `BELIEVED` vs `CONFIRMED`
(`FactAuthority` in `world_model.h`, `FactAuthorityLevel` at the backend
boundary). This WI defines and documents the policy verifiers use, so plan
repair and analysis rest on explicit semantics rather than ambiguity.

**Deliverables:**
- [ ] `AuthorityView` enum on verifier construction:
      `ConfirmedOnly` | `ConfirmedThenBelieved` | `All`.
- [ ] Default `ConfirmedThenBelieved`: confirmed (perception) facts override
      believed (plan-applied) predictions when both exist.
- [ ] Documented semantics added to `02-world-model.md` cross-reference and the
      assurance plan: believed = optimistic prediction; confirmed = observed
      truth; verifiers must be told which to trust.

**Acceptance criteria:** A fixture where a believed fact and a confirmed fact
conflict produces different (documented) verifier verdicts under each
`AuthorityView`.

**Dependencies:** WI-2.2
**Effort:** Medium

---

## Phase 3: Observability, Audit & Data Substrate

### WI-3.1: Neuro audit trail (Observability Layer 6)

**Description:** Extend the 5-layer observability stack with a dedicated,
append-only neuro audit trail recording every advisor call. This is the single
provenance source for replay, assurance, and "did the neural component affect
behaviour?" questions.

**Deliverables:**
- [ ] `NeuroAuditLog` (mirrors `PlanAuditLog` design: in-memory + optional JSONL
      sink, `ame_neuro_events.jsonl`). One record per `Advisor::advise` call:
  ```
  record_id, ts_us, integration_kind, backend_id, model_id,
  request_digest, proposal_digest, outcome, verdict_reason,
  evidence[], latency_ms, retries, affected_behaviour(bool)
  ```
- [ ] Request/proposal stored as digests + bounded raw payload (full payload
      gated by a verbosity flag to control log volume).
- [ ] Foxglove channel registration behind `AME_FOXGLOVE` so neuro events stream
      alongside existing layers.

**Acceptance criteria:** Every advisor outcome produces exactly one JSONL record;
records round-trip parse; an accepted proposal is linkable to the plan/BT it
influenced.

**Dependencies:** WI-0.1
**Effort:** Medium

---

### WI-3.2: PlanAuditLog provenance fields

**Description:** Close the loop between neural proposals and the planning record
the research doc asks for (`heuristic_source` and friends), so a plan episode
self-documents any neural influence.

**Deliverables:**
- [ ] Additive optional fields on `PlanAuditLog::Episode`:
      `neuro_record_ids[]` (links to Layer 6), `heuristic_source`
      (`"symbolic"` default), `goal_source`, `repair_source`.
- [ ] Defaults preserve current JSONL shape when neural is off (purely additive).

**Acceptance criteria:** With neural off, episode JSON is unchanged. With an
accepted proposal, the episode references the corresponding `NeuroAuditLog`
record id.

**Dependencies:** WI-3.1
**Effort:** Small

---

### WI-3.3: Log indexing / windowing & training-data export

**Description:** The shared retrieval substrate the review flags as a
pre-requisite for evidence-review and learned-model options. A use-case-agnostic
reader that indexes the existing JSONL streams (BT events, WM audit, plan audit,
neuro audit) by time and entity, returns bounded windows, and exports labelled
samples — without prescribing how any option consumes them.

**Deliverables:**
- [ ] `AuditIndex`: load N JSONL streams, build time + entity (object/fluent)
      indices, query `window(t0, t1)` and `around(fact_key, ±k)`.
- [ ] Token/size-bounded window assembly (for context-limited LLM consumers).
- [ ] `TrainingExport`: emit `(state, goal, plan, cost, outcome)` tuples from
      successful plan episodes (feeds learned-heuristic/anomaly options later).
- [ ] Evidence-citation helper: map a free-form reference back to concrete log
      record ids (enforces "cite evidence rather than opinion").

**Acceptance criteria:** Given fixture JSONL, `window()`/`around()` return the
correct records; a bounded window respects a size cap; export tuples validate
against a documented schema.

**Dependencies:** WI-3.1
**Effort:** Large

---

## Phase 4: Configuration, Policy & Lifecycle

### WI-4.1: Neuro configuration & policy envelope

**Description:** One declarative place to enable/disable integrations, bind them
to backend tiers, set budgets, and pin model versions — so deployment posture is
data, not recompilation. Extends the existing `PolicyEnvelope`
(`autonomy_backend.h`) rather than inventing a parallel mechanism.

**Deliverables:**
- [ ] `NeuroConfig` (JSON-loadable): per-integration `{enabled, backend_id,
      model_id, FallbackPolicy, verbosity, authority_view}`.
- [ ] Extend `PolicyEnvelope` with an optional `neuro` block; default = all
      disabled (pure symbolic).
- [ ] Global kill-switch and per-integration kill-switch honoured at runtime by
      `Advisor` (flips outcome to `Disabled`, logs it).
- [ ] Model/version pinning recorded in every `NeuroAuditLog` record (data
      lineage pre-requisite).

**Acceptance criteria:** Config with everything disabled yields pure-symbolic
behaviour and `Disabled` audit outcomes; flipping one integration on routes only
that path through its advisor.

**Dependencies:** WI-1.2, WI-1.3, WI-3.1
**Effort:** Medium

---

### WI-4.2: Integration seams in `ame_core` (hooks, no logic)

**Description:** Add the minimal, neural-free extension points in `ame_core`
where advisors *can* attach, defaulting to today's behaviour. These are empty
seams — the actual options are out of scope — but they must exist for the
infrastructure to be usable and must compile/behave identically when unused.

**Deliverables:**
- [ ] `Planner` gains an optional, default-null hook for action-ordering /
      heuristic advice consumed inside search tie-breaking (seam for Options A/D).
      `Planner::solve(const WorldModel&)` signature unchanged.
- [ ] Executor/replan path gains an optional, default-null repair-proposal hook
      consulted *before* full replanning (seam for Option B).
- [ ] Goal-ingress and evidence-sink seams (seam for Options E/F/G) exposed via
      existing callback/sink patterns (`pushIntent`, execution sinks).
- [ ] Each hook is `#if defined(AME_NEURO)`-guarded or a null-object default so
      the symbolic build is unaffected.

**Acceptance criteria:** With no advisor attached, planner/executor outputs are
identical to baseline across the existing test suite (all 73 pass). Attaching a
`MockBackend`-backed advisor demonstrably influences tie-breaking/repair while
still producing a verified result.

**Dependencies:** WI-1.2
**Effort:** Large

---

## Phase 5: Testing & Assurance Harness

### WI-5.1: Neural test doubles & contract tests

**Description:** Make every neural path testable with zero network/model
dependency, and pin the propose-verify-fallback contract with adversarial cases
the review explicitly requires.

**Deliverables:**
- [ ] `MockBackend` scenarios: valid, malformed, empty, oversized, slow,
      hanging, erroring, adversarial/out-of-context responses.
- [ ] Contract test suite asserting: invalid proposals are *always* rejected;
      fallback always reachable within budget; exactly one audit record per call;
      symbolic output unchanged when disabled.
- [ ] Codec fuzz tests for `RequestCodec`/`ProposalCodec` (garbage in → clean
      rejection, never a crash or unverified apply).

**Acceptance criteria:** Suite runs offline in CI; adversarial responses never
produce an accepted-yet-invalid proposal.

**Dependencies:** WI-1.2, WI-2.x
**Effort:** Medium

---

### WI-5.2: Replay & latency-pressure harness

**Description:** Validate behaviour against recorded missions and under timing
stress, per the review's "replay tests using audit logs" and "performance tests
under timeout pressure".

**Deliverables:**
- [ ] Replay driver: feed recorded JSONL through `AuditIndex` into advisors with
      `MockBackend` replaying historical proposals; assert verdicts/outcomes match
      expectations.
- [ ] Latency-injection runs proving the wall-time bound (WI-1.3) holds
      end-to-end and that planning/execution still complete via fallback.
- [ ] Determinism check: with neural disabled, replayed runs reproduce baseline
      plans bit-for-bit.

**Acceptance criteria:** Replays are reproducible; no latency profile causes a
missed fallback or an unverified apply.

**Dependencies:** WI-3.3, WI-5.1
**Effort:** Medium

---

### WI-5.3: Assurance trigger points & documentation alignment

**Description:** Document, at the infrastructure level, the assurance posture the
review asks for — when AMLAS/SACE obligations attach, what evidence each maturity
level needs, and the acceptance criteria a concrete integration must meet before
it may run in the control path. Also fix the cross-document gaps the review
identified.

**Deliverables:**
- [ ] "Neural component acceptance criteria" checklist (latency budget, fallback
      path, minimum logging fields, operator visibility) in this plan, referenced
      from `autonomy_assurance_plan.md`.
- [ ] Assurance trigger table: advisory/offline (low) vs planner-adjacent
      (medium, bounded experiment) vs in-loop (high, AMLAS required).
- [ ] Roadmap fix: register "Extension 8: Neuro-Symbolic Reasoning" explicitly
      (research doc says Extension 8; `07-extensions.md` lists it as a
      Candidate — align wording and dependencies).
- [ ] Align assurance-plan wording so goal *interpretation* is described as a
      future capability, not an implemented one (review gap #2).

**Acceptance criteria:** A reviewer can determine, from the docs alone, exactly
what evidence any proposed integration needs before each deployment tier.

**Dependencies:** WI-4.1
**Effort:** Small

---

## Deliverable Summary

| Phase | Work item | Effort | Gates |
|-------|-----------|--------|-------|
| 0 | WI-0.1 Build flags & libs | S | `ame_core` unchanged; baseline tests pass |
| 0 | WI-0.2 `INeuralBackend` | M | Async + cancellable, mockable |
| 1 | WI-1.1 `IVerifier` | S | Deterministic, side-effect-free |
| 1 | WI-1.2 `Advisor` envelope | L | All outcomes covered, 1 audit/call |
| 1 | WI-1.3 `FallbackPolicy` | M | Wall-time bound proven |
| 2 | WI-2.1 Fluent validator | S | Rejects unknown keys |
| 2 | WI-2.2 Forward-sim verifier | L | Matches LAPKT semantics |
| 2 | WI-2.3 Authority resolution | M | Documented BELIEVED/CONFIRMED policy |
| 3 | WI-3.1 Neuro audit (Layer 6) | M | One record per call |
| 3 | WI-3.2 PlanAudit provenance | S | Additive, off = unchanged |
| 3 | WI-3.3 Index/window/export | L | Bounded windows, schema'd export |
| 4 | WI-4.1 Config & policy | M | Kill-switches, version pinning |
| 4 | WI-4.2 Core seams | L | Baseline identical when unused |
| 5 | WI-5.1 Test doubles/contract | M | Adversarial never accepted |
| 5 | WI-5.2 Replay/latency harness | M | Reproducible, bound holds |
| 5 | WI-5.3 Assurance & docs | S | Tier evidence documented |

**Critical path:** WI-0.1 → WI-0.2 → WI-1.1 → WI-1.2 → (WI-2.2, WI-3.1) →
WI-4.2. Once that spine exists, options A–G are each a small adapter:
*define Request/Proposal codecs + a verifier (usually reusing WI-2.x) + a prompt
or feature mapping*, then register an `Advisor` against a backend. No option
re-implements orchestration, budgeting, fallback, or audit.

## Explicitly Out of Scope (deferred to per-option plans)

- Any concrete prompt engineering, goal-interpretation logic, heuristic scoring,
  repair strategy, mission-analyst Q&A, or anomaly model.
- Choice of specific LLM/model or training pipelines.
- UI/operator surfaces for reviewing proposals.

These build *on* this infrastructure and are sequenced per the adoption order in
the research doc and review (Phase 1: E, F → Phase 2: A, B → Phase 3: D, G →
Phase 4: C).
