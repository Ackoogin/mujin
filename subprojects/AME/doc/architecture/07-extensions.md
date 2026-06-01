# Extensions Roadmap (Future + Cross-Cutting)

This document now focuses on **future extensions and roadmap-level guidance**.

Completed extension material has been consolidated into the main architecture documents:

- Perception integration + thread safety -> [02-world-model.md](02-world-model.md)
- Runtime extension nodes (`InvokeService`, `ExecutePhaseAction`, `DelegateToAgent`) -> [04-execution.md](04-execution.md)
- ROS2 wiring for extension nodes -> [06-ros2.md](06-ros2.md)

---

## Temporal Planning (Extension 7) -- Future

PDDL 2.1 durative actions with STN (Simple Temporal Network) conversion. See [`temporal_extension_research.md`](../../../../doc/research/AME/temporal_extension_research.md) for full planner evaluation.

### Planner Strategy

LAPKT does not natively support temporal planning. Recommended approach uses an external temporal planner alongside the existing LAPKT STRIPS path:

| Option | Role | Rationale |
|--------|------|-----------|
| **OPTIC** (primary) | Near-term temporal backend | C++, PDDL 2.1 + 3.0 trajectory constraints, proven IPC planner, subprocess integration |
| **Aries** (evaluate) | Medium-term replacement candidate | Rust, actively developed, built-in STN/Difference Logic solver, hierarchical + temporal |
| **TFD** (fallback) | Alternative if OPTIC licence is problematic | GPL, PlanSys2 ROS2 integration exists |
| **LAPKT** (retained) | STRIPS execution planner | Unchanged for non-temporal domains; fast, auditable |

Integration pattern: `IPlannerBackend` abstraction selects LAPKT for STRIPS domains and the temporal planner for `:durative-actions` domains.

### Proposed Architecture

```
PDDL 2.1 Domain --> PddlParser (extended) --> IPlannerBackend
                                                   |
                                    +--------------+--------------+
                                    v              v              v
                               LapktBackend   OpticBackend   (future)
                               (STRIPS)       (subprocess)   AriesBackend
```

### Implementation Phases

| Phase | Work Item | Effort |
|-------|-----------|--------|
| 7a | PDDL 2.1 parser (`:durative-action`, `:functions`) | Medium |
| 7b | WorldModel numeric fluent store + audit | Medium |
| 7c | `IPlannerBackend` abstraction + OPTIC subprocess | Medium |
| 7d | STN data structure + consistency check | Low |
| 7e | PlanCompiler temporal mode (STN -> BT `Parallel`/`Timeout`) | High |
| 7f | VAL integration for temporal plan validation | Low |
| 7g | Temporal invariant monitoring (`ReactiveSequence`) | Low |
| 7h | End-to-end temporal tests | Medium |
| 7i | Aries evaluation + migration | Medium |

---

## Neuro-Symbolic Integration (Extension 8) -- Core Infrastructure Implemented

Neural components remain advisory; the symbolic stack stays authoritative.

Background research: [`neuro_symbolic_reasoning.md`](../../../../doc/research/AME/neuro_symbolic_reasoning.md).
Work plan (all items complete): [`neuro_symbolic_infrastructure_plan.md`](../../../../doc/plans/AME/neuro_symbolic_infrastructure_plan.md).

### Design Principle: Propose-Verify-Fallback

Every neural integration follows this mandatory envelope:

```
Neural backend --> proposal --> IVerifier<Proposal> --> (accept) --> use
                                                    --> (reject) --> symbolic fallback
```

A disabled or timed-out backend always falls back to the symbolic path with zero
impact on safety or correctness. The symbolic baseline is preserved unconditionally.

### Library Structure

The implementation lives in the `ame_neuro` optional static library, gated by
`-DAME_NEURO=ON` (propagated as `AME_NEURO=1` to all compilation units).
`ame_neuro` depends only on `ame_core` — no ROS2, no ONNX, no libcurl.

```
ame_core  (always built)
  |- Planner       -- HeuristicHook seam  (#if AME_NEURO)
  \- ExecutorComponent -- RepairHook seam (#if AME_NEURO)

ame_neuro (AME_NEURO=ON)
  |- Backend layer       INeuralBackend, BackendExecutor, BackendRegistry, MockBackend
  |- Advisor<Req,Prop>   propose-verify-fallback orchestrator
  |- Verifiers           IVerifier, GroundedFluentVerifier, ForwardSimVerifier
  |- Policy              FallbackPolicy, AuthorityView, NeuroConfig
  \- Observability       NeuroAuditLog (Layer 6), AuditIndex (cross-stream query)
```

### Implemented Components

| Component | Header | Description |
|-----------|--------|-------------|
| `CancelToken` / `CancelSource` | `neuro/cancel_token.h` | C++17 cooperative cancellation via `shared_ptr<atomic<bool>>` |
| `INeuralBackend` | `neuro/neural_backend.h` | Async transport abstraction; `submit()` returns `std::future<NeuralResponse>` |
| `BackendExecutor` | `neuro/backend_executor.h` | Bounded pool + circuit breaker; detached threads with promise/future |
| `BackendRegistry` | `neuro/backend_registry.h` | Named registry; owns backends and executors together |
| `MockBackend` | `neuro/mock_backend.h` | Scripted responses, cooperative/non-cooperative hang, latency simulation |
| `IVerifier<P>` | `neuro/verifier.h` | Deterministic symbolic gate; `Verdict{accepted, reason, evidence}` |
| `FallbackPolicy` | `neuro/fallback_policy.h` | `latency_budget_ms`, `max_retries`, `retry_backoff_ms`; four static factories |
| `Advisor<Req,Prop>` | `neuro/advisor.h` | Full PVF orchestrator; 6 outcomes; emits exactly one `NeuroAuditRecord` per call |
| `AuthorityView` | `neuro/authority_view.h` | `All` / `ConfirmedThenBelieved` / `ConfirmedOnly` — verifier trust policy |
| `GroundedFluentVerifier` | `neuro/grounded_fluent_verifier.h` | Checks fluent keys exist in WorldModel; optional allow-list governance hook |
| `ForwardSimVerifier` | `neuro/forward_sim_verifier.h` | Applies plan steps to snapshot copy; checks preconditions, effects, goals |
| `NeuroAuditLog` | `neuro/neuro_audit_log.h` | Layer 6 observability; append-only JSONL; one record per `Advisor::advise()` |
| `AuditIndex` | `neuro/audit_index.h` | Loads bt/wm/plan/neuro JSONL; `window()`, `around()`, `training_export()`, `cite()` |
| `NeuroConfig` | `neuro/neuro_config.h` | JSON-loadable per-integration config; global kill-switch; model version pinning |

### Integration Seams in ame_core

Two null-object hook seams are compiled into `ame_core` when `AME_NEURO=1`:

| Seam | Location | Purpose |
|------|----------|---------|
| `HeuristicHook` | `Planner` | Called before LAPKT solve; scores influence action ordering (Options A/D) |
| `RepairHook` | `ExecutorComponent` | Called before full replan on BT failure; return empty to use full replan (Option B) |

Both default to `nullptr`; symbolic behaviour is identical when no hook is attached.

### PlanAuditLog Provenance (Layer 5 extension)

`Episode` in `plan_audit_log.h` gained four additive provenance fields:

| Field | Default | Meaning |
|-------|---------|---------|
| `heuristic_source` | `"symbolic"` | `"neural_hook"` when HeuristicHook fires |
| `goal_source` | `"symbolic"` | Set by goal-interpretation integrations |
| `repair_source` | `"symbolic"` | `"neural_hook"` when RepairHook fires |
| `neuro_record_ids` | `[]` | IDs of `NeuroAuditRecord` entries for this episode |

### Adoption Order for Use-Case Integrations

1. **Low-risk** — goal interpretation, offline mission analysis (no online control path)
2. **Medium-risk** — planner-adjacent heuristic guidance, repair proposal suggestions
3. **High-risk** — learned heuristics in the online control loop (requires formal assurance evidence)

### Build Presets

| Preset | Purpose |
|--------|---------|
| `neuro-dev` | AME+NEURO, no Foxglove/gRPC — fast iteration cycle |
| `neuro-dev-release` / `neuro-dev-debug` | Release/Debug builds of the above |
| `all-on` | All features including `AME_NEURO=ON` |

