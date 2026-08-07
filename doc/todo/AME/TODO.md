# Remaining Work

All vertical slice steps (1-8) and extensions 1-6 are complete, including multi-agent planning and ROS2 extension wiring. This document consolidates the roadmap for remaining work.

---

## PCL Integration Compliance (open; item 1 is an urgent correctness fix)

The AME component-layer migration to PCL is complete; what remains is the
ROS2 wrapper compliance gap left by the direct-call compatibility bridges.
Detail, rationale, and definition-of-done live in
[`pcl_migration_plan.md`](../../plans/AME/pcl_migration_plan.md).

| # | Item | Notes |
|---|------|-------|
| 1 | **Fix `ExecutorNode` ingress race** — `~/load_bt` / `~/stop_execution` call `loadAndExecute()` / `haltExecution()` directly from ROS2 service callbacks, racing `on_tick()` on the PCL executor thread | **Urgent correctness fix.** Add `enqueueBtXml()` / `enqueueHalt()` to `ExecutorComponent`, drain in `on_tick()`, drop `exec_mutex_` |
| 2 | Route ROS2 world-model writes through PCL ingress | `WorldModelComponent::setFact()` is mutex-protected but sidesteps the single-writer executor model |
| 3 | Remove `PlannerNode` detached planning thread | Route the ROS2 action to the existing PCL `plan` service |
| 4 | AME ROS2 PCL transport | `pcl_transport_t` backed by ROS2 so AME PCL ports surface as ROS2 topics/services; then strip the direct bypass services/publishers |
| 5 | Compatibility-test focus | Executor enqueue serialization, bridge-over-ingress, transport mapping, `combined_main` smoke |

Related deferred trigger on the PYRAMID side: "AME contract canonicalization"
(`doc/todo/PYRAMID/TODO.md`, WS-D), which would let AME consume the generated
ROS2 bindings directly.

---

## Planned actions own their preconditions and effects (open)

`PlanCompiler::emitActionUnit` wraps each plan step in a `Sequence` holding one
`CheckWorldPredicate` per precondition, then the registered action node, then one
`SetWorldPredicate` per add effect and delete effect. Those two node types were written
to get planning and execution working from end to end. They are the wrong shape to keep,
for two reasons.

A compiled tree for a three-step mission is about twenty nodes, of which three are the
mission. Everything that draws, reviews or replays that tree has to account for the rest.

More importantly, writing an effect as a node on the tree asserts that the world changed
because the action returned success. That is a fair account of predicted state and no
account at all of real state. How the state after an action is established is a decision
that belongs to the action, and the tree should not be making it on the action's behalf.

| # | Item | Notes |
|---|------|-------|
| 1 | Emit one node per plan step | The step's grounded preconditions, add effects and delete effects travel with the node, as attributes on the emitted element, so a compiled tree still describes itself to anything that reads it later |
| 2 | Planned-action base class in `ame_core` | Checks the preconditions before the concrete node's own work runs, applies the effects once it succeeds, and leaves to the derived class how the resulting state is established |
| 3 | Stand-in node for simulation and validation | Takes a configured number of ticks, succeeds or fails as configured, applies the declared effects as `BELIEVED` facts. This is what the tree does today, moved inside a node |
| 4 | Goal guard becomes a single condition node | The other place `CheckWorldPredicate` is emitted. Replacing it leaves no fact-level plumbing anywhere in generated output |
| 5 | Withdraw `CheckWorldPredicate` and `SetWorldPredicate` from mission execution | They occur only in generated output and in the tests that assert on it. No hand-written tree in the repository uses them |
| 6 | Update the tests and documents that describe the current tree shape | `test_plan_compiler`, `test_e2e_pipeline`, `test_integration`, `test_multi_agent`, `test_extensions`, `test_observability`, `test_e2e_spatial_routing`; architecture files `02-world-model.md`, `04-execution.md`, `05-observability.md`, `06-ros2.md`; the planning and execution user guide |

**Out of scope.** Establishing the state after an action from confirmed observation is a
deployment concern and stays outside core AME. A deployment supplies its own action nodes
and already decides how it confirms that a commanded action happened. What core AME owes
those nodes is the declared preconditions and effects, and a compiled tree that no longer
asserts effects on their behalf. The seam that supports this is already in place:
`FactAuthority` distinguishes `BELIEVED` from `CONFIRMED` facts and
`WorldModel::hasAuthorityConflict()` reports the two disagreeing. Related: the
State-Authority Semantics prerequisite under Extension 8.

**Why it is tracked here.** The change is in `ame_core` and the executor, so downstream
users can take it and check it against their own action nodes without waiting on, or
depending on, any authoring-tool work. The graphical authoring tool needs it as well, for
the simulation-run screens; item B0 of
[`authoring_tool_plan.md`](../../plans/AME/authoring_tool_plan.md) states the case from
that side and points here for the work itself.

---

## Extension 7: Temporal Planning (not started)

PDDL 2.1 durative actions with STN conversion. See [`temporal_extension_research.md`](../../research/AME/temporal_extension_research.md) for planner evaluation.

### Recommended Approach

- **Primary:** OPTIC (C++, PDDL 2.1 + 3.0, subprocess invocation)
- **Medium-term:** Evaluate Aries (Rust, built-in STN solver, hierarchical + temporal)
- **Fallback:** TFD (GPL, PlanSys2 ROS2 integration exists)
- **LAPKT retained** for STRIPS domains; temporal planner only for `:durative-actions`

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

## Extension 8: Neuro-Symbolic Integration

Neural components assist, but the symbolic system remains authoritative. See [`neuro_symbolic_reasoning.md`](../../research/AME/neuro_symbolic_reasoning.md) and [`neuro_symbolic_reasoning_review.md`](../../reviews/AME/neuro_symbolic_reasoning_review.md).

### Pre-requisites
- **State-Authority Semantics:** Clarify `BELIEVED` (plan effects) vs `CONFIRMED` (perception) facts in WorldModel
- **Neural Acceptance Criteria:** Latency budget (500ms), fallback to LAPKT on timeout/error, audit logging

### Phase 1 (Low-Risk)
- **LLM Goal Interpreter:** Natural language -> grounded PDDL goals; symbolic validation rejects invalid fluents
- **LLM Mission Analyst:** Offline analysis of audit logs; evidence-cited explanations

### Phase 2+ (Deferred)
- Planner-adjacent: LLM Heuristic Guide, Plan Repair
- Data-driven: Learned Heuristic, Anomaly Detector
- Authoring: Neural PDDL Domain Authoring

---

## Autonomy Assurance Gaps

From SACE Stage 8 analysis. See [`autonomy_assurance_plan.md`](../../plans/AME/autonomy_assurance_plan.md).

### High-Priority Verification

| Gap | Objective | Actions |
|-----|-----------|---------|
| Property-Based Planner Testing | Prove solver soundness over random valid domains | PDDL fuzzer + test harness asserting plan simulation |
| Adversarial Perception Testing | Reject malicious/stale/inconsistent data | Fault-injection middleware for `setFact()` ROS2 service |
| Safe-State Integration | Verify degradation to safe state | E2E tests: planning timeout, comms loss, unmapped actions -> fallback BT |

### Medium-Priority

| Gap | Objective |
|-----|-----------|
| OOC Detection | Bounds-checking on WorldModel + `CheckContext` BT node for geofencing |
| Threat Modelling | `threat_model.md` for spoofing, PDDL injection, denial-of-planning |
| Performance Benchmarks | CI thresholds on LAPKT time; BT tick stress tests at 50Hz |
| Compiler Correctness | BT -> linear action sequence round-trip verification |

---

## Hardening (production-readiness)

### Planner
- Solver portfolio: fast heuristic first, fall back to complete search
- Configurable time/node budget with timeout handling

### ActionRegistry
- Dynamic registration from config file (YAML/JSON)
- Type-checked parameter binding against PDDL schema
- Startup validation: all PDDL actions have registered implementations

### BT Nodes
- Failure taxonomy: TRANSIENT (retry), PERMANENT (replan + blacklist), FATAL (abort)
- `WaitForFact`, `Timeout` decorator, `RetryWithBackoff` nodes

### PlanCompiler
- Serialise compiled BT to file for inspection
- Emit DOT graph of causal structure
- Complex DAG join-point synchronisation

### MissionExecutor
- Progressive replan: retry -> local replan -> full replan -> relax goal -> abort
- Replan budget (max N replans)
- Pre-replan world model consistency checks

### PDDL Parser
- Structured error messages with line/column
- Schema validation: predicate arities, type consistency, unreachable goals

### Configuration & Packaging
- YAML config for domain paths, solver selection, action mappings, logging
- ROS2 lifecycle graceful shutdown with state persistence
- Debian/colcon package, Docker image, CI/CD pipeline

### Testing
- Property-based / fuzz tests for registration ordering
- Systematic fault injection
- Planning time vs. domain size benchmarks
- GitHub Actions CI: build matrix, ctest, coverage

---

## Post-Extension (future)

- Plan quality optimisation (anytime search, LPG-style local improvement)
- Advanced heuristics (landmark-based for larger domains)
- Dynamic object lifecycle (creation/destruction mid-mission)
- Resource-aware flow scheduling (mutex on shared resources)
- Dynamic re-allocation on agent failure
- Coordination constraints (mutex resources, sync barriers)
- Gazebo/Isaac Sim integration

