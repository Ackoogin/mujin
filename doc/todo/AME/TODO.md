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

## Planned-action contract: follow-ups (open)

The planned-action contract has landed. `PlanCompiler` emits one element per plan step
carrying its grounded state contract as ports; `PlannedActionNode` checks the
preconditions and commits the effects around the concrete action's own work;
`SimulatedAction`, the `PlannedAction` decorator and the `GoalReached` condition make up
the rest of the generated vocabulary; `SetWorldPredicate` is gone and
`CheckWorldPredicate` survives only for hand-written trees. `IWorldStateAccess` is the
seam through which all of this reads and writes facts, so the ROS2 executor can work
through its `GetFact` and `SetFact` services. Two mechanisms then decide how much
evidence a precondition needs: `ame_required_authority` raises the bar for a whole node
at load time, which is what DevEnv's "Preconditions" control sets, and
`(:confirmed-predicates ...)` in the domain routes named facts into
`ame_confirmed_preconditions`, which are only ever satisfied from observed state.
Architecture file [`04-execution.md`](../../../subprojects/AME/doc/architecture/04-execution.md)
describes the result.

What follows is what a review of that work, together with the restore in commit
`69d17c0`, found still outstanding.

### 1. The authoring tool silently drops `(:confirmed-predicates ...)`

`pddl_importer.cpp` reads `:types`, `:predicates`, `:constants`, `:domain`, `:objects`,
`:init` and `:goal`. It does not read `:confirmed-predicates`, the project model has no
field to hold it, and `pddl_generator.cpp` never writes it. So importing a domain that
declares confirmed predicates and generating from the project again drops the
declaration. The compiler then stops routing those facts into
`ame_confirmed_preconditions`, and an action that should have waited for observed state
accepts predicted state instead. Nothing reports this.

The fix is the ordinary one: read the section on import, hold it in the project, write it
on generate, and add a round-trip test alongside the existing importer and generator
tests. It is listed first because it is a silent downgrade of a safety-relevant
declaration rather than a missing feature.

### 2. Six BT node types are documented here but absent from the code

[`04-execution.md`](../../../subprojects/AME/doc/architecture/04-execution.md), under
"Guard and dispatch nodes", says `ExecutorComponent::on_configure()` registers
`AuthorisationGuard`, `GeofenceGuard`, `TawsGuard`, `EnsureAltitude`, `FormationHold` and
`IdentifyTarget`. None of those names exist anywhere in `subprojects/AME`. They exist in
the standalone AME repository, whose `executor_component.cpp` does register all six.
Missing here: the six sources and headers, the `world_fact_guard_utils.h` they share, and
`tests/test_native_bt_nodes.cpp`, which is around 584 lines in total. The prose arrived in
the restore; the code did not.

Four of them are condition nodes and are used the way `CheckWorldPredicate` is, from
hand-written and subtree bindings. `FormationHold` and `IdentifyTarget` are
`BT::StatefulActionNode` rather than `PlannedActionNode`. Their own test drives them from
hand-written XML, where being plain BT nodes is fine, but if a deployment binds either of
them as a simple-node action implementation it must merge the planned-action base ports
first, or BehaviorTree.CPP will refuse to load the tree.

**Blocked on the repository organisation decision.** Whether these are copied in, taken
from the standalone repository as the upstream, or left to the deployment that needs them
depends on which repository is going to own AME's node library. Settle that first;
copying now would only have to be undone.

### 3. The written contract is out of date

The port list in `04-execution.md` and in
[`planning_execution_user_guide.md`](../../../subprojects/AME/doc/guides/planning_execution_user_guide.md)
names five ports. There are seven: `ame_confirmed_preconditions` and
`ame_neg_preconditions` are undocumented.

Neither document mentions that an action with no registered implementation now aborts the
compile with "Unregistered action in plan" unless `PlanCompiler::setStubUnregisteredActions(true)`
is set, which the authoring shell and the Python bindings do and nothing else does.
Failing closed is the right default for production, but it is a change in behaviour that
an integrator meets as an exception, so it belongs in the guide.

### 4. No assurance requirement covers domain-declared confirmed predicates

`autonomy_assurance_plan.md` gained SR-05a, which covers a deployment overriding
`commitEffects()`. The second route to confirmed state, declaring predicates in the
domain, has no requirement of its own, and finding 1 above is exactly the sort of failure
such a requirement would catch.

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
- ~~**State-Authority Semantics:** Clarify `BELIEVED` (plan effects) vs `CONFIRMED` (perception) facts in WorldModel~~ — done. The distinction is now visible in execution as well as in the world model: see the planned-action contract above for `ame_required_authority` and `(:confirmed-predicates ...)`
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
- Dynamic registration from config file (YAML/JSON). `registerActionFile()` covers one
  action at a time from a pre-authored BT XML file; what is still missing is loading the
  whole mapping table from configuration
- Type-checked parameter binding against PDDL schema
- ~~Startup validation: all PDDL actions have registered implementations~~ — served at
  compile time instead: `PlanCompiler` aborts with "Unregistered action in plan" unless
  stub mode is enabled

### BT Nodes
- Failure taxonomy: TRANSIENT (retry), PERMANENT (replan + blacklist), FATAL (abort)
- `WaitForFact`, `Timeout` decorator, `RetryWithBackoff` nodes

### PlanCompiler
- Serialise compiled BT to file for inspection
- Emit DOT graph of causal structure
- ~~Complex DAG join-point synchronisation~~ — served conservatively by the phased
  compiler, which puts a barrier between execution phases. Joining at the individual
  step rather than at a phase boundary would still let more run in parallel

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

