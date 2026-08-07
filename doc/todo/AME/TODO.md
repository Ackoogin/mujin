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

## Planned-action contract: follow-ups (one open, blocked)

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

A review of that work, together with the restore in commit `69d17c0`, found four things
still outstanding. Three are now done; the fourth is blocked on a decision.

### 1. The authoring tool silently drops `(:confirmed-predicates ...)` — done

`pddl_importer.cpp` now reads the section, `PredicateDef` carries a `confirmed` flag that
the project file stores, and `pddl_generator.cpp` writes the section back out when any
predicate is flagged. The predicate editor has a checkbox for it, so a declaration can be
made in the tool as well as preserved through an import. An entry naming a predicate the
domain does not declare fails the import rather than being ignored. Covered by tests for
import, an import-and-generate round trip, the undeclared-name case, the flag surviving
save and load, and the generated section reaching `WorldModel` through `PddlParser`.

### 2. The written contract is out of date — done

`04-execution.md` now lists all seven planned-action ports in a table, and
[`planning_execution_user_guide.md`](../../../subprojects/AME/doc/guides/planning_execution_user_guide.md)
lists the six the compiler emits plus `ame_required_authority`. Both explain how
`(:confirmed-predicates ...)` and `ame_required_authority` differ, which is who decides
and how far the demand reaches.

Both documents, and the pitfalls section of
[`long_running_action_integration.md`](../../../subprojects/AME/doc/guides/long_running_action_integration.md),
now say that an action with no registered implementation aborts the compile with
"Unregistered action in plan", and that `PlanCompiler::setStubUnregisteredActions(true)`
is for tools reasoning about a model rather than for integrations.

### 3. No assurance requirement covers domain-declared confirmed predicates — done

`autonomy_assurance_plan.md` gained SR-05b, which requires a declared predicate to reach
the compiled tree as an `ame_confirmed_preconditions` entry, and requires any tool that
reads a domain and writes one back to preserve the declaration. Both of its verification
methods now exist: the compiler test in `test_plan_compiler.cpp` and the authoring tool's
round-trip test. The operating-conditions table in the same document no longer describes
precondition gating as `CheckWorldPredicate` nodes.

### 4. Six BT node types were documented here but absent from the code — references removed, code still open

[`04-execution.md`](../../../subprojects/AME/doc/architecture/04-execution.md) claimed
that `ExecutorComponent::on_configure()` registers `AuthorisationGuard`, `GeofenceGuard`,
`TawsGuard`, `EnsureAltitude`, `FormationHold` and `IdentifyTarget`. None of those names
exist anywhere in `subprojects/AME`; they exist in the standalone AME repository, along
with the `world_fact_guard_utils.h` they share and `tests/test_native_bt_nodes.cpp`,
around 584 lines in total. The prose arrived in the restore; the code did not.

Since the code is not here, the documents no longer name it. `04-execution.md` and
`long_running_action_integration.md` now describe what the executor actually registers,
and say that a deployment wanting a bespoke guard node registers its own, deriving it
from `PlannedActionNode` or merging `withBasePorts(...)` so that BehaviorTree.CPP will
load a compiled tree that uses it. Nothing in this repository is left describing nodes
that are not in it.

**Whether the nodes themselves should be here is still blocked on the repository
organisation decision.** Copying them in, taking the standalone repository as the
upstream, or leaving them to the deployment that needs them all depend on which
repository is going to own AME's node library. Settle that first; copying now would only
have to be undone. Note that four of the six are condition nodes used the way
`CheckWorldPredicate` is, from hand-written and subtree bindings, while `FormationHold`
and `IdentifyTarget` are plain `BT::StatefulActionNode`s — fine in the hand-written XML
their own test drives them from, but a deployment binding either as a simple-node action
implementation must merge the planned-action base ports first.

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

