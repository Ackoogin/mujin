# Implementation Reference

**Status: IMPLEMENTED** — Vertical slice steps 1–8 complete. Extensions 1–6 complete.

This document describes the implemented architecture for the UAV search-and-classify example. See `architecture/` for the full reference; see `TODO.md` for remaining work.

---

## Source Tree Layout

```
include/ame/
    world_model.h
    type_system.h
    action_registry.h
    plan_compiler.h
    planner.h
    pddl_parser.h
    bt_logger.h                    # Layer 2: structured BT event stream
    wm_audit_log.h                 # Layer 3: world model audit log
    plan_audit_log.h               # Layer 5: plan audit trail
    foxglove_bridge.h              # Layer 4: Foxglove WebSocket bridge
    bt_nodes/
        check_world_predicate.h
        set_world_predicate.h
src/
    world_model.cpp
    type_system.cpp
    action_registry.cpp
    plan_compiler.cpp
    planner.cpp
    pddl_parser.cpp
    bt_logger.cpp
    wm_audit_log.cpp
    plan_audit_log.cpp
    foxglove_bridge.cpp
    bt_nodes/
        check_world_predicate.cpp
        set_world_predicate.cpp
    main.cpp
tests/
    CMakeLists.txt
    test_world_model.cpp
    test_action_registry.cpp
    test_plan_compiler.cpp
    test_pddl_parser.cpp
    test_planner.cpp
    test_integration.cpp
    test_e2e_pipeline.cpp
    test_observability.cpp         # Layers 1-5 observability tests
doc/
    concept.md
    extensions.md
    plan.md
    quickstart.md                  # Getting started + Foxglove integration
```

---

## Step 1: Project Scaffolding ✓

Build infrastructure — directories, CMake updates, Google Test integration.

- `include/ame/`, `src/`, `tests/`, `domains/` directory structure
- Google Test via FetchContent
- Test targets in `tests/CMakeLists.txt`

## Step 2: TypeSystem and WorldModel Core ✓

Authoritative world model with eager grounding.

- `TypeSystem`: type hierarchy + object registry
- `WorldModel`: predicate schemas, eager grounding, dynamic bitset storage, BiMap for fluent index ↔ string key, version counter
- Tests: `tests/test_world_model.cpp`

## Step 3: PDDL Projection (WorldModel → LAPKT) ✓

Project world model state into `STRIPS_Problem` for LAPKT solvers.

- `projectToSTRIPS()`, `currentStateAsSTRIPS()`
- Action schemas with preconditions, add/delete effects as fluent ID sets

## Step 4: PDDL Parser Integration (FF-Parser) ✓

Load domain/problem from `.pddl` files via LAPKT's FF-parser.

- `include/ame/pddl_parser.h`, `src/pddl_parser.cpp`
- UAV example: `domains/uav_search/domain.pddl`, `problem.pddl`

## Step 5: ActionRegistry ✓

Map PDDL action names to BT implementations.

- `registerAction()`, `registerActionSubTree()`, `registerActionSubTreeFile()`
- `resolve(action_name, params) → ActionImpl`

## Step 6: BT Node Types ✓

World-model-aware BT nodes.

- `CheckWorldPredicate`: ConditionNode querying `WorldModel::getFact()`
- `SetWorldPredicate`: SyncActionNode calling `WorldModel::setFact()`

## Step 7: Plan-to-BT Compiler ✓

Convert LAPKT plan into executable BT XML.

- **CausalGraph**: effect-precondition edges + delete-conflict tracking
- **Flow Extraction**: topological sort, independent flows → Parallel
- **Action Unit Generation**: precondition checks + action + effect updates
- **Tree Composition**: Sequence/Parallel/join-point patterns
- Sequential fallback mode for debugging

## Step 8: End-to-End Vertical Slice ✓

UAV search-and-classify example running end-to-end.

1. Parse PDDL → WorldModel
2. Register BT action mappings
3. Solve with LAPKT (BFS(f)/SIW)
4. Compile plan to BT
5. Execute tree to completion
6. Verify goal state + replan-on-failure

---

## Component Reference (Slice vs. Production)

This section documents what each component provides in the vertical slice vs. production needs. Extensions 1–6 are now complete; see `TODO.md` for remaining gaps.

### WorldModel

| Aspect | Slice delivers | Production needs | Extension |
|--------|---------------|------------------|-----------|
| Threading | Single-threaded, no locking | Versioned snapshots: BT reads consistent snapshot, perception writes to pending buffer, swap between ticks | ext 5 (Thread Safety) |
| Persistence | In-memory only, lost on exit | SQLite-backed state log; recoverable after restart | ext 1 Layer 3 |
| Audit log | `version()` counter only | Every `setFact()` emits structured entry (fact, value, source, timestamp, wm_version) to SQLite/JSONL | ext 1 Layer 3 |
| Numeric fluents | Boolean predicates only | PDDL 2.1 numeric fluents (fuel level, battery, distance) — requires extending bitset to typed value store | ext 7 (Temporal) |
| Conditional effects | Not supported | Actions whose effects depend on state at execution time — requires effect evaluation at tick time, not compile time | ext 7 |
| Object lifecycle | Objects added at init, never removed | Dynamic object creation/destruction mid-mission (e.g., discovered targets) — requires re-grounding without invalidating fluent indices | post-ext |

### TypeSystem

| Aspect | Slice delivers | Production needs | Extension |
|--------|---------------|------------------|-----------|
| Type hierarchy | Single inheritance (name → parent) | Sufficient for STRIPS. Multi-inheritance or interfaces needed only if domain complexity demands it | — |
| Type checking | Validated at grounding time | Runtime type assertions on `setFact()` calls from perception (reject malformed updates) | ext 3 (Perception) |

### PDDL Parser

| Aspect | Slice delivers | Production needs | Extension |
|--------|---------------|------------------|-----------|
| Language level | STRIPS (`:typing`, `:strips`) | PDDL 2.1 (`:durative-actions`, `:fluents`) for temporal planning | ext 7 |
| Parser backend | FF-parser via `libff_parser` | FF-parser handles STRIPS well. Temporal features may require PDDL4J or a custom parser | ext 7 |
| Error reporting | Parser errors surfaced as-is from FF | Structured error messages with line/column and domain-specific hints | hardening |
| Domain validation | Basic: parsed domain loads without crash | Schema validation: predicate arities, type consistency, unreachable goals detection | hardening |

### Planner (LAPKT Integration)

| Aspect | Slice delivers | Production needs | Extension |
|--------|---------------|------------------|-----------|
| Solver | Single solver: BFS(f) or SIW | Solver portfolio: try fast heuristic first, fall back to complete search. Configurable per domain | hardening |
| Timeout | None — solver runs until done or crashes | Configurable time/node budget. On timeout, return best partial plan or signal replanning with relaxed goal | hardening |
| Plan quality | First plan found (satisficing) | Optional plan optimisation pass (anytime search, plan improvement via LPG-style local search) | post-ext |
| Heuristic selection | Hardcoded (whatever BFS(f)/SIW defaults to) | Domain-specific heuristic configuration. Landmark-based heuristics for larger domains | post-ext |
| Multi-agent | Single-agent planning | MA-STRIPS or task allocation layer that decomposes goal among agents and plans per-agent | post-ext |
| Temporal | Not supported | Durative actions with temporal constraints. STN-based scheduling post-planning | ext 7 |

### ActionRegistry

| Aspect | Slice delivers | Production needs | Extension |
|--------|---------------|------------------|-----------|
| Registration | Static, at startup, programmatic | Dynamic registration: load from config file (YAML/JSON mapping PDDL names → BT implementations) | hardening |
| Parameter binding | String substitution (`{param0}`) | Type-checked binding: verify parameter types match PDDL schema at registration time | hardening |
| Validation | None — resolve fails silently if action not registered | Startup validation: check all PDDL actions have registered implementations. Warn on unused registrations | hardening |
| Service mapping | BT nodes only | `InvokeService` node type for PYRAMID SDK calls — maps PDDL actions to external service requests | ext 4 (PYRAMID) |

### BT Node Types

| Aspect | Slice delivers | Production needs | Extension |
|--------|---------------|------------------|-----------|
| Node set | `CheckWorldPredicate`, `SetWorldPredicate`, `ReplanOnFailure` | + `InvokeService` (PYRAMID), `WaitForFact` (perception-driven conditions), `Timeout` decorator, `RetryWithBackoff` | ext 4, hardening |
| Action implementations | Stubs (log and succeed) | Real implementations: ROS2 action clients, PYRAMID service calls, hardware drivers | ext 2, ext 4 |
| Error semantics | FAILURE = trigger replan | Failure taxonomy: TRANSIENT (retry), PERMANENT (replan with action blacklist), FATAL (abort mission) | hardening |
| Blackboard | WorldModel pointer in root blackboard | Per-subtree blackboard scoping for hierarchical plans. Parameter remapping for reusable sub-trees | ext 6 (Hierarchical) |

### Plan-to-BT Compiler

| Aspect | Slice delivers | Production needs | Extension |
|--------|---------------|------------------|-----------|
| Causal graph | Effect-precondition edges + delete-conflict tracking | Correct for STRIPS. Temporal planning requires temporal causal links (start/end time points) | ext 7 |
| Flow decomposition | Independent causal chains → Parallel | Correct for fully-independent flows. Resource contention (two actions needing same robot) requires resource-aware scheduling | post-ext |
| Join points | Blackboard-flag guard pattern | Works for simple joins. Complex DAGs may need explicit synchronisation nodes or barrier patterns | hardening |
| Compiled output | XML string | + serialise to file for offline inspection. + emit DOT graph of causal structure for debugging | ext 1 Layer 5 |
| Temporal actions | Not supported | Durative actions → STN conversion → timed Parallel/Sequence with deadline decorators | ext 7 |
| Plan audit | Not logged | Each compilation episode logged: init state, goal, solver, plan, causal graph, compiled XML | ext 1 Layer 5 |

### MissionExecutor

| Aspect | Slice delivers | Production needs | Extension |
|--------|---------------|------------------|-----------|
| Replan trigger | Any action FAILURE → replan | Failure classification: transient failures retry in-place; permanent failures replan with action blacklisting; fatal failures abort | hardening |
| Replan strategy | Full replan from current state | Progressive: (1) retry failed action, (2) replan from current step, (3) full replan, (4) relax goal, (5) abort | hardening |
| Goal management | Fixed goal set for entire mission | Dynamic goals: hierarchical goal decomposition, goal priority ordering, goal abandonment on timeout | ext 6 |
| Concurrency | Single tree, sequential tick | Hierarchical: top-level mission BT ticks phase sub-trees, each with own planning episode | ext 6 |
| Monitoring | Console output | Structured events to observability stack (BT transitions, replan episodes, world model changes) | ext 1 Layers 2–5 |
| Safety | None | Pre-conditions on replan: verify world model consistency before re-solving. Replan budget (max N replans before abort) | hardening |

### Observability — IMPLEMENTED

| Aspect | Slice delivers | Current state | Extension |
|--------|---------------|---------------|-----------|
| BT logging | `std::cout` prints | **Done:** `SqliteLogger` enabled (Layer 1) + `AmeBTLogger` (Layer 2) with JSONL + callback sinks | ext 1 Layers 1–2 |
| WM logging | `version()` counter | **Done:** `WmAuditLog` (Layer 3) with source-tagged fact changes, JSONL output | ext 1 Layer 3 |
| Live monitoring | None | **Done:** `FoxgloveBridge` (Layer 4) — Foxglove WebSocket server on `ws://localhost:8765` with `/bt_events` + `/wm_audit` channels | ext 1 Layer 4 |
| Plan audit | None | **Done:** `PlanAuditLog` (Layer 5) — JSONL with init state, goals, solver, timing, plan actions, compiled BT XML | ext 1 Layer 5 |
| Performance profiling | None | **Done:** `TreeObserver` wired (Layer 1). Chrome Tracing via `MinitraceLogger` available | ext 1 Layer 1 |

### Deployment

| Aspect | Slice delivers | Production needs | Extension |
|--------|---------------|------------------|-----------|
| Runtime | Single-process CLI executable | ROS2 node graph: WorldModel node, Planner node, Executor node, Perception nodes | ext 2 (ROS2) |
| Configuration | Hardcoded domain paths and solver | Config file (YAML): domain paths, solver selection, action registry mappings, logging sinks | hardening |
| Lifecycle | Start → run → exit | ROS2 lifecycle management: configure → activate → deactivate. Graceful shutdown with state persistence | ext 2 |
| Distribution | Monolithic | Single-node (in-process) and multi-node (distributed) from same code via ROS2 service abstraction | ext 2 |
| Packaging | CMake build, no install target | Debian/colcon package. Docker image. CI/CD pipeline with test + package stages | hardening |

### Testing

| Aspect | Slice delivers | Production needs | Extension |
|--------|---------------|------------------|-----------|
| Unit tests | GTest: WorldModel, ActionRegistry, PlanCompiler, PDDLParser | Same, plus property-based tests (fuzz predicate/object registration ordering) | hardening |
| Integration tests | Full pipeline with stubs | + tests with simulated perception (inject state changes during execution) | ext 3 |
| Fault injection | Single injected failure in Step 8 | Systematic: random action failures, perception delays, stale state, planner timeouts | hardening |
| Simulation | None | Gazebo/Isaac Sim integration: BT actions drive simulated robot, perception reads simulated sensors | post-ext |
| Benchmarks | None | Planning time vs. domain size. Tick throughput. Replan latency | hardening |
| CI | None | GitHub Actions: build matrix (GCC/Clang, Release/Debug), `ctest`, coverage report | hardening |

### Legend

- **ext N** — references extension N in `extensions.md`
- **ext 1 Layer N** — references specific observability layer in extension 1
- **hardening** — production-readiness work that doesn't require new architecture, just robustness improvements to existing components
- **post-ext** — future work beyond the extension roadmap; not yet planned in detail

---

## Dependencies

| Dependency | Source | Purpose | Status |
|------------|--------|---------|--------|
| BehaviorTree.CPP 4.6.2 | FetchContent | BT execution + SQLite logging | Integrated |
| LAPKT Devel2.0 (core) | FetchContent | STRIPS model, BRFS search | Integrated |
| Google Test 1.14 | FetchContent | Unit/integration testing (73 tests) | Integrated |
| SQLite3 | System | BT.CPP SQLite logging backend | Integrated |
| websocketpp 0.8.2 | FetchContent | Foxglove WebSocket server | Integrated (optional: `AME_FOXGLOVE`) |
| Standalone Asio 1.28 | FetchContent | Async I/O for websocketpp | Integrated (optional: `AME_FOXGLOVE`) |
