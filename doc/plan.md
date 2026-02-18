# Implementation Plan

Minimal vertical slice: UAV search-and-classify example running end-to-end. Architecture is described in `concept.md`; post-slice extensions in `extensions.md`.

---

## Source Tree Layout

```
include/mujin/
    world_model.h
    type_system.h
    action_registry.h
    plan_compiler.h
    pddl_parser.h
    bt_nodes/
        check_world_predicate.h
        set_world_predicate.h
        replan_on_failure.h
src/
    world_model.cpp
    type_system.cpp
    action_registry.cpp
    plan_compiler.cpp
    pddl_parser.cpp
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
    test_integration.cpp
domains/
    uav_search/
        domain.pddl
        problem.pddl
```

---

## Step 1: Project Scaffolding

**Goal:** Build infrastructure — directories, CMake updates, Google Test integration.

- Create `include/mujin/`, `src/`, `tests/`, `domains/` directory structure
- Add Google Test via FetchContent in CMakeLists.txt
- Add a `tests/CMakeLists.txt` with a test target
- Update `src/CMakeLists.txt` to glob headers from `include/mujin/`
- Verify build still works with existing `main.cpp`

**Acceptance:** `cmake --build` succeeds, `ctest` runs an empty test suite.

## Step 2: TypeSystem and WorldModel Core

**Goal:** Implement the authoritative world model with eager grounding.

**Files:** `include/mujin/type_system.h`, `include/mujin/world_model.h`, `src/type_system.cpp`, `src/world_model.cpp`

- `TypeSystem`: type hierarchy (name → parent), object registry (name → type)
- `WorldModel`:
  - `registerPredicate(name, param_types)` — stores predicate schemas
  - `addObject(name, type)` — registers object; triggers eager grounding of all predicates that can bind to this type
  - Grounded facts stored in a **dynamic bitset** (compact; LAPKT's own `Bit_Set` or `std::vector<uint64_t>`) not `std::vector<bool>`
  - `BiMap<unsigned, std::string>` for fluent index ↔ string key
  - `setFact(string_key, bool)` and `getFact(string_key) → bool` for string-based access (BT nodes)
  - `setFact(fluent_id, bool)` and `getFact(fluent_id) → bool` for index-based access (LAPKT projection)
  - `version()` — monotonic counter, incremented on any state change

**Tests:** `tests/test_world_model.cpp`
- Register types, objects, predicates
- Verify eager grounding produces expected fluent count
- Set/get facts by string key and by index
- Verify version increments

**Acceptance:** All unit tests pass.

## Step 3: PDDL Projection (WorldModel → LAPKT)

**Goal:** Project world model state into a `STRIPS_Problem` usable by LAPKT solvers.

**Added to:** `world_model.h/.cpp`

- `projectToSTRIPS(STRIPS_Problem& prob)` — maps fluents, actions, init state, goal state
- `currentStateAsSTRIPS(const STRIPS_Problem& prob) → State*` — snapshot of current state as LAPKT state vector
- Action schemas stored in WorldModel or a separate `DomainModel`:
  - Preconditions, add effects, delete effects (as fluent ID sets)
  - Used both for STRIPS projection and for causal graph construction

**Tests:** `tests/test_world_model.cpp` (extended)
- Build UAV domain programmatically
- Project to STRIPS_Problem
- Verify fluent/action counts match
- Verify init/goal state correctness

**Acceptance:** Round-trip: WorldModel → STRIPS_Problem → State matches expected values.

## Step 4: PDDL Parser Integration (FF-Parser)

**Goal:** Load domain and problem from `.pddl` files instead of programmatic construction.

**Approach:** Enable LAPKT's FF-parser (`ff_to_aptk`).

- Add `libff_parser` as a FetchContent dependency (from `https://github.com/LAPKT-dev/libff_parser`)
- Add `${LAPKT_SRC}/translate/pddl/ff/ff_to_aptk.cxx` to the `lapkt_core` build
- Create `include/mujin/pddl_parser.h` and `src/pddl_parser.cpp`:
  - Wraps `aptk::FF_Parser::get_problem_description()`
  - Populates a `WorldModel` from the parsed `STRIPS_Problem` (reverse projection: extract types, objects, predicates, facts from the grounded problem)
- Write UAV example domain files: `domains/uav_search/domain.pddl`, `domains/uav_search/problem.pddl`

**Fallback:** If `libff_parser` integration proves problematic, implement a minimal PDDL tokenizer for STRIPS-level domains.

**Tests:** `tests/test_pddl_parser.cpp`
- Parse UAV domain + problem
- Verify WorldModel contains expected types, objects, predicates, initial facts

**Acceptance:** `WorldModel` populated identically whether constructed programmatically or parsed from `.pddl` files.

## Step 5: ActionRegistry

**Goal:** Map PDDL action names to BT implementations.

**Files:** `include/mujin/action_registry.h`, `src/action_registry.cpp`

- `registerAction(pddl_name, bt_node_type, reactive=false)` — simple node mapping
- `registerActionSubTree(pddl_name, subtree_xml_template, reactive=false)` — XML template with `{param0}`, `{param1}` placeholders
- `registerActionSubTreeFile(pddl_name, xml_path, reactive=false)` — load from file
- `resolve(action_name, params) → ActionImpl` — returns resolved BT fragment (node type or instantiated XML)
- `ActionImpl` struct carries: resolved XML string, whether to use `ReactiveSequence` or `Sequence`, the parameter bindings

**Tests:** `tests/test_action_registry.cpp`
- Register simple action, resolve, verify node type
- Register template, resolve with params, verify substitution
- Verify reactive flag propagation

**Acceptance:** All unit tests pass.

## Step 6: BT Node Types

**Goal:** Implement the world-model-aware BT nodes.

**Files:** `include/mujin/bt_nodes/check_world_predicate.h`, `include/mujin/bt_nodes/set_world_predicate.h`, corresponding `.cpp` files

- `CheckWorldPredicate`: ConditionNode, reads `predicate` port (string key), queries `WorldModel::getFact()`, returns SUCCESS/FAILURE
- `SetWorldPredicate`: SyncActionNode, reads `predicate` + `value` ports, calls `WorldModel::setFact()`, returns SUCCESS
- Both receive `WorldModel*` via a shared pointer stored in the root blackboard

**Tests:** Within `tests/test_integration.cpp`
- Create a tree with CheckWorldPredicate + SetWorldPredicate
- Verify reading and writing world state through BT ticks

**Acceptance:** Nodes correctly read/write world model state during tree execution.

## Step 7: Plan-to-BT Compiler

**Goal:** Convert a LAPKT plan into executable BT XML.

**Files:** `include/mujin/plan_compiler.h`, `src/plan_compiler.cpp`

Sub-components:

1. **CausalGraph** — adjacency list of plan step dependencies. For each pair (i, j) where i < j: if any add-effect of step i is a precondition of step j, add edge i→j. Also track delete-effect conflicts for mutex detection.

2. **Flow Extraction** — topological sort of causal graph. Group into flows: steps with no cross-flow dependencies form separate flows. Steps that depend on multiple flows are join points.

3. **Action Unit Generation** — for each plan step, emit:
   ```xml
   <Sequence|ReactiveSequence name="action_name_params">
       <CheckWorldPredicate predicate="pre1" expected="true"/>
       ...
       <resolved BT fragment from ActionRegistry>
       <SetWorldPredicate predicate="add1" value="true"/>
       <SetWorldPredicate predicate="del1" value="false"/>
   </Sequence|ReactiveSequence>
   ```
   Sequence vs. ReactiveSequence comes from the ActionRegistry's per-action reactive flag.

4. **Tree Composition** — wrap flows in Parallel/Sequence structure:
   - Single flow → top-level Sequence
   - Multiple flows → Parallel node with success_count = flow_count
   - Shared actions (join points) → blackboard-flag guard pattern

5. **Output** — XML string compatible with `BT::BehaviorTreeFactory::createTreeFromText()`

**Sequential fallback mode:** skip causal graph, emit all steps as a single Sequence. For debugging.

**Tests:** `tests/test_plan_compiler.cpp`
- Compile a simple 2-action sequential plan → verify Sequence structure
- Compile a plan with independent actions → verify Parallel structure
- Compile UAV example → verify expected flow decomposition

**Acceptance:** Compiled BT XML is valid and loadable by BT.CPP factory.

## Step 8: End-to-End Vertical Slice

**Goal:** UAV search-and-classify example running end-to-end.

**File:** `src/main.cpp` (rewrite)

Pipeline:
1. Parse `domains/uav_search/domain.pddl` + `problem.pddl` → WorldModel
2. Register BT action mappings (stub implementations that log and succeed)
3. Project to STRIPS_Problem, solve with LAPKT (BFS(f) or SIW)
4. Compile plan to BT
5. Sync world model → blackboard
6. Tick tree to completion
7. Verify goal state reached in world model

Also wire up a MissionExecutor-style loop with basic replan-on-failure (inject a failure to demonstrate replanning).

**Tests:** `tests/test_integration.cpp`
- Full pipeline test with assertions at each stage
- Test replanning: modify world state mid-execution, verify new plan generated

**Acceptance:** UAV example completes, goal state verified, replan path exercised. `ctest` passes all unit and integration tests.

---

## Slice-to-Production Gap Analysis

Steps 1–8 deliver a working vertical slice. This section itemises what each component provides in the slice vs. what a production deployment requires. Each gap references the extension that closes it (see `extensions.md`).

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

### Observability

| Aspect | Slice delivers | Production needs | Extension |
|--------|---------------|------------------|-----------|
| BT logging | `std::cout` prints | `SqliteLogger` + `MinitraceLogger` + custom `MujinBTLogger` (structured JSON events) | ext 1 Layers 1–2 |
| WM logging | `version()` counter | Full audit log: every fact change with timestamp, source, old/new value | ext 1 Layer 3 |
| Live monitoring | None | Web dashboard (D3.js tree view + WM state table) or Foxglove via ROS2 topics | ext 1 Layer 4 |
| Plan audit | None | Per-episode log: init state, goal, solver config, plan, causal graph, compiled BT XML | ext 1 Layer 5 |
| Performance profiling | None | Chrome Tracing / Perfetto via `MinitraceLogger`. Per-node tick counts and durations via `TreeObserver` | ext 1 Layer 1 |

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
| BehaviorTree.CPP 4.6.2 | FetchContent | BT execution | Integrated |
| LAPKT Devel2.0 (core) | FetchContent | STRIPS model, search | Integrated |
| LAPKT FF-parser | FetchContent (libff_parser) | PDDL parsing | To add |
| Google Test | FetchContent | Unit/integration testing | To add |
