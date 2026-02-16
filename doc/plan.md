# Implementation Plan

This plan implements the architecture described in `concept.md`, guided by the decisions recorded in `design-questions.md`. The goal is a **minimal vertical slice** — the UAV search-and-classify example running end-to-end — then iterative hardening.

## Decided Design Parameters

| # | Question | Decision |
|---|----------|----------|
| 1 | Fact storage | **Eager grounding (A)** — storage-efficient (bitset, not `vector<bool>`) |
| 2 | Domain specification | **Hybrid (C)** — PDDL file for planning model, separate BT binding config. LAPKT has an FF-based C parser (`ff_to_aptk`) we can enable |
| 3 | Blackboard sync | **WM authoritative (A)** — blackboard is read-only view; mutations go through `SetWorldPredicate` |
| 4 | Action unit | **Configurable per-action (C)** — ActionRegistry specifies reactive vs. non-reactive per action |
| 5 | Causal graph | **Effect-precondition matching (A)** — with sequential fallback for debugging |
| 6 | Singleton nodes | **Blackboard flags (B)** — duplicate nodes guarded by done-flags |
| 7 | Error handling | **Perception-corrected (C)** — BT triggers actions but ground truth comes from external systems; world model is updated externally |
| 8 | Code organisation | **Component directories (B)** — `include/mujin/` + `src/`, may extract to library later |
| 9 | Testing | **Both (C)** — Google Test for unit tests + integration tests |
| 10 | Phase 1 scope | **Vertical slice (A)** — UAV example end-to-end |

---

## ROS2 Node Decomposition

The system is intended to run inside ROS2. The core architecture should be **ROS-agnostic** (pure C++ library), with thin ROS2 adapter layers. This enables:

- Testing and development without ROS2
- Swapping the execution backend (BT or otherwise) without touching the planning/world model core
- Running everything in-process (single node) for simple deployments, or distributed across nodes for production

### Candidate Node Boundaries

```
┌─────────────────────────────────────────────────────────────────┐
│                    Single-Node (Development)                     │
│                                                                  │
│  WorldModel + Planner + Compiler + Executor — all in-process    │
└─────────────────────────────────────────────────────────────────┘

         ▼  can be split into  ▼

┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│  WorldModel  │  │   Planner    │  │  Executor    │
│    Node      │  │    Node      │  │    Node      │
│              │  │              │  │              │
│ - Owns state │  │ - LAPKT      │  │ - BT.CPP     │
│ - ROS2 svc:  │  │ - ROS2 svc:  │  │ - Ticks tree │
│   get/set    │  │   plan(goal) │  │ - Calls WM   │
│   facts      │  │   → returns  │  │   svc for    │
│ - Publishes  │  │   plan       │  │   precond/   │
│   state on   │  │              │  │   effects    │
│   change     │  │              │  │              │
└──────┬───────┘  └──────────────┘  └──────┬───────┘
       │                                    │
       │  ◀──── Perception nodes ────▶      │
       │       update facts via svc         │
       │                                    │
       └────── shared lifecycle mgr ────────┘
```

### Design Principles for the Split

1. **WorldModel is the natural service boundary.** It exposes `get_fact`, `set_fact`, `query_state` as ROS2 services and publishes a `/world_state` topic on change. All other nodes are clients.

2. **Planner Node is stateless.** It receives a `STRIPS_Problem` snapshot + goal, returns a plan. Could be an action server for long-running solves. Easy to swap for a different planner.

3. **Executor Node owns the BT runtime.** It receives a compiled plan (or XML), ticks the tree, calls WorldModel services for precondition checks and effect application. This is the component most likely to be replaced (e.g., with a different execution framework).

4. **Perception Nodes are independent writers.** They call `set_fact` on the WorldModel service. The executor doesn't need to know about them — the world model just updates.

### What This Means for the Code

- All core classes (`WorldModel`, `ActionRegistry`, `PlanToBTCompiler`, BT nodes) are plain C++ with no ROS2 dependency
- ROS2 adapter code lives in a separate directory (e.g., `src/ros2/`) added later
- The `WorldModel` class exposes a C++ API; the ROS2 node wraps it with services/topics
- For the vertical slice, everything runs in-process (single `main()`)

---

## Source Tree Layout

```
include/mujin/
    world_model.h
    type_system.h
    action_registry.h
    plan_compiler.h
    pddl_parser.h              # wraps LAPKT FF-parser
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
    main.cpp                    # vertical slice demo
tests/
    CMakeLists.txt
    test_world_model.cpp
    test_action_registry.cpp
    test_plan_compiler.cpp
    test_pddl_parser.cpp
    test_integration.cpp        # UAV example end-to-end
domains/
    uav_search/
        domain.pddl
        problem.pddl
```

---

## Implementation Steps

### Step 1: Project Scaffolding

**Goal:** Build infrastructure — directories, CMake updates, Google Test integration.

- Create `include/mujin/`, `src/`, `tests/`, `domains/` directory structure
- Add Google Test via FetchContent in CMakeLists.txt
- Add a `tests/CMakeLists.txt` with a test target
- Update `src/CMakeLists.txt` to glob headers from `include/mujin/`
- Verify build still works with existing `main.cpp`

**Acceptance:** `cmake --build` succeeds, `ctest` runs an empty test suite.

### Step 2: TypeSystem and WorldModel Core

**Goal:** Implement the authoritative world model with eager grounding.

**Files:** `include/mujin/type_system.h`, `include/mujin/world_model.h`, `src/type_system.cpp`, `src/world_model.cpp`

Key implementation details:
- `TypeSystem`: type hierarchy (name → parent), object registry (name → type)
- `WorldModel`:
  - `registerPredicate(name, param_types)` — stores predicate schemas
  - `addObject(name, type)` — registers object; triggers eager grounding of all predicates that can bind to this type
  - Grounded facts stored in a **dynamic bitset** (compact; LAPKT's own `Bit_Set` or `std::vector<uint64_t>`) not `std::vector<bool>`
  - `BiMap<unsigned, std::string>` for fluent index ↔ string key
  - `setFact(string_key, bool)` and `getFact(string_key) → bool` for string-based access (used by BT nodes)
  - `setFact(fluent_id, bool)` and `getFact(fluent_id) → bool` for index-based access (used by LAPKT projection)
  - `version()` — monotonic counter, incremented on any state change

**Tests:** `tests/test_world_model.cpp`
- Register types, objects, predicates
- Verify eager grounding produces expected fluent count
- Set/get facts by string key and by index
- Verify version increments

**Acceptance:** All unit tests pass.

### Step 3: PDDL Projection (WorldModel → LAPKT)

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

### Step 4: PDDL Parser Integration (FF-Parser)

**Goal:** Load domain and problem from `.pddl` files instead of programmatic construction.

**Approach:** Enable LAPKT's FF-parser (`ff_to_aptk`).

- Add `libff_parser` as a FetchContent dependency (from `https://github.com/LAPKT-dev/libff_parser`)
- Add `${LAPKT_SRC}/translate/pddl/ff/ff_to_aptk.cxx` to the `lapkt_core` build
- Create `include/mujin/pddl_parser.h` and `src/pddl_parser.cpp`:
  - Wraps `aptk::FF_Parser::get_problem_description()`
  - Populates a `WorldModel` from the parsed `STRIPS_Problem` (reverse projection: extract types, objects, predicates, facts from the grounded problem)
- Write UAV example domain files: `domains/uav_search/domain.pddl`, `domains/uav_search/problem.pddl`

**Fallback:** If `libff_parser` integration proves problematic (platform issues, complex C build), implement a minimal PDDL tokenizer for STRIPS-level domains. This is a bounded task — STRIPS PDDL is a simple S-expression format.

**Tests:** `tests/test_pddl_parser.cpp`
- Parse UAV domain + problem
- Verify WorldModel contains expected types, objects, predicates, initial facts

**Acceptance:** `WorldModel` populated identically whether constructed programmatically or parsed from `.pddl` files.

### Step 5: ActionRegistry

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

### Step 6: BT Node Types

**Goal:** Implement the world-model-aware BT nodes.

**Files:** `include/mujin/bt_nodes/check_world_predicate.h`, `include/mujin/bt_nodes/set_world_predicate.h`, corresponding `.cpp` files

- `CheckWorldPredicate`: ConditionNode, reads `predicate` port (string key), queries `WorldModel::getFact()`, returns SUCCESS/FAILURE
- `SetWorldPredicate`: SyncActionNode, reads `predicate` + `value` ports, calls `WorldModel::setFact()`, returns SUCCESS
- Both receive `WorldModel*` via a shared pointer stored in the root blackboard (BT.CPP v4 pattern: store in blackboard as `WorldModel*`)

**Tests:** Within `tests/test_integration.cpp`
- Create a tree with CheckWorldPredicate + SetWorldPredicate
- Verify reading and writing world state through BT ticks

**Acceptance:** Nodes correctly read/write world model state during tree execution.

### Step 7: Plan-to-BT Compiler

**Goal:** Convert a LAPKT plan into executable BT XML.

**Files:** `include/mujin/plan_compiler.h`, `src/plan_compiler.cpp`

Sub-components:

1. **CausalGraph** — adjacency list of plan step dependencies
   - For each pair (i, j) where i < j: if any add-effect of step i is a precondition of step j, add edge i→j
   - Also track delete-effect conflicts for mutex detection

2. **Flow Extraction** — identify independent causal chains
   - Topological sort of causal graph
   - Group into flows: steps with no cross-flow dependencies form separate flows
   - Steps that depend on multiple flows are join points

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
   The choice of Sequence vs. ReactiveSequence comes from the ActionRegistry's per-action reactive flag.

4. **Tree Composition** — wrap flows in Parallel/Sequence structure:
   - Single flow → top-level Sequence
   - Multiple flows → Parallel node with success_count = flow_count
   - Shared actions (join points) → blackboard-flag guard pattern

5. **Output** — XML string compatible with `BT::BehaviorTreeFactory::createTreeFromText()`

**Sequential fallback mode:** skip causal graph, emit all steps as a single Sequence. Useful for debugging.

**Tests:** `tests/test_plan_compiler.cpp`
- Compile a simple 2-action sequential plan → verify Sequence structure
- Compile a plan with independent actions → verify Parallel structure
- Compile UAV example → verify expected flow decomposition

**Acceptance:** Compiled BT XML is valid and loadable by BT.CPP factory.

### Step 8: End-to-End Vertical Slice

**Goal:** UAV search-and-classify example running end-to-end.

**File:** `src/main.cpp` (rewrite)

Pipeline:
1. Parse `domains/uav_search/domain.pddl` + `problem.pddl` → WorldModel
2. Register BT action mappings (stub implementations that just log and succeed)
3. Project to STRIPS_Problem, solve with LAPKT (BFS(f) or SIW)
4. Compile plan to BT
5. Sync world model → blackboard
6. Tick tree to completion
7. Verify goal state reached in world model

Also wire up a `MissionExecutor`-style loop with basic replan-on-failure (inject a failure to demonstrate replanning).

**Tests:** `tests/test_integration.cpp`
- Full pipeline test with assertions at each stage
- Test replanning: modify world state mid-execution, verify new plan generated

**Acceptance:** UAV example completes, goal state verified, replan path exercised. `ctest` passes all unit and integration tests.

---

## Dependency Summary

| Dependency | Source | Purpose | Status |
|------------|--------|---------|--------|
| BehaviorTree.CPP 4.6.2 | FetchContent | BT execution | Already integrated |
| LAPKT Devel2.0 (core) | FetchContent | STRIPS model, search | Already integrated |
| LAPKT FF-parser | FetchContent (libff_parser) | PDDL parsing | **To add** |
| Google Test | FetchContent | Unit/integration testing | **To add** |

---

## What Is Deferred

These are explicitly out of scope for the vertical slice but tracked for subsequent work:

- **ROS2 node wrappers** — `src/ros2/` with WorldModel service node, Planner action server, Executor node. Deferred until core works standalone.
- **Perception integration** — external fact updates. The `WorldModel::setFact()` API supports this; the external caller (perception node) is out of scope.
- **PYRAMID service nodes** — `InvokeService` BT node. Requires PYRAMID SDK integration.
- **Thread safety** — versioned snapshots for concurrent access. Needed for ROS2 multi-node but not for the single-threaded vertical slice.
- **Hierarchical planning** — `ExecutePhaseAction` and sub-planners. Deferred to post-slice.
- **Temporal planning** — PDDL 2.1 durative actions, STN conversion. Deferred.
- **Groot visualization** — currently disabled. Can re-enable for debugging later.
