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

## Dependencies

| Dependency | Source | Purpose | Status |
|------------|--------|---------|--------|
| BehaviorTree.CPP 4.6.2 | FetchContent | BT execution | Integrated |
| LAPKT Devel2.0 (core) | FetchContent | STRIPS model, search | Integrated |
| LAPKT FF-parser | FetchContent (libff_parser) | PDDL parsing | To add |
| Google Test | FetchContent | Unit/integration testing | To add |
