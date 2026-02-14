# Design Questions & Implementation Options

This document captures open design decisions and clarification questions arising from `concept.md`. Each section presents the question, the options, and their trade-offs. Decisions here will shape the Phase 1-3 implementation.

---

## 1. WorldModel Internal Representation

**Question:** How should grounded facts be stored internally?

The concept describes a `std::vector<bool> facts_` indexed by fluent ID, with a `BiMap<unsigned, std::string> fluent_map_`. This works for a fixed or slowly-growing set of ground predicates but has implications for how predicates are grounded.

### Option A: Eager Full Grounding

Pre-compute all possible groundings of each predicate schema against the registered objects. For `at(robot, zone)` with 3 robots and 5 zones, this creates 15 fluent entries immediately.

- Pro: Simple indexing, matches LAPKT's expected input directly
- Pro: `projectToSTRIPS()` is trivial — the fluent vector maps 1:1
- Con: Combinatorial explosion with many object types or high-arity predicates
- Con: Must re-ground when objects are added

### Option B: Lazy Grounding (on first `setFact`/`getFact`)

Only create fluent entries when a specific grounding is first referenced. Unseen groundings are implicitly false.

- Pro: Avoids combinatorial explosion
- Pro: Natural for sparse domains (most predicates are false)
- Con: Fluent indices are assigned non-deterministically (depends on call order)
- Con: LAPKT projection must handle gaps — the planner needs all *relevant* fluents even if currently false

### Option C: Domain-Driven Grounding

Parse the PDDL domain/problem to determine which groundings are reachable (delete-relaxation reachability analysis), then only instantiate those. This is what most planners do internally.

- Pro: Correct and complete without waste
- Con: Requires a PDDL parser or manual specification of reachable groundings
- Con: More complex to implement

**My inclination:** Option A for Phase 1 (simple, correct, matches the LAPKT model), with a note that we may need to move to Option C for larger domains.

**User reponse:** Option A with care about storage efficiency. 

---

## 2. PDDL Domain Specification

**Question:** How does the system learn about the PDDL domain (action schemas, predicates, types)?

The concept shows the WorldModel and ActionRegistry being populated programmatically in C++. But PDDL domains are normally specified in `.pddl` files.

### Option A: Programmatic-Only (C++ API)

Define everything in code: `wm.registerPredicate("at", {"vehicle", "location"})`, `registry.registerAction("move", "MoveNode")`, etc.

- Pro: No parser needed
- Pro: Full compile-time checking of types
- Con: Domain changes require recompilation
- Con: Can't reuse standard PDDL benchmark domains

### Option B: PDDL File Parser

Parse `.pddl` domain and problem files at startup. Derive predicates, types, actions, and initial state from the files.

- Pro: Standard format, reusable domains
- Pro: Domain changes don't require recompilation
- Con: Need a PDDL parser (write one, or use an existing library)
- Con: Still need C++ code or configuration to map PDDL actions to BT implementations

### Option C: Hybrid — PDDL Domain File + Programmatic BT Binding

Parse the `.pddl` file for the planning model (predicates, types, action schemas with preconditions/effects). Use a separate configuration (C++ API or config file) to map each PDDL action name to its BT implementation.

- Pro: Clean separation of planning model from execution model
- Pro: Can use standard PDDL domains for testing
- Con: Two sources of truth for action definitions (PDDL schema + BT mapping)

**Needs your input:** Is there an existing PDDL parser you'd prefer, or should we write a minimal one? Alternatively, is programmatic-only acceptable for the initial phases?

---

## 3. WorldModel ↔ Blackboard Sync Direction

**Question:** Should the blackboard be a read-only view of the world model, or should BT nodes write back through the blackboard?

The concept describes two paths:
- `syncToBlackboard()`: world model → blackboard (one-way push)
- `applyEffects()`: explicit effects → world model (bypasses blackboard)

But BT.CPP's natural pattern is for nodes to read/write the blackboard directly.

### Option A: World Model is Authoritative, Blackboard is Read-Only View

BT nodes read predicates from the blackboard but never write predicate values there. All state changes go through `SetWorldPredicate` nodes that call `world_model_->setFact()` directly, which then pushes to the blackboard.

- Pro: Single source of truth, no sync bugs
- Pro: World model version counter captures all changes
- Con: Every BT state mutation needs a dedicated node type
- Con: Can't use BT.CPP's built-in `SetBlackboard` node for predicate changes

### Option B: Bidirectional Sync

BT nodes can write predicate values to the blackboard. The world model periodically pulls changes from the blackboard (e.g., at tick boundaries).

- Pro: More natural BT.CPP usage patterns
- Pro: Simpler action node implementations
- Con: Two sources of truth — risk of desync
- Con: Change tracking is harder (who changed what?)

### Option C: Blackboard *is* the World Model

Instead of a separate WorldModel class, use BT.CPP's Blackboard as the single store. Provide a projection function that reads blackboard state into a LAPKT `STRIPS_Problem` when planning is needed.

- Pro: Eliminates sync entirely
- Con: Blackboard is string-keyed, untyped — fragile for the PDDL projection
- Con: No structured type system or object registry
- Con: Hard to version or snapshot

**My inclination:** Option A — it's the cleanest for auditability and matches the concept's design. The overhead of `SetWorldPredicate` nodes is minimal since the compiler generates them automatically.

---

## 4. Action Unit: ReactiveSequence vs. Sequence

**Question:** Should precondition checks in the action unit use `ReactiveSequence` (re-checked every tick) or plain `Sequence` (checked once)?

The concept uses `ReactiveSequence`, meaning preconditions are re-evaluated on every tick while the action executes.

### Option A: ReactiveSequence (as in concept)

Preconditions are continuously monitored. If `path_clear(A,B)` becomes false while `MoveAction` is running, the action is halted.

- Pro: Truly reactive — responds to world changes during execution
- Con: The action must handle being halted mid-execution (cleanup, partial effects)
- Con: Preconditions checked against the *planning* model may not reflect real-time sensor data anyway

### Option B: Plain Sequence (check-once)

Preconditions are verified before the action starts. Once the action begins, it runs to completion (or failure).

- Pro: Simpler action lifecycle — no mid-execution interrupts
- Pro: Matches PDDL semantics more closely (preconditions checked at action start)
- Con: Can't react to precondition violations during execution

### Option C: Configurable Per-Action

Allow the ActionRegistry to specify whether each action uses reactive or non-reactive precondition checking.

- Pro: Flexibility — reactive for long-running actions, non-reactive for atomic ones
- Con: More configuration surface

**Needs your input:** How long-running are your typical actions? If actions are fast relative to the world changing, Option B may be sufficient and simpler. If actions like `search_sector` run for extended periods, Option A or C makes more sense.

---

## 5. Causal Graph Construction

**Question:** How should the plan-to-BT compiler determine causal dependencies between plan steps?

### Option A: Effect-Precondition Matching (as in concept / PlanSys2)

For each pair of plan steps (i, j) where i < j, check if any add-effect of step i matches a precondition of step j. If so, j causally depends on i.

- Pro: Well-understood algorithm (PlanSys2 validates it)
- Pro: Enables maximum parallelism
- Con: Requires knowing the grounded preconditions and effects of each action (available from the PDDL model)

### Option B: Conservative Sequential

Don't build a causal graph. Execute all plan steps in sequence.

- Pro: Trivially correct
- Con: No parallelism — wastes multi-agent capability
- Con: Doesn't use the information available

### Option C: Planner-Provided Partial Order

Some planners (temporal planners, partial-order planners) output ordering constraints directly. Use those instead of reconstructing them.

- Pro: Uses planner's own analysis
- Con: LAPKT's standard solvers output totally-ordered plans
- Con: Would require a different planner or post-processing

**My inclination:** Option A for the compiler, with Option B available as a fallback/debug mode. Option C is worth noting for future temporal planning support.

---

## 6. Singleton Nodes for Shared Actions

**Question:** When the same grounded action appears in multiple execution flows (e.g., `assemble` depends on parts from two flows), how should this be handled?

PlanSys2 uses a "Singleton" pattern where the same BT node instance is referenced from multiple places.

### Option A: Singleton Pattern (PlanSys2 approach)

The shared action is instantiated once and referenced by multiple flows. Whichever flow reaches it first executes it; the other flow gets a SUCCESS result immediately.

- Pro: Prevents duplicate execution
- Con: BT.CPP doesn't natively support shared node instances across tree branches — would need custom implementation

### Option B: Synchronisation via Blackboard Flags

The shared action is duplicated across flows, but guarded by a blackboard flag. The first execution sets a "done" flag; the second checks the flag and skips.

- Pro: Uses standard BT.CPP mechanisms
- Pro: Simpler to implement
- Con: Duplicate nodes in the tree (cosmetic issue)

### Option C: Barrier/Join Node

Insert a custom synchronisation node that waits for all prerequisite flows to complete before allowing the shared action to proceed. Only one instance of the shared action exists, placed after the barrier.

- Pro: Clean semantics — matches the causal graph directly
- Con: Requires a custom BT node type
- Con: More complex tree structure

**Needs your input:** How common are shared actions in your expected plans? If rare, Option B is simplest. If common (e.g., multi-agent assembly tasks), Option C may be worth the complexity.

---

## 7. Error Handling and Partial Effects

**Question:** When an action fails mid-execution, what happens to the world model?

### Option A: All-or-Nothing

Effects are only applied after the action succeeds. If `MoveAction` fails, the world model still shows the robot at the original position.

- Pro: Simple, predictable
- Con: If the robot *actually* moved partway, the world model is wrong
- Con: Replanning from stale state may produce invalid plans

### Option B: Partial Effect Application

Actions can apply partial effects as they execute (e.g., `at(robot,A)` → false as soon as movement begins). On failure, the action applies whatever effects reflect the actual state.

- Pro: World model stays closer to reality
- Con: Actions must explicitly manage partial effect reporting
- Con: More complex action node interface

### Option C: Perception-Corrected

On failure, ignore PDDL effects entirely. Instead, rely on a perception system to update the world model with observed ground truth before replanning.

- Pro: Most robust — doesn't trust the plan's model of what happened
- Con: Requires a perception system that can update the world model
- Con: Latency between failure and perception update

**Needs your input:** Do you have a perception pipeline that can independently verify world state? If so, Option C is most robust. Otherwise, Option A is safest to start with.

---

## 8. Build and Code Organisation

**Question:** How should the new classes be organised in the source tree?

The current structure is flat (`src/main.cpp`). As we add WorldModel, ActionRegistry, PlanToBTCompiler, BT node types, etc., we need structure.

### Option A: Flat with Namespaces

```
src/
  main.cpp
  world_model.h / .cpp
  action_registry.h / .cpp
  plan_compiler.h / .cpp
  bt_nodes.h / .cpp
```

- Pro: Simple, low overhead
- Con: Doesn't scale well past ~10 files

### Option B: Component Directories

```
include/mujin/
  world_model.h
  action_registry.h
  plan_compiler.h
  bt_nodes/
    check_world_predicate.h
    set_world_predicate.h
    ...
src/
  world_model.cpp
  action_registry.cpp
  plan_compiler.cpp
  bt_nodes/
    ...
  main.cpp
```

- Pro: Clean separation of headers and implementation
- Pro: Scales well, conventional C++ layout

### Option C: Library + Application Split

```
lib/mujin-core/
  include/mujin/...
  src/...
  CMakeLists.txt          (builds libmujin_core)
src/
  main.cpp                (links against libmujin_core)
  CMakeLists.txt
```

- Pro: Core logic is a reusable library
- Pro: Enables separate test executable linking against the library
- Con: More CMake boilerplate

**Needs your input:** How do you see this project being consumed — standalone executable, library integrated into a larger system, or both?

---

## 9. Testing Strategy

**Question:** What testing approach should we adopt?

### Option A: Integration Tests Only (expand main.cpp)

Add more scenarios to `main.cpp` that exercise the full pipeline: build world model → plan → compile BT → execute → verify.

- Pro: Tests the real integration, minimal infrastructure
- Con: Hard to isolate failures
- Con: Slow iteration

### Option B: Unit Tests with a Framework (Google Test / Catch2)

Add a test framework and write focused tests for each component: WorldModel state management, causal graph construction, BT compilation, etc.

- Pro: Fast feedback, isolates bugs
- Pro: Standard practice
- Con: Another dependency to manage

### Option C: Both

Unit tests for components, plus integration tests for the full pipeline.

- Pro: Best coverage
- Con: More code to maintain

**Needs your input:** Do you have a preference for test framework? Google Test and Catch2 both work well with CMake FetchContent.

---

## 10. Scope of Initial Implementation

**Question:** What should Phase 1 actually deliver as a working end-to-end?

The concept's Phase 1 is "World Model + LAPKT Integration" but it's useful to define what "done" looks like concretely.

### Option A: Minimal Vertical Slice

Implement just enough of WorldModel + PlanToBTCompiler to run the UAV example from Section 5 end-to-end: register domain, set initial state, plan with LAPKT, compile to BT, execute, verify goal state.

- Pro: Proves the full architecture works early
- Con: Requires touching all components (even if partially)

### Option B: Bottom-Up by Component

Implement WorldModel fully (with tests), then ActionRegistry (with tests), then PlanToBTCompiler, etc. Each component is complete before moving on.

- Pro: Each piece is solid before building on it
- Con: No end-to-end validation until later

### Option C: The Concept's Phased Approach

Follow the 5-phase roadmap in concept.md exactly.

- Pro: Already documented and thought through
- Con: Phase 1 doesn't produce an end-to-end demonstration

**My inclination:** Option A — a minimal vertical slice gives the earliest signal that the architecture holds together. Components can be hardened in subsequent iterations.

---

## Summary of Decisions Needed

| # | Question | Recommended | Needs Input |
|---|----------|-------------|-------------|
| 1 | Fact storage | Eager grounding (A) | Confirm domain size |
| 2 | Domain specification | Hybrid (C) or programmatic (A) | Parser preference |
| 3 | Blackboard sync | WM authoritative (A) | Confirm |
| 4 | ReactiveSequence | Configurable (C) | Action duration info |
| 5 | Causal graph | Effect-precondition matching (A) | Confirm |
| 6 | Singleton nodes | Blackboard flags (B) | Shared action frequency |
| 7 | Error handling | All-or-nothing (A) initially | Perception pipeline? |
| 8 | Code organisation | Component dirs (B) or library (C) | Consumption model |
| 9 | Testing | Both (C) | Framework preference |
| 10 | Phase 1 scope | Vertical slice (A) | Confirm |
