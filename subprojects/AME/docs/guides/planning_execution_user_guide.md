# Planning and Execution User Guide

This guide explains how to use the AME planning/execution system in practical terms.

It is written for operators, integrators, and developers who need to run missions and connect real services, without needing to read all architecture internals first.

> This guide consolidates the previous split PYRAMID service integration notes into one end-to-end workflow.

---

## 1) What the system does (in one page)

AME turns mission intent into executable behavior:

1. You define mission rules and goals in PDDL (domain + problem).
2. The planner finds a valid action sequence.
3. The plan compiler converts those actions into a Behavior Tree (BT).
4. The executor runs that BT, calling real integrations (for example PYRAMID services).
5. If execution fails, the system can replan from the current world state.

Think of it as:

- **PDDL** = "What must be true, and what actions are allowed"
- **Planner** = "What sequence can reach the goal"
- **Behavior Tree** = "How to run that sequence robustly at runtime"
- **World model** = "Current truth the system believes"

---

## 2) Core concepts and PDDL terminology

If PDDL terminology is new, this section is the minimum you need.

### Predicate

A **predicate** is a true/false statement about the world.

Examples:

- `(at uav1 base)`
- `(searched sector_a)`
- `(connected node_a node_b)`

### Fluent

In this codebase, a **fluent** is the internal grounded instance of a predicate (an indexed fact in the world model/planner state).

Practical meaning:

- You write high-level predicates in PDDL.
- The system grounds them to concrete combinations like `(at uav1 base)`.
- Those grounded facts become fluents used by planning and execution.

### Action schema vs grounded action

- **Action schema**: template in domain PDDL, e.g. `(move ?robot ?from ?to)`.
- **Grounded action**: fully bound at planning time, e.g. `move(uav1, base, sector_a)`.

### Preconditions and effects

- **Preconditions**: facts that must be true before an action executes.
- **Effects**: facts that become true/false after an action.

Example for move:

- Precondition: `(at uav1 base)`
- Add effect: `(at uav1 sector_a)`
- Delete effect: `(at uav1 base)`

### Domain vs problem

- **Domain**: reusable model (types, predicates, actions).
- **Problem**: mission instance (objects, initial state, goals).

---

## 3) End-to-end mission flow

### Step A: Model the mission in PDDL

Create domain and problem files with:

- action schemas that reflect real capabilities,
- valid initial facts,
- mission goals.

### Step B: Register action execution templates

The planner is intentionally execution-agnostic. Use `ActionRegistry` to map each PDDL action name to a BT subtree snippet.

At compile time, placeholders such as `{param0}`, `{param1}` are replaced with grounded values.

### Step C: Compile plan into a BT

`PlanCompiler` wraps each action with:

- precondition checks (`CheckWorldPredicate`),
- execution node/subtree (your mapping),
- world-state effect updates (`SetWorldPredicate`).

### Step D: Execute and monitor

The BT executes action units. If an execution node fails (timeout/service error/etc.), mission execution can trigger replanning and continue from updated state.

---

## 4) Consolidated PYRAMID service integration

This section replaces the old standalone PYRAMID guide.

### 4.1 Implement an `IPyramidService` adapter

`InvokeService` is the runtime node that bridges BT execution to your backend integration.

Create an adapter implementing:

- `call(...)` for sync use (optional in many flows),
- `callAsync(...)` to start non-blocking requests,
- `pollResult(...)` to report `PENDING/SUCCESS/FAILURE/CANCELLED`,
- `cancelCall(...)` for timeout/halt cancellation.

`ServiceMessage` is key-value (`string -> string`), so adapters translate between AME fields and SDK-native messages.

### 4.2 Bind PDDL actions to `InvokeService`

Example mapping idea:

- PDDL action `move` → service `mobility`, operation `move`
- Parameter bindings come from grounded action args.

You can combine:

- `param_names` + `param_values` for argument mapping,
- `request_json` for fixed request fields.

If keys overlap, mapped parameters override conflicting request defaults.

### 4.3 Register required BT node types

Your BT factory must register:

- `CheckWorldPredicate`
- `SetWorldPredicate`
- `InvokeService`

### 4.4 Put dependencies on the blackboard

Before ticking a compiled tree, set:

- `pyramid_service` (`IPyramidService*`)
- `world_model`

If `pyramid_service` is missing/null, `InvokeService` fails immediately.

### 4.5 Timeouts, cancellation, and replanning behavior

- Default `timeout_ms` is 5000.
- `timeout_ms="0"` means no timeout.
- On timeout or external halt, `InvokeService` calls `cancelCall(...)`.
- `FAILURE` from `InvokeService` is handled by standard replan-on-failure behavior.

### 4.6 Simulation/testing path

Use `MockPyramidService` for deterministic tests.

For richer failure/latency tests, provide a custom test adapter that delays completion or forces failures.

---

## 5) Behavior Tree nodes you will use most

### Built-in execution nodes (common)

- **`CheckWorldPredicate`**: condition check against world state.
- **`SetWorldPredicate`**: write add/delete effects.
- **`InvokeService`**: async backend/PYRAMID call.
- **`ExecutePhaseAction`**: hierarchical node that performs sub-plan → compile → execute for phase goals.
- **`DelegateToAgent`**: multi-agent delegation node.

### `Sequence` vs `ReactiveSequence`

When action mapping is marked *reactive*, the compiler emits a `ReactiveSequence` so preconditions are re-checked while the action is still running.

Use reactive mode for long-running actions that may become invalid due to live world updates.

---

## 6) Custom BT nodes and custom subtrees

You can extend behavior without changing the planner.

### 6.1 Custom BT nodes

Create a custom BT node when you need runtime logic not covered by existing nodes:

- sensor gate checks,
- safety decorators,
- mission-specific control logic,
- specialized service/result interpretation.

Typical steps:

1. Implement node class with BT.CPP API.
2. Register the node type in `BehaviorTreeFactory`.
3. Reference it from action subtrees or top-level trees.

### 6.2 Custom action subtrees

`ActionRegistry::registerActionSubTree(action_name, xml, reactive)` lets you define custom per-action execution fragments.

This is the preferred mechanism to customize action runtime behavior while preserving PDDL planning compatibility.

Good subtree patterns:

- service call with retry decorator,
- additional guard checks before/after service call,
- domain-specific telemetry/event nodes.

### Parameter substitution rules

Within subtree XML:

- `{param0}`, `{param1}`, ... map to grounded action arguments by position.
- Ensure argument order matches your PDDL action signature.

---

## 7) Hierarchical missions with `ExecutePhaseAction`

Use `ExecutePhaseAction` when a single mission should be decomposed into phases (for example: transit -> search -> classify -> exfil).

What it does:

1. reads phase goals,
2. plans for those goals,
3. compiles a subtree,
4. executes that subtree,
5. returns status to parent tree.

Benefits:

- cleaner mission orchestration,
- easier audit of parent/child planning episodes,
- better reuse of shared phase patterns.

---

## 8) Node deployment options (how to run it)

AME supports several deployment patterns. Choose based on latency, scale, and architecture constraints.

### Option 1: In-process (single executor)

- World model, planner, and executor share process/executor context.
- Lowest latency and simplest setup.
- Good default for development and single-platform deployments.

### Option 2: Distributed (service-backed)

- Nodes communicate by ROS2 services/actions/topics.
- Supports process and machine separation.
- Better isolation and distributed deployment flexibility.

### Option 3: Multi-agent

- One shared world model + planner/dispatcher.
- Multiple agent executors (namespaced by `agent_id`).
- Use for coordinated missions across multiple platforms.

### Option 4: Multi-planner

- One world model with union domain.
- Multiple planners with specialized domain subsets.
- Useful when mission classes are distinct but share global state.

---

## 9) Recommended onboarding path

For new teams:

1. Run the demo and inspect generated audit logs.
2. Start with one small PDDL domain/problem.
3. Register one or two actions with simple `InvokeService` subtrees.
4. Validate happy-path execution with `MockPyramidService`.
5. Add real adapter and timeout/failure tests.
6. Enable reactive actions where world volatility demands it.
7. Introduce `ExecutePhaseAction` only after flat plans are stable.

---

## 10) Troubleshooting quick reference

- **Action exists in PDDL but does nothing useful**  
  Usually missing/misconfigured `ActionRegistry` subtree mapping.

- **`InvokeService` fails immediately**  
  Check `pyramid_service` blackboard pointer and adapter lifetime.

- **Unexpected precondition failures during execution**  
  Verify world-model updates from perception and confirm whether action should be reactive.

- **Plan found but mission still fails**  
  Inspect service adapter result mapping (`pollResult` statuses, response parsing, timeout behavior).

- **Multi-node ROS2 deployment behaves differently from in-process**  
  Check service timeouts, lifecycle activation order, and domain loading consistency.

---

## 11) Where to go next

- Quick run/test setup: `subprojects/AME/docs/guides/quickstart.md`
- Planning internals and action mapping: `subprojects/AME/docs/architecture/03-planning.md`
- Execution internals and node details: `subprojects/AME/docs/architecture/04-execution.md`
- Deployment details (ROS2 modes): `subprojects/AME/docs/architecture/06-ros2.md`
- Full architecture map: `subprojects/AME/docs/architecture/01-overview.md`

