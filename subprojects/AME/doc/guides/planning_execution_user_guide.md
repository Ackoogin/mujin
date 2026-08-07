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

### Step B: Register action implementations

The planner is intentionally execution-agnostic. Use `ActionRegistry` to map
each PDDL action name to either one BT node type or a BT subtree snippet.

At compile time, placeholders such as `{param0}`, `{param1}` are replaced with grounded values.

### Step C: Compile plan into a BT

`PlanCompiler` emits one action element for each plan step. The element is
named after the grounded action and carries four contract attributes:

- `ame_preconditions`,
- `ame_add_effects`,
- `ame_del_effects`,
- `ame_reactive`.

A simple registered implementation receives these ports directly. A registered
subtree is wrapped in the `PlannedAction` decorator. If an action is not
registered, the compiler emits a `SimulatedAction` with the same contract.
The goal guard contains one `GoalReached` condition for the whole mission.

Planned actions accept a fifth port, `ame_required_authority`, which the
compiler does not emit. It defaults to `any`, meaning a precondition counts
whether it was predicted by a plan effect or observed. Whoever loads the tree
can set it to `confirmed` to make the action start only on observed facts;
DevEnv's "Preconditions" control does exactly that before it loads a tree.

### Step D: Execute and monitor

Each planned action checks its own preconditions and applies effects only after
its work succeeds. If an execution node fails because of a timeout, service
error, or another runtime problem, mission execution can trigger replanning
and continue from updated state.

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

- PDDL action `move` -> service `mobility`, operation `move`
- Parameter bindings come from grounded action args.

You can combine:

- `param_names` + `param_values` for argument mapping,
- `request_json` for fixed request fields.

If keys overlap, mapped parameters override conflicting request defaults.

### 4.3 Register required BT node types

Your BT factory must register:

- `GoalReached`
- `PlannedAction`
- `SimulatedAction`
- `InvokeService`

Register `CheckWorldPredicate` as well when hand-written trees use that
condition node.

### 4.4 Put dependencies on the blackboard

Before ticking a compiled tree, set:

- `pyramid_service` (`IPyramidService*`)
- `world_state` (`IWorldStateAccess*`)

Existing in-process setups can continue to provide `world_model`
(`WorldModel*`). Planned actions use it as a fallback when `world_state` is not
present.

If `pyramid_service` is missing/null, `InvokeService` fails immediately.

### 4.5 Timeouts, cancellation, and replanning behavior

- Default `timeout_ms` is 5000.
- `timeout_ms="0"` means no timeout.
- On timeout or external halt, `InvokeService` calls `cancelCall(...)`.
- `FAILURE` from `InvokeService` is handled by standard replan-on-failure behavior.

### 4.6 Simulation/testing path

Use `SimulatedAction` for a deterministic stand-in when no action
implementation is registered. It defaults to one tick and success, and it
records declared effects as believed facts. Use its `ticks` and `success`
ports to configure duration and outcome. Use `MockPyramidService` when the
test needs to exercise an `InvokeService` mapping.

For richer failure/latency tests, provide a custom test adapter that delays completion or forces failures.

---

## 5) Behavior Tree nodes you will use most

### Built-in execution nodes (common)

- **`PlannedActionNode`**: base class that checks a simple action's contract and commits effects after success.
- **`PlannedAction`**: decorator that gives the same contract to a multi-node action subtree.
- **`SimulatedAction`**: configurable stand-in for an action without a registered implementation.
- **`GoalReached`**: checks all mission goal facts in one condition node.
- **`CheckWorldPredicate`**: condition for hand-written trees that need one explicit fact check.
- **`InvokeService`**: async backend/PYRAMID call.
- **`ExecutePhaseAction`**: hierarchical node that performs sub-plan -> compile -> execute for phase goals.
- **`DelegateToAgent`**: multi-agent delegation node.

### Reactive planned actions

When an action mapping is marked *reactive*, the compiler sets
`ame_reactive="true"` on that action. Its planned-action base or decorator
then checks the preconditions again on every tick while the work is running.

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

An action implementation used directly by `ActionRegistry::registerAction()`
derives from `PlannedActionNode`. Its `providedPorts()` calls
`withBasePorts(...)`, and its action work is implemented through
`onActionStart()`, `onActionRunning()`, and `onActionHalted()`.

The base `commitEffects()` records declared add and delete effects as believed
facts after success. A deployed action overrides this method when it needs to
record only the state that its external system or sensors actually confirmed.
AME does not require confirmed state after an action.

Typical steps:

1. Implement the node as a `PlannedActionNode` when it is a planned action.
2. Register the node type in `BehaviorTreeFactory`.
3. Reference it from action subtrees or top-level trees.

### 6.2 Custom action subtrees

`ActionRegistry::registerActionSubTree(action_name, xml, reactive)` lets you define custom per-action execution fragments.

The compiler wraps the template in `PlannedAction`, so the template must have
one root node. That root can be a control node containing as many child nodes
as the implementation needs.

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

- **Action exists in PDDL but runs only as a simulation**
  The action has no `ActionRegistry` mapping. Register its deployed node type
  or subtree when it should command an external system.

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

- Quick run/test setup: `subprojects/AME/doc/guides/quickstart.md`
- Planning internals and action mapping: `subprojects/AME/doc/architecture/03-planning.md`
- Execution internals and node details: `subprojects/AME/doc/architecture/04-execution.md`
- Deployment details (ROS2 modes): `subprojects/AME/doc/architecture/06-ros2.md`
- Full architecture map: `subprojects/AME/doc/architecture/01-overview.md`
