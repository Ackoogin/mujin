# Execution: PlanCompiler, BT Nodes, Replanning

## Plan-to-BT Compiler

`PlanCompiler` (`include/ame/plan_compiler.h`) converts a LAPKT plan into executable BT XML.

### Algorithm

1. **Causal graph** -- for each pair (i, j) where i < j: if any add-effect of step i is a precondition of step j, add edge i->j. Also track delete-effect conflicts for mutex detection.

2. **Execution phase extraction** -- repeatedly collect every step whose causal
   predecessors have already been scheduled. Each phase is sorted by original
   plan index. A phase with multiple steps can run in parallel; phase boundaries
   are conservative synchronization barriers.

3. **Planned action generation** -- each plan step becomes one action element.
   A simple registered node receives the grounded state contract as ports:
   ```xml
   <MoveAction name="move(uav1,base,sector_a)"
       param0="uav1" param1="base" param2="sector_a"
       ame_preconditions="(at uav1 base)"
       ame_add_effects="(at uav1 sector_a)"
       ame_del_effects="(at uav1 base)"
       ame_reactive="false"/>
   ```
   A registered XML subtree is the single child of a `PlannedAction`
   decorator carrying the same ports. The registry's reactive flag supplies the
   `ame_reactive` value.

   **An action with no registry entry aborts the compile.** `compile()` and
   `compileSequential()` throw `std::runtime_error("Unregistered action in plan:
   <name>")`. This is the fail-closed default: emitting a node that does nothing
   but write the action's predicted effects would report success for work that
   was never dispatched. `setStubUnregisteredActions(true)` turns the step into a
   `SimulatedAction` instead, which is for tools reasoning about a model rather
   than commanding anything — the authoring tool and the Python bindings set it,
   and nothing else does. An integrator who has not registered every action meets
   this as an exception at compile time rather than as a silent success at run
   time.

4. **Tree composition**:
   - No parallel-ready phase -> top-level `Sequence`
   - Parallel-ready phases -> top-level `Sequence` containing one child per phase
   - Single-step phase -> action unit emitted directly
   - Multi-step phase -> `Parallel` node with `success_count = phase_size`
   - A `ReactiveFallback` uses `GoalReached` conditions before the plan body, so
     a completed mission is not run again on a later tick. Disjunctive goals emit
     one `GoalReached` per alternative inside a `Fallback`.

5. **Output** -- XML string loadable by `BT::BehaviorTreeFactory::createTreeFromText()`

**Sequential fallback mode** (all steps in one `Sequence`) is available for debugging.

## BT Node Types

Planned actions obtain `IWorldStateAccess*` from the `world_state` blackboard
entry. For compatibility with existing in-process trees, they fall back to the
`WorldModel*` stored as `world_model`.

### PlannedActionNode (StatefulAction base)

Concrete planned actions derive from `PlannedActionNode` and merge
`withBasePorts(...)` into their own `providedPorts()`. There are seven base
ports.

| Port | Carries |
|------|---------|
| `ame_preconditions` | Grounded facts that must be true before the action starts |
| `ame_confirmed_preconditions` | Grounded facts that must be true *and* observed, never merely predicted. The compiler puts a precondition here when the domain declares its predicate in `(:confirmed-predicates ...)` |
| `ame_neg_preconditions` | Grounded facts that must be false before the action starts |
| `ame_add_effects` | Grounded facts the action records as true when it succeeds |
| `ame_del_effects` | Grounded facts the action records as false when it succeeds |
| `ame_reactive` | Whether to recheck the preconditions on every tick, not only at the start |
| `ame_required_authority` | The evidence *every* precondition must carry: `any`, the default, or `confirmed` |

The base checks all preconditions before calling `onActionStart()`. When the
reactive port is true, it checks them again before each call to
`onActionRunning()`. It calls `commitEffects()` only after the concrete action
reports `SUCCESS`. The default implementation records add effects as true and
delete effects as false with `BELIEVED` authority.

A deployed action can override `commitEffects()` to record what it actually
confirmed. The core does not enforce confirmed state. A synchronous action
only needs to implement `onActionStart()` and return a final status.

`ame_required_authority` decides what a precondition has to carry before the
action will start. The default, `any`, accepts a fact whether it was predicted
by a plan effect or observed by perception. Set to `confirmed`, the action
starts only when every precondition carries `CONFIRMED` authority, which is how
a deployment, or a tool driving one, asks for execution to proceed on evidence
rather than on prediction. The plan compiler does not emit this attribute; it is
set by whoever loads the tree, as DevEnv does from its "Preconditions" control.
An `IWorldStateAccess` implementation that cannot report authority answers
`BELIEVED`, so such an action refuses to run rather than proceeding on a fact
that cannot be judged.

The two ways of demanding observed state answer to different people.
`ame_required_authority` raises the bar for one node's whole precondition set,
and it is set at load time by whoever is running the tree.
`ame_confirmed_preconditions` names individual facts, and the compiler fills it
in from the domain's `(:confirmed-predicates ...)` declaration, so the demand
follows the fact wherever it is used as a precondition and does not depend on
who loaded the tree. One action can therefore mix the two kinds: a strike that
may proceed on a predicted "airborne" but must have an observed "authorised".

### PlannedAction (Decorator)

Applies the same contract to a registered multi-node XML template. It checks
preconditions, ticks its one child, and commits effects only when that child
succeeds.

### SimulatedAction (StatefulAction)

Provides deterministic stand-in behavior for unregistered actions and future
authoring-tool simulations. Its `ticks` input defaults to one and its `success`
input defaults to true. It inherits the normal planned-action checks and
effect commit behavior.

### GoalReached (Condition)

Reads the semicolon-separated grounded facts in its `goals` port and succeeds
only when every fact is true through the world-state access interface.

### CheckWorldPredicate (Condition)

Reads `predicate` port (string key), queries `WorldModel::getFact()`, returns SUCCESS/FAILURE.

This node remains available for hand-written condition and contingency trees.
The compiler does not emit it.

### Replan signalling (no dedicated node)

There is no `ReplanOnFailure` decorator node. Replanning is not signalled from
inside the tree: when the root tree returns `FAILURE`, `ExecutorComponent`
publishes `FAILURE` on its status port and the autonomy backend
(`CurrentAmeBackendAdapter`) observes that status and drives the replan (see
[Replanning](#replanning) below).

### ExecutePhaseAction (StatefulAction)

Orchestrates a full **plan -> compile -> execute** cycle for a sub-goal set, enabling hierarchical decomposition within a parent behaviour tree.

- Ports: `phase_goals`, optional `phase_name`
- Planning paths:
  1. Direct (`planner`, `plan_compiler`, `action_registry` on blackboard)
  2. Component path (`planner_component`) for distributed ROS2 deployments
- Causal audit integration: writes `episode_id`, `parent_episode_id`, and `phase_name` when `plan_audit_log` is available
- Lifecycle hooks: `onActionStart()` plans and creates the subtree,
  `onActionRunning()` ticks it, and `onActionHalted()` halts it.

### InvokeService (StatefulAction)

Asynchronous PYRAMID service invocation BT node. Keeps the core SDK-agnostic via `IPyramidService`.

- Request construction: explicit request JSON + optional PDDL parameter auto-mapping (`param_names`/`param_values`)
- Async lifecycle:
  - `onActionStart()` -> `callAsync()`
  - `onActionRunning()` -> `pollResult()` until terminal status
  - `onActionHalted()` -> `cancelCall()`
- Timeout control: `timeout_ms` (default 5000, `0` disables timeout)
- Blackboard dependency: `pyramid_service` must contain `IPyramidService*`

### DelegateToAgent (StatefulAction)

Leader-delegation node for multi-agent execution. Plans and runs a subtree scoped to a specific `agent_id` and `agent_goals`.

- Marks agent unavailable while delegated subtree runs
- Injects agent context into the compiled subtree blackboard
- Restores availability on completion or halt

### Dispatch nodes

`ExecutorComponent::on_configure()` registers `CheckWorldPredicate`,
`GoalReached`, `PlannedAction` and `SimulatedAction`. Once an `ActionRegistry` is
attached it also registers one `AmeDispatchNode` per registered verb, under the
verb's own name, which is the bridge to `IExecutionSink`.

`ame_core` ships no domain-specific guard leaves. A permission gate such as
"authorised" is written as an ordinary domain precondition, so the planned-action
base refuses to start the action until the fact holds, and declaring its
predicate in `(:confirmed-predicates ...)` makes that fact one the deployment
has to observe rather than one a plan may predict. Gating is therefore fail-closed
by construction and needs no node of its own. A deployment that wants a bespoke
guard node registers it on the factory itself, alongside its own action nodes.

## Replanning

There is no monolithic `MissionExecutor` class. The plan → execute → replan loop
is split across PCL components and driven by an autonomy backend:

| Piece | Header | Role |
|-------|--------|------|
| `ExecutorComponent` | `executor_component.h` | Owns the BT. `loadAndExecute(bt_xml)` loads a compiled tree; `on_tick()` (default 50 Hz) calls `tickOnce()` and publishes `IDLE`/`RUNNING`/`SUCCESS`/`FAILURE` on its `executor/status` port. |
| `PlannerComponent` | `planner_component.h` | Plans and compiles, publishing compiled BT XML on its `bt_xml` port for the executor to load. |
| `CurrentAmeBackendAdapter` | `current_ame_backend_adapter.h` | Implements `IAutonomyBackend`; orchestrates the loop. |

`CurrentAmeBackendAdapter::step()` ticks the executor and, when
`ExecutorComponent::lastStatus()` is `FAILURE`, snapshots the current world model,
replans from that state, and reloads the executor — bounded by
`policy_.max_replans` (default 3), after which it stops.

The demo executable (`src/apps/main.cpp`) uses the simpler single-shot form: it
compiles one tree and drives it with `BT::Tree::tickWhileRunning()`.

On failure the sequence is: halt tree, snapshot world model (which may have been
updated by perception since the failure), replan from current state, recompile,
swap tree, resume ticking — up to `max_replans` times.

### Repair Hook Seam (`AME_NEURO`)

`ExecutorComponent` exposes a `RepairHook` that fires before the baseline FAILURE
path when `ame_neuro` is linked:

```
BT tick → FAILURE
  │
  ├─[repair_hook_ set]──► hook(failed_step, current_wm)
  │                          │
  │                          ├─[non-empty BT XML]──► loadAndExecute(xml)
  │                          │                       publish RUNNING; continue ticking
  │                          │
  │                          └─[empty / exception]──► baseline FAILURE path
  │
  └─[no hook]──► baseline FAILURE path (ExecutorComponent publishes FAILURE;
                  PlannerComponent triggers full symbolic replan)
```

`failed_step` is computed by counting how many top-level planned-action children of
the root sequence have `SUCCESS` status in the live BT tree at failure time.

The hook closure is responsible for compiling PlanStep proposals to BT XML (it
captures `PlanCompiler` and `ActionRegistry` from its own scope).

No hook → baseline FAILURE path identical to `AME_NEURO=OFF`.
See [08-neuro-symbolic.md § 9](08-neuro-symbolic.md) for the full seam specification.
