# Autonomy Backend Shell

This guide describes the implemented whole-system swap surface in:

- `include/ame/autonomy_backend.h`
- `include/ame/current_ame_backend_adapter.h`
- `src/lib/autonomy_backend.cpp`

The goal of this surface is to wrap the **current AME stack** as one replaceable autonomy backend without changing planner/compiler internals.

## What It Wraps

The current implementation is `ame::CurrentAmeBackendAdapter`.

It wraps:

- `WorldModel`
- `Planner`
- `PlanCompiler`
- `ActionRegistry`
- `ExecutorComponent`
- `InvokeService` calls through an `IPyramidService` proxy

It does **not** replace or rewrite those components. Instead, it exposes a stable shell around them:

`state + intent in -> current AME stack runs -> goal dispatches and/or PYRAMID commands + decision records out`

## Why This Exists

The existing architecture already has good internal separation, but external integrations still tend to think in planner-specific or BT-specific terms.

`IAutonomyBackend` provides a higher-level boundary so callers can integrate against a single whole-system interface instead of binding directly to:

- planner calls
- BT compilation details
- world-model mutation conventions

That makes the current AME stack one backend implementation among future alternatives.

## Public Surface

The implemented top-level interface is `ame::IAutonomyBackend`.

`include/ame/autonomy_backend.h` is backend-neutral and intentionally does not include AME planning or world-model headers.

`include/ame/current_ame_backend_adapter.h` contains the AME-specific implementation that wraps the current stack.

Main ingress methods:

- `start(SessionRequest)`
- `pushState(StateUpdate)`
- `pushIntent(MissionIntent)`
- `step()`

Main egress methods:

- `pullCommands()`
- `pullGoalDispatches()`
- `pullDecisionRecords()`
- `pushCommandResult(CommandResult)`
- `pushDispatchResult(DispatchResult)`
- `readSnapshot()`

## Current Adapter Semantics

`CurrentAmeBackendAdapter` now supports two egress modes:

- **delegation mode**
  - emits `GoalDispatch` artefacts for assigned agents
- **execution mode**
  - runs the local BT and emits PYRAMID `ActionCommand` artefacts from `InvokeService`

The mode is selected by policy:

- `PolicyEnvelope.enable_goal_dispatch = true`
  - agent assignment / goal delegation path
- `PolicyEnvelope.enable_goal_dispatch = false`
  - local execution / PYRAMID command path

### Execution Mode

In execution mode, `CurrentAmeBackendAdapter` behaves as a **live planner/compiler/executor wrapper**:

1. `start()` sets the active goals in the wrapped `WorldModel`
2. `step()` runs the existing `Planner`
3. the resulting plan is compiled with the existing `PlanCompiler`
4. the compiled BT is loaded into the existing `ExecutorComponent`
5. the BT runs as normal
6. when an `InvokeService` node calls `IPyramidService::callAsync(...)`, the adapter's proxy intercepts that call and emits an `ActionCommand`
7. the caller dispatches that PYRAMID command externally
8. `pushCommandResult()` feeds the service result back into the proxy
9. the running BT resumes naturally via `pollResult(...)`
10. on failure:
   - the BT fails through its normal execution path
   - the adapter marks itself ready to replan on the next `step()`

This means the egress commands are now the **actual PYRAMID service calls invoked by the running behaviour tree**, not a precomputed command list derived directly from the symbolic plan.

### Delegation Mode

In delegation mode, the adapter uses the current `GoalAllocator` against registered agents in the `WorldModel`:

1. `start()` sets the active goals in the wrapped `WorldModel`
2. `step()` checks policy and available agents
3. the adapter groups and assigns mission goals to agents
4. `pullGoalDispatches()` returns one or more `GoalDispatch` artefacts
5. the caller forwards those dispatches to subordinate agents or backends
6. `pushDispatchResult()` reports completion or failure back to the adapter
7. the adapter either:
   - returns to `READY` / `COMPLETE`
   - or triggers replanning according to policy

This gives the same shell a clean place for multi-agent task allocation without conflating delegation with runtime service invocation.

## Example Usage

```cpp
#include "ame/action_registry.h"
#include "ame/autonomy_backend.h"
#include "ame/current_ame_backend_adapter.h"
#include "ame/world_model.h"

ame::WorldModel wm;
ame::ActionRegistry registry;

// Domain setup omitted for brevity.
wm.setFact("(at uav1 base)", true, "init", ame::FactAuthority::CONFIRMED);
registry.registerActionSubTree(
    "move",
    "<InvokeService service_name=\"mobility\" operation=\"move\" "
    "param_names=\"?robot;?from;?to\" "
    "param_values=\"{param0};{param1};{param2}\" timeout_ms=\"0\"/>");
registry.registerActionSubTree(
    "search",
    "<InvokeService service_name=\"imaging\" operation=\"search\" "
    "param_names=\"?robot;?sector\" "
    "param_values=\"{param0};{param1}\" timeout_ms=\"0\"/>");

ame::CurrentAmeBackendAdapter backend(wm, registry);

backend.start({
    "mission-001",
    {{"(searched sector_a)"}},
    {3}
});

backend.step();

auto decisions = backend.pullDecisionRecords();
auto commands = backend.pullCommands();

for (const auto& command : commands) {
  // External dispatcher / robot middleware executes the emitted PYRAMID call.
  // Example fields:
  //   command.service_name == "mobility"
  //   command.operation == "move"
  //   command.request_fields["robot"] == "uav1"
  backend.pushCommandResult({
      command.command_id,
      ame::CommandStatus::SUCCEEDED,
      {},
      "dispatcher"
  });
}

backend.step();
auto snapshot = backend.readSnapshot();
```

In the current AME adapter, the first `pullCommands()` after planning typically returns the first PYRAMID call the BT actually invokes, not the whole symbolic plan. After the external result is pushed back in, a later `step()` may emit the next PYRAMID call.

## Example: Goal Dispatch / Agent Assignment

```cpp
ame::WorldModel wm;
wm.registerAgent("uav1", "uav");
wm.registerAgent("uav2", "uav");

ame::ActionRegistry registry;
ame::CurrentAmeBackendAdapter backend(wm, registry);

ame::SessionRequest request;
request.session_id = "mission-002";
request.intent.goal_fluents = {"(searched sector_a)"};
request.policy.max_replans = 3;
request.policy.enable_goal_dispatch = true;

backend.start(request);
backend.step();

auto dispatches = backend.pullGoalDispatches();
for (const auto& dispatch : dispatches) {
  // Forward goals to the selected agent/backend.
  // Example:
  //   dispatch.agent_id == "uav1"
  //   dispatch.goals == {"(searched sector_a)"}
  backend.pushDispatchResult({
      dispatch.dispatch_id,
      ame::CommandStatus::SUCCEEDED,
      {},
      "agent-backend"
  });
}
```

## Example: Using Confirmed Effects

If an external dispatcher or perception layer can provide ground-truth results, pass them via `CommandResult::observed_updates`.

```cpp
backend.pushCommandResult({
    move_command.command_id,
    ame::CommandStatus::SUCCEEDED,
    {
        {"(at uav1 base)", false, "perception:gps", ame::FactAuthorityLevel::CONFIRMED},
        {"(at uav1 sector_a)", true, "perception:gps", ame::FactAuthorityLevel::CONFIRMED}
    },
    "dispatcher"
});
```

When observed updates are supplied, they are applied back into the wrapped `WorldModel` before the BT continues.

## Data Objects

### Ingress

- `SessionRequest`
  - starts a backend session
- `MissionIntent`
  - currently wraps goal fluents
- `StateUpdate`
  - applies authoritative fact changes
- `FactUpdate`
  - single fact mutation with source and authority

### Egress

- `ActionCommand`
  - command id, action name, signature, `service_name`, `operation`, `request_fields`
- `GoalDispatch`
  - dispatch id, target `agent_id`, delegated `goals`
- `DispatchResult`
  - result for a previously emitted `GoalDispatch`
- `DecisionRecord`
  - session metadata, solve time, action signatures, compiled BT XML
- `AutonomyBackendSnapshot`
  - wrapper-visible state, outstanding commands, outstanding goal dispatches, decision history

## Current Limitations

This first implementation is deliberately conservative.

It currently:

- wraps the existing planner/compiler/executor flow and intercepts only `InvokeService`-based external actions
- supports goal delegation only through the current `GoalAllocator` heuristic
- assumes external command execution happens through the PYRAMID service boundary
- does not yet expose richer runtime node-level execution provenance beyond the emitted command records and existing BT logs
- treats command success without observed updates as service success with no additional state confirmation
- uses `WorldModel` goals directly as the current mission-intent representation
- replans at the whole-plan level rather than doing local repair

These are acceptable limitations for a first swap surface because the design goal is still not to redesign AME internals. The goal is to provide a stable outer shell that a future backend can also implement while preserving the current BT runtime behaviour.

## Tests

Coverage for the implemented surface lives in:

- `tests/test_autonomy_backend.cpp`
- `tests/test_autonomy_backend_python.py`

The tests cover:

- decision emission from the current stack
- goal dispatch / agent assignment emission
- runtime PYRAMID command emission from the running BT
- world-state progression from successful command results
- confirmed observed updates overriding predicted effects
- failure-driven replanning through the wrapper surface

## Recommended Next Steps

If this shell becomes a real integration boundary, likely follow-on work is:

1. add richer session policy fields
2. expose backend-neutral telemetry sinks
3. expose explicit correlation between BT node instance and emitted PYRAMID command
4. expose explicit correlation between delegated goal dispatch and downstream agent execution episodes
5. support richer service response payload mapping back into world state or blackboard
6. support rolling-horizon or one-command-at-a-time dispatch policies above the current BT runtime
