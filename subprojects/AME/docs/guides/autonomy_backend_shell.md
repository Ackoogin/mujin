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

It does **not** replace or rewrite those components. Instead, it exposes a stable shell around them:

`state + intent in -> current AME stack -> action commands + decision records out`

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
- `pullDecisionRecords()`
- `pushCommandResult(CommandResult)`
- `readSnapshot()`

## Current Adapter Semantics

`CurrentAmeBackendAdapter` behaves as an **episodic planner-backed command generator**:

1. `start()` sets the active goals in the wrapped `WorldModel`
2. `step()` runs the existing `Planner`
3. the resulting plan is compiled with the existing `PlanCompiler`
4. the adapter emits:
   - `DecisionRecord` containing planning metadata and compiled BT XML
   - `ActionCommand` records derived from the plan's grounded actions
5. the caller dispatches those commands externally
6. `pushCommandResult()` feeds execution outcomes back into the backend
7. on success:
   - observed fact updates are applied if provided
   - otherwise predicted action effects are applied as `BELIEVED`
8. on failure:
   - the adapter marks itself ready to replan on the next `step()`

This gives a complete state-ingress / action-egress loop while keeping the existing solver and compiler untouched.

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
registry.registerAction("move", "MoveAction");
registry.registerAction("search", "SearchAction");

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
  // External dispatcher / robot middleware executes command.signature.
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

When observed updates are supplied, they take precedence over predicted plan effects.

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
  - command id, action name, signature, parameters, predicted effects
- `DecisionRecord`
  - session metadata, solve time, action signatures, compiled BT XML
- `AutonomyBackendSnapshot`
  - wrapper-visible state, outstanding commands, decision history

## Current Limitations

This first implementation is deliberately conservative.

It currently:

- wraps the existing planner/compiler flow, but does not embed the BT runtime behind the shell
- emits all planned commands as an external batch
- treats command success without observed updates as predicted `BELIEVED` state change
- uses `WorldModel` goals directly as the current mission-intent representation
- replans at the whole-plan level rather than doing local repair

These are acceptable limitations for a first swap surface because the design goal is not to redesign AME internals. The goal is to provide a stable outer shell that a future backend can also implement.

## Tests

Coverage for the implemented surface lives in:

- `tests/test_autonomy_backend.cpp`

The tests cover:

- command and decision emission from the current stack
- world-state progression from successful command results
- confirmed observed updates overriding predicted effects
- failure-driven replanning through the wrapper surface

## Recommended Next Steps

If this shell becomes a real integration boundary, likely follow-on work is:

1. add richer session policy fields
2. expose backend-neutral telemetry sinks
3. support rolling-horizon or one-command-at-a-time dispatch modes
4. optionally wrap the BT runtime as an internal execution mode behind the same interface
