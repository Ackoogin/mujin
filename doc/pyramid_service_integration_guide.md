# PYRAMID Service Integration Guide

How to connect PDDL-planned actions to real PYRAMID SDK service calls via `InvokeService`.

---

## Overview

The AME pipeline separates **planning** from **execution**. The LAPKT planner reasons over abstract PDDL preconditions and effects; it knows nothing about PYRAMID services. At execution time, the `InvokeService` BT node bridges this gap by mapping grounded PDDL parameters into asynchronous service calls through the `IPyramidService` interface.

```
PDDL domain          ActionRegistry              PlanCompiler            BT Executor
 (move ?r ?f ?t)  -->  "move" -> template  -->  XML with InvokeService --> callAsync()
                       with {param0}...         + precondition checks      via IPyramidService
                                                + effect writes
```

## Step 1: Implement IPyramidService

Create a concrete adapter that connects to your PYRAMID SDK backend. The interface is defined in `include/ame/pyramid_service.h`:

```cpp
#include "ame/pyramid_service.h"

class MyPyramidAdapter : public ame::IPyramidService {
public:
  bool call(const std::string& service_name,
            const std::string& operation,
            const ame::ServiceMessage& request,
            ame::ServiceMessage& response) override {
    // Synchronous call — translate ServiceMessage fields to your SDK request,
    // invoke the SDK, populate response fields. Return true on success.
  }

  uint64_t callAsync(const std::string& service_name,
                     const std::string& operation,
                     const ame::ServiceMessage& request) override {
    // Initiate a non-blocking call. Store the pending request internally.
    // Return a unique request ID.
  }

  AsyncCallStatus pollResult(uint64_t request_id,
                             ame::ServiceMessage& response) override {
    // Check if the async call has completed.
    // Return PENDING, SUCCESS, FAILURE, or CANCELLED.
    // On SUCCESS/FAILURE, populate response.
  }

  void cancelCall(uint64_t request_id) override {
    // Cancel a pending async call. No-op if already complete.
  }
};
```

`ServiceMessage` is a simple `string -> string` key-value bag. Your adapter is responsible for translating these fields to and from your SDK's native types.

For testing and simulation, use the built-in `MockPyramidService` which completes all calls immediately with `SUCCESS`.

## Step 2: Register PDDL Actions with InvokeService Templates

Use `ActionRegistry::registerActionSubTree()` to bind PDDL action names to `InvokeService` XML templates. The `{param0}`, `{param1}`, ... placeholders are substituted with grounded parameter values at compile time.

```cpp
ame::ActionRegistry registry;

// PDDL: (move ?robot ?from ?to)
// param0=robot, param1=from, param2=to (positional, matching PDDL param order)
registry.registerActionSubTree("move",
    R"(<InvokeService service_name="mobility" operation="move"
        param_names="?robot;?from;?to"
        param_values="{param0};{param1};{param2}"/>)");

// PDDL: (search ?robot ?sector)
registry.registerActionSubTree("search",
    R"(<InvokeService service_name="sensors" operation="area_search"
        param_names="?robot;?sector"
        param_values="{param0};{param1}"/>)");

// PDDL: (classify ?robot ?sector)
// Here we add explicit request fields alongside PDDL params
registry.registerActionSubTree("classify",
    R"(<InvokeService service_name="imaging" operation="classify"
        request_json="confidence_threshold=0.8"
        param_names="?robot;?sector"
        param_values="{param0};{param1}"/>)");
```

### How Param Mapping Works

When the planner produces the grounded action `move(uav1, base, sector_a)`:

1. `ActionRegistry::resolve("move", {"uav1", "base", "sector_a"})` substitutes placeholders:
   - `{param0}` -> `uav1`, `{param1}` -> `base`, `{param2}` -> `sector_a`
2. `PlanCompiler::emitActionUnit()` wraps the result with precondition checks and effects
3. At tick time, `InvokeService::mergeParamBindings()` strips the `?` prefix from param names and merges them into the `ServiceMessage`:
   - `robot=uav1`, `from=base`, `to=sector_a`

If both `request_json` and `param_names`/`param_values` are provided, the fields are merged (param bindings override any key conflicts).

## Step 3: Register InvokeService with the BT Factory

Register the node type and the two world-model nodes that the compiler emits for precondition checks and effect application:

```cpp
BT::BehaviorTreeFactory factory;

// Required by PlanCompiler output
factory.registerNodeType<ame::CheckWorldPredicate>("CheckWorldPredicate");
factory.registerNodeType<ame::SetWorldPredicate>("SetWorldPredicate");

// The service invocation node
factory.registerNodeType<ame::InvokeService>("InvokeService");
```

## Step 4: Provide the Service on the Blackboard

`InvokeService` looks up the key `"pyramid_service"` on the BT blackboard at each `onStart()`. Set it before ticking:

```cpp
MyPyramidAdapter adapter;

auto tree = factory.createTreeFromText(compiled_xml);
tree.rootBlackboard()->set("pyramid_service",
                           static_cast<ame::IPyramidService*>(&adapter));
tree.rootBlackboard()->set("world_model", &wm);
```

The adapter must outlive the tree. If the pointer is null or missing, `InvokeService` returns `FAILURE` immediately.

## Step 5: Plan, Compile, Execute

With everything registered, the standard pipeline works unchanged:

```cpp
// Plan
ame::Planner planner;
auto result = planner.solve(wm);

// Compile to BT XML (InvokeService nodes are emitted automatically)
ame::PlanCompiler compiler;
std::string xml = compiler.compile(result.plan, wm, registry);

// Execute
auto tree = factory.createTreeFromText(xml);
tree.rootBlackboard()->set("pyramid_service",
                           static_cast<ame::IPyramidService*>(&adapter));
tree.rootBlackboard()->set("world_model", &wm);

BT::NodeStatus status = BT::NodeStatus::RUNNING;
while (status == BT::NodeStatus::RUNNING) {
  status = tree.tickOnce();
}
```

### What the Compiled XML Looks Like

For a plan step `move(uav1, base, sector_a)`, the compiler emits:

```xml
<Sequence name="move(uav1,base,sector_a)">
    <CheckWorldPredicate predicate="(at uav1 base)"/>
    <InvokeService service_name="mobility" operation="move"
                   param_names="?robot;?from;?to"
                   param_values="uav1;base;sector_a"/>
    <SetWorldPredicate predicate="(at uav1 sector_a)" value="true"/>
    <SetWorldPredicate predicate="(at uav1 base)" value="false"/>
</Sequence>
```

## Timeout and Cancellation

Configure per-action timeouts via the `timeout_ms` port (default 5000ms):

```cpp
registry.registerActionSubTree("long_survey",
    R"(<InvokeService service_name="sensors" operation="deep_survey"
        timeout_ms="30000"
        param_names="?robot;?area"
        param_values="{param0};{param1}"/>)");
```

- Set `timeout_ms="0"` for no timeout (useful for indefinite operations)
- On timeout, `InvokeService` calls `cancelCall()` on the adapter and returns `FAILURE`
- If the BT halts the node (e.g., during replanning), `onHalted()` also cancels the pending call

## Reactive Actions

To re-check preconditions every tick (useful for actions that can be pre-empted by perception updates), pass `reactive = true` when registering:

```cpp
registry.registerActionSubTree("search",
    R"(<InvokeService service_name="sensors" operation="area_search"
        param_names="?robot;?sector"
        param_values="{param0};{param1}"/>)",
    /*reactive=*/true);
```

This makes the compiler wrap the action unit in a `ReactiveSequence` instead of a `Sequence`, so `CheckWorldPredicate` nodes are re-evaluated on every tick while `InvokeService` returns `RUNNING`.

## Testing with MockPyramidService

For unit tests and simulation, use `MockPyramidService`:

```cpp
ame::MockPyramidService mock;
tree.rootBlackboard()->set("pyramid_service",
                           static_cast<ame::IPyramidService*>(&mock));

// All service calls complete immediately with SUCCESS
auto status = tree.tickOnce();
EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
```

For more realistic test scenarios (delayed completion, failures), subclass `IPyramidService` with custom poll behaviour. See `tests/test_extensions.cpp` for examples including `DelayedService` and `AlwaysFailService`.

## Replanning

When `InvokeService` returns `FAILURE` (timeout, service error, or cancellation), the standard replanning mechanism handles recovery:

1. `MissionExecutor` detects the BT `FAILURE`
2. It snapshots the current `WorldModel` (which may have been updated by perception)
3. LAPKT replans from the current state
4. `PlanCompiler` produces a new tree with fresh `InvokeService` calls
5. Execution resumes

No special service-side handling is needed — the adapter just needs to report `FAILURE` via `pollResult()` and the pipeline handles the rest.

## Summary of Key Files

| File | Role |
|------|------|
| `include/ame/pyramid_service.h` | `IPyramidService` interface, `ServiceMessage`, `MockPyramidService` |
| `include/ame/bt_nodes/invoke_service.h` | `InvokeService` BT node declaration |
| `src/ame/bt_nodes/invoke_service.cpp` | `InvokeService` implementation (param mapping, async lifecycle) |
| `include/ame/action_registry.h` | `ActionRegistry` — binds PDDL names to BT templates |
| `src/ame/action_registry.cpp` | Template substitution and resolution |
| `src/ame/plan_compiler.cpp` | `PlanCompiler` — emits action units with precondition/effect guards |
| `tests/test_extensions.cpp` | Test suite with mock services and param mapping verification |
