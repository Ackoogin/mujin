# Extensions

Additional capabilities built on top of the core pipeline. Extensions 1–6 are implemented; extension 7 is future work.

## Perception Integration (Extension 3)

**Files:** `include/ame/world_model.h`, `ros2/include/ame_ros2/world_model_node.hpp`, `ros2/msg/Detection.msg`

Perception data flows into the WorldModel via two paths:

### ROS2 `/detections` Topic

`WorldModelNode` subscribes to `/detections` (`ame_ros2::msg::Detection`) and maps detections to `setFact()` calls with `FactAuthority::CONFIRMED`:

```cpp
// Detection message fields:
string entity_id        // e.g., "uav1"
string entity_type      // e.g., "robot"
string[] property_keys  // e.g., ["at"]
string[] property_values // e.g., ["sector_a"]
float32 confidence      // [0.0, 1.0]
string sensor_source    // e.g., "camera_front"
```

Detections below `perception.confidence_threshold` (default 0.5) are filtered.

### In-Process Mutation Queue

For direct sensor integration without ROS2 overhead:

```cpp
// From a sensor thread:
unsigned id = wm.fluentIndex("(at uav1 sector_a)");
wm.enqueueMutation(id, true, "perception:camera", FactAuthority::CONFIRMED);

// From the BT tick thread, between ticks:
wm.applyQueuedMutations();  // atomically applies all queued updates
```

### Authority Conflict Detection

When perception disagrees with plan predictions:

```cpp
if (wm.hasAuthorityConflict(fluent_id, perceived_value)) {
    // Log warning, trigger replan, or escalate
}
```

## PYRAMID Service Nodes (Extension 4)

**Files:** `include/ame/pyramid_service.h`, `include/ame/bt_nodes/invoke_service.h`, `src/ame/bt_nodes/invoke_service.cpp`

`InvokeService` is a `BT::StatefulActionNode` that maps PDDL actions to asynchronous PYRAMID SDK service calls via the `IPyramidService` abstract interface. The core library remains SDK-agnostic; concrete adapters implement `IPyramidService`.

### Async Lifecycle

```
onStart()    — Builds request (explicit fields + PDDL param bindings),
                initiates callAsync(), starts timeout clock
onRunning()  — Polls pollResult(); returns RUNNING while PENDING,
                SUCCESS/FAILURE on completion; enforces timeout
onHalted()   — Cancels the pending async call via cancelCall()
```

### IPyramidService Interface

```cpp
class IPyramidService {
    // Synchronous (blocking) call — retained for backward compatibility
    virtual bool call(service_name, operation, request, response) = 0;

    // Async API
    virtual uint64_t callAsync(service_name, operation, request) = 0;
    virtual AsyncCallStatus pollResult(request_id, response) = 0;
    virtual void cancelCall(request_id) = 0;
};
```

`AsyncCallStatus` enum: `PENDING`, `SUCCESS`, `FAILURE`, `CANCELLED`.

### InvokeService Ports

```cpp
class InvokeService : public BT::StatefulActionNode {
    static PortsList providedPorts() {
        return { InputPort<std::string>("service_name"),
                 InputPort<std::string>("operation"),
                 InputPort<unsigned>("timeout_ms", 5000, "Timeout in ms (0 = none)"),
                 InputPort<std::string>("request_json"),
                 InputPort<std::string>("param_names"),   // PDDL param auto-mapping
                 InputPort<std::string>("param_values"),   // PDDL param auto-mapping
                 OutputPort<std::string>("response_json") };
    }
};
```

### PDDL Parameter Auto-Mapping

When `param_names` and `param_values` are provided (semicolon-separated), the node strips leading `?` from PDDL parameter names and merges the name/value pairs into the service request. This enables `ActionRegistry` to automatically translate grounded PDDL parameters into service call fields:

```xml
<InvokeService service_name="mobility" operation="move"
               param_names="?robot;?from;?to"
               param_values="uav1;base;sector_a"/>
<!-- Request will contain: robot=uav1, from=base, to=sector_a -->
```

### Timeout Handling

- Default timeout: 5000ms
- Set `timeout_ms="0"` for no timeout
- On timeout, `cancelCall()` is invoked and the node returns `FAILURE`

The blackboard key `"pyramid_service"` must hold an `IPyramidService*`. `MockPyramidService` is provided for testing (async calls complete immediately on first poll).

### ROS2 Integration

`ExecutorNode` auto-registers `InvokeService` and injects `IPyramidService*` via `setPyramidService()`. In-process demos use `MockPyramidService`; production deployments should provide a concrete adapter wrapping the PYRAMID SDK.

## Thread Safety (Extension 5)

**Files:** `include/ame/world_model.h`, `src/ame/world_model.cpp`

Thread safety is built directly into `WorldModel` using RCU (Read-Copy-Update) semantics:

### Immutable Snapshots

```cpp
// BT tick thread captures snapshot at tick start:
auto snapshot = wm.captureSnapshot();  // WorldStateSnapshotPtr

// Read from snapshot (immutable, safe across concurrent writes):
bool val = snapshot->getFact(fluent_id);
auto& meta = snapshot->getMetadata(fluent_id);
uint64_t ver = snapshot->version;
```

### Synchronisation

- **State access**: `std::shared_mutex` (multiple readers, exclusive writer)
- **Mutation queue**: Separate `std::mutex` for lock-free enqueue from perception threads
- **Atomic batch apply**: `applyQueuedMutations()` acquires exclusive lock, applies all queued updates

### WorldStateData

```cpp
class WorldStateData {
public:
    std::vector<uint64_t> state_bits;
    std::vector<FactMetadata> fact_metadata;
    uint64_t version;

    bool getFact(unsigned id) const;
    const FactMetadata& getMetadata(unsigned id) const;
};

using WorldStateSnapshotPtr = std::shared_ptr<const WorldStateData>;
```

Snapshots are immutable and reference-counted — safe to hold across multiple BT ticks while live state evolves.

## Hierarchical Planning (Extension 6)

**Files:** `include/ame/bt_nodes/execute_phase_action.h`, `src/bt_nodes/execute_phase_action.cpp`

`ExecutePhaseAction` is a `BT::StatefulActionNode` that orchestrates a full plan–compile–execute cycle for a sub-goal set, enabling hierarchical decomposition of complex missions.

```xml
<ExecutePhaseAction
    phase_goals="(searched sector_a);(classified sector_a)"
    phase_name="recon_phase"/>
```

### Planning Paths

Two planning paths are supported:

1. **Direct** (in-process): requires `"world_model"`, `"planner"`, `"plan_compiler"`, `"action_registry"`, `"bt_factory"` on the blackboard.
2. **PlannerComponent** (distributed-ready): requires `"planner_component"`, `"world_model"`, `"bt_factory"`. When a `PlannerComponent*` is found on the blackboard, it takes precedence — enabling the same BT node to work in both monolithic and distributed deployments.

### Audit Trail (Causal Links)

When `"plan_audit_log"` (`PlanAuditLog*`) is on the blackboard, `ExecutePhaseAction` records each sub-planning episode with hierarchical metadata:

- `episode_id` — unique ID auto-assigned by `PlanAuditLog`
- `parent_episode_id` — links to the parent phase's episode (0 = top-level)
- `phase_name` — human-readable label from the `phase_name` port

After a phase completes, its `episode_id` is written to the blackboard key `"parent_episode_id"`, so any nested `ExecutePhaseAction` nodes in the compiled sub-tree inherit the causal link.

```json
{"episode_id":1,"parent_episode_id":0,"phase_name":"recon_phase","solver":"BRFS",...}
{"episode_id":2,"parent_episode_id":1,"phase_name":"search_sub","solver":"BRFS",...}
```

### Lifecycle

- `onStart()` — parse goals, plan (direct or component), compile BT XML, record audit episode, create sub-tree
- `onRunning()` — tick the compiled subtree once per BT cycle
- `onHalted()` — halt and destroy the subtree

### ROS2 Integration

`ExecutorNode` auto-registers `ExecutePhaseAction` and exposes setters for the required blackboard keys: `setPlanner()`, `setPlanCompiler()`, `setActionRegistry()`, `setPlanAuditLog()`. In-process mode: `combined_main.cpp` wires these from `PlannerNode` accessors. Distributed mode: inject `PlannerComponent*` via `setPlannerComponent()`.

## Multi-Agent Delegation

**Files:** `include/ame/bt_nodes/delegate_to_agent.h`, `src/bt_nodes/delegate_to_agent.cpp`

`DelegateToAgent` is a `BT::StatefulActionNode` that delegates a set of goals to a specific agent, planning and executing actions scoped to that agent. Supports the PYRAMID leader-delegation pattern where a leader plans at the goal level and delegates sub-goals.

```xml
<DelegateToAgent agent_id="uav1"
                 agent_goals="(searched sector_a);(classified sector_a)"/>
```

### Blackboard Keys

Same as `ExecutePhaseAction`: `world_model`, `planner`, `plan_compiler`, `action_registry`, `bt_factory`, and optionally `plan_audit_log` / `parent_episode_id`.

### Lifecycle

- `onStart()` — mark agent unavailable, parse goals, plan with agent context, compile BT XML, create sub-tree
- `onRunning()` — tick the compiled subtree once per BT cycle
- `onHalted()` — halt subtree, restore agent availability

See [`../multi_agent_implementation_plan.md`](../multi_agent_implementation_plan.md) for the full architecture.

### ROS2 Integration

`ExecutorNode` auto-registers `DelegateToAgent`. All required blackboard keys are shared with `ExecutePhaseAction`. For distributed multi-agent execution, `AgentDispatcherNode` provides the `~/dispatch_goals` service and transport layer for dispatching BT XML to remote agent executors.

## Temporal Planning (Extension 7) — Future

PDDL 2.1 durative actions with STN (Simple Temporal Network) conversion. See [`../temporal_extension_research.md`](../temporal_extension_research.md) for the full planner evaluation.

### Planner Strategy

LAPKT does not natively support temporal planning. The recommended approach uses an external temporal planner alongside the existing LAPKT STRIPS path:

| Option | Role | Rationale |
|--------|------|-----------|
| **OPTIC** (primary) | Near-term temporal backend | C++, PDDL 2.1 + 3.0 trajectory constraints, proven IPC planner, subprocess integration |
| **Aries** (evaluate) | Medium-term replacement candidate | Rust, actively developed, built-in STN/Difference Logic solver, hierarchical + temporal |
| **TFD** (fallback) | Alternative if OPTIC licence is problematic | GPL, PlanSys2 ROS2 integration exists |
| **LAPKT** (retained) | STRIPS execution planner | Unchanged for non-temporal domains; fast, auditable |

Integration pattern: `IPlannerBackend` abstraction selects LAPKT for STRIPS domains and the temporal planner for `:durative-actions` domains.

### Architecture Overview

```
PDDL 2.1 Domain --▶ PddlParser (extended) --▶ IPlannerBackend
                                                   │
                                    ┌--------------┼--------------┐
                                    ▼              ▼              ▼
                               LapktBackend   OpticBackend   (future)
                               (STRIPS)       (subprocess)   AriesBackend
                                    │              │
                                    ▼              ▼
                              Linear Plan    Timed Plan
                                    │              │
                                    ▼              ▼
                              PlanCompiler   PlanCompiler
                              (Sequence)     (STN → Parallel/Timeout)
                                    │              │
                                    └------┬-------┘
                                           ▼
                                      BT Executor
```

### STN (Simple Temporal Network)

The temporal planner's output (timed action list) is converted into an STN:

- **Nodes:** Time-points for each action's start and end
- **Edges:** Temporal difference constraints (min/max delays)
- **Consistency:** Bellman-Ford negative cycle check
- **Scheduling:** Earliest-start-time computation via SSSP
- **Compilation:** Topological layers of concurrent actions → `ParallelNode` groups with `TimeoutNode` decorators

### BT Execution of Temporal Plans

| BT.CPP Primitive | Temporal Role |
|------------------|---------------|
| `ParallelNode` | Concurrent durative actions (same time layer) |
| `StatefulActionNode` | Non-blocking action returning `RUNNING` for its duration |
| `TimeoutNode` | STN upper-bound enforcement (deadline) |
| `DelayNode` | STN lower-bound enforcement (earliest start) |
| `ReactiveSequence` | PDDL 2.1 `over all` invariant monitoring |

### Validation

**VAL** (BSD, C++) validates temporal plans against the PDDL 2.1 domain before BT compilation. Catches temporal mutex conflicts, invariant breaches, and constraint violations.

### Required Infrastructure

- PDDL 2.1 parser extensions (`:durative-action`, `:functions`, `at start`/`at end`/`over all`)
- WorldModel numeric fluent store (typed value map alongside Boolean bitset)
- `IPlannerBackend` abstraction for planner selection
- STN data structure (~200-300 LOC: graph, consistency check, scheduling)
- PlanCompiler temporal compilation path (STN → BT)
- VAL post-planning validation step

Depends on core vertical slice + hierarchical planning (both done).
