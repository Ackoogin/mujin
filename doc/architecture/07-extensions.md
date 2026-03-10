# Extensions

Additional capabilities built on top of the core pipeline. Extensions 1–6 are implemented; extension 7 is future work.

## Perception Integration (Extension 3)

**Files:** `include/mujin/perception_bridge.h`, `src/perception_bridge.cpp`

External systems update world model state via `PerceptionBridge::updateFact()`. Updates are buffered (thread-safe) and applied atomically between BT ticks via `flush()`. Every update is tagged with source `"perception"` (optionally `"perception:<subtag>"`) in the audit log.

```cpp
PerceptionBridge bridge(world_model);

// From a sensor thread:
bridge.updateFact("(at uav1 sector_a)", true, "camera_front");

// From the BT tick thread, between ticks:
bridge.flush();  // applies buffered updates, fires audit callbacks
```

Complements the ROS2 `set_fact` service for in-process sensor integration without ROS2 overhead.

## PYRAMID Service Nodes (Extension 4)

**Files:** `include/mujin/pyramid_service.h`, `include/mujin/bt_nodes/invoke_service.h`, `src/bt_nodes/invoke_service.cpp`

`InvokeService` BT node maps PDDL actions to PYRAMID SDK service calls via the `IPyramidService` abstract interface. The core library remains SDK-agnostic; concrete adapters implement `IPyramidService::call()`.

```cpp
class InvokeService : public BT::SyncActionNode {
    static PortsList providedPorts() {
        return { InputPort<std::string>("service_name"),
                 InputPort<std::string>("operation"),
                 InputPort<std::string>("request_json"),
                 OutputPort<std::string>("response_json") };
    }
};
```

The blackboard key `"pyramid_service"` must hold an `IPyramidService*`. `MockPyramidService` is provided for testing.

## Thread Safety (Extension 5)

**Files:** `include/mujin/world_model_snapshot.h`, `src/world_model_snapshot.cpp`

`SnapshotManager` takes atomic point-in-time copies of WorldModel state. The BT tick thread reads from a stable snapshot; perception threads write to the live WorldModel; `SnapshotManager::publish()` swaps in a new consistent snapshot between ticks.

```cpp
SnapshotManager manager(world_model);

// Perception thread:
bridge.flush();        // applies buffered updates to live WorldModel
manager.publish();     // snapshot for next BT tick

// BT tick thread:
auto snap = manager.current();   // shared_ptr, stays alive across publish()
bool v = snap->getFact("(at uav1 sector_a)");
```

Uses `std::shared_mutex` for concurrent reader safety. WorldModel itself remains single-writer (via `PerceptionBridge::flush()` serialisation).

## Hierarchical Planning (Extension 6)

**Files:** `include/mujin/bt_nodes/execute_phase_action.h`, `src/bt_nodes/execute_phase_action.cpp`

`ExecutePhaseAction` is a `BT::StatefulActionNode` that orchestrates a full plan–compile–execute cycle for a sub-goal set, enabling hierarchical decomposition of complex missions.

```xml
<ExecutePhaseAction
    phase_goals="(searched sector_a);(classified sector_a)"
    phase_name="recon_phase"/>
```

Blackboard keys required: `"world_model"`, `"planner"`, `"plan_compiler"`, `"action_registry"`, `"bt_factory"`. PlanAuditLog (Layer 5) captures sub-planning episodes automatically.

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
PDDL 2.1 Domain ──▶ PddlParser (extended) ──▶ IPlannerBackend
                                                   │
                                    ┌──────────────┼──────────────┐
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
                                    └──────┬───────┘
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
