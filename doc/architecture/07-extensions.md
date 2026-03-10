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

PDDL 2.1 durative actions with STN (Simple Temporal Network) conversion. Requires:

- Temporal planner backend (LAPKT temporal extensions or external solver like OPTIC)
- PDDL 2.1 parser support (`:durative-actions`, `:fluents`)
- Numeric fluents in WorldModel (extend bitset to typed value store)
- Temporal causal links in PlanCompiler (start/end time points)
- Durative actions → STN → timed Parallel/Sequence with deadline decorators

Depends on core vertical slice + hierarchical planning (both done).
