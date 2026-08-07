# WorldModel & TypeSystem

The WorldModel is the central shared state of the system. Components in the
same process can access it by reference. Behavior tree actions use the
`IWorldStateAccess` interface so the same nodes also work when the world model
is in another process.

## TypeSystem

`TypeSystem` (`include/ame/type_system.h`) manages a single-inheritance type hierarchy and object registry:

- Type hierarchy: name -> parent (e.g., `uav` -> `vehicle` -> `object`)
- Object registry: name -> type (e.g., `uav1` -> `uav`)
- Validated at grounding time

## WorldModel API

```cpp
class WorldModel {
public:
    // Object/type management
    ObjectId addObject(const std::string& name, const std::string& type);

    // Predicate management
    PredicateId registerPredicate(const std::string& name,
                                  const std::vector<std::string>& param_types);

    // State manipulation (authoritative)
    void setFact(PredicateId pred, const std::vector<ObjectId>& args, bool value);
    bool getFact(PredicateId pred, const std::vector<ObjectId>& args) const;

    // String-keyed access (used by BT nodes)
    // authority: BELIEVED (plan effects) or CONFIRMED (perception)
    void setFact(const std::string& key, bool value,
                 const std::string& source = "",
                 FactAuthority authority = FactAuthority::BELIEVED);
    bool getFact(const std::string& key) const;
    FactMetadata getFactMetadata(const std::string& key) const;

    // Index-based access (used by LAPKT projection)
    void setFact(unsigned fluent_id, bool value,
                 const std::string& source = "",
                 FactAuthority authority = FactAuthority::BELIEVED);
    bool getFact(unsigned fluent_id) const;
    FactMetadata getFactMetadata(unsigned fluent_id) const;

    // Thread-safe snapshots (RCU semantics)
    WorldStateSnapshotPtr captureSnapshot() const;

    // Mutation queue for batched perception updates
    void enqueueMutation(unsigned id, bool value,
                         const std::string& source,
                         FactAuthority authority = FactAuthority::CONFIRMED);
    size_t applyQueuedMutations();
    bool hasPendingMutations() const;

    // PDDL projection
    void projectToSTRIPS(aptk::STRIPS_Problem& prob) const;
    aptk::State* currentStateAsSTRIPS() const;

    // Change tracking
    uint64_t version() const;  // monotonic, increments on any state change
};
```

## Eager Grounding

When an object is added, all predicates that can bind to its type are immediately grounded. Fluent indices are stable -- once assigned, they never change. This makes `projectToSTRIPS()` a direct mapping and allows replanning without rebuilding the problem.

Facts are stored as a dynamic bitset (`vector<uint64_t>`) -- compact and efficient. A `BiMap<unsigned, std::string>` maps fluent indices to string keys (e.g., `"at(uav1,sectorA)"`).

## Authoritative State

The WorldModel owns all truth:

- Planned actions read and write facts through the `world_state` blackboard
  entry, which contains an `IWorldStateAccess*`.
- `LocalWorldStateAccess` forwards those calls to an in-process `WorldModel*`.
- If `world_state` is absent, planned actions use the existing `world_model`
  blackboard entry as a compatibility fallback.
- A distributed executor supplies a service-backed implementation instead.
- LAPKT gets a **snapshot projection** via `projectToSTRIPS()`
- External systems update state via WorldModel APIs and ROS2 adapters (see [06-ros2.md](06-ros2.md))

The interface is intentionally small:

```cpp
class IWorldStateAccess {
public:
    virtual bool getFact(const std::string& key) = 0;
    virtual bool setFact(const std::string& key, bool value,
                         const std::string& source) = 0;
    virtual FactAuthority factAuthority(const std::string& key);
};
```

The local and ROS2 implementations record writes made by the default planned
action behavior as `BELIEVED` facts.

`factAuthority()` is only consulted by an action whose `ame_required_authority`
port asks for confirmed preconditions. It is the one method with a default
implementation, which answers `BELIEVED`, so an implementation that cannot
report authority causes such an action to refuse to run rather than to proceed
on a fact whose provenance is unknown. The local implementation reads
`WorldModel::getFactMetadata()`; the ROS2 one reads the authority field of the
`GetFact` service response.

## Error Model

On action failure, the action does not apply its PDDL effects. When an action
succeeds, the core planned-action base records its declared effects as
`BELIEVED`. A deployed action can override `commitEffects()` and record only
the state that its external system or sensors actually established. The core
does not require confirmed state after an action.

## State Authority Semantics

Facts are tagged with authority to distinguish their provenance:

| Authority | Meaning | Source |
|-----------|---------|--------|
| `BELIEVED` | Predicted by plan effects | Default `PlannedActionNode::commitEffects()` |
| `CONFIRMED` | Observed by perception | Perception callbacks, `/detections` topic |

```cpp
enum class FactAuthority { BELIEVED, CONFIRMED };

struct FactMetadata {
    FactAuthority authority;
    uint64_t timestamp_us;   // microseconds since epoch
    std::string source;      // originator tag
};
```

Use `hasAuthorityConflict(id, perceived_value)` to detect when perception disagrees with plan predictions.

## Thread-Safe Snapshots

Concurrent perception callbacks and BT tick threads require synchronisation. WorldModel uses RCU (Read-Copy-Update) semantics:

- **Readers** (BT tick): Call `captureSnapshot()` at tick start, read from immutable `WorldStateSnapshotPtr`
- **Writers** (perception): Call `setFact()` directly or queue via `enqueueMutation()`
- **Synchronisation**: `std::shared_mutex` protects state; mutation queue uses separate `std::mutex`

```cpp
// BT tick thread:
auto snapshot = wm.captureSnapshot();  // immutable copy
bool val = snapshot->getFact(fluent_id);
auto& meta = snapshot->getMetadata(fluent_id);

// Perception thread (option 1 - immediate):
wm.setFact(key, true, "perception:camera", FactAuthority::CONFIRMED);

// Perception thread (option 2 - batched):
wm.enqueueMutation(id, true, "perception:lidar", FactAuthority::CONFIRMED);
// ... later, between BT ticks:
wm.applyQueuedMutations();  // atomically applies all queued updates
```

## Audit Callback

Every `setFact()` call fires an `AuditCallback` with: world model version, timestamp, fact key, new value, and source tag. This powers Layer 3 observability (see [05-observability.md](05-observability.md)).

## Perception Integration (Consolidated from Extension 3)

Perception updates enter `WorldModel` through two supported paths:

1. **ROS2 detections topic** (distributed integration)
   - `WorldModelNode` subscribes to `/detections` (`ame_ros2::msg::Detection`).
   - Detections below `perception.confidence_threshold` are ignored.
   - Accepted detections are mapped to `setFact(..., FactAuthority::CONFIRMED)`.

2. **In-process mutation queue** (low-overhead integration)
   - Sensor threads call `enqueueMutation(...)` without taking the world-state write lock.
   - The BT tick thread calls `applyQueuedMutations()` between ticks for atomic batch apply.

This preserves deterministic execution while still allowing asynchronous perception input.

## Thread Safety Model (Consolidated from Extension 5)

`WorldModel` is safe for concurrent readers/writers by design:

- **Immutable snapshots:** `captureSnapshot()` returns `shared_ptr<const WorldStateData>` for lock-safe reads across BT ticks.
- **Reader/writer lock:** `std::shared_mutex` protects live world-state access.
- **Mutation queue lock:** dedicated `std::mutex` protects queued perception writes.
- **Conflict visibility:** `hasAuthorityConflict()` surfaces disagreement between BELIEVED (planned effects) and CONFIRMED (observed reality).

Use pattern:

```cpp
// sensor thread
wm.enqueueMutation(fluent_id, true, "perception:camera", FactAuthority::CONFIRMED);

// tick thread
wm.applyQueuedMutations();
auto snap = wm.captureSnapshot();
```
