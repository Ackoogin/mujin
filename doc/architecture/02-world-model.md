# WorldModel & TypeSystem

The WorldModel is the central shared state of the system. All components access it by reference.

## TypeSystem

`TypeSystem` (`include/ame/type_system.h`) manages a single-inheritance type hierarchy and object registry:

- Type hierarchy: name → parent (e.g., `uav` → `vehicle` → `object`)
- Object registry: name → type (e.g., `uav1` → `uav`)
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

    // Blackboard projection (one-way push, blackboard is read-only)
    void syncToBlackboard(BT::Blackboard::Ptr bb) const;

    // Change tracking
    uint64_t version() const;  // monotonic, increments on any state change
};
```

## Eager Grounding

When an object is added, all predicates that can bind to its type are immediately grounded. Fluent indices are stable — once assigned, they never change. This makes `projectToSTRIPS()` a direct mapping and allows replanning without rebuilding the problem.

Facts are stored as a dynamic bitset (`vector<uint64_t>`) — compact and efficient. A `BiMap<unsigned, std::string>` maps fluent indices to string keys (e.g., `"at(uav1,sectorA)"`).

## Authoritative State

The WorldModel owns all truth:

- The BT blackboard is a **read-only view** pushed via `syncToBlackboard()`
- All mutations go through `SetWorldPredicate` BT nodes that call `setFact()` directly
- LAPKT gets a **snapshot projection** via `projectToSTRIPS()`
- External systems update state via `PerceptionBridge::updateFact()` (see [07-extensions.md](07-extensions.md))

## Error Model

On action failure, the BT does not apply PDDL effects. Ground truth comes from external perception systems calling `setFact()`. The WorldModel never trusts the plan's model of what happened.

## State Authority Semantics

Facts are tagged with authority to distinguish their provenance:

| Authority | Meaning | Source |
|-----------|---------|--------|
| `BELIEVED` | Predicted by plan effects | BT `SetWorldPredicate` nodes |
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
