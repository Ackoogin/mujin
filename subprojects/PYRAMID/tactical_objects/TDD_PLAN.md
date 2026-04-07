# Tactical Objects TDD Plan

Concrete red/green/refactor plan for implementing `pyramid/tactical_objects`.

Each phase produces one test file and one production file (or small group). Every phase follows strict TDD:

- **RED**: write a failing test that compiles but does not pass
- **GREEN**: write the minimum production code to make it pass
- **REFACTOR**: clean up without changing behavior

Entities can be created by two paths:

1. **Evidence path**: observations arrive on the subscriber, pass through `CorrelationEngine`, and produce correlated objects automatically.
2. **Direct path**: a caller invokes `createObject` / `updateObject` / `deleteObject` to manage entities without correlation. These objects carry a `direct` provenance marker and bypass the correlation pipeline.

Both paths store objects in the same `ObjectStore` and are queryable through the same `QueryEngine`.

---

## Phase 0 — Types and UUID Key

**Goal**: establish the shared value types and the hashable UUID wrapper used everywhere.

**Files created**:

- `include/TacticalObjectsTypes.h`
- `tests/tactical_objects/Test_Types.cpp`

### RED 0.1 — Enum round-trip

```
///< REQ_TACTICAL_OBJECTS_001: Enums for object type, affiliation, battle dimension, etc.
TEST(TacticalTypes, ObjectTypeEnumCoversAllValues)
  - Construct each ObjectType value (Platform, Person, Equipment, Unit, Formation, Installation, Feature, Route, Point, Area, Zone).
  - Assert each has a distinct underlying integer.
  - FAILS: TacticalObjectsTypes.h does not exist.
```

### GREEN 0.1

Create `TacticalObjectsTypes.h` with:

- `enum class ObjectType { ... }`
- `enum class Affiliation { ... }`
- `enum class BattleDimension { ... }`
- `enum class MilStatus { ... }`
- `enum class Echelon { ... }`
- `enum class Mobility { ... }`
- `enum class ZoneType { ... }`
- `enum class RelationshipType { ... }`

### RED 0.2 — UUIDKey hashable wrapper

```
///< REQ_TACTICAL_OBJECTS_001: UUIDKey wraps pyramid::core::uuid::UUID for use in unordered containers.
TEST(TacticalTypes, UUIDKeyHashAndEquality)
  - Generate two UUIDs via UUIDHelper::generateV4().
  - Wrap each in UUIDKey.
  - Insert both into std::unordered_set<UUIDKey>.
  - Assert set size == 2.
  - Assert same UUID wrapped twice compares equal and hashes identically.
  - FAILS: UUIDKey does not exist.
```

### GREEN 0.2

Add `struct UUIDKey` to `TacticalObjectsTypes.h`:

- holds `pyramid::core::uuid::UUID`
- provides `operator==` and `std::hash<UUIDKey>` specialization

### RED 0.3 — Core data structs

```
///< REQ_TACTICAL_OBJECTS_002: SourceRef, Position, MilClassProfile structs exist and are default-constructible.
TEST(TacticalTypes, CoreStructsDefaultConstruct)
  - Default-construct SourceRef, Position, MilClassProfile, Observation, ObjectDefinition.
  - Assert default confidence == 0.0.
  - FAILS: structs do not exist.
```

### GREEN 0.3

Add to `TacticalObjectsTypes.h`:

- `struct Position { double lat, lon, alt; }`
- `struct Velocity { double north, east, down; }`
- `struct SourceRef { std::string source_system, source_entity_id, source_track_id; ... }`
- `struct MilClassProfile { BattleDimension battle_dim; Affiliation affiliation; std::string role; MilStatus status; Echelon echelon; Mobility mobility; bool hq, task_force, feint_dummy, installation; std::string source_sidc; }`
- `struct Observation { ... }`
- `struct ObjectDefinition { ObjectType type; ... MilClassProfile mil_class; ... }`
- `struct ObjectUpdate { ... }`

### REFACTOR 0

Verify all enums have stream insertion operators for test diagnostics. No behavior changes.

---

## Phase 1 — ObjectStore

**Goal**: sparse ECS-style storage with UUID identity, versioning, and optional component tables.

**Files created**:

- `include/store/ObjectStore.h`
- `include/store/ObjectComponents.h`
- `src/store/ObjectStore.cpp`
- `tests/tactical_objects/Test_ObjectStore.cpp`

### RED 1.1 — Create object and get UUID

```
///< REQ_TACTICAL_OBJECTS_001: ObjectStore allocates a UUID for each new object.
TEST(ObjectStore, CreateObjectReturnsNonNullUUID)
  - Construct ObjectStore.
  - Call createObject(ObjectType::Platform).
  - Assert returned UUID != null UUID.
  - FAILS: ObjectStore does not exist.
```

### GREEN 1.1

Implement `ObjectStore` with:

- `UUID createObject(ObjectType type)` — allocates UUID via `UUIDHelper::generateV4()`, inserts into dense + sparse structures.
- Internal struct `EntityRecord { UUID id; ObjectType type; uint64_t version; }`.

### RED 1.2 — Sparse lookup consistency

```
///< REQ_TACTICAL_OBJECTS_002: Dense iteration count matches sparse lookup count.
TEST(ObjectStore, DenseAndSparseLookupConsistent)
  - Insert 100 objects.
  - Iterate dense storage, count entries.
  - Retrieve each by UUID from sparse map.
  - Assert counts match and every lookup succeeds.
```

### GREEN 1.2

Implement `getRecord(const UUID&)` and dense iteration (`forEachObject(callback)`).

### RED 1.3 — Optional component tables

```
///< REQ_TACTICAL_OBJECTS_003: Entity exists without all components populated.
TEST(ObjectStore, SparseComponentsAbsentByDefault)
  - Create an object.
  - Query hasComponent<KinematicsComponent>(id) -> false.
  - Query hasComponent<MilClassComponent>(id) -> false.
  - Assert both return false, not default-populated.
```

### GREEN 1.3

Implement `ComponentTable<T>` template with:

- `bool has(const UUID&) const`
- `const T* get(const UUID&) const`
- `void set(const UUID&, T)`
- `void remove(const UUID&)`

Add component table instances for each component type inside `ObjectStore`.

### RED 1.4 — Version increment

```
///< REQ_TACTICAL_OBJECTS_004: Version increments on mutation.
TEST(ObjectStore, VersionIncrementsOnUpdate)
  - Create an object, note version v0.
  - Set its KinematicsComponent.
  - Assert version v1 > v0.
  - Set its MilClassComponent.
  - Assert version v2 > v1.
```

### GREEN 1.4

Wire `setComponent` to call `bumpVersion(id)`.

### RED 1.5 — Delete object

```
///< REQ_TACTICAL_OBJECTS_003: Deleted objects are removed from all tables.
TEST(ObjectStore, DeleteRemovesFromAllTables)
  - Create an object, set KinematicsComponent.
  - Call deleteObject(id).
  - Assert getRecord(id) returns nullptr.
  - Assert hasComponent<KinematicsComponent>(id) returns false.
  - Assert dense count decreased by 1.
```

### GREEN 1.5

Implement `deleteObject(const UUID&)` with swap-and-pop on dense array and removal from all component tables.

### RED 1.6 — Sparse storage efficiency

```
///< REQ_TACTICAL_OBJECTS_045: Sparse objects don't allocate unused component entries.
TEST(ObjectStore, SparseObjectsDontAllocateUnusedComponents)
  - Insert 50 objects with identity only.
  - Assert MilClassComponent table size == 0.
  - Assert KinematicsComponent table size == 0.
  - Set MilClassComponent on 1 object.
  - Assert MilClassComponent table size == 1.
```

### GREEN 1.6

Already passes if component tables are independent. Verify and adjust if needed.

### REFACTOR 1

Extract `ComponentTable<T>` into `ObjectComponents.h`. Make `ObjectStore` hold a tuple or map of component tables.

---

## Phase 2 — MilClassEngine

**Goal**: store, normalize, and derive symbol keys from military classification fields.

**Files created**:

- `include/milclass/MilClassEngine.h`
- `src/milclass/MilClassEngine.cpp`
- `tests/tactical_objects/Test_MilClassEngine.cpp`

### RED 2.1 — Store and retrieve all fields

```
///< REQ_TACTICAL_OBJECTS_014: MilClassEngine stores all 2525B fields.
TEST(MilClassEngine, StoreAndRetrieveAllFields)
  - Construct MilClassEngine with an ObjectStore.
  - Create an object.
  - Call setProfile(id, profile) with all fields populated (BattleDimension::Air, Affiliation::Hostile, role="fighter", ...).
  - Call getProfile(id).
  - Assert every field matches what was set.
  - FAILS: MilClassEngine does not exist.
```

### GREEN 2.1

Implement `MilClassEngine`:

- `void setProfile(const UUID&, const MilClassProfile&)` — stores into `ObjectStore`'s `MilClassComponent` table.
- `tl::optional<MilClassProfile> getProfile(const UUID&) const`

### RED 2.2 — Preserve source SIDC

```
///< REQ_TACTICAL_OBJECTS_015: Source SIDC is preserved.
TEST(MilClassEngine, PreserveSourceSIDC)
  - Set a profile with source_sidc = "SHGPUCFRMS-----".
  - Retrieve profile.
  - Assert source_sidc == "SHGPUCFRMS-----".
```

### GREEN 2.2

Already passes if `MilClassProfile` stores `source_sidc` and `setProfile` copies it.

### RED 2.3 — Deterministic derived symbol key

```
///< REQ_TACTICAL_OBJECTS_016: Derived symbol key is deterministic.
TEST(MilClassEngine, DerivedSymbolKeyDeterministic)
  - Set same profile on two different objects.
  - Call deriveSymbolKey(id_a) and deriveSymbolKey(id_b).
  - Assert keys are identical non-empty strings.
  - Change one field on id_b.
  - Assert keys now differ.
```

### GREEN 2.3

Implement `std::string deriveSymbolKey(const UUID&) const` — concatenates normalized field values into a canonical string key.

### REFACTOR 2

Ensure `MilClassProfile` has a clear `operator==` for test assertions.

---

## Phase 3 — SpatialIndex

**Goal**: grid-based spatial index for fast region candidate retrieval.

**Files created**:

- `include/spatial/SpatialIndex.h`
- `src/spatial/SpatialIndex.cpp`
- `tests/tactical_objects/Test_SpatialIndex.cpp`

### RED 3.1 — Insert and query by region

```
///< REQ_TACTICAL_OBJECTS_022: SpatialIndex returns overlapping candidates.
TEST(SpatialIndex, InsertAndQueryByRegion)
  - Construct SpatialIndex with cell_size_deg = 1.0.
  - Insert object A at (51.5, -0.1), object B at (48.8, 2.3), object C at (51.4, -0.2).
  - Query region covering (51.0..52.0, -1.0..0.0).
  - Assert result contains A and C, not B.
  - FAILS: SpatialIndex does not exist.
```

### GREEN 3.1

Implement `SpatialIndex`:

- `void insert(const UUID&, const Position&)`
- `std::vector<UUID> queryRegion(double min_lat, double max_lat, double min_lon, double max_lon) const`
- Internal grid: `std::unordered_map<CellKey, std::vector<UUID>>`

### RED 3.2 — Update position

```
///< REQ_TACTICAL_OBJECTS_043: Moving one object updates only that object's index.
TEST(SpatialIndex, UpdatePositionReindexes)
  - Insert A at (10.0, 20.0).
  - Insert B at (10.0, 20.0).
  - Move A to (50.0, 60.0).
  - Query original region -> only B.
  - Query new region -> only A.
```

### GREEN 3.2

Implement `void update(const UUID&, const Position& old_pos, const Position& new_pos)`.

### RED 3.3 — Remove from index

```
TEST(SpatialIndex, RemoveFromIndex)
  - Insert A, then remove A.
  - Query A's region -> empty.
```

### GREEN 3.3

Implement `void remove(const UUID&, const Position&)`.

### RED 3.4 — Scale candidate filtering

```
///< REQ_TACTICAL_OBJECTS_042: Regional query over 10k objects returns bounded candidate set.
TEST(SpatialIndex, TenThousandObjectsLocalQuery)
  - Insert 10,000 objects uniformly distributed across (-90..90, -180..180).
  - Query a 2x2 degree region.
  - Assert candidate count < 200 (significantly less than 10,000).
```

### GREEN 3.4

Already passes if grid is working correctly.

### REFACTOR 3

Tune default cell size. Add `remove(UUID)` variant that looks up stored position internally if needed.

---

## Phase 4 — ZoneEngine

**Goal**: zone CRUD, geometry checks, and boundary transition detection.

**Files created**:

- `include/zone/ZoneEngine.h`
- `src/zone/ZoneEngine.cpp`
- `tests/tactical_objects/Test_ZoneEngine.cpp`

### RED 4.1 — Create polygon zone

```
///< REQ_TACTICAL_OBJECTS_017: ZoneEngine supports polygon zones.
TEST(ZoneEngine, CreatePolygonZone)
  - Construct ZoneEngine.
  - Create a polygon zone with 4 vertices forming a square.
  - Assert zone is retrievable by ID.
  - Assert zone type == ZoneType::Polygon.
  - FAILS: ZoneEngine does not exist.
```

### GREEN 4.1

Implement `ZoneEngine` with:

- `UUID upsertZone(const ZoneDefinition&)`
- `tl::optional<ZoneDefinition> getZone(const UUID&) const`
- `ZoneDefinition` struct with geometry variant

### RED 4.2 — Create circle zone

```
///< REQ_TACTICAL_OBJECTS_017: ZoneEngine supports circle zones.
TEST(ZoneEngine, CreateCircleZone)
  - Create a circle zone at (51.5, -0.1) radius 5000m.
  - Assert zone is retrievable.
```

### GREEN 4.2

Extend geometry variant to include circle (center + radius_m).

### RED 4.3 — Bounding box cache

```
///< REQ_TACTICAL_OBJECTS_018: Bounding box cached for non-point geometries.
TEST(ZoneEngine, BoundingBoxContainsAllVertices)
  - Create polygon zone with known vertices.
  - Retrieve cached bounding box.
  - Assert all vertices lie within the bounding box.
```

### GREEN 4.3

Compute and cache bounding box on zone creation.

### RED 4.4 — Point inside polygon

```
///< REQ_TACTICAL_OBJECTS_019: Point-in-polygon check.
TEST(ZoneEngine, PointInsidePolygon)
  - Create a square polygon zone.
  - Assert isInside(center_point, zone_id) == true.
  - Assert isInside(far_away_point, zone_id) == false.
```

### GREEN 4.4

Implement ray-casting point-in-polygon algorithm.

### RED 4.5 — Point inside circle

```
///< REQ_TACTICAL_OBJECTS_019: Point-in-circle check.
TEST(ZoneEngine, PointInsideCircle)
  - Create circle zone center=(51.5, -0.1), radius=1000m.
  - Assert point 500m away -> inside.
  - Assert point 2000m away -> outside.
```

### GREEN 4.5

Implement haversine distance check.

### RED 4.6 — Boundary transition detection

```
///< REQ_TACTICAL_OBJECTS_020: Enter and leave transitions detected.
TEST(ZoneEngine, BoundaryTransitionDetection)
  - Create polygon zone.
  - Evaluate object at point outside zone -> state = Outside.
  - Move object to point inside zone -> transition = Enter.
  - Move object back outside -> transition = Leave.
```

### GREEN 4.6

Implement `ZoneTransition evaluateTransition(const UUID& object_id, const UUID& zone_id, const Position& current_pos)` with previous-state tracking.

### RED 4.7 — Time-gated zone

```
///< REQ_TACTICAL_OBJECTS_021: Expired zones excluded from results.
TEST(ZoneEngine, ExpiredZoneExcluded)
  - Create zone with active_until = past timestamp.
  - Evaluate object inside zone geometry.
  - Assert no active relationship reported.
```

### GREEN 4.7

Add time check to zone evaluation.

### RED 4.8 — Zone semantic typing

```
///< REQ_TACTICAL_OBJECTS_055: Zone semantic type is preserved and queryable.
TEST(ZoneEngine, ZoneSemanticTypeRoundTrip)
  - Create zones tagged as AOI, NoGoArea, KillBox, and SensorCoverageArea.
  - Retrieve each by ID and assert semantic type matches what was stored.
  - Query/filter by semantic type and assert only matching zones are returned.
```

### GREEN 4.8

Extend `ZoneDefinition` with an explicit semantic type enum and preserve it through storage, retrieval, and filtering.

### RED 4.9 — Geodesic spatial reasoning

```
///< REQ_TACTICAL_OBJECTS_056: Circle and proximity checks use geodesic reasoning.
TEST(ZoneEngine, GeodesicDistanceUsedForCircleChecks)
  - Create a circle zone at high latitude with radius in meters.
  - Evaluate one point that is inside by haversine distance and one that is outside.
  - Assert the result matches geodesic distance rather than naive planar degrees.
```

### GREEN 4.9

Use haversine or equivalent geodesic calculations for circle containment and proximity checks.

### REFACTOR 4

Extract geometry math into helper functions. Add support for remaining geometry types (polyline, ellipse, corridor) as stubs returning bounding-box-only results.

---

## Phase 5 — CorrelationEngine

**Goal**: evidence-based entity creation via correlation, merge, split, and lineage.

**Files created**:

- `include/correlation/CorrelationEngine.h`
- `src/correlation/CorrelationEngine.cpp`
- `tests/tactical_objects/Test_CorrelationEngine.cpp`

### RED 5.1 — First observation creates new object

```
///< REQ_TACTICAL_OBJECTS_010: Low-score observation creates new object.
TEST(CorrelationEngine, FirstObservationCreatesObject)
  - Construct CorrelationEngine with empty ObjectStore and SpatialIndex.
  - Submit one observation at (51.5, -0.1).
  - Assert ObjectStore now contains 1 correlated object.
  - Assert correlated object has lineage referencing the observation.
  - FAILS: CorrelationEngine does not exist.
```

### GREEN 5.1

Implement `CorrelationEngine`:

- `CorrelationResult processObservation(const Observation&)`
- On empty candidate set: create track, create correlated object, record lineage.

### RED 5.2 — Compatible observation merges

```
///< REQ_TACTICAL_OBJECTS_011: High-score observation merges into existing object.
TEST(CorrelationEngine, CompatibleObservationMerges)
  - Submit observation A at (51.5, -0.1) from source "radar".
  - Submit observation B at (51.5001, -0.1001) from source "ais", same affiliation.
  - Assert ObjectStore still contains 1 correlated object.
  - Assert correlated object has 2 source references.
```

### GREEN 5.2

Implement correlation scoring:

- Query `SpatialIndex` for candidates.
- Score each candidate.
- If best score >= `merge_threshold`: attach observation to existing object.

### RED 5.3 — Distant observation creates new object

```
///< REQ_TACTICAL_OBJECTS_010: Distant observation creates separate object.
TEST(CorrelationEngine, DistantObservationCreatesNewObject)
  - Submit observation A at (51.5, -0.1).
  - Submit observation B at (10.0, 20.0).
  - Assert ObjectStore contains 2 correlated objects.
```

### GREEN 5.3

Already passes if scoring is correct and spatial gate works.

### RED 5.4 — Deterministic scoring

```
///< REQ_TACTICAL_OBJECTS_009: Same input produces same score.
TEST(CorrelationEngine, DeterministicScore)
  - Submit observation A.
  - Record score for observation B against the object created by A.
  - Reset state, repeat.
  - Assert scores are identical.
```

### GREEN 5.4

Already passes if scoring is pure function of input.

### RED 5.5 — Spatial prefilter

```
///< REQ_TACTICAL_OBJECTS_008: Only local candidates are scored.
TEST(CorrelationEngine, SpatialPrefilterLimitsScoring)
  - Insert 100 objects scattered globally.
  - Submit 1 observation at known position.
  - Instrument or mock SpatialIndex to count candidates returned.
  - Assert candidate count << 100.
```

### GREEN 5.5

Already passes if `CorrelationEngine` queries `SpatialIndex` before scoring.

### RED 5.6 — Source reference association

```
///< REQ_TACTICAL_OBJECTS_006: Correlated object retains all contributing source refs.
TEST(CorrelationEngine, SourceRefsRetained)
  - Submit 3 observations from different sources, all merging to same object.
  - Assert correlated object source_refs.size() == 3.
  - Assert each source_system is distinct and present.
```

### GREEN 5.6

Accumulate source refs on the correlated object's `CorrelationComponent` during merge.

### RED 5.7 — Lineage retention

```
///< REQ_TACTICAL_OBJECTS_007: Lineage references contributing observation IDs.
TEST(CorrelationEngine, LineageRetained)
  - Submit 2 observations that merge.
  - Retrieve lineage from correlated object.
  - Assert lineage contains both observation UUIDs.
```

### GREEN 5.7

Append to lineage list during merge.

### RED 5.8 — Confidence aggregation

```
///< REQ_TACTICAL_OBJECTS_013: Aggregate confidence from weighted sources.
TEST(CorrelationEngine, ConfidenceAggregation)
  - Submit observation A with confidence 0.9.
  - Submit observation B with confidence 0.3, merging into same object.
  - Assert correlated object confidence is between 0.3 and 0.9.
```

### GREEN 5.8

Implement weighted confidence aggregation in merge step.

### RED 5.9 — Split on incompatibility

```
///< REQ_TACTICAL_OBJECTS_012: Split after sustained incompatible evidence.
TEST(CorrelationEngine, SplitOnSustainedIncompatibility)
  - Set split_incompatibility_count = 3.
  - Submit initial observation (friendly, ground).
  - Submit 3 consecutive observations at same position but with hostile affiliation.
  - Assert ObjectStore now contains 2 correlated objects.
  - Assert both have lineage.
```

### GREEN 5.9

Implement incompatibility counter on tracks. When counter >= threshold, split.

### REFACTOR 5

Extract scoring into a `CorrelationScorer` helper. Make thresholds configurable via `CorrelationConfig` struct.

---

## Phase 6 — QueryEngine

**Goal**: compound predicate queries over the object store and indexes.

**Files created**:

- `include/query/QueryEngine.h`
- `src/query/QueryEngine.cpp`
- `tests/tactical_objects/Test_QueryEngine.cpp`

### RED 6.1 — Query by UUID

```
///< REQ_TACTICAL_OBJECTS_025: Lookup by internal UUID.
TEST(QueryEngine, QueryByUUID)
  - Insert 3 objects into ObjectStore.
  - Call query with specific UUID.
  - Assert exactly 1 result with correct UUID.
  - FAILS: QueryEngine does not exist.
```

### GREEN 6.1

Implement `QueryEngine`:

- `QueryResult query(const QueryRequest&) const`
- `QueryRequest` has optional UUID field.

### RED 6.2 — Query by external ID

```
///< REQ_TACTICAL_OBJECTS_025: External identifier resolves to correct object.
TEST(QueryEngine, QueryByExternalId)
  - Insert object with source_ref { source_system="radar", source_entity_id="T42" }.
  - Query by source_system="radar", source_entity_id="T42".
  - Assert result returns the correct object.
```

### GREEN 6.2

Add source reference index lookup to QueryEngine.

### RED 6.3 — Query by affiliation

```
TEST(QueryEngine, QueryByAffiliation)
  - Insert 5 objects: 3 friendly, 2 hostile.
  - Query affiliation == Hostile.
  - Assert 2 results.
```

### GREEN 6.3

Add affiliation predicate to query evaluation.

### RED 6.4 — Compound predicate

```
///< REQ_TACTICAL_OBJECTS_024: Compound predicate across type, affiliation, and freshness.
TEST(QueryEngine, CompoundPredicate)
  - Insert mix of objects varying by type, affiliation, and freshness.
  - Query type==Platform AND affiliation==Hostile AND age < 60s.
  - Assert only matching subset returned.
```

### GREEN 6.4

Implement predicate chaining in query evaluation.

### RED 6.5 — Region query with spatial post-filter

```
///< REQ_TACTICAL_OBJECTS_023: Exact post-filter removes false positives.
TEST(QueryEngine, RegionQueryPostFilter)
  - Insert objects with positions that overlap in grid cells but not in exact polygon.
  - Query with polygon region.
  - Assert false positives from grid index are removed.
```

### GREEN 6.5

Wire SpatialIndex for candidates, then apply exact geometry check.

### REFACTOR 6

Normalize `QueryRequest` builder API for ergonomic test setup.

---

## Phase 7 — TacticalObjectsRuntime

**Goal**: composition root integrating all engines. Exposes both evidence and direct entity paths.

**Files created**:

- `include/TacticalObjectsRuntime.h`
- `src/TacticalObjectsRuntime.cpp`
- `tests/tactical_objects/Test_TacticalObjectsRuntime.cpp`

### RED 7.1 — Runtime construction without PCL

```
///< REQ_TACTICAL_OBJECTS_030: Runtime facade works without PCL executor.
TEST(TacticalObjectsRuntime, ConstructWithoutPCL)
  - Construct TacticalObjectsRuntime with default config.
  - Assert getCapability() returns valid snapshot.
  - FAILS: TacticalObjectsRuntime does not exist.
```

### GREEN 7.1

Implement `TacticalObjectsRuntime`:

- Constructor creates `ObjectStore`, `SpatialIndex`, `CorrelationEngine`, `ZoneEngine`, `MilClassEngine`, `QueryEngine`, `EventBus`, `Logger`.
- `getCapability()` returns a `CapabilitySnapshot`.

### RED 7.2 — Evidence path: observation creates entity

```
///< REQ_TACTICAL_OBJECTS_005: Observation batch processed in order.
TEST(TacticalObjectsRuntime, EvidencePathCreatesEntity)
  - Construct runtime.
  - Submit ObservationBatch with 1 observation.
  - Call getObject() with returned ID.
  - Assert object exists with correct position and affiliation.
```

### GREEN 7.2

Wire `upsertObservationBatch` → `CorrelationEngine::processObservation` → `ObjectStore`.

### RED 7.3 — Direct path: create entity via service

```
///< REQ_TACTICAL_OBJECTS_026: Direct create stores object with direct provenance.
TEST(TacticalObjectsRuntime, DirectPathCreatesEntity)
  - Construct runtime.
  - Call createObject(ObjectDefinition{ type=Platform, position={51.5, -0.1, 0}, affiliation=Friendly, battle_dim=Ground, role="armor" }).
  - Assert returned UUID is non-null.
  - Call getObject(id).
  - Assert object type == Platform.
  - Assert mil_class.affiliation == Friendly.
  - Assert mil_class.battle_dim == Ground.
  - Assert mil_class.role == "armor".
  - Assert lineage provenance marker == "direct".
```

### GREEN 7.3

Implement `createObject(const ObjectDefinition&)`:

- Allocate UUID via ObjectStore.
- Set all provided components (identity, kinematics, MilClass, etc.).
- Record lineage with provenance = `direct`.
- Index in SpatialIndex.
- Emit ObjectCreated event on EventBus.

### RED 7.4 — Direct path: update entity

```
///< REQ_TACTICAL_OBJECTS_027: Direct update applies partial mutation and reindexing.
TEST(TacticalObjectsRuntime, DirectPathUpdatesEntity)
  - Create object via direct path with affiliation=Friendly.
  - Call updateObject(id, ObjectUpdate{ affiliation=Hostile }).
  - Retrieve object.
  - Assert affiliation == Hostile.
  - Assert version incremented.
```

### GREEN 7.4

Implement `updateObject(const UUID&, const ObjectUpdate&)`:

- Validate object exists.
- Apply each non-empty field from `ObjectUpdate` to the appropriate component table.
- Bump version.
- Re-index spatial if position changed.
- Emit ObjectUpdated event.

### RED 7.5 — Direct path: delete entity

```
///< REQ_TACTICAL_OBJECTS_028: Direct delete removes entity from store and index.
TEST(TacticalObjectsRuntime, DirectPathDeletesEntity)
  - Create object via direct path.
  - Call deleteObject(id).
  - Assert getObject(id) returns empty.
  - Assert SpatialIndex no longer contains id.
```

### GREEN 7.5

Implement `deleteObject(const UUID&)`:

- Remove from ObjectStore (all component tables).
- Remove from SpatialIndex.
- Emit ObjectDeleted event.

### RED 7.6 — Evidence and direct objects are queryable together

```
///< REQ_TACTICAL_OBJECTS_029: Evidence and direct objects are queryable together.
TEST(TacticalObjectsRuntime, BothPathsQueryableTogether)
  - Create object A via evidence path (observation).
  - Create object B via direct path.
  - Query all objects with type==Platform.
  - Assert result contains both A and B.
```

### GREEN 7.6

Already passes if both paths store into the same `ObjectStore`.

### RED 7.7 — Evidence path batch ordering

```
///< REQ_TACTICAL_OBJECTS_005: Batch processed in input order.
TEST(TacticalObjectsRuntime, ObservationBatchOrderPreserved)
  - Subscribe to EventBus for ObjectCreated events.
  - Submit batch of 3 observations at distinct positions.
  - Assert 3 ObjectCreated events emitted.
  - Assert event order matches input order.
```

### GREEN 7.7

Process observations sequentially within `upsertObservationBatch`.

### RED 7.8 — Zone upsert and object-zone relationship

```
TEST(TacticalObjectsRuntime, ZoneRelationshipEvaluation)
  - Create polygon zone via upsertZone.
  - Create object at position inside zone via direct path.
  - Query by zone relationship (inside zone).
  - Assert object is in result set.
```

### GREEN 7.8

Wire `ZoneEngine` evaluation into query path.

### RED 7.9 — Stale object expiry

```
///< REQ_TACTICAL_OBJECTS_031: Tick expires stale objects.
TEST(TacticalObjectsRuntime, StaleObjectExpiry)
  - Configure stale_timeout = 100ms.
  - Submit observation.
  - Advance mock clock past 100ms.
  - Call tick().
  - Assert object is marked stale or retired.
```

### GREEN 7.9

Implement staleness check in `tick()`.

### RED 7.10 — Capability snapshot

```
///< REQ_TACTICAL_OBJECTS_032: Capability snapshot has configured values.
TEST(TacticalObjectsRuntime, CapabilitySnapshot)
  - Construct runtime with specific config (merge_threshold=0.7, etc.).
  - Call getCapability().
  - Assert snapshot contains object count, zone count, correlation config values.
```

### GREEN 7.10

Populate `CapabilitySnapshot` from current state and config.

### RED 7.11 — Domain events emitted

```
///< REQ_TACTICAL_OBJECTS_039: Events for create, update, correlate, split, zone_enter, zone_leave.
TEST(TacticalObjectsRuntime, DomainEventsEmitted)
  - Subscribe to EventBus for all tactical event types.
  - Create object (direct) -> expect ObjectCreated.
  - Update object -> expect ObjectUpdated.
  - Submit observation that merges -> expect ObjectCorrelated.
  - Submit incompatible observations -> expect ObjectSplit.
  - Move object into zone -> expect ZoneEntered.
  - Move object out of zone -> expect ZoneLeft.
  - Assert all 6 event types received.
```

### GREEN 7.11

Emit events from each relevant code path.

### RED 7.12 — Logger integration

```
///< REQ_TACTICAL_OBJECTS_040: Missing-information diagnostics are emitted.
TEST(TacticalObjectsRuntime, LoggerIntegration)
  - Construct runtime with injected Logger.
  - Submit incomplete data that blocks confident classification or correlation.
  - Assert Logger history identifies the missing information with warning-level diagnostics.
```

### GREEN 7.12

Add logging calls to `createObject`, `upsertObservationBatch`, `deleteObject`.

### RED 7.13 — Batch correlation determinism

```
///< REQ_TACTICAL_OBJECTS_044: Same batch in same order produces same result.
TEST(TacticalObjectsRuntime, BatchCorrelationDeterminism)
  - Build identical ObservationBatch.
  - Process in runtime A, snapshot result graph.
  - Process in fresh runtime B, snapshot result graph.
  - Assert object count, lineage shape, and versions match.
```

### GREEN 7.13

Already passes if all operations are deterministic.

### RED 7.14 — Behavior estimation round-trip

```
///< REQ_TACTICAL_OBJECTS_053: Behavior pattern and intent hypothesis round-trip.
TEST(TacticalObjectsRuntime, BehaviorEstimationRoundTrip)
  - Create an object via direct path with behavior fields populated.
  - Retrieve object and assert behavior pattern and intent hypothesis match.
  - Query using behavior predicate and assert the object is returned.
```

### GREEN 7.14

Store behavior fields in the object store and expose them through retrieval and query filtering.

### RED 7.15 — Operational state and freshness

```
///< REQ_TACTICAL_OBJECTS_054: Operational state and freshness are queryable.
TEST(TacticalObjectsRuntime, OperationalStateAndFreshnessQuery)
  - Create or update an object with operational_state = Airborne and freshness = Confirmed.
  - Query by operational state and freshness window.
  - Assert the object is returned with the expected values.
```

### GREEN 7.15

Store operational state and freshness in runtime-managed components and include them in query predicates.

### REFACTOR 7

Extract `RuntimeConfig` struct. Ensure `createObject` and `upsertObservationBatch` share common indexing and event-emission code paths.

---

## Phase 8 — TacticalObjectsCodec

**Goal**: JSON serialization boundary for PCL messages.

**Files created**:

- `include/TacticalObjectsCodec.h`
- `src/TacticalObjectsCodec.cpp`
- `tests/tactical_objects/Test_TacticalObjectsCodec.cpp`

### RED 8.1 — UUID round-trip

```
///< REQ_TACTICAL_OBJECTS_041: UUID serializes and deserializes to canonical string.
TEST(TacticalObjectsCodec, UUIDRoundTrip)
  - Generate UUID.
  - Serialize to JSON string.
  - Deserialize back.
  - Assert UUIDs are equal.
  - FAILS: TacticalObjectsCodec does not exist.
```

### GREEN 8.1

Implement codec with `nlohmann::json` for UUID serialization.

### RED 8.2 — Observation batch round-trip

```
TEST(TacticalObjectsCodec, ObservationBatchRoundTrip)
  - Build ObservationBatch with 3 observations.
  - Encode to JSON bytes.
  - Decode back.
  - Assert all 3 observations match field-by-field.
```

### GREEN 8.2

Implement `encodeObservationBatch` / `decodeObservationBatch`.

### RED 8.3 — ObjectDefinition round-trip (direct create)

```
TEST(TacticalObjectsCodec, ObjectDefinitionRoundTrip)
  - Build ObjectDefinition with all fields (type, position, mil_class, identity, etc.).
  - Encode to JSON, decode back.
  - Assert all fields match.
```

### GREEN 8.3

Implement `encodeObjectDefinition` / `decodeObjectDefinition`.

### RED 8.4 — QueryRequest/Response round-trip

```
TEST(TacticalObjectsCodec, QueryRoundTrip)
  - Build QueryRequest with compound predicates.
  - Encode, decode.
  - Assert predicates match.
  - Build QueryResponse with 2 result objects.
  - Encode, decode.
  - Assert results match.
```

### GREEN 8.4

Implement query codec.

### RED 8.5 — ZoneDefinition round-trip

```
TEST(TacticalObjectsCodec, ZoneDefinitionRoundTrip)
  - Build ZoneDefinition (polygon with 5 vertices, validity window).
  - Encode, decode.
  - Assert all fields match including geometry vertices.
```

### GREEN 8.5

Implement zone codec.

### REFACTOR 8

Extract common JSON helpers (position, enums, timestamps) into codec utilities.

---

## Phase 9 — TacticalObjectsComponent (PCL Integration)

**Goal**: wire runtime to PCL lifecycle, ports, subscribers, and service handlers.

**Files created**:

- `include/TacticalObjectsComponent.h`
- `src/TacticalObjectsComponent.cpp`
- `tests/tactical_objects/Test_TacticalObjectsComponent.cpp`

### RED 9.1 — Configure creates all ports

```
///< REQ_TACTICAL_OBJECTS_033: All ports created during on_configure.
TEST(TacticalObjectsComponent, ConfigureCreatesAllPorts)
  - Construct TacticalObjectsComponent.
  - Call configure().
  - Assert state == PCL_STATE_CONFIGURED.
  - Assert all expected port pointers are non-null (pub_events_, sub_observation_ingress_, svc_create_object_, svc_query_, etc.).
  - FAILS: TacticalObjectsComponent does not exist.
```

### GREEN 9.1

Implement `TacticalObjectsComponent : public pcl::Component` with `on_configure()` creating all ports.

### RED 9.2 — Activate succeeds

```
TEST(TacticalObjectsComponent, ActivateSucceeds)
  - Configure, then activate.
  - Assert state == PCL_STATE_ACTIVE.
```

### GREEN 9.2

Implement `on_activate()`.

### RED 9.3 — Evidence ingress via subscriber callback

```
///< REQ_TACTICAL_OBJECTS_035: Observation ingress through subscriber callback.
TEST(TacticalObjectsComponent, EvidenceIngressViaSubscriber)
  - Configure + activate.
  - Build ObservationBatch with 1 observation, encode to JSON.
  - Dispatch via pcl_executor_dispatch_incoming() to "tactical_objects.observation_ingress".
  - Query runtime for objects.
  - Assert 1 object created.
```

### GREEN 9.3

Wire subscriber callback: decode message, call `runtime_->upsertObservationBatch(...)`.

### RED 9.4 — Direct create via service handler

```
///< REQ_TACTICAL_OBJECTS_036: Direct create is exposed through service handler.
TEST(TacticalObjectsComponent, DirectCreateViaService)
  - Configure + activate.
  - Build ObjectDefinition (Platform, Friendly, Ground, role="infantry"), encode to JSON as request.
  - Invoke svc_create_object_ handler directly (or via executor dispatch).
  - Decode response.
  - Assert response contains non-null UUID.
  - Query runtime -> object exists with correct fields.
```

### GREEN 9.4

Wire `svc_create_object_` handler: decode `ObjectDefinition`, call `runtime_->createObject(...)`, encode response with UUID.

### RED 9.5 — Direct update via service handler

```
///< REQ_TACTICAL_OBJECTS_036: Direct update is exposed through service handler.
TEST(TacticalObjectsComponent, DirectUpdateViaService)
  - Create object via direct service.
  - Build ObjectUpdate { affiliation = Hostile }, encode as request.
  - Invoke svc_update_object_ handler.
  - Assert response success.
  - Query object -> affiliation == Hostile.
```

### GREEN 9.5

Wire `svc_update_object_` handler: decode `ObjectUpdate`, call `runtime_->updateObject(...)`.

### RED 9.6 — Direct delete via service handler

```
///< REQ_TACTICAL_OBJECTS_036: Direct delete is exposed through service handler.
TEST(TacticalObjectsComponent, DirectDeleteViaService)
  - Create object via direct service.
  - Invoke svc_delete_object_ handler with object UUID.
  - Assert response success.
  - Query -> object not found.
```

### GREEN 9.6

Wire `svc_delete_object_` handler.

### RED 9.7 — Query via service handler

```
///< REQ_TACTICAL_OBJECTS_037: Query through service handler.
TEST(TacticalObjectsComponent, QueryViaService)
  - Create 2 objects (1 evidence, 1 direct).
  - Encode QueryRequest { type = Platform }.
  - Invoke svc_query_ handler.
  - Decode response.
  - Assert 2 results returned.
```

### GREEN 9.7

Wire `svc_query_` handler: decode request, call `runtime_->query(...)`, encode response.

### RED 9.8 — Get single object via service handler

```
TEST(TacticalObjectsComponent, GetObjectViaService)
  - Create 1 object.
  - Invoke svc_get_object_ handler with its UUID.
  - Decode response.
  - Assert response contains full object with mil_class fields.
```

### GREEN 9.8

Wire `svc_get_object_` handler.

### RED 9.9 — Zone upsert via service handler

```
TEST(TacticalObjectsComponent, ZoneUpsertViaService)
  - Encode ZoneDefinition (polygon).
  - Invoke svc_upsert_zone_ handler.
  - Assert response contains zone UUID.
  - Query zones -> zone exists.
```

### GREEN 9.9

Wire `svc_upsert_zone_` handler.

### RED 9.10 — Zone remove via service handler

```
TEST(TacticalObjectsComponent, ZoneRemoveViaService)
  - Create zone.
  - Invoke svc_remove_zone_ handler.
  - Assert zone no longer retrievable.
```

### GREEN 9.10

Wire `svc_remove_zone_` handler.

### RED 9.11 — Tick drives housekeeping

```
///< REQ_TACTICAL_OBJECTS_038: on_tick delegates to runtime tick.
TEST(TacticalObjectsComponent, TickDrivesHousekeeping)
  - Configure + activate with stale_timeout = 1ms.
  - Create object via evidence.
  - Sleep 10ms.
  - Call pcl_executor_spin_once().
  - Assert object is marked stale.
```

### GREEN 9.11

Implement `on_tick(double dt)` calling `runtime_->tick(...)`.

### RED 9.12 — No ports after configure

```
///< REQ_TACTICAL_OBJECTS_034: No port creation after configure.
TEST(TacticalObjectsComponent, NoPortsAfterConfigure)
  - Configure + activate.
  - Assert port count matches expected total.
  - Run several service calls and evidence ingests.
  - Assert port count unchanged.
```

### GREEN 9.12

Already passes if all ports are created in `on_configure()`.

### RED 9.13 — Interest service handler

```
///< REQ_TACTICAL_OBJECTS_057: Interest registration and cancellation exposed by service boundary.
TEST(TacticalObjectsComponent, InterestServiceRoundTrip)
  - Configure + activate.
  - Encode an interest registration request with affiliation, geography, and time filters.
  - Invoke `svc_interest_` handler and assert response contains a stable interest identifier.
  - Invoke cancellation for that identifier and assert runtime interest registry no longer contains it.
```

### GREEN 9.13

Wire `svc_interest_` handler: decode register/cancel requests, call runtime interest APIs, and encode the result.

### REFACTOR 9

Clean up handler boilerplate with a `handleService(codec_fn, runtime_fn)` helper template.

---

## Phase 10 — RelationshipIndex

**Goal**: represent first-class relationships between tactical objects with confidence and provenance.

**Files created**:

- `include/relationship/RelationshipIndex.h`
- `src/relationship/RelationshipIndex.cpp`
- `tests/tactical_objects/Test_RelationshipIndex.cpp`

### RED 10.1 — Store generic relationship

```
///< REQ_TACTICAL_OBJECTS_049: Relationship records are stored as first-class linked entities.
TEST(RelationshipIndex, StoreAndRetrieveRelationship)
  - Create two objects.
  - Insert a relationship between them with a UUID, type, confidence, and source.
  - Assert the relationship is retrievable from both subject and object lookups.
```

### GREEN 10.1

Implement `RelationshipIndex` storage keyed by relationship UUID plus subject/object reverse indexes.

### RED 10.2 — Hierarchical relationship query

```
///< REQ_TACTICAL_OBJECTS_050: Hierarchical relationships are queryable.
TEST(RelationshipIndex, HierarchicalRelationshipQuery)
  - Insert member-of-unit and equipment-on-platform relationships.
  - Query parent and child links.
  - Assert the expected linked objects are returned.
```

### GREEN 10.2

Add hierarchical relationship filters and traversal helpers.

### RED 10.3 — Tactical relationship query

```
///< REQ_TACTICAL_OBJECTS_051: Tactical relationships are queryable.
TEST(RelationshipIndex, TacticalRelationshipQuery)
  - Insert escorting, engaging, and protecting relationships.
  - Query by tactical relationship type.
  - Assert only exact matches are returned.
```

### GREEN 10.3

Add tactical relationship filtering over the same relationship store.

### RED 10.4 — Confidence and provenance retention

```
///< REQ_TACTICAL_OBJECTS_052: Relationship confidence and provenance are retained.
TEST(RelationshipIndex, ConfidenceAndProvenanceRetained)
  - Insert one relationship with confidence = 0.65 and source metadata.
  - Retrieve the relationship.
  - Assert confidence and provenance values match exactly.
```

### GREEN 10.4

Persist confidence and provenance in relationship records and expose them through retrieval APIs.

### REFACTOR 10

Normalize relationship type helpers so `QueryEngine` and runtime can share the same predicates.

---

## Phase 11 — Interest and Temporal History

**Goal**: cover the remaining HLR areas for interest requirements and temporal query/history retention.

**Files created**:

- `include/interest/InterestManager.h`
- `src/interest/InterestManager.cpp`
- `include/history/TacticalHistory.h`
- `src/history/TacticalHistory.cpp`
- `tests/tactical_objects/Test_InterestManager.cpp`
- `tests/tactical_objects/Test_TacticalHistory.cpp`

### RED 11.1 — Register compound interest requirement

```
///< REQ_TACTICAL_OBJECTS_046: Compound interest requirement is accepted and persisted.
TEST(InterestManager, RegisterCompoundInterestRequirement)
  - Register an interest requirement covering affiliation, area, behavior, and time window.
  - Assert a stable interest ID is returned.
  - Assert stored criteria round-trip exactly.
```

### GREEN 11.1

Implement interest registration with a stable identifier and stored criteria model.

### RED 11.2 — Cancel, expire, and supersede interest

```
///< REQ_TACTICAL_OBJECTS_047: Active interests support cancellation, expiry, and supersession.
TEST(InterestManager, CancelExpireAndSupersedeInterest)
  - Register three interests.
  - Cancel one explicitly, advance time to expire one, and supersede one with a replacement.
  - Assert only the replacement remains active.
```

### GREEN 11.2

Implement lifecycle management for interest records.

### RED 11.3 — Temporal snapshot query

```
///< REQ_TACTICAL_OBJECTS_048: As-of query returns the correct historical object state.
TEST(TacticalHistory, QueryStateAtTime)
  - Create one object and record multiple timestamped updates.
  - Query state as-of each timestamp.
  - Assert the returned version matches the retained snapshot for that time.
```

### GREEN 11.3

Persist retained snapshots or deltas sufficient for deterministic as-of query.

### RED 11.4 — Temporal interval query

```
///< REQ_TACTICAL_OBJECTS_048: Interval query returns retained states across a time window.
TEST(TacticalHistory, QueryStatesOverInterval)
  - Record multiple updates over a retained interval.
  - Query for a window containing a subset of those updates.
  - Assert only the expected historical states are returned in order.
```

### GREEN 11.4

Add interval retrieval over retained history records.

### RED 11.5 — Requirement achievability for active finding

```
///< REQ_TACTICAL_OBJECTS_058: AOI-based tactical object requirement achievability is assessed.
TEST(TacticalObjectsRuntime, RequirementAchievabilityAssessment)
  - Construct runtime with a capability profile covering only selected evidence types.
  - Submit an AOI-based requirement to actively find hostile ground units within a time window.
  - Assert the achievability response reflects current capability, constraints, and expected evidence availability.
```

### GREEN 11.5

Implement achievability assessment for tactical object requirements using runtime capability, constraints, and evidence availability.

### RED 11.6 — Tactical object solution determination

```
///< REQ_TACTICAL_OBJECTS_059: Tactical object solution is determined for a requirement.
TEST(TacticalObjectsRuntime, TacticalObjectSolutionDetermination)
  - Submit a tactical object requirement with object type, AOI, and timeliness criteria.
  - Request the planned solution.
  - Assert the solution contains the intended object criteria, AOI, timing, and predicted quality fields.
```

### GREEN 11.6

Add an object-solution model that captures how the runtime intends to satisfy a tactical object requirement.

### RED 11.7 — Component-agnostic derived evidence requirement

```
///< REQ_TACTICAL_OBJECTS_060: Derived evidence requirement is expressed without naming another PRA component.
TEST(InterestManager, DerivedEvidenceRequirementIsComponentAgnostic)
  - Register an AOI-based active-finding requirement.
  - Retrieve the derived evidence requirement.
  - Assert it describes needed evidence and supporting information in tactical-object terms.
  - Assert it does not encode a named downstream PRA component.
```

### GREEN 11.7

Derive `Object_Solution_Evidence_Requirement` records in component-agnostic terms suitable for bridge-mediated fulfilment.

### RED 11.8 — Requirement-to-evidence-to-object traceability

```
///< REQ_TACTICAL_OBJECTS_061: Source requirement, derived evidence, observations, and correlated objects remain traceable.
TEST(TacticalObjectsRuntime, RequirementEvidenceObjectTraceability)
  - Submit an AOI requirement and capture the derived evidence requirement ID.
  - Submit observations associated with that derived evidence requirement.
  - Produce one correlated object.
  - Assert the correlated object traces back to the observations, derived evidence requirement, and source tactical object requirement.
```

### GREEN 11.8

Retain requirement and evidence lineage links alongside the existing observation-to-object lineage.

### RED 11.9 — Measurement criteria acceptance

```
///< REQ_TACTICAL_OBJECTS_062: Measurement criteria captured and applied during quality assessment.
TEST(TacticalObjectsRuntime, MeasurementCriteriaAcceptance)
  - Register a measurement criterion (e.g. location confidence >= 0.9) against an interest requirement.
  - Provide object data meeting and not meeting the criterion.
  - Verify the quality assessment uses the criterion to distinguish the two cases.
```

### GREEN 11.9

Implement measurement criteria storage and application within quality determination.

### RED 11.10 — Progress reporting against interest requirement

```
///< REQ_TACTICAL_OBJECTS_063: Progress is reported against an active Object_Interest_Requirement.
TEST(InterestManager, ProgressReportingAgainstRequirement)
  - Register an interest requirement.
  - Provide partial evidence satisfying some criteria.
  - Request progress report.
  - Assert it reflects partial fulfilment and remaining gaps.
```

### GREEN 11.10

Implement progress tracking and reporting within the interest management subsystem.

### RED 11.11 — Object probability density capture

```
///< REQ_TACTICAL_OBJECTS_064: Probability density data captured and stored for objects within locations.
TEST(TacticalObjectsRuntime, ObjectProbabilityDensityCapture)
  - Submit probability density data for an object type within a region.
  - Retrieve the stored density.
  - Assert the density values and region match the submitted data.
```

### GREEN 11.11

Implement probability density capture and storage.

### RED 11.12 — Capability progression prediction

```
///< REQ_TACTICAL_OBJECTS_065: Capability progression is predicted over time and with use.
TEST(TacticalObjectsRuntime, CapabilityProgressionPrediction)
  - Configure the runtime with known source availability.
  - Request a capability progression prediction over a time interval.
  - Assert the prediction reflects expected changes based on source availability.
```

### GREEN 11.12

Implement capability progression prediction taking account of source availability and usage patterns.

### REFACTOR 11

Share timestamp helpers and retention policy code between runtime tick handling and temporal query support.

---

## Summary: Test Count by Phase

| Phase | File | Tests |
|-------|------|-------|
| 0 | Test_Types.cpp | 3 |
| 1 | Test_ObjectStore.cpp | 6 |
| 2 | Test_MilClassEngine.cpp | 3 |
| 3 | Test_SpatialIndex.cpp | 4 |
| 4 | Test_ZoneEngine.cpp | 9 |
| 5 | Test_CorrelationEngine.cpp | 9 |
| 6 | Test_QueryEngine.cpp | 5 |
| 7 | Test_TacticalObjectsRuntime.cpp | 15 |
| 8 | Test_TacticalObjectsCodec.cpp | 5 |
| 9 | Test_TacticalObjectsComponent.cpp | 13 |
| 10 | Test_RelationshipIndex.cpp | 4 |
| 11 | Test_InterestManager.cpp + Test_TacticalHistory.cpp + Test_TacticalObjectsRuntime.cpp | 12 |
| **Total** | | **88** |

## Entity Creation Path Summary

| Path | Entry Point | Internal Flow | Provenance |
|------|------------|---------------|------------|
| **Evidence** | subscriber `observation_ingress` | Codec → `upsertObservationBatch` → `CorrelationEngine` → correlate/create/merge → ObjectStore | `correlated` with full lineage |
| **Direct** | service `create_object` | Codec → `createObject` → ObjectStore + SpatialIndex + MilClass | `direct` with caller attribution |

Both paths share:

- the same `ObjectStore`
- the same `SpatialIndex` registration
- the same `MilClassComponent` storage
- the same `QueryEngine` for retrieval
- the same `EventBus` notifications
- the same `ZoneEngine` relationship evaluation

The only difference is that evidence-path objects go through correlation gating, scoring, merge/split, and confidence aggregation, while direct-path objects are stored as-provided with caller-attributed provenance.
