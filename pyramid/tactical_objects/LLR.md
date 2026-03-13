# Tactical Objects Low-Level Requirements

Low-level requirements for `pyramid/tactical_objects`, derived from the high-level requirements in `REQUIREMENTS.md` and the design decisions in `DESIGN.md`.

Each LLR maps to at least one unit or component test. Tests use requirement tags in the form `///< REQ_TACTICAL_OBJECTS_NNN: ...`.

See `TDD_PLAN.md` for the full red/green implementation plan tied to these requirements.

---

## 1. Object Identity and Storage

### REQ_TACTICAL_OBJECTS_001 - Stable UUID Allocation
`ObjectStore` shall allocate a `pyramid::core::uuid::UUID` for every new object, observation, track, relationship, and zone.

**Traces**: TOBJ.003

**Verification**: `Test_ObjectStore.cpp` creates one instance of each record type and asserts that each ID is non-null and unique.

### REQ_TACTICAL_OBJECTS_002 - Dense and Sparse Lookup Consistency
`ObjectStore` shall maintain dense storage for iteration and sparse lookup by UUID for O(1) expected access.

**Traces**: TOBJ.038

**Verification**: `Test_ObjectStore.cpp` inserts `N` objects, retrieves each by UUID, and verifies dense iteration count matches sparse lookup count.

### REQ_TACTICAL_OBJECTS_003 - Optional Component Tables
`ObjectStore` shall permit an entity to exist without every component table being populated.

**Traces**: TOBJ.039

**Verification**: `Test_ObjectStore.cpp` inserts an object with identity only and confirms kinematics and military classification lookups report absent rather than default-populated.

### REQ_TACTICAL_OBJECTS_004 - Version Increment on Mutation
`ObjectStore` shall increment an object's version each time any authoritative component for that object is modified.

**Traces**: TOBJ.002

**Verification**: `Test_ObjectStore.cpp` updates the same object twice and verifies strictly increasing version numbers.

---

## 2. Observation Handling and Lineage

### REQ_TACTICAL_OBJECTS_005 - Observation Batch Ingest
`TacticalObjectsRuntime` shall accept a batch of observations and process them in input order.

**Traces**: TOBJ.018, TOBJ.042

**Verification**: `Test_TacticalObjectsRuntime.cpp` submits an ordered batch and verifies emitted events preserve processing order.

### REQ_TACTICAL_OBJECTS_006 - Source Reference Association
Each fused object shall retain all distinct source references that contributed to it.

**Traces**: TOBJ.004

**Verification**: `Test_FusionEngine.cpp` fuses two compatible observations from different sources and verifies both source references are retained.

### REQ_TACTICAL_OBJECTS_007 - Lineage Retention
Each fused object shall retain lineage entries referencing contributing observations and tracks.

**Traces**: TOBJ.020

**Verification**: `Test_FusionEngine.cpp` ingests observations, performs a fusion, and verifies lineage contains the expected observation and track IDs.

---

## 3. Fusion

### REQ_TACTICAL_OBJECTS_008 - Candidate Prefilter by Spatial Index
`FusionEngine` shall obtain correlation candidates from `SpatialIndex` before exact scoring.

**Traces**: TOBJ.040

**Verification**: `Test_FusionEngine.cpp` loads distant objects and verifies only local candidates are scored.

### REQ_TACTICAL_OBJECTS_009 - Deterministic Correlation Score
`FusionEngine` shall calculate a deterministic correlation score from configured gates and fixed field weights.

**Traces**: TOBJ.021

**Verification**: `Test_FusionEngine.cpp` evaluates the same input twice and verifies identical score and decision.

### REQ_TACTICAL_OBJECTS_010 - Create New Object on Low Score
If all candidate scores are below `create_threshold`, `FusionEngine` shall create a new track and a new fused object.

**Traces**: TOBJ.021, TOBJ.022

**Verification**: `Test_FusionEngine.cpp` provides a spatially distant observation and verifies object count increases by one.

### REQ_TACTICAL_OBJECTS_011 - Merge on High Score
If the best candidate score is >= `merge_threshold`, `FusionEngine` shall attach the observation to the existing fused object.

**Traces**: TOBJ.022

**Verification**: `Test_FusionEngine.cpp` provides two compatible observations and verifies a single fused object remains.

### REQ_TACTICAL_OBJECTS_012 - Split on Sustained Incompatibility
If incompatible evidence exceeds the configured split threshold for the configured number of updates, `FusionEngine` shall split the contributing track into a new fused object.

**Traces**: TOBJ.023

**Verification**: `Test_FusionEngine.cpp` injects repeated incompatible updates and verifies the original fused object is split.

### REQ_TACTICAL_OBJECTS_013 - Confidence Aggregation
`FusionEngine` shall update fused object confidence using weighted contribution from source confidence and recency.

**Traces**: TOBJ.019

**Verification**: `Test_FusionEngine.cpp` fuses high-confidence and low-confidence observations and verifies the aggregate confidence lies within configured bounds.

---

## 4. Military Classification (2525B Data)

### REQ_TACTICAL_OBJECTS_014 - Military Classification Fields
`MilClassEngine` shall store: battle dimension, affiliation, role (function ID), status, echelon, mobility, and boolean flags for headquarters, task force, feint/dummy, and installation.

**Traces**: TOBJ.025, TOBJ.026, TOBJ.027, TOBJ.028, TOBJ.029, TOBJ.030, TOBJ.031

**Verification**: `Test_MilClassEngine.cpp` populates each field and verifies it is retrievable from the stored profile.

### REQ_TACTICAL_OBJECTS_015 - Preserve Source SIDC
`MilClassEngine` shall preserve a source-provided SIDC (symbol identification coding) when one is present.

**Traces**: TOBJ.032

**Verification**: `Test_MilClassEngine.cpp` ingests an object with a source SIDC and verifies the code is retained alongside normalized fields.

### REQ_TACTICAL_OBJECTS_016 - Derived Symbol Key
`MilClassEngine` shall generate a deterministic symbol key from stored classification fields.

**Traces**: TOBJ.032

**Verification**: `Test_MilClassEngine.cpp` submits the same fields twice and verifies the derived key is identical.

---

## 5. Geography and Zones

### REQ_TACTICAL_OBJECTS_017 - Supported Geometry Types
`ZoneEngine` shall support point, polyline, polygon, circle, ellipse, and corridor geometry definitions.

**Traces**: TOBJ.033

**Verification**: `Test_ZoneEngine.cpp` inserts one zone of each supported geometry type and verifies storage and retrieval succeed.

### REQ_TACTICAL_OBJECTS_018 - Bounding Box Cache
`ZoneEngine` shall compute and cache a bounding box for every non-point geometry.

**Traces**: TOBJ.040

**Verification**: `Test_ZoneEngine.cpp` inserts a polygon and verifies the cached bounds contain all vertices.

### REQ_TACTICAL_OBJECTS_019 - Point in Zone
`ZoneEngine` shall determine whether a point object is inside or outside a polygon or circle zone.

**Traces**: TOBJ.036

**Verification**: `Test_ZoneEngine.cpp` evaluates inside and outside cases for both polygon and circle zones.

### REQ_TACTICAL_OBJECTS_020 - Boundary Transition Detection
`ZoneEngine` shall emit enter and leave transitions by comparing current zone relationship state with prior state.

**Traces**: TOBJ.036

**Verification**: `Test_ZoneEngine.cpp` moves an object across a zone boundary and verifies `enter` then `leave` transitions.

### REQ_TACTICAL_OBJECTS_021 - Time-Gated Zone Activity
Inactive zones shall be excluded from zone relationship results outside their validity interval.

**Traces**: TOBJ.037

**Verification**: `Test_ZoneEngine.cpp` queries an expired zone and verifies no active relationship is reported.

---

## 6. Indexing and Query

### REQ_TACTICAL_OBJECTS_022 - Spatial Candidate Retrieval
`SpatialIndex` shall return all objects whose cached bounds overlap the queried region's bounds.

**Traces**: TOBJ.040

**Verification**: `Test_SpatialIndex.cpp` loads objects in multiple cells and verifies only overlapping candidates are returned.

### REQ_TACTICAL_OBJECTS_023 - Exact Post-Filter
`QueryEngine` shall apply exact spatial checks after index prefiltering.

**Traces**: TOBJ.009

**Verification**: `Test_QueryEngine.cpp` uses overlapping bounding boxes with exact non-overlap and verifies false positives are removed.

### REQ_TACTICAL_OBJECTS_024 - Compound Predicate Query
`QueryEngine` shall support compound predicates across type, affiliation, zone relationship, and freshness.

**Traces**: TOBJ.007

**Verification**: `Test_QueryEngine.cpp` inserts mixed objects and verifies only the exact matching subset is returned.

### REQ_TACTICAL_OBJECTS_025 - External Identifier Query
`QueryEngine` shall support exact lookup by source system and source entity identifier.

**Traces**: TOBJ.008

**Verification**: `Test_QueryEngine.cpp` inserts source references and verifies the requested external identifier resolves to the correct object.

---

## 7. Direct Entity Management

### REQ_TACTICAL_OBJECTS_026 - Direct Create
`TacticalObjectsRuntime::createObject()` shall store an object with all provided components and record `direct` provenance without invoking fusion.

**Traces**: TOBJ.001, TOBJ.002, TOBJ.003

**Verification**: `Test_TacticalObjectsRuntime.cpp` creates an object via `createObject()`, retrieves it, and verifies all fields and provenance marker.

### REQ_TACTICAL_OBJECTS_027 - Direct Update
`TacticalObjectsRuntime::updateObject()` shall apply partial field updates to an existing object, bump its version, and re-index spatial if position changed.

**Traces**: TOBJ.002, TOBJ.041

**Verification**: `Test_TacticalObjectsRuntime.cpp` updates an object field and verifies the change, version increment, and spatial consistency.

### REQ_TACTICAL_OBJECTS_028 - Direct Delete
`TacticalObjectsRuntime::deleteObject()` shall remove the object from all component tables and indexes.

**Traces**: TOBJ.001

**Verification**: `Test_TacticalObjectsRuntime.cpp` deletes an object and verifies it is absent from store and spatial index.

### REQ_TACTICAL_OBJECTS_029 - Both Paths Queryable Together
Objects created via evidence (fusion) and via direct CRUD shall be queryable through the same `QueryEngine`.

**Traces**: TOBJ.007, TOBJ.008

**Verification**: `Test_TacticalObjectsRuntime.cpp` creates objects via both paths and verifies a single query returns both.

---

## 8. Runtime Behavior

### REQ_TACTICAL_OBJECTS_030 - Runtime Facade Stability
`TacticalObjectsRuntime` shall expose a single facade API that can be exercised without constructing a PCL executor.

**Traces**: Design D7

**Verification**: `Test_TacticalObjectsRuntime.cpp` creates the runtime directly and performs ingest, query, and zone update operations.

### REQ_TACTICAL_OBJECTS_031 - Stale Object Expiry
`TacticalObjectsRuntime::tick()` shall mark or retire objects whose freshest observation exceeds the configured stale timeout.

**Traces**: TOBJ.017

**Verification**: `Test_TacticalObjectsRuntime.cpp` advances time past the configured timeout and verifies the object becomes stale or retired.

### REQ_TACTICAL_OBJECTS_032 - Capability Snapshot
`TacticalObjectsRuntime` shall produce a capability snapshot containing counts, supported geometry types, and fusion configuration state.

**Traces**: TOBJ.043

**Verification**: `Test_TacticalObjectsRuntime.cpp` initializes the runtime and verifies the capability snapshot contains configured values.

---

## 9. PCL Integration

### REQ_TACTICAL_OBJECTS_033 - Ports Created During Configure
`TacticalObjectsComponent` shall create all publishers, subscribers, and services during `on_configure()`.

**Traces**: Design D7, PCL lifecycle

**Verification**: `Test_TacticalObjectsComponent.cpp` calls `configure()` and verifies all expected ports are created successfully.

### REQ_TACTICAL_OBJECTS_034 - No Port Creation After Configure
`TacticalObjectsComponent` shall not attempt to add ports after `on_configure()` has completed.

**Traces**: PCL lifecycle

**Verification**: `Test_TacticalObjectsComponent.cpp` verifies runtime operations after configure do not allocate additional ports.

### REQ_TACTICAL_OBJECTS_035 - Evidence Ingress Through Subscriber
Observation ingress shall be handled through a PCL subscriber callback or a directly invoked equivalent test seam.

**Traces**: TOBJ.018

**Verification**: `Test_TacticalObjectsComponent.cpp` dispatches an observation message into the subscriber callback and verifies runtime state changes.

### REQ_TACTICAL_OBJECTS_036 - Direct CRUD Through Service Handlers
`create_object`, `update_object`, and `delete_object` shall be exposed as PCL service handlers that decode requests, invoke runtime methods, and encode responses.

**Traces**: TOBJ.001, TOBJ.002

**Verification**: `Test_TacticalObjectsComponent.cpp` invokes each service handler and verifies the expected runtime state changes.

### REQ_TACTICAL_OBJECTS_037 - Query Through Service Handler
Object query shall be exposed through a PCL service handler that decodes the request, invokes runtime query, and encodes the response.

**Traces**: TOBJ.007, TOBJ.008

**Verification**: `Test_TacticalObjectsComponent.cpp` invokes the service handler and verifies the serialized response contains the expected object.

### REQ_TACTICAL_OBJECTS_038 - Tick-Driven Housekeeping
`TacticalObjectsComponent::on_tick()` shall delegate housekeeping work to `runtime_->tick(...)`.

**Traces**: TOBJ.041

**Verification**: `Test_TacticalObjectsComponent.cpp` activates the component, runs one tick, and verifies stale handling or capability publication occurs.

---

## 10. pyramid::core Integration

### REQ_TACTICAL_OBJECTS_039 - Domain Events on EventBus
The runtime shall publish internal domain events on `pyramid::core::event::EventBus` for object created, object updated, object deleted, object fused, object split, zone entered, and zone left.

**Traces**: TOBJ.045

**Verification**: `Test_TacticalObjectsRuntime.cpp` subscribes to the event bus and verifies each event is emitted for the corresponding operation.

### REQ_TACTICAL_OBJECTS_040 - Component Logging
The runtime shall write informational and warning diagnostics to `pyramid::core::logging::Logger`.

**Traces**: TOBJ.044

**Verification**: `Test_TacticalObjectsRuntime.cpp` injects a logger, triggers a notable operation, and verifies log history contains the expected message level.

### REQ_TACTICAL_OBJECTS_041 - UUID Serialization Boundary
The codec shall serialize and deserialize internal `UUID` values in a stable canonical string form.

**Traces**: TOBJ.003

**Verification**: `Test_TacticalObjectsComponent.cpp` round-trips a UUID through the codec and verifies equality.

---

## 11. Performance and Scale

### REQ_TACTICAL_OBJECTS_042 - Thousand-Entity Query Baseline
The query path shall return region results over 10,000 point objects in bounded time without full-store exact geometry evaluation.

**Traces**: TOBJ.038, TOBJ.040

**Verification**: `Test_SpatialIndex.cpp` loads 10,000 objects, performs a region query, and asserts candidate count is significantly lower than total population for a local query.

### REQ_TACTICAL_OBJECTS_043 - Incremental Index Maintenance
Updating one object's position shall only update that object's spatial index membership.

**Traces**: TOBJ.041

**Verification**: `Test_SpatialIndex.cpp` moves one indexed object and verifies unaffected objects retain their original index placement.

### REQ_TACTICAL_OBJECTS_044 - Batch Fusion Determinism
Processing the same observation batch in the same order shall produce the same fused object graph and versions.

**Traces**: TOBJ.022, TOBJ.024

**Verification**: `Test_TacticalObjectsRuntime.cpp` processes identical batches in two fresh runtimes and compares resulting object IDs, lineage shape, and versions.

### REQ_TACTICAL_OBJECTS_045 - Memory-Reasonable Sparse Storage
An object with no zone, behavior, or military classification components shall not allocate entries in those component tables.

**Traces**: TOBJ.039

**Verification**: `Test_ObjectStore.cpp` inserts sparse objects and verifies optional component table counts remain unchanged.

---

## Traceability Summary

| LLR | TOBJ | Test File |
|-----|------|-----------|
| 001 | 003 | Test_ObjectStore |
| 002 | 038 | Test_ObjectStore |
| 003 | 039 | Test_ObjectStore |
| 004 | 002 | Test_ObjectStore |
| 005 | 018, 042 | Test_TacticalObjectsRuntime |
| 006 | 004 | Test_FusionEngine |
| 007 | 020 | Test_FusionEngine |
| 008 | 040 | Test_FusionEngine |
| 009 | 021 | Test_FusionEngine |
| 010 | 021, 022 | Test_FusionEngine |
| 011 | 022 | Test_FusionEngine |
| 012 | 023 | Test_FusionEngine |
| 013 | 019 | Test_FusionEngine |
| 014 | 025-031 | Test_MilClassEngine |
| 015 | 032 | Test_MilClassEngine |
| 016 | 032 | Test_MilClassEngine |
| 017 | 033 | Test_ZoneEngine |
| 018 | 040 | Test_ZoneEngine |
| 019 | 036 | Test_ZoneEngine |
| 020 | 036 | Test_ZoneEngine |
| 021 | 037 | Test_ZoneEngine |
| 022 | 040 | Test_SpatialIndex |
| 023 | 009 | Test_QueryEngine |
| 024 | 007 | Test_QueryEngine |
| 025 | 008 | Test_QueryEngine |
| 026 | 001-003 | Test_TacticalObjectsRuntime |
| 027 | 002, 041 | Test_TacticalObjectsRuntime |
| 028 | 001 | Test_TacticalObjectsRuntime |
| 029 | 007, 008 | Test_TacticalObjectsRuntime |
| 030 | D7 | Test_TacticalObjectsRuntime |
| 031 | 017 | Test_TacticalObjectsRuntime |
| 032 | 043 | Test_TacticalObjectsRuntime |
| 033 | PCL | Test_TacticalObjectsComponent |
| 034 | PCL | Test_TacticalObjectsComponent |
| 035 | 018 | Test_TacticalObjectsComponent |
| 036 | 001, 002 | Test_TacticalObjectsComponent |
| 037 | 007, 008 | Test_TacticalObjectsComponent |
| 038 | 041 | Test_TacticalObjectsComponent |
| 039 | 045 | Test_TacticalObjectsRuntime |
| 040 | 044 | Test_TacticalObjectsRuntime |
| 041 | 003 | Test_TacticalObjectsComponent |
| 042 | 038, 040 | Test_SpatialIndex |
| 043 | 041 | Test_SpatialIndex |
| 044 | 022, 024 | Test_TacticalObjectsRuntime |
| 045 | 039 | Test_ObjectStore |
