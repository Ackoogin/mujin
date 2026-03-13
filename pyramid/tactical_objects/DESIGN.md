# Tactical Objects Concrete Design

This document defines a concrete implementation design for `pyramid/tactical_objects` that is detailed enough to drive TDD.

It assumes:

- `PCL` is the lifecycle and execution boundary
- `pyramid::core` is the required C++ support library
- the tactical objects business logic is implemented as a C++14 library behind a `pcl::Component`
- the implementation must support MIL-STD-2525B semantics, geographic zones, thousands of entities, and entity fusion

## 1. Chosen Architecture

The implementation shall be split into two layers:

- `TacticalObjectsComponent`: a `pcl::Component` subclass that owns ports, lifecycle, parameter handling, and serialization boundaries
- `TacticalObjectsRuntime`: a pure C++ domain runtime that owns the object store, indexes, fusion logic, zone logic, and query logic

`TacticalObjectsComponent` is intentionally thin. All tactical reasoning shall live in `TacticalObjectsRuntime` so that unit tests can exercise domain logic without standing up a `pcl::Executor`.

## 2. Required Platform Use

### 2.1 PCL Usage

The component shall use:

- `pcl::Component` as the authoring interface
- `pcl::Executor` as the runtime executor
- subscriber callbacks for ingress of observations and zone updates
- service handlers for query and CRUD style control-plane requests
- `on_tick()` for bounded housekeeping work such as stale-object expiry, deferred index maintenance, and periodic capability publication

All ports shall be created in `on_configure()` only.

All state mutation shall happen on the single PCL executor thread.

External producers shall use `pcl_executor_post_incoming()` via the transport adapter rather than mutating the runtime directly.

### 2.2 pyramid::core Usage

The runtime shall use:

- `pyramid::core::uuid::UUID` and `UUIDHelper` for internal object, observation, track, zone, and relationship identifiers
- `pyramid::core::event::EventBus` for internal domain events between subsystems
- `pyramid::core::logging::Logger` for internal diagnostic and audit logging

The runtime shall not depend on `pyramid::core::job::JobSystem` for core business logic because PCL already provides a deterministic single-thread execution model. If future asynchronous adapters are needed, they must post results back into the component through PCL ingress.

## 3. High-Level Structure

```text
PCL transport -> TacticalObjectsComponent -> TacticalObjectsRuntime
                                         -> Codec
                                         -> Capability publisher

TacticalObjectsRuntime
  -> ObjectStore
  -> SpatialIndex
  -> RelationshipIndex
  -> FusionEngine
  -> ZoneEngine
  -> MilClassEngine
  -> QueryEngine
  -> InterestManager
  -> TacticalHistory
  -> EventBus
  -> Logger
```

## 4. Concrete Class Design

## 4.1 Container Boundary

### `TacticalObjectsComponent`

Purpose:

- expose PCL lifecycle
- create ports
- decode inbound messages
- route requests into `TacticalObjectsRuntime`
- publish result events and capability snapshots

Primary members:

- `std::shared_ptr<TacticalObjectsRuntime> runtime_`
- `std::shared_ptr<TacticalObjectsCodec> codec_`
- `pcl_port_t* pub_events_`
- `pcl_port_t* pub_capability_`
- `pcl_port_t* sub_observation_ingress_`
- `pcl_port_t* sub_zone_ingress_`
- `pcl_port_t* svc_create_object_`
- `pcl_port_t* svc_update_object_`
- `pcl_port_t* svc_delete_object_`
- `pcl_port_t* svc_query_`
- `pcl_port_t* svc_get_object_`
- `pcl_port_t* svc_upsert_zone_`
- `pcl_port_t* svc_remove_zone_`
- `pcl_port_t* svc_interest_`

Chosen PCL topics and services:

Subscribers (evidence ingress path -- entities created via fusion):

- subscriber `tactical_objects.observation_ingress` type `TacticalObservationEnvelope`
- subscriber `tactical_objects.zone_ingress` type `TacticalZoneEnvelope`

Publishers:

- publisher `tactical_objects.events` type `TacticalObjectEvent`
- publisher `tactical_objects.capability` type `TacticalObjectsCapability`

Provided services (direct CRUD -- entities created/modified without fusion):

- service `tactical_objects.create_object` type `TacticalObjectCreate`
- service `tactical_objects.update_object` type `TacticalObjectUpdate`
- service `tactical_objects.delete_object` type `TacticalObjectDelete`

Provided services (query):

- service `tactical_objects.query` type `TacticalQuery`
- service `tactical_objects.get_object` type `TacticalObjectGet`

Provided services (zone management):

- service `tactical_objects.upsert_zone` type `TacticalZoneUpsert`
- service `tactical_objects.remove_zone` type `TacticalZoneRemove`

Provided services (interest):

- service `tactical_objects.interest` type `TacticalInterestRequest`

The component therefore supports two entity creation paths:

1. **Evidence path**: observations arrive on the subscriber, pass through `FusionEngine`, and produce fused objects automatically.
2. **Direct path**: a caller invokes `create_object` / `update_object` / `delete_object` services to manage entities without fusion. These objects are stored with a `direct` provenance marker and bypass correlation and fusion scoring.

### `TacticalObjectsCodec`

Purpose:

- isolate message serialization from runtime logic
- allow unit tests to bypass serialization

V1 concrete choice:

- use `nlohmann::json` from `pyramid/core/external/json.hpp` for control-plane requests and responses
- use the same codec for observation ingress in the first implementation to reduce integration risk
- support batching at the message level so high-rate sources can submit many observations in one ingress message

Rationale:

- it is already available in the repository
- it keeps the first implementation simple enough for TDD
- batching offsets some per-message overhead

If profiling later shows codec cost dominates, the codec can be replaced without changing runtime APIs.

## 4.2 Domain Runtime

### `TacticalObjectsRuntime`

Purpose:

- composition root for all domain subsystems
- single public API used by the component and direct unit tests

Primary methods:

Evidence path (fusion-based entity creation):

- `upsertObservationBatch(const ObservationBatch&)`

Direct path (explicit entity CRUD without fusion):

- `createObject(const ObjectDefinition&)`
- `updateObject(const UUID&, const ObjectUpdate&)`
- `deleteObject(const UUID&)`

Zone management:

- `upsertZone(const ZoneDefinition&)`
- `removeZone(const UUID&)`

Query:

- `getObject(const UUID&) const`
- `query(const QueryRequest&) const`

Other:

- `registerInterest(const InterestRequest&)`
- `cancelInterest(const UUID&)`
- `tick(const TickContext&)`
- `getCapability() const`

Primary collaborators:

- `ObjectStore`
- `SpatialIndex`
- `RelationshipIndex`
- `FusionEngine`
- `ZoneEngine`
- `MilClassEngine`
- `QueryEngine`
- `InterestManager`
- `TacticalHistory`
- `pyramid::core::event::EventBus`
- `pyramid::core::logging::Logger`

### `ObjectStore`

Purpose:

- authoritative in-memory storage of objects, observations, tracks, zones, and relationships
- sparse ECS-style component storage

Storage model:

- one dense vector of entity records per entity family
- sparse maps from `UUID` to dense index
- separate component tables for optional facets

Component tables:

- `IdentityComponent`
- `ClassificationComponent`
- `AffiliationComponent`
- `KinematicsComponent`
- `BehaviorComponent`
- `GeometryComponent`
- `ZoneComponent`
- `MilClassComponent`
- `FusionComponent`
- `QualityComponent`
- `LifecycleComponent`

Chosen implementation detail:

- use a simple sparse-set style layout with `std::vector<size_t>` dense arrays and `std::unordered_map<UUIDKey, size_t>` sparse lookup
- define `UUIDKey` as a thin hashable wrapper around `pyramid::core::uuid::UUID`

This gives ECS-like locality without introducing a third-party ECS library.

### `SpatialIndex`

Purpose:

- accelerate region, proximity, and zone-membership prefiltering

Chosen implementation:

- maintain a grid index over WGS84 latitude and longitude using configurable cell size
- index points by position cell
- index non-point geometries by bounding box cell coverage
- run exact geometry checks after coarse candidate retrieval

Rationale:

- it is implementable with no new dependencies
- it is deterministic
- it is sufficient for thousands of entities
- it is easy to unit test

### `RelationshipIndex`

Purpose:

- store first-class relationship records between tactical objects
- support hierarchical, organizational, and tactical relationship queries

Chosen implementation:

- one relationship record per edge with its own UUID
- reverse indexes by subject UUID and object UUID
- store relationship type, confidence, provenance, and validity timestamps

### `ZoneEngine`

Purpose:

- maintain geographic zones
- compute object/zone relationships
- emit enter, leave, and intersection state changes

Chosen geometry model:

- store canonical coordinates in WGS84 latitude, longitude, altitude
- support point, polyline, polygon, circle, ellipse, and corridor
- represent every shape with a canonical type plus cached bounding box

Exact spatial checks:

- point-in-polygon for polygons
- geodesic or haversine distance for point-circle and nearest checks
- corridor check as distance to polyline <= corridor width
- boundary crossing determined by previous and current relationship states

### `FusionEngine`

Purpose:

- correlate incoming observations against existing tracks and fused objects
- create new tracks when no existing candidate passes gating
- merge tracks into fused objects
- split fused objects when evidence diverges

Chosen correlation strategy:

- deterministic score-based gating
- candidate set generated by `SpatialIndex`
- scoring dimensions:
  - source continuity
  - external identifier match
  - spatial distance gate
  - time gap gate
  - classification compatibility
  - affiliation compatibility

Score outcome:

- score >= merge threshold: same fused object candidate
- score between create and merge thresholds: attach to existing track but do not fuse identity-critical fields
- score < create threshold: create new track and new fused object

Split rule:

- if a fused object accumulates mutually incompatible observations beyond configured thresholds for `N` consecutive updates, split the lowest-confidence contributing track into a new fused object

### `MilClassEngine`

Purpose:

- store and normalize the military classification fields that drive MIL-STD-2525B
- preserve source-provided SIDCs when present

Stored fields per object:

- battle dimension (ground, air, sea surface, subsurface, space, SOF)
- affiliation (friendly, hostile, neutral, unknown, assumed friend, suspect, joker, faker, pending)
- role / function identifier (infantry, armor, artillery, rotary wing, EW, logistics, C2, medical, etc.)
- status (present, anticipated, planned, known)
- echelon (team through army group)
- mobility (wheeled, tracked, towed, rail, over-snow, sled, barge, amphibious)
- flags: headquarters, task force, feint/dummy, installation

Output:

- `MilClassProfile` containing all stored fields
- optional preserved source SIDC
- deterministic derived symbol key for rendering or export

### `QueryEngine`

Purpose:

- evaluate exact queries against store and indexes

Supported predicates:

- by internal UUID
- by external source reference
- by object type
- by classification
- by affiliation
- by military classification fields (battle dimension, affiliation, role, echelon)
- by relationship
- by zone relationship
- by quality thresholds
- by time freshness
- by historical time point or retained interval
- by geographic region

### `InterestManager`

Purpose:

- store active interest requirements
- support registration, cancellation, expiry, and supersession

Chosen implementation:

- stable UUID per interest requirement
- index active interests by type, affiliation, geography, and expiry time
- runtime evaluates matching interests against object and zone events

### `TacticalHistory`

Purpose:

- retain sufficient object snapshots or deltas for temporal query
- enforce configured history retention boundaries

Chosen implementation:

- append-only per-object history records with timestamps and version numbers
- bounded retention controlled by `history_retention_ms`
- query helpers for as-of and interval retrieval

## 5. Canonical Data Model

## 5.1 Identity

Every persistent record shall have:

- internal `UUID`
- `created_at`
- `updated_at`
- `source_refs`
- `version`

### `SourceRef`

Fields:

- `source_system`
- `source_entity_id`
- `source_track_id`
- `first_seen`
- `last_seen`

## 5.2 Observation

Fields:

- `observation_id`
- `received_at`
- `observed_at`
- `source_ref`
- `object_hint_type`
- `position`
- `velocity`
- `classification_hint`
- `affiliation_hint`
- `confidence`
- `uncertainty_radius_m`
- `source_sidc`
- `lineage_parent_ids`

## 5.3 Fused Object

Fields:

- `object_id`
- `object_type`
- `identity`
- `classification`
- `affiliation`
- `kinematics`
- `behavior`
- `geometry`
- `mil_class`
- `quality`
- `lineage`
- `status`

## 5.4 Zone

Fields:

- `zone_id`
- `zone_type`
- `geometry`
- `active_from`
- `active_until`
- `priority`
- `owner`
- `semantics`

## 6. Lifecycle Behavior

### `on_configure()`

Responsibilities:

- load parameters
- construct `runtime_`
- construct `codec_`
- create all PCL ports
- configure tick rate

Parameters:

- `tick_rate_hz`
- `stale_object_timeout_ms`
- `history_retention_ms`
- `spatial_grid_cell_deg`
- `fusion.merge_threshold`
- `fusion.create_threshold`
- `fusion.split_incompatibility_count`

### `on_activate()`

Responsibilities:

- publish initial capability state
- mark runtime active

### subscriber callback for observation ingress

Responsibilities:

- decode `ObservationBatch`
- call `runtime_->upsertObservationBatch(...)`
- publish emitted domain events

### subscriber callback for zone ingress

Responsibilities:

- decode zone create or update request
- call `runtime_->upsertZone(...)`
- publish emitted domain events

### service handler for query

Responsibilities:

- decode `QueryRequest`
- call `runtime_->query(...)`
- encode `QueryResponse`

### `on_tick(double dt)`

Responsibilities:

- expire stale observations and objects
- update zone relationship transitions if needed
- publish capability snapshot at configured cadence
- flush any deferred event publications

### `on_cleanup()`

Responsibilities:

- destroy runtime-owned state
- clear cached publications

## 7. Proposed Source Layout

```text
pyramid/tactical_objects/
  CMakeLists.txt
  HLR.md
  LLR.md
  DESIGN.md
  ARCHITECTURE.md
  TDD_PLAN.md
  include/
    TacticalObjectsComponent.h
    TacticalObjectsRuntime.h
    TacticalObjectsTypes.h
    TacticalObjectsCodec.h
    store/ObjectStore.h
    store/ObjectComponents.h
    spatial/SpatialIndex.h
    relationship/RelationshipIndex.h
    fusion/FusionEngine.h
    milclass/MilClassEngine.h
    zone/ZoneEngine.h
    query/QueryEngine.h
    interest/InterestManager.h
    history/TacticalHistory.h
  src/
    TacticalObjectsComponent.cpp
    TacticalObjectsRuntime.cpp
    TacticalObjectsCodec.cpp
    store/ObjectStore.cpp
    spatial/SpatialIndex.cpp
    relationship/RelationshipIndex.cpp
    fusion/FusionEngine.cpp
    milclass/MilClassEngine.cpp
    zone/ZoneEngine.cpp
    query/QueryEngine.cpp
    interest/InterestManager.cpp
    history/TacticalHistory.cpp
```

Test layout:

```text
tests/tactical_objects/
  Test_ObjectStore.cpp
  Test_SpatialIndex.cpp
  Test_RelationshipIndex.cpp
  Test_FusionEngine.cpp
  Test_ZoneEngine.cpp
  Test_MilClassEngine.cpp
  Test_QueryEngine.cpp
  Test_InterestManager.cpp
  Test_TacticalHistory.cpp
  Test_TacticalObjectsRuntime.cpp
  Test_TacticalObjectsCodec.cpp
  Test_TacticalObjectsComponent.cpp
```

## 8. TDD Strategy

See `TDD_PLAN.md` for the full red/green/refactor plan.

See `LLR.md` for the 61 low-level requirements with traceability and verification notes.

Implementation order:

1. types and UUID key support
2. `ObjectStore`
3. `SpatialIndex`
4. `ZoneEngine`
5. `MilClassEngine`
6. `FusionEngine`
7. `QueryEngine`
8. `TacticalObjectsRuntime`
9. `TacticalObjectsComponent`

Rules:

- no production class is added without a failing test
- each LLR maps to at least one unit or component test
- tests shall use requirement tags in the form `///< REQ_TACTICAL_OBJECTS_NNN: ...`

## 9. First Implementation Cut

The first implementation cut should include:

- point objects
- polygon and circle zones
- exact affiliation and source reference queries
- deterministic score-based fusion
- lineage retention
- military classification field storage (battle dimension, affiliation, role, status, echelon, flags, mobility)
- relationship storage with hierarchical and tactical predicates
- interest registration, cancellation, expiry, and supersession
- retained temporal as-of and interval query support
- behavior estimation and operational state fields
- PCL subscriber and service integration

The first implementation cut may defer:

- optimized ellipse and corridor acceleration so long as correctness is preserved
- relationship-query indexing optimizations beyond the initial reverse indexes
- history compaction optimizations beyond a bounded append-only baseline

## 10. Recommended First Test Sequence

1. `Test_ObjectStore.cpp`
2. `Test_MilClassEngine.cpp`
3. `Test_SpatialIndex.cpp`
4. `Test_ZoneEngine.cpp`
5. `Test_RelationshipIndex.cpp`
6. `Test_FusionEngine.cpp`
7. `Test_QueryEngine.cpp`
8. `Test_InterestManager.cpp`
9. `Test_TacticalHistory.cpp`
10. `Test_TacticalObjectsRuntime.cpp`
11. `Test_TacticalObjectsCodec.cpp`
12. `Test_TacticalObjectsComponent.cpp`

This sequence builds the lowest-risk foundation first, covers the full HLR surface area before integration, and leaves PCL wiring until the domain model is stable.
