# Tactical Objects Low-Level Requirements

Low-level requirements for `pyramid/tactical_objects`, derived from the high-level requirements in `HLR.md` and the design decisions in `DESIGN.md`.

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
Each correlated object shall retain all distinct source references that contributed to it.

**Traces**: TOBJ.004

**Verification**: `Test_CorrelationEngine.cpp` correlates two compatible observations from different sources and verifies both source references are retained.

### REQ_TACTICAL_OBJECTS_007 - Lineage Retention
Each correlated object shall retain lineage entries referencing contributing observations and tracks.

**Traces**: TOBJ.020

**Verification**: `Test_CorrelationEngine.cpp` ingests observations, performs correlation, and verifies lineage contains the expected observation and track IDs.

---

## 3. Correlation

### REQ_TACTICAL_OBJECTS_008 - Candidate Prefilter by Spatial Index
`CorrelationEngine` shall obtain correlation candidates from `SpatialIndex` before exact scoring.

**Traces**: TOBJ.040

**Verification**: `Test_CorrelationEngine.cpp` loads distant objects and verifies only local candidates are scored.

### REQ_TACTICAL_OBJECTS_009 - Deterministic Correlation Score
`CorrelationEngine` shall calculate a deterministic correlation score from configured gates and fixed field weights.

**Traces**: TOBJ.021

**Verification**: `Test_CorrelationEngine.cpp` evaluates the same input twice and verifies identical score and decision.

### REQ_TACTICAL_OBJECTS_010 - Create New Object on Low Score
If all candidate scores are below `create_threshold`, `CorrelationEngine` shall create a new track and a new correlated object.

**Traces**: TOBJ.021, TOBJ.022

**Verification**: `Test_CorrelationEngine.cpp` provides a spatially distant observation and verifies object count increases by one.

### REQ_TACTICAL_OBJECTS_011 - Merge on High Score
If the best candidate score is >= `merge_threshold`, `CorrelationEngine` shall attach the observation to the existing correlated object.

**Traces**: TOBJ.022

**Verification**: `Test_CorrelationEngine.cpp` provides two compatible observations and verifies a single correlated object remains.

### REQ_TACTICAL_OBJECTS_012 - Split on Sustained Incompatibility
If incompatible evidence exceeds the configured split threshold for the configured number of updates, `CorrelationEngine` shall split the contributing track into a new correlated object.

**Traces**: TOBJ.023

**Verification**: `Test_CorrelationEngine.cpp` injects repeated incompatible updates and verifies the original correlated object is split.

### REQ_TACTICAL_OBJECTS_013 - Confidence Aggregation
`CorrelationEngine` shall update correlated object confidence using weighted contribution from source confidence and recency.

**Traces**: TOBJ.019

**Verification**: `Test_CorrelationEngine.cpp` correlates high-confidence and low-confidence observations and verifies the aggregate confidence lies within configured bounds.

---

## 4. Object Classification (Exploitation: MIL-STD-2525B)

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

## 5. Spatial Reasoning (Exploitation Design)

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

### REQ_TACTICAL_OBJECTS_025 - Identifier Query
`QueryEngine` shall support exact lookup by internal UUID and by source system and source entity identifier.

**Traces**: TOBJ.008

**Verification**: `Test_QueryEngine.cpp` verifies both internal UUID lookup and external identifier lookup resolve to the correct object.

---

## 7. Direct Entity Management

### REQ_TACTICAL_OBJECTS_026 - Direct Create
`TacticalObjectsRuntime::createObject()` shall store an object with all provided components and record `direct` provenance without invoking correlation.

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
Objects created via evidence (correlation) and via direct CRUD shall be queryable through the same `QueryEngine`.

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
`TacticalObjectsRuntime` shall produce a capability snapshot containing counts, supported geometry types, and correlation configuration state.

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
The runtime shall publish internal domain events on `pyramid::core::event::EventBus` for object created, object updated, object deleted, object correlated, object split, zone entered, and zone left.

**Traces**: TOBJ.045

**Verification**: `Test_TacticalObjectsRuntime.cpp` subscribes to the event bus and verifies each event is emitted for the corresponding operation.

### REQ_TACTICAL_OBJECTS_040 - Missing Information Diagnostics
The runtime shall identify and log missing or incomplete information that blocks confident classification, correlation, military classification completeness, or zone reasoning.

**Traces**: TOBJ.044

**Verification**: `Test_TacticalObjectsRuntime.cpp` injects a logger, submits incomplete data, and verifies diagnostics identify the missing information and expected severity.

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

### REQ_TACTICAL_OBJECTS_044 - Batch Correlation Determinism
Processing the same observation batch in the same order shall produce the same correlated object graph and versions.

**Traces**: TOBJ.022, TOBJ.024

**Verification**: `Test_TacticalObjectsRuntime.cpp` processes identical batches in two fresh runtimes and compares resulting object IDs, lineage shape, and versions.

### REQ_TACTICAL_OBJECTS_045 - Memory-Reasonable Sparse Storage
An object with no zone, behavior, or military classification components shall not allocate entries in those component tables.

**Traces**: TOBJ.039

**Verification**: `Test_ObjectStore.cpp` inserts sparse objects and verifies optional component table counts remain unchanged.

---

## 12. Additional HLR Coverage

### REQ_TACTICAL_OBJECTS_046 - Interest Requirement Registration
`TacticalObjectsRuntime::registerInterest()` shall accept interest requirements over object type, identity, affiliation, status, source, quality, behavior, time window, and geography, and persist them behind a stable interest identifier.

**Traces**: TOBJ.005

**Verification**: `Test_InterestManager.cpp` registers a compound interest requirement and verifies the stored criteria round-trip with a stable interest identifier.

### REQ_TACTICAL_OBJECTS_047 - Interest Cancellation and Supersession
The runtime shall support explicit cancellation, expiry, and supersession of active interest requirements.

**Traces**: TOBJ.006

**Verification**: `Test_InterestManager.cpp` cancels one interest, lets another expire, supersedes a third, and verifies only the expected requirements remain active.

### REQ_TACTICAL_OBJECTS_048 - Temporal Snapshot and Interval Query
`QueryEngine` shall support querying object state at a point in time and across a retained history interval.

**Traces**: TOBJ.010

**Verification**: `Test_TacticalHistory.cpp` applies timestamped updates to one object and verifies as-of and interval queries return the correct retained versions.

### REQ_TACTICAL_OBJECTS_049 - Relationship Record Storage
`RelationshipIndex` shall store relationships between tactical objects as first-class records linked by UUID.

**Traces**: TOBJ.011

**Verification**: `Test_RelationshipIndex.cpp` creates a relationship and verifies it is retrievable from both subject and object perspectives.

### REQ_TACTICAL_OBJECTS_050 - Hierarchical Relationship Query
The runtime and query path shall represent and query hierarchical relationships such as equipment-on-platform, member-of-unit, and unit-in-formation.

**Traces**: TOBJ.012

**Verification**: `Test_RelationshipIndex.cpp` inserts hierarchical relationships and verifies parent and child queries return the expected linked objects.

### REQ_TACTICAL_OBJECTS_051 - Tactical Relationship Query
The runtime and query path shall represent and query tactical relationships such as tracking, escorting, engaging, protecting, following, and supporting.

**Traces**: TOBJ.013

**Verification**: `Test_RelationshipIndex.cpp` inserts tactical relationships and verifies query predicates return only matching linked objects.

### REQ_TACTICAL_OBJECTS_052 - Relationship Confidence and Provenance
Each stored relationship shall retain confidence and source provenance, and expose both through query results.

**Traces**: TOBJ.014

**Verification**: `Test_RelationshipIndex.cpp` stores a relationship with confidence and source metadata and verifies both are preserved in the retrieved record.

### REQ_TACTICAL_OBJECTS_053 - Behavior Estimation Storage
The runtime shall store and return behavior pattern and intent hypothesis fields when they are provided for an object.

**Traces**: TOBJ.015

**Verification**: `Test_TacticalObjectsRuntime.cpp` creates or updates an object with behavior fields and verifies they round-trip through retrieval and query.

### REQ_TACTICAL_OBJECTS_054 - Operational State Storage
The runtime and query path shall store and query operational state values and their freshness.

**Traces**: TOBJ.016

**Verification**: `Test_TacticalObjectsRuntime.cpp` updates an object's operational state and freshness, then verifies state-based query and retrieval return the expected values.

### REQ_TACTICAL_OBJECTS_055 - Zone Semantic Typing
`ZoneEngine` shall preserve semantic zone type such as AOI, patrol area, restricted area, no-go area, kill box, phase line, boundary, route corridor, and sensor coverage area, and expose it in retrieval and query results.

**Traces**: TOBJ.034

**Verification**: `Test_ZoneEngine.cpp` stores zones with distinct semantic types and verifies semantic type round-trip and query filtering.

### REQ_TACTICAL_OBJECTS_056 - Geodesic Spatial Reasoning
`ZoneEngine` and `QueryEngine` shall use geodesic distance or containment calculations for circle, nearest, and proximity checks when a flat-earth approximation would materially change the result.

**Traces**: TOBJ.035

**Verification**: `Test_ZoneEngine.cpp` evaluates latitude-sensitive distance and containment cases and verifies the accepted result matches geodesic reasoning.

### REQ_TACTICAL_OBJECTS_057 - Interest Service Integration
`TacticalObjectsComponent` shall expose interest registration and cancellation through the `tactical_objects.interest` service boundary.

**Traces**: TOBJ.005, TOBJ.006

**Verification**: `Test_TacticalObjectsComponent.cpp` invokes the interest service to register and cancel requirements and verifies the runtime interest state changes accordingly.

---

## 13. Measurement, Progress, and Capability Progression

### REQ_TACTICAL_OBJECTS_062 - Measurement Criteria Acceptance
`TacticalObjectsRuntime` shall accept Measurement_Criterion/criteria that define how the predicted or actual quality of a Tactical_Object or Object_Interest_Requirement is assessed, and apply them during quality determination.

**Traces**: TOBJ.050

**Verification**: `Test_TacticalObjectsRuntime.cpp` registers a measurement criterion against an interest requirement and verifies the quality assessment uses the criterion.

### REQ_TACTICAL_OBJECTS_063 - Progress Reporting
`TacticalObjectsRuntime` shall report progress against each active Object_Interest_Requirement, including partial fulfilment, remaining gaps, and estimated completeness.

**Traces**: TOBJ.051

**Verification**: `Test_InterestManager.cpp` registers an interest requirement, provides partial evidence, and verifies the progress report reflects partial fulfilment.

### REQ_TACTICAL_OBJECTS_064 - Object Probability Density Capture
`TacticalObjectsRuntime` shall capture and store probability densities of particular Tactical_Objects within locations.

**Traces**: TOBJ.052

**Verification**: `Test_TacticalObjectsRuntime.cpp` submits probability density data for an object within a region and verifies it is stored and retrievable.

### REQ_TACTICAL_OBJECTS_065 - Capability Progression Prediction
`TacticalObjectsRuntime` shall predict how its Capability will change over time and with use, taking account of expected information source availability and operational constraints.

**Traces**: TOBJ.053

**Verification**: `Test_TacticalObjectsRuntime.cpp` configures the runtime with known source availability changes and verifies the capability progression prediction reflects expected degradation or improvement.

---

## 14. Requirement Fulfilment and Evidence Derivation

### REQ_TACTICAL_OBJECTS_058 - Requirement Achievability Assessment
`TacticalObjectsRuntime` shall assess whether a tactical object requirement is achievable given current capability, constraints, area of interest, timeliness, and expected evidence availability.

**Traces**: TOBJ.046

**Verification**: `Test_TacticalObjectsRuntime.cpp` submits an AOI-based requirement under two capability configurations and verifies the achievability result changes accordingly.

### REQ_TACTICAL_OBJECTS_059 - Tactical Object Solution Determination
The runtime shall determine an object solution for a tactical object requirement, including intended object types, area of interest, timing, and predicted quality or completeness.

**Traces**: TOBJ.047

**Verification**: `Test_TacticalObjectsRuntime.cpp` submits a tactical object requirement and verifies the returned solution contains the expected AOI, object criteria, and predicted-quality fields.

### REQ_TACTICAL_OBJECTS_060 - Derived Evidence Requirement
When additional evidence is needed to satisfy a tactical object requirement, the runtime shall derive one or more component-agnostic `Object_Solution_Evidence_Requirement` records that describe the needed evidence or supporting information without naming another PRA component.

**Traces**: TOBJ.048

**Verification**: `Test_InterestManager.cpp` registers an AOI-based active-finding requirement and verifies the derived evidence requirement expresses evidence needs in tactical-object terms rather than naming a downstream component.

### REQ_TACTICAL_OBJECTS_061 - Requirement and Evidence Traceability
The runtime shall retain traceability links between a source tactical object requirement, its derived evidence requirements, the observations or supporting information received in response, and any correlated objects produced.

**Traces**: TOBJ.049

**Verification**: `Test_TacticalObjectsRuntime.cpp` submits an AOI requirement, records derived evidence requirements and subsequent observations, and verifies the resulting correlated object can be traced back to the source requirement.

---

## 15. Active Find Scenario — Solution Dependency and End-to-End Flow

### REQ_TACTICAL_OBJECTS_066 - ActiveFind Mode Dispatch at Service Boundary
`TacticalObjectsComponent::handleSubscribeInterest()` shall distinguish `query_mode=active_find` from `read_current` (or absent). When the mode is `active_find`, the handler shall invoke `determineSolution()` and include the solution and derived evidence requirements in the service response. When the mode is `read_current` or absent, the handler shall register the interest without triggering solution derivation.

**Traces**: TOBJ.005, TOBJ.047

**Verification**: `Test_TacticalObjectsComponent_HLR.cpp` invokes the `subscribe_interest` service with `query_mode=active_find` and verifies the response contains `solution_id`, `predicted_quality`, `predicted_completeness`, and a non-empty `evidence_requirements` array. A second invocation without `query_mode` verifies the response contains `interest_id` but no `solution_id`.

### REQ_TACTICAL_OBJECTS_067 - Evidence Requirement Publication
When an ActiveFind interest is registered, `TacticalObjectsComponent` shall publish each derived `DerivedEvidenceRequirement` as a JSON message on the `evidence_requirements` topic, containing `requirement_id`, `source_interest_id`, and `evidence_description`.

**Traces**: TOBJ.048, TOBJ.047

**Verification**: `Test_TacticalObjectsComponent_HLR.cpp` registers a subscriber on `evidence_requirements`, invokes `subscribe_interest` with `query_mode=active_find`, and verifies the subscriber receives a JSON message with the expected fields.

### REQ_TACTICAL_OBJECTS_068 - Interest-Entity Matching for Streaming
`TacticalObjectsComponent::on_tick()` shall iterate active interests, match entities in the store against each interest's criteria using `InterestManager::matchesInterest()`, and publish matching entities as binary `entity_updates` frames to subscribed consumers.

**Traces**: TOBJ.005, TOBJ.045

**Verification**: `Test_TacticalObjects_E2E.cpp` (`AdaClientZoneInterestReceivesEntityEvidence`) registers an interest, creates an entity matching the criteria, spins the executor, and verifies the entity_updates subscriber receives at least one frame containing the matching entity.

### REQ_TACTICAL_OBJECTS_069 - Battle Dimension Filter in Interest Matching
`InterestManager::matchesInterest()` shall reject entities whose `MilClassComponent.battle_dim` does not match the interest criteria's `battle_dimension`, when that criterion is specified.

**Traces**: TOBJ.005, TOBJ.025

**Verification**: `Test_InterestManager_Matching.cpp` creates entities with different battle dimensions (e.g. Ground, SeaSurface, Air) and verifies that an interest with `battle_dimension=SeaSurface` matches only the SeaSurface entity.

### REQ_TACTICAL_OBJECTS_070 - ActiveFind Closed-Loop Integration
When an ActiveFind interest is registered, the full closed-loop shall execute: (1) the component derives evidence requirements and publishes them, (2) an evidence provider receives the requirement and submits observations to `observation_ingress`, (3) the component correlates those observations into tracked entities, and (4) the component streams entity updates matching the interest criteria back to the subscribing client.

**Traces**: TOBJ.047, TOBJ.048, TOBJ.049, TOBJ.005

**Verification**: `Test_TacticalObjects_E2E.cpp` (`ActiveFindSolutionDrivesEvidenceProvider`) exercises the full three-party flow (client, tactical_objects, evidence provider) and asserts: solution_id is returned, evidence provider receives a derived requirement, entity updates arrive at the client, and the received entity matches the original `Hostile Platform SeaSurface` criteria.

### REQ_TACTICAL_OBJECTS_071 - Solution Response Encoding Contract
The `subscribe_interest` service response for an ActiveFind request shall be a JSON object containing: `interest_id` (string UUID), `solution_id` (string UUID), `predicted_quality` (double, 0..1), `predicted_completeness` (double, 0..1), and `evidence_requirements` (array of objects each with `requirement_id`, `source_interest_id`, `evidence_description`).

**Traces**: TOBJ.047

**Verification**: `Test_TacticalObjectsComponent_HLR.cpp` invokes `subscribe_interest` with `query_mode=active_find`, parses the JSON response, and asserts the presence and types of all specified fields.

### REQ_TACTICAL_OBJECTS_072 - Derived Evidence Requirement Carries Interest Criteria
Each `DerivedEvidenceRequirement` produced by `InterestManager::deriveEvidenceRequirement()` shall carry a copy of the source interest's criteria (object_type, affiliation, battle_dimension, area) so that an evidence provider can fulfil the requirement without further queries.

**Traces**: TOBJ.048

**Verification**: `Test_InterestManager.cpp` registers an ActiveFind interest with all criteria fields populated, derives an evidence requirement, and verifies the derived requirement's `criteria` fields match the source interest's criteria.

---

## Traceability Summary

| LLR | TOBJ | Test File |
|-----|------|-----------|
| 001 | 003 | Test_ObjectStore |
| 002 | 038 | Test_ObjectStore |
| 003 | 039 | Test_ObjectStore |
| 004 | 002 | Test_ObjectStore |
| 005 | 018, 042 | Test_TacticalObjectsRuntime |
| 006 | 004 | Test_CorrelationEngine |
| 007 | 020 | Test_CorrelationEngine |
| 008 | 040 | Test_CorrelationEngine |
| 009 | 021 | Test_CorrelationEngine |
| 010 | 021, 022 | Test_CorrelationEngine |
| 011 | 022 | Test_CorrelationEngine |
| 012 | 023 | Test_CorrelationEngine |
| 013 | 019 | Test_CorrelationEngine |
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
| 046 | 005 | Test_InterestManager |
| 047 | 006 | Test_InterestManager |
| 048 | 010 | Test_TacticalHistory |
| 049 | 011 | Test_RelationshipIndex |
| 050 | 012 | Test_RelationshipIndex |
| 051 | 013 | Test_RelationshipIndex |
| 052 | 014 | Test_RelationshipIndex |
| 053 | 015 | Test_TacticalObjectsRuntime |
| 054 | 016 | Test_TacticalObjectsRuntime |
| 055 | 034 | Test_ZoneEngine |
| 056 | 035 | Test_ZoneEngine |
| 057 | 005, 006 | Test_TacticalObjectsComponent |
| 058 | 046 | Test_TacticalObjectsRuntime |
| 059 | 047 | Test_TacticalObjectsRuntime |
| 060 | 048 | Test_InterestManager |
| 061 | 049 | Test_TacticalObjectsRuntime |
| 062 | 050 | Test_TacticalObjectsRuntime |
| 063 | 051 | Test_InterestManager |
| 064 | 052 | Test_TacticalObjectsRuntime |
| 065 | 053 | Test_TacticalObjectsRuntime |
| 066 | 005, 047 | Test_TacticalObjectsComponent_HLR |
| 067 | 048, 047 | Test_TacticalObjectsComponent_HLR |
| 068 | 005, 045 | Test_TacticalObjects_E2E |
| 069 | 005, 025 | Test_InterestManager_Matching |
| 070 | 047, 048, 049, 005 | Test_TacticalObjects_E2E |
| 071 | 047 | Test_TacticalObjectsComponent_HLR |
| 072 | 048 | Test_InterestManager |
