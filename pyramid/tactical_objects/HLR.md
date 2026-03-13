# AUTOMTK-TACTICAL-OBJECTS Requirements

Requirements for the `tactical_objects` component, following the PYRAMID component style (see `ref/components.md` Data Fusion and Geography sections).

The component stores, updates, queries, and fuses real-world entities of tactical significance together with geographic zones.

## Subject Matter

Tactical Objects covers:

- storing tactical entities and their state
- tracking where object information came from and how confident it is
- storing the military classification data needed to drive MIL-STD-2525B rendering (battle dimension, affiliation, role, status, echelon, etc.)
- storing geographic areas, routes, corridors, and zones
- fusing entity data from multiple sources

**Exclusions:** raw sensor processing, map rendering, route planning, and engagement decisions. This component provides authoritative object data to those functions.

## Key Entities

| Entity | Description |
| :--- | :--- |
| **Tactical_Object** | The system's current best representation of a real-world entity, feature, or tactical construct. |
| **Tactical_Object_Type** | What kind of thing it is: platform, person, equipment, unit, formation, installation, obstacle, route, point, or zone. |
| **Observation** | A single source report about a possible object at a point in time, with origin, timestamp, confidence, and uncertainty. |
| **Track** | A time-ordered sequence of observations believed to refer to the same entity. |
| **Fused_Object** | A Tactical_Object built from one or more observations or tracks. |
| **Evidence_Lineage** | The chain linking a Tactical_Object back to the observations, tracks, sources, and fusion steps that produced it. |
| **Object_Quality** | Confidence, completeness, freshness, consistency, and positional uncertainty for a Tactical_Object. |
| **Identity** | Callsign, track number, platform class, unit designation, and other naming attributes. |
| **Affiliation** | Friend, hostile, neutral, unknown, assumed friend, suspect, joker, faker, or pending. |
| **Battle_Dimension** | Ground, air, sea surface, subsurface, space, or SOF. |
| **Status** | Present, anticipated, planned, or known. |
| **Role** | The MIL-STD-2525B function identifier describing what the entity does (e.g. infantry, armor, rotary wing, electronic warfare). |
| **Echelon** | Team, squad, section, platoon, company, battalion, regiment, brigade, division, corps, army, or higher. |
| **Kinematic_State** | Position, orientation, velocity, acceleration, and when that data was valid. |
| **Behavior_State** | Activity, operational state, and intent hypothesis. |
| **Relationship** | A tactical, hierarchical, organizational, or physical link between objects. |
| **Geometry** | A spatial shape: point, polyline, polygon, circle, ellipse, corridor, or volume. |
| **Geographic_Zone** | A tactically significant area with geometry, type, and optional time window (e.g. AOI, no-go zone, kill box, patrol area). |
| **Zone_Relationship** | How an object relates to a zone: inside, outside, entering, leaving, intersecting, or distance. |
| **Interest_Requirement** | A request to maintain awareness of objects matching given criteria, area, or mission context. |
| **Fusion_Constraint** | A restriction on fusion: source rules, confidence thresholds, identity policies, or time windows. |
| **Capability** | The component's current ability to maintain objects, fuse evidence, answer queries, and supply complete military classification data. |

## Responsibilities

### capture_tactical_object_requirements
- Accept requirements for which object types, areas, and mission contexts to represent.

### capture_observations
- Accept observations, tracks, and supporting data from external sources with provenance and uncertainty.

### maintain_tactical_objects
- Create, update, correlate, split, merge, and retire Tactical_Objects as evidence changes.

### maintain_fusion_lineage
- Keep lineage between observations, tracks, fusion decisions, and resulting objects.

### maintain_military_classification
- Keep the fields needed to drive MIL-STD-2525B: battle dimension, affiliation, role, status, echelon, mobility, headquarters flag, task force flag, feint/dummy flag, installation flag, and modifiers.

### maintain_geographic_zones
- Create, update, and query geographic zones and their relationships to objects.

### determine_object_relationships
- Determine hierarchical, organizational, tactical, and proximity relationships between objects.

### determine_zone_relationships
- Determine containment, intersection, approach, and boundary crossing between objects and zones.

### assess_object_quality
- Assess confidence, freshness, consistency, and uncertainty of objects and fused identities.

### assess_capability
- Report the component's current and predicted ability to maintain state at required scale and quality.

## Design Decisions

### D1 - Canonical Internal Model
Use a single internal model for tactical objects. Treat external message formats and display standards as adapters.

**Why**: Avoids coupling to any single feed or display standard and enables fusion across heterogeneous sources.

### D2 - Separate Observation, Track, and Fused Object Layers
Keep source observations, correlated tracks, and authoritative fused objects as distinct record types.

**Why**: Separating raw evidence from intermediate correlation from authoritative state supports explainability, auditability, and controlled fusion.

### D3 - Provenance Is First-Class
Every fused object retains lineage to the observations, tracks, sources, and rules that built it.

**Why**: Required for trust, debugging, operator explanation, and fusion rollback.

### D4 - Store Military Classification Data, Not Symbol Codes
Store the semantic fields (battle dimension, affiliation, role, status, echelon, etc.) that drive 2525B, not rendered icon codes. Preserve source symbol codes when provided, but keep normalized fields so symbols can be regenerated after fusion or reclassification.

**Why**: The same object must support reasoning, exchange, and display. A stored icon code becomes stale when identity or confidence changes.

### D5 - Zones Are First-Class Objects
Geographic zones have geometry, type, temporal validity, and relationship queries. They are operational constructs, not display overlays.

**Why**: Zones drive filtering, alerting, planning, and behavior interpretation.

### D6 - ECS-Style Runtime Storage
Use sparse-component (ECS-like) storage for hot-path object state.

**Why**: Thousands of entities with frequent updates and spatial queries need cache-friendly, selective-update storage.

### D7 - Domain API Over ECS Internals
ECS storage is an internal detail behind a domain-oriented API.

**Why**: Consumers interact with objects, zones, tracks, and relationships, not storage mechanics.

### D8 - Pluggable Fusion Policy
Fusion, correlation, merge, and split decisions use pluggable rules and thresholds.

**Why**: Correlation policies vary by source mix, mission context, and object class.

### D9 - Multi-Index Query Strategy
Maintain dedicated indexes for identity, source references, time, and geography.

**Why**: Efficient retrieval at scale needs more than a single object map, especially for region and interest queries.

### D10 - Event-Driven Updates With Snapshot Access
Support event-driven ingestion and updates while providing queryable current-state snapshots.

**Why**: Data arrives incrementally, but most consumers need coherent current state.

## Detailed Requirements

## Core Object Model

### TOBJ.001 - Object Types
Support tactical object types: platform, person, equipment, unit, formation, installation, feature, route, point, area, and zone.

**Rationale**: Covers the full range of battlespace entities.

### TOBJ.002 - Object Properties
Each object can carry: identity, classification, affiliation, battle dimension, role, status, echelon, kinematics, behavior, operational state, provenance, and quality.

**Rationale**: These fields are the minimum needed for tactical reasoning and 2525B rendering.

### TOBJ.003 - Stable Internal ID
Assign each Tactical_Object a stable internal UUID independent of any source-provided identifier.

**Rationale**: Source IDs are not globally unique or stable across fusion events.

### TOBJ.004 - External ID Association
Support one-to-many mapping between a Tactical_Object and external identifiers, track numbers, and naming schemes.

**Rationale**: The same real-world entity may have different IDs across providers.

## Object Interest

### TOBJ.005 - Interest Requirement
Accept interest requirements based on type, identity, affiliation, status, source, quality, behavior, time, or geography.

**Rationale**: Focused awareness.

### TOBJ.006 - Interest Cancellation
Support cancellation, expiry, and supersession of active interest requirements.

**Rationale**: Lifecycle management.

## Query Support

### TOBJ.007 - Criteria Query
Support queries matching compound predicates across identity, affiliation, battle dimension, role, quality, relationships, and spatial state.

**Rationale**: Filtered retrieval.

### TOBJ.008 - Single Object Query
Support retrieval by internal ID or by external source reference.

**Rationale**: Direct lookup.

### TOBJ.009 - Region Query
Support queries for objects within, intersecting, entering, or nearest to a region or zone.

**Rationale**: Geographic filtering and alerting.

### TOBJ.010 - Temporal Query
Support retrieval of object state at a given time or over an interval where history is retained.

**Rationale**: Auditability and after-action review.

## Relationship Tracking

### TOBJ.011 - Object Relationships
Track relationships between tactical objects.

**Rationale**: Association awareness.

### TOBJ.012 - Hierarchical Relationships
Support hierarchical and organizational relationships: equipment-on-platform, member-of-unit, unit-in-formation.

**Rationale**: Organizational structure.

### TOBJ.013 - Tactical Relationships
Support tactical relationships: tracking, escorting, engaging, protecting, following, supporting.

**Rationale**: Situation awareness.

### TOBJ.014 - Relationship Confidence
Maintain confidence and source provenance for each relationship.

**Rationale**: Relationships are often inferred and must be explainable.

## Behavior and State

### TOBJ.015 - Behavior Estimation
Report behavior patterns and intent hypotheses when available.

**Rationale**: Behavioral awareness.

### TOBJ.016 - Operational State
Represent activity state: active, inactive, damaged, destroyed, on-ground, airborne, afloat, stationary, moving.

**Rationale**: Activity awareness.

### TOBJ.017 - State Freshness
Track freshness and validity windows for each state element. Expose whether state is current, stale, predicted, or confirmed.

**Rationale**: Consumers must know whether data is trustworthy.

## Fusion and Provenance

### TOBJ.018 - Multi-Source Ingest
Ingest and integrate object information from multiple external sources.

**Rationale**: Comprehensive picture.

### TOBJ.019 - Confidence Tracking
Maintain and report confidence, uncertainty, and completeness for each field and for the fused object as a whole.

**Rationale**: Quality awareness.

### TOBJ.020 - Evidence Lineage
Maintain lineage from each object back to contributing observations, tracks, sources, and fusion operations.

**Rationale**: Traceability and explainability.

### TOBJ.021 - Entity Correlation
Determine whether new observations or tracks match an existing object or require a new one.

**Rationale**: Correlation is the basis of coherent entity state.

### TOBJ.022 - Entity Fusion
Fuse multiple observations or tracks into a single object while preserving source identity, lineage, and quality.

**Rationale**: Unified picture requires controlled amalgamation.

### TOBJ.023 - Merge and Split
Support explicit merge and split of objects, retaining lineage across transitions.

**Rationale**: Correlation decisions change as evidence evolves.

### TOBJ.024 - Fusion Constraints
Support configurable fusion constraints based on source, time window, object type, quality thresholds, and mission context.

**Rationale**: Rules vary by scenario and confidence model.

## Military Classification (MIL-STD-2525B Data)

### TOBJ.025 - Battle Dimension
Store the object's battle dimension: ground, air, sea surface, subsurface, space, or SOF.

**Rationale**: Battle dimension is the primary domain classifier in 2525B and drives downstream symbol rendering and filtering.

### TOBJ.026 - Affiliation
Store affiliation: friendly, hostile, neutral, unknown, assumed friend, suspect, joker, faker, or pending.

**Rationale**: Affiliation is a core 2525B field and a primary filter for rules of engagement and display.

### TOBJ.027 - Role / Function Identifier
Store a function identifier describing what the entity does, using the 2525B function ID scheme (e.g. infantry, armor, artillery, rotary wing, electronic warfare, logistics, C2, medical).

**Rationale**: Role determines the symbol shape and tactical meaning.

### TOBJ.028 - Status
Store operational status: present, anticipated, planned, or known.

**Rationale**: Status affects the symbol rendering style (solid vs dashed) and operational reasoning.

### TOBJ.029 - Echelon
Store organizational echelon when applicable: team, squad, section, platoon, company, battalion, regiment, brigade, division, corps, army, or higher.

**Rationale**: Echelon determines the size/scale indicator in 2525B and matters for force aggregation.

### TOBJ.030 - Indicator Flags
Store boolean flags for: headquarters, task force, feint/dummy, and installation.

**Rationale**: These are distinct 2525B modifiers that affect symbol rendering and tactical interpretation.

### TOBJ.031 - Mobility Indicator
Store mobility type when applicable: wheeled, tracked, towed, rail, over-snow, sled, barge, amphibious.

**Rationale**: Mobility affects the 2525B symbol modifier and movement planning.

### TOBJ.032 - Source Symbol Code Preservation
When a source provides a complete 2525B SIDC (symbol identification coding), preserve it alongside the normalized fields. Allow symbols to be regenerated from normalized fields after fusion or reclassification.

**Rationale**: Source fidelity and internal consistency are both needed.

## Geography and Zones

### TOBJ.033 - Geometry Types
Support point, polyline, polygon, circle, ellipse, corridor, and optionally volume geometries for objects and zones.

**Rationale**: Tactical reasoning needs more than point tracks.

### TOBJ.034 - Zone Semantics
Represent semantically distinct zones: AOI, patrol area, restricted area, no-go area, kill box, phase line, boundary, route corridor, sensor coverage area.

**Rationale**: Different zone types drive different downstream behavior.

### TOBJ.035 - Geodesic Spatial Reasoning
Perform spatial calculations using geographic coordinates. Do not assume a flat-earth model where it would materially degrade accuracy.

**Rationale**: Geographic correctness matters for containment, distance, and boundary crossing.

### TOBJ.036 - Zone Relationship Queries
Determine whether an object is inside, outside, intersecting, entering, leaving, or nearest to a zone.

**Rationale**: Zone-based alerting and monitoring are core use cases.

### TOBJ.037 - Temporal Zones
Support validity intervals on zones so they can be active only during specific phases.

**Rationale**: Tactical areas are often time-limited.

## Scale and Performance

### TOBJ.038 - Thousands of Entities
Efficiently maintain and query at least thousands of concurrent objects and zones.

**Rationale**: Operational pictures routinely exceed what naive per-object designs can handle.

### TOBJ.039 - Sparse Component Storage
Allow optional storage of object facets (kinematics, military classification, fusion state, relationships) so sparse entities do not pay for unused fields.

**Rationale**: ECS-style sparsity improves memory and update efficiency at scale.

### TOBJ.040 - Spatial Indexing
Maintain spatial indexes for region, proximity, and zone queries over large populations.

**Rationale**: Geographic queries must stay fast at scale.

### TOBJ.041 - Incremental Updates
Process updates incrementally without recomputing unrelated objects.

**Rationale**: Real-time feeds need bounded update cost.

### TOBJ.042 - Bulk Ingest
Support efficient bulk ingest and replay of observations and object updates.

**Rationale**: Initialization, backfill, and replay are common needs.

## Interfaces and Quality

### TOBJ.043 - Capability Reporting
Report the component's ability to maintain state, fuse evidence, answer geographic queries, and supply complete military classification data.

**Rationale**: Consumers need confidence in the service itself.

### TOBJ.044 - Missing Information Reporting
Identify missing data that prevents confident classification, correlation, military classification completeness, or zone reasoning.

**Rationale**: Gaps should be visible so sensors or operators can fill them.

### TOBJ.045 - Snapshot and Event Access
Provide both current-state snapshot access and event-oriented update streams.

**Rationale**: Some consumers need authoritative state; others need incremental changes.

## Implementation Notes

- Expose a domain model API: `TacticalObject`, `Observation`, `Track`, `Zone`, `Relationship`.
- Use ECS-style sparse components for hot-path storage.
- Attach provenance, confidence, and uncertainty to individual fields, not just the whole object.
- Maintain dedicated indexes for object ID, external references, affiliation, type, time, and spatial lookup.
- Store the normalized military classification fields (battle dimension, affiliation, role, status, echelon, flags, mobility) as the authoritative data. Treat 2525B symbol code generation as an output concern driven by those fields.
