# AUTOMTK-TACTICAL-OBJECTS Requirements

Requirements for the `tactical_objects` component, following the PYRAMID component standard (5.4.2.60 of the PYRAMID Technical Standard V1-O).

The component maintains knowledge of objects which exist, or are assumed to exist, within the battlespace and their relationship to each other.

## Subject Matter

The subject matter of Tactical Objects is objects of tactical importance which exist, or are assumed to exist within the battlespace, their Relationships between each other, and their Behaviour.

**Exclusions (per PYRAMID):**

- The detectability of a Tactical_Object.
- The level of risk a Tactical_Object poses to the Exploiting Platform.
- Low level sensor data calculation.
- How the data from the Object_Information_Source is derived.

**Additional exploitation exclusions:** raw sensor processing, map rendering, route planning, and engagement decisions. This component provides authoritative object data to those functions.

## Responsibilities

Responsibilities are taken directly from PYRAMID Technical Standard 5.4.2.60.4. Identifiers are included for HLR traceability.

### RESP.001 - capture_object_interest_requirement_for_tactical_object
- To capture provided Object_Interest_Requirements (e.g. the scope of the information required and the frequency that it is reported) for Tactical_Objects.

### RESP.002 - capture_measurement_criteria_for_tactical_object
- To capture provided Measurement_Criterion/criteria for Tactical_Objects.

### RESP.003 - capture_tactical_objects_constraints
- To capture provided Constraints for Tactical_Objects solutions (e.g. do not process classified information or stop processing information derived from a specified source).

### RESP.004 - identify_whether_requirement_is_achievable
- To identify whether an Object_Interest_Requirement is still achievable given current or predicted Capability and Constraints.

### RESP.005 - determine_requirement_solution
- To determine a solution to Object_Interest_Requirements.

### RESP.006 - determine_potential_objects
- To determine the potential for Tactical_Objects to exist at locations.

### RESP.007 - determine_object_information_confidence
- To determine a level of confidence in the individual Tactical_Object's characteristics (e.g. behaviour, allegiance and association between tactical objects).

### RESP.008 - determine_additional_information
- To determine additional information required to satisfy Object_Interest_Requirements, e.g. improved Tactical_Object confidence required.

### RESP.009 - determine_object_relationships
- To determine the Relationships and dependencies between Tactical_Objects (e.g. if a radar is part of a specific vehicle, or if a specific vehicle is part of this formation and is the flight lead).

### RESP.010 - estimate_object_behaviour
- To estimate the Behaviour exhibited by Tactical Objects, including individual Behaviour (e.g. that object is loitering) and Behaviour between Tactical Objects (e.g. that Exploiting Platform is tracking that tank, that Exploiting Platform is landing at that airfield). Behaviour also includes the operational state of a Tactical_Object (e.g. ready for operation, operational on ground, operational in air, or non-operational).

### RESP.011 - identify_progress
- To identify the progress against an Object_Interest_Requirement.

### RESP.012 - determine_quality_of_tactical_object_of_interest
- To determine the quality of the Tactical_Object_of_Interest provided by Tactical Objects during execution, measured against given Object_Interest_Requirements and Measurement_Criterion/criteria.

### RESP.013 - capture_object_information
- To capture information about Tactical_Objects, including but not limited to: their allegiance to organisations, kinematic behaviour (e.g. velocity, acceleration or path), location (e.g. at this position, within this region) and classification, including their type (e.g. surface ship or emitter), specific type (e.g. Type-45, Captor radar) and registration (e.g. HMS Daring).

### RESP.014 - capture_object_probability_densities
- To capture the probability densities of particular Tactical_Objects within locations.

### RESP.015 - assess_capability
- To assess the Capability to provide Tactical_Object information (e.g. determination and estimation of object characteristics) taking account of system health and observed anomalies (e.g. normal behaviour and impacts due to failures, damage, usage or ageing).

### RESP.016 - identify_missing_information
- To identify missing information which could improve the certainty or specificity of the component's Capability (i.e. determination and estimation of object characteristics) assessment.

### RESP.017 - predict_capability_progression
- To predict the progression of the Tactical Objects Capability over time and with use.

## Services

Services are taken directly from PYRAMID Technical Standard 5.4.2.60.7.

| Service | Type | Description |
| :--- | :--- | :--- |
| **Object_Of_Interest** | Provided | Determines objects of interest in response to an Object_Interest_Requirement. |
| **Object_Solution_Evidence** | Consumed | Identifies supporting information needed to acquire or generate information about the existence of Tactical_Objects and their details. Also consumes indications of whether the required information can be provided. |
| **Matching_Objects** | Provided | Determines which objects satisfy a specified set of criteria and provides their identifiers (not associated details). |
| **Specific_Object_Detail** | Provided | Provides information in relation to a specified object (e.g. course, speed, altitude, location, relationships, behaviour). |
| **Object_Evidence** | Consumed | Obtains object information needed to acquire or generate information about the existence of Tactical_Objects and their details. |
| **Constraint** | Provided | Assesses restrictions that constrain Tactical Objects behaviour with respect to determining information about Tactical_Objects. |
| **Capability** | Provided | Assesses the current and predicted capability to provide information about Tactical_Objects. |
| **Capability_Evidence** | Consumed | Consumes the current and predicted state of capabilities that this component depends on, and identifies any missing information, required to determine its own Capability. |

## Design Decisions

### D1 - Canonical Internal Model
Use a single internal model for tactical objects. Treat external message formats and display standards as adapters.

**Why**: Avoids coupling to any single feed or display standard and enables correlation across heterogeneous sources.

### D2 - Separate Observation, Track, and Correlated Object Layers
Keep source observations, correlated tracks, and authoritative correlated objects as distinct record types.

**Why**: Separating raw evidence from intermediate correlation from authoritative state supports explainability, auditability, and controlled integration.

### D3 - Provenance Is First-Class
Every correlated object retains lineage to the observations, tracks, sources, and rules that built it.

**Why**: Required for trust, debugging, operator explanation, and correlation rollback.

### D4 - Store Military Classification Data, Not Symbol Codes (Exploitation)
Store the semantic fields (battle dimension, affiliation, role, status, echelon, etc.) that drive 2525B, not rendered icon codes. Preserve source symbol codes when provided, but keep normalized fields so symbols can be regenerated after correlation or reclassification.

**Why**: The same object must support reasoning, exchange, and display. A stored icon code becomes stale when identity or confidence changes. This is an exploitation-specific design choice instantiating PYRAMID's abstract `classification` and `allegiance` attributes from the Specific_Object_Detail and Object_Evidence interfaces.

### D5 - Zone and Region Reasoning (Exploitation)
Maintain internal representations of zones and regions to support region-based queries, interest requirement areas, and spatial alerting. Zone data may be sourced from the Geography component or provided externally via interest requirements.

**Why**: PYRAMID requires determining potential objects at locations, matching objects by spatial criteria, and supporting region-based interest requirements. Internal zone reasoning is an exploitation design choice to deliver these capabilities. The component does not claim ownership of geographic features (that is Geography's responsibility).

### D6 - ECS-Style Runtime Storage
Use sparse-component (ECS-like) storage for hot-path object state.

**Why**: Thousands of entities with frequent updates and spatial queries need cache-friendly, selective-update storage.

### D7 - Domain API Over ECS Internals
ECS storage is an internal detail behind a domain-oriented API.

**Why**: Consumers interact with objects, zones, tracks, and relationships, not storage mechanics.

### D8 - Pluggable Correlation Policy
Correlation, merge, and split decisions use pluggable rules and thresholds.

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

**Rationale**: These fields are the minimum needed for tactical reasoning.

### TOBJ.003 - Stable Internal ID
Assign each Tactical_Object a stable internal UUID independent of any source-provided identifier.

**Rationale**: Source IDs are not globally unique or stable across correlation events.

### TOBJ.004 - External ID Association
Support one-to-many mapping between a Tactical_Object and external identifiers, track numbers, and naming schemes.

**Rationale**: The same real-world entity may have different IDs across providers.

## Object Interest and Measurement

### TOBJ.005 - Interest Requirement
Accept interest requirements based on type, identity, affiliation, status, source, quality, behavior, time, or geography.

**Rationale**: Focused awareness.

### TOBJ.006 - Interest Cancellation
Support cancellation, expiry, and supersession of active interest requirements.

**Rationale**: Lifecycle management.

### TOBJ.050 - Measurement Criteria
Accept and store measurement criteria against which the predicted or actual quality of Tactical_Objects or Object_Interest_Requirements are assessed.

**Rationale**: PYRAMID mandates capture and use of Measurement_Criterion for quality assessment.

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

## Correlation and Provenance

### TOBJ.018 - Multi-Source Ingest
Ingest and integrate object information from multiple external sources.

**Rationale**: Comprehensive picture.

### TOBJ.019 - Confidence Tracking
Maintain and report confidence, uncertainty, and completeness for each field and for the correlated object as a whole.

**Rationale**: Quality awareness.

### TOBJ.020 - Evidence Lineage
Maintain lineage from each object back to contributing observations, tracks, sources, and correlation operations.

**Rationale**: Traceability and explainability.

### TOBJ.021 - Entity Correlation
Determine whether new observations or tracks match an existing object or require a new one.

**Rationale**: Correlation is the basis of coherent entity state.

### TOBJ.022 - Entity Integration
Integrate multiple observations or tracks into a single object while preserving source identity, lineage, and quality.

**Rationale**: Unified picture requires controlled amalgamation of evidence from multiple sources.

### TOBJ.023 - Merge and Split
Support explicit merge and split of objects, retaining lineage across transitions.

**Rationale**: Correlation decisions change as evidence evolves.

### TOBJ.024 - Constraints
Support configurable constraints based on source, time window, object type, quality thresholds, and mission context. This includes both correlation-specific and externally imposed operational constraints.

**Rationale**: Rules vary by scenario and confidence model. PYRAMID's Constraint entity covers any externally imposed limit on how or when information from sources may be used.

## Object Classification (Exploitation: MIL-STD-2525B)

The following requirements instantiate PYRAMID's abstract classification, allegiance, and type attributes (from Specific_Object_Detail and Object_Evidence interfaces) using MIL-STD-2525B field semantics. This is an exploitation design choice (Design Decision D4).

### TOBJ.025 - Battle Dimension
Store the object's battle dimension: ground, air, sea surface, subsurface, space, or SOF.

**Rationale**: Battle dimension is the primary domain classifier in 2525B and drives downstream symbol rendering and filtering.

### TOBJ.026 - Affiliation
Store affiliation: friendly, hostile, neutral, unknown, assumed friend, suspect, joker, faker, or pending.

**Rationale**: Affiliation is a core field and a primary filter for rules of engagement and display.

### TOBJ.027 - Role / Function Identifier
Store a function identifier describing what the entity does, using the 2525B function ID scheme (e.g. infantry, armor, artillery, rotary wing, electronic warfare, logistics, C2, medical).

**Rationale**: Role determines the symbol shape and tactical meaning.

### TOBJ.028 - Status
Store operational status: present, anticipated, planned, or known.

**Rationale**: Status affects the symbol rendering style and operational reasoning.

### TOBJ.029 - Echelon
Store organizational echelon when applicable: team, squad, section, platoon, company, battalion, regiment, brigade, division, corps, army, or higher.

**Rationale**: Echelon determines the size/scale indicator and matters for force aggregation.

### TOBJ.030 - Indicator Flags
Store boolean flags for: headquarters, task force, feint/dummy, and installation.

**Rationale**: These are distinct modifiers that affect symbol rendering and tactical interpretation.

### TOBJ.031 - Mobility Indicator
Store mobility type when applicable: wheeled, tracked, towed, rail, over-snow, sled, barge, amphibious.

**Rationale**: Mobility affects the symbol modifier and movement planning.

### TOBJ.032 - Source Symbol Code Preservation
When a source provides a complete 2525B SIDC (symbol identification coding), preserve it alongside the normalized fields. Allow symbols to be regenerated from normalized fields after correlation or reclassification.

**Rationale**: Source fidelity and internal consistency are both needed.

## Spatial Reasoning (Exploitation Design)

The following requirements support PYRAMID's spatial query and region-based interest capabilities. Zone and region management is an exploitation design choice (Design Decision D5) to deliver `determine_potential_objects`, `Matching_Objects`, and region-based `Object_Interest_Requirements`. Geographic feature ownership remains with the Geography component.

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
Allow optional storage of object facets (kinematics, military classification, correlation state, relationships) so sparse entities do not pay for unused fields.

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
Report the component's ability to maintain state, correlate evidence, answer geographic queries, and supply complete classification data.

**Rationale**: Consumers need confidence in the service itself.

### TOBJ.044 - Missing Information Reporting
Identify missing data that prevents confident classification, correlation, classification completeness, or spatial reasoning.

**Rationale**: Gaps should be visible so sensors or operators can fill them.

### TOBJ.045 - Snapshot and Event Access
Provide both current-state snapshot access and event-oriented update streams.

**Rationale**: Some consumers need authoritative state; others need incremental changes.

### TOBJ.051 - Progress Reporting
Report progress against each active Object_Interest_Requirement, including partial fulfilment, remaining gaps, and estimated time to completion.

**Rationale**: PYRAMID mandates identifying progress against requirements.

### TOBJ.052 - Object Probability Densities
Capture and report the probability densities of particular Tactical_Objects within locations.

**Rationale**: PYRAMID mandates capturing probability densities of objects within locations.

### TOBJ.053 - Capability Progression Prediction
Predict how the component's Capability will change over time and with use, taking account of expected information source availability and operational constraints.

**Rationale**: PYRAMID mandates predicting capability progression.

## Requirement Fulfilment and Dependency Derivation

### TOBJ.046 - Requirement Achievability
Assess whether a tactical object requirement is achievable given current capability, applicable constraints, area of interest, timeliness, and expected evidence availability.

**Rationale**: Active finding requirements need an explicit achievability assessment, not just passive ingestion.

### TOBJ.047 - Tactical Object Solution
Determine an object solution for satisfying a tactical object requirement, including the intended object types, area of interest, timing, predicted quality, and completeness expectations.

**Rationale**: A requirement to actively find entities needs an explicit solution model that can be evaluated and monitored.

### TOBJ.048 - Derived Evidence Requirement
When satisfying a tactical object requirement requires additional evidence, derive one or more evidence requirements that express the needed evidence or supporting information in component-agnostic terms, without naming or embedding another PRA component's responsibilities.

**Rationale**: This allows tactical objects to drive active finding while respecting PYRAMID bridge and component-boundary rules.

### TOBJ.049 - Requirement and Evidence Traceability
Maintain traceability between the source tactical object requirement, any derived evidence requirements, the resulting observations or supporting information, and the correlated objects produced from them.

**Rationale**: Operators and downstream logic need to understand why a search was initiated, what evidence was requested, and what objects resulted.

## Responsibility Traceability

| Requirement | Responsibilities |
| :--- | :--- |
| `TOBJ.001` | `RESP.013` |
| `TOBJ.002` | `RESP.013`, `RESP.007` |
| `TOBJ.003` | `RESP.013` |
| `TOBJ.004` | `RESP.013` |
| `TOBJ.005` | `RESP.001` |
| `TOBJ.006` | `RESP.001` |
| `TOBJ.007` | `RESP.006` |
| `TOBJ.008` | `RESP.006` |
| `TOBJ.009` | `RESP.006` |
| `TOBJ.010` | `RESP.006` |
| `TOBJ.011` | `RESP.009` |
| `TOBJ.012` | `RESP.009` |
| `TOBJ.013` | `RESP.009` |
| `TOBJ.014` | `RESP.009`, `RESP.007` |
| `TOBJ.015` | `RESP.010` |
| `TOBJ.016` | `RESP.010` |
| `TOBJ.017` | `RESP.007`, `RESP.012` |
| `TOBJ.018` | `RESP.013` |
| `TOBJ.019` | `RESP.007` |
| `TOBJ.020` | `RESP.007` |
| `TOBJ.021` | `RESP.006` |
| `TOBJ.022` | `RESP.006`, `RESP.007` |
| `TOBJ.023` | `RESP.006` |
| `TOBJ.024` | `RESP.003` |
| `TOBJ.025` | `RESP.013` |
| `TOBJ.026` | `RESP.013` |
| `TOBJ.027` | `RESP.013` |
| `TOBJ.028` | `RESP.013` |
| `TOBJ.029` | `RESP.013` |
| `TOBJ.030` | `RESP.013` |
| `TOBJ.031` | `RESP.013` |
| `TOBJ.032` | `RESP.013` |
| `TOBJ.033` | `RESP.006` |
| `TOBJ.034` | `RESP.006` |
| `TOBJ.035` | `RESP.006` |
| `TOBJ.036` | `RESP.006` |
| `TOBJ.037` | `RESP.006` |
| `TOBJ.038` | `RESP.006`, `RESP.013` |
| `TOBJ.039` | `RESP.013` |
| `TOBJ.040` | `RESP.006` |
| `TOBJ.041` | `RESP.013` |
| `TOBJ.042` | `RESP.013` |
| `TOBJ.043` | `RESP.015` |
| `TOBJ.044` | `RESP.016` |
| `TOBJ.045` | `RESP.013` |
| `TOBJ.046` | `RESP.004` |
| `TOBJ.047` | `RESP.005` |
| `TOBJ.048` | `RESP.008` |
| `TOBJ.049` | `RESP.008`, `RESP.011` |
| `TOBJ.050` | `RESP.002` |
| `TOBJ.051` | `RESP.011` |
| `TOBJ.052` | `RESP.014` |
| `TOBJ.053` | `RESP.017` |

## Implementation Notes

- Expose a domain model API: `TacticalObject`, `Observation`, `Track`, `Zone`, `Relationship`.
- Use ECS-style sparse components for hot-path storage.
- Attach provenance, confidence, and uncertainty to individual fields, not just the whole object.
- Maintain dedicated indexes for object ID, external references, affiliation, type, time, and spatial lookup.
- Store the normalized classification fields (battle dimension, affiliation, role, status, echelon, flags, mobility) as exploitation design data. Treat symbol code generation as an output concern.
- Represent source tactical object requirements, derived evidence requirements, and their relationships explicitly enough to preserve lineage and support bridge-mediated dependency fulfilment.
- Zone geometry and region reasoning are internal exploitation design; geographic feature authority belongs to the Geography component.
