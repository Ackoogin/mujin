# Tactical Objects Architecture

This document describes the internal architecture of the `tactical_objects` component with visual diagrams and end-to-end worked examples.

For high-level requirements see `REQUIREMENTS.md`. For the concrete class design see `DESIGN.md`. For low-level requirements and traceability see `LLR.md`. For the TDD implementation plan see `TDD_PLAN.md`.

---

## 1. System Context

The `tactical_objects` component sits inside a PYRAMID container managed by PCL. External systems push data in through PCL transport adapters and pull state through PCL services.

```mermaid
graph LR
    subgraph External Systems
        S1[Sensor Feed A]
        S2[Sensor Feed B]
        S3[C2 System]
        S4[Operator Console]
    end

    subgraph PCL Container
        T[Transport Adapter]
        TOC[TacticalObjectsComponent]
    end

    S1 -->|observations| T
    S2 -->|observations| T
    S3 -->|zone updates| T
    T -->|pcl_executor_post_incoming| TOC

    TOC -->|events| T
    TOC -->|capability| T
    T -->|events| S4

    S4 -->|query / CRUD| T
    T -->|service request| TOC
```

---

## 2. Component Layers

The component is split into a thin PCL wrapper and a pure domain runtime that can be tested in isolation.

```mermaid
graph TB
    subgraph TacticalObjectsComponent
        LC[PCL Lifecycle<br/>on_configure / on_activate / on_tick / on_cleanup]
        PORTS[PCL Ports<br/>publishers / subscribers / services]
        CODEC[TacticalObjectsCodec<br/>JSON encode / decode]
    end

    subgraph TacticalObjectsRuntime
        OS[ObjectStore<br/>ECS sparse-set storage]
        SI[SpatialIndex<br/>grid over WGS84]
        FE[FusionEngine<br/>correlate / merge / split]
        ZE[ZoneEngine<br/>geometry + transitions]
        MC[MilClassEngine<br/>2525B field storage]
        QE[QueryEngine<br/>compound predicates]
        EB[EventBus<br/>pyramid::core]
        LG[Logger<br/>pyramid::core]
    end

    LC --> PORTS
    PORTS --> CODEC
    CODEC --> TacticalObjectsRuntime
    OS --- SI
    OS --- FE
    OS --- ZE
    OS --- MC
    OS --- QE
    FE --> EB
    ZE --> EB
```

---

## 3. Object Store (ECS Layout)

The `ObjectStore` uses a sparse-set pattern. Each entity has a UUID. Optional component tables hold facets. Not every entity populates every table.

```mermaid
graph LR
    subgraph Entity Registry
        E1[UUID-001]
        E2[UUID-002]
        E3[UUID-003]
    end

    subgraph Component Tables
        ID[IdentityComponent]
        CL[ClassificationComponent]
        AF[AffiliationComponent]
        KN[KinematicsComponent]
        MC2[MilClassComponent]
        FC[FusionComponent]
        GC[GeometryComponent]
        QC[QualityComponent]
        LC2[LifecycleComponent]
    end

    E1 --> ID
    E1 --> CL
    E1 --> KN
    E1 --> MC2
    E1 --> FC
    E1 --> QC
    E1 --> LC2

    E2 --> ID
    E2 --> AF
    E2 --> GC
    E2 --> LC2

    E3 --> ID
    E3 --> CL
    E3 --> KN
    E3 --> MC2
    E3 --> LC2
```

UUID-001 is a fully-tracked fused entity. UUID-002 is a geographic zone (no kinematics or fusion). UUID-003 is a sparse entity with only identity, classification, kinematics, military classification, and lifecycle.

---

## 4. Two Entity Creation Paths

Entities enter the store through two distinct paths.

```mermaid
flowchart TB
    subgraph Evidence Path
        OBS[Observation arrives<br/>on subscriber] --> DECODE_E[Codec decodes<br/>ObservationBatch]
        DECODE_E --> FE2[FusionEngine<br/>correlate + score]
        FE2 -->|score >= merge| MERGE[Attach to existing<br/>fused object]
        FE2 -->|score < create| NEW[Create new track<br/>+ fused object]
        FE2 -->|sustained incompatibility| SPLIT[Split fused object]
        MERGE --> STORE_E[ObjectStore<br/>update + reindex]
        NEW --> STORE_E
        SPLIT --> STORE_E
    end

    subgraph Direct Path
        REQ[CRUD service request] --> DECODE_D[Codec decodes<br/>ObjectDefinition]
        DECODE_D --> RUNTIME[Runtime.createObject /<br/>updateObject / deleteObject]
        RUNTIME --> STORE_D[ObjectStore<br/>insert or mutate]
    end

    STORE_E --> IDX[SpatialIndex<br/>reindex]
    STORE_D --> IDX
    IDX --> EVENTS[EventBus emits<br/>created / updated / deleted]
```

Evidence-path objects carry `fused` provenance and retain lineage to contributing observations and tracks. Direct-path objects carry `direct` provenance and bypass correlation.

Both paths feed the same `ObjectStore` and are queryable through the same `QueryEngine`.

---

## 5. Fusion Pipeline Detail

```mermaid
sequenceDiagram
    participant Sub as Subscriber Callback
    participant Codec as TacticalObjectsCodec
    participant RT as TacticalObjectsRuntime
    participant SI as SpatialIndex
    participant FE as FusionEngine
    participant OS as ObjectStore
    participant MC as MilClassEngine
    participant EB as EventBus

    Sub->>Codec: raw bytes
    Codec->>RT: ObservationBatch
    loop each observation
        RT->>SI: queryCandidates(position, gate_radius)
        SI-->>RT: candidate UUIDs
        RT->>FE: correlate(observation, candidates)
        FE->>FE: score each candidate
        alt score >= merge_threshold
            FE->>OS: attachObservation(existing_id, obs)
            FE->>MC: updateClassification(existing_id, obs hints)
            FE-->>RT: MergeResult
        else score < create_threshold
            FE->>OS: createTrack(obs)
            FE->>OS: createFusedObject(track)
            FE->>MC: setClassification(new_id, obs hints)
            FE-->>RT: CreateResult
        else sustained incompatibility
            FE->>OS: splitFusedObject(source_id, divergent_track)
            FE-->>RT: SplitResult
        end
        RT->>SI: reindex(affected_id)
        RT->>EB: publish(ObjectEvent)
    end
```

---

## 6. Zone Relationship Evaluation

```mermaid
sequenceDiagram
    participant RT as TacticalObjectsRuntime
    participant ZE as ZoneEngine
    participant SI as SpatialIndex
    participant OS as ObjectStore
    participant EB as EventBus

    RT->>ZE: evaluateZoneRelationships(object_id)
    ZE->>OS: getPosition(object_id)
    OS-->>ZE: lat/lon/alt
    ZE->>SI: queryZoneCandidates(position)
    SI-->>ZE: candidate zone UUIDs
    loop each zone candidate
        ZE->>OS: getZoneGeometry(zone_id)
        OS-->>ZE: geometry
        ZE->>ZE: exactContainmentCheck(position, geometry)
        ZE->>ZE: compareWithPriorState(object_id, zone_id)
        alt transition detected
            ZE->>EB: publish(ZoneEnter or ZoneLeave)
        end
    end
    ZE-->>RT: updated relationships
```

---

## 7. Query Flow

```mermaid
sequenceDiagram
    participant Svc as Service Handler
    participant Codec as TacticalObjectsCodec
    participant RT as TacticalObjectsRuntime
    participant QE as QueryEngine
    participant SI as SpatialIndex
    participant OS as ObjectStore

    Svc->>Codec: raw request bytes
    Codec->>RT: QueryRequest
    RT->>QE: evaluate(request)
    alt has spatial predicate
        QE->>SI: regionCandidates(bounds)
        SI-->>QE: candidate UUIDs
        QE->>QE: exactSpatialFilter(candidates)
    end
    alt has attribute predicates
        QE->>OS: filterByPredicates(type, affiliation, role, ...)
        OS-->>QE: matching UUIDs
    end
    QE->>QE: intersect result sets
    QE->>OS: hydrate(final UUIDs)
    OS-->>QE: full objects
    QE-->>RT: QueryResponse
    RT-->>Codec: QueryResponse
    Codec-->>Svc: serialized response
```

---

## 8. PCL Lifecycle

```mermaid
stateDiagram-v2
    [*] --> Unconfigured
    Unconfigured --> Configured: on_configure()
    Configured --> Active: on_activate()
    Active --> Active: on_tick(dt)
    Active --> Configured: on_deactivate()
    Configured --> Unconfigured: on_cleanup()

    state Configured {
        [*] --> PortsCreated
        PortsCreated --> RuntimeReady
        note right of RuntimeReady
            runtime_ constructed
            codec_ constructed
            all ports created
            tick rate configured
        end note
    }

    state Active {
        [*] --> Processing
        Processing --> Processing: subscriber callbacks
        Processing --> Processing: service handlers
        Processing --> Processing: tick housekeeping
        note right of Processing
            capability published
            stale expiry runs
            zone transitions checked
        end note
    }
```

---

## 9. Military Classification Data Flow

```mermaid
flowchart LR
    subgraph Input Sources
        OBS_SIDC[Observation with<br/>source SIDC]
        OBS_HINTS[Observation with<br/>classification hints]
        DIRECT[Direct CRUD with<br/>explicit fields]
    end

    subgraph MilClassEngine
        PARSE[Parse / normalize<br/>input fields]
        STORE_MC[Store normalized fields<br/>battle_dimension<br/>affiliation<br/>role<br/>status<br/>echelon<br/>mobility<br/>flags]
        PRESERVE[Preserve source SIDC<br/>if provided]
        DERIVE[Derive symbol key<br/>from stored fields]
    end

    subgraph Output
        PROFILE[MilClassProfile]
        KEY[Symbol Key]
    end

    OBS_SIDC --> PARSE
    OBS_HINTS --> PARSE
    DIRECT --> PARSE
    PARSE --> STORE_MC
    PARSE --> PRESERVE
    STORE_MC --> DERIVE
    DERIVE --> KEY
    STORE_MC --> PROFILE
    PRESERVE --> PROFILE
```

The MilClassEngine stores normalized semantic fields. Source SIDCs are preserved but never used as the authoritative representation. Symbol keys can be regenerated after any field change.

---

## 10. End-to-End Examples

### 10.1 Radar Detection Creates a New Entity (Evidence Path)

**Scenario**: A radar feed submits an observation of an unknown aircraft. No existing entity matches.

```mermaid
sequenceDiagram
    participant Radar as Radar Feed
    participant Trans as PCL Transport
    participant Comp as TacticalObjectsComponent
    participant Codec as Codec
    participant RT as Runtime
    participant SI as SpatialIndex
    participant FE as FusionEngine
    participant OS as ObjectStore
    participant MC as MilClassEngine
    participant EB as EventBus
    participant Pub as Event Publisher

    Radar->>Trans: JSON observation<br/>{source: "radar-1", pos: [51.5, -1.2], type_hint: "air", confidence: 0.7}
    Trans->>Comp: subscriber callback (raw bytes)
    Comp->>Codec: decode(raw bytes)
    Codec-->>Comp: ObservationBatch [1 obs]
    Comp->>RT: upsertObservationBatch(batch)
    RT->>SI: queryCandidates(51.5, -1.2, gate=50km)
    SI-->>RT: [] (no candidates)
    RT->>FE: correlate(obs, [])
    FE->>OS: createTrack(obs)
    OS-->>FE: track_id = UUID-T1
    FE->>OS: createFusedObject(track=UUID-T1)
    OS-->>FE: object_id = UUID-001
    FE-->>RT: CreateResult{object_id=UUID-001}
    RT->>MC: setClassification(UUID-001, battle_dim=AIR, affiliation=UNKNOWN)
    RT->>SI: index(UUID-001, 51.5, -1.2)
    RT->>EB: publish(ObjectCreated{UUID-001})
    Comp->>Pub: publish event JSON<br/>{type: "object_created", id: "UUID-001"}
```

**Result**: A new fused object UUID-001 exists in the store with battle dimension AIR, affiliation UNKNOWN, confidence 0.7, and a single track UUID-T1 backed by one observation.

---

### 10.2 Second Sensor Confirms the Same Entity (Evidence Path - Merge)

**Scenario**: An ADS-B feed reports the same aircraft. The observation is close enough to merge.

```mermaid
sequenceDiagram
    participant ADSB as ADS-B Feed
    participant Trans as PCL Transport
    participant Comp as TacticalObjectsComponent
    participant RT as Runtime
    participant SI as SpatialIndex
    participant FE as FusionEngine
    participant OS as ObjectStore
    participant MC as MilClassEngine
    participant EB as EventBus

    ADSB->>Trans: JSON observation<br/>{source: "adsb", pos: [51.501, -1.199], callsign: "BAW123", affiliation: "friendly"}
    Trans->>Comp: subscriber callback
    Comp->>RT: upsertObservationBatch(batch)
    RT->>SI: queryCandidates(51.501, -1.199, gate=50km)
    SI-->>RT: [UUID-001]
    RT->>FE: correlate(obs, [UUID-001])
    FE->>FE: score(UUID-001, obs) = 0.92
    Note over FE: 0.92 >= merge_threshold (0.8)
    FE->>OS: attachObservation(UUID-001, obs)
    FE->>OS: addSourceRef(UUID-001, "adsb", "BAW123")
    FE-->>RT: MergeResult{object_id=UUID-001}
    RT->>MC: updateClassification(UUID-001, affiliation=FRIENDLY)
    RT->>SI: reindex(UUID-001, 51.501, -1.199)
    RT->>EB: publish(ObjectUpdated{UUID-001})
    RT->>EB: publish(ObjectFused{UUID-001, sources=["radar-1", "adsb"]})
```

**Result**: UUID-001 now has two source references (radar-1 and adsb), affiliation upgraded to FRIENDLY, higher confidence from dual-source corroboration, and lineage tracing back to both observations.

---

### 10.3 Operator Creates an Entity Directly (Direct Path)

**Scenario**: An operator manually plots a known hostile ground unit at a specific location.

```mermaid
sequenceDiagram
    participant Op as Operator Console
    participant Trans as PCL Transport
    participant Comp as TacticalObjectsComponent
    participant Codec as Codec
    participant RT as Runtime
    participant OS as ObjectStore
    participant MC as MilClassEngine
    participant SI as SpatialIndex
    participant EB as EventBus

    Op->>Trans: service request<br/>{type: "create", object_type: "unit",<br/>position: [34.0, 44.3],<br/>battle_dimension: "ground",<br/>affiliation: "hostile",<br/>role: "armor",<br/>echelon: "battalion",<br/>status: "present"}
    Trans->>Comp: service handler (create_object)
    Comp->>Codec: decode(request)
    Codec-->>Comp: ObjectDefinition
    Comp->>RT: createObject(definition)
    RT->>OS: insertObject(definition, provenance=DIRECT)
    OS-->>RT: object_id = UUID-050
    RT->>MC: setClassification(UUID-050,<br/>battle_dim=GROUND, affiliation=HOSTILE,<br/>role=ARMOR, echelon=BATTALION,<br/>status=PRESENT)
    RT->>SI: index(UUID-050, 34.0, 44.3)
    RT->>EB: publish(ObjectCreated{UUID-050, provenance=DIRECT})
    Comp-->>Op: response {id: "UUID-050", status: "created"}
```

**Result**: UUID-050 exists in the store with `direct` provenance, full military classification, and is queryable alongside evidence-based entities.

---

### 10.4 Zone Creation and Boundary Alert

**Scenario**: A no-go zone is created. An existing entity is evaluated and found inside.

```mermaid
sequenceDiagram
    participant C2 as C2 System
    participant Trans as PCL Transport
    participant Comp as TacticalObjectsComponent
    participant RT as Runtime
    participant ZE as ZoneEngine
    participant OS as ObjectStore
    participant SI as SpatialIndex
    participant EB as EventBus
    participant Pub as Event Publisher

    C2->>Trans: service request<br/>{type: "upsert_zone", zone_type: "no_go",<br/>geometry: {type: "polygon", coords: [...]},<br/>active_from: "2026-03-13T08:00Z",<br/>active_until: "2026-03-14T08:00Z"}
    Trans->>Comp: service handler (upsert_zone)
    Comp->>RT: upsertZone(definition)
    RT->>OS: insertZone(definition)
    OS-->>RT: zone_id = UUID-Z1
    RT->>SI: indexZone(UUID-Z1, bbox)

    Note over RT: Evaluate existing objects against new zone
    RT->>ZE: evaluateAllAgainstZone(UUID-Z1)
    ZE->>SI: queryObjectsInBbox(zone_bbox)
    SI-->>ZE: [UUID-001, UUID-050]
    loop each candidate
        ZE->>OS: getPosition(candidate)
        ZE->>ZE: pointInPolygon(position, zone_geometry)
    end
    Note over ZE: UUID-050 is inside the polygon
    ZE->>EB: publish(ZoneEnter{object=UUID-050, zone=UUID-Z1})
    Comp->>Pub: publish event<br/>{type: "zone_enter", object: "UUID-050", zone: "UUID-Z1", zone_type: "no_go"}
```

**Result**: Zone UUID-Z1 is stored with temporal validity. UUID-050 triggered a `zone_enter` event because it falls inside the polygon. Downstream consumers can alert on hostile units inside no-go zones.

---

### 10.5 Query for All Hostile Ground Units Near a Zone

**Scenario**: An operator queries for all hostile ground units within 10 km of the no-go zone.

```mermaid
sequenceDiagram
    participant Op as Operator Console
    participant Trans as PCL Transport
    participant Comp as TacticalObjectsComponent
    participant Codec as Codec
    participant RT as Runtime
    participant QE as QueryEngine
    participant SI as SpatialIndex
    participant OS as ObjectStore

    Op->>Trans: service request<br/>{type: "query",<br/>predicates: {<br/>  affiliation: "hostile",<br/>  battle_dimension: "ground",<br/>  near_zone: "UUID-Z1",<br/>  distance_km: 10<br/>}}
    Trans->>Comp: service handler (query)
    Comp->>Codec: decode(request)
    Codec-->>Comp: QueryRequest
    Comp->>RT: query(request)
    RT->>QE: evaluate(request)
    QE->>SI: regionCandidates(zone_bbox expanded by 10km)
    SI-->>QE: [UUID-001, UUID-050, UUID-051, UUID-052]
    QE->>QE: exactDistanceFilter(candidates, zone_geometry, 10km)
    Note over QE: UUID-001 is 45km away, filtered out
    QE->>OS: filterByPredicates(affiliation=HOSTILE, battle_dim=GROUND)
    Note over QE: UUID-051 is friendly, filtered out
    QE->>QE: intersect spatial + attribute results
    QE->>OS: hydrate([UUID-050, UUID-052])
    OS-->>QE: full object data
    QE-->>RT: QueryResponse{objects=[UUID-050, UUID-052], total=2}
    RT-->>Comp: QueryResponse
    Comp->>Codec: encode(response)
    Codec-->>Comp: JSON bytes
    Comp-->>Op: response<br/>{objects: [{id: "UUID-050", ...}, {id: "UUID-052", ...}], total: 2}
```

**Result**: The query returns exactly the two hostile ground entities within 10 km of the no-go zone. Spatial index pruned the candidate set before attribute filtering.

---

### 10.6 Tick-Based Stale Object Expiry

**Scenario**: No updates arrive for a tracked entity for longer than the configured timeout.

```mermaid
sequenceDiagram
    participant PCL as PCL Executor
    participant Comp as TacticalObjectsComponent
    participant RT as Runtime
    participant OS as ObjectStore
    participant SI as SpatialIndex
    participant EB as EventBus

    Note over PCL: 5 minutes since last update to UUID-001
    PCL->>Comp: on_tick(dt=1.0)
    Comp->>RT: tick({wall_time=T, dt=1.0})
    RT->>OS: findStaleObjects(threshold=300s, now=T)
    OS-->>RT: [UUID-001]
    RT->>OS: setLifecycleStatus(UUID-001, STALE)
    RT->>SI: deindex(UUID-001)
    RT->>EB: publish(ObjectStale{UUID-001, last_update=T-300})

    Note over PCL: 10 more minutes, still no update
    PCL->>Comp: on_tick(dt=1.0)
    Comp->>RT: tick({wall_time=T+600, dt=1.0})
    RT->>OS: findRetirableObjects(stale_age=600s, now=T+600)
    OS-->>RT: [UUID-001]
    RT->>OS: retire(UUID-001)
    RT->>EB: publish(ObjectRetired{UUID-001})
```

**Result**: UUID-001 transitions from active to stale after 5 minutes without updates, then is retired after 10 more minutes. Its spatial index entry is removed when it becomes stale to avoid false query results.

---

## 11. Module Dependency Graph

```mermaid
graph BT
    subgraph pyramid::core
        UUID[UUID / UUIDHelper]
        EB2[EventBus]
        LOG[Logger]
    end

    subgraph tactical_objects
        TYPES[TacticalObjectsTypes]
        OS2[ObjectStore]
        SI2[SpatialIndex]
        MC3[MilClassEngine]
        ZE2[ZoneEngine]
        FE3[FusionEngine]
        QE2[QueryEngine]
        RT2[TacticalObjectsRuntime]
        CODEC2[TacticalObjectsCodec]
        COMP[TacticalObjectsComponent]
    end

    subgraph External
        PCL[pcl::Component]
        JSON[nlohmann::json]
    end

    TYPES --> UUID
    OS2 --> TYPES
    OS2 --> UUID
    SI2 --> TYPES
    MC3 --> TYPES
    ZE2 --> SI2
    ZE2 --> OS2
    ZE2 --> EB2
    FE3 --> SI2
    FE3 --> OS2
    FE3 --> MC3
    FE3 --> EB2
    QE2 --> OS2
    QE2 --> SI2
    RT2 --> OS2
    RT2 --> SI2
    RT2 --> FE3
    RT2 --> ZE2
    RT2 --> MC3
    RT2 --> QE2
    RT2 --> EB2
    RT2 --> LOG
    CODEC2 --> TYPES
    CODEC2 --> JSON
    COMP --> RT2
    COMP --> CODEC2
    COMP --> PCL
```

---

## 12. Source File Map

| Module | Header | Source | Test |
|--------|--------|--------|------|
| Types | `include/TacticalObjectsTypes.h` | (header-only) | (covered by other tests) |
| ObjectStore | `include/store/ObjectStore.h` | `src/store/ObjectStore.cpp` | `Test_ObjectStore.cpp` |
| ObjectComponents | `include/store/ObjectComponents.h` | (header-only) | (covered by ObjectStore tests) |
| SpatialIndex | `include/spatial/SpatialIndex.h` | `src/spatial/SpatialIndex.cpp` | `Test_SpatialIndex.cpp` |
| FusionEngine | `include/fusion/FusionEngine.h` | `src/fusion/FusionEngine.cpp` | `Test_FusionEngine.cpp` |
| MilClassEngine | `include/milclass/MilClassEngine.h` | `src/milclass/MilClassEngine.cpp` | `Test_MilClassEngine.cpp` |
| ZoneEngine | `include/zone/ZoneEngine.h` | `src/zone/ZoneEngine.cpp` | `Test_ZoneEngine.cpp` |
| QueryEngine | `include/query/QueryEngine.h` | `src/query/QueryEngine.cpp` | `Test_QueryEngine.cpp` |
| Runtime | `include/TacticalObjectsRuntime.h` | `src/TacticalObjectsRuntime.cpp` | `Test_TacticalObjectsRuntime.cpp` |
| Codec | `include/TacticalObjectsCodec.h` | `src/TacticalObjectsCodec.cpp` | `Test_TacticalObjectsComponent.cpp` |
| Component | `include/TacticalObjectsComponent.h` | `src/TacticalObjectsComponent.cpp` | `Test_TacticalObjectsComponent.cpp` |

All paths are relative to `pyramid/tactical_objects/`. Tests are under `tests/tactical_objects/`.
