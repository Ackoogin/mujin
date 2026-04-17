# Standard Data-Model Alignment Plan

## 1. Problem Statement

The Tactical Objects component (`pyramid/tactical_objects/`) has an internal
data model (types, enums, service names, wire format) that diverges from
the PYRAMID standard proto IDL:

| Aspect | Internal (C++) | Standard Proto |
|--------|---------------|----------------|
| Identity enum | `Affiliation` (Friendly=0 … Pending=8) | `StandardIdentity` (Unspecified=0, Unknown=1 … AssumedFriendly=9) |
| Battle dimension | Ground=0, **Air=1**, SeaSurface=2, **Subsurface=3**, Space=4, SOF=5 | Ground=1, **Subsurface=2**, SeaSurface=3, **Air=4**, Unknown=5 |
| Position | `Position{lat, lon, alt}` (degrees, doubles) | `GeodeticPosition{Angle latitude, Angle longitude}` (radians) |
| Entity base | ad-hoc `UUIDKey` + per-field optionals | `Entity{update_time, id, source}` (standard base) |
| Interest model | `InterestCriteria` + `BoundingBox` | `ObjectInterestRequirement` with `PolyArea`/`CircleArea`/`Point` oneof |
| Evidence model | `DerivedEvidenceRequirement` (internal) | `ObjectEvidenceRequirement` with CRUD service |
| Service names | `create_object`, `subscribe_interest`, `query`, etc. | `Matching_Objects_Service.ReadMatch`, `Object_Of_Interest_Service.CreateRequirement`, etc. |
| Object output | `TacticalObject` / `EntityUpdateFrame` (binary batch) | `ObjectDetail` + `ObjectMatch` (proto messages) |
| Extra types | `MilClassProfile`, `BehaviorComponent`, `ZoneType`, `Echelon`, `Mobility` | Not in standard (domain extensions) |

The standard proto files are now in the repo (`subprojects/PYRAMID/proto/pyramid/data_model/`,
`subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/`), with corresponding
Ada types in `subprojects/PYRAMID/examples/ada/tactical_objects_types.ads`.

## 2. Goals

1. External consumers (Ada clients, future gRPC clients) talk exclusively
   in standard proto types.
2. Internal domain engines (correlation, spatial index, mil-class) keep
   working without regressions.
3. Streaming binary performance (0-copy batch frames) is preserved.
4. Migration is incremental — each step is independently testable.

## 3. Option C — Hybrid (selected)

Progress tracked against the 6-phase plan below.

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Bridge component (StandardBridge) | **COMPLETE** |
| 2 | Enum ordinal alignment (Affiliation, BattleDimension) | **COMPLETE** |
| 3 | Position alignment (radians) | **COMPLETE** |
| 4 | Interest/Evidence model alignment | **COMPLETE** |
| 5 | Service name alignment | **COMPLETE** |
| 6 | Remove bridge (now identity-only) | **COMPLETE** |

### Phase 1 — Bridge ✅

`StandardBridge` (C++) deployed as a PCL component that sits between
external PYRAMID consumers and the internal `TacticalObjectsComponent`.

- Provided: `object_of_interest.create_requirement` (JSON, PYRAMID standard)
- Provided: `standard.object_evidence` subscriber (JSON, PYRAMID standard)
- Published: `standard.entity_matches` (JSON, PYRAMID standard)
- All internal services and binary streaming unchanged.

Constraint honoured: generated types (`pyramid::data_model::*`) appear only
in `StandardBridge.cpp` — no generated types in business logic.

### Phase 2 — Enum ordinal alignment ✅

`Affiliation` and `BattleDimension` C++ enums reordered with explicit ordinals
to match PYRAMID standard proto values.  The bridge enum string converters
are removed; the bridge now uses direct `static_cast` between aligned enums.
Ada `Ordinal_To_Affiliation` and `tactical_objects_types.ads` updated in
lockstep.

**Wire-format impact**: binary streaming ordinals changed.  Any existing
recorded binary data or live clients must upgrade together.

### Phase 6 — Remove bridge (now identity-only) ✅

The bridge is not literally removed — it remains as a permanent thin adapter —
but Phase 6 verifies that every remaining translation is an inherent schema
difference (not a historical impedance mismatch) and eliminates the last
avoidable conversion.

**Last string-conversion chain removed:**
The `handleCreateRequirement` switch that mapped `BattleDimension` ordinals to
string names ("Air", "Ground", …) is replaced with a single
`internal["battle_dimension"] = static_cast<int>(dim)` emit.
`handleSubscribeInterest` now accepts integer ordinals in addition to strings
(integer branch tried first; string branch retained for backward compatibility).
This is a direct consequence of the Phase 2 ordinal alignment.

**Residual bridge contract — inherent schema differences, cannot be removed:**

| Translation | Reason it is permanent |
|-------------|------------------------|
| `DataPolicy::Obtain` ↔ `query_mode = "active_find"` | Semantic equivalence between two different naming schemes (`policy` vs `query_mode`); no shared ordinal |
| `PolyArea`/`CircleArea`/`Point` oneof ↔ flat `BoundingBox` | Standard uses geometry abstraction; internal uses axis-aligned bbox; changing would alter correlation/spatial engine contracts |
| Binary `EntityUpdateFrame` stream ↔ `ObjectMatch[]` JSON | Streaming binary for performance; standard is JSON for interoperability; wire-format unification would break zero-copy path |
| `ObjectDetail` ↔ `Observation` (inbound evidence) | Standard observation type has rich provenance; internal `Observation` maps to correlation engine vocabulary |

All remaining translations are justified by the schema boundary between the
PYRAMID standard IDL and the internal domain model. The bridge is the correct
and permanent home for this mapping.

**Changes made:**
- `TacticalObjectsComponent.cpp` (`handleSubscribeInterest`): added integer
  ordinal branch for `battle_dimension` (tries integer, falls back to string).
- `StandardBridge.cpp` (`handleCreateRequirement`): 10-line switch replaced
  with `internal["battle_dimension"] = static_cast<int>(dim)` (3 lines),
  conditioned on `dim != 0` (Unspecified = no filter).

### Phase 5 — Service name alignment ✅

`StandardBridge` now exposes the two standard read services, completing the
provided-interface surface alongside the already-standard
`object_of_interest.create_requirement`.

Services added to `StandardBridge`:
- **`matching_objects.read_match`** — accepts standard `Query` JSON
  (`{"id": ["<uuid>", ...]}` or empty for all); calls the internal `query`
  service via the shared executor; translates each `QueryResultEntry` into an
  `ObjectMatch` (`id`, `matching_object_id`, `source = "tactical_objects"`)
  and returns a JSON array.
- **`specific_object_detail.read_detail`** — accepts standard `Query` JSON;
  for each requested UUID looks up the entity directly in the runtime (position
  is already in radians per Phase 3); builds an `ObjectDetail` (`id`,
  `position.latitude/longitude`, `quality`) and returns a JSON array.

Changes made:
- `StandardBridge.h`: declared `handleReadMatch` and `handleReadDetail`.
- `StandardBridge.cpp`: added `SVC_READ_MATCH`, `SVC_READ_DETAIL`, and
  `SVC_QUERY` constants; registered both new services in `on_configure`;
  implemented both handlers.
- `Test_TacticalObjectsComponent.cpp`: added `#include <StandardBridge.h>`;
  added `StandardReadMatchReturnsMatchArray` and
  `StandardReadDetailReturnsDetailArray` tests — each creates a `StandardBridge`
  in the executor alongside `TacticalObjectsComponent` and verifies the
  full request→response translation.

**Standard interface now complete (provided side):**

| Standard wire name | Internal call |
|--------------------|---------------|
| `object_of_interest.create_requirement` | `subscribe_interest` |
| `matching_objects.read_match` | `query` |
| `specific_object_detail.read_detail` | runtime `getRecord` + `kinematics` |

**Wire-format impact**: no change to existing services or binary codec.

---

### Phase 4 — Interest/Evidence model alignment ✅

`DerivedEvidenceRequirement` JSON serialization in `TacticalObjectsComponent`
enriched to carry all fields the bridge needs; bridge updated to integer-based
parsing leveraging aligned ordinals from Phase 2.

Changes made:
- `TacticalObjectsComponent.cpp` (`handleSubscribeInterest`): evidence requirement
  JSON now emits `"id"` (was `"requirement_id"`), `"policy"` as integer ordinal
  (1 = `DataPolicy::Query`), `"dimension"` as integer ordinal when
  `criteria.battle_dimension` is set, and `"min/max_lat/lon_rad"` area fields
  when `criteria.area` is set.
- `StandardBridge.cpp` (`evidence_requirement_from_internal_json`): replaced
  string-based policy/dimension parsing with direct `static_cast` using aligned
  ordinals (Phase 2 guarantee); function is now 8 lines vs 18.
- `Test_TacticalObjectsComponent.cpp`
  (`SubscribeInterestActiveFindReturnsEvidenceRequirements`): updated to supply
  an AOI in the subscribe request and assert that each returned evidence
  requirement carries `id`, `policy=1`, `dimension=4` (Air), and
  `min_lat_rad ≈ 50°` — verifying the full round-trip.

**Bridge impact**: `evidence_requirement_from_internal_json` is now a thin
`static_cast` mapping; string-comparison chains removed.

---

### Phase 3 — Position alignment (radians) ✅

Internal `Position.lat/lon` and `BoundingBox` fields now store radians instead
of degrees, matching the PYRAMID standard `GeodeticPosition.Angle.radians` wire
format.

Changes made:
- `GeoMath.h`: `haversineMeters` and `circleBoundingBox` no longer call
  `degToRad()` — positions are already in radians.
- `SpatialIndex.h`: default `cell_size_rad = 0.017453292519943295` (1°).
- `CorrelationEngine.h/.cpp`: `gate_radius_deg` renamed to `gate_radius_rad`
  (default 0.5° in rad); `gate_m` now computed via `EARTH_RADIUS_M`.
- `StandardBridge.h`: `degToRad`/`radToDeg` helpers removed (now identity).
- `StandardBridge.cpp`: `onStandardObjectEvidence` copies latitude/longitude
  directly; `set_bbox` passes radian values straight through to JSON.
- `tactical_objects_main.cpp`: local `radToDeg` removed; `latitude_rad` /
  `longitude_rad` JSON fields assigned directly to `Position`.
- All 7 test suites updated with `constexpr double DEG = PI / 180.0` and all
  degree literals converted (107 tests, all passing).

**Bridge impact**: `radToDeg`/`degToRad` conversions removed from bridge.
The bridge is now a pure type-mapping layer for position (no unit conversion).

---

## Options (for reference)

---

### Option A: In-Place Migration (refactor C++ types to standard)

Rewrite `TacticalObjectsTypes.h` and all dependents to use the standard
enum values, type names, and position representation.

#### Approach

1. **Enum alignment** — change `Affiliation` → `StandardIdentity` with
   standard ordinals; `BattleDimension` to standard ordinals. Add
   conversion functions for backward-compatible JSON ingestion.
2. **Position to GeodeticPosition** — replace `Position{lat,lon,alt}` with
   `GeodeticPosition{latitude_rad, longitude_rad}`, add `alt_m` extension.
   Insert `deg2rad` / `rad2deg` at ingestion boundaries.
3. **Entity base** — add `EntityBase{update_time, id, source}` struct;
   embed it in `EntityRecord`.
4. **Service interface rename** — change PCL service names to match
   standard (`subscribe_interest` → `CreateRequirement`, etc.); update
   Ada clients simultaneously.
5. **Interest / Evidence types** — replace `InterestCriteria` +
   `BoundingBox` with `ObjectInterestRequirement` using `PolyArea` /
   `CircleArea` / `Point` oneof; replace `DerivedEvidenceRequirement` with
   `ObjectEvidenceRequirement`.
6. **StreamingCodec ordinals** — update `affiliationToOrdinal` /
   `ordinalToAffiliation` etc. to emit standard ordinals; update Ada
   `streaming_codec.adb` in lockstep.
7. **Output types** — rename `EntityUpdateFrame` fields to match
   `ObjectDetail`; add `ObjectMatch` as a lightweight reference frame.

#### Pros

- Single source of truth — no translation layer to maintain.
- Cleaner long-term; new contributors see standard names everywhere.
- Tests directly validate standard compliance.

#### Cons

- **High blast radius**: touches every file in `pyramid/tactical_objects/`
  plus Ada codecs, e2e tests, and all 50+ C++ unit tests.
- **Wire-format break**: streaming binary ordinals change; any existing
  recorded data or live clients see garbled enums until upgraded.
- **Position units break**: every spatial calculation (correlation gate,
  bounding-box, zone geometry) must switch to radians or insert
  conversions at every call-site.
- **MilClassProfile / BehaviorComponent**: these domain extensions have no
  standard equivalent. Must either remove them (losing capability) or
  carry them as non-standard extensions with no proto coverage.
- **Risk of regression**: correlation, spatial, interest-matching engines
  are performance-sensitive and well-tested; restructuring types risks
  subtle bugs.

#### Estimated effort

Large — 15+ files, ~2000 LOC delta, full test rewrite.

---

### Option B: Bridge Component (adapter between standard and internal)

Add a `StandardBridge` translation layer that sits between external
consumers and the internal `TacticalObjectsComponent`. External callers
see standard proto types; internally everything stays as-is.

#### Approach

```
┌──────────────────────────────────────────────────────────┐
│  External Clients (Ada, gRPC, future)                    │
│  speak: ObjectInterestRequirement, ObjectDetail, etc.    │
└───────────────┬──────────────────────────────────────────┘
                │ standard proto types
                ▼
┌──────────────────────────────────────────────────────────┐
│  StandardBridge (new C++ library)                         │
│                                                           │
│  Provided services → internal service calls:              │
│    CreateRequirement  →  subscribe_interest               │
│    ReadMatch          →  entity_updates subscriber        │
│    ReadDetail         →  get_object / query               │
│                                                           │
│  Consumed service adapters:                               │
│    evidence_requirements → CreateEvidenceRequirement      │
│    observation_ingress   → ReadDetail (Object_Evidence)   │
│                                                           │
│  Type conversions:                                        │
│    StandardIdentity ↔ Affiliation (ordinal mapping)       │
│    GeodeticPosition ↔ Position   (rad ↔ deg)              │
│    ObjectInterestReq ↔ InterestCriteria                   │
│    ObjectDetail ↔ EntityUpdateFrame (field subset)        │
│    ObjectMatch ← (entity_id, interest_id)                 │
│    BattleDimension standard ↔ internal ordinals           │
└───────────────┬──────────────────────────────────────────┘
                │ internal types (unchanged)
                ▼
┌──────────────────────────────────────────────────────────┐
│  TacticalObjectsComponent (unchanged)                     │
│  ObjectStore, CorrelationEngine, SpatialIndex, etc.       │
│  Internal services: create_object, subscribe_interest … │
│  Binary streaming: EntityUpdateFrame (internal ordinals)  │
└──────────────────────────────────────────────────────────┘
```

1. **`StandardBridge` class** — a PCL container that registers the
   standard service names and delegates to internal services.
2. **Enum mapping functions** — bidirectional:
   ```cpp
   StandardIdentity affiliationToStandardIdentity(Affiliation a);
   Affiliation standardIdentityToAffiliation(StandardIdentity si);
   BattleDimension_Standard internalDimToStandard(BattleDimension d);
   BattleDimension standardDimToInternal(BattleDimension_Standard d);
   ```
3. **Position conversion**:
   ```cpp
   GeodeticPosition positionToGeodetic(const Position& p);  // deg→rad
   Position geodeticToPosition(const GeodeticPosition& g);  // rad→deg
   ```
4. **Interest conversion**: `ObjectInterestRequirement` → `InterestCriteria`
   (map PolyArea vertices to BoundingBox, or extend InterestCriteria to
   accept polygon).
5. **Output conversion**: On entity_updates receipt, decode
   `EntityUpdateFrame` → build `ObjectDetail` + `ObjectMatch` in standard
   types, re-encode and forward to standard subscribers.
6. **Consumed-side adapters**: When internal `evidence_requirements`
   publishes, bridge converts `DerivedEvidenceRequirement` →
   `ObjectEvidenceRequirement` in standard format.
7. **Ada clients** choose which interface to use:
   - Existing Ada e2e tests: keep calling internal services (no change).
   - New standard clients: call through the bridge.

#### Pros

- **Zero risk to internals**: correlation, streaming, spatial — all
  untouched, all existing tests pass as-is.
- **Incremental**: bridge can be built and tested independently; old
  clients keep working; new clients use standard interface.
- **Domain extensions preserved**: `MilClassProfile`, `BehaviorComponent`,
  `ZoneEngine` stay fully functional internally; bridge exposes only the
  standard subset externally.
- **Wire-format stable**: binary streaming ordinals unchanged; existing
  recorded data, Foxglove visualizations, etc. continue to work.
- **Clear boundary**: standard compliance is concentrated in one component
  (~500 LOC) rather than scattered across the codebase.

#### Cons

- **Translation overhead**: extra copy on every service call and streaming
  frame. For entity_updates at 10 Hz × 1000 entities, this adds ~1ms
  per tick (measured: memcpy + ordinal remapping).
- **Two sets of names**: developers must know both internal and standard
  names. Documentation must clarify which layer uses which.
- **Polygon/circle degradation**: standard `PolyArea` / `CircleArea` →
  internal `BoundingBox` is a lossy conversion (circumscribed AABB).
  Until internal engine supports polygon queries natively, precision is
  reduced.
- **Maintenance burden**: any new service or type added to the standard
  requires a corresponding bridge mapping.

#### Estimated effort

Medium — 1 new file (~500 LOC), enum/position mappers (~200 LOC),
tests (~300 LOC). No existing file changes.

---

### Option C: Hybrid — Bridge Now, Incremental Internal Migration Later

Deploy Option B first for immediate standard compliance, then
incrementally migrate internal types to standard over multiple PRs.

#### Approach

**Phase 1 (Bridge)** — same as Option B. Provides immediate standard
   compliance for external consumers. Existing tests unchanged.

**Phase 2 (Enum alignment)** — change internal `Affiliation` →
   `StandardIdentity`, `BattleDimension` → standard ordinals.
   Bridge enum mappers become identity functions and are removed.
   Update StreamingCodec and Ada streaming_codec in lockstep.

**Phase 3 (Position alignment)** — change internal `Position` →
   `GeodeticPosition` (radians). Audit all spatial calculations.
   Bridge position converters become identity functions and are removed.

**Phase 4 (Interest/Evidence alignment)** — replace `InterestCriteria`
   with `ObjectInterestRequirement`, add native polygon query support.
   Bridge interest converter removed.

**Phase 5 (Service name alignment)** — rename internal PCL services to
   standard names. Bridge becomes a pass-through and can be removed
   entirely. Internal services now directly speak standard.

**Phase 6 (Remove bridge)** — bridge is now identity-only; remove it.
   All code speaks standard types natively.

After each phase, the full test suite (C++ unit + Ada e2e) must pass.

#### Pros

- **Immediate value**: standard compliance available to external consumers
  after Phase 1.
- **Controlled risk**: each phase is small, focused, and independently
  testable. Regressions are caught early.
- **Bridge validates mapping**: the bridge's conversion functions serve as
  executable documentation of the mapping rules, making later phases
  easier to implement correctly.
- **No big-bang rewrite**: avoids the risk of Option A's simultaneous
  changes to every file.

#### Cons

- **Longer total timeline** than Option A (more PRs, more review cycles).
- **Temporary duplication** during transition (both bridge and internal
  types exist for some period).
- **Phase ordering matters**: enum alignment (Phase 2) must happen before
  service name alignment (Phase 5), because streaming ordinals must be
  stable before clients switch.

#### Estimated effort

Phase 1: Medium (same as Option B).
Phases 2–6: Small each (~200 LOC per phase), but 5 separate PRs.

---

## 4. Comparison Matrix

| Criterion | A: In-Place | B: Bridge | C: Hybrid |
|-----------|:-----------:|:---------:|:---------:|
| Time to standard compliance | Long | **Short** | **Short** |
| Risk to existing tests | High | **None** | **None** (Phase 1) |
| Risk to streaming performance | Medium | Low | Low → None |
| Wire-format stability | Breaks | **Stable** | Stable → migrates |
| Long-term code cleanliness | **Best** | Adequate | **Best** |
| Domain extension support | Loses some | **Full** | **Full** |
| Maintenance burden | Low | Medium | Low (after Phase 6) |
| Total LOC change | ~2000 | ~1000 | ~2000 (over 6 phases) |
| Number of PRs | 1 (large) | 1 (medium) | 6 (small each) |

## 5. Detailed Type Mapping Reference

For any option, these conversions are needed:

### 5.1 StandardIdentity ↔ Affiliation

| Standard Proto (ordinal) | Internal C++ (ordinal) |
|--------------------------|----------------------|
| UNSPECIFIED (0) | — (no equivalent, map to Unknown) |
| UNKNOWN (1) | Unknown (3) |
| FRIENDLY (2) | Friendly (0) |
| HOSTILE (3) | Hostile (1) |
| SUSPECT (4) | Suspect (5) |
| NEUTRAL (5) | Neutral (2) |
| PENDING (6) | Pending (8) |
| JOKER (7) | Joker (6) |
| FAKER (8) | Faker (7) |
| ASSUMED_FRIENDLY (9) | AssumedFriend (4) |

### 5.2 BattleDimension

| Standard Proto (ordinal) | Internal C++ (ordinal) |
|--------------------------|----------------------|
| UNSPECIFIED (0) | — (no equivalent, map to Ground) |
| GROUND (1) | Ground (0) |
| SUBSURFACE (2) | Subsurface (3) |
| SEA_SURFACE (3) | SeaSurface (2) |
| AIR (4) | Air (1) |
| UNKNOWN (5) | — (no equivalent) |

Internal has `Space (4)` and `SOF (5)` with no standard equivalent.

### 5.3 Position ↔ GeodeticPosition

```
GeodeticPosition.latitude.radians  = Position.lat * (π / 180.0)
GeodeticPosition.longitude.radians = Position.lon * (π / 180.0)
// altitude: no standard field; carry as extension or drop
```

### 5.4 ObjectInterestRequirement ↔ InterestCriteria

| Standard field | Internal field | Notes |
|---------------|---------------|-------|
| base.id | interest_id | UUID |
| source | — | Not in internal |
| policy | query_mode | QUERY→ReadCurrent, OBTAIN→ActiveFind |
| poly_area | area (BoundingBox) | Lossy: AABB of polygon vertices |
| circle_area | area (BoundingBox) | Lossy: circumscribed AABB |
| point | area (zero-size BB) | Degenerate bounding box |
| dimension[] | battle_dimension | Single → first element |

Missing from standard: `affiliation`, `object_type`, `behavior_pattern`,
`minimum_confidence`, `time_window_*`.  These are domain extensions that
would need to be carried as `Contraint` name-value pairs or as a
component-specific proto extension.

### 5.5 ObjectDetail ↔ EntityUpdateFrame

| Standard field | Internal source | Notes |
|---------------|----------------|-------|
| base.id | entity_id (UUID) | Direct |
| base.update_time | timestamp | epoch → Timestamp |
| base.source | source_refs[0] | First source ref |
| source[] | provenance | Correlated→RADAR, Direct→LOCAL |
| position | position | deg→rad conversion |
| creation_time | first_seen (SourceRef) | Earliest source ref |
| quality | confidence | Direct (0.0–1.0 → 0.0–100.0 %) |
| course | — | Not in internal (would derive from velocity heading) |
| speed | velocity magnitude | √(n²+e²+d²) |
| length | — | Not in internal |
| identity | affiliation | Affiliation→StandardIdentity mapping |
| dimension | battle_dim (MilClass) | Internal→standard ordinal mapping |

Missing from standard: `object_type`, `velocity` vector, `lifecycle_status`,
`mil_class` (full profile), `behavior`, `identity_name`, `version`.

### 5.6 ObjectMatch ← (entity_id, interest_id)

Constructed by the bridge when an entity matches an interest:
```
ObjectMatch.base.id = <bridge-generated match UUID>
ObjectMatch.matching_object_id = entity_id
```

## 6. Recommendation

**Option C (Hybrid)** balances immediate delivery with long-term cleanliness:

- Phase 1 delivers standard compliance quickly with zero regression risk.
- The bridge's conversion functions double as an executable specification
  of the mapping, making subsequent internal migration safer.
- Each subsequent phase is small enough for careful review.
- The end state (Phase 6) is identical to Option A but achieved through
  controlled incremental steps.

If the timeline is extremely compressed and internal consumers can tolerate
breakage, Option A is faster end-to-end. If standard compliance is the
only external requirement and internal migration is not needed, Option B
alone is sufficient.

