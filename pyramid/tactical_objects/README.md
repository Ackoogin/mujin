# Tactical Objects

A PYRAMID component that maintains authoritative knowledge of battlespace objects — their identity, kinematics, classification, relationships, and zone membership — and exposes that knowledge through PCL services and subscribers.

Conforms to **PYRAMID Technical Standard 5.4.2.60**.

---

## Quick start

```cpp
// Embed in a PCL executor
TacticalObjectsComponent comp;
comp.configure();
comp.activate();

pcl::Executor exec;
exec.add(comp);

// Ingest an observation via the subscriber
auto batch = TacticalObjectsCodec::encodeObservationBatch(myBatch);
std::string payload = batch.dump();
pcl_msg_t msg{}; msg.data = payload.data(); msg.size = payload.size();
exec.dispatchIncoming("observation_ingress", &msg);

// Query via service
auto req = TacticalObjectsCodec::encodeQueryRequest(QueryRequest{});
// ... invoke "query" service ...
```

Or drive the domain runtime directly in tests (no PCL overhead):

```cpp
TacticalObjectsRuntime rt;
auto id = rt.createObject(def);
rt.processObservation(obs);
auto resp = rt.query(QueryRequest{});
```

---

## Architecture

```
PCL transport
    └- TacticalObjectsComponent    (PCL lifecycle + ports + codec)
           └- TacticalObjectsRuntime  (pure domain logic, testable in isolation)
                  ├- ObjectStore          ECS sparse-set, UUID-keyed
                  ├- SpatialIndex         WGS84 grid, O(1) region queries
                  ├- CorrelationEngine    evidence → entity (merge / split / lineage)
                  ├- ZoneEngine           polygon/circle geometry + transition detection
                  ├- MilClassEngine       MIL-STD-2525B field storage + symbol key derivation
                  ├- QueryEngine          compound predicate filter (type / affil / region / age)
                  ├- RelationshipIndex    hierarchical + tactical relationships, reverse-indexed
                  ├- InterestManager      interest requirements + derived evidence + progress
                  └- TacticalHistory      timestamped snapshots, as-of / interval retrieval
```

The component layer is intentionally thin — it owns only port wiring, JSON serialisation via `TacticalObjectsCodec`, and the PCL lifecycle. All domain reasoning lives in `TacticalObjectsRuntime` so unit tests can exercise it without a running executor.

---

## PCL ports

| Port | Type | Direction | Purpose |
|------|------|-----------|---------|
| `observation_ingress` | `application/json` | subscriber | Ingest `ObservationBatch`; triggers correlation |
| `create_object` | `application/json` | service | Create an object from an `ObjectDefinition`; returns `object_id` |
| `update_object` | `application/json` | service | Apply an `ObjectUpdate` to an existing object |
| `delete_object` | `application/json` | service | Delete an object by UUID |
| `query` | `application/json` | service | Compound-predicate query; returns `QueryResponse` |
| `get_object` | `application/json` | service | Fetch a single object by UUID |
| `upsert_zone` | `application/json` | service | Create or replace a `ZoneDefinition`; returns `zone_id` |
| `remove_zone` | `application/json` | service | Remove a zone by UUID |

All JSON schemas are defined in `TacticalObjectsCodec` and exercised by `Test_TacticalObjectsCodec.cpp`.

---

## Key types

Defined in `include/TacticalObjectsTypes.h`:

| Type | Purpose |
|------|---------|
| `ObjectType` | Platform, Person, Equipment, Unit, Formation, Installation, … |
| `Affiliation` | Friendly, Hostile, Neutral, Unknown, Suspect, … |
| `BattleDimension` | Ground, Air, SeaSurface, Subsurface, Space, SOF |
| `ZoneType` | AOI, PatrolArea, RestrictedArea, NoGoArea, KillBox, … |
| `RelationshipType` | Hierarchical, Tactical, Organisational |
| `Position` | WGS84 lat / lon / alt |
| `Observation` | Single sensor report (source ref, position, confidence, SIDC hint) |
| `ObjectDefinition` | Full object spec for direct creation |
| `ObjectUpdate` | Partial update (all fields optional) |
| `ZoneDefinition` | Zone geometry (circle or polygon) + metadata |
| `QueryRequest` | Compound predicate filter |

---

## Build

The component is part of the PYRAMID monorepo. Build as part of the top-level project:

```bat
cmake --preset default
cmake --build build --config Release -j%NUMBER_OF_PROCESSORS%
```

Two CMake targets are produced:

| Target | Contents |
|--------|----------|
| `tactical_objects` | Pure domain runtime + codec; no PCL dependency |
| `tactical_objects_component` | PCL component wrapper; depends on `tactical_objects` + `pcl_core` |

---

## Tests

Sixteen test binaries cover the component. Run all of them:

```bat
ctest --test-dir build --output-on-failure -C Release -R "^test_tobj"
```

| Binary | What it tests |
|--------|---------------|
| `test_tobj_types` | Enum values and type defaults |
| `test_tobj_object_store` | ECS CRUD, component tables, versioning |
| `test_tobj_milclass` | MIL-STD-2525B field storage and symbol key derivation |
| `test_tobj_spatial` | Grid index, region queries, position updates |
| `test_tobj_zone` | Zone CRUD, point-in-polygon/circle, boundary transitions |
| `test_tobj_correlation` | Evidence ingestion, merge, split, lineage |
| `test_tobj_query` | Compound predicates: type, affiliation, region, age |
| `test_tobj_relationship` | Insert / remove / reverse-index queries |
| `test_tobj_codec` | JSON round-trips for every message type |
| `test_tobj_interest` | Interest lifecycle, derived evidence, progress |
| `test_tobj_history` | Snapshot record, as-of and interval retrieval |
| `test_tobj_runtime` | `TacticalObjectsRuntime` façade integration |
| `test_tobj_zone_perf` | Performance: thousands of entities and zones |
| `test_tobj_component` | PCL services and subscriber end-to-end |
| `test_tobj_component_robustness` | Stress, concurrency, and error injection |
| `test_tobj_component_hlr` | Explicit TOBJ / RESP requirement trace tags |

### Statement coverage

100% statement coverage is verified with GCC `--coverage` + gcovr. To reproduce:

```bash
cmake -S . -B build-coverage \
  -DCMAKE_BUILD_TYPE=Debug -DAME_FOXGLOVE=OFF \
  -DCMAKE_CXX_FLAGS="--coverage -O0 -fno-elide-constructors" \
  -DCMAKE_EXE_LINKER_FLAGS="--coverage"
cmake --build build-coverage -j$(nproc) --target tactical_objects tactical_objects_component \
  test_tobj_types test_tobj_object_store test_tobj_milclass test_tobj_spatial \
  test_tobj_zone test_tobj_correlation test_tobj_query test_tobj_relationship \
  test_tobj_codec test_tobj_interest test_tobj_history test_tobj_runtime \
  test_tobj_zone_perf test_tobj_component test_tobj_component_robustness \
  test_tobj_component_hlr
# run all binaries, then:
gcovr --root . --filter "pyramid/tactical_objects/src/.*" \
  --object-directory build-coverage \
  --gcov-ignore-errors source_not_found \
  --gcov-ignore-errors no_working_dir_found --txt
```

> **Note:** `-fno-elide-constructors` is required to prevent GCC 13's mandatory
> C++17 copy elision from marking destructor epilogues as unreachable (`=====`)
> in gcov, which would produce false negatives on functions returning local
> objects by value.

---

## Documentation

| Document | Purpose |
|----------|---------|
| `ARCHITECTURE.md` | Layer diagram, data-flow, worked examples |
| `DESIGN.md` | Concrete class design driving TDD |
| `HLR.md` | High-level requirements (TOBJ.001–TOBJ.053, RESP.001–RESP.017) |
| `LLR.md` | Low-level requirements per engine |
| `HLR_COVERAGE.md` | Requirement → test traceability matrix |
| `TDD_PLAN.md` | Test-driven development plan and implementation sequence |
