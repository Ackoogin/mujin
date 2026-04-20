# Tactical Objects

A PYRAMID component that maintains authoritative knowledge of battlespace objects — their identity, kinematics, classification, relationships, and zone membership — and exposes that knowledge through PCL services and subscribers.

Conforms to **PYRAMID Technical Standard 5.4.2.60**.

Current status: the runtime, generated PYRAMID service bridge, binary streaming codec, interest-filtered updates, resync service, JSON/FlatBuffers/Protobuf generated binding paths, and Ada/C++ socket smoke flows are implemented and covered by tests. See [`generated_bindings_status.md`](../../../doc/reports/PYRAMID/generated_bindings_status.md) for the current conformance snapshot.

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
    └- tactical_objects_app
           ├- StandardBridge          generated PYRAMID provided/consumed interface
           └- TacticalObjectsRuntime  pure domain logic, testable in isolation
                  ├- ObjectStore          ECS sparse-set, UUID-keyed
                  ├- SpatialIndex         WGS84 grid, O(1) region queries
                  ├- CorrelationEngine    evidence -> entity (merge / split / lineage)
                  ├- ZoneEngine           polygon/circle geometry + transition detection
                  ├- MilClassEngine       MIL-STD-2525B field storage + symbol key derivation
                  ├- QueryEngine          compound predicate filter (type / affil / region / age)
                  ├- RelationshipIndex    hierarchical + tactical relationships, reverse-indexed
                  ├- InterestManager      interest requirements + derived evidence + progress
                  └- TacticalHistory      timestamped snapshots, as-of / interval retrieval
```

The component layer is intentionally thin. `TacticalObjectsComponent` owns internal port wiring and internal JSON serialisation via `TacticalObjectsCodec`; `StandardBridge` owns the generated PYRAMID service boundary. All domain reasoning lives in `TacticalObjectsRuntime` so unit tests can exercise it without a running executor.

`StandardBridge` is the production standard-interface adapter. It hosts the generated PYRAMID Tactical Objects services and topics using generated dispatch/codecs, then maps standard `ObjectInterestRequirement`, `ObjectEvidenceRequirement`, `ObjectMatch`, and `ObjectDetail` values to and from the internal runtime model.

---

## Internal PCL ports

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

## Standard PYRAMID interface

`tactical_objects_app` exposes the generated Tactical Objects provided interface on its remote socket-facing executor. The app can be started with `--content-type application/json`, `--content-type application/flatbuffers`, or `--content-type application/protobuf`.

Generated binding architecture, content-type selection rules, and topic helper
usage are documented in
[`../doc/architecture/generated_bindings.md`](../doc/architecture/generated_bindings.md). Current
Tactical Objects generated-binding status and conformance coverage are tracked
in [`generated_bindings_status.md`](../../../doc/reports/PYRAMID/generated_bindings_status.md).

| Service/topic | Direction | Payload |
|---------------|-----------|---------|
| `object_of_interest.create_requirement` | service | `ObjectInterestRequirement -> Identifier` |
| `object_of_interest.read_requirement` | service | `Query -> ObjectInterestRequirement[]` |
| `object_of_interest.update_requirement` | service | `ObjectInterestRequirement -> Ack` |
| `object_of_interest.delete_requirement` | service | `Identifier -> Ack` |
| `matching_objects.read_match` | service | `Query -> ObjectMatch[]` |
| `specific_object_detail.read_detail` | service | `Query -> ObjectDetail[]` |
| `standard.object_evidence` | consumed topic | `ObjectDetail` |
| `standard.evidence_requirements` | published topic | `ObjectEvidenceRequirement` |
| `standard.entity_matches` | published topic | `ObjectMatch[]` |

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
| `Position` | WGS84 latitude / longitude in radians, altitude in metres |
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

Three CMake targets are produced:

| Target | Contents |
|--------|----------|
| `tactical_objects` | Pure domain runtime + codec; no PCL dependency |
| `tactical_objects_component` | PCL component wrapper plus generated-binding standard adapter; depends on `tactical_objects` + `pcl_core` |
| `tactical_objects_app` | Standalone node with local runtime and remote socket-facing generated PYRAMID interface |

Build just the standalone app:

```bat
cmake --build build --config Release --target tactical_objects_app
build\subprojects\PYRAMID\tactical_objects\Release\tactical_objects_app.exe --port 19123
```

Topology:

- Remote/provided side: `StandardBridge` runs on the socket-facing executor and exposes the generated provided services plus standard match/evidence topics.
- Local/runtime side: `TacticalObjectsRuntime` processes observations, interests, queries, and stream subscribers.
- Consumed-side stub: the app provides a local generated consumed-service stub for evidence-requirement smoke flows.
- Optional smoke input: add `--demo-evidence` to inject an internal demo observation into the runtime.

Remote smoke-test client:

```bat
cmake --build build --config Release --target tactical_objects_test_client
build\subprojects\PYRAMID\tactical_objects\Release\tactical_objects_test_client.exe --host 127.0.0.1 --port 19123
```

Run the focused real-app standard-interface tests:

```bat
ctest --test-dir build -C Release -R "tobj_cpp_app_client_(e2e|flatbuffers_e2e|protobuf_e2e)|tobj_ada_active_find_app(_flatbuffers)?_e2e" --output-on-failure
```

---

## Tests

The Tactical Objects unit/integration binaries are registered as `test_tobj_*`
targets. Run them with:

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
| `test_tobj_streaming_codec` | Binary stream frame codec |
| `test_tobj_runtime_streaming` | Runtime stream subscriber behaviour |
| `test_tobj_interest_matching` | Interest matching and active-find behaviour |
| `test_tobj_e2e` | In-process runtime/component integration |
| `test_tobj_socket_e2e` | Socket transport integration |
| `test_tobj_component` | PCL services and subscriber end-to-end |
| `test_tobj_component_robustness` | Stress, concurrency, and error injection |
| `test_tobj_component_hlr` | Explicit TOBJ / RESP requirement trace tags |

### Statement coverage

Statement coverage can be measured with GCC `--coverage` + gcovr. To reproduce:

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
  test_tobj_component_hlr test_tobj_streaming_codec test_tobj_runtime_streaming \
  test_tobj_interest_matching test_tobj_e2e test_tobj_socket_e2e
# run all binaries, then:
gcovr --root . --filter "subprojects/PYRAMID/tactical_objects/src/.*" \
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
| [`ARCHITECTURE.md`](../doc/architecture/tactical_objects/ARCHITECTURE.md) | Layer diagram, data-flow, worked examples |
| [`DESIGN.md`](../doc/architecture/tactical_objects/DESIGN.md) | Concrete class design and implementation structure |
| [`HLR.md`](../doc/requirements/tactical_objects/HLR.md) | High-level requirements (`TOBJ.001`–`TOBJ.053`, `PYR-RESP-0729`–`PYR-RESP-0745`) |
| [`LLR.md`](../doc/requirements/tactical_objects/LLR.md) | Low-level requirements per engine |
| [`HLR_COVERAGE.md`](../../../doc/reports/PYRAMID/tactical_objects/HLR_COVERAGE.md) | Requirement → test traceability matrix |
