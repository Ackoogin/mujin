# Generic Proto Binding — Implementation Progress

Living progress log for implementing a generic binding approach that supports
both **PYRAMID-style** protos and **arbitrary/generic** proto contracts, per:

- `doc/reports/PYRAMID/arbitrary_proto_binding_gap_analysis.md`
- `doc/reports/PYRAMID/binding_hard_coupling_removal_report.md`

Coding and unit testing are delegated to Codex; this repo owner orchestrates,
reviews, and integrates. Last updated: 2026-07-01.

## Strategy

Route every generator/build decision through a neutral `BindingContract` plus an
explicit **naming policy**:

- `pyramid_compat` — reproduces current generated names/namespaces exactly, so
  existing apps and tests do not regress.
- `generic` — derives all names from the proto package/service/message identity,
  with no reserved `pyramid` / `data_model` / `components` / `services` segments.

Backward compatibility is a hard constraint: default layout stays `pyramid` and
must produce byte-for-byte identical output until callers migrate.

## Work Breakdown & Status

| # | Increment | Status |
|---|-----------|--------|
| 1 | Neutral `BindingContract` + `NamingPolicy` (pyramid_compat / generic); `--contract-layout` flag | **Done** (Codex, verified) |
| 2 | Generic C++ JSON binding path (content-classified, role-neutral facades) | **Done** (Codex, verified) |
| 3 | Generator artifact manifest (types/codecs/cabi/services/plugins/protos) | **Done** (Codex, verified) |
| 4 | Generic protobuf / FlatBuffers / gRPC / ROS2 backends (import-root & manifest driven) | **Done for C++ backends** (4a FlatBuffers/Protobuf; 4b gRPC/ROS2) |
| 5 | Manifest-driven CMake + explicit topic metadata (retire package inference) | **Done** (5a topics + 5b manifest CMake; main agent, verified) |
| 6 | Ada parity for generic layout | **Done** — types + codecs + role-neutral service facades, GNAT-compiled (main agent) |

### Increment 1 & 2 — delivered (2026-07-01)

New `pim/binding_contract.py`: content-based classification (`type_modules`,
`service_modules`, `wrapper_modules`), per-service type closures, stable schema
IDs, and two naming policies (`PyramidCompatNamingPolicy`, `GenericNamingPolicy`)
behind a `NamingPolicy` protocol. `generate_bindings.py` gained
`--contract-layout {pyramid,generic}` (default `pyramid`). `cpp_codegen.py`
generators accept an injected `naming_policy`.

Verification:
- New `tests/test_generic_binding_contract.py` (3 tests) passes: generic
  `example.telemetry` proto → `example::telemetry` namespace, role-neutral
  `Telemetry` facade, zero `pyramid` tokens in generic output; content
  classification + service closure; and a **full-filename regression lock** on
  default pyramid C++/JSON output.
- Default pyramid output confirmed **byte-for-byte unchanged** (`diff -qr`
  baseline vs patched, exit 0).
- Pre-existing `tests/test_binding_generation_dependencies.py` still exits 0.

Scope carried forward: generic path currently emits type headers, JSON codecs,
and a role-neutral service facade. Generic **C-ABI** and the **JSON codec
plugin** are not yet emitted; Ada generic path not yet wired (Increment 6).

Definition of done for #1–2 (from gap analysis Phase 1): a plain proto package
with `message Pose` + `service Telemetry` produces compiling C++ JSON bindings
and dispatches through a PCL service handler, while Tactical Objects output is
unchanged under `pyramid_compat`.

### Increment 3 — delivered (2026-07-01)

`pim/generate_bindings.py` now emits `binding_manifest.json` after generation
for both `pyramid` and `generic` contract layouts. The manifest records the
layout, inferred proto import root, proto files relative to that import root,
and generated artifact roles: `types`, `json_codecs`, `cabi`, `services`,
`codec_plugins`, `flatbuffers_schemas`, `protobuf_protos`, and
`grpc_service_protos`.

Implementation decision: the C++ JSON orchestration records concrete files
written around each existing generator call and classifies those paths by role;
it does not introduce filename globbing or reclassify artifacts from
`pyramid_*` patterns. Backend-specific fields are present for the stable schema,
but remain empty for JSON-only generation except for selected future
protobuf/gRPC proto inputs.

Verification:
- New `tests/test_binding_manifest.py` covers generic `example.telemetry`,
  PYRAMID JSON/C++ role coverage, plugin metadata, and the guard that the only
  extra PYRAMID output file is `binding_manifest.json`.
- `python3 -m pytest subprojects/PYRAMID/tests -q` passed: 6 tests.
- Default PYRAMID C++/JSON non-manifest output was checked byte-for-byte
  unchanged against a pre-change generated baseline with `diff -qr` and sorted
  SHA-256 manifests, excluding only `binding_manifest.json`.

### Increment 4a — delivered (2026-07-01)

Protobuf and FlatBuffers C++ backend generation now receives the shared
`BindingContract`, naming policy, and inferred proto import root from
`generate_bindings.py`.

Protobuf wrapper includes are derived from the proto file path relative to the
import root (`telemetry.proto` -> `telemetry.pb.h` for a root-level generic
proto), with the old `pyramid/...` anchor kept only as a direct-call
compatibility fallback. The generated wrapper namespace is policy-derived.

FlatBuffers service-wire generation now groups services through policy-provided
service module keys/names. Generic service packages generate package-derived
schemas/codecs such as `example_telemetry_services.fbs` under
`example.telemetry`, while PYRAMID layout keeps the legacy grouped
`pyramid_services_*` names. Generic service codecs include per-package type
headers and do not emit the PYRAMID data-model umbrella header or alias.

Verification:
- New `tests/test_generic_flatbuffers_protobuf.py` covers generic
  `example.telemetry` FlatBuffers and Protobuf generation, plus PYRAMID
  FlatBuffers/Protobuf filename and symbol regression locks.
- PYRAMID FlatBuffers C++ output was generated before and after the change and
  compared with `diff -qr`; result: no differences.
- PYRAMID Protobuf C++ output was generated before and after the change and
  compared with `diff -qr`; result: no differences.
- Generic FlatBuffers and Protobuf outputs were checked to contain no
  case-insensitive `pyramid` token leakage.

### Increment 4b — delivered (2026-07-01)

gRPC and ROS2 C++ transport backend generation now receives and uses the shared
`BindingContract`, naming policy, and inferred proto import root from
`generate_bindings.py`.

ROS2 service selection is driven by parsed service declarations under generic
layout, so flat service files such as `example.telemetry.service.proto` generate
transport projections and `.srv` IDL without requiring a `.services.` package
segment. The ROS2 support file base, support namespace, envelope type, route
prefix, and per-service transport file base are policy-derived. PYRAMID layout
still emits `pyramid_ros2_transport_support.*`,
`pyramid::transport::ros2`, `/pyramid/...` route names, and the existing
`pyramid_msgs` marshal codec byte-for-byte unchanged. Generic layout deliberately
does not emit the PYRAMID-specific ROS2 marshal codec until that typed codec is
made package-neutral.

gRPC service-module transport files now derive their generic file base and
facade namespace from the naming policy, and the plugin aggregator file/symbol
prefix is policy-derived. Generic gRPC generated code includes
import-root-relative `service.pb.h` and `service.grpc.pb.h`, qualifies
unqualified flat-package RPC request/response types with the declaring package,
and does not apply the PYRAMID base-type short-name map. PYRAMID layout retains
the legacy transport filenames, aggregator filenames/symbols, and include paths.

Verification:
- New `tests/test_generic_grpc_ros2.py` covers generic flat-namespace ROS2 and
  gRPC generation, import-root-relative gRPC includes, zero generic `pyramid`
  token leakage, and PYRAMID filename/symbol regression locks.
- `python3 -m pytest subprojects/PYRAMID/tests -q` passed: 13 tests.
- PYRAMID ROS2 output was generated before and after the change and compared
  with `diff -qr`; result: no differences.
- PYRAMID gRPC output was generated before and after the change and compared
  with `diff -qr`; result: no differences.
- Generic ROS2 and gRPC output directories were checked with
  `grep -Rin pyramid`; result: no matches.

### Increment 5a — delivered (2026-07-01, by main agent; Codex was rate-limited)

`pim/standard_topics.py` is now **data-driven** and contains no domain
branching. The Tactical Objects topic table (wire names, `pyramid.data_model.tactical.*`
payload types, provided/consumed subscribe/publish sets) moved into an explicit
JSON data file `pim/topic_metadata/tactical_objects_topics.json`. The module
exposes `TopicMetadata` (empty = no topics), `load_topic_metadata(path)`,
`EMPTY_METADATA`, and keeps the `topic_spec` / `topics_for_service` public API.
The module-level default is the compat (tactical) metadata so every existing
call site is unchanged; `topics_for_service` selects topics purely by
metadata-declared `package_match`, not a built-in `tactical_objects` check.
Generic layout emits no topics (its service path never calls topic code, and
`EMPTY_METADATA` returns `({}, {})` for any package including a tactical one).

Verification:
- New `tests/test_topic_metadata.py` (5 tests): default reproduces the exact
  tactical subscribe/publish sets and `entity_matches` spec; empty metadata
  yields no topics even for a tactical package; a source guard asserts no
  domain data/branching remains in the module logic (only the `.json` path
  constant may name the domain); pyramid layout still bakes `standard.*` wire
  names into the tactical provided facade; generic layout emits no `standard.`
  topics.
- Full suite: `python3 -m pytest subprojects/PYRAMID/tests -q` → **18 passed**.
- PYRAMID **C++** and **Ada** output regenerated before/after the change and
  compared with `diff -qr`: **identical** in both languages.

### Increment 5b — delivered (2026-07-01, by main agent)

CMake can now build generated bindings from `binding_manifest.json` roles
instead of `pyramid_*` filename globs. Two small, unit-tested CMake modules were
added under `subprojects/PYRAMID/cmake/`:

- `pyramid_manifest.cmake` — reads a manifest and returns source lists per role
  (`pyramid_manifest_sources`, with optional filename `SUFFIX` filter),
  codec-plugin descriptors filtered by content type (`pyramid_manifest_plugins`),
  and scalar fields (`pyramid_manifest_field`). Uses `string(JSON ...)`.
- `pyramid_binding_sources.cmake` — a `pyramid_binding_sources(OUT ROLE ... GLOBS ...)`
  wrapper that returns the manifest role list in `manifest` mode and the legacy
  glob list in `glob` mode (also falling back to globs when a role is absent).

`CMakeLists.txt` gained a `PYRAMID_BINDING_SOURCE_MODE` cache option
(**default `glob`**, alternative `manifest`). The FlatBuffers schema list and the
combined data-model+service JSON codec source list are now selected through
`pyramid_binding_sources`; in `manifest` mode the `json_codecs` role already
carries both data-model and per-component codecs, so the separate
`pyramid_components_*_codec.cpp` append is guarded off to avoid double counting.

Verification (no full project configure was possible here — it fetches
BehaviorTree.CPP / LAPKT / gRPC over the network — so the CMake logic was
verified in `cmake -P` script mode and via a wiring dry-run):
- `cmake/tests/test_pyramid_manifest.cmake` and
  `cmake/tests/test_pyramid_binding_sources.cmake` run under `cmake -P` and are
  driven from the Python suite by `tests/test_manifest_cmake_helper.py` (3 tests),
  against freshly generated pyramid and generic manifests.
- Selection wrapper proven: absent role falls back to glob; `.hpp` never leaks
  into a `.cpp` list; glob mode is byte-identical to the previous globs.
- Wiring dry-run over both the pyramid tree and the richer `pim/test` tree (which
  has 16 component codecs) shows **identical source counts in glob vs manifest
  mode** (pim/test: 48 == 48 codecs; pyramid: 7 codecs / 10 fbs each).
- Full suite: `python3 -m pytest subprojects/PYRAMID/tests -q` → **21 passed**.

**Real configure + build verification (2026-07-01, network enabled):**
`cmake --preset all-off -DPYRAMID_BINDING_SOURCE_MODE=manifest` configures
cleanly, prints `PYRAMID binding sources: manifest mode (...)` (confirming the
manifest was read after the generator wrote it — an ordering fix moved the read
to after the generation `execute_process`), and
`cmake --build build-all-off --target pyramid_generated_codecs` **compiles and
links `libpyramid_generated_codecs.a` from the manifest-selected codec sources**
(exit 0). So "CMake creates codec targets from the manifest" is verified with a
real build, not just `cmake -P`.

Staged remainder (documented): the per-data-model-module C-ABI marshal loop
still derives module identity by regex on `pyramid_data_model_*_cabi_marshal.cpp`
filenames, so marshal sources, codec-plugin targets, and the protoc/gRPC proto
input paths remain on globs/fixed paths. The default stays `glob` (the fully
consistent, long-proven path) until those are also manifest-driven; `manifest`
mode is a verified opt-in that currently sources codecs + FlatBuffers schemas
from the manifest and globs the rest.

### Increment 6 — delivered (2026-07-01, by main agent)

`_generate_json_ada` in `pim/generate_bindings.py` gained a `generic` branch
mirroring the C++ one: it classifies type modules via the `BindingContract` and
generates Ada native types + JSON codecs for arbitrary proto packages. This
works without new naming-policy plumbing because the Ada package transform is
already package-derived (`AdaTypesGenerator._ada_pkg_for_file`:
`example.telemetry` -> `Example.Telemetry.Types`); the pyramid-specific parts
(`_DATA_MODEL_TYPES_PKGS`, `_find_proto_root` requiring a `pyramid/` dir) sit
only on the pyramid path.

Verification:
- New `tests/test_generic_ada.py` (2 tests): generic `example.telemetry`
  produces `example-telemetry-types.ads` + `..._codec.ads/.adb` using
  `package Example.Telemetry.Types` with **no `Pyramid` token**; pyramid layout
  still emits `Pyramid.Data_Model.Tactical.Types`.
- Pyramid Ada binding files are byte-identical to the **original pre-refactor
  generator** (`diff -qr`, only the added `binding_manifest.json` differs).
- Full suite: `python3 -m pytest subprojects/PYRAMID/tests -q` → **23 passed**.

### Increment 6 (service facades) — delivered (2026-07-01, GNAT-verified)

The generic Ada **service facade** is now emitted. The existing
`AdaServiceGenerator` could not be reused: its `ProtoService` **filters RPCs to
CRUD names only** (`OP_PREFIXES` = Create/Read/Update/Delete, via `rpc.op`), so
arbitrary RPCs like `GetPose` are silently dropped — a deep domain assumption.

A new `AdaGenericServiceGenerator` (in `pim/ada_codegen.py`) emits a minimal,
**role-neutral** facade from a `proto_parser.ProtoFile` (which keeps every RPC):
per-RPC wire-name constants plus one access-to-function handler per RPC in a
`Service_Handlers` record — mirroring the C++ generic `ServiceHandler`. It
depends only on the generated types package (no PCL/CRUD/PYRAMID coupling), and
homes flat in the proto namespace (`example.telemetry` ->
`Example.Telemetry.Services`).

Verification (**GNAT 10.5.0 available**):
- New `tests/test_generic_ada.py` now also asserts the facade
  (`package Example.Telemetry.Services`, `Handle_Telemetry_Get_Pose`,
  `access function (Request : Pose) return Pose`) and runs a **GNAT semantic
  compile** (`gnatmake -gnatc`) over the generated types + codec + facade for a
  two-RPC non-CRUD service — exit 0.
- Full suite: `python3 -m pytest subprojects/PYRAMID/tests -q` → **24 passed**.
- Pyramid Ada output remains byte-identical (`diff -qr`, 0 differences excluding
  the added manifest) after all `ada_codegen.py` changes.

## Redundant / Hard-Coded Domain Knowledge Found

Things discovered during implementation that are redundant, brittle, or bake
domain knowledge into reusable infrastructure. These are removal/consolidation
targets — they should live behind the `pyramid_compat` policy or be deleted.

| Item | Location | Problem | Disposition |
|------|----------|---------|-------------|
| ~~**Hard-coded Tactical Objects topics**~~ **DONE (5a)** | `pim/standard_topics.py` | `topics_for_service()` keyed off the substring `tactical_objects` and returned hardcoded topic names/payload types. Pure domain knowledge in a shared path. | **Resolved**: module is now data-driven; tactical set lives in `pim/topic_metadata/tactical_objects_topics.json`; default is compat metadata, generic default is **no topics**. |
| Data-model package root | `pim/generate_bindings.py` `_discover_data_model_files()` | Only `pyramid.data_model*` packages are treated as type modules — arbitrary type protos are silently dropped. | Content-based classification via `BindingContract`. (Increment 1) |
| Service package marker | `pim/generate_bindings.py` `_discover_service_message_files()` + both service loops | Services only generated when package contains `.services.`. | Classify by parsed `service` decls. (Increment 1) |
| Domain namespace constants | `pim/cpp_codegen.py` `_DATA_MODEL_PROTO_ROOT`, `_DATA_MODEL_TYPES_NS`, `_DATA_MODEL_TYPES_HEADER` | Types forced under `pyramid::domain_model`, umbrella `pyramid_data_model_types.hpp`. | Behind naming policy. (Increment 1) |
| Legacy service namespace collapse | `pim/cpp_codegen.py` `_legacy_service_namespace()` | Skips a reserved set `{pyramid, components, data_model, base, services}` and re-homes services under `pyramid::services::*` / `pyramid_services_*`. | Behind `pyramid_compat` policy; generic default keeps package path. (Increment 1) |
| Role inference from package | `pim/cpp_codegen.py` `_is_provided()`, `pim/ada_codegen.py` | Endpoint role derived from the word `provided`/`consumed` in the package. | Explicit role metadata; role-neutral default. (Increment 1/6) |
| Hardcoded base-type short-name map | `pim/cpp_codegen.py` `BASE_TYPE_MAP` | Maps specific `pyramid.data_model.base.*` / `common.*` FQNs to short names. | Domain-specific; confine to `pyramid_compat`. (review) |
| Protobuf include root anchor | `pim/backends/protobuf_backend.py` | `.pb.h` include path anchors on the path segment `pyramid` when present. | Derive from configured proto import root. (Increment 4) |
| FlatBuffers service grouping | `pim/backends/flatbuffers_backend.py` | Grouping depends on `.services.` + pyramid segment skipping. | Group by service-module identity from contract. (Increment 4) |
| FlatBuffers PYRAMID enum umbrella casts | `pim/backends/flatbuffers_backend.py` | Existing PYRAMID service codecs cast bare data-model enums through `pyramid::domain_model` rather than the declaring package namespace; generic layout must not inherit that umbrella behavior. | Preserved only in `pyramid_compat`; generic resolves enum namespaces from the declaring proto package. (Increment 4a) |
| ROS2 service detection | `pim/backends/ros2_backend.py` | Service packages identified by `.services.` segment. | Drive from manifest service entries. (Increment 4) |
| ROS2 support names and routes | `pim/backends/ros2_backend.py` | Support files, support namespace, envelope type, and route prefixes were fixed as `pyramid_ros2_transport_support`, `pyramid::transport::ros2`, `pyramid_ros2/PclEnvelope`, and `/pyramid/...`. | Behind naming policy; generic derives package-neutral support names and route prefix. (Increment 4b) |
| ROS2 typed marshal codec | `pim/ros2_marshal_codegen.py`, `pim/ros2_ir.py` | Typed ROS2 marshal codec is explicitly tied to `pyramid::domain_model`, `pyramid_msgs`, and `pyramid_ros2_codec.hpp`. | Keep emitted only for `pyramid_compat`; make package-neutral before enabling for generic. (Increment 4b / future) |
| gRPC plugin aggregator names | `pim/backends/grpc_backend.py` | Aggregator files and stable C ABI symbols were fixed as `pyramid_grpc_plugin_*`. | Behind naming policy; generic derives aggregator file/symbol prefix from the service package. (Increment 4b) |
| gRPC proto include root | `pim/backends/grpc_backend.py` | `.grpc.pb.h` include path used a repository `proto` fallback and generic `.pb.h` includes were not emitted. | Generic path derives `.pb.h` and `.grpc.pb.h` from the passed proto import root; PYRAMID path remains byte-for-byte compatible. (Increment 4b) |
| gRPC unqualified service RPC types | `pim/backends/grpc_backend.py` | RPC request/response type names were emitted by splitting the literal type string, so flat-package unqualified types became `::PoseRequest` instead of `::example::telemetry::PoseRequest`. | Resolve unqualified RPC types against the declaring service package. (Increment 4b) |
| CMake `pyramid_*` globs **PARTIAL (5b)** | `subprojects/PYRAMID/CMakeLists.txt` | Targets created from hardcoded `pyramid_data_model_*`, `pyramid_components_*`, `pyramid_services_*` filename patterns; `protoc` over `pyramid/data_model/*.proto`; gRPC over `pyramid/components/*.proto`. | **Partly resolved**: `PYRAMID_BINDING_SOURCE_MODE=manifest` drives FlatBuffers schemas + JSON codec sources from the manifest via tested `cmake/pyramid_*` helpers; default stays `glob`. Marshal per-module loop, codec-plugin targets, and protoc/gRPC input paths still glob/fixed (need module identity in manifest + a network-enabled configure test). |
| Duplicate `ProtoFile`/`parse_proto` | `pim/cpp_codegen.py` vs `pim/proto_parser.py` | `cpp_codegen.py` defines its own lightweight `ProtoFile`/`parse_proto` (line ~299–332) parallel to the richer `proto_parser.ProtoFile`/`parse_proto_tree`. Two parsers of the same IDL is a redundancy/skew risk. | Consolidate onto `proto_parser` model where feasible. (review / opportunistic) |
| Checked-in Tactical Objects protobuf support | `src/protobuf_support/*` | Domain-specific protobuf codec support sits on the active path. | Move to generated/manifest-selected; keep tactical shim as compat only. (Increment 4/5) |

## Target Generic Contract Shape

The real generic contract to support (beyond the `example.telemetry` toy) is
broadly: **one or two namespaces total** (not deep `pyramid.*` hierarchies);
some proto files carry **general message types** (shared data model); other
proto files carry **services + refined message types** (request/response
wrappers) with the **services declared flat in the main namespace(s)** — not in
a `.services.` sub-package, refined types co-located in that same namespace.
`GenericNamingPolicy` homes services and refined types flat in `package::...`;
`BindingContract.wrapper_modules` (service files that also declare messages) is
exactly this "services + refined types in one file" case and must get both
type/codec generation and a service facade. Generic-layout fixtures should
include this two-namespace / flat-service shape.

## Notes / Decisions

- The existing `pim/contract_resolver.py` is **unrelated** to the new
  `BindingContract`: it matches SysML activities to service interfaces from
  JSON, not proto classification. New work lives in `pim/binding_contract.py`.
- `pim/proto_parser.py` (`parse_proto_tree`, `ProtoTypeIndex`) is already
  package-agnostic and is reused as the parse front-end; the coupling is all
  downstream (classification, naming, build).

## Acceptance Signals To Track

- [x] Non-PYRAMID fixture generates C++ JSON bindings (generation + structure
      verified; a native C++ compile of the fixture is not run in-suite).
- [x] Same fixture generates FlatBuffers, Protobuf, gRPC and ROS2 C++ (and Ada
      types + codecs + a role-neutral service facade, **GNAT-compiled**), all
      with zero `pyramid` tokens.
- [x] CMake selects codec + FlatBuffers-schema sources from the manifest under
      `PYRAMID_BINDING_SOURCE_MODE=manifest` — **verified with a real configure
      + build** (`libpyramid_generated_codecs.a` links from manifest-selected
      sources). Marshal/plugin/proto-input conversion + default flip staged.
- [x] gRPC/ROS2 generation does not require fixed `pyramid/...` package paths
      (service detection is by parsed `service` decls under generic layout).
- [x] Tactical Objects / pyramid output unchanged: C++ (JSON/protobuf/
      flatbuffers/gRPC/ROS2) and Ada binding files are byte-identical to the
      original generator; 23 Python tests + `cmake -P` tests pass.
- [~] Domain strings confined to compat: `standard_topics.py` is data-driven
      (tactical data in JSON); C++ pyramid names live behind
      `PyramidCompatNamingPolicy`. A full repo-wide sweep of every generator/
      backend for residual `pyramid_*` literals is still outstanding (Ada
      service generator + some backend compat shims).
