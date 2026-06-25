# Progress: Plugin-Only Binding Model (Option A — C-ABI typed value)

Living progress log for the work described in
[`plugin_only_binding_handover.md`](./plugin_only_binding_handover.md).
Updated as work proceeds. Branch `feat/transport-codec-plugin-strategy-a`.

## Terminology (keep these distinct)

- **Codec** — *wire encoding only*: `encode`/`decode` between a typed value and a
  byte payload (JSON, FlatBuffers, Protobuf …). The `pcl_codec_t` vtable, the
  `pcl_codec_registry`, and the generated `*_codec_plugin` artifacts are all
  **codec-only** — they know nothing about how bytes are moved.
- **Transport** — *moving bytes*: publish/subscribe, request/response, streaming
  over a concrete medium (sockets, shared memory, gRPC, ROS2). The `pcl_transport_t`
  vtable and transport plugins live on this side.
- **Binding / plugin** — the deployable combination a component or bridge actually
  uses = **codec + transport** (plus the generated facade gluing them to the
  contract). When this document says "codec plugin" it means strictly the wire-
  encoding half; a full binding additionally needs a transport.

This effort so far implements the **codec** half (the C-ABI typed-value boundary
between component and codec plugin). Transport plugins are a separate follow-up.

## Design decisions (this effort)

- **Per-component codec plugins.** Each codec plugin targets a single PYRAMID
  component contract (e.g. `tactical_objects`) and only marshals the dependency
  closure of *its* root schema types, so no plugin contains the whole system
  data model. (Provided/consumed sub-splitting was considered and **dropped** —
  see below.)
- **Bridge case drives registry composition.** A single process (a PYRAMID
  bridge) typically spans several components — it consumes component A and
  provides toward component B — so it must load several per-component codec
  plugins at once, all under the same `content_type`. The codec registry
  therefore allows **multiple codecs per content_type**, and dispatch selects by
  `schema_id` (try-each, fail-closed). Transport is orthogonal and handled by the
  transport side.
- **C-ABI data model mirrors the native module split.** C structs are emitted
  per data-model proto module (`common`, `base`, `tactical`, …) in
  `pyramid_data_model_<module>_cabi.h`, mirroring the existing
  `pyramid_data_model_<module>_types.hpp` layout exactly (same field order, base
  inlining, oneof-as-optional). A per-component codec plugin includes only the
  cabi module headers in its closure.
- **Ownership: uniform-owned.** `to_c` malloc-deep-copies all variable-length
  fields into the C struct; `from_c` deep-copies into the native value;
  `pyramid_<Type>_c_free` releases. Both encode and decode call `_free` after use
  (symmetric with `pcl_codec_t.free_msg`). One full copy per direction.
- **Green-keeping.** The static `if/else` fallback stays in place during the
  C-ABI rollout; the plugin-only flip (remove fallback) is a later, deliberate
  step. Full suite kept green per phase.

## Baseline (verified)

- `build-flatbuffers-only` builds clean; full suite 582/582 pass.
- Live generator path: `generate_bindings.py` → `proto_parser`, `codec_backends`,
  `cpp_codegen`, `ada_codegen`, `backends`.

## Status

| Item | Description | Status |
|------|-------------|--------|
| 1 | C-ABI data model header generator (`*_cabi.h`) | **done** |
| 2 | C++ marshalling generator (`to_c`/`from_c`/`_free`) | **done** |
| 3 | Rework C++ codec plugins to take C struct (per-component) | **done** |
| 4 | Facade marshals native↔C-struct via registry path | **done** |
| 5 | CMake wiring (cabi marshal sources into pyramid_generated_codecs) | **done** |
| 6 | Tests: round-trip + swap-without-rebuild; full suite green | **done (582/582)** |
| 7 | Deprecation cleanup (remove dead generator files) | **done** |
| 8 | ~~Split codec plugins provided/consumed~~ — **dropped** (per-component is enough) | n/a |
| 9 | Codec registry composition: many codecs per content_type, dispatch by schema_id (bridge case) | **done (585/585)** |

### Item 7 detail (removed — dead SysML-path generators)

Confirmed unreferenced by the live pipeline (`generate_bindings.py` imports only
`proto_parser`, `codec_backends`, `cpp_codegen`, `ada_codegen`, `backends`) and by
CMake/scripts/tests. Removed:
`cpp_model_generator.py`, `ada_model_generator.py`, `python_model_generator.py`,
`mock_generator.py`, `test_generator.py`, `activity_parser.py`.
Kept `sysml_parser.py` (documented standalone Cameo XMI → JSON import utility, not
part of the proto codegen path). Generation re-verified (cpp+ada, exit 0).

### Items 3–6 detail (committed approach)

- **Codec plugins (`_write_codec_plugin_impl`)** now take the frozen C struct as
  the `void* value`: encode casts to `const pyramid_<T>_c*`, `from_c`s to native,
  then calls the existing wire codec (`toJson`/`toBinary`); decode runs the wire
  codec to native then `to_c`s into the caller's `pyramid_<T>_c*` (plugin-allocated,
  component frees). Scalar aliases stay on the static path (don't cross the C-ABI
  boundary). Plugins include only the cabi marshal headers in their type closure.
- **Facade (`_write_impl`)** registry encode/decode now marshal through generated
  per-facade dispatch (`pyramid_cabi_encode`/`pyramid_cabi_decode`) keyed by
  `schema_id`: native → `to_c` → plugin → free (encode), and plugin → C struct →
  `from_c` → native → free (decode). Schema types not covered (aliases) return 0
  so the existing static fallback still runs. Call sites unchanged.
- **CMake:** `pyramid_data_model_*_cabi_marshal.cpp` globbed into
  `pyramid_generated_codecs` (PIC), so both components and codec plugins link
  `to_c`/`from_c`/`_free`.
- **Test:** `test_codec_plugin_swap.cpp` updated to drive the plugin across the
  C-ABI boundary (marshal to/from `pyramid_ObjectDetail_c`). Proves swap json↔
  flatbuffers without rebuilding and plugin-output == static `toJson`.
- **Verified:** clean build + **full suite 582/582 green** in `build-flatbuffers-only`.
- Granularity: plugins remain **per-component** (`pyramid_codec_json_tactical_objects`
  etc.) — no mega plugin. Provided/consumed split is item 8 (the "ideally" refinement).

### Items 1+2 detail (committed approach)

- New module `subprojects/PYRAMID/pim/cabi_codegen.py`:
  - `CabiTypesGenerator` → `pyramid_datamodel_cabi.h` (umbrella: `PYRAMID_DATAMODEL_ABI_VERSION`,
    `pyramid_str_t`, `pyramid_slice_t`) + per-module `pyramid_data_model_<m>_cabi.h`
    (C structs `pyramid_<Type>_c`, enums as `int32_t`, bool as `uint8_t`, optionals as
    `has_<x>` + value, oneof same, repeated as `pyramid_slice_t`, strings as
    `pyramid_str_t`), plus `pyramid_<Type>_c_free` declarations.
  - `CabiMarshalGenerator` → per-module `pyramid_data_model_<m>_cabi_marshal.{hpp,cpp}`
    with `pyramid::cabi::to_c`/`from_c` and `extern "C" pyramid_<Type>_c_free`.
  - Shared field-walk `iter_cabi_members` mirrors `CppTypesGenerator`'s
    `_inline_base_fields`/oneof layout field-for-field. `find_scalar_wrappers`
    extracted from `cpp_codegen.py` for reuse (no output change to native types).
- **Ownership: uniform-owned.** `to_c` malloc-deep-copies all variable-length data;
  `from_c` deep-copies into native; `_free` releases. Both directions call `_free`
  after use. One full copy per direction. (Deviates from the handover's
  "borrow on encode" — borrow is infeasible for repeated/nested message arrays
  since native stores native structs, not C structs. Revisit for perf later.)
- **Verified:** standalone generation exits 0; all 7 modules' marshalling pass
  `g++ -std=c++17 -fsyntax-only`; a round-trip test (ObjectDetail, ObjectEvidenceRequirement,
  PlanningGoal incl. oneof-string + absent variants) passes under ASan+UBSan.
- Bug fixed during review: oneof-string fields are `tl::optional<std::string>` in
  native; added an `opt_string` member kind so struct + marshalling agree.

### Item 8 dropped: provided/consumed split

Per-component granularity already prevents mega-plugins, which was the real
requirement. Splitting each component's codec plugin again into `provided` /
`consumed` adds deployment churn (target renames, harness/example updates) for
little benefit, so it is **not** being done.

### Item 9: codec registry composition (bridge case)

The motivating need is the **bridge**: one process consumes component A and
provides toward component B, so it loads several per-component **codec** plugins
at once — all registered under the same `content_type` (e.g. `application/json`).
The old registry held one codec per content_type and rejected duplicates, which
blocked this.

Change (codec side only; transport is unaffected):

- `pcl_codec_registry_register` now allows **multiple codecs per content_type**
  (only re-registering the identical vtable pointer is rejected). ABI-version and
  NULL checks unchanged.
- New `pcl_codec_registry_get_at(registry, content_type, index)` iterates the
  codecs registered for a content_type in registration order.
- The generated facade's registry encode/decode iterate those codecs and try each
  by `schema_id` until one handles it; if none do (or the schema is a scalar
  alias) it falls back to the static codec.

## Follow-up phases (out of scope this pass)

- Full plugin-only flip: remove static fallback, default-plugin load.
- Ada C-ABI marshalling + same `.so` loaded from Ada (cross-language proof).
- **Transport** plugin + coupled gRPC/ROS2 target plugin (the transport half of a
  full binding).

## Log

- 2026-06-24: Investigated current state, confirmed baseline green, recorded
  design decisions above.
- 2026-06-24: Items 1+2 (C-ABI data model + C++ marshalling) implemented in
  `cabi_codegen.py`; round-trip verified under ASan/UBSan.
- 2026-06-24: Items 3–6 (plugins take C struct, facade marshals via registry,
  CMake wiring, swap test) implemented; **full suite 582/582 green**.
- 2026-06-24: Item 7 (removed 6 dead SysML-path generators).
- 2026-06-24: Dropped provided/consumed split (item 8). Clarified terminology:
  codec = wire encoding, transport = comms, binding/plugin = both.
- 2026-06-24: Item 9 done — codec registry allows many codecs per content_type
  (`pcl_codec_registry_get_at`, reject only same-pointer re-register); facade
  dispatch tries each by schema_id. New PCL registry tests + a bridge test
  (`CodecRegistryBridge`: tactical + autonomy codec plugins coexist under
  `application/json`, each schema routed to the right plugin). **Full suite
  585/585 green.**
- 2026-06-25: Phase 1 C++ cleanup: generated component bindings now expose typed
  `subscribe*` topic wrappers that decode to native payloads before invoking
  callbacks; `tactical_objects_test_client` now uses `pcl::Component`,
  `pcl::Executor`, and `ConsumedService` for native RPC/topic flow, with codec
  plugins limited to main() deployment wiring.
- 2025-06-25: Closed the Tactical Objects C++ e2e client evidence loop: the
  clean client now subscribes to evidence requirements with generated typed
  wrappers, publishes typed `ObjectDetail` evidence through the consumed binding,
  and the standalone bridge converts internal degree AOIs back to standard
  radian geometry before forwarding evidence requirements.
- 2026-06-25: Ada C-ABI parity implemented. Added generated Ada mirror packages
  (`Pyramid.Data_Model.*.Cabi`) for the frozen `pyramid_<Type>_c` layouts,
  including native Ada <-> C struct marshalling and imports for
  `pyramid_<Type>_c_free`. Generated Ada service registry paths now marshal
  native records to the shared C struct before plugin encode and decode back
  from that struct after plugin decode, while preserving the static fallback and
  interim hand-written Ada JSON codec. Added `ada_cpp_codec_roundtrip`, which
  loads the same C++-built `pyramid_codec_json_tactical_objects` `.so` from Ada
  and round-trips an Ada-native `ObjectDetail` through the plugin boundary.
  Linked Ada executables that include generated service bodies with
  `pyramid_generated_codecs` so the shared C free functions resolve. Verified:
  `pyramid_ada_all`, full `flatbuffers-only-release` build, and **586/586 CTest
  green**.
