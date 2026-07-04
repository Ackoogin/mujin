# PYRAMID — Consolidated TODO & Plan

**Last consolidated: 2026-07-04. All items verified against the tree on
2026-07-04** (both contract trees regenerated with all backends; statuses
below reflect generated output and build wiring, not just the source docs).
This is the single tracker for all outstanding PCL/PYRAMID work. It
consolidates the remaining items from:

| Source doc | Role now |
|------------|----------|
| [`transport_plugins.md`](../../plans/PYRAMID/transport_plugins.md) | Capability statement + detail for the ROS2 typed-wire work (WS-A) and deferred transport items |
| [`architecture_cleanup_plan.md`](../../plans/PYRAMID/architecture_cleanup_plan.md) | Detail for the structural cleanup items (WS-C); items 1–2 executed |
| [`generator_refactor_plan.md`](../../plans/PYRAMID/generator_refactor_plan.md) | **Executed 2026-07-03**; its recorded follow-ups are tracked here (B1, C4, C5, D-list) |
| [`pubsub_contract_generation_plan.md`](../../plans/PYRAMID/pubsub_contract_generation_plan.md) | **Executed 2026-07-02** (phases 0–5); carried-forward notes tracked here (C5, D-list) |
| [`standard_alignment_plan.md`](../../plans/PYRAMID/standard_alignment_plan.md) | Stable design reference for Tactical Objects; one open design point (D-list) |
| [`review_pyramid_bindings_pluggability.md`](../../reviews/PYRAMID/review_pyramid_bindings_pluggability.md) | Review record; facade closure steps 1–7 done, step 8 tracked here (C6) |
| `subprojects/PYRAMID/pim/test_harness/FINDINGS.md` | Harness reference; its one open follow-up is B2 |

How the plugin/codec system works lives in the architecture reference:
[`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md)
and
[`ros2_transport_semantics.md`](../../../subprojects/PYRAMID/doc/architecture/ros2_transport_semantics.md).

## Delivered (context, not work)

Plugin binding v1 (fail-closed codec/transport `.so` plugins, C++ + Ada);
json/flatbuffers/protobuf codec plugins; socket/shm/udp decoupled transports;
gRPC and ROS2 coupled plugins (both directions); the transport capability
model with compose-time fail-closed routing; native ROS2 IDL + round-trip
verified `domain_model`↔`pyramid_msgs` marshalling; contract-derived pub/sub
topics (MBSE-stamped `pyramid.options.Interaction` + port-grammar fallback,
QoS in the contract, correlated request/requirement pairs proven over PCL and
a PUBSUB-only route); the generator monolith split (`pim/cpp/`, `pim/ada/`,
no module globals, deterministic emission).

## Standing regression bar (applies to every item below)

1. Default `pyramid` layout output stays **byte-for-byte identical**
   (`diff -qr` against a pre-change generated baseline) unless the item's
   acceptance says otherwise.
2. `python3 -m pytest subprojects/PYRAMID/tests -q` green, plus
   `python3 -m unittest subprojects/PYRAMID/pim/test_proto_parser.py`.
3. For generator changes touching Ada: object-compile the generated Ada for
   both trees (`gnatgcc -c -gnat2020`).
4. End-of-workstream (slow): `viability_check.sh`, `build_comms_test.sh`,
   `build_plugin_load_test.sh`, `build_contract_routing_test.sh`, and the
   packaged-SDK import smoke (`package_sdk.sh` then
   `python3 -c "import generate_bindings"` from the packaged `generator/`).

---

## Execution order

Bugs first (small, known, sharpen the baseline), then the headline ROS2
typed-wire workstream (whose first step is itself a generator cleanup), with
the hygiene items running alongside or after. Deferred items stay parked
behind explicit triggers.

| Order | Item | Size |
|-------|------|------|
| 1 | ~~B1 Ada consumed-side parent package stub~~ **✅ done 2026-07-04** | S |
| 2 | ~~B3 Ada FlatBuffers codec misses reserved-word rename~~ **✅ done 2026-07-04** | S |
| 3 | ~~B2 Close out FlatBuffers JSON-bridge nested packages~~ **✅ done 2026-07-04** | S |
| 4 | ~~A1 Package-neutral ROS2 marshal codegen~~ **✅ done 2026-07-04** | M |
| 5 | ~~A2 `application/ros2` registry codec plugin~~ **✅ done 2026-07-04 (ament build unverified in-sandbox)** | M |
| 6 | A3 Typed `RclcppRuntimeAdapter` on the live wire | M/L |
| 7 | A4 Plain-rclcpp interop proof | S |
| 8 | C1 Retire `src/protobuf_support/` | S/M |
| 9 | C2 Manifest-driven CMake completion + default flip | M |
| 10 | ~~C3 Domain literals behind the compat policy + source guard~~ **✅ done 2026-07-04** | M |
| 11 | C6 Document remaining allowed facade leaks | S |
| 12 | C4 Retire codegen shims (after one SDK release) | S |
| 13 | C5 Windows parity pass | S/M |

C1–C3 are independent of WS-A and of each other; they can be picked up in
parallel whenever there is slack. C4 is time-gated, C5 environment-gated.

---

## WS-B — Known defects (do first)

### B1. Ada consumed-side generic-layout parent package stub missing

**✅ Done 2026-07-04.** Root cause: the generic-layout service generator
(`pim/ada/generic_service_gen.py`, `AdaGenericServiceGenerator.generate`) never
called `_ensure_parent_packages` for the child `…-consumed-services` facade it
emits, so the `…-consumed.ads` parent stub was skipped whenever no consumed
*types* package existed. Fix: emit parent stubs for each generated generic
service package. Added `tests/test_generic_ada.py` regression coverage for the
three previously-missing stubs plus a GNAT object-compile gate for the consumed
child services. Failing trio object-compile 0/3 → 3/3; pyramid Ada output
byte-identical (75/75 source files).

Recorded in `generator_refactor_plan.md` follow-ups; **re-verified 2026-07-04**
by regenerating the generic tree — exactly three parent packages are withed
by children but never emitted: `pim_osprey.sensors`,
`pim_osprey.tactical_objects`, and `pim_seaspray.sensors`
`…services.consumed`. These are the 3 of 174 units failing the
`gnatgcc -c -gnat2020` object-compile.

- **Plan:** in the Ada service generator, emit the parent `…-consumed.ads`
  package stub whenever any child consumed unit is generated, regardless of
  whether a consumed types package exists. Add the failing trio to the Ada
  object-compile gate so the gap cannot reopen.
- **Accept:** 174/174 generic-tree units object-compile; pyramid tree
  byte-identical.

### B2. Close out: FlatBuffers `.cpp` JSON-bridge for nested packages

**✅ Done 2026-07-04.** Confirmed no generator change was needed — the fix was
already in place. Added
`test_pim_flatbuffers_json_bridge_uses_full_namespace_for_nested_data_model`
to `tests/test_generic_flatbuffers_protobuf.py` locking the nested
`common_pim_components.authorisation` full-namespace include/toJson/fromJson
(and asserting the truncated single-segment form is absent); re-ran the
`pim/test` FlatBuffers compile gate (`build_comms_test.bat`) — generation,
flatc, nested-codec compile, link, and JSON/FlatBuffers round-trip all pass;
marked the FINDINGS.md follow-up CLOSED.

Carried from `pim/test_harness/FINDINGS.md` and cleanup plan item 7 as
"assumes single-segment packages" — **that appears already fixed** (verified
2026-07-04): the bridge now derives includes and calls through
`_cpp_type_namespace_for_type(full_type, naming_policy)`, and the generated
output for the nested `common_pim_components.authorisation` package carries
the correct full-namespace include and `toJson`/`fromJson` calls. What's
missing is the closing evidence: no nested-package fixture exists in
`test_generic_flatbuffers_protobuf.py`, and the FB compile gate hasn't been
re-run against this case.

- **Plan:** add the nested-package fixture test; run the FlatBuffers compile
  gate on the `pim/test` tree; update the stale FINDINGS.md follow-up.
- **Accept:** fixture test green; FlatBuffers codecs for the `pim/test` tree
  compile; pyramid output unchanged.

### B3. Ada FlatBuffers codec skips the reserved-word package rename (new, found 2026-07-04)

**✅ Done 2026-07-04.** Routed the FlatBuffers backend's two inline Ada
package-segment helpers (`_service_group_names`, `_ada_pkg_from_proto_pkg` in
`pim/backends/flatbuffers_backend.py`) through
`pim/ada/naming.py`'s `_ada_pkg_segment`, so reserved segments like `generic`
escape to `Generic_Pkg` matching the core generators. Added
`test_generic_flatbuffers_ada_escapes_reserved_package_segments` (no bare
`Generic_Pim.Generic.` refs) and extended the object-compile gate with
`test_generic_ada_flatbuffers_service_codecs_object_compile`. FB Ada
object-compile: failing → 8/8 pass; pyramid output byte-identical.

The core Ada generators escape Ada reserved words in package segments
(`generic` → `Generic_Pkg`, via `_ada_pkg_segment` /
`_ADA_RESERVED_WORDS` in `pim/ada/naming.py`), but the FlatBuffers backend
derives Ada package names on its own path: every generated
`flatbuffers/ada/*-flatbuffers_codec.{ads,adb}` that touches the generic
tree's `generic_pim.generic` data-model package emits
`with Pyramid.Data_Model.Generic_Pim.Generic.Types` — `Generic` is a
reserved word, so these units are illegal Ada and reference a unit that is
(correctly) emitted as `Generic_Pkg.Types`. 10+ files affected on the
`pim/test` tree. Not caught earlier because the Ada object-compile gate
covers the core tree, not the FlatBuffers Ada codec output.

- **Plan:** route the FlatBuffers backend's Ada package derivation through
  `pim/ada/naming.py`'s segment escaping; extend the Ada object-compile gate
  to include `flatbuffers/ada/` output.
- **Accept:** no `Generic_Pim.Generic.` references in generated output; the
  FlatBuffers Ada codec units object-compile; pyramid tree byte-identical.

---

## WS-A — Typed ROS2 codec on the live wire (headline)

Much of this stack is already delivered — the remaining work is the last
wire switch, not greenfield. In the tree today:

- **Done:** `pim/ros2_idl_codegen.py` emits real `.msg`/`.srv` from the proto
  contracts (invoked by the ros2 backend; `pyramid_msgs` built by colcon via
  `scripts/build_ros2_transport.sh`); `pim/ros2_marshal_codegen.py` emits the
  `domain_model`↔`pyramid_msgs` codec (`pyramid_ros2_codec.hpp`), round-trip
  verified by `test_ros2_codec_roundtrip` in the ament test suite.
- **Exists, envelope-only:** the coupled plugin already registers
  `application/ros2` (`plugins/pyramid_ros2_coupled_plugin.cpp`) — as a
  passthrough envelope codec; `RclcppRuntimeAdapter`
  (`ros2/src/rclcpp_runtime_adapter.cpp`) exists but is keyed to
  `PclEnvelope.msg`, the only message in `ros2/msg/`.

What's left: make the live path use the typed codec/messages instead of the
envelope. Detail: `transport_plugins.md` §1 and cleanup plan item 6. Keep the
envelope path as the decoupled codec-over-ROS2 fallback; native typed becomes
the default.

### A1. Make the ROS2 marshal codegen package-neutral (prerequisite)

**✅ Done 2026-07-04.** Added four naming-policy accessors
(`ros2_message_package`, `ros2_codec_header`, `ros2_codec_namespace`,
`ros2_data_model_namespace`) to the `NamingPolicy` protocol,
`PyramidCompatNamingPolicy` (returning today's exact `pyramid_msgs` /
`pyramid_ros2_codec.hpp` / `pyramid::ros2_codec` / `pyramid::domain_model`
strings), and `GenericNamingPolicy` (package-derived equivalents). Threaded a
naming policy + package through `DomainIR`, `Ros2MarshalGenerator`, and
`generate_ros2_codec`; `ros2_backend.py` now generates the typed codec on the
generic layout too (gate removed) and passes `support_package`. Added
`test_generic_ros2_typed_codec_is_package_neutral` (zero `pyramid` tokens).
Pyramid ros2 output byte-identical (independently re-verified via
old-vs-new `diff -qr`).

`pim/ros2_marshal_codegen.py`/`pim/ros2_ir.py` are tied to
`pyramid::domain_model`, `pyramid_msgs`, and `pyramid_ros2_codec.hpp`, so
generic layouts cannot get a typed ROS2 codec — the ros2 backend explicitly
gates codec generation on `layout == 'pyramid'`
(`pim/backends/ros2_backend.py`; verified 2026-07-04: the generic tree gets
`ros2/idl` + transport support but no `ros2/codec/`). Do this **before** the
wire switch so A2/A3 do not land on a domain-coupled generator.

- **Plan:** route package name, msg-package name, and codec header name
  through the naming policy (`PyramidCompatNamingPolicy` supplies today's
  values), keeping pyramid output byte-identical.
- **Accept:** generic-layout fixture generates a compiling typed ROS2 codec
  with zero `pyramid` tokens; pyramid `pyramid_msgs` output unchanged.

### A2. Back the existing `application/ros2` registration with the typed codec

**✅ Done 2026-07-04 (generation + structural; live ament build unverified).**
Added a `ros2` branch to the codec-plugin emitter (`pim/cpp/codec_plugin_gen.py`),
reusing the binary-backend path with content-type `application/ros2` and the
A1 naming-policy accessors (`ros2_codec_namespace` → `pyramid::ros2_codec`,
`ros2_codec_header` → `pyramid_ros2_codec.hpp`); emitted into `ros2/cpp/` as
`*_ros2_codec_plugin.cpp`. Extended `pim/ros2_marshal_codegen.py` with
varint-framed `toBinary(vector)` / `fromBinary{Msg}Array` helpers (purely
additive) for the streaming/array schema types. `generate_bindings.py` maps
`_ros2_` plugin files to `application/ros2` in the manifest. Wired the ament
`ros2/CMakeLists.txt` to build each `*_ros2_codec_plugin.cpp` as a `MODULE`
against `rclcpp`/`pyramid_msgs`/`pcl_core_external`; excluded these plugins from
both the ament support-lib glob and the top-level `pyramid_ros2_transport` glob
(they only build inside ament). Coupled-plugin passthrough remains the envelope
fallback. Added `tests/test_ros2_codec_plugin_generation.py` (emission,
disabled-backend absence, ament wiring, top-level exclusion). Existing
`test_plugin_only_fail_closed` binary still passes; non-ros2 codec-plugin output
byte-identical (7 files).
**Residual:** a real ament/colcon/rclcpp build + registry-load of the typed
codec could not run in-sandbox — needs a Humble environment to close out (shared
with A3/A4).

The content type and codec-vtable registration already exist in the coupled
plugin as a passthrough envelope codec.

- **Plan:** add a ros2 branch to the codec-plugin emitter (post-split home in
  `pim/cpp/`), backed by the generated `ros2_codec` from
  `pyramid_ros2_codec.hpp`. The typed codec must build **inside the ament
  package** (rclcpp/pyramid_msgs only resolve there), unlike the
  json/fb/protobuf plugins built by the top-level CMake — wire it into the
  ament `CMakeLists` the IDL generator already maintains; the passthrough
  stays as the envelope fallback.
- **Accept:** typed codec loads through the standard registry path in the
  ament build; fail-closed behaviour preserved when absent.

### A3. Switch `RclcppRuntimeAdapter` to typed messages

The adapter exists and works — over `PclEnvelope`.

- **Plan:** publish/serve the `pyramid_msgs` messages (e.g.
  `rclcpp::GenericPublisher`/`GenericSubscription` keyed by the
  `pyramid_msgs/...` type name) so `ros2 topic echo` shows typed fields,
  replacing `PclEnvelope`/`PclService`/`PclOpenStream` on the default path.
  Contract QoS (already carried into `TopicBinding`) selects the ROS2
  profile.
- **Accept:** existing ROS2 transport tests green on the typed path; envelope
  fallback still selectable by config.

### A4. Interop proof

- **Plan:** a plain `rclcpp` node (no PYRAMID linkage) round-tripping the
  generated IDL against a PYRAMID component; add to the harness.
- **Accept:** round-trip green on Humble; documented in
  `ros2_transport_semantics.md`.

---

## WS-C — Structural / hygiene

### C1. Retire checked-in `src/protobuf_support/` (cleanup item 3)

The hand-maintained `pyramid_services_tactical_objects_protobuf_codec.{hpp,cpp}`
is the last non-generated binding artifact on an otherwise generated path
(still wired as `pyramid_protobuf_support` in the top-level CMake).
**Verified 2026-07-04:** the protobuf backend *already emits* per-service
protobuf codecs on the generic layout (13 service codec files on the
`pim/test` tree) — the pyramid compat layout just doesn't get them. This is
an enable-and-diff job, not greenfield emitter work.

- **Plan:** extend the existing service-codec emission to the pyramid compat
  layout; record it in `binding_manifest.json` under a role; select via
  `pyramid_binding_sources`. Keep the checked-in copy until the generated
  output is diff-verified against it, then delete.
- **Accept:** `src/protobuf_support/` deleted; protobuf E2E tests
  (`tobj_cpp_app_client_protobuf_e2e` etc.) green from generated source.

### C2. Complete manifest-driven CMake and flip the default (cleanup item 4)

`PYRAMID_BINDING_SOURCE_MODE` still defaults to `glob` (`CMakeLists.txt:115`).
**Verified 2026-07-04 — further along than the cleanup plan text:** the
manifest already carries `codec_plugins`, `protobuf_protos`, and
`grpc_service_protos` roles, and `pyramid_manifest.cmake` already has the
matching accessors (e.g. `pyramid_manifest_plugins`). What remains is
consumption: only 2 of ~14 source-selection sites in the PYRAMID
`CMakeLists.txt` go through the manifest-capable `pyramid_binding_sources()`
(FlatBuffers schemas + JSON codec sources); the other 12 are raw
`pyramid_glob_generated()` calls, and the per-module C-ABI marshal loop
still regex-parses module identity from
`pyramid_data_model_*_cabi_marshal.cpp` filenames.

- **Plan:** add module identity to `binding_manifest.json`; convert the 12
  remaining glob call sites (marshal sources + loop, service codecs, plugin
  targets, protobuf support, gRPC/ROS2 transports) to roles; verify the full
  preset matrix (`default`, `all-on`, `all-off`, `flatbuffers-only`); flip
  the default to `manifest`, keep `glob` one release as fallback.
- **Accept:** manifest mode selects every generated source; glob and manifest
  produce identical target source lists on both trees; default flipped.

### C3. Confine residual domain knowledge to the compat policy (cleanup item 5)

**✅ Done 2026-07-04.** Made `PyramidCompatNamingPolicy.base_type_map` (in
`binding_contract.py`) the single source of truth for the
`pyramid.data_model.base.*`/`common.*` short-name mapping; `pim/cpp/naming.py`,
`pim/ada/naming.py`, and `pim/backends/grpc_backend.py` now consume it instead
of each hardcoding a copy (`GenericNamingPolicy.base_type_map == {}`). Added
`tests/test_no_domain_literals.py` asserting the map key literals appear only in
the compat policy, not the consumer modules, and that the map stays a semantic
anchor (each value == last segment). Byte-identical pyramid output re-verified
via old-vs-new `diff -qr` across cpp+ada+grpc+ros2. **Residual (not blocking):**
a lone `packages_with_messages.discard('pyramid.data_model.base')` domain
literal remains in `_service_codec_imports` (`cpp/naming.py`) — a no-op on the
generic layout; route through `data_model_proto_root` in a later pass.

`BASE_TYPE_MAP` (now in `pim/cpp/naming.py`) hardcodes
`pyramid.data_model.base.*`/`common.*` short-name mapping; residual
`pyramid_*` literals remain in the Ada service generator and some backend
compat shims.

- **Plan:** move `BASE_TYPE_MAP` behind `PyramidCompatNamingPolicy`; sweep
  `pim/` for `pyramid`-literal tokens outside `binding_contract.py`'s compat
  policy, `topic_metadata/`, and compat-marked branches; add a source-guard
  test (the `test_topic_metadata.py` pattern) asserting the generic code path
  contains no domain literals.
- **Accept:** source-guard test in place and green; byte-identical pyramid
  output.

### C4. Retire the `cpp_codegen.py` / `ada_codegen.py` shims (time-gated)

The shims carry only the externally consumed surface (pinned by
`tests/test_codegen_export_surface.py`); the SDK ships the generator to third
parties, so they stay **one SDK release**, then delete (with the export-surface
test updated to the new module paths).

### C5. Windows parity pass (environment-gated)

Accumulated from the refactor and pub/sub work: `package_sdk.bat` packaged-SDK
smoke, and the updated comms/plugin-load `.bat` harness scripts, have been
authored but only run on Linux. Run the `.bat` suite once a Windows
environment is available; fix whatever falls out.

### C6. Document the remaining allowed facade leaks (review §6.9 step 8)

Facade closure steps 1–7 are done. If any shim-level API (`_Json`, `grpc_*`)
must stay public for ABI reasons, mark it compatibility-only in generated docs
and tests, and make the typed facade the copied example everywhere.

---

## WS-D — Deferred, with explicit triggers

No action until the trigger fires; listed so nothing silently drops.

| Item | Trigger | Notes |
|------|---------|-------|
| ROS2 actions (`RPC_ACTION`) | First production user needing actions | First-class by decision (reserved `ACTION` pattern). `action` contract construct → `RPC_ACTION` cap, generated `.action` IDL (depends on A1–A4), adapter action server/client. |
| Opt-in capability adapters (`PUBSUB over RPC_STREAM`, `RPC_UNARY over PUBSUB` for free-form services) | Concrete need | Stay strictly fail-closed until then. The correlated-pair pattern (pub/sub plan §1/§5) is the ready-made spec; contract-derived port topics already cover the MBSE tree. |
| FlatBuffers codec-plugin independence | Before declaring binary codecs fully independent plugin artifacts | FB plugins currently require the generated JSON codec closure as a wrapper-conversion bridge (pub/sub plan carried note). |
| Ada ROS2 runtime | Ada consumer of ROS2 transport | Ada has generated ROS2 constants/specs only; no rclcpp-equivalent runtime. |
| Top-level (non-ament) ROS2 plugin target | Only if ament-free builds need the coupled plugin | rclcpp discoverability keeps it in the ament/colcon build today. |
| ROS2 stream-cancel direct test | First production ROS2 user | Runtime cancel works; add dedicated coverage then. |
| AME contract canonicalization | AME exposes its interface as canonical PYRAMID `.proto` | Unblocks AME consuming the generated ROS2 bindings directly. |
| `ada/service_body_gen.py` `_write_body` split (~1.3k lines) | Next substantive change inside that emitter | Splitting means threading the output stream and dozens of locals — do it when already in there. |
| `pcl_transport_shared_memory.c` split (~2.2k lines) | Peer-identity threading work grows it further | Split along existing seams (ring/mailbox, frame codec, gateway dispatch, plugin entry) within the same target. |
| Tactical Objects bulk-detail path | Consumers need full detail in bulk | Decide between a standard batch-detail path vs overloading the match stream (`standard_alignment_plan.md`, Remaining Design Point). |
| Interaction-pattern options for the legacy tree / side-table deletion | Only if the frozen-compat stance changes | Phase 5 of the pub/sub plan resolved this as *frozen compat, new consumers forbidden*; `standard_topics.py` stays scoped to the legacy layout. |
