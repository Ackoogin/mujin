# PYRAMID — Consolidated TODO & Plan

**Last consolidated: 2026-07-04.** This is the single tracker for all
outstanding PCL/PYRAMID work. It consolidates the remaining items from:

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
| 1 | B1 Ada consumed-side parent package stub | S |
| 2 | B2 FlatBuffers JSON-bridge nested packages | S |
| 3 | A1 Package-neutral ROS2 marshal codegen | M |
| 4 | A2 `application/ros2` registry codec plugin | M |
| 5 | A3 Typed `RclcppRuntimeAdapter` on the live wire | M/L |
| 6 | A4 Plain-rclcpp interop proof | S |
| 7 | C1 Retire `src/protobuf_support/` | M |
| 8 | C2 Manifest-driven CMake completion + default flip | M |
| 9 | C3 Domain literals behind the compat policy + source guard | M |
| 10 | C6 Document remaining allowed facade leaks | S |
| 11 | C4 Retire codegen shims (after one SDK release) | S |
| 12 | C5 Windows parity pass | S/M |

C1–C3 are independent of WS-A and of each other; they can be picked up in
parallel whenever there is slack. C4 is time-gated, C5 environment-gated.

---

## WS-B — Known defects (do first)

### B1. Ada consumed-side generic-layout parent package stub missing

Recorded in `generator_refactor_plan.md` follow-ups. Consumed-side
generic-layout facades whose service has no consumed types package get no
parent package stub (e.g. `pyramid-components-pim_osprey-sensors-services-
consumed.ads` is withed but never emitted); 3 of 174 generic-tree units fail
the `gnatgcc -c -gnat2020` object-compile.

- **Plan:** in the Ada service generator, emit the parent `…-consumed.ads`
  package stub whenever any child consumed unit is generated, regardless of
  whether a consumed types package exists. Add the failing trio to the Ada
  object-compile gate so the gap cannot reopen.
- **Accept:** 174/174 generic-tree units object-compile; pyramid tree
  byte-identical.

### B2. FlatBuffers `.cpp` JSON-bridge include/call derivation for nested packages

Carried from `pim/test_harness/FINDINGS.md` and cleanup plan item 7. The
bridge assumes single-segment data-model packages; nested packages (e.g.
`common_pim_components.authorisation`) need the full-namespace treatment
already applied to the header path.

- **Plan:** reuse the native-namespace derivation (post-split home:
  `pim/cpp/naming.py`) for the bridge include/call derivation; add a
  nested-package fixture to `test_generic_flatbuffers_protobuf.py`.
- **Accept:** FlatBuffers codecs for the `pim/test` tree compile; pyramid
  output unchanged.

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

`pim/ros2_marshal_codegen.py`/`pim/ros2_ir.py` are tied to
`pyramid::domain_model`, `pyramid_msgs`, and `pyramid_ros2_codec.hpp`, so
generic layouts cannot get a typed ROS2 codec. Do this **before** the wire
switch so A2/A3 do not land on a domain-coupled generator.

- **Plan:** route package name, msg-package name, and codec header name
  through the naming policy (`PyramidCompatNamingPolicy` supplies today's
  values), keeping pyramid output byte-identical.
- **Accept:** generic-layout fixture generates a compiling typed ROS2 codec
  with zero `pyramid` tokens; pyramid `pyramid_msgs` output unchanged.

### A2. Back the existing `application/ros2` registration with the typed codec

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
is the last non-generated binding artifact on an otherwise generated path.

- **Plan:** teach the protobuf backend to emit the service-codec support it
  covers; record it in `binding_manifest.json` under a role; select via
  `pyramid_binding_sources`. Keep the checked-in copy until the generated
  output is diff-verified against it, then delete.
- **Accept:** `src/protobuf_support/` deleted; protobuf E2E tests
  (`tobj_cpp_app_client_protobuf_e2e` etc.) green from generated source.

### C2. Complete manifest-driven CMake and flip the default (cleanup item 4)

`PYRAMID_BINDING_SOURCE_MODE` still defaults to `glob` (`CMakeLists.txt:115`);
manifest mode covers FlatBuffers schemas + JSON codec sources only. The
per-module C-ABI marshal loop still parses module identity from filenames;
codec-plugin targets and protoc/gRPC proto inputs remain on globs/fixed paths.

- **Plan:** add module identity and plugin/proto-input roles to
  `binding_manifest.json`; extend `pyramid_manifest.cmake` accessors with
  matching `cmake -P` tests; convert the remaining consumers; verify the full
  preset matrix (`default`, `all-on`, `all-off`, `flatbuffers-only`); flip
  the default to `manifest`, keep `glob` one release as fallback.
- **Accept:** manifest mode selects every generated source; glob and manifest
  produce identical target source lists on both trees; default flipped.

### C3. Confine residual domain knowledge to the compat policy (cleanup item 5)

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
