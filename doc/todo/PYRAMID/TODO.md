# PYRAMID — Consolidated TODO & Plan

**Last consolidated: 2026-07-10.** This is the single tracker for remaining
PCL/PYRAMID work. Completed workstreams have been folded into "Delivered"
below; the executed plans, reviews, and status reports that used to carry
their detail were removed in the 2026-07-10 doc review — their design
intents are summarised in
[`doc/plans/PYRAMID/README.md`](../../plans/PYRAMID/README.md) and their
full text is in git history.

Live companion documents:

| Document | Role |
|----------|------|
| [`pyramid_split_and_tobj_pim_migration_plan.md`](../../plans/PYRAMID/pyramid_split_and_tobj_pim_migration_plan.md) | **Live plan (2026-07-06, not yet scheduled):** capability/consumers subproject split + Tactical Objects migration onto the PIM Osprey port-grammar contract; subsumes E5 when executed |
| [`pyramid_user_guide.md`](../../../subprojects/PYRAMID/doc/guides/pyramid_user_guide.md) | Single high-level user guide (design intent, usage, diagrams) |
| [`standard_alignment.md`](../../../subprojects/PYRAMID/doc/architecture/tactical_objects/standard_alignment.md) | Stable design reference for shipped Tactical Objects; one open design point (D-list row below) |
| [`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md) / [`ros2_transport_semantics.md`](../../../subprojects/PYRAMID/doc/architecture/ros2_transport_semantics.md) | How the plugin/codec system and the ROS2 mapping work |

## Delivered (context, not work)

Plugin binding v1 (fail-closed codec/transport `.so` plugins, C++ + Ada);
json/flatbuffers/protobuf codec plugins; socket/shm/udp decoupled transports;
gRPC and ROS2 coupled plugins (both directions); the transport capability
model with compose-time fail-closed routing; native ROS2 IDL + round-trip
verified `domain_model`↔`pyramid_msgs` marshalling, typed on the live wire
with a plain-rclcpp interop proof (WS-A, closed 2026-07-04); contract-derived
pub/sub topics (MBSE-stamped `pyramid.options.Interaction` + port-grammar
fallback, QoS in the contract, correlated request/requirement pairs proven
over PCL and a PUBSUB-only route); the generator monolith split (`pim/cpp/`,
`pim/ada/`, no module globals, deterministic emission); manifest-driven CMake
source selection (opt-in via `-DPYRAMID_BINDING_SOURCE_MODE=manifest`);
domain literals confined to `PyramidCompatNamingPolicy`; `cpp_codegen.py`/
`ada_codegen.py` compat shims retired; Windows `.bat` harness suite verified
(WS-C, closed 2026-07-04); the in-process/local-peer C++ facade (explicit
local routing, facade-owned pub/sub, both proven same-executor with no
transport adapter — WS-E E1–E4/E6, closed 2026-07-04); the C++ interaction
facade (`RequestPortClient`/`RequestPortProvider`/`InformationPortSink`/
`InformationPortSource`, proven cross-process over real SHM by
`agra_seam_interchange_test`); an A-GRA facade example in `examples/cpp/`
(F2(a)) and `examples/ada/` (F2(c)), both done 2026-07-10; the Ada
interaction facade's single-process runtime dispatch (F1, done 2026-07-10 —
see below for what's still open).

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

| Order | Item | Size |
|-------|------|------|
| 1 | F1 remainder: cross-process/remote-transport proof, D4 narrow case, CI wiring | M |
| 2 | F2(b) Tactical Objects example (rides on PIM migration plan) | S/M |
| 3 | E5 Classify or migrate `StandardBridge` raw PCL wiring | S/M |

---

## WS-E — In-process service/pub/sub facade closure

E1–E4 and E6 are done (see Delivered). E5 remains open.

### E5. Classify or migrate `StandardBridge` raw PCL wiring

- **Plan:** either migrate `StandardBridge` to the generated service/topic
  facade where practical or document remaining raw PCL calls as
  framework-adapter exceptions with source-guard coverage.
- **Accept:** copied examples and ordinary components use the facade; any raw
  generated-service/topic PCL calls in `StandardBridge` are allowlisted with a
  reason.

---

## WS-F — Interaction facade follow-ons

### F1. Ada interaction-facade runtime parity — remaining scope

Single-process runtime dispatch (both `rpc` and `pubsub` realizations,
`submit()`/`transitions()`, D1/D2/D3/D4-primary) is done — see Delivered.
Remaining, not blockers:

- **Cross-process/remote-transport proof** mirroring `agra_seam_interchange_test`
  (rpc/rpc, pubsub/pubsub, mixed, over real SHM). `Client_Bind`/
  `Provider_Bind` currently always route locally
  (`Pcl_Bindings.PCL_ROUTE_LOCAL` hardcoded); remote routing needs a
  `Config_Json`/`Transport_Config` parameter threaded through, a mechanical
  follow-up, not a design change.
- **D4's narrower case**: an *already-open* RPC Read stream doesn't get a
  re-send when a late pub/sub-realized command changes state for an id it's
  already watching (C++'s `republishSnapshotFor()` on the inbound-command
  path has no Ada equivalent yet — documented gap in the generated code
  comments, not silently wrong).
- **CI/CTest wiring**: the proof
  (`pim/test_harness/agra_ada_interaction_facade_proof.adb` /
  `build_agra_ada_interaction_facade_proof.sh`) is a standalone script
  today, matching the other `pim/test_harness/build_*.sh` proofs that also
  aren't CTest-registered; wiring GNAT detection + the proof into the
  standing regression bar is unstarted.

### F2. Convert extant examples to the port abstraction

`examples/cpp/agra_interaction_facade_example.cpp` (F2(a)) and
`examples/ada/agra_interaction_facade_example.adb` (F2(c), done 2026-07-10 —
`--binding=rpc|pubsub`, built via `build_agra_interaction_facade_example.sh`,
mirrors the C++ example using F1's Client_Bind/Provider_Bind/Handlers
pattern) are done — see Delivered. Remaining:

- **(b)** convert the Tactical Objects showcase when its contract becomes
  grammar-conforming — rides on
  [`pyramid_split_and_tobj_pim_migration_plan.md`](../../plans/PYRAMID/pyramid_split_and_tobj_pim_migration_plan.md)
  (the legacy CRUD services are not port-grammar shapes today).

---

## WS-D — Deferred, with explicit triggers

No action until the trigger fires; listed so nothing silently drops.

| Item | Trigger | Notes |
|------|---------|-------|
| ROS2 actions (`RPC_ACTION`) | First production user needing actions | First-class by decision (reserved `ACTION` pattern). `action` contract construct → `RPC_ACTION` cap, generated `.action` IDL, adapter action server/client. |
| Opt-in capability adapters (`PUBSUB over RPC_STREAM`, `RPC_UNARY over PUBSUB` for free-form services) | Concrete need | Stay strictly fail-closed until then. For *grammar-conforming* Request/Information ports this is now just a route-line realization choice (no adapter needed); this row covers *free-form (non-grammar) services only*. |
| FlatBuffers codec-plugin independence | Before declaring binary codecs fully independent plugin artifacts | FB plugins currently require the generated JSON codec closure as a wrapper-conversion bridge. |
| Ada ROS2 runtime | Ada consumer of ROS2 transport | Ada has generated ROS2 constants/specs only; no rclcpp-equivalent runtime. |
| Top-level (non-ament) ROS2 plugin target | Only if ament-free builds need the coupled plugin | rclcpp discoverability keeps it in the ament/colcon build today. |
| ROS2 stream-cancel direct test | First production ROS2 user | Runtime cancel works; add dedicated coverage then. |
| Plugin-level threading conformance harness | First production ROS2/gRPC user, or any threading regression | The in-tree transports pass `PclTransportThreading`; the ROS2/gRPC coupled plugins satisfy the executor-threading contract in code (documented in `pcl/pcl_transport.h`) but lack plugin-level `PyramidPluginThreading.Ros2*`/`Grpc*` harness coverage. Optional at the same time: route the shared-memory `respond`/`stream_send`/`stream_end`/`stream_cancel` bus-lock operations through the egress worker. |
| AME contract canonicalization | AME exposes its interface as canonical PYRAMID `.proto` | Unblocks AME consuming the generated ROS2 bindings directly. |
| `ada/service_body_gen.py` `_write_body` split (~1.3k lines) | Next substantive change inside that emitter | Splitting means threading the output stream and dozens of locals — do it when already in there. |
| `pcl_transport_shared_memory.c` split (~2.2k lines) | Peer-identity threading work grows it further | Split along existing seams (ring/mailbox, frame codec, gateway dispatch, plugin entry) within the same target. |
| `contract_routing_manifest.py` support for real (non-stub) transports | A generated manifest needs to target SHM/UDP/etc. rather than `contract_transport_plugin.c` | Today it only emits `{"mode":"rpc"\|"pubsub"}` config for the NULL-vtable stub. Needs `bus_name`/`participant_id` (SHM) and `remote_host`/`remote_port`/`local_port`/`peer_id` (UDP) config emission, plus the counterpart-participant-id peer-alias convention. |
| Tactical Objects bulk-detail path | Consumers need full detail in bulk | Decide between a standard batch-detail path vs overloading the match stream ([`standard_alignment.md`](../../../subprojects/PYRAMID/doc/architecture/tactical_objects/standard_alignment.md), Remaining Design Point). |
| Interaction-pattern options for the legacy tree / side-table deletion | Only if the frozen-compat stance changes | Resolved as *frozen compat, new consumers forbidden*; `standard_topics.py` stays scoped to the legacy layout. |
