# PYRAMID — Consolidated TODO & Plan

**Last consolidated: 2026-07-14.** This is the single tracker for remaining
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
| [`uci_mms_conversion_plan.md`](../../plans/PYRAMID/uci_mms_conversion_plan.md) | **Live plan:** XSD-to-proto profile ladder; Phase 3/P1 is live-proven, while Phase 4/P2 supplies the detailed background for WS-G below |
| [`oms_agra_compatibility.md`](../../../subprojects/PYRAMID/doc/architecture/oms_agra_compatibility.md) | Current support boundary and evidence for UCI 2.5/AMS-GRA versus formal A-GRA |
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
| 4 | G1 Formal A-GRA P2 OMS codec and CAL validation (requires the G1 prerequisites) | L |

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

## WS-G — Formal A-GRA OMS codec closure

### G1. Generate and validate the A-GRA P2 OMS/CAL profile

**Status: in progress (2026-07-14).** Checkpoint log for session handover:

- **Prerequisites 1–3 discharged.** Decision record:
  [`pim/uci_profiles/README.md`](../../../subprojects/PYRAMID/pim/uci_profiles/README.md)
  ("P2 conversion-scope decision"). Root list approved and then grown to 20
  (the two `*CommandStatus` messages the Command-2 pattern needs); size
  budget 1,169 messages + 297 enums; validator designated: Sleet
  v2026.06.01 loaded with the pinned A-GRA 5.0a schema.
- **Validator leg de-risked live** (WSL podman, container
  `sleet-agra-probe`, host port 21412, config under git-ignored
  `external/ams-gra/sleet-agra/`): Sleet loads the A-GRA schema (841
  elements / 4,845 types), accepts OWP `INIT` with schema literal
  `005.0a.ASK`, routes PUB/SUB/MSG, schema-validates payloads (rejected a
  harness-generated instance on a UUID facet), and fails closed on wrong
  schema, unknown service, and unauthorized topic. Evidence to be folded
  into `pim/test_harness/FINDINGS.md` at step 7.
- **Drop-delta finding:** A-GRA 5.0a UUIDs are `xs:hexBinary` length-16
  (32 hex chars, no hyphens), unlike UCI 2.5's hyphenated form; the
  la-cal-harness generator hardcodes hyphenated UUIDs, so P2 instance
  generation must set UUIDs itself.
- **Step 1 (materialize the contract): landed in the working tree.** The
  20-root P2 tree is generated and checked in under
  `pim/uci_generated/agra_5_0a/` with byte-stability and wire-name-coverage
  guards in `pim/test_xsd2proto.py`. A converter defect the drop exposed
  (XSD enums carrying their own `UNSPECIFIED` literal colliding with the
  synthetic zero sentinel) is fixed with `_XSD_LITERAL` disambiguation.
- **Step 3 (generator shapes): probed complete, with one correction from
  step 4 (see below).** A P2-wide probe found zero missing shapes in the
  C++/Ada OMS-JSON emitters; the repeated-choice wire form is pinned from
  the la-cal-harness against the A-GRA XSD (member element name keyed to a
  JSON array in the parent; fixtures under
  `pim/test_oms_json_gen_fixtures/repeated_choice_member/`). Note the
  probe's limit: it only checks that emission does not raise, so it cannot
  see an emitted rule that is wrong for the drop. Step 4 found exactly one
  such case (the UUID validator).
- **Step 2 (interaction overlay): landed in the working tree.** The overlay
  lives at `pim/agra_p2_seam/` and adds 16 services over the generated P2
  data model: four Request/Requirement command services (mission plan,
  mission plan activation, planning function settings, approval request)
  and twelve single-variant Information services. Topic names are the A-GRA
  global element names; every operation is reliable/volatile with queue
  depth 10. Notes for whoever picks this up:
  - Correlation identifiers for each command/status pair are documented in
    `pim/agra_p2_seam/README.md`. All of them bottom out in `ID_Type.uuid`,
    which is 16 bytes and 32 unhyphenated hex characters on the wire.
  - The overlay keeps byte-for-byte copies of the generated data-model
    proto, `wire_names.json`, and the shared options proto rather than the
    P1 overlay's symlinks, because a native Windows checkout may
    materialize a symlink as a text file. `pim/test_agra_p2_seam.py`
    compares each copy against its source, which guards a wire-name lookup
    that would otherwise fail silently.
  - Schema and drop identity travel through a new generic mechanism: an
    input tree may contain `binding_metadata.json`, and
    `generate_bindings.py` copies that object verbatim into the generated
    manifest's `metadata` field. This keeps contract-specific identity out
    of the generator's command-line options.
  - `binding_contract.py` previously located a service's request,
    requirement, or information topic only by matching derived wire-name
    suffixes. Explicit contract topics (like A-GRA's element names) do not
    carry those suffixes, so it now falls back to the RPC role that owns
    the topic.
- **Step 4 (offline fidelity): the round-trip loop is closed for all 20
  roots; goldens and Ada parity still to do.** The P1 pattern now has a P2
  twin in `pim/test_oms_json_gen.py`
  (`AgraHarnessRoundTripTest`, one test per profile root plus unknown-root
  and malformed-input negatives), driven by
  `pim/test_oms_json_gen_fixtures/p2_smoke_driver.cpp`. Demonstrated green
  2026-07-15 in WSL (Ubuntu, g++ 13.3): the P2 codec builds and all 20
  roots decode and re-encode to a semantically identical document, with P1
  re-run alongside to confirm no regression.
  - **Defect this found (now fixed): the generated codec validated every
    UUID against the UCI 2.5 hyphenated form.** The emitter picked the
    validator by field *name*, so the A-GRA codec rejected every
    schema-valid A-GRA UUID on both encode and decode, and all 20 roots
    failed. The fix keys the validator on the converted field *type*,
    which already carries the facet difference: UCI 2.5 types UUID as
    `xs:string` with the RFC-4122 pattern and converts to `string` (36
    characters, hyphenated), while A-GRA 5.0a types it as `xs:hexBinary`
    length 16 and converts to `bytes` (32 hex digits, no hyphens). Each
    validator is emitted only when the tree uses it, which keeps the
    frozen seam golden byte-identical. Pinned by `AgraUuidFormTest` and
    `P1SidecarGenerationTest.test_uci_uuids_keep_the_hyphenated_rfc_4122_form`,
    both of which run without a toolchain.
  - **Harness gap handled in our tests, not in the harness.** The
    la-cal-harness emits hyphenated UUIDs whatever the schema says, so
    `_normalize_agra_uuids` rewrites them to the A-GRA wire form before
    they reach the codec. Patching the harness instead would have weakened
    the evidence, since the round trip is only meaningful while the
    harness stays an independent implementation of the same XSD.
    `AgraUuidNormalizationTest` checks both halves against the schema type
    itself (53 UUID fields across the 20 roots: every raw value invalid,
    every rewritten value valid), so if the harness ever gains facet
    awareness this fails loudly rather than silently rewriting nothing.
  - The Ada emitter needs no matching change: it is encode-only and does
    not validate UUIDs, so it has no hardcoded lexical form. The Ada
    hits for `uuid` are all inside the frozen seam template.
  - Remaining in step 4: deterministic goldens, schema-drop mismatch
    negatives, and C++/Ada wire-parity coverage for P2.
  - Note for reproduction: the harness lives at
    `external/ams-gra/la-cal-harness/`, not the upstream long name the
    tests previously defaulted to, so the P1 round trip had been skipping
    silently in this checkout. `_harness_src` now accepts both names, and
    `LACAL_HARNESS_SRC` still overrides. Its generator needs `lxml`,
    `xmlschema`, and `exrex`, which are not in the repo's Python
    environment; the WSL run used a venv with those installed.
- Remaining: step 5 (distinct codec plugin + `--gra` SDK packaging), step
  6 (bidirectional Sleet interop + fail-closed negatives), step 7
  (evidence publication; the compatibility matrix and user guide still
  describe P2 as offline-only until then).

This workstream closes the gap between the
offline-convertible formal A-GRA 5.0a/P2 schema and a distributable,
interoperable OMS JSON codec. The current `pim/agra_example/` remains a
non-wire port-grammar fixture and is not an input to this workstream.

**Goal:** produce a schema-derived A-GRA P2 contract, generated OMS JSON codec,
interaction facade, and LA-CAL deployment profile that can be packaged by the
offline SDK and validated against an independent conformant endpoint. This is
codec/transport support, not a claim of full A-GRA platform compliance.

**Prerequisites (must be recorded before implementation starts):**

1. Pin the authorized A-GRA 5.0a schema source and checksum, and resolve the
   redistribution posture. Raw XSD remains fetch-not-vendor unless its licence
   explicitly permits redistribution.
2. Approve the P2 root list and closure budget from
   `pim/uci_profiles/p2_agra_planning_core.json`, including the exact OMS
   schema identifier used during LA-CAL `INIT`.
3. Identify an independent A-GRA-compatible OMS/CAL validator or peer plus
   authoritative fixtures. UCI 2.5 Sleet evidence cannot validate the UCI 2.3
   plus `MA_*` A-GRA drop.

**Plan:**

1. **Materialize the contract.** Re-run `xsd2proto --check` for P2, prune the
   transitive closure deliberately, and produce a deterministic proto tree,
   closure report, and authoritative `wire_names.json`. No wire name may depend
   on snake-to-Pascal fallback.
2. **Add the interaction overlay.** Define the minimum Request/Requirement and
   Information ports needed by the approved P2 roots, with A-GRA topic names,
   correlation identifiers, QoS floors, and schema/drop identity explicit in
   the generated manifest.
3. **Close generator shapes.** Extend and test the OMS generator only where P2
   demonstrates a missing XSD shape (choice, enum, attributes, optional/nil,
   inheritance, or wrapper unwrapping). Generate C++ and Ada bindings/codecs
   from the same proto plus wire-name sidecar; do not add hand-written `MA_*`
   encoding tables.
4. **Prove offline fidelity.** Validate representative and boundary instances
   for every selected global root against the authoritative XSD; add semantic
   round trips, deterministic goldens, malformed-input negatives, schema-drop
   mismatch negatives, and C++/Ada wire-parity coverage.
5. **Create a distinct runtime profile.** Build a profile-specific
   `application/oms-json` codec plugin whose artifact/schema identity cannot be
   confused with `pyramid_codec_oms_json_uci`. Pair it with the existing
   `pyramid_lacal_transport_plugin`, and make `create_sdk --gra --proto-dir`
   package the matching P2 contract and codec rather than an unrelated fixture
   or UCI 2.5 starter codec.
6. **Run independent interoperability.** Exercise LA-CAL `INIT`, `SUB`, `PUB`,
   and `MSG` in both directions against the designated A-GRA peer. Cover at
   least one `MA_*` command/status correlation and one information publication,
   including fail-closed schema, capability, and QoS mismatches.
7. **Publish the evidence boundary.** Record schema commit/checksum, profile
   roots, generated artifact hashes, validator/peer version, commands, and
   results. Update the user guide, SDK guide, and compatibility matrix without
   broadening the claim beyond the tested P2 profile.

**Acceptance:**

- A clean checkout can reproducibly materialize the approved P2 proto,
  `wire_names.json`, closure report, facade, and codec without hand-authored
  wire mappings.
- Every selected P2 global root passes authoritative offline XSD validation and
  C++ encode/decode tests; generated Ada output object-compiles and matches C++
  wire bytes, unless Ada support is explicitly waived in the profile decision.
- The profile-specific codec dynamically loads through the PCL codec ABI and
  refuses the wrong schema/drop; the LA-CAL transport retains its existing ABI,
  PUBSUB capability, teardown, and QoS tests.
- The packaged offline SDK builds the P2 bindings and codec, then smoke-tests a
  representative `MA_*` payload through the packaged codec and LA-CAL plugin.
- Bidirectional traffic is accepted by an independent A-GRA-compatible
  OMS/CAL peer. Until this evidence exists, status remains **offline-only** and
  the repository must not describe formal A-GRA OMS support as delivered.

**Non-goals:** full P3/Table 3-1 coverage; EXI/DMS offboard transport;
`agra_c2_bridge` mission semantics, approval policy, or AME goal mapping; and
general A-GRA certification beyond the selected, evidenced P2 OMS/CAL profile.

Detailed conversion design and current P2 measurements remain in
[`uci_mms_conversion_plan.md`](../../plans/PYRAMID/uci_mms_conversion_plan.md)
Phase 4; the live support statement remains
[`oms_agra_compatibility.md`](../../../subprojects/PYRAMID/doc/architecture/oms_agra_compatibility.md).

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
