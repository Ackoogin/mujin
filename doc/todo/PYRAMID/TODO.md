# PYRAMID — Consolidated TODO & Plan

**Last consolidated: 2026-07-20.** This is the single tracker for remaining
PCL/PYRAMID work. Completed workstreams have been folded into "Delivered"
below; the executed plans, reviews, and status reports that used to carry
their detail were removed in the 2026-07-10 doc review — their design
intents are summarised in
[`doc/plans/PYRAMID/README.md`](../../plans/PYRAMID/README.md) and their
full text is in git history.

Live companion documents:

| Document | Role |
|----------|------|
| [`port_grammar_entity_rename_plan.md`](../../plans/PYRAMID/port_grammar_entity_rename_plan.md) | **Live plan (2026-07-17, proposed):** coordinated breaking rename of the Request-port result/update role, wrapper, topic, manifest leg, and generated APIs from `Requirement` to `Entity`, while retaining legitimate domain requirements |
| [`high_efficiency_process_bindings_plan.md`](../../plans/PYRAMID/high_efficiency_process_bindings_plan.md) | **Live plan (2026-07-19, proposed):** gap analysis + TODO for a native in-process payload path in the generated port bindings — distinguishes Tier A (same executor, zero-copy pointer handoff, PCL.022) from Tier B (sibling executor, native transfer over cross-thread ingress, PCL.025/026); the typed facades serialize even on local routes today |
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
| 1 | G1 Formal A-GRA P2 OMS codec and CAL validation (in progress; reach a stable checkpoint before changing generated contracts) | L |
| 2 | H1 Rename the port-grammar result/update role from `Requirement` to `Entity` | L |
| 3 | F1 remainder: cross-process/remote-transport proof, D4 narrow case, CI wiring | M |
| 4 | F2(b) Tactical Objects example (rides on PIM migration plan and follows H1) | S/M |
| 5 | E5 Classify or migrate `StandardBridge` raw PCL wiring | S/M |
| 6 | I1 Make the generated component codec selectable at process startup | S/M |

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
- **Step 4 (offline fidelity): complete.** The round-trip loop is closed
  for all 20 roots, and the deterministic golden, C++/Ada wire parity, and
  schema-drop mismatch negatives all landed on 2026-07-15 (see the two
  sub-entries at the end of this step). The P1 pattern now has a P2
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
  - **Deterministic golden and C++/Ada wire parity (`P2AdaCompileParityTest`).**
    `p2_smoke_driver.cpp` gained a self-test mode, and
    `p2_ada_parity_driver.adb` builds the same
    `MA_MissionPlanCommandStatusMT` value; both print wire JSON
    byte-identical to `test_oms_json_gen_fixtures/p2_parity_golden.json`.
    Demonstrated green 2026-07-15 in WSL: the Ada encoder object-compiles
    over the full 1,192-message A-GRA closure and its output matches the
    C++ codec's exactly, so the profile's Ada support is not waived. The
    golden sits beside the P1 one, where the drop difference is visible at
    a glance: the same field carries `123E4567E89B42D3A456426614174001` in
    A-GRA and `123e4567-e89b-42d3-a456-426614174001` in UCI 2.5. What the
    golden proves is byte-stability and agreement between the two
    emitters, not schema validity; schema fidelity remains the harness
    round trip's job, because only there does the input come from a
    foreign implementation of the XSD.
  - **Schema-drop mismatch negatives (`SchemaDropMismatchTest`).** Both
    codecs must fail closed on the other drop's traffic, and both
    directions are tested, since a codec that refuses foreign traffic but
    emits it is equally broken. This matters because the two drops share
    element names down to the message envelope (`MessageData`,
    `MessageHeader`, `SecurityInformation`), so wrong-drop content on the
    right topic looks plausible until something checks it. Two independent
    things do: the UUID lexical form (the runtime inverse of the defect
    above -- before the fix, the wrong-drop document was the one the A-GRA
    codec would have accepted) and the required elements of the message
    body. Each test asserts the codec refused on decode rather than merely
    that the driver exited nonzero, so a crash or an unknown root id
    cannot pass vacuously.
  - Note for reproduction: the harness lives at
    `external/ams-gra/la-cal-harness/`, not the upstream long name the
    tests previously defaulted to, so the P1 round trip had been skipping
    silently in this checkout. `_harness_src` now accepts both names, and
    `LACAL_HARNESS_SRC` still overrides. Its generator needs `lxml`,
    `xmlschema`, and `exrex`, which are not in the repo's Python
    environment; the WSL run used a venv with those installed.
- **Step 5 (distinct runtime profile): in progress.**
  - **Schema identity landed.** The generated OMS-JSON codec now carries the
    contract's `binding_metadata.json` as a schema identity and checks it in
    `pcl_codec_plugin_entry`, whose `config_json` the PCL ABI leaves opaque
    and plugin-specific for exactly this purpose. An identity key the loader
    names must agree; an unparseable or non-object config is refused rather
    than ignored; a NULL config still means "no configuration". Trees without
    metadata emit neither identity nor check, which keeps the frozen seam
    golden byte-identical.
  - **The build now creates the codec target.** Two gaps had kept the build
    from seeing the generated OMS codec at all: `oms_json` was not in the
    backend list even with `PYRAMID_ENABLE_OWP` on, and the OMS-JSON plugin
    was neither recorded in the binding manifest nor given the right content
    type (an OMS-JSON filename also contains `_json_`, so it was classified
    as `application/json`). Fixed; the target is
    `pyramid_codec_oms_json_agra`, built from the manifest role.
  - **Naming decision (approved 2026-07-15):** the hand-written UCI 2.5
    starter is now `pyramid_codec_oms_json_uci_starter`, freeing the plain
    `pyramid_codec_oms_json_<module>` name for codecs generated from a real
    contract — including the UCI one, which previously would have collided
    with the fixture outright. `--gra` never packages the starter.
  - **`--gra` packages the contract's codec.** `package_sdk` selects the
    codecs generated from the selected contract and asks the binding manifest
    whether one is expected, so a contract that declares a codec but failed to
    build it fails the package, while a port-grammar contract that declares
    none (`agra_example`) packages with a warning rather than a spurious
    error. `create_sdk --gra` warns when pointed at `agra_example`.
    Demonstrated 2026-07-15 in WSL: `--gra --proto-dir pim/agra_p2_seam`
    produces a package whose `plugins/` holds exactly
    `libpyramid_codec_oms_json_agra.so` and
    `libpyramid_lacal_transport_plugin.so`, with the contract,
    `wire_names.json`, and `binding_metadata.json` under `proto/`.
  - **Three FlatBuffers defects found and fixed on the way to `--verify`.**
    `create_sdk --gra --verify` failed inside the packaged build, in the
    FlatBuffers backend rather than the OMS codec. All three are pre-existing
    and reached by any XSD-derived tree, so the P1/UCI projection had them
    too; none affects the OMS codec, the LA-CAL transport, or any wire
    evidence recorded above.
    1. **A union arm may not be an enum.** A FlatBuffers union member must be
       a table. String arms already got a wrapper table; enum arms did not, so
       `flatc` rejected the schema outright ("type referenced but not
       defined": `OwnerProducerChoiceType.GovernmentIdentifier`). Enum arms
       are now wrapped the same way.
    2. **One union was shared by every choice type in the contract.** Unions
       were named from the oneof name alone, and `xsd2proto` names every
       `xs:choice` oneof `choice`, so all 183 tables carrying a choice pointed
       at a single `ChoiceUnion` holding the arms of whichever message was
       emitted first. This is the serious one: a build failure is loud, but
       this is a silently wrong projection. Unions are now named per
       (message, oneof).
    3. **Two arms of one choice may share a type.** An unnamed union arm
       contributes its *type* name to the union's implicit enum, so a choice
       with two elements of the same type (`CapabilityTaxonomyType`, twice)
       produced a duplicate. Arms are now named after their proto field, which
       proto guarantees unique within the oneof.
    Verified with real `flatc`: all six A-GRA schemas compile, including the
    1,192-message data model. Note for whoever picks this up: the FlatBuffers
    C++ codec flattens oneof arms and never reads the union, so these schema
    changes have no C++ codec counterpart.
  - **A fourth FlatBuffers defect, in the C++ codec rather than the schema:
    `bytes` was projected as `[ubyte]`.** That made the FlatBuffers side a
    `std::vector<uint8_t>` while the generated struct's field is
    `std::string`, so the codec's `out.uuid = msg.uuid` did not compile in
    either direction. `bytes` now maps to a FlatBuffers `string`, which is
    what the rest of the backend already assumed (`_is_generated_string_like`
    has always treated it as string-like). This is faithful rather than a
    workaround: in this repository a proto `bytes` field holds *text*, not
    raw binary — `xsd2proto` emits it only for `xs:hexBinary` and
    `xs:base64Binary`, whose values are the lexical form (A-GRA's UUID is 32
    hex characters), the generated C++ type is `std::string`, and the
    OMS-JSON codec writes it as a JSON string. No existing wire format moved:
    `bytes` appears exactly once in the repository's contracts (A-GRA's
    `ID_Type.uuid`), so the `[ubyte]` projection had never been exercised.
    This defect is A-GRA-specific, which is why P1 never surfaced it: UCI 2.5
    types its UUID as `xs:string`.
  - Remaining in step 5: the packaged-SDK `MA_*` smoke through the codec and
    the LA-CAL plugin (the acceptance bullet), and the dynamic-load evidence
    the SDK's `sdk_gra_plugin_load_smoke` provides.
- Remaining: step 6 (bidirectional Sleet interop + fail-closed negatives),
  step 7 (evidence publication; the compatibility matrix and user guide still
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

## WS-H — Port-grammar terminology

### H1. Rename the Request-port result/update role to `Entity`

**Status: proposed, not yet scheduled (2026-07-17).**

Rename the grammar wrapper `_Service_Requirement` to `_Service_Entity`, the
derived `.requirement` topic to `.entity`, and the interaction/configuration
leg `requirement_leg` to `entity_leg`. Regenerate all PIM, P1, P2, and P3
contracts and the C++/Ada APIs together.

This is a coordinated breaking contract change, not a global replacement of
the English word "requirement". Domain types such as
`ObjectEvidenceRequirement`, endpoint capability requirements, and formal
HLR/LLR requirements remain.

The full impact analysis, compatibility policy, phased implementation plan,
validation matrix, and completion criteria are in
[`port_grammar_entity_rename_plan.md`](../../plans/PYRAMID/port_grammar_entity_rename_plan.md).

---

## WS-I — Generated component codec selection

### I1. Make the generated component codec selectable at process startup

**Status: mechanism implemented in the C++ toolchain (2026-07-20); FlatBuffers
end-to-end proof and packaged-example refresh still open.**

Implemented so far (tracked source; the regenerated `pcl_pyramid_sdk_pim_test`
SDK is git-ignored, so its refreshed skeletons/scaffolds are on disk only):

- The `.ports` format was **deliberately extended and documented** (the option
  this item left open): a `codec CONTENT_TYPE PLUGIN` line loads a codec and
  sets the process default content type (first line wins), and a `port_codec
  PORT CONTENT_TYPE` line overrides one port so a process can speak several
  codecs. Parsed and validated in `pcl_process_runtime.c`; exposed as
  `pcl_process_runtime_content_type` / `pcl_process_runtime_port_content_type`
  and `ProcessRuntime::contentType()` / `contentTypeFor()`.
- The generated skeleton ctor now takes a `ContentTypeResolver` and threads a
  per-port content type into every port facade it owns (see
  `pim/cpp/component_skeleton_gen.py`). Default stays `application/json`.
- The generated scaffold self-wires: the component builds its own handler set,
  and `main` passes `runtime.contentTypeFor(port)` so each port uses the codec
  its `codec`/`port_codec` line selected.
- Fail-closed: a `port_codec` naming an unknown port or an unloaded codec is
  rejected with a clear error; the process-default codec load tolerates a codec
  a build-time fallback already registered. Pinned by
  `test_pcl_process_runtime.cpp` (`SelectsCodecFromPortsFile`,
  `SelectsPerPortCodecFromPortsFile`, `RejectsInvalidPortCodecLines`) and the
  regenerated component-skeleton baseline; JSON verified end-to-end by building
  and running the `pim_osprey_sensors` scaffold.

Remaining for I1's acceptance:

- End-to-end scaffold run with `application/flatbuffers` (the mechanism is
  codec-agnostic, but only JSON has been exercised end-to-end so far).
- A generator test that pins constructor/resolver propagation directly (beyond
  the golden baseline), and the packaged-SDK example/doc refresh.

Original analysis follows.

The generated C++ service and port facades accept a content type, but default
it to `application/json`. The generated component skeleton constructs every
port facade without passing a content type, so the default becomes fixed for
the component. The generated application scaffold reinforces that choice: its
CMake target embeds and loads the JSON codec plugin only. The `.ports` file
selects transport plugins and routes; it does not select a codec.

As a result, an SDK can build and package both JSON and FlatBuffers codec
plugins while an application built from the generated component skeleton
cannot select FlatBuffers without changing generated code. Loading only the
FlatBuffers plugin is not sufficient because the skeleton still tries to bind
its ports with `application/json` and fails the codec-registry check.

- **Plan:** add an explicit component codec setting to the generated C++
  skeleton constructor or a small process-startup configuration object.
  Thread the selected content type into every generated port facade owned by
  the skeleton. Update generated scaffolds to load the matching component
  codec plugin and pass the same content type to the skeleton. Keep JSON as the
  compatibility default. Keep codec selection separate from the existing
  per-port transport selection unless the `.ports` format is deliberately
  extended and documented.
- **Fail closed:** startup must report a clear configuration error when the
  selected content type is empty, unsupported, or has no matching loaded
  codec. A loaded codec plugin must not silently change the selected content
  type, and loading both codec plugins must not make registration order choose
  the component's wire format.
- **Accept:** the same generated component and application scaffold run with
  either `application/json` or `application/flatbuffers` without editing
  generated files. RPC and pub/sub realizations use the selected content type
  consistently for requests, responses, stream frames, and publications.
  Add generator tests that pin constructor/configuration propagation and
  end-to-end scaffold tests for both codecs, including the missing-plugin and
  content-type/plugin-mismatch failures. Refresh the packaged SDK example and
  document that `.ports` continues to select transports, not codecs.

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
| `cabi_codegen.py` non-deterministic typedef order | Next substantive change to `cabi_codegen.py`, or the first byte-stability guard failure it causes | `pyramid_data_model_generic_*_cabi.h` emits the generic container typedefs (the `*List`/`*Queue` `pyramid_slice_t` wrappers) in an order that varies between two runs of the *same* generator — observed as `RequirementList`/`RequirementQueue` swapping places. The output is functionally identical, but the instability undermines standing-regression-bar #1 (byte-for-byte identical output) and shows up as spurious diffs when regenerating a packaged SDK. Cause is an unsorted Python `set` iteration in the emitter (e.g. `set(self._aliases.keys())`); the fix is to sort the emission order (and ideally add a two-run determinism check). Found 2026-07-20 while regenerating the `pcl_pyramid_sdk_pim_test` SDK. |
