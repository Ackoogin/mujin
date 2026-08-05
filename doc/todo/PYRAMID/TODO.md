# PYRAMID — Consolidated TODO & Plan

**Last consolidated: 2026-08-04.** This is the single tracker for **remaining**
PCL/PYRAMID work.

Finished work is no longer listed here. It is tracked on the roadmap dashboard
([`doc/roadmap/pyramid.yaml`](../../roadmap/pyramid.yaml), rendered to
`doc/roadmap/site/pyramid.html`), whose "Delivered" stream carries the same
context this file used to hold in prose. Anything removed from here remains in
git history, and the design intents of the plans retired in the 2026-07-10 doc
review are summarised in
[`doc/plans/PYRAMID/README.md`](../../plans/PYRAMID/README.md).

**Open defects are ordered in
[`defect_remediation_plan.md`](../../plans/PYRAMID/defect_remediation_plan.md)**,
which takes priority over the feature workstreams below. Five of the seven are
fixed and the Windows suite is now green at 828 of 828, so acceptance criteria
in this file can be demonstrated again.

**Two defects are in flight and uncommitted (2026-08-05):** D-4, codec dispatch
colliding on unqualified type names, and D-5, the generated JSON codec accepting
input that is not JSON. Changes for both are in the PYRAMID working tree and
**none of it has been verified** — the run was cancelled during its verification
step. Read the "In-flight work" section of the plan before touching either, and
do not commit without running the verification listed there.

**A note on paths.** PCL and PYRAMID became standalone submodules on 2026-07-22
and most paths moved: what was `pim/agra_p2_seam/` is now
`proofs/contracts/agra_p2_seam/`, and `subprojects/PYRAMID/tests/` is now
`proofs/tests/`. Paths in the WS-G checkpoint log below are pre-split and left as
written, because they record what was true at the time.

Live companion documents:

| Document | Role |
|----------|------|
| [`defect_remediation_plan.md`](../../plans/PYRAMID/defect_remediation_plan.md) | **Live plan (2026-08-04):** the open defects, ordered, with a proposed fix and acceptance for each. Takes priority over the workstreams below |
| [`port_grammar_entity_rename_plan.md`](../../plans/PYRAMID/port_grammar_entity_rename_plan.md) | **Delivered (2026-07-17):** retained for its impact analysis, compatibility policy, and validation matrix |
| [`high_efficiency_process_bindings_plan.md`](../../plans/PYRAMID/high_efficiency_process_bindings_plan.md) | **Live plan (2026-07-19), largely delivered:** a native in-process payload path in the generated port bindings, distinguishing Tier A (same executor, pointer handoff) from Tier B (sibling executor, native transfer over cross-thread ingress). Everything except the generator emission for native *services* is done; the plan's own status line still reads "first cut scheduled" and understates it |
| [`pyramid_split_and_tobj_pim_migration_plan.md`](../../plans/PYRAMID/pyramid_split_and_tobj_pim_migration_plan.md) | **Live plan (2026-07-06, not yet scheduled):** capability/consumers subproject split + Tactical Objects migration onto the PIM Osprey port-grammar contract; subsumes E5 when executed |
| [`uci_mms_conversion_plan.md`](../../plans/PYRAMID/uci_mms_conversion_plan.md) | **Live plan:** XSD-to-proto profile ladder; Phase 3/P1 is live-proven, while Phase 4/P2 supplies the detailed background for WS-G below |
| [`oms_agra_compatibility.md`](../../../subprojects/PYRAMID/proofs/doc/architecture/oms_agra_compatibility.md) | Proof support boundary and evidence for UCI 2.5/AMS-GRA versus formal A-GRA |
| [`pyramid_user_guide.md`](../../../subprojects/PYRAMID/doc/guides/pyramid_user_guide.md) | Single high-level user guide (design intent, usage, diagrams) |
| [`standard_alignment.md`](../../../subprojects/PYRAMID/proofs/doc/architecture/tactical_objects/standard_alignment.md) | Stable design reference for shipped Tactical Objects; one open design point (D-list row below) |
| [`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md) / [`ros2_transport_semantics.md`](../../../subprojects/PYRAMID/doc/architecture/ros2_transport_semantics.md) | How the plugin/codec system and the ROS2 mapping work |

## Standing regression bar (applies to every item below)

1. Default `pyramid` layout output stays **byte-for-byte identical**
   (`diff -qr` against a pre-change generated baseline) unless the item's
   acceptance says otherwise.
2. `python3 -m pytest subprojects/PYRAMID/tests -q` green, plus
   `python3 -m unittest subprojects/PYRAMID/pim/test_proto_parser.py`.
   **Both are green (checked 2026-08-04 on Windows):** the pytest suite reports
   59 passed and 6 skipped, and `test_proto_parser` runs 4 tests with no
   failures. The six skips are the `test_sdk_packaging_contract.py` cases that
   exercise the Linux packager shell scripts and cannot run on Windows; they
   skip with that reason rather than failing. Any *failure* here is now breakage
   from current work.
3. For generator changes touching Ada: object-compile the generated Ada for
   both trees (`gnatgcc -c -gnat2020`).
4. **CTest is green: 927 of 927 on Linux, and 828 of 828 on Windows in a clean
   shell (checked 2026-08-05).** Windows was unrunnable until 2026-08-04 — it
   aborted during test discovery — so any failure now is breakage from current
   work and should be treated as such. What was fixed to get here is recorded in
   [`defect_remediation_plan.md`](../../plans/PYRAMID/defect_remediation_plan.md).

   Three cautions when reading a result here.

   - **Build the Ada binaries first**
     (`cmake --build --preset release --target pyramid_ada_all`). Without them
     the Ada tests skip and report success, which is not the same evidence.
   - **Do not add the generated-codec directory to `PATH` to make something
     pass.** It masks real defects: that workaround is why this file once
     reported six Windows failures when there were eight.
   - **The first parallel build after a reconfigure fails** with exit 1 and no
     compiler or linker diagnostic. Running the identical command again
     succeeds. This is reproducible and is not a broken tree.
5. End-of-workstream (slow): `viability_check.sh`, `build_comms_test.sh`,
   `build_plugin_load_test.sh`, `build_contract_routing_test.sh`, and the
   packaged-SDK import smoke (`package_sdk.sh` then
   `python3 -c "import generate_bindings"` from the packaged `generator/`).

---

## Execution order

**Defects come first (added 2026-08-04).** The items below are features; the
open defects are ordered separately in
[`defect_remediation_plan.md`](../../plans/PYRAMID/defect_remediation_plan.md),
and the first three of those should be done before anything in this table. The
reason is bar item 4: the suite cannot currently report whether a change broke
something on Windows, so no acceptance below can be demonstrated there. The
plan's order is: restore the suite (discovery, the bridge plugin path, the Ada
crash), refresh the stale baselines, then the two codec defects that accept data
they should refuse — the unqualified-schema-id collision recorded in WS-D and
the JSON codec's permissive parse.

| Order | Item | Size |
|-------|------|------|
| 1 | G1 Formal A-GRA P2 OMS codec and CAL validation (in progress; reach a stable checkpoint before changing generated contracts) | L |
| 2 | F1 remainder: cross-process/remote-transport proof, D4 narrow case, CI wiring | M |
| 3 | F2(b) Tactical Objects example (rides on the PIM migration plan) | S/M |
| 4 | E5 Classify or migrate `StandardBridge` raw PCL wiring | S/M |
| 5 | G2 Write the P3 tracker entry (editing only; the source material exists) | S |
| 6 | WS-J remainder: the join map and its guard, the two committed tests, the assembly step | M |

---

## WS-E — In-process service/pub/sub facade closure

E1–E4 and E6 are delivered. E5 remains open.

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
`submit()`/`transitions()`, D1/D2/D3/D4-primary) is delivered. Remaining, not
blockers:

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

### F2(b). Convert the Tactical Objects showcase to the port abstraction

The C++ and Ada facade examples (F2(a) and F2(c)) are delivered. What remains is
the Tactical Objects showcase, which can only be converted once its contract
becomes grammar-conforming — its legacy create/read/update/delete services are
not port-grammar shapes today. This rides on
[`pyramid_split_and_tobj_pim_migration_plan.md`](../../plans/PYRAMID/pyramid_split_and_tobj_pim_migration_plan.md)
and no longer waits on the `Entity` rename, which is delivered.

---

## WS-G — Formal A-GRA OMS codec closure

### G1. Generate and validate the A-GRA P2 OMS/CAL profile

**Status: in progress (2026-07-14).** Checkpoint log for session handover:

- **Prerequisites 1–3 discharged.** Decision record:
  [`pim/uci_profiles/README.md`](../../../subprojects/PYRAMID/proofs/profiles/uci/README.md)
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
  into the P2 profile documentation at step 7 (the retired findings ledger is
  available in PYRAMID git history).
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
- **Defect found 2026-08-04: the compatibility matrix understates step 4 and
  step 5.** The one-page status table in
  [`oms_agra_compatibility.md`](../../../subprojects/PYRAMID/proofs/doc/architecture/oms_agra_compatibility.md)
  (lines 32–34) still records the formal A-GRA column as "Not generated — the P2
  tree exists only as an offline conversion artifact" for the C++ OMS-JSON
  codec, and "Not attempted" for both the Ada codec and the interaction facade.
  All three exist and are recorded above as proven offline: the codec target
  `pyramid_codec_oms_json_agra` builds and is packaged by `--gra`, all 20 roots
  round-trip, the Ada encoder object-compiles over the full closure and matches
  the C++ wire bytes, and the overlay is at `proofs/contracts/agra_p2_seam/`.
  The error understates rather than overclaims, which is the safer direction,
  but this document is named in `CLAUDE.md` and above as the live support
  statement, so it is what an outside reader consults. The three rows should
  state what is proven offline while leaving the live-interop claim untouched,
  since steps 6 and 7 genuinely have not happened. Correcting them does not
  require waiting for step 7.

### G2. Write the A-GRA P3 Core MMS tracker entry

**Status: open, and now an editing task.** Commits `6d65410` ("P3 Core MMS
contract: profile, seam, gated build") and `12e9bf9` ("Make P3 source-compatible
with P2") landed a P3 profile, the `agra_p3_seam/` overlay, and the generated
tree, all committed. Nothing in this file describes that work, while WS-G above
lists "full P3/Table 3-1 coverage" as a non-goal — which reads as contradicting
the tree until you know the evidence boundary.

The 2026-07-21 review concluded this needed someone with the original context.
It no longer does: the seam's own README supplies what the entry requires.
[`proofs/contracts/agra_p3_seam/README.md`](../../../subprojects/PYRAMID/proofs/contracts/agra_p3_seam/README.md)
records that P3 pairs the converted Core MMS data model (2,856 messages from
343 roots) with per-interface component service protos for the C2, MS, P2P and
VI interfaces on both provided and consumed sides — 951 services derived from
Table 3-1 — with topics taken from the schema's global element names and every
operation reliable and volatile at queue depth 10. Which side a service lands on
is inferred from the Direction column of Table 3-1 rather than hand-authored.

Crucially, it also states its own evidence boundary, which is what resolves the
apparent contradiction with the P2 non-goal above: "unlike the P1/P2 seams it
has no offline fidelity or live interop evidence", and no compliance claim
follows from the seam existing. So the non-goal is not contradicted — P3 exists
as a generated contract, not as an evidenced profile. Writing this into WS-G is
now an editing task.

This workstream closes the gap between the
offline-convertible formal A-GRA 5.0a/P2 schema and a distributable,
interoperable OMS JSON codec. The current `pim/agra_example/` remains a
non-wire port-grammar fixture and is not an input to this workstream.

**Goal:** produce a schema-derived A-GRA P2 contract, generated OMS JSON codec,
interaction facade, and LA-CAL deployment profile that can be packaged by the
offline SDK and validated against an independent conformant endpoint. This is
codec/transport support, not a claim of full A-GRA platform compliance.

**Prerequisites: all three discharged** (schema source and checksum pinned,
root list and closure budget approved, independent validator designated). The
detail is in the G1 checkpoint log above.

**Plan.** Steps 1–4 are complete; see the checkpoint log above for what each
produced and the defects each uncovered. Remaining:

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
[`oms_agra_compatibility.md`](../../../subprojects/PYRAMID/proofs/doc/architecture/oms_agra_compatibility.md).

---

## Scope note: A-GRA does not use the port grammar

**Recorded 2026-08-05**, after transient A-GRA port-grammar files were seen
being regenerated into the working tree.

The port grammar is for **Request ports**, whose primary realisation is RPC.
A-GRA is a different shape: it uses **CommandStatus ports**, whose primary
realisation is publish/subscribe. So A-GRA contracts should not use the port
grammar at all. This is a statement about which contract uses which construct,
**not** a deprecation — the port grammar remains the right thing for
Request-shaped contracts, and nothing outside A-GRA changes.

How that maps onto the tree today. There are three port kinds. `request` and
`information` are resolved from the grammar, by walking an interface block's
generalizations to `RequestService` or `ProviderService` respectively.
`command_status` is not resolved from the grammar: `_infer_command_status` in
`sysml_parser.py` spots an interface block pairing a command payload with a
status payload, and `_write_command_status_services` in `proto_generator.py`
emits the A-GRA `CommandStatusPort` shape. The A-GRA P3 seam already works this
way, pairing Command/Status elements into `CommandStatusPort` facades and
publishing everything else as one-way Information services.

What this does and does not affect:

- **E5 and F2(b) are unaffected.** Both move Request-shaped, CRUD-style
  Tactical Objects services onto the port abstraction, which is exactly what
  the grammar is for. They should proceed as written.
- **`agra_example`**, described in WS-G as a non-wire port-grammar fixture, is
  the one thing that sits on the wrong side of this line: an A-GRA-named fixture
  built on the Request-port grammar. Worth revisiting, and the likely source of
  the regenerated files that prompted this note.
- **J4's interim** looks better than it was recorded as. See below.

## WS-J — A-GRA message types as PYRAMID component port payloads (via the MBSE model)

**Opened 2026-07-24.** This workstream lets a native PYRAMID component,
authored in the SysML model, type one of its ports directly with an A-GRA
message, so the component contract is *realised with* the A-GRA data model
rather than using an invented local type that is later bridged to A-GRA at the
edge. The worked example is Mission Autonomy's `Objectives` component
publishing an action plan on a Provided Planning port whose payload is the
A-GRA `MA_ActionPlanMT` message.

The message bodies stay owned by `pim/xsd2proto.py` (the XSD-to-proto
converter), which remains authoritative for A-GRA per drop; the model owns only
the architecture — which component provides or consumes which port, and in
which direction. The two meet at the `pyramid.data_model.agra` proto module: a
component service file imports it and references its messages, exactly as the
`pim/agra_p3_seam/` interface services already do.

**Why A-GRA does not simply flow through the normal MBSE path.** A-GRA is
imported into SysML by the Open Arsenal XSD importer as blocks stereotyped
`OpenArsenal_XSDComponent` (carrying `qName`, `namespace`, `uciVersion`,
`xsdKind`, `xsdCompositor`, `xsdSequencePosition` tags) — a different shape
from PIM blocks, and 2856 messages across externally-versioned drops. Modelling
those natively would make the generator redefine every message from the model,
forking the XSD's authority and losing the byte-for-byte regeneration guard.
So A-GRA blocks are referenced, not re-authored. Full background and the
options considered are in the design discussion that opened this workstream.

**J1 (generator resolver branch) and J4 (command/status port inference) are
delivered** — see the roadmap's `MBSE-J1` and `MBSE-J4`. Two things about them
still bear on the open items below. J1's proof ran from a scratch directory and
is not in the repository, which is what J3 is for. J4 infers the port kind
rather than reading it from the grammar, because an interface block pairing a
command with a status generalises neither existing port kind.

J4's inference was recorded as an interim pending a `CommandStatus` base block
being added to the grammar alongside `RequestService` and `ProviderService`.
**The scope note above means that base block should not be added**: the grammar
is for Request ports, and A-GRA is not a Request-port shape, so putting
CommandStatus into it would mix the two. The inference is therefore nearer the
intended end state than the interim it was recorded as. If its classification is
ever made explicit, that belongs in the A-GRA side of the tooling rather than in
the grammar.

### J2. Real parser and join map — mostly DONE (the "real XMI" step, 2026-07-24)

Driven from the actual Cameo exports (`PRA_NA_PYRAMID_PIM.xml` referencing
`A-GRA_Domain_Model.xml`), which model an `Objectives` component whose provided
`MAAction` interface block pairs `command → MA_Action` and
`status → MA_ActionStatus`.

Reference resolution against those exports is **done**: an A-GRA-typed property
carries a cross-file type reference rather than pointing at a local block, and
the parser now extracts it and materialises one proxy class per referenced
element, which the J1 branch consumes. This means the 50 MB A-GRA export does
not need parsing at all, because the PIM export carries the identity.
Recognising a *local* `OpenArsenal_XSDComponent` block — the shape the original
scope assumed — is still worth adding for models that embed A-GRA blocks
directly. Remaining:

1. **Authoritative join map from `pim/xsd2proto.py` — still TODO.** The element
   name from the `referentPath` (`MA_Action`) is the XSD element, whereas the
   proto message is its complexType (`MA_ActionMT`). The generator resolves that
   with an injected `agra_type_map` (element → message name); for the real-XMI
   run it was passed `agra_p3_seam/wire_names.json` `roots`. The authoritative
   source remains a sidecar emitted by `xsd2proto` (see the note below on the
   suffix convention) — that is the piece still outstanding.

2. **Guard the join — still TODO.** Add a test that every referenced A-GRA
   element resolves to a message that actually exists in the A-GRA proto,
   failing closed on a miss, keeping the two independent importers from
   diverging across drops.

### J5. Promote the real-XMI proof to a committed test — TODO

Fold the parser + generator + bind assertions (run from a scratch directory
against the real exports) into a committed `pim/` test, ideally from a small
extracted XMI fragment so the 21 MB export is not a test input.

### J3. Promote the spike to a committed test — TODO

Move the fixture and assertions from the scratch spike into `pim/` as a proper
test (for example `test_agra_component_port.py`): a small parsed-model input
plus the checks that the emitted component service imports and references
`pyramid.data_model.agra` and does not redefine the message. Once J2(1) lands,
re-point it at the parser output from the real XMI fragment rather than a
hand-authored model.

### J6. Assembly step: two front-ends into one contract tree — TODO (sketch)

The pipeline assumes the A-GRA data model is generated **separately**: the MBSE
generator emits component service protos that import
`pyramid/data_model/pyramid.data_model.agra.proto` and reference its `*MT`
messages, but never writes that file. Producing a complete, bindable contract is
therefore a two-step assembly today, done by hand (copy the A-GRA data model and
options proto in beside the generated component protos, then run
`generate_bindings.py`). This item sketches a wrapper that does that assembly in
one command. It is a convenience/orchestration step — the two-step flow is
already correct without it.

**Inputs**

- The SysML PIM export (XMI), for example `PRA_NA_PYRAMID_PIM.xml`, whose
  A-GRA-typed ports reference the A-GRA element identities.
- The A-GRA data model, supplied one of two ways:
  - *reuse* a pre-generated A-GRA contract tree (for example `agra_p3_seam/`),
    which already carries `pyramid.data_model.agra.proto` and `wire_names.json`;
    or
  - *regenerate* from the A-GRA XSD via `xsd2proto.py` for a chosen profile/drop.

**Steps**

1. **Obtain the A-GRA data model + join map.** Either copy the pre-generated
   `pyramid.data_model.agra.proto` and read `wire_names.json`, or run
   `xsd2proto.py` to produce both. The `roots` map (element name -> message
   name, for example `MA_Action` -> `MA_ActionMT`) is the interim
   `agra_type_map`; the authoritative source is the xsd2proto sidecar once J2(2)
   lands.
2. **Model -> component protos.** Parse and resolve the PIM XMI
   (`sysml_parser`), then run `proto_generator` with `agra_type_map` set from
   step 1. This emits only `pyramid/components/**.services.*.proto`; it does not
   emit the A-GRA data model.
3. **Assemble one tree.** Place the A-GRA data model proto and
   `pyramid.options.proto` alongside the generated component protos in a single
   output directory so imports resolve.
4. **Guard (once J2(3) exists).** Fail closed if any A-GRA element referenced by
   a component does not resolve to a message present in the A-GRA proto.
5. **Bindings.** Run `generate_bindings.py` over the assembled tree.

**Output:** one contract tree (component services + A-GRA data model + options)
plus `binding_manifest.json`, ready to build.

**Design points to decide when implementing**

- **A-GRA source selection:** a `--agra-contract <dir>` (reuse) versus
  `--agra-xsd <path> --profile <p>` (regenerate); reuse is enough for the
  current flow.
- **Drop/version pinning:** record the A-GRA drop and `schema_version` used (from
  the reused contract's `binding_metadata.json` / `wire_names.json`) in the
  assembled output, so the model side and data side are known to match. This is
  the concrete guard against the version-skew fragility noted under J2(2): the
  model references element identities, and the assembly is where those are tied
  to a specific A-GRA drop.
- **Determinism:** the assembly must be byte-stable (standing regression bar #1)
  so a packaged contract regenerates identically.

**Illustrative shape (not final):**

```
python pim/assemble_agra_contract.py \
    --pim-xmi PRA_NA_PYRAMID_PIM.xml \
    --agra-contract proofs/contracts/agra_p3_seam \
    --out build/objectives_contract
# -> build/objectives_contract/pyramid/components/....services.provided.proto (generated)
# -> build/objectives_contract/pyramid/data_model/pyramid.data_model.agra.proto (copied)
# -> build/objectives_contract/pyramid/options/pyramid.options.proto (copied)
# then: python pim/generate_bindings.py build/objectives_contract build/bindings
```

### Constraints to hold (decided in the opening discussion)

- **Model the A-GRA element, not the `*MT` complexType.** A-GRA/UCI XSD names an
  outer global **element** (for example `MA_Action`, which Open Arsenal imports
  with `xsdKind='element'` and `qName='uci:MA_Action'`) and a **complexType**
  that types it (`MA_ActionMT`, "Message Type"), which in turn contains a
  "Message Data Type" (`MA_ActionMDT`). `xsd2proto` emits the proto `message`
  under the complexType name (`MA_ActionMT`); there is no bare `message
  MA_Action`. The element is the publishable message and carries the wire/topic
  name, so a port property links the **element** (`MA_Action`), and the tooling
  applies the element → complexType join (`MA_Action` → `MA_ActionMT`) via the
  join map. Linking `*MT`/`*MDT` directly would bind the model to xsd2proto's
  internal type naming and lose the element identity that becomes the topic.
- **Only named A-GRA roots and complexTypes are referenceable.** `xsd2proto`
  synthesises messages for anonymous inline complexTypes and list wrappers that
  have no standalone XSD type, and Open Arsenal models those as nested
  structure with no corresponding block. A port can therefore be typed only
  against a named A-GRA message (the 343 roots and the named complexTypes), not
  against a synthesised inner type.
- **"Shared surface" is at the namespace and topic level, not the type level.**
  The A-GRA data model is self-contained: it carries its own UCI equivalents of
  `Identifier`, `Timestamp`, and the base wrappers, and does not reuse
  `pyramid.data_model.base`/`common`. An A-GRA-typed port is a first-class
  provided or consumed PYRAMID port (native package, role, topic, QoS), but its
  payload is UCI-idiomatic. Decide, per component, whether a component may mix
  A-GRA-typed and PIM-typed ports or whether A-GRA-typed ports stay on their own
  components.

**Relationship to WS-G.** WS-G is about the A-GRA OMS/CAL *wire codec* and
platform-compatibility evidence. WS-J is about *modelling*: using A-GRA message
types to realise PYRAMID component contracts through the MBSE-to-proto path. The
two share the `pyramid.data_model.agra` module but make no compliance claim on
each other.

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
| Tactical Objects bulk-detail path | Consumers need full detail in bulk | Decide between a standard batch-detail path vs overloading the match stream ([`standard_alignment.md`](../../../subprojects/PYRAMID/proofs/doc/architecture/tactical_objects/standard_alignment.md), Remaining Design Point). |
| Interaction-pattern options for the legacy tree / side-table deletion | Only if the frozen-compat stance changes | Resolved as *frozen compat, new consumers forbidden*; `standard_topics.py` stays scoped to the legacy layout. |
| Codec dispatch picks a codec by unqualified type name, so two contracts that share a type name collide | Any process that loads codec plugins generated from two different contracts, or the next hardening pass on codec dispatch | The registry deliberately lets several codecs share one content type and expects dispatch to tell them apart by schema id — but the schema id is the *unqualified* type name (`Ack`, `Query`, `Identifier`), so two contracts that each define a type of that name are indistinguishable. `pyramid_try_registry_encode`/`pyramid_try_registry_decode` in the generated service implementation walk every codec registered for the content type in registration order and use the first one that recognises the name, so whichever plugin was loaded first answers for all of them. Nothing reports a conflict: the value is encoded against the wrong definition, and any field the winning definition does not have is silently dropped. Found 2026-08-04 in `test_pcl_generated_interaction_facade`, which was loading the main contract tree's codec plugins as well as the three A-GRA example plugins it asks for by name. The main tree's `Ack` has only `success`; the A-GRA example's `Ack` also has `identifier`, so the provider's ack arrived at the consumer with an empty identifier, and the transition query keyed on that identifier then matched nothing. That test was fixed on the day by excluding it from the blanket plugin-environment loop in `proofs/tests/CMakeLists.txt` (it now loads only its own three plugins), which restores a green suite but leaves the collision itself untouched — any deployment that mixes contracts still hits it. A real fix is one of: key the registry on a contract-qualified schema id (the proto package is already known at generation time); give each contract its own registry handle and pass it to the generated encode/decode helpers instead of using the process-wide default; or have dispatch check that the value it is handed matches the schema it knows and fail closed on a mismatch rather than losing fields. The last of these also covers the row below, and both are cases of a codec accepting input it should refuse. |
| Generated JSON codec accepts non-JSON input | Any hardening pass on codec fail-closed behaviour, or the first time a wrong-format payload is silently accepted in a deployment | Handed a FlatBuffers payload for `ObjectDetail`, the generated JSON codec **returns success** and fills the target with default values rather than rejecting the input. Found 2026-07-21 while writing `test_ports_file_codec_selection_e2e.cpp`: an early version of that test asserted "the JSON codec must refuse these bytes" and failed. The permissive JSON parse accepts a leading FlatBuffers byte as a bare JSON value, so nothing downstream notices. This matters because a peer misconfigured to the wrong codec then produces silently empty data instead of a startup error. Note the contrast with the schema-drop mismatch negatives in WS-G step 4, where the OMS codec is explicitly required to fail closed in both directions. The e2e test was written to compare the *decoded value* rather than the decode status, so it does not depend on this behaviour either way and will keep passing once it is fixed. |
