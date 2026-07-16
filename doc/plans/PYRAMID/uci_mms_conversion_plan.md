# UCI MMS Conversion Plan — from seam test cases to a GRA-scale message set

**Status:** in progress — **Phases 0 and 1 complete** (2026-07-11). Both
drops are fetched and sha256-pinned (A-GRA 5.0a from the public a-gra repo;
UCI 2.5 from the public USAF standard repo at the exact revision Sleet's
`fetch-spec` pins, discovered via the sleet repo's schema-sourcing doc).
The converter is built, fixture-tested, and proven strict-clean on both
real drops: P1 → 515 messages + 188 enums from 6 roots (**tree checked in**
under `pim/uci_generated/uci_2_5_0/`, byte-stability test-guarded), P2 →
1,163 messages + 297 enums from 18 roots (tree not yet checked in — Phase-4
pruning gate). The D6 reconciliation against `uci_seam_example` is recorded
in `pim/uci_generated/README.md`: conversion supersedes the hand contract
in fidelity, wire-compatible throughout, one new codec-gen rule identified
(`base`-field flattening, Phase 2). `pim/test_xsd2proto.py`: 27 tests.
**Phase 2 is complete** (see its progress note): the C++ OMS-JSON emitter
is generalized (sidecar wire names, enums, oneofs, nested-base
flattening), the seam output is frozen byte-identical (golden +
`pim/test_oms_json_gen.py`), the generated P1 codec compiles and
round-trips end-to-end, the D4(a) loop is closed — schema-derived
instances from the foreign `la-cal-harness` generator round-trip
semantically identical through the generated codec for **all six P1
roots** — and **Ada parity is discharged**: the generalized Ada encoder
object-compiles over the full P1 tree (GNAT 13.3) and produces wire JSON
byte-identical to the C++ codec (checked-in parity golden). Next: Phase 3
(re-point the live Sleet harnesses at generated artifacts). Incorporates
a review of the A-GRA integration state as of 2026-07-11 (§2).
**Date:** 2026-07-11
**Design source:**
[`doc/research/AME/a_gra_standard_review.md`](../../research/AME/a_gra_standard_review.md)
§4 (XSD→proto mapping rules, profile-first strategy), §6.2 (Minimum Message
Set volumetrics);
[`ams_gra_oms_cal_join.md`](../../research/AME/ams_gra_oms_cal_join.md) §6
(rung 3: "profile tooling grows two emitters").
**Companions:**
[`la_cal_integration_plan.md`](la_cal_integration_plan.md) (rung 1 — the
transport + codec machinery this plan scales up),
[`kitty_hawk_pcl_consumer_plan.md`](kitty_hawk_pcl_consumer_plan.md) (the
first consumer of this plan's Phase-3 output).

---

## 1. What this plan is

The LA-CAL integration proved the whole pipeline — proto contract →
generated bindings + interaction facade → OMS JSON codec → `owp` transport →
real Sleet, with Ada parity — for a **hand-authored vocabulary of four UCI
messages**. This plan is the review of that state (§2–§3) and the concrete
path from those small test cases to a **minimum message set (MMS) that
supports a GRA**, by converting the message set from the governing XSD
rather than hand-authoring it (the `xsd2proto` work the research docs call
"rung 3").

"MMS to support a GRA" has two distinct readings, and the plan serves both
with one toolchain:

- **The working MMS for the AMS GRA environment** — the handful of UCI 2.5
  messages the Kitty Hawk simulation actually exchanges, plus the
  command/status seam pair. Convertible *and provable live today* (Sleet
  validates every frame against `UCI_MessageDefinitions_v2_5_0.xsd`).
- **The formal A-GRA MMS** — the L1 compliance surface from the MA L1
  Compliance Document Table 3-1: ~327 unique messages against
  `A-GRA_MessageDefinitions_v5_0_a.xsd` (UCI 2.3 + ~123 `MA_*` extensions),
  with the planning/autonomy substance concentrated in the `MA_*` set.
  Convertible with the same tool, but with **no live validator in this
  repo** (the A-GRA wire stack is EXI/DMS offboard and OMS/CAL onboard, and
  Sleet's schema is a different drop) — so its validation story is offline
  XSD-instance validation, honestly labelled as such.

**Standing rule carried over:** the conversion carries no semantics.
Everything A-GRA-*meaningful* (Action→goal mapping, approval gates, plan
bookkeeping, RBAC, status fan-out) belongs to a future `agra_c2_bridge` and
to AME — which now lives in a separate repo. This repo's deliverable is the
contract, codecs, bindings, and transports that such a bridge consumes.

---

## 2. Review — where the A-GRA integration actually is

What exists and is verified (all 2026-07-11, see
`subprojects/PYRAMID/pim/test_harness/FINDINGS.md`):

| Layer | Artifact | State |
|-------|----------|-------|
| Contract (proving) | `pim/agra_example/` — A-GRA-vocabulary contract (`MA_Action`, `MA_ActionPlan`), grammar/options/classifier discipline | Done; **not** schema-valid against any real UCI drop (Sleet would reject it) — it proved the port grammar, not the vocabulary |
| Contract (schema-valid) | `pim/uci_seam_example/` — UCI-2.5-faithful `ActionCommand`/`ActionCommandStatus` Request-port contract | Done; hand-authored, mechanically XSD-shaped |
| Codec (hand-written) | `pyramid_oms_json_codec_uci.cpp` — 4 UCI roots: `PositionReport`, `SignalReport`, `ActionCommand`, `ActionCommandStatus` | Done; golden-fixture tested against captured harness payloads |
| Codec (generated) | `pim/backends/oms_json_backend.py` + `pim/{cpp,ada}/oms_json_codec_gen.py` | Done for the seam contract only; byte-equivalent to the hand codec; Ada object-compiles |
| Transport | LA-CAL plugin (`owp` over WebSocket), fail-closed INIT, PUBSUB-only caps | Done; capability matrix row recorded |
| Interaction seam | Correlated request/requirement over real Sleet, both raw-primitive and **generated-facade** forms; rpc-impossible negative | Done (`lacal_seam_test`, `lacal_generated_seam_test`) |
| Interop | Both directions vs the independently-authored `la-cal-harness` (XSD-derived generator/validator) | Done |
| Live environment | Full Kitty Hawk stack persistent under `external/ams-gra/` (git-ignored); OWP sniffer confirms live `PositionReport`, `ObservationMeasurementReport`, `ServiceStatus` | Done |
| XSD-driven anything | — | **Does not exist.** No `xsd2proto`; no XSD is pinned in-repo (`doc/research/AME/a-gra-main/` was a local working copy, not checked in; the UCI 2.5 XSD lives only in the git-ignored external checkout) |

The honest summary: **the pipeline is proven end-to-end, but every message
in it was hand-authored, and the vehicle for scaling — the XSD — has no
tooling and no pinned presence in the repo.** The distance from "4 messages"
to "an MMS" is not more of the same hand work; it is a converter plus the
generalizations in §3.

---

## 3. Gap analysis — what breaks between 4 messages and an MMS

1. **Hand-authoring does not scale and cannot stay faithful.** The A-GRA
   drop is 841 top-level messages / ~4,800 complex types; even the ~123
   `MA_*` planning core plus its transitive closure is far past
   hand-transcription. Fidelity drift between a hand proto and the XSD is
   already the known risk the interop tests exist to catch — at MMS scale
   only generation from the same parse keeps proto, codec, and schema
   aligned.
2. **Wire-name derivation is heuristic and will mis-render at scale.** The
   OMS-JSON generator derives UCI element names from proto field names via
   snake→Pascal plus a two-entry acronym table (`id`→`ID`, `uuid`→`UUID`).
   Real UCI element names include acronym and casing forms that heuristic
   cannot reproduce (`WGS_HAE`, `LOS`, `EO`, `RF_…`, digit-bearing names).
   The XSD is the authority; the converter must carry each element's exact
   wire name through as explicit per-field metadata, demoting the heuristic
   to a fallback for hand-authored contracts.
3. **The codec generator's shape support is deliberately narrow.** Today:
   no `oneof` (the A-GRA XSD has 445 `xs:choice` sites), no enums (~930
   simple types/enums), optional-string presence conflated with emptiness,
   and the two Request-wrapper structs plus the root list are hard-coded to
   the seam contract's names. All four must be generalized before any
   profile bigger than the seam pair can ride the generated path.
4. **Two schema drops, not one.** The runnable environment validates
   UCI 2.5; the formal MMS is defined against A-GRA 5.0a (UCI 2.3 base).
   These are siblings, not the same schema — the same message name can
   differ between drops. The converter must be drop-parameterized, each
   drop's output must live in its own package/tree, and generated types
   must never be shared across drops.
5. **Diff and field-number stability.** Deterministic field numbers from
   XSD declaration order are diff-stable across converter re-runs, but an
   upstream schema update that inserts an element renumbers everything
   after it. Since the wire here is OMS JSON (name-keyed, not
   number-keyed), renumbering is survivable — but each generated tree must
   be pinned to an exact schema version, and a drop upgrade is a new tree,
   reviewed as a diff, never an in-place mutation.
6. **Codegen volume.** Even the P2 profile's closure will multiply the
   generated C++/Ada/frozen-C-struct surface severalfold. Per-package
   backend selection already exists; the converter must report closure size
   so profiles are pruned deliberately (the review's "scale creep" risk),
   and Ada generation can be deferred per-profile where nothing consumes it
   yet.
7. **The MMS is not where to start.** The compliance MMS (~327 messages,
   Core, no partial compliance) is a target to *reach via profiles*, not a
   first deliverable. The profile ladder in §5 keeps every step provable.

---

## 4. Design decisions (fixed up front)

- **D1 — One converter, drop-parameterized.** `pim/xsd2proto.py` takes
  `(XSD set, profile manifest)` and emits a self-contained proto tree per
  drop: `pim/uci_generated/uci_2_5_0/…` and `pim/uci_generated/agra_5_0a/…`
  with distinct proto packages. No type sharing across drops, ever.
- **D2 — Profile manifests + checked-in output.** A YAML manifest per
  profile lists top-level message names; the converter computes the
  transitive type closure, emits it, and prints a closure report (type and
  message counts). Generated protos are **checked in** (like the rest of
  the contract trees) so schema-drop upgrades and converter changes are
  reviewable diffs; a CI test re-runs the converter and asserts
  byte-identical output (the existing generator regression-bar discipline).
- **D3 — Wire names are data, not derivation.** The converter emits each
  field's exact XSD element name as explicit metadata consumed by the
  OMS-JSON codec generator (sidecar `wire_names.json` next to the generated
  tree, or a `pyramid.options` field option — decided in Phase 1 by
  whichever survives `proto_parser.py` with less invasiveness). The
  snake→Pascal heuristic remains only for hand-authored contracts, and a
  converter self-check flags every name the heuristic would have gotten
  wrong (measures how much the metadata is earning).
- **D4 — Validation ladder, honestly tiered.**
  (a) *Offline:* every generated codec's output for generated sample
  instances validates against the XSD, reusing the `la-cal-harness`
  XSD-derived generator/validator machinery already used in Phase 4
  interop (SKIP-safe on absent XSD, same as today).
  (b) *Golden:* for the four existing roots, the new pipeline must be
  byte-equivalent to the hand-written codec (which becomes a frozen golden
  fixture).
  (c) *Live:* UCI 2.5 profiles are additionally proven through real Sleet
  and the Kitty Hawk stack. A-GRA 5.0a profiles get (a) only — stated in
  the artifacts, no implied compliance claim.
- **D5 — Mapping rules per the standard review §4.2.** `xs:choice`→`oneof`
  (nested wrapper message when the choice is repeated), extension→
  composition (`X base = 1;`), enums with a `*_UNSPECIFIED = 0` sentinel
  added, `xs:documentation` carried as comments, `uci:version` retained as
  comments, deterministic field numbers from declaration order. Deviations
  discovered against the real XSD are recorded in the converter's README as
  amendments to that table.
- **D6 — `uci_seam_example` is the reconciliation fixture.** Phase 1's
  first output is the converted `ActionCommand`/`ActionCommandStatus`
  closure; it is structurally reconciled against the hand-authored contract
  (differences documented and resolved toward the XSD). Once Phase 3 lands,
  the seam harnesses run on the generated tree and the hand tree is
  retained only as the golden fixture of record.
- **D7 — No semantics, no bridge, no transport growth.** Out of scope:
  `agra_c2_bridge`, anything AME-side, EXI/DMS, OMS/CAL C++ plugin
  (rung 4). This plan only widens the message surface the existing
  machinery carries.

---

## 5. Profile ladder

| Profile | Drop | Contents | Validation tier (D4) |
|---------|------|----------|----------------------|
| **P1 — Kitty Hawk working set** | UCI 2.5 | `PositionReport`, `SignalReport`, `ActionCommand`, `ActionCommandStatus`, `ObservationMeasurementReport`, `ServiceStatus` (+ closure) | (a)+(b)+(c) — live vs Sleet and the full stack |
| **P2 — A-GRA planning core** | A-GRA 5.0a | The `MA_*` planning/approval substance: `MA_MissionPlan*`, `MA_PlanningFunction*`, `MA_ApprovalPolicy/Request*`, `MA_Action*`/`MA_Task*`, `MA_Response*`, `MissionContingencyAlert` + referenced UCI plan/approval/status types (per review §6.2; a deliberate subset of the ~123 `MA_*` messages, pruned by closure report) | (a) offline only |
| **P3 — full Core MMS** | A-GRA 5.0a | The Table 3-1 surface incl. capability-family quintets (343 messages by direct row parse; the review's "~327" was summary arithmetic) | Validation not scheduled; only under a real compliance tasking (needs EXI/DMS besides — review §7 Phase 4). The *contract itself* exists as of 2026-07-16: converted tree, generated interaction seam (`pim/agra_p3_seam/`), and an OFF-by-default build gate (`PYRAMID_ENABLE_AGRA_P3`) — see `pim/uci_profiles/README.md` §"P3 conversion-scope decision"; no fidelity or interop evidence, no compliance claim |

P1 is deliberately the same message set the LA-CAL and Kitty Hawk plans
already touch: converting it replaces hand work with generated work under
tests that already exist, which is the cheapest possible proof of the
converter. P2 is where the plan starts paying toward A-GRA: it is the
contract input a future `agra_c2_bridge` (and the AME repo) would consume.

---

## 6. Phases

### Phase 0 — Schema acquisition and pinning (small)

**Progress: complete (2026-07-11).** Infrastructure:
`pim/schemas/{schema_manifest.json,fetch_schemas.py,README.md}`,
`pim/uci_profiles/{p1_kitty_hawk,p2_agra_planning_core}.json` (JSON, not
YAML: pim stays stdlib-only). **Both drops fetched and sha256-pinned**:
A-GRA 5.0a via raw.githubusercontent.com; UCI 2.5 via a `git`-kind source
from the public USAF UCI standard repo
(`gitlab.com/open-arsenal/uci/standard` @ `73a286fc...` — the exact
revision Sleet's own `make fetch-spec` pins, found via the sleet repo's
`docs/schema-sourcing.md`), **including `UCI_SecurityMarkings_v2_5_0.xsd`
which MessageDefinitions xs:includes**. P2's roots reconciled against the
real drop's 841 top-level elements (the review-guessed
`MA_MissionPlanStatus` does not exist; `MA_MissionPlanActivationStatus`
does). Redistribution posture recorded as unresolved → fetch-not-vendor in
`pim/schemas/README.md` (the UCI upstream's "Government Owned" license
marker noted there).

1. Determine redistribution posture for both XSD sets (the a-gra repo is a
   public release; the review's Phase-0 residual — confirm CUI status
   before deriving checked-in artifacts — is resolved here, once, in
   writing).
2. If vendorable: check the XSDs in under `subprojects/PYRAMID/pim/schemas/`
   with provenance headers (upstream repo, tag/commit, sha256). If not:
   a fetch script + pinned sha256, and everything XSD-dependent SKIPs with
   a printed reason (the established Sleet/GNAT pattern).
3. Write the P1 and P2 profile manifests (contents per §5; P2 message list
   drawn from the compliance document's §4 tables).

**Exit gate:** both XSDs resolvable and pinned by hash; manifests reviewed;
redistribution posture recorded.

### Phase 1 — `xsd2proto` converter, proven on P1 (medium)

**Progress: complete — exit gate met (2026-07-11).** `pim/xsd2proto.py`
implemented (stdlib-only: `xml.etree`, no xmlschema/lxml dependency) with
the D5 mapping rules, D3 wire-name sidecar, closure report, deterministic
emission, and `--check`; two documented rule deviations (temporal types →
`string` matching the shipped OMS codec; prefixed enum values with
`*_UNSPECIFIED` sentinels). `pim/test_xsd2proto.py` (27 tests, all green
with both drops present) pins the mapping table against a hand-written
fixture XSD and re-derives both real conversions from the pinned drops.
**P1 converted strict-clean and checked in**
(`pim/uci_generated/uci_2_5_0/`: 515 messages, 6 synthesized, + 188 enums
from 6 roots; byte-stability guarded by
`test_checked_in_p1_tree_is_current`). **D6 reconciliation documented** in
`pim/uci_generated/README.md`: the conversion supersedes the hand contract
in fidelity (real choice arms, real `ID_Type` composition, real facets),
is wire-compatible throughout, and surfaces one required Phase-2 codec-gen
rule — XSD-extension `base` fields must flatten into the parent JSON
object on the wire (implemented in Phase 2). Bonus at-scale proof: P2 also
converts strict-clean (1,163 messages + 297 enums, `--check`-stable,
parseable; hub fan-in `ID_Type` 74 / `SystemID_Type` 49 /
`ForeignKeyType` 40) — its tree stays untracked until the Phase-4 pruning
gate.

1. `pim/xsd2proto.py`: parse the XSD (Python `xmlschema` or `lxml` — pick
   in-phase; no new C++ deps), resolve the profile closure, emit protos per
   D1/D2/D5 plus the D3 wire-name metadata. Deterministic single-pass
   emission, `--check` mode for CI byte-identity.
2. Closure report output (message/type/enum counts, per-hub-type fan-in) so
   profile pruning is informed.
3. Convert P1. Reconcile the `ActionCommand` closure against
   `uci_seam_example` (D6) and the other four roots against the frozen-C
   structs the hand codec uses; document every divergence.
4. Unit tests: mapping-rule table cases (choice, repeated choice,
   extension, enum sentinel, optional/repeated), golden output for P1,
   closure-report snapshot.

**Exit gate:** P1 protos generated, checked in, byte-stable under re-run;
reconciliation against the hand-authored contract documented; closure
report in the converter README.

### Phase 2 — Generalize the OMS-JSON codec generator (medium)

**Progress (2026-07-11): the C++ emitter is generalized and proven; Ada
generalization and the D4(a) XSD-validation harness remain.**
`pim/cpp/oms_json_codec_gen.py` now emits one plugin per UCI-shaped
data-model package: package/roots/wrapper-structs derived from the contract
(no seam hard-coding), `wire_names.json` consumed as the authoritative name
source (measured: 191 P1 fields where the old heuristic would mis-render
the element name), enums as XSD literals with `*_UNSPECIFIED` rejected,
oneofs per the pinned choice rule (active member's element key directly in
the parent — pinned from the `la-cal-harness` XSD-derived generator before
coding, as this phase required), nested-base flattening for 2-deep
extension chains, and omit-empty arrays on the sidecar path.  The frozen
regression bar holds **by construction**: the seam-tree output is
byte-identical to the pre-generalization emitter (golden fixture +
`pim/test_oms_json_gen.py::SeamRegressionTest`).  Proven live in this
environment: the generated P1 codec (515 messages) object-compiles clean
with g++ 13 and completes a byte-stable encode→decode→re-encode round trip
(`OmsJsonCompileSmokeTest`, opt-in via `OMS_JSON_COMPILE_SMOKE=1`), with
correct wire output verified key-by-key (`CUI_Basic`, `"RECEIVED"`,
`OwnerProducer` choice, flattened `CommandID`).  Repeated `xs:choice`
carriers fail generation loudly (`OmsJsonShapeError`) rather than emitting
wrong JSON — absent from P1, tracked for P2.  **The D4(a) validation loop
is closed** (`HarnessRoundTripTest`, demonstrated green 2026-07-11): for
every P1 root, a minimal schema-valid instance produced by the
independently-authored `la-cal-harness` XSD-derived generator decodes and
re-encodes through the generated codec to a semantically identical
document (`$type` annotations stripped per the Phase-4 interop
convention) — simultaneously a decode-fidelity, encode-fidelity, and
schema-shape check, all six roots passing on first run.  **Ada parity is
discharged** (2026-07-11, GNAT 13.3 installed into the dev environment):
`ada/oms_json_codec_gen.py` gains a generalized emitter for
wire_names.json-backed packages — encode-only `To_Oms_Json` per profile
root (the C++ plugin remains the PCL wire implementation), JSON keys
statically sorted at generation time to match nlohmann's object ordering,
XSD enum literals with `*_Unspecified` raising, codegen-time flattening of
retained extension-base members, omit-empty arrays, and the documented
Ada presence envelope (optional strings by emptiness, optional enums by
the zero sentinel the conversion added for exactly this, optional
messages by generated `Is_Default_*` probes — the native Ada record layer
carries `Has_` flags only for optional scalars and oneof members, and the
Ada→C marshal has the same limit).  Gates passed: the generated P1
encoder **object-compiles clean over all 515 messages** (`gnatmake
-gnat2020`), and an Ada driver building the same `ActionCommandStatusMT`
as the C++ self-test produces wire JSON **byte-identical to the C++
output** (checked-in parity golden; `AdaCompileParityTest` holds both
languages to it).  The seam template remains byte-frozen for the seam
contract.

1. Remove the seam hard-coding: package name, root-message list, and
   Request-wrapper structs all derived from the contract tree + binding
   manifest (the port grammar already identifies request/requirement legs;
   the wrapper unwrap-to-root rule stays at the facade/transport boundary
   exactly as today).
2. Add the missing shapes: `oneof` (xs:choice representation pinned from
   OMSC-SPC-013 + harness fixtures before coding), enums (wire form =
   XSD enumeration literal via D3 metadata), true presence tracking for
   optional scalars/strings, and the D3 wire-name table as the naming
   source.
3. Ada emitter parity for every new shape (object-compile gate, GNAT
   pattern as today).
4. Validation harness per D4(a): for each profile message, generate
   instances (reuse/extend the `la-cal-harness` XSD-derived generator),
   round-trip encode→decode→encode through the generated C++ codec, and
   XSD-validate the wire form. CTest-registered where XSD-independent;
   SKIP-safe where not.

**Exit gate:** generated codecs for all of P1 pass D4(a)+(b); `owp.*` and
codec suites green; Ada parity holds.

### Phase 3 — P1 live proof and hand-codec retirement (small/medium)

**Progress: complete — exit gate met, live-verified (2026-07-12).**
`pim/uci_p1_seam/` adds the PIM interaction/information overlay over the
checked-in converted P1 tree; the generated seam witness now builds from
it, exercising the real `ActionCommandMT`/`ActionCommandStatusMT` C++ shape
(composed `MessageType base`, `ID_Type` composition, real `oneof` arms).
`ObservationMeasurementReport`/`ServiceStatus` (with the companion
`PositionReport`/`SignalReport` topics) ride the generated path as four
information-service ports. The legacy hand codec is explicitly frozen as
the byte-equivalence fixture; no message was added to it. Both live
proofs pass against the persistent `external/ams-gra` Kitty Hawk stack from
a from-clean rebuild: `build_lacal_generated_seam_test.sh` → `PASS:
generated UCI facade LA-CAL seam over Sleet`; `build_kittyhawk_consumer_test.sh`
→ all four information topics PASS with real decoded samples (see
`pim/test_harness/FINDINGS.md`'s dated entry for the exact counts and the
three real bugs this surfaced and fixed — a `wire_names.json` symlink
pointed at the wrong path, `oms_json_codec_gen.py` never generated
wrapper-unwrap handling for single-variant information wrappers, and the
LA-CAL plugin's OWP message-name mapping needed the same generalization).
`python3 -m pytest subprojects/PYRAMID/tests`, `pim/test_proto_parser.py`,
and `pim/test_oms_json_gen.py` stay green; default `pyramid` contract
layout output confirmed byte-identical to pre-change.

1. Re-point the seam harnesses (`lacal_seam_test`,
   `lacal_generated_seam_test`) and the Phase-4 interop driver at the
   generated P1 tree; rerun against pinned Sleet — all existing PASSes
   reproduced on generated artifacts.
2. Deliver the `ObservationMeasurementReport`/`ServiceStatus` codecs the
   [Kitty Hawk consumer plan](kitty_hawk_pcl_consumer_plan.md) needs from
   the generated path (that plan's deliverable 1 collapses into "add the
   message to the P1 manifest").
3. Demote `pyramid_oms_json_codec_uci.cpp` to golden fixture: no new
   messages are ever added to it; a comment points here.

**Exit gate:** every live LA-CAL/Sleet proof runs on XSD-generated protos
and codecs; the hand-written codec is frozen.

### Phase 4 — P2: the A-GRA planning-core profile (medium/large)

1. Convert P2 from `A-GRA_MessageDefinitions_v5_0_a.xsd` (own tree, own
   packages per D1). Measure the closure; prune the manifest until the
   closure is deliberate, not accidental (`RequirementConstraintsType`-style
   hubs are the known ballooning risk).
2. Offline validation per D4(a) against the 5.0a XSD; no live leg exists
   and none is claimed.
3. Project the Command-2/ActionRequest-2 families onto Request-port
   contracts (correlated request/requirement pairs — the pattern the seam
   already proved) and Data-1/Status-1 onto information topics, for the
   messages P2 covers. This yields the consumable contract surface for a
   future `agra_c2_bridge`.
4. Record the drop-delta findings (UCI 2.3-base vs 2.5 divergences actually
   observed) in the converter README — input to any later single-source
   strategy, and evidence for the "siblings, not the same schema" risk.

**Exit gate:** P2 tree + contracts generated, offline-validated,
closure-reported, checked in; a worked example (e.g.
`MA_MissionPlanCommand` → `MA_MissionPlan` → approval → execution status
message sequence, offline round-trip) demonstrating the planning-core
vocabulary end-to-end through codec + facade, mock-brokered.

### Phase 5 (conditional, unscheduled) — toward P3 / compliance

Only under a real compliance tasking: capability-family long-tail stubs
mass-produced from the same converter, EXI codec backend + DMS-conformant
DDS transport, OMS/CAL onboard adapter. Tracked in the a-gra review §7
(Phase 4 row); deliberately not planned here.

---

## 7. Risks and open questions

| Risk | Handling |
|------|----------|
| XSD redistribution/CUI posture blocks vendoring | Phase-0 gate; fetch-script + hash-pin fallback keeps everything SKIP-safe |
| OMS JSON rules for choice/enum/attribute shapes under-specified by the markdown spec conversions | Pin from the official OMSC-SPC-013 text + `la-cal-harness` fixtures *before* Phase-2 coding (the Phase-0-of-LA-CAL precedent); Sleet acceptance is the live arbiter for UCI 2.5 |
| Drop divergence (UCI 2.3-base vs 2.5) invalidates "one converter" | D1 already isolates trees; Phase-4 step 4 measures the actual delta before any sharing is attempted |
| Closure ballooning on P2 | Converter closure report + deliberate manifest pruning as an exit-gate item |
| Codegen volume slows builds (Ada, frozen-C structs) | Per-package backend selection; Ada deferred per-profile until a consumer exists; measure at Phase-2 exit |
| Generated-vs-hand byte-equivalence too strict once shapes generalize | Byte-equivalence is required only for the four frozen roots; everything else is round-trip + XSD-validity (D4) |
| Schema churn upstream (`MA_*` set pending OACWG submission) | Drop-pinned trees; upgrades are new trees reviewed as diffs (D2); never in-place |
| `proto_parser.py`/options machinery can't carry per-field wire names cleanly | D3 keeps the sidecar-file option open specifically so the converter never blocks on options-plumbing work |
