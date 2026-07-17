# Conversion profile manifests

Input manifests for `pim/xsd2proto.py` — one per profile of the
[UCI MMS conversion plan](../../../../doc/plans/PYRAMID/uci_mms_conversion_plan.md)
§5 ladder. A manifest names the schema drop (pinned in
`pim/schemas/schema_manifest.json`), the target proto package, and the
top-level **root message names** to convert; the converter resolves the
transitive type closure from the roots and reports its size, so profiles
are pruned deliberately rather than growing by accident.

Format deviation from the plan text: manifests are **JSON, not YAML** — the
pim toolchain is Python-stdlib-only and stays that way (PyYAML exists in
some dev environments but is not a dependency anywhere else in `pim/`).
Commentary lives in `_comment` arrays.

| Profile | Drop | Ladder rung | Validation tier | Converted (2026-07-11) |
|---------|------|-------------|-----------------|------------------------|
| `p1_kitty_hawk.json` | `uci_2_5_0` | P1 — working set, provable live vs Sleet | (a)+(b)+(c) | strict-clean: 515 messages + 188 enums from 6 roots; tree checked in |
| `p2_agra_planning_core.json` | `agra_5_0a` | P2 — A-GRA `MA_*` planning core | (a) offline, plus A-GRA-schema Sleet interop per WS-G/G1 | strict-clean: 1,169 messages + 297 enums from 20 roots; tree checked in (scope decision below) |
| `p3_agra_core_mms.json` | `agra_5_0a` | P3 — full Core MMS | none — no fidelity ladder run, no compliance claim (scope decision below) | strict-clean: 2,856 messages + 501 enums from 343 roots; tree checked in under `pim/uci_generated_p3/` (separate output root, not `pim/uci_generated/`); interaction seam generated into `pim/agra_p3_seam/` (722 services across the four Table 3-1 interfaces, plus 16 retained P2 compatibility services) |

Rules of the road (plan D1/D2):

- A profile targets exactly one drop; generated types are never shared
  across drops.
- Generated output is checked in and must be byte-stable under converter
  re-runs (`xsd2proto.py --check` is the CI guard).
- Growing a profile means reading the closure report the converter prints
  and pruning hub types deliberately.

## P2 conversion-scope decision — G1 prerequisite record (2026-07-14)

This section discharges the three WS-G/G1 prerequisites
([`doc/todo/PYRAMID/TODO.md`](../../../../doc/todo/PYRAMID/TODO.md#ws-g--formal-a-gra-oms-codec-closure))
before implementation.

A note on the word "closure", used throughout the conversion tooling: when
a profile selects root messages, every root references other message
types, which reference others in turn. The complete set of types reached
this way — everything the converter must generate — is called the
profile's *(transitive type) closure*. It is the "everything these roots
pull in" set, not the lambda concept. This and other recurring repository
terms are defined in [`doc/GLOSSARY.md`](../../../../doc/GLOSSARY.md).

### 1. Schema source and redistribution posture

Unchanged from Phase 0: the authorized source is the pinned `agra_5_0a`
drop in [`pim/schemas/schema_manifest.json`](../schemas/schema_manifest.json)
(schema_version `005.0a.ASK-20260423-f1380e7`, sha256-pinned
`A-GRA_MessageDefinitions_v5_0_a.xsd` + `A-GRA_SecurityMarkings_v5_0_a.xsd`).
Posture remains **fetch-not-vendor** (see `pim/schemas/README.md`): raw XSDs
are never committed; the generated tree carries drop name, schema version,
and source hash in its headers.

### 2. Root list and generated-set size budget — approved

**Decision: all 18 original roots of `p2_agra_planning_core.json` are
retained, plus two added the same day (see below), and the approved size
budget is everything the 20 roots pull in — 1,169 messages (30
synthesized) + 297 enums.**

The original 18-root measurement (2026-07-14, byte-identical to the
2026-07-11 measurement) was 1,163 messages + 297 enums. Two roots were
then added: the A-GRA drop annotates `MA_MissionPlanActivationCommand`
and `MA_PlanningFunctionSettingsCommand` with the Command-2 interaction
pattern, which requires each command to have a paired `*CommandStatus`
message for the correlated status reply — and the drop does carry
`MA_MissionPlanActivationCommandStatus` and
`MA_PlanningFunctionSettingsCommandStatus` as top-level elements, but the
original list omitted them (it predates the interaction-pattern
extraction). Without them, two of the profile's four command/status
exchanges cannot be expressed. Measured cost of adding both: +6 messages,
+0 enums (each contributes +3 messages beyond what the other roots
already pull in; alone they convert to 35 msg/16 enum and 19 msg/14 enum
respectively).

To make this pruning decision deliberate rather than accidental, each root
was measured two ways with `xsd2proto.py` against the pinned drop. The
**"converted alone"** column is how many messages and enums the conversion
produces if that root were the only root in the profile. The **"cost of
keeping it"** column is how many messages and enums would disappear from
the full 1,163-message set if that one root were dropped — that is, the
types only this root needs. A root whose cost is small rides almost
entirely on types the other roots need anyway.

| Root | Converted alone | Cost of keeping it |
|------|-----------------|--------------------|
| `MA_Action` | 332 msg / 102 enum | +10 msg / +1 enum |
| `MA_ActionStatus` | 124 / 25 | +2 / +0 |
| `MA_ApprovalPolicy` | 119 / 76 | +14 / +2 |
| `MA_ApprovalRequest` | 69 / 14 | +11 / +1 |
| `MA_ApprovalRequestStatus` | 21 / 15 | +3 / +2 |
| `MA_MissionPlan` | 557 / 190 | +7 / +0 |
| `MA_MissionPlanActivationCommand` | 55 / 14 | +3 / +0 |
| `MA_MissionPlanCommand` | 546 / 190 | +2 / +0 |
| `MA_MissionPlanCommandStatus` | 45 / 16 | +9 / +3 |
| `MA_MissionPlanActivationStatus` | 29 / 13 | +3 / +1 |
| `MA_MissionPlanExecutionStatus` | 20 / 14 | +4 / +1 |
| `MA_PlanningFunction` | 102 / 60 | +28 / +3 |
| `MA_PlanningFunctionSettingsCommand` | 299 / 118 | +5 / +1 |
| `MA_PlanningFunctionStatus` | 296 / 116 | +2 / +0 |
| `MA_Response` | 741 / 225 | +46 / +3 |
| `MA_Task` | 720 / 194 | +210 / +33 |
| `MA_TaskStatus` | 124 / 25 | +2 / +0 |
| `MissionContingencyAlert` | 94 / 59 | +11 / +2 |
| `MA_MissionPlanActivationCommandStatus` (added) | 35 / 16 | +3 / +0 |
| `MA_PlanningFunctionSettingsCommandStatus` (added) | 19 / 14 | +3 / +0 |

What the table shows: the generated set is dominated by types *shared*
between roots — widely-referenced UCI building blocks such as `ID_Type`
(referenced by 74 other types), `SystemID_Type` (49), and `ForeignKeyType`
(40); see `closure_report.json` in the generated tree. Because of that
sharing, dropping any single root other than `MA_Task` would shrink the
generated set by fewer than 50 messages (under 4%). Dropping `MA_Task`
would shrink it by 210, but tasking is half the substance of a planning core,
and `MA_TaskStatus` would be left without the message it reports on. So
no root is an accidental passenger: each one either costs little to keep
or earns its cost. The resulting budget — about 2.3 times the size of the
P1 tree, which already compiles cleanly in both C++ and Ada — is accepted,
and the actual build cost is re-measured when the generated code first
compiles.

### 3. Independent validator designation

**Decision: the independent OMS/CAL peer for G1 step 6 is Sleet
v2026.06.01** (the independently-authored AMS-GRA reference CAL server,
pinned container
`registry.gitlab.com/open-arsenal/ams-gra/hello-world-sk/infra/sleet:v2026.06.01`),
**run ad hoc under WSL podman with its schema validation pointed at the
pinned `A-GRA_MessageDefinitions_v5_0_a.xsd`** (Sleet's schema is
path-configurable — the harness already supports `SLEET_SCHEMA_PATH`, and
the container's schema is replaceable by mount). Offline instance
generation/validation additionally uses the independently-authored
`la-cal-harness` XSD-derived generator/validator driven with the A-GRA
XSD (`UCI_XSD_PATH` is drop-agnostic).

The exact OMS schema identifier sent during LA-CAL `INIT` is
**`005.0a.ASK`**. Measured against the pinned Sleet v2026.06.01 on
2026-07-14: Sleet loads the A-GRA MessageDefinitions XSD (841 elements,
4,845 types), normalizes the root `version` attribute
`005.0a.ASK-20260423-f1380e7` to the loaded version `005.0a.ASK`, requires
its `schema_version` config to equal that loaded value, and accepted an
OWP `INIT` carrying `"schema":"005.0a.ASK"` with `+OK`. The LA-CAL plugin
sends this via its `schema` config field, never its `002.5.0` default.

**Evidence boundary this designation supports:** Sleet-with-A-GRA-schema
proves bidirectional OMS/CAL wire conformance against the authoritative
A-GRA 5.0a schema through an independently implemented CAL server. It is
*not* a formal A-GRA platform peer; no A-GRA platform-compliance claim
follows from it, and none is made. UCI 2.5 Sleet evidence remains
inapplicable to this drop.

## P3 conversion-scope decision (2026-07-16)

`p3_agra_core_mms.json` targets the ladder rung
[`uci_mms_conversion_plan.md`](../../../../doc/plans/PYRAMID/uci_mms_conversion_plan.md)
§5 calls **P3 — full Core MMS**, marked there as *"Not scheduled; only under
a real compliance tasking"* (Phase 5). This profile and the CMake option
gating its build (`PYRAMID_ENABLE_AGRA_P3`, OFF by default; see
`subprojects/PYRAMID/CMakeLists.txt`) exist so the proto tree and a generic
JSON codec for it can be produced and built on demand, without that tasking
having happened. **No A-GRA MA L1 compliance claim is made, and none should
be inferred from this profile's existence.**

### Root list — mechanically derived from Table 3-1, not hand-curated

Unlike P1 and P2, whose root lists were read and approved message-by-message
against the a-gra standard review, P3's 343 roots were parsed directly out of
Table 3-1 ("MMS for MA L1 Interfaces") of
`ref/a-gra-main/Documentation/ASK 5.0a MA L1 Compliance Document.pdf` — every
row across all four interfaces (C2, MS, P2P, VI) and all MUC tags (Core and
optional alike), deduplicated by message name. `ref/a-gra-main/` is a local,
git-ignored (`ref/*`) reference checkout of the upstream `open-arsenal/a-gra`
repository; its `Schema/*.xsd` files were hash-verified against the
`agra_5_0a` pin in `pim/schemas/schema_manifest.json` before use (sha256
match, both files) — see `pim/schemas/README.md`'s `AGRA_XSD_DIR` env-source
for the fetch-not-vendor mechanism this satisfies.

The compliance document's table uses each message's complex*type* name
(the `MT` suffix), not the XSD global *element* name `xsd2proto.py` roots
need. Names were resolved via the schema's own
`<xs:element name="X" type="Y">` mapping, not by stripping the suffix
textually — two roots (`DLZ`, `MA_WEZ`) have a type suffix of literal `_MT`
rather than `MT` appended to the element name, which a blind strip gets
wrong. All 343 resolved elements exist in the pinned XSD; the closure
converts strict-clean with zero skipped constructs. All 20 profile P2 roots
are a subset of these 343, as expected (P2 is P3's planning-core slice, both
against the same drop).

The a-gra standard review's own volumetric estimate (§6.2) is "~327" —
close to, but not exactly, this profile's 343. That estimate is explicitly
approximate and was derived by a different method (interface-count
arithmetic on the compliance document's summary numbers, not a row-by-row
parse); this profile's count comes from parsing every table row directly
against the schema, so the two are not expected to match exactly and this
list is authoritative for what actually got converted.

### Depth: contract and codecs — not the fidelity ladder

Per the ladder table, this profile has validation tier **none**: no offline
fidelity ladder (goldens, round-trips, malformed-input negatives) and no
live interop evidence has been run against it, unlike P1/P2's (a)/(a)+(b)+(c)
tiers. `PYRAMID_ENABLE_AGRA_P3` builds a real, compiling C++ data model,
`application/json` codec, and `application/oms-json` codec plugin for the
full closure — proof the proto/codegen pipeline scales to Core MMS size —
but that is build scope, not a compliance or wire-fidelity claim.

### The interaction seam: `pim/agra_p3_seam/`

A data model alone is not a usable contract: the P1/P2 pattern pairs the
converted tree with component service protos carrying the PYRAMID
pubsub/rpc port-grammar annotations (`pyramid_op`: PUBLISH/SUBSCRIBE
topics with QoS). P1's and P2's seams (`pim/uci_p1_seam/`,
`pim/agra_p2_seam/`) are hand-authored; at 343 roots that is not viable,
so P3's seam is **generated** by `pim/gen_interaction_seam.py` from four
checked-in inputs: this profile's manifest, the converted tree,
`p3_agra_core_mms_interfaces.json` — the full Table 3-1
interface/direction data (668 rows: which message travels on which of the
four compliance-document interfaces, C2/MS/P2P/VI, and in which
direction), parsed from the same PDF as the root list and resolved to
element names the same way. The union of the interface table's messages
equals the root list exactly (test-guarded) — and the checked-in P2 seam.

The derivation rules mirror the P1/P2 grammar and are documented in the
generator's docstring; in brief: `X`/`XStatus` pairs where `X` ends in
"Command" or "Request" become correlated Request/Entity services
(the A-GRA Command-2 pattern), everything else becomes a single-variant
Information service; the provided/consumed split and PUBLISH/SUBSCRIBE
polarity follow the table's direction column (the MA system executes what
C2 commands, and itself commands the mission systems and vehicle); topics
are the bare element names (the LA-CAL/Sleet routing key, as in P2); all
operations are RELIABLE/VOLATILE depth 10 (P2's approved floor). Result:
eight P3 component protos (four interfaces × provided/consumed), 722
P3-derived services, and P2's two component protos with their 16 existing
services. P3 retains P2's `pyramid.data_model.agra` package, port grammar,
and component protos byte for byte, so unchanged P2 client code can use the
P3 seam. P2 and P3 are alternative selected contracts and must not be linked
together. The seam regenerates byte-identically — `test_agra_p3_seam.py`
reruns the generator and compares, so hand-edits to the seam fail loudly;
the same file pins the per-component service counts, polarity, QoS, and
copy integrity, and (gated behind `AGRA_P3_BINDINGS_SMOKE=1`, since it
takes over a minute) runs the full binding generation over the seam.

### Generator fixes this profile's scale forced (both landed 2026-07-16)

1. **Reserved-word field names.** `COMINT_ChangeDwellType` has a boolean
   field literally named `Delete`; `xsd2proto.py`'s `snake_case()`
   produced the bare C++ keyword `delete`, which MSVC rejected at every
   use site in the generated code. `snake_case()` now suffixes an
   underscore when the result is a C++ reserved word (`delete` →
   `delete_`); the wire name (the sidecar's `element` entry, still
   `Delete`) is untouched. The escape list is deliberately C++ only —
   adding Ada reserved words would rename `begin`, `end`, `range`,
   `task`, and `type` fields throughout P1/P2's frozen, evidence-carrying
   trees (confirmed as `--check` drift both ways). Neither P1 nor P2
   reaches a C++ reserved word, so both trees are byte-identical under
   the fix (`--check` clean). The seam generator imports the same
   function rather than carrying its own copy — the root element
   `Operator` produces the field `operator_`, and an unescaped local
   variant demonstrably emitted the bare keyword into the OMS codec
   plugin's service-wire structs.

2. **Extension bases that collapse to a scalar.** The OMS-JSON emitter
   refused the tree with a misleading `repeated xs:choice carriers are
   not yet supported` error on `MissionEnvironmentObjectEntityType.base`.
   The actual shape: an XSD `complexContent` extension whose base type
   has a single scalar field (`Value`), which the types emitter collapses
   to a plain `std::string base;` member instead of inlining it away —
   so the field had no wire key of its own. The emitter now (sidecar path
   only) emits such a member under the collapsed field's element name.
   The heuristic/no-sidecar path is byte-frozen against the
   Sleet-verified seam golden and is deliberately unchanged.

Note for whoever runs the pim suite:
`test_oms_json_gen.py::SeamRegressionTest` fails on this checkout **before
and after** these changes (pre-existing golden drift, reproduced on a
clean stash of the working tree); it is not a regression from either fix.

Do not grow or shrink this profile's root list or interface table without
re-parsing Table 3-1, re-running `xsd2proto.py --check`, and regenerating
the seam.
