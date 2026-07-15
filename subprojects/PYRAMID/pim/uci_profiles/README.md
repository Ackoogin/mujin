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
