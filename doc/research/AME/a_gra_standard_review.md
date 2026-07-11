# A-GRA (Autonomy Government Reference Architecture) Standard Review

**Scope:** Applicability of the A-GRA / ASK 5.0a standard
([github.com/open-arsenal/a-gra](https://github.com/open-arsenal/a-gra)) to the
AME planning/execution stack and the PYRAMID generated-bindings system.
Of particular interest: (1) converting the A-GRA-defined API into our proto
contract system, and (2) how A-GRA's planning/actions interface could work with
AME.

**Date:** 2026-07-02 (interface-volume deep dive added same day)
**Companion:** [`a_gra_e2e_worked_example.md`](a_gra_e2e_worked_example.md) —
message-level worked example (planning → routing + EO/PO sensor control) with
sequence diagrams, PYR-RESP conformance matrix, and a service-over-pub/sub
alignment analysis for the PCL/PYRAMID bindings.
[`ams_gra_starter_kit_review.md`](ams_gra_starter_kit_review.md) (2026-07-08)
— review of the sibling **AMS GRA** hello-world starter kit: a public
LA-CAL/OMS/UCI simulation environment relevant to the onboard MS leg and the
"OMS/CAL access may be controlled" risk below, plus MEL data-plane mappings
onto PCL.
[`ams_gra_oms_cal_join.md`](ams_gra_oms_cal_join.md) (2026-07-11) — how the
PYRAMID binding layer joins an OMS CAL: the now-public OMS v2.5 spec set
(CAL / C++ CAL / LA-CAL), what is and isn't standardized, and the
plugin-shaped integration options enabled by the interaction-facade seam.
**Inputs reviewed:** `A-GRA_MessageDefinitions_v5_0_a.xsd` (155,020 lines, 8.6 MB),
`A-GRA_SecurityMarkings_v5_0_a.xsd` (5,472 lines), ASK 5.0a Start Here Guide
(public release, 21 APR 2026), and the eight interface-volume PDFs from the
local repo copy (`doc/research/AME/a-gra-main/`): ICD (621 pp), C2 (385 pp),
MA L1 Compliance (402 pp), Peer (309 pp), Mission Systems (164 pp), Vehicle
Interface (135 pp), Mission Plan (10 pp), Mission Debrief (6 pp). Section 6
records the volume-grounded findings; earlier sections are schema-grounded
with corrections noted inline.

---

## 1. Executive Summary

A-GRA is the **US Air Force's government-owned interface standard for Mission
Autonomy (MA)** on Autonomous Collaborative Platforms (ACPs), currently being
integrated across the Collaborative Combat Aircraft (CCA) program. It is
built on **UCI (Universal Command and Control Interface)** messages plus MA
extensions, published as XSD, and organised around six Level-1 (L1)
interfaces: Command & Control (C2), Vehicle Interface (VI), Mission Systems
(MS), Mission Planning (MP), Mission Debrief (MD), and Peer-to-Peer (P2P).

Key conclusions:

1. **A-GRA and AME solve the same problem at the same architectural seam.**
   A-GRA's MA is exactly the role AME plays: mission-level planning,
   prioritisation, execution management, and replanning, sitting between C2
   above and vehicle/mission systems below. A-GRA's
   Effect → Action → Task → Capability-Command decomposition, its
   plan/approve/execute lifecycle, and its command/status feedback pattern map
   cleanly onto AME's PlanningRequirement → Plan → ExecutionRequirement →
   ExecutionRun → RequirementPlacement contract.

2. **Wholesale XSD→proto conversion is feasible but should not be the goal.**
   The message-definition schema is very large (841 top-level messages, ~4,800
   complex types, ~930 simple types/enums). The XSD dialect is
   conversion-friendly (no substitution groups, modest use of abstract types,
   heavy but regular use of `xs:choice`), so a mechanical `xsd2proto` pass is
   practical. However, we should convert a **profile** — the ~40–80 messages
   AME/PYRAMID actually exchange — and keep the converter + profile manifest
   so the subset can grow.

3. **The highest-value adoption is a thin A-GRA bridge in front of the
   existing `autonomy_backend` contract**, analogous to how `StandardBridge`
   fronts Tactical Objects and `PyramidAutonomyBridge` fronts AME. AME's
   internal surfaces (`IAutonomyBackend`, `IExecutionSink`, `ActionRegistry`)
   need little or no change; the work is translation plus a small number of
   genuine capability gaps (approval/consent flow, temporal plan windows,
   security markings).

4. **A-GRA validates several AME design decisions** — separation of planning
   from execution approval, explicit command cancellation semantics, CRUD-like
   object-state signalling, and audit/status messages for every command — and
   provides a credible external target for AME's "component of a larger MOSA
   ecosystem" story alongside PYRAMID.

5. **Interface-volume verdict (deep dive, §6): no static-plan blocker.**
   A-GRA does *not* assume static mission plans — the Mission Plan volume
   declares mission data "dynamic", and the C2/Peer volumes define dedicated
   sequences for autonomous contingency replanning, operator-commanded
   replans, and replan status reporting. Replanning is a compliance-relevant
   *expectation* on MA, not a conflict with it. The genuine deltas are: (a) an
   approval gate *inside* the replan loop (AME currently swaps BTs without a
   hold point), (b) plan-shape projection (A-GRA MA emits typed UCI plans
   including kinematic RoutePlans, so an A-GRA "MA" is AME *plus* PYRAMID
   route/vehicle components), (c) the wire stack (EXI over DDS/DMS offboard,
   OMS/CAL/ASB onboard — not plain XML), and (d) compliance scale (the Core
   Minimum Message Set is ~327 unique messages, far above the ~40–80 profile
   guessed in §4.1; profile-first adoption remains right, but full L1
   compliance is a much bigger commitment).

---

## 2. What A-GRA Is

### 2.1 Provenance and intent

- **A-GRA** = Autonomy Government Reference Architecture; **ASK** is the
  released package (v5.0a, first public release 21 APR 2026, AFLCMC contact).
- Purpose: standardise MA interfaces so autonomy software can be competed,
  swapped, and ported between platforms (MOSA / anti-vendor-lock). Publicly
  reported as integrated on YFQ-42/YFQ-44 CCAs with mission autonomy from
  multiple vendors.
- Messages are **UCI plus MA extensions**; the XSD target namespace is the
  AFRL UCI namespace (`vdl.afrl.af.mil/programs/oam`), schema version string
  `005.0a.ASK-20260423-f1380e7`.
- Compliance is **interaction-based**: correct message sequences with required
  fields populated, checked by a government-provided automated L1 compliance
  harness (sequence/message-level validation today).

### 2.2 The six L1 interfaces

| Interface | Content (from Start Here Guide) | Nearest concept in this repo |
|-----------|--------------------------------|------------------------------|
| **C2** | Direct/Indirect/Responsive command categories, team forming/tasking, RBAC for ACPs, weapon-employment command flows | AME `autonomy_backend` provided services (PlanningRequirement / ExecutionRequirement ingress) |
| **Vehicle Interface (VI)** | MA ↔ Flight Autonomy split; control modes (HSA, waypoint following, curve following); command validation (accept/reject); FA always retains control | AME `IExecutionSink` egress toward platform components; PYRAMID vehicle-side components |
| **Mission Systems (MS)** | Sensor/comms/weapons interactions, PNT and system status to MA, Decentralized Message Service | Tactical Objects, sensor_data_interpretation components; `StateUpdate` ingress into WorldModel |
| **Mission Planning (MP)** | Mission Data Package (pre-mission load), system of plans/sub-plans, ROE and configuration settings | AME PDDL domain/problem load, `MissionIntent`, contingency domains |
| **Mission Debrief (MD)** | Debrief/replay of MA data, on-board debrief systems | AME observability stack (JSONL audit logs, PlanAuditLog, Foxglove) |
| **P2P** | Team formation, leader election, package COP distribution, peer contingencies | AME `goal_allocator` / `agent_dispatcher` / `GoalDispatch` (partial) |

L2 (intra-MA component interfaces) is explicitly deferred to future A-GRA
releases — significant for us, because PYRAMID's component decomposition is
effectively our L2, and A-GRA does not currently constrain it.

### 2.3 Message model anatomy

Every A-GRA message follows a strict envelope pattern:

- Top-level element `X` of type `XMT` ("message type"), extending the common
  `MessageType` base:
  - `SecurityInformation : SecurityInformationType` — full IC-ISM-style
    marking block (classification, owner/producer, SCI controls,
    dissemination, releasability, declassification, etc. — defined in the
    separate SecurityMarkings XSD).
  - `MessageHeader : HeaderType` — `SystemID`, `Timestamp`, `SchemaVersion`,
    `Mode`, optional `ServiceID`, optional `MissionID`.
- Payload `MessageData : XMDT` ("message data type") carrying the actual
  content.
- Many MT types carry an optional `ObjectState : ObjectStateEnum` — a
  publisher-side NEW/UPDATED/REMOVED signal about the underlying software
  object (explicitly *not* a commanded CRUD operation, but a pub/sub analogue
  of one).

Messages are annotated with a small set of **UCI interaction primitives**
that classify the exchange pattern (counts across the schema):

| Primitive | Count | Meaning | PYRAMID analogue |
|-----------|-------|---------|------------------|
| `Data-1` | 233 | published data object | provided read/stream (out) |
| `Command-2` | 233 | command + separate `*CommandStatus` | requirement create + Achievement/progress |
| `ActionRequest-2` | 160 | request + separate `*RequestStatus` | requirement create + product read |
| `Status-1` | 144 | published status | read-only product (out) |
| `DataRecord-1` | 50 | historical/record data | audit/debrief products |
| `DataRequest-2` | 22 | query for data | `Query` → stream read |

This is the single most important structural fact for us: **the entire
841-message surface reduces to six interaction shapes**, and all six map onto
the EntityActions CRUD + read-only-product pattern our proto system already
generates.

### 2.4 The planning/actions hierarchy

A-GRA's C2 requirement model is layered (each with its own message family:
the object itself, `*Status`, `*Plan`, `*PlanCommand`, `*PlanStatus`,
`*PlanExecutionStatus`, `*ApprovalStatus`, `*CancelCommand`, ...):

```
Effect        (desired outcome, EffectTypeEnum: DEFEAT, DENY, DELAY, ...)
  └─ Action   (C2 requirement, ActionTypeEnum: FIND_SEARCH, ATTACK, TRACK,
     │         PROTECT_ESCORT, COLLECT, MONITOR_OBSERVE, ~85 values)
     └─ Task  (capability-shaped tasking, TaskType)
        └─ Capability Command (per-capability messages: AMTI_Command,
                COMINT_Command, StrikeCommand, CommPointingCommand, ...)
```

Key structures:

- `ActionMDT`: `ActionID`, `ActionType` (enum), `ActionConstraints :
  RequirementConstraintsType`, `ActionGuidance : RequirementGuidanceType`,
  `TargetObject`, `SecondaryObject`, `Metadata : RequirementMetadataType`.
- `TaskMDT`: `TaskID`, `TaskType`, `TaskConstraints`, `TaskGuidance`,
  `Metadata` — same requirement/constraint/guidance pattern one level down.
- `ActionPlanMDT`: `ActionPlanID`, `PlanCommandID`, `Plan : ActionPlanType`,
  `PlanInputs` — a plan is an addressable object tied to the command that
  requested it and the inputs it consumed (provenance built in).
- `MissionPlanMDT`: plan hierarchy (`SubPlans`, `ExecutionSequence`,
  parent/subordinate MissionPlan IDs, `Window : DateTimeRangeType`,
  `Applicability`).
- Explicit lifecycle messages: `ActionPlanApprovalStatus`,
  `ActionPlanValidation(Command)`, `ActionPlanExecutionStatus`,
  `ActionPlanningStatus`, `ActionExecutionApprovalStatus`,
  `ApprovalRequest`/`ApprovalPolicy`/`ApprovalAuthority` (RBAC + consent),
  and `ActionCancelCommand` with carefully specified race-condition semantics
  (cancellation as an explicit first-class message because planning inputs
  "can't easily be canceled" — cancelled objects remain for history).

The C2 concept of operations (from the OV-1): MA **self-generates** requests
(e.g. a strike request when a threat emerges) which a human "Quarterback"
merely approves; the team then self-nominates a shooter, executes, and
returns to station. Human-on-the-loop approval of machine-generated plans is
the core interaction — which is precisely the seam AME's Plan-as-addressable-
resource + separate ExecutionRequirement design was built for.

---

## 3. A-GRA vs PYRAMID: positioning

| Aspect | PYRAMID (this repo) | A-GRA |
|--------|--------------------|-------|
| Owner | UK MOD reference architecture | US DAF (AFLCMC), CCA program |
| Primary artefact | Component responsibilities + our proto service contracts | Message set (UCI XSD) + interaction sequences |
| Interface style | EntityActions CRUD services per component, generated bindings | Pub/sub message exchange with 6 interaction primitives |
| Envelope | `Entity` base (id, source, update_time) | `MessageType` (security markings + header), `ObjectState` |
| Wire format | JSON / FlatBuffers / Protobuf via codecs, transport plugins (gRPC, ROS2, PCL sockets) | Offboard (C2/Peer): UCI serialized as EXI over DDS-based DMS; onboard (VI/MS): OMS/CAL over an Abstract Service Bus (see §6.3) |
| Compliance | Conformance tests in-repo | Government L1 compliance harness (sequence-level) |
| Granularity | Small, curated data model (~60 messages) | 841 messages, ~4,800 types, exhaustive domain coverage |
| Intra-autonomy structure | PYRAMID components = our L2 | L2 deferred to future releases |

They are complementary rather than competing: PYRAMID gives us the component
decomposition and binding/transport machinery; A-GRA gives us an
externally-recognised **L1 vocabulary** for what an MA (i.e. AME) must say to
C2, the vehicle, and its peers. Because A-GRA's L2 is unpublished, adopting
A-GRA at the boundary does not conflict with PYRAMID internally.

---

## 4. Converting the A-GRA API to our proto system

### 4.1 Don't convert everything — define a profile

Converting all 155k lines would produce thousands of proto messages, swamp
`pim/generate_bindings.py` output (build-local C++ *and* Ada trees are globbed
and compiled), and buy nothing: most capability families (COMINT, AMTI, air
sampling, cargo, strike variants...) are irrelevant to current AME/PYRAMID
work. Recommended: a **profile manifest** (YAML/JSON checked in next to the
converter) listing the top-level messages to convert; the converter pulls in
the transitive type closure automatically.

Suggested initial profile (~40 top-level messages → a few hundred types after
closure — **superseded by §6.2**, which re-anchors the profile on the ~123
`MA_*` extension messages now that the compliance document's MMS has been
read; the list below is retained as the original schema-derived guess):

- **Planning/C2 core:** `Action`, `ActionStatus`, `ActionPlan`,
  `ActionPlanStatus`, `ActionPlanCommand`, `ActionPlanCommandStatus`,
  `ActionPlanExecutionStatus`, `ActionPlanApprovalStatus`,
  `ActionPlanningStatus`, `ActionCancelCommand(Status)`, `Task`,
  `TaskStatus`, `MissionPlan`, `MissionPlanStatus`, `MissionPlanCommand(Status)`
- **Approval/consent:** `ApprovalRequest(Status)`, `ApprovalPolicy`,
  `ApprovalAuthority`, `ActionExecutionApprovalStatus`
- **State/status:** `SystemStatus`-family, `CapabilityStatus` for the one or
  two capabilities we exercise, position/PNT report used by MS volume
- **Support types:** `MessageType`, `HeaderType`, ID types,
  `RequirementConstraintsType`, `RequirementGuidanceType`,
  `DateTimeRangeType`, `SecurityInformationType` (flattened, see 4.4)

### 4.2 Mechanical XSD→proto3 mapping rules

The schema is unusually conversion-friendly: **zero substitution groups**,
only 54 abstract types, and a fully regular MT/MDT/ID-type discipline. The
awkward XSD features (attributes-vs-elements, mixed content, facet patterns)
are essentially absent. Proposed rules:

| XSD construct | Occurrences | proto3 mapping |
|---------------|------------:|----------------|
| `xs:complexType` (concrete) | ~4,800 | `message` |
| `xs:sequence` children | — | numbered fields in declaration order |
| `minOccurs="0"` | pervasive | `optional` field |
| `maxOccurs="unbounded"` | 2,033 | `repeated` |
| `xs:choice` | 445 | `oneof` (wrap in nested message when the choice is itself repeated — proto3 forbids `repeated oneof`) |
| `xs:extension base="uci:X"` | 2,130 | composition per existing PYRAMID convention: `X base = 1;` (matches `common.Entity base = 1` style already used in `pyramid.data_model.*`) — **not** flattening, so shared code/codecs can treat the base uniformly |
| abstract complexType | 54 | message + `oneof` dispatcher in referencing sites, or drop if outside profile closure |
| `xs:simpleType` enumeration | ~930 | `enum` with `*_UNSPECIFIED = 0` first (A-GRA enums have no zero-sentinel; we must add one — this is our convention, see `FactAuthorityLevel`) |
| ID types (`ActionID_Type`, ...) | hundreds | collapse to `pyramid.data_model.base.Identifier` + a `string type_name` only where the ID family matters; or generate 1:1 wrapper messages for fidelity (recommended: 1:1 wrappers, they are cheap and preserve the standard's type-safety intent) |
| `DateTimeType` | — | `google.protobuf.Timestamp` |
| `DurationType` | — | `google.protobuf.Duration` |
| `xs:documentation` | everywhere | carried over as `//` comments (the annotations are high-value: they contain normative behaviour, e.g. the ActionCancelCommand race-condition rules) |
| `uci:version` attribute per type | everywhere | comment line (`// uci_version: 003.000.003.000`) plus one file-level constant for the schema version string; needed for the `SchemaVersion` header field |

Namespace/packaging: keep the converted tree in its own package family so its
provenance is unambiguous and it can never silently mix with the curated
PYRAMID data model:

```
proto/agra/agra.data_model.<volume>.proto      // e.g. agra.data_model.c2
proto/agra/agra.security.proto
proto/pyramid/components/agra_c2_bridge.services.{provided,consumed}.proto
```

### 4.3 Envelope handling

The UCI `MessageType` envelope duplicates concerns our stack already owns:

- `MessageHeader.SystemID/ServiceID/Timestamp` ≈ `Entity.source` +
  `Entity.update_time` + transport metadata.
- `ObjectState` (NEW/UPDATED/REMOVED publisher signal) ≈ EntityActions
  Create/Update/Delete — an almost exact semantic match, which is the main
  reason the bridge translation stays thin.

Recommendation: convert `MessageType`/`HeaderType` faithfully (the bridge
must populate them on egress to be compliance-testable), but **do not** let
them leak inward — inside PYRAMID, strip to `Entity` semantics at the bridge,
exactly as `StandardBridge` maps standard types to the tactical runtime's
domain model today.

### 4.4 Security markings

`SecurityInformationType` (IC-ISM-style: classification, owner/producer, SCI
controls, dissemination controls, releasability, declass) has no counterpart
anywhere in the PYRAMID data model. Options:

1. Convert the full marking block into `agra.security` and carry it opaquely
   through the bridge (needed for wire compliance).
2. Additionally add a minimal `SecurityMarking` message to
   `pyramid.data_model.common` so internal products (plans, audit records)
   can retain markings end-to-end.

Recommend (1) now, (2) only when a real tasking requires internal marking
propagation — it is invasive (touches every Entity) and policy-laden.

### 4.5 Tooling and build integration

- Write `pim/xsd2proto.py` (or a `pim/` subtool) consuming the XSD + profile
  manifest, emitting proto files in one deterministic pass; check the
  *generated protos* in (like the rest of `proto/`), not just the tool, so
  diffs are reviewable and the standard drop is versioned (`5.0a` in a
  comment header, mirroring `uci:version` retention).
- The existing pipeline (`generate_bindings.py` → C++/Ada facades, JSON /
  FlatBuffers / Protobuf codecs, gRPC/ROS2 projections) then works unchanged
  on the new packages. Watch two costs:
  - **Codegen volume.** Even a 300-type closure roughly triples the current
    data-model surface. Ada binding generation and the FlatBuffers schema
    projection are the likely pain points; consider generating only C++ +
    JSON/Protobuf codecs for `agra.*` initially (manifest-driven binding
    sources already support per-package selection).
  - **Wire format.** *(Corrected by the ICD, see §6.3.)* Offboard compliance
    is not plain XML: C2/Peer messages are UCI serialized as **EXI**
    (Efficient XML Interchange, W3C) carried by the DDS-based Decentralized
    Messaging Service; onboard VI/MS traffic rides an OMS/CAL Abstract
    Service Bus. Our codec set has none of these backends. For actual L1
    compliance we would need an EXI codec backend for `agra.*` types plus a
    DMS-conformant DDS transport (the ROS2 transport plugin gives us DDS
    experience, but DMS mandates its own IDL wrapper and topic naming — see
    §6.3), or to treat wire serialisation entirely as a bridge-local concern.
    Until a compliance target exists, JSON round-tripping of the converted
    types is sufficient for internal use.

---

## 5. Planning/actions interface: how A-GRA works with AME

### 5.1 Concept mapping

| A-GRA | AME / PYRAMID autonomy contract | Fit |
|-------|--------------------------------|-----|
| `Effect` / `Action` (+ constraints, guidance, target object) | `PlanningRequirement` + `PlanningGoal` (RequirementReference or PDDL expression) | Good — `ActionTypeEnum` value + `TargetObject` translate to goal fluents (e.g. `FIND_SEARCH` + area → `(found ?t)` style goals); constraints map to `PlanningPolicy`/domain selection |
| `Task` | Decomposed goals / `GoalDispatch` to agents | Good |
| `ActionPlan` / `ActivityPlan` (addressable, with `PlanInputs` provenance) | `Plan` (addressable resource; `planning_requirement_id`, `world_version`, `solve_time_ms`, steps, compiled BT XML) | Very good — both make the plan a first-class resource with provenance |
| `ActionPlanApprovalStatus`, `ApprovalRequest`, QB consent flow | Plan/execution split: plan is created, then a separate `ExecutionRequirement` references the approved `plan_id` | Structural fit; AME lacks an explicit approval state machine (gap, §5.3) |
| `ActionPlanCommand` (activate) | `CreateExecutionRequirement(plan_id)` | Direct |
| `ActionPlanExecutionStatus` / `ActionStatus` | `ExecutionRun` (`ExecutionState`, `Achievement`, `replan_count`, outstanding placements) | Direct |
| Capability commands (`AMTI_Command`, `StrikeCommand`, ...) + `*CommandStatus` | `ActionCommand` egress via `IExecutionSink` → typed `RequirementPlacement` on target components; `CommandResult` feedback | Direct — `ExecutionBinding{action_name → target_component/service/type}` is exactly the ActionRegistry-style table that would map PDDL operators to A-GRA capability commands |
| `ActionCancelCommand` (explicit, race-aware, history-preserving) | `DeleteExecutionRequirement` / `requestStop(DRAIN|IMMEDIATE)`; cancelled runs retained | Good; A-GRA's annotations give us precise semantics to adopt for drain behaviour |
| `ActionPlanningStatus` (planning in progress/failed) | `DecisionRecord` stream, `plan_success`, NeuroAuditLog | Good |
| `ObjectState` NEW/UPDATED/REMOVED publications (tracks, system status) | `StateUpdate`/`WorldFactUpdate` into WorldModel; Tactical Objects evidence flow | Good — MS-volume track/status messages are ingress to `State_Service` and/or Tactical Objects, which then ground WorldModel facts |
| Mission Data Package (MP volume) | PDDL domain + problem + ActionRegistry + contingency domains loaded pre-mission | Conceptual match; format work needed |
| MD volume (debrief/replay) | 5/6-layer observability stack, JSONL audit logs | Strong story — our audit trail exceeds sequence-level compliance needs |
| P2P (team forming, leader election, COP) | `goal_allocator`, `agent_dispatcher`, `GoalDispatch` | Partial — AME has multi-agent goal allocation, not team membership/leader election |

### 5.2 Proposed integration shape

Follow the proven two-bridge pattern; AME core does not change:

```
A-GRA C2 node (XML/UCI messages, Action/ActionPlan*/ApprovalRequest, ...)
        │  agra.* converted protos at the boundary
        ▼
  agra_c2_bridge (new PYRAMID component)
        │  translates A-GRA planning messages ↔ autonomy_backend services
        ▼
  Planning_Requirement_Service / Plan_Service / Execution_Requirement_Service
  Execution_Run_Service / Requirement_Placement_Service / State_Service
        │  (existing pyramid.components.autonomy_backend contract)
        ▼
  PyramidAutonomyBridge → IAutonomyBackend (AME WorldModel/Planner/
  PlanCompiler/BT executor, replan-on-failure)
        │
        ▼
  IExecutionSink → ExecutionBinding table → typed RequirementPlacements
  (≙ A-GRA capability commands via VI/MS volumes on the egress side)
```

Representative sequence (DCA vignette from the OV-1):

1. C2 publishes `Action` (`ActionType=PROTECT_CAP`, target area, constraints).
   Bridge → `CreatePlanningRequirement` with goals derived from the
   ActionType→goal-fluent mapping table; publishes `ActionPlanningStatus`.
2. AME solves; `Plan` resource appears. Bridge publishes `ActionPlan` (with
   `PlanInputs` from the requirement) and `ActionPlanStatus`, then
   `ApprovalRequest` toward the QB role.
3. QB approves (`ActionPlanApprovalStatus: APPROVED`, or an
   `ActionPlanCommand` activation). Bridge →
   `CreateExecutionRequirement{plan_id}`.
4. AME executes the compiled BT; each `ActionCommand`/placement surfaces as
   the mapped capability command with `*CommandStatus` feedback; bridge
   publishes `ActionPlanExecutionStatus`/`ActionStatus` from `ExecutionRun`.
5. Threat pop-up → MS-volume track messages → `StateUpdate`; WorldModel fact
   change triggers AME replan; new plan → new approval round-trip (policy
   decides whether replans within guidance re-use standing approval —
   `ApprovalPolicy` covers exactly this).
6. Bingo fuel → contingency goal; `ActionCancelCommand` on the CAP action →
   drain-mode stop of the run, run record retained for MD.

A fully message-identified version of this shape (FIND_SEARCH tasking →
PO/EO sensor control with route control delegated down the sensing chain —
Sensing places the pose/coverage requirement on Routes, which flies it via
the VI route lifecycle — → track report, plus tiered contingency variants
including an approval-gated replan) is worked through in
[`a_gra_e2e_worked_example.md`](a_gra_e2e_worked_example.md).

### 5.3 Genuine gaps (where AME needs new capability, not just translation)

1. **Approval/consent state machine — including inside the replan loop.**
   A-GRA makes plan approval, execution approval, and consent requests
   (`*ConsentRequest`) first-class, with RBAC (`ApprovalAuthority`, operator
   roles). AME's plan/execution split gives the right pause point for the
   *initial* plan, but the volumes go further: `MA_ApprovalPolicyMT` with
   `ByTriggerPolicy.TriggerSource = AUTONOMOUS_TRIGGER` and
   `PlanActivation.Policy.ApprovalRequirement = APPROVAL_REQUIRED` means an
   *autonomous contingency replan* must be published and held for operator
   approval before execution (§6.1). AME's executor replans internally and
   swaps the BT immediately (`IAutonomyBackend`: `DecisionRecord` is emitted
   per replan but nothing can block on it; `PolicyEnvelope` has only
   `max_replans`). Smallest change: a `require_plan_approval` policy knob in
   `PolicyEnvelope` plus a `PENDING_APPROVAL` backend/session state with an
   approve/reject ingress, so the bridge can hold each replan against the
   active `ApprovalPolicy` (`AUTO_APPROVE` degenerates to today's
   behaviour). RBAC itself stays in the bridge/C2 layer.
2. **Temporal semantics.** `MissionPlan.Window`, `ExecutionSequence`,
   `DateTimeRangeType` constraints, and access-assessment messages assume
   plans positioned in time. AME is classical STRIPS today; this is the
   same driver already documented in
   `doc/research/AME/temporal_extension_research.md` — A-GRA strengthens the
   case for the STN/temporal track.
3. **Plan hierarchy.** `MissionPlan` ⊃ sub-plans ⊃ `ActionPlan`/`ActivityPlan`
   with parent/subordinate links. AME produces flat plans per requirement.
   The bridge can synthesise a single-level hierarchy initially; real
   hierarchical planning (mission → action decomposition) is future work and
   aligns with the multi-agent extension research.
4. **Team/P2P behaviours.** Leader election, team membership, COP
   distribution are outside AME scope; they belong to a peer coordination
   component (bridge-level or new component), with AME consuming the results
   as agents/goals.
5. **Security markings end-to-end** (§4.4).
6. **Vehicle Interface control modes.** HSA / waypoint / curve-following
   command sequences with FA accept/reject are below AME's current
   execution granularity — they are the *implementation* of egress placements
   for movement actions, i.e. an `ExecutionBinding` target component, not an
   AME change.
7. **Kinematic plan production (MA boundary is wider than AME).** A-GRA MA
   is expected to *generate* typed UCI plans — `MA_RoutePlanMT` (e.g. CAP
   routes computed by a package leader), `MA_TaskPlanMT`, plus `MissionPlan`
   hierarchies with `ExecutionSequence` linking kinematic and action
   ExecutionPlanSets, and to track/report per-set execution status (§6.1,
   §6.2). AME produces symbolic action sequences, not routes. In PYRAMID
   terms this is fine — an A-GRA "MA" maps to a *composition* of PYRAMID
   components (AME ≈ Objectives/Tasks, plus Routes, Vehicle Guidance,
   Trajectory Prediction feeding the bridge), and A-GRA's unpublished L2
   leaves that decomposition free — but the bridge alone cannot synthesise
   RoutePlans out of nothing; a route-planning component is a prerequisite
   for the C2 sequences that exchange them.
8. **Constraint/priority machinery and MECL.** The C2 volume expects MA to
   honour operator-adjustable Constraint Priority Lists (default priorities
   for geozone types), Execution Priority Lists, `MA_IgnoreConstraintCommand`
   (deliberate constraint override), and Acceptable-Level-of-Risk commands;
   the Mission Plan volume requires evaluating the `MA_CapabilitySetMT`
   MECL (minimum essential capabilities) against `CapabilityStatus` streams
   to raise Mission Capability Fault contingencies (§6.5, §6.6). AME has no
   soft-constraint or capability-sweep machinery; the MECL sweep is a small
   bridge-side monitor, but priority/risk-conditioned planning feeds the
   same planner-capability track as temporal extension (plan-quality
   metrics, `predicted_quality`).

### 5.4 What A-GRA offers AME (adoptable ideas independent of full adoption)

- **Cancellation semantics.** The `ActionCancelCommand` documentation is a
  precise treatment of cancel-vs-inevitable-execution races and
  history-preserving removal; worth folding into `StopMode::DRAIN` semantics
  and the executor's cancellation tests.
- **`ObjectState` publisher signal.** A clean pattern for pub/sub CRUD that
  our ROS2 topic projections could adopt for entity streams.
- **Requirement constraints/guidance split.** `RequirementConstraintsType`
  (hard) vs `RequirementGuidanceType` (soft preference) is a useful
  refinement for `PlanningRequirement` when AME grows plan-quality metrics
  (`predicted_quality` already exists on `Plan`).
- **Interaction-primitive taxonomy.** Annotating our services with the
  Data-1/Command-2/... classification would make conformance sequences
  explicit and machine-checkable, mirroring the A-GRA compliance-harness
  approach in our own test suite.
- **Service-over-pub/sub mechanism.** A-GRA reconstructs the entire
  request/response surface as correlated topic pairs (command + `*Status`
  correlated by ID, topic name == message name, one generic payload wrapper,
  `ObjectState` for CRUD) with no RPC anywhere — a proven pattern for
  projecting EntityActions onto pure pub/sub transports. Analysed in detail,
  with a concrete phased plan for pub/sub generation from the new-shape
  proto contracts, in
  [`doc/plans/PYRAMID/pubsub_contract_generation_plan.md`](../../plans/PYRAMID/README.md).

---

## 6. Interface-volume deep dive

Findings from reading the eight ASK 5.0a volumes (local copy:
`doc/research/AME/a-gra-main/`). This section resolves the "interface volumes
not yet analysed" caveat from the original review and grounds the blocker
assessment.

### 6.1 Replanning: A-GRA expects it — the constraint is *approval*, not stasis

The headline concern — that A-GRA might assume static, pre-loaded mission
plans and conflict with AME's replan-on-failure core — is **refuted by the
volumes**:

- The Mission Plan volume states mission data "is dynamic, allowing C2, Peer,
  Mission Systems, and contingency driven updates during mission execution",
  and that the Mission Data Package "should be filled out thoroughly
  pre-mission, but it can also be updated during mission execution".
- C2 is defined as three paradigms: **Direct** (live commands that supersede
  pre-planned actions), **Planned/Indirect** (activate/deactivate/update/send
  mission plans mid-mission), and **Responsive** (trigger→response automation).
- The C2 and Peer volumes define dedicated sequences: *Define Autonomous
  Contingency Re-Plan*, *Triggered Autonomous Contingency Re-Plan*, *Command
  Mission Replan*, and *MA Provides Replan Status to C2*. Autonomous
  replanning is a configurable Core behaviour, not an exception.

The machinery (C2 volume §1.2.2, §1.2.5):

- **Contingency reactions** are configured per trigger as one of: alert only,
  activate a pre-loaded plan, or **autonomous replanning**, via
  `MA_PlanningFunctionSettingsCommandMT` (e.g.
  `MissionPlanningAutonomy.Trigger.CommsLost.Threshold` for lost-comms;
  `AutonomousPlanningResponseChoice.AutonomousPlanningAction` selects the
  planning process by `PlanningProcessID`). Trigger→response pairs
  (`MA_ResponseMT`, `ResponseType = PLAN_ACTIVATION`) cover the
  pre-loaded-plan case.
- **Approval policy gates activation, per trigger source.**
  `MA_ApprovalPolicyMT` distinguishes `AUTONOMOUS_TRIGGER` from
  operator-commanded replanning and sets
  `PlanActivation.Policy.ApprovalRequirement` to `AUTO_APPROVE` (execute
  immediately) or `APPROVAL_REQUIRED` (publish the generated plan, send
  `MA_ApprovalRequestMT`, wait for `MA_ApprovalRequestStatusMT` with
  `APPROVED` + `OperatorID`; MA re-publishes the plan until approved).
- **Triggered flow**: `MissionContingencyAlertMT` (status `IN_PROGRESS`) →
  generated plan published (any `*PlanMT`) → approval round-trip if required →
  `*PlanExecutionStatusMT` during execution → closing
  `MissionContingencyAlertMT` (`ConflictState = CLEARED`, action status
  `SUCCESSFUL`). This is exactly AME's replan loop with an audit/approval
  envelope around it.
- **Operator-commanded replan**: `MA_MissionPlanCommandMT` → MA generates and
  publishes `MA_MissionPlanMT` *for approval*, linking it back via
  `ResultingPlanIdentifier` in the command status — the same
  plan-as-addressable-resource-with-provenance shape as AME's `Plan`.
- **`ForPlanningUseOnly`** on `*Plan` messages marks intermediate/candidate
  plans that "should never trigger functionality, be subject to approvals, be
  activated" — a ready-made slot for AME's candidate/what-if plans and
  contingency-verifier outputs.

The genuine constraint that survives: an **activated** `MissionPlan` is the
standing behavioural intent — "MA is required to attempt to execute its parts
to completion in order", tracking progress (per-`ExecutionPlanSet` status when
an `ExecutionSequence` is present) "which is also used when responding to
ad-hoc Direct C2 or contingencies, and **resuming the mission plan**". This is
a plan-following-with-interrupts model layered above AME's
replan-from-current-state model. It is reconcilable — AME's persistent goals
play the role of the standing plan, and replans are how parts get achieved —
but the bridge must maintain plan-position bookkeeping (which
ExecutionPlanSets map to which PlanningRequirement goals, and Achievement →
`ExecutionPlanSetStatus`), and MA must *resume* superseded mission-plan parts
after a Direct C2 interrupt rather than treat the interrupt as the new
mission. Also note *MA Response to Conflicting C2 Updates*: simultaneous
conflicting updates from multiple C2 nodes are resolved by stored custody
precedence, with rejected data marked `PlanningState = SUPERSEDED` — a
bridge-level dedup/arbitration duty.

### 6.2 Minimum Message Set volumetrics (compliance scale, measured)

The MA L1 Compliance Document's MMS table (Table 3-1) is the definitive
compliance surface. Parsed counts of **unique messages** per interface:

| Interface | UCI 2.3 | `MA_*` extensions | Total unique |
|-----------|--------:|------------------:|-------------:|
| C2        | 161     | 100               | **261**      |
| Peer (P2P)| 149     | 88                | **237**      |
| MS        | 85      | 23                | **108**      |
| VI        | 6       | 13                | **19**       |
| **Distinct across all** | **204** | **123** | **~327** |

(C2 and P2P overlap heavily; MP compliance is defined as minimum Mission
Data *elements* per data load rather than exchange messages; MD has no MMS —
its single requirement is externally transferring all debrief-relevant ASB
data.)

Implications:

- **The ~40–80-message profile estimate in §4.1 was optimistic for
  compliance.** Nearly everything in the MMS — including every capability
  family (COMINT, AMTI, ESM, SAR, SMTI, Strike, Refuel, CargoDelivery,
  WeatherRadar, IFF, EA, PO, AirSample...) with its
  Capability/CapabilityStatus/Command/CommandStatus/SettingsCommand quintet —
  is tagged **Core MUC**, and the compliance document allows "no tiers of
  compliance or partial compliance". Optional MUCs (Carrier, ESM, ALR, AMTI,
  PO, Comms, PNT, Leader) cover only a small remainder.
- The profile-first strategy stands for *internal adoption and demos*
  (interaction shapes are regular, so the converter cost is linear), but a
  claim of L1 C2 compliance means supporting ~261 message types on that one
  interface — most as thin accept/reject or status stubs, which is
  boilerplate the generator pipeline could mass-produce, but a commitment to
  be costed honestly.
- The `MA_*` extension set is fully documented (compliance doc §4 defines all
  ~123 with field tables) and is where the planning/autonomy substance lives:
  `MA_MissionPlan*`, `MA_PlanningFunction*`, `MA_ApprovalPolicy/Request*`,
  `MA_Action*`/`MA_Task*`, `MA_FlightCommand*`, `MA_ConstraintPriorityList*`,
  `MA_RulesOfEngagement*`, `MA_PackageManagement*`. **The initial conversion
  profile should be re-anchored on the `MA_*` extension set (~123 messages)
  plus the UCI plan/approval/status types they reference** — this supersedes
  the §4.1 message list, which guessed at UCI names before the volumes were
  read.

### 6.3 Transport and wire format (corrects §4.5)

The ICD (§3.3, §3.5) and compliance requirements MA-L1-008..014 pin the stack
far more concretely than "XML per XSD":

- **Onboard (VI, MS):** the Open Mission Systems (OMS) standard with the
  Critical Abstraction Layer (CAL) API over an Abstract Service Bus (ASB).
  MA-L1-008: MA shall communicate with Mission Systems *only* via the
  platform-hosted ASB. Specific CAL/serialization/ASB implementations are
  integration-time choices (Mission Systems Integrator responsibility; VI
  reached through an "OMS Isolator" at the airworthiness boundary).
- **Offboard (C2, Peer):** UCI messages encoded as **EXI 1.0** (W3C Efficient
  XML Interchange), carried by the **Decentralized Messaging Service (DMS)**:
  DDS middleware, RTPS with SPDP/SEDP discovery (MA-L1-011), a mandated
  `MA_DataPayloadWrapper` IDL for all OTA topics (MA-L1-012), topic names
  exactly matching the wrapped message name (MA-L1-013), CDR representation,
  TLS/mTLS/DTLS, mesh operation. MA wraps outbound traffic in
  `MA_TxDataPayloadCommandMT` and receives `MA_RxDataPayloadMT` (both in the
  MS MMS — the comms path itself is part of the MS interface).
- The Autonomy Schema is a UCI 2.3-based custom schema, to be formally
  submitted as a UCI extension schema (OACWG) in a future release — expect
  churn in the `MA_*` set.

For us: PYRAMID's ROS2 transport plugin is DDS-underneath, so DDS competence
exists in-repo, but DMS conformance is a distinct backend (fixed IDL wrapper,
topic discipline, EXI payloads), and OMS/CAL/ASB is an entirely separate
onboard framework. Both stay **bridge-local**: nothing inward of
`agra_c2_bridge` needs to know. The practical consequence for the tooling
plan: the "XML codec" in Phase 4 becomes "EXI codec + DMS-conformant DDS
transport", and any onboard (VI/MS) compliance claim additionally means an
OMS/ASB adapter — significantly more than a serializer.

### 6.4 Vehicle Interface: airworthiness boundary and the failsafe plan

- MA is explicitly **non-safety-critical**; Flight Autonomy (FA) and the VMS
  sit inside a safety-critical enclave behind the "Airworthiness Boundary",
  reached via the VI OMS Isolator (message validation lives there and/or in
  FA). FA always retains authority and accepts/rejects MA commands. This
  matches AME's positioning (mission-level executor above platform guards)
  and keeps certification burden off AME.
- **MA Failsafe** (VI volume §1.2.1.3): MA must register a *failsafe plan*
  with FA — an `MA_ResponseMT` (`PLAN_ACTIVATION`) whose RoutePlans terminate
  in a **pre-planned recovery route stored on FA**, optionally preceded by
  MA-generated transit routes, updated by version as the mission evolves.
  This is a direct external mirror of AME's contingency-domain /
  safe-state-reachability work: the contingency verifier's "safe state always
  reachable" invariant becomes "a valid failsafe plan is registered at all
  times", a concrete compliance hook for
  `doc/guides/contingency_verifier.md` outputs.
- **Route plan lifecycle**: routes are uploaded, checksum-validated,
  validated as plans, brought to `READY_FOR_ACTIVATION`, then activated via
  `MA_MissionPlanActivationCommandMT` (`BySubPlan`, RoutePlanActivationType) —
  a UCI plan-activation state machine the egress side must drive. Control
  modes (curve-following with Bézier segments, HSA/CSA, waypoint-following)
  and Control Mode Authorization confirm §5.3 item 6: below AME granularity,
  owned by the `ExecutionBinding` target component.
- VI MMS is small (19 unique messages) — the thinnest compliant surface, and
  a sensible first E2E target after the C2 planning core.

### 6.5 Planner capability discovery: `MA_PlanningFunction*`

*Receive Autonomy Settings* (C2 volume §1.2.5.15) defines a discovery
interface that maps almost 1:1 onto AME's introspection surfaces:

| A-GRA `PlanningInterface` value | AME/PYRAMID counterpart |
|---------------------------------|-------------------------|
| `PLAN_COMMAND` (create plans) | `Planner::solve()` via `CreatePlanningRequirement` |
| `PLAN_VALIDATION` | **contingency verifier** (exhaustive safe-state check) |
| `AUTOMATED_PLANNING` (auto plan activation) | replan-on-failure loop; `AUTO_APPROVE` path |
| `REQUIREMENT_MANAGEMENT` | PlanningRequirement/ExecutionRequirement CRUD |
| `PLAN_SCORES` / `PLAN_METRICS` | `Plan.predicted_quality`, `solve_time_ms`, plan metrics |
| `PLAN_ASSESSMENT` | DecisionRecord / NeuroAuditLog assessment stream |

Each planning service publishes `MA_PlanningFunctionMT` with its supported
`PlanType` and a `PlanningProcessID` that C2 can command directly and
reconfigure via `MA_PlanningFunctionSettingsCommandMT`;
`MA_PlanningFunctionStatusMT` aggregates up to 10 PlanTypes platform-wide.
`describeCapabilities()` on `IAutonomyBackend` is the natural source for
these advertisements, modestly enriched. This is the cleanest early win in
the standard: AME can advertise honestly today (PLAN_COMMAND,
REQUIREMENT_MANAGEMENT, PLAN_VALIDATION via the verifier) and grow.

### 6.6 Mission Plan volume: data package, MECL, dynamic updates

- The Mission Data Package (pre-mission load, e.g. JMPS-produced) initialises
  Mission Data; plans are **inactive** until explicitly activated, and
  inactive plans may still be used "for computational purposes such as state
  estimation and planning" — matching AME's contingency domains held in
  reserve. FA has its own data bundle; anything MA needs from it must be
  duplicated into the Mission Data Bundle.
- **MECL**: `MA_CapabilitySetMT` lists minimum essential capabilities per
  mission/mission plan; MA must sweep it against `CapabilityStatus` messages
  and raise a Mission Capability Fault contingency on violation (then handled
  by the §6.1 contingency machinery). New but small: a bridge-side monitor
  emitting a contingency trigger into the same path as any other trigger.
- The MP interface itself is **passive** ("isn't an active interface for
  message exchanges") — it defines expected data/format; C2 and P2P reuse its
  element-level query/send/update interactions mid-mission. So "Mission Data
  Package ingestion" for AME is: translate the planning-relevant elements to
  PDDL problem/domain + ActionRegistry + contingency configuration at load,
  and accept element updates through the same ingress as C2 state updates.

### 6.7 Peer volume: prescribed team algorithms

Beyond generic COP sharing, the Peer volume *prescribes algorithms*: **Bully
Leader Election** and **Maximum Consensus Leader Election** (plus off-nominal
variants, package formation/join/leave, leader-assigned succession). Team
Lead MA can configure member MAs' autonomous contingency replanning with the
same `PlanningFunctionSettings`/`ApprovalPolicy` machinery as C2 (§6.1) —
i.e. the team lead is a C2-like authority. This confirms §5.3 item 4:
team/leader-election is a distinct peer-coordination component (with concrete,
testable algorithm requirements), with AME consuming the results as
agents/goals via `goal_allocator`/`agent_dispatcher`.

### 6.8 Cross-cutting duties the bridge must own (volume-sourced)

- **Authorization on every command**: each command sequence requires MA to
  verify the sender (RBAC/custody) and reject with
  `CommandProcessingStateReason = INELIGIBLE_CONTROL_SOURCE` if unauthorized.
  The C2 RBAC package (roles, permissions, admin flows, `RBAC Construct`)
  stays entirely in the bridge.
- **Heartbeats**: periodic heartbeat to C2 and peers; lost-comms triggers key
  off a configurable heartbeat-absence threshold — pairs with the lost-comms
  contingency configuration (§6.1).
- **Status fan-out**: periodic and on-change execution status
  (`MA_MissionPlanExecutionStatusMT` with `Source = ACTUAL`,
  `*PlanExecutionStatusMT`, `TaskStatusMT`, CAP execution reports with
  leader-side aggregation) — all derivable from `ExecutionRun`/placements,
  but the publication cadence/aggregation logic is bridge work.

---

## 7. Recommended adoption path

| Phase | Work | Output |
|-------|------|--------|
| 0 | ~~Obtain and read the ICD + C2 + MP volumes properly~~ **Done — see §6.** Residual: confirm distribution constraints for derived artefacts (package is public; check CUI status of the PDFs before publishing converted protos) | Go/no-go, refined profile — **go; no blockers found** |
| 1 | `pim/xsd2proto.py` + profile manifest **anchored on the `MA_*` extension set (~123 messages) + referenced UCI plan/approval/status types** (§6.2); generate `agra.data_model.*`, `agra.security` protos; C++ bindings + JSON/Protobuf codecs only | Converted, reviewable A-GRA profile in `proto/agra/` |
| 2 | ActionType→goal-fluent mapping table (per supported mission type) + `ExecutionBinding` table for capability commands; `MA_PlanningFunctionMT` advertisement from `describeCapabilities()` (§6.5); smallest demo: `MA_MissionPlanCommandMT` → plan → publish `MA_MissionPlanMT` → approve → execute → status complete, against a stub C2 client | `agra_c2_bridge` component + E2E test mirroring the existing tactical `StandardBridge` pattern |
| 3 | `PENDING_APPROVAL` state + `require_plan_approval` policy in `IAutonomyBackend` (§5.3 item 1); drain-cancel semantics; execution-status fidelity (`Source=ACTUAL`, per-ExecutionPlanSet status, resume-after-interrupt bookkeeping §6.1); MECL monitor (§6.6); sequence-level conformance tests modelled on the A-GRA L1 harness | Compliance-shaped test suite |
| 4 (conditional) | EXI codec backend + DMS-conformant DDS transport (§6.3) if real harness compliance becomes a target (onboard claims additionally need an OMS/ASB adapter); MMS long-tail stubs for the capability families (§6.2); temporal window support per the existing temporal research plan; P2P/team component with Bully / Maximum-Consensus leader election (§6.7) | External compliance |

**Effort signal:** Phases 1–2 are comparable to the Tactical Objects standard
alignment effort (converter + one bridge + tests), because the interaction
model was found to be congruent with the existing `autonomy_backend`
contract. Phases 3–4 are where genuinely new capability lives.

---

## 8. Risks and open questions

- **Compliance scale.** The Core MMS is ~327 unique messages with no partial
  compliance allowed (§6.2). Profile-first adoption is unaffected, but any
  roadmap that says "L1 compliant" must budget for the capability-family long
  tail and the transport stack (EXI + DMS/DDS offboard, OMS/CAL/ASB onboard),
  not just the planning core.
- **Transport dependencies.** DMS conformance details live in a DMS
  specification (DDS settings, topic mappings, IDL, QoS) referenced but not
  included in the public ASK drop; OMS/CAL access may be controlled. Actual
  compliance work has external-artefact dependencies to confirm early.
  *Update 2026-07-08:* the OMS/CAL leg is partially mitigated for
  prototyping — the AMS GRA hello-world starter kit ships a public LA-CAL
  (WebSocket+JSON) implementation and UCI simulation environment; see
  [`ams_gra_starter_kit_review.md`](ams_gra_starter_kit_review.md) §5.
  EXI/DMS remains unmitigated.
  *Update 2026-07-11:* the OMS/CAL access risk is now **fully retired at
  the specification level** — the complete OMS Standard v2.5 document set,
  including the CAL Specification (OMSC-SPC-001), the C++ CAL API spec
  (OMSC-SPC-008), and the Language-Agnostic CAL wire spec (OMSC-SPC-013),
  is public at github.com/open-arsenal/oms. What remains non-public is any
  C++ CAL *implementation* (platform deliverables). Join analysis:
  [`ams_gra_oms_cal_join.md`](ams_gra_oms_cal_join.md).
- **Standard churn.** 5.0a is the first public release (Apr 2026), two
  commits in the repo; field numbering/versioning discipline for regenerated
  protos must be designed in from the start (deterministic field numbers from
  XSD declaration order; converter re-runs must be diff-stable).
- **Scale creep.** The type closure of even a modest profile can balloon via
  `RequirementConstraintsType`-style hub types; the converter should report
  closure size and the profile should be pruned deliberately.
- **Dual-standard drift.** A-GRA vocabulary at the boundary + PYRAMID
  vocabulary internally means two names for most concepts; the concept map in
  §5.1 should be maintained as the bridge's design contract to prevent
  semantic drift.
- **Distribution/markings.** The schemas ship with full IC security-marking
  machinery; our repo and outputs are unmarked. Any real C2 integration needs
  a policy decision on marking propagation (deliberately deferred in §4.4).

---

## 9. References

- Worked E2E example (companion doc): [`a_gra_e2e_worked_example.md`](a_gra_e2e_worked_example.md)
- AMS GRA starter kit review (companion doc): [`ams_gra_starter_kit_review.md`](ams_gra_starter_kit_review.md)
- OMS CAL join analysis (companion doc): [`ams_gra_oms_cal_join.md`](ams_gra_oms_cal_join.md)
- A-GRA repository: https://github.com/open-arsenal/a-gra (Schema/, Documentation/, ASK 5.0a Start Here Guide); local copy at `doc/research/AME/a-gra-main/`
- USAF A-GRA/CCA reporting: [The Aviationist](https://theaviationist.com/2026/02/14/usaf-integrates-a-gra-architecture-mission-autonomy-ccas/), [ExecutiveGov](https://www.executivegov.com/articles/air-force-a-gra-cca-open-architecture)
- In-repo: `subprojects/PYRAMID/proto/pyramid/data_model/pyramid.data_model.autonomy.proto`,
  `subprojects/PYRAMID/proto/pyramid/components/pyramid.components.autonomy_backend.services.provided.proto`,
  `subprojects/AME/include/ame/{autonomy_backend.h,execution_sink.h,pyramid_autonomy_bridge.h,action_registry.h}`,
  `subprojects/PYRAMID/doc/architecture/generated_bindings.md`,
  `subprojects/PYRAMID/doc/architecture/tactical_objects/standard_alignment.md`,
  `doc/research/AME/temporal_extension_research.md`
