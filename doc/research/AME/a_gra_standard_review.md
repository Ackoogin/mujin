# A-GRA (Autonomy Government Reference Architecture) Standard Review

**Scope:** Applicability of the A-GRA / ASK 5.0a standard
([github.com/open-arsenal/a-gra](https://github.com/open-arsenal/a-gra)) to the
AME planning/execution stack and the PYRAMID generated-bindings system.
Of particular interest: (1) converting the A-GRA-defined API into our proto
contract system, and (2) how A-GRA's planning/actions interface could work with
AME.

**Date:** 2026-07-02
**Inputs reviewed:** `A-GRA_MessageDefinitions_v5_0_a.xsd` (155,020 lines, 8.6 MB),
`A-GRA_SecurityMarkings_v5_0_a.xsd` (5,472 lines), ASK 5.0a Start Here Guide
(public release, 21 APR 2026). The eight interface-volume PDFs
(ICD, C2, Vehicle, Mission Systems, Mission Plan, Mission Debrief, Peer, L1
Compliance) were not analysed in depth for this review; findings below are
grounded in the schemas and the Start Here Guide.

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
| Wire format | JSON / FlatBuffers / Protobuf via codecs, transport plugins (gRPC, ROS2, PCL sockets) | XML per XSD (transport unspecified at L1; DMS mentioned for MS) |
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
closure):

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
  - **XML wire format.** A-GRA compliance is on XML instances of the XSD. Our
    codec set has no XML backend. For actual L1 compliance testing we would
    need either an XML codec backend for `agra.*` types (proto3 ↔ XML is
    mechanical given the converter knows the original XSD shape) or to treat
    XML serialisation as a bridge-local concern. Until a compliance target
    exists, JSON round-tripping of the converted types is sufficient for
    internal use.

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

### 5.3 Genuine gaps (where AME needs new capability, not just translation)

1. **Approval/consent state machine.** A-GRA makes plan approval,
   execution approval, and consent requests (`*ConsentRequest`) first-class,
   with RBAC (`ApprovalAuthority`, operator roles). AME's plan/execution
   split gives the right pause point, but nothing models *who may approve
   what* or holds plans in `PENDING_APPROVAL`. Smallest change: an approval
   state on the `Plan` resource plus bridge-enforced policy; RBAC stays in
   the bridge/C2 layer.
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

---

## 6. Recommended adoption path

| Phase | Work | Output |
|-------|------|--------|
| 0 | Obtain and read the ICD + C2 + MP volumes properly (this review is schema-grounded); confirm distribution constraints for derived artefacts (package is public but marked "properly implementing interactions" compliance; check CUI status of the PDFs) | Go/no-go, refined profile |
| 1 | `pim/xsd2proto.py` + profile manifest; generate `agra.data_model.*`, `agra.security` protos; C++ bindings + JSON/Protobuf codecs only | Converted, reviewable A-GRA profile in `proto/agra/` |
| 2 | ActionType→goal-fluent mapping table (per supported mission type) + `ExecutionBinding` table for capability commands; smallest demo: `Action(FIND_SEARCH)` → plan → approve → execute → `ActionStatus` complete, against a stub C2 client | `agra_c2_bridge` component + E2E test mirroring the existing tactical `StandardBridge` pattern |
| 3 | Approval state on `Plan`, drain-cancel semantics, `ActionPlanExecutionStatus` fidelity; sequence-level conformance tests modelled on the A-GRA L1 harness | Compliance-shaped test suite |
| 4 (conditional) | XML codec backend for `agra.*` if real harness compliance becomes a target; temporal window support per the existing temporal research plan; P2P/team component | External compliance |

**Effort signal:** Phases 1–2 are comparable to the Tactical Objects standard
alignment effort (converter + one bridge + tests), because the interaction
model was found to be congruent with the existing `autonomy_backend`
contract. Phases 3–4 are where genuinely new capability lives.

---

## 7. Risks and open questions

- **Interface volumes not yet analysed.** The normative sequence definitions
  live in the PDFs (especially the ICD and C2 volume); the schema alone
  over-states freedom. Phase 0 must precede committing to bridge behaviour.
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

## 8. References

- A-GRA repository: https://github.com/open-arsenal/a-gra (Schema/, Documentation/, ASK 5.0a Start Here Guide)
- USAF A-GRA/CCA reporting: [The Aviationist](https://theaviationist.com/2026/02/14/usaf-integrates-a-gra-architecture-mission-autonomy-ccas/), [ExecutiveGov](https://www.executivegov.com/articles/air-force-a-gra-cca-open-architecture)
- In-repo: `subprojects/PYRAMID/proto/pyramid/data_model/pyramid.data_model.autonomy.proto`,
  `subprojects/PYRAMID/proto/pyramid/components/pyramid.components.autonomy_backend.services.provided.proto`,
  `subprojects/AME/include/ame/{autonomy_backend.h,execution_sink.h,pyramid_autonomy_bridge.h,action_registry.h}`,
  `subprojects/PYRAMID/doc/architecture/generated_bindings.md`,
  `doc/plans/PYRAMID/standard_alignment_plan.md`,
  `doc/research/AME/temporal_extension_research.md`
