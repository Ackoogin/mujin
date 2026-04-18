# Autonomy Backend CRUD Alignment

This document reviews the current autonomy backend alignment with PYRAMID
EntityActions CRUD semantics and the canonical Tactical Objects requirement
pattern.

The important distinction is:

- CRUD shape alone is not enough.
- Behaviour should be invoked by placing, updating, or cancelling typed
  Requirements on component services.
- Component outputs should be read as entities, products, matches, details,
  capabilities, progress, and achievement state.

The autonomy backend can still use AME's current session and behaviour-tree
runtime internally. The external PYRAMID-facing contract should treat the
backend as a requirement placer/coordinator, not as an ad hoc command bus.

## Reference Contracts

| File | Role |
|------|------|
| `pyramid/data_model/common.proto` | Defines `Entity`, `Requirement`, `Achievement`, `Capability`, and `Query` |
| `pyramid/data_model/tactical.proto` | Canonical Tactical Objects entities and requirements |
| `pyramid/components/tactical_objects/services/provided.proto` | Canonical provided requirement CRUD and read-only products |
| `pyramid/components/tactical_objects/services/consumed.proto` | Canonical consumed evidence requirement and capability reads |
| `pyramid/data_model/autonomy.proto` | Current autonomy backend data model projection |
| `pyramid/components/autonomy_backend/services/provided.proto` | Current autonomy backend CRUD-shaped service projection |
| `PYRAMID_COMPONENT_RESPONSIBILITIES.md` | Responsibility guidance for requirement capture, solution determination, progress, and quality |

## Current Verdict

The autonomy backend proto is mechanically close to CRUD alignment, but the
runtime semantics are only partially aligned with the PYRAMID component model.

| Area | Current state | Alignment verdict |
|------|---------------|-------------------|
| Proto enum defaults | `autonomy.proto` has explicit `*_UNSPECIFIED = 0` values | Aligned at proto layer |
| Proto addressability | Autonomy entity messages include `pyramid.data_model.common.Entity base` | Aligned at proto layer |
| Request fields | `Command.request_fields` is `repeated StringKeyValue` | Aligned at proto layer |
| C++/Python AME interface | Uses session structs, enum values without explicit unspecified sentinel, no `Entity`/`Requirement` base | Not aligned |
| Egress reads | `pullCommands()`, `pullGoalDispatches()`, and `pullDecisionRecords()` drain transient queues | Not aligned with idempotent EntityActions reads |
| Behaviour invocation | `ActionCommand` exposes `service_name`, `operation`, and request fields | CRUD-shaped, but not canonical PYRAMID requirement placement |
| Runtime execution | BT `InvokeService` calls are intercepted and emitted as `ActionCommand` objects | Useful implementation shim, but should project externally as requirement CRUD |
| Session lifecycle | `start()`, `step()`, `requestStop()` are mapped to `Create/Update/DeleteSession` in proto | Acceptable shell-control mapping, but not a domain behaviour pattern |

## Canonical Tactical Objects Pattern

Tactical Objects is the clearest current example of the standard shape.

Provided services:

| Service | Operation | Meaning |
|---------|-----------|---------|
| `Object_Of_Interest_Service.CreateRequirement` | create `ObjectInterestRequirement` | Place a requirement for tactical object interest |
| `Object_Of_Interest_Service.ReadRequirement` | read requirements | Inspect active or historical requirement state |
| `Object_Of_Interest_Service.UpdateRequirement` | update requirement | Refine or progress the requirement |
| `Object_Of_Interest_Service.DeleteRequirement` | delete by id | Cancel/remove the requirement |
| `Matching_Objects_Service.ReadMatch` | read `ObjectMatch` | Read output references matching the requirement |
| `Specific_Object_Detail_Service.ReadDetail` | read `ObjectDetail` | Read detailed output entities |

Consumed services:

| Service | Operation | Meaning |
|---------|-----------|---------|
| `Object_Solution_Evidence_Service.CreateRequirement` | create `ObjectEvidenceRequirement` | Place a derived requirement for evidence |
| `Object_Source_Capability_Service.ReadCapability` | read `Capability` | Query whether sources can satisfy the need |
| `Object_Evidence_Service.ReadDetail` | read `ObjectDetail` | Consume evidence products |

The data model reinforces the same pattern:

- `ObjectInterestRequirement` and `ObjectEvidenceRequirement` inherit
  `pyramid.data_model.common.Requirement`.
- `Requirement` carries an `Entity` base and `Achievement` status.
- `ObjectMatch` and `ObjectDetail` inherit `Entity`.
- Products are read; requirements are created, updated, or deleted.

## Responsibility Guidance

`PYRAMID_COMPONENT_RESPONSIBILITIES.md` consistently frames component behaviour
around requirements, solutions, progress, and quality.

For Tactical Objects:

| Responsibility | Alignment implication |
|----------------|----------------------|
| `COMP-060-R01` capture object interest requirements | External invocation starts by placing `ObjectInterestRequirement` |
| `COMP-060-R05` determine requirement solution | The component decides how to satisfy the requirement |
| `COMP-060-R08` determine additional information | The component may derive further requirements, such as evidence needs |
| `COMP-060-R11` identify progress | Progress belongs on the requirement/achievement path |
| `COMP-060-R12` determine quality | Quality is measured against the requirement and criteria |

For Tasks:

| Responsibility | Alignment implication |
|----------------|----------------------|
| `COMP-062-R01` capture task requirements | Top-level mission/tasking should enter as a requirement, not a raw command |
| `COMP-062-R05` determine implementation solution | AME planning maps naturally to solution determination |
| `COMP-062-R07` coordinate solution enactment | AME behaviour-tree execution coordinates requirement placement on components |
| `COMP-062-R08` identify progress of solution | Progress should be observable through requirement/achievement state and output entities |

This means the autonomy backend should be treated as a requirement-driven
coordinator. Its behaviour-tree leaves should place or action component
requirements, then read resulting entities or achievement state.

## Required Alignment Direction

### 1. Keep session control as backend shell control

`Session_Service` can remain the lifecycle surface for starting, stepping,
stopping, and inspecting an autonomy backend run:

| Current AME method | Current proto projection | Status |
|--------------------|--------------------------|--------|
| `start(SessionRequest)` | `CreateSession(Session)` | Acceptable shell control |
| `readSnapshot()` | `ReadSession(Query)` | Needs idempotent/query semantics |
| `step()` | `UpdateSession(SessionStepRequest)` | Acceptable shell control |
| `requestStop(StopMode)` | `DeleteSession(SessionStopRequest)` | Acceptable shell control |

This surface should not be treated as the general way to invoke domain
behaviour. It controls the backend run.

### 2. Treat mission intent as a requirement input

`MissionIntent` is currently a list of PDDL goal fluents. That is useful inside
AME, but it is not yet a PYRAMID requirement model.

Target alignment:

- represent top-level user/system tasking as a typed Requirement when the
  relevant component contract exists
- retain PDDL fluents as AME-internal planning representation
- keep traceability from the PYRAMID `Requirement.base.base.id` to AME goals,
  plan records, BT execution, derived component requirements, and output
  entities

Until a canonical Tasks/Autonomy requirement proto exists, `MissionIntent`
should be documented as an adapter-local bootstrap input, not as the final
standard requirement contract.

### 3. Replace external command semantics with requirement placement

`ActionCommand` currently says:

```protobuf
message Command {
  pyramid.data_model.common.Entity base = 1;
  string command_id = 2;
  string action_name = 3;
  string signature = 4;
  string service_name = 5;
  string operation = 6;
  repeated StringKeyValue request_fields = 7;
}
```

That is a generic operation call envelope. It can carry "call
`object_of_interest.create_requirement`", but it does not make the requirement
the first-class behaviour invocation.

Target alignment:

- BT leaves should issue typed component EntityActions:
  - create/update/delete `Requirement` entities for work to be done
  - read output `Entity` products, capabilities, matches, details, or progress
  - update/delete requirements to refine or cancel behaviour
- the autonomy backend should persist those placements as requirement/action
  records with stable identifiers
- `Command` should either be renamed/reworked as a requirement placement record,
  or remain an internal/compatibility egress artefact below the PYRAMID standard
  boundary

For Tactical Objects, an AME action that needs object search should externally
become:

```text
Object_Of_Interest_Service.CreateRequirement(ObjectInterestRequirement)
ReadMatch(Query)
ReadDetail(Query)
UpdateRequirement(ObjectInterestRequirement) or DeleteRequirement(Identifier)
```

not:

```text
Command(service_name="tactical_objects", operation="search", ...)
```

### 4. Make reads idempotent and queryable

Current AME egress methods drain vectors:

```cpp
std::vector<ActionCommand> pullCommands();
std::vector<GoalDispatch> pullGoalDispatches();
std::vector<DecisionRecord> pullDecisionRecords();
```

EntityActions `Read*` operations should be idempotent queries over retained
state. The backend needs stores, not just queues:

| Entity | Store requirement |
|--------|-------------------|
| Requirement placements / commands | Query by id, status, target service, session, and read/ack state |
| Goal dispatches | Query by dispatch id, agent id, status, and session |
| Decision records | Query by id, session, world version, and planning attempt |
| Session snapshots | Query current and optionally historical snapshots by session id |

If one-shot delivery is still needed for a transport, it should be an adapter
policy around `Query.one_shot`, not the only backend state model.

### 5. Feed results back as entity and requirement state

`CommandResult` and `DispatchResult` are currently callback-style ingress
messages. They can remain useful internally, but externally the aligned pattern
is:

- requirement `Achievement.status`/quality/achievability changes
- output entities become readable from the producing component
- observed world facts are ingested as state/evidence entities with traceable
  source identifiers
- cancellation/refinement is expressed as `DeleteRequirement` or
  `UpdateRequirement`

For tactical-object behaviours, the result of actioning a requirement is not
primarily "command succeeded"; it is the availability and quality of
`ObjectMatch`, `ObjectDetail`, derived evidence requirements, and the
requirement achievement state.

## Implementation Priorities

### Phase 1: Documented compatibility boundary

1. Keep the current AME C++/Python session interface intact.
2. Treat `Command` as a compatibility egress record, not the canonical standard
   behaviour invocation model.
3. Add documentation/tests showing Tactical Objects behaviour invocation as
   `CreateRequirement` plus read-only product reads.
4. Preserve traceability from `Command.command_id` to the typed requirement id
   while the compatibility layer exists.

### Phase 2: Queryable stores

1. Replace drain-only egress semantics with retained stores behind `Read*`
   methods.
2. Support `Query.id` and `Query.one_shot` consistently.
3. Keep explicit status for pending, running/actioning, achieved, failed, and
   cancelled requirement placements.
4. Expose decision records and snapshots as queryable entities.

### Phase 3: Requirement-native BT invocation

1. Make BT `InvokeService` or its replacement construct typed requirement
   payloads for target component services.
2. Generate or configure action-to-requirement mappings from the component
   proto surface, not from arbitrary string operation names.
3. For Tactical Objects, map search/interest behaviours to
   `ObjectInterestRequirement` and output reads to `ObjectMatch`/`ObjectDetail`.
4. Use `UpdateRequirement`/`DeleteRequirement` for refinement and cancellation.

### Phase 4: Standard autonomy/tasking model

1. Introduce or adopt a canonical Tasks/Autonomy requirement proto for top-level
   mission/tasking.
2. Map AME `MissionIntent` to that requirement model at the boundary.
3. Keep PDDL, plans, and BT XML internal unless emitted as decision/audit
   records.

## File Locations

| Component | Path |
|-----------|------|
| C++ autonomy shell | `subprojects/AME/include/ame/autonomy_backend.h` |
| Current C++ adapter | `subprojects/AME/include/ame/current_ame_backend_adapter.h` |
| Current C++ implementation | `subprojects/AME/src/lib/autonomy_backend.cpp` |
| Python autonomy shell | `subprojects/AME/autonomy_backend.py` |
| Autonomy data model proto | `subprojects/PYRAMID/proto/pyramid/data_model/autonomy.proto` |
| Autonomy services proto | `subprojects/PYRAMID/proto/pyramid/components/autonomy_backend/services/provided.proto` |
| Tactical Objects data model proto | `subprojects/PYRAMID/proto/pyramid/data_model/tactical.proto` |
| Tactical Objects provided proto | `subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/provided.proto` |
| Tactical Objects consumed proto | `subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/consumed.proto` |
| PYRAMID responsibilities | `subprojects/PYRAMID/PYRAMID_COMPONENT_RESPONSIBILITIES.md` |
