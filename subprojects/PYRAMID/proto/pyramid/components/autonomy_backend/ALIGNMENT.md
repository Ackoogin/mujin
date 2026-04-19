# AME Autonomy Backend Alignment

This document records the target PYRAMID-facing AME proto shape.

Objectives and Tasks own objective/task semantics. They delegate planning and
optional execution to AME by placing an AME-specific
`PlanningExecutionRequirement`. AME then plans, executes through typed
component EntityActions, and exposes plans, execution runs, and placement
traceability as read-only products.

This is the new target shape. The existing AME backend implementation can be
changed to match it; the proto does not preserve the previous session/command
compatibility surface.

## Design Decisions

| Decision | Rationale |
|----------|-----------|
| AME has its own `PlanningExecutionRequirement` | Objectives/Tasks need a typed way to delegate planning/execution without AME owning their schemas |
| AME does not define Objective or Task requirement schemas | Objective/Task contracts remain owned by their components |
| Upstream traceability uses `RequirementReference` | AME can refer back to Objective/Task/component requirements by id, service, component, and type |
| AME outputs are read-only products | Plans, execution runs, and requirement placements are products of AME planning/execution |
| Component actioning is typed EntityActions | AME should place/update/delete typed component requirements and read products/capabilities |
| No generic command bus in the target proto | `service_name + operation + request_fields` is replaced by `RequirementPlacement` traceability |

## Reference Contracts

| File | Role |
|------|------|
| `pyramid/data_model/common.proto` | Defines `Entity`, `Requirement`, `Achievement`, `Capability`, and `Query` |
| `pyramid/data_model/autonomy.proto` | Defines the target AME data model |
| `pyramid/components/autonomy_backend/services/provided.proto` | Defines AME provided services |
| `pyramid/data_model/tactical.proto` | Concrete component requirement/product example |
| `pyramid/components/tactical_objects/services/provided.proto` | Example provided requirement CRUD and products |
| `pyramid/components/tactical_objects/services/consumed.proto` | Example consumed evidence and capability services |
| `PYRAMID_COMPONENT_RESPONSIBILITIES.md` | Responsibility guidance for Objectives, Tasks, Tactical Objects, progress, quality, and capability |

## Target Contract

New data-model elements in `pyramid/data_model/autonomy.proto`:

| Element | Purpose |
|---------|---------|
| `PlanningExecutionRequirement` | Typed `Requirement` placed on AME by Objectives, Tasks, or another coordinator |
| `RequirementReference` | Traceability to upstream Objective/Task/component requirements without AME owning those schemas |
| `PlanningGoal` | Goal accepted by AME, as either a requirement reference or an AME-domain expression |
| `PlanningPolicy` | Planning and execution policy knobs such as replanning limits |
| `AgentState` | Backend-neutral view of available agents/resources |
| `StateUpdate` / `WorldFactUpdate` | State ingress for AME's world model |
| `Capabilities` | Read-only AME capability declaration |
| `Plan` / `PlanStep` | Read-only plan products produced by AME |
| `PlannedComponentInteraction` | Planned EntityActions interaction with a target component |
| `ExecutionRun` | Read-only execution state and achievement for a delegated requirement |
| `RequirementPlacement` | Read-only trace of typed EntityActions interactions AME performs against target components |

Provided services in
`pyramid/components/autonomy_backend/services/provided.proto`:

| Service | Operations | Purpose |
|---------|------------|---------|
| `Capabilities_Service` | `ReadCapabilities` | Discover AME backend capabilities |
| `Planning_Execution_Service` | `CreateRequirement`, `ReadRequirement`, `UpdateRequirement`, `DeleteRequirement` | Primary AME delegation surface |
| `State_Service` | `CreateState`, `UpdateState`, `DeleteState` | Push authoritative or believed world state into AME |
| `Plan_Service` | `ReadPlan` | Read AME plan products |
| `Execution_Run_Service` | `ReadRun` | Read AME execution run products |
| `Requirement_Placement_Service` | `ReadPlacement` | Read traceability for typed component requirement placements and reads |

The top-level interaction is:

```text
Objectives/Tasks
  -> Planning_Execution_Service.CreateRequirement(PlanningExecutionRequirement)
  -> Plan_Service.ReadPlan(Query)
  -> Execution_Run_Service.ReadRun(Query)
  -> Requirement_Placement_Service.ReadPlacement(Query)
```

State updates enter separately:

```text
World model producer
  -> State_Service.CreateState(StateUpdate)
  -> State_Service.UpdateState(StateUpdate)
  -> State_Service.DeleteState(Identifier)
```

## Responsibility Mapping

Objectives responsibilities imply upstream mission-level concerns:

| Responsibility | AME relationship |
|----------------|------------------|
| `COMP-039-R01` capture requirements | Objectives capture objective intent before delegating planning/execution |
| `COMP-039-R05` determine implementation scheme | Objectives decide when AME is the implementation mechanism |
| `COMP-039-R07` identify dependencies | Objectives can reference those dependencies when delegating to AME |
| `COMP-039-R09` coordinate objective enactment | AME can execute the delegated plan while Objectives retain objective-level coordination |
| `COMP-039-R10` identify progress of objective | Objectives read AME execution products and map them to objective progress |
| `COMP-039-R11` determine actual quality of solution | Objectives assess objective quality using AME products and component outputs |

Tasks responsibilities are the closest conceptual fit for AME:

| Responsibility | AME relationship |
|----------------|------------------|
| `COMP-062-R01` capture task requirements | Tasks capture tasking, then delegate planning/execution to AME |
| `COMP-062-R05` determine implementation solution | AME produces the plan for the delegated requirement |
| `COMP-062-R06` satisfy dependencies between derived needs | AME orders plan steps and component placements |
| `COMP-062-R07` coordinate solution enactment | AME enacts the plan through typed component EntityActions |
| `COMP-062-R08` identify progress of solution | AME exposes `ExecutionRun` and requirement achievement state |
| `COMP-062-R09` evaluate solution quality | AME exposes predicted plan quality; Tasks retain task-level quality assessment |
| `COMP-062-R10` determine implementation solution cost | Future AME plan products can carry cost once a standard cost model exists |

AME does not define `ObjectiveRequirement`, `TaskRequirement`, or
`DerivedNeed`. It accepts `RequirementReference` values that point to upstream
requirements owned by those components.

## Tactical Objects Example

When AME needs tactical object search as part of a plan, the aligned interaction
is typed component CRUD:

```text
Object_Of_Interest_Service.CreateRequirement(ObjectInterestRequirement)
Object_Of_Interest_Service.ReadRequirement(Query)
Matching_Objects_Service.ReadMatch(Query)
Specific_Object_Detail_Service.ReadDetail(Query)
Object_Of_Interest_Service.UpdateRequirement(ObjectInterestRequirement)
Object_Of_Interest_Service.DeleteRequirement(Identifier)
```

AME records those interactions as `RequirementPlacement` products:

| `RequirementPlacement` field | Example |
|------------------------------|---------|
| `planning_execution_requirement_id` | id of the AME requirement delegated by Tasks/Objectives |
| `plan_id` / `plan_step_id` | AME plan and step that caused the interaction |
| `target_component` | `tactical_objects` |
| `target_service` | `Object_Of_Interest_Service` |
| `target_type` | `pyramid.data_model.tactical.ObjectInterestRequirement` |
| `operation` | `REQUIREMENT_PLACEMENT_OPERATION_CREATE_REQUIREMENT` |
| `target_requirement_id` | id returned by `CreateRequirement` |
| `progress` | current progress of the placement |

The result of this interaction is not a generic command result. It is observed
through target component requirement achievement, readable `ObjectMatch` /
`ObjectDetail` products, AME `ExecutionRun` state, and AME world-state updates.

## Query Semantics

Current public `pyramid.data_model.common.Query` supports:

```protobuf
message Query {
  repeated pyramid.data_model.base.Identifier id = 1;
  optional bool one_shot = 2;
}
```

AME read services should therefore support:

- lookup by `id[]`
- repeated idempotent reads when `one_shot` is unset or false
- one-shot delivery only when `Query.one_shot` is set

If status, session, target-service, or world-version filtering is needed
publicly, that should be a future common-query extension rather than a hidden
AME-specific convention.

## Backend Implementation Direction

1. Replace the old session/command backend boundary with
   `PlanningExecutionRequirement` CRUD.
2. Map created requirements into AME planning problems and execution runs.
3. Expose generated plans through `Plan_Service.ReadPlan`.
4. Execute plans through generated target component service bindings, starting
   with Tactical Objects.
5. Record typed component interactions as `RequirementPlacement` products.
6. Expose execution state through `Execution_Run_Service.ReadRun` and
   `Requirement.base.status` / `Achievement`.
7. Ingest external world-state changes through `State_Service`.

## Review Checklist

- AME is invoked by `CreateRequirement(PlanningExecutionRequirement)`, not by
  session start or generic command.
- Objectives and Tasks remain upstream owners of their own requirement schemas.
- AME outputs are products: `Plan`, `ExecutionRun`, and
  `RequirementPlacement`.
- Component behaviours are typed EntityActions against target component
  services.
- Repeated non-`one_shot` reads are idempotent.
- PDDL/BT/LAPKT details remain AME implementation details unless exposed as AME
  products such as `Plan.compiled_bt_xml`.

## File Locations

| Component | Path |
|-----------|------|
| AME data model proto | `subprojects/PYRAMID/proto/pyramid/data_model/autonomy.proto` |
| AME provided services proto | `subprojects/PYRAMID/proto/pyramid/components/autonomy_backend/services/provided.proto` |
| Common data model proto | `subprojects/PYRAMID/proto/pyramid/data_model/common.proto` |
| Tactical Objects data model proto | `subprojects/PYRAMID/proto/pyramid/data_model/tactical.proto` |
| Tactical Objects provided proto | `subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/provided.proto` |
| Tactical Objects consumed proto | `subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/consumed.proto` |
| PYRAMID responsibilities | `subprojects/PYRAMID/PYRAMID_COMPONENT_RESPONSIBILITIES.md` |
