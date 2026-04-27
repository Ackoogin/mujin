# AME Autonomy Backend Alignment

This document records the target PYRAMID-facing AME proto shape.

Objectives and Tasks own objective/task semantics. They delegate planning to AME
by placing an AME-specific `PlanningRequirement`, then delegate execution by
placing an `ExecutionRequirement` that references an approved plan. AME then
exposes plans as addressable resources and execution runs / placement
traceability as read-only products.

This is the new target shape. The existing AME backend implementation can be
changed to match it; the proto does not preserve the previous session/command
compatibility surface.

## Design Decisions

| Decision | Rationale |
|----------|-----------|
| AME has separate `PlanningRequirement` and `ExecutionRequirement` types | Objectives/Tasks need typed ways to delegate planning and execution without AME owning their schemas |
| AME does not define Objective or Task requirement schemas | Objective/Task contracts remain owned by their components |
| Upstream traceability uses `RequirementReference` | AME can refer back to Objective/Task/component requirements by id, service, component, and type |
| Plans are addressable resources | Plans can be produced by AME planning or imported by a coordinator before being executed by id |
| Execution outputs are read-only products | Execution runs and requirement placements are products of AME execution |
| Component actioning is typed EntityActions | AME should place/update/delete typed component requirements and read products/capabilities |
| No generic command bus in the target PYRAMID proto | `service_name + operation + request_fields` is replaced by typed requirement placement traceability at the PYRAMID boundary |
| AME internals may still use command-shaped execution | AME remains usable as a planning/execution library outside PYRAMID; wrapper code maps command-shaped plan leaves to typed PYRAMID placements where bindings exist |

## Reference Contracts

| File | Role |
|------|------|
| `pyramid/data_model/common.proto` | Defines `Entity`, `Requirement`, `Achievement`, `Capability`, and `Query` |
| `pyramid/data_model/autonomy.proto` | Defines the target AME data model |
| `pyramid/components/autonomy_backend/services/provided.proto` | Defines AME provided services |
| `pyramid/data_model/tactical.proto` | Concrete component requirement/product example |
| `pyramid/components/tactical_objects/services/provided.proto` | Example provided requirement CRUD and products |
| `pyramid/components/tactical_objects/services/consumed.proto` | Example consumed evidence and capability services |
| `subprojects/PYRAMID/doc/architecture/PYRAMID_COMPONENT_RESPONSIBILITIES.md` | Responsibility guidance for Objectives, Tasks, Tactical Objects, progress, quality, and capability |

## Target Contract

New data-model elements in `pyramid/data_model/autonomy.proto`:

| Element | Purpose |
|---------|---------|
| `PlanningRequirement` | Typed `Requirement` placed on AME when a coordinator needs a plan |
| `ExecutionRequirement` | Typed `Requirement` placed on AME when a coordinator wants an approved plan executed |
| `RequirementReference` | Traceability to upstream Objective/Task/component requirements without AME owning those schemas |
| `PlanningGoal` | Goal accepted by AME, as either a requirement reference or an AME-domain expression |
| `PlanningPolicy` / `ExecutionPolicy` | Policy knobs such as replanning limits and placement concurrency |
| `AgentState` | Backend-neutral view of available agents/resources |
| `StateUpdate` / `WorldFactUpdate` | State ingress for AME's world model |
| `Capabilities` | Read-only AME capability declaration |
| `Plan` / `PlanStep` | Addressable plan resources produced by AME or imported by coordinators |
| `PlannedComponentInteraction` | Planned EntityActions interaction with a target component |
| `ExecutionRun` | Read-only execution state and achievement for an execution requirement |
| `RequirementPlacement` | Read-only trace of typed EntityActions interactions AME performs against target components |

Provided services in
`pyramid/components/autonomy_backend/services/provided.proto`:

| Service | Operations | Purpose |
|---------|------------|---------|
| `Capabilities_Service` | `ReadCapabilities` | Discover AME backend capabilities |
| `Planning_Requirement_Service` | `CreatePlanningRequirement`, `ReadPlanningRequirement`, `UpdatePlanningRequirement`, `DeletePlanningRequirement` | AME planning delegation surface |
| `Execution_Requirement_Service` | `CreateExecutionRequirement`, `ReadExecutionRequirement`, `UpdateExecutionRequirement`, `DeleteExecutionRequirement` | AME execution delegation surface |
| `State_Service` | `CreateState`, `UpdateState`, `DeleteState` | Push authoritative or believed world state into AME |
| `Plan_Service` | `CreatePlan`, `ReadPlan`, `UpdatePlan`, `DeletePlan` | Manage AME plan resources |
| `Execution_Run_Service` | `ReadRun` | Read AME execution run products |
| `Requirement_Placement_Service` | `ReadPlacement` | Read traceability for typed component requirement placements and reads |

The top-level interaction is:

```text
Objectives/Tasks
  -> Planning_Requirement_Service.CreatePlanningRequirement(PlanningRequirement)
  -> Plan_Service.ReadPlan(Query)
  -> Execution_Requirement_Service.CreateExecutionRequirement(ExecutionRequirement)
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

## Progress To Date

The current implementation has established the new contract and an in-process
bridge that exercises it through generated PYRAMID service bindings.

| Area | Current state |
|------|---------------|
| Proto contract | `autonomy.proto` defines `PlanningRequirement`, `ExecutionRequirement`, `StateUpdate`, `Capabilities`, `Plan`, `ExecutionRun`, and `RequirementPlacement` |
| Provided services | `provided.proto` exposes AME capabilities, separate planning and execution requirement CRUD, plan CRUD, state ingress, and read-only run/placement services |
| PYRAMID generation | C++ service bindings, FlatBuffers schemas, JSON codecs, and protobuf generation include the autonomy backend contract |
| AME bridge | `PyramidAutonomyBridge` implements the generated autonomy backend service handler in-process |
| Retained resources/products | The bridge keeps requirement, state, plan, run, and placement stores behind read operations instead of drain-only queues |
| Query semantics | `Plan`, `ExecutionRun`, and `RequirementPlacement` reads support retained repeated reads and `Query.one_shot` removal |
| Planning path | `PlanningRequirement` entities are planned and exposed as CRUD-addressable `Plan` resources |
| Execution path | `ExecutionRequirement` entities execute approved plans and produce `ExecutionRun` products and placement traceability |
| State feedback | `State_Service.CreateState` / `UpdateState` update the AME world model and can complete waiting execution steps |
| Requirement lifecycle | Planning and execution update calls refresh their stored requirements; execution delete removes the requirement and marks matching runs cancelled |
| Execution abstraction | AME plan leaves now submit through `IExecutionSink`, allowing command-only or typed-placement-backed execution |
| Binding policies | `CommandOnly`, `PreferTypedPlacement`, and `RequireTypedPlacement` behavior is implemented; strict policy rejects unbound commands and fails the run |
| Test coverage | Contract, bridge, execution-sink, and backend execution behaviors are covered by focused C++ tests |

Implementation files of interest:

| File | Role |
|------|------|
| `subprojects/AME/include/ame/pyramid_autonomy_bridge.h` | Public bridge options and generated service handler implementation surface |
| `subprojects/AME/src/lib/pyramid_autonomy_bridge.cpp` | In-process AME/PYRAMID contract bridge |
| `subprojects/AME/include/ame/execution_sink.h` | AME execution abstraction for command-only and typed-placement modes |
| `subprojects/AME/src/lib/execution_sink.cpp` | Execution sink implementations and binding logic |
| `subprojects/AME/src/lib/autonomy_backend.cpp` | Existing AME backend adapter wired through `IExecutionSink` |
| `subprojects/AME/tests/test_ame_autonomy_contract.cpp` | Generated-service contract and bridge behavior tests |
| `subprojects/AME/tests/test_execution_sink.cpp` | Command-only, typed-placement, fallback, and strict-policy tests |

Focused verification command:

```bat
ctest --test-dir build -C Release -R "ExecutionSink|AutonomyBackend|AmeAutonomyContract|ProtoBindings|CodecDispatch" --output-on-failure
```

Use that selected suite for AME/PYRAMID contract and execution-binding changes.

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
| `COMP-062-R10` determine implementation solution cost | Future AME plan resources can carry cost once a standard cost model exists |

AME does not define `ObjectiveRequirement`, `TaskRequirement`, or
`DerivedNeed`. It accepts `RequirementReference` values that point to upstream
requirements owned by those components.

## Execution Binding Model

AME needs to support two deployment shapes:

1. PYRAMID deployments, where execution should be visible as typed
   requirement placements against target component services.
2. Library or non-PYRAMID deployments, where AME can execute command-shaped BT
   leaves without requiring PYRAMID component contracts.

The implementation therefore separates AME-internal execution from the
PYRAMID-facing contract:

| Layer | Shape | Purpose |
|-------|-------|---------|
| AME plan leaf | `ActionCommand` / BT `InvokeService` style command | Internal execution intent produced by AME planning/BT execution |
| Execution sink | `IExecutionSink` | Host-selected boundary that accepts AME execution intent |
| PYRAMID wrapper | `RequirementBindingExecutionSink` | Maps known commands to typed `RequirementPlacement` records |
| Non-PYRAMID host | `CommandQueueExecutionSink` or custom sink | Exposes command-shaped execution directly to the host |
| Public PYRAMID read model | `RequirementPlacement` | Read-only trace of typed requirement placement/progress |

The current execution policies are:

| Policy | Behavior |
|--------|----------|
| `CommandOnly` | Every AME execution leaf remains command egress; no typed placement is required |
| `PreferTypedPlacement` | Bound actions become typed placements; unbound actions fall back to command egress |
| `RequireTypedPlacement` | Bound actions become typed placements; unbound actions are rejected and the run fails |

Execution bindings are deliberately host/configuration data rather than baked
into AME's proto. A binding maps an AME action shape to a PYRAMID target:

| Binding concept | Example |
|-----------------|---------|
| `action_name` | `search` |
| command `service_name` / `operation` | `imaging` / `search` |
| target component | `tactical_objects` |
| target service | `Object_Of_Interest_Service` |
| target requirement type | `pyramid.data_model.tactical.ObjectInterestRequirement` |
| placement operation | `CREATE_REQUIREMENT` |

The default PYRAMID wrapper currently includes example bindings for `move` and
`search`, and hosts can register or replace bindings through
`PyramidAutonomyBridge::registerExecutionBinding`. These bindings currently
project AME execution into retained `RequirementPlacement` products. The next
implementation step is to connect those placements to real generated component
service clients where concrete component contracts exist.

This model keeps the public PYRAMID API free of a generic command bus while
still allowing AME to remain a reusable library for command-oriented users.

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
| `execution_requirement_id` | id of the AME execution requirement delegated by Tasks/Objectives |
| `planning_requirement_id` | id of the AME planning requirement that produced the plan |
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

AME resource/product read services should therefore support:

- lookup by `id[]`
- repeated idempotent reads when `one_shot` is unset or false
- one-shot delivery only when `Query.one_shot` is set

Current implementation note: `Plan_Service.ReadPlan`,
`Execution_Run_Service.ReadRun`, and
`Requirement_Placement_Service.ReadPlacement` implement `one_shot` removal.
`Plan_Service` also supports explicit create, update, and delete for externally
approved or imported plans.
Planning and execution requirement reads are currently retained/idempotent only;
whether requirements should also be removed by `one_shot` remains an open
bridge-hardening decision.

If status, session, target-service, or world-version filtering is needed
publicly, that should be a future common-query extension rather than a hidden
AME-specific convention.

## Backend Implementation Status

| Direction | Status |
|-----------|--------|
| Replace the old session/command backend boundary with planning and execution requirement CRUD | Implemented in `PyramidAutonomyBridge` |
| Map planning requirements into AME planning problems and execution requirements into execution runs | Implemented for expression goals, approved plans, and available-agent state |
| Expose generated and imported plans through `Plan_Service` CRUD | Implemented |
| Record typed component interactions as `RequirementPlacement` products | Implemented through `RequirementBindingExecutionSink` |
| Expose execution state through `Execution_Run_Service.ReadRun` and requirement status / achievement | Implemented for waiting, achieved, failed, and cancelled states |
| Ingest external world-state changes through `State_Service` | Implemented for create/update state feedback |
| Execute plans through generated target component service bindings | Remaining work; placements are recorded locally but not yet invoked against real generated clients |

## Remaining Plan

The remaining work should proceed in slices that keep the contract testable
after each step.

### Slice 1: Contract Stabilisation

Status: implementation complete; external consumer review remains.

Completed:

- `PlanningRequirement` and `ExecutionRequirement` are the AME delegation surfaces.
- `Plan` is a CRUD-addressable resource; `ExecutionRun` and
  `RequirementPlacement` are read-only products.
- `State_Service` is AME world-state ingress.
- Objectives/Tasks references are traceability only until those component protos
  exist.
- Generated JSON/FlatBuffers/protobuf coverage includes autonomy messages.

Open:

- Review field naming and enum values with PYRAMID consumers before treating
  the proto as stable.
- Decide whether placement records need explicit fields for command fallback
  traceability, or whether fallback remains an AME-library concern only.

### Slice 2: Backend Bridge Hardening

Status: bridge behavior is substantially implemented; audit/replay hardening
remains.

Completed:

- Retained repeated reads are implemented for product stores.
- `Query.one_shot` removal is implemented for plan, run, and placement
  reads.
- `DeleteExecutionRequirement` cancels matching execution runs.
- State feedback can move waiting execution runs and placements to completed.
- Strict binding-policy failures mark the run and requirement failed.

Open:

- Improve `UpdatePlanningRequirement` / `UpdateExecutionRequirement` handling for
  requirement refinement.
- Decide whether planning/execution requirement reads should support
  `one_shot`; current requirement reads are retained/idempotent.
- Persist enough plan/run/placement metadata for replay and audit.
- Add failure causes to requirement achievement and execution-run products.

### Slice 3: Typed Component Invocation

Status: binding/projection layer complete; real service invocation remains.

- Introduce a component service client abstraction behind
  `RequirementBindingExecutionSink`.
- For each bound action, construct the target component requirement using a
  typed mapper instead of only recording placement metadata.
- Call generated target component service bindings where contracts exist.
- Read target requirement/product/capability state and feed the result back into
  AME `RequirementPlacement`, `ExecutionRun`, and world-state updates.
- Keep `CommandOnly` available for non-PYRAMID users and for actions without
  PYRAMID component contracts.

Tactical Objects can remain the first concrete CRUD example, but it is not
expected to cover all execution needs. Future execution-oriented components
such as mobility, guidance, sensor control, communications, or effectors should
provide their own requirement CRUD contracts and then be registered as bindings.

### Slice 4: Test Client And Scenario Coverage

Status: generated-service and focused scenario tests exist; standalone client
and UI-facing smoke coverage remain.

Completed:

- Generated-service bridge tests create requirements, read plans/runs/placements,
  push state feedback, and re-read completion state.
- Plan-only and plan-and-execute fixtures exist.
- Strict binding-policy failure is covered.
- Preferred binding-policy fallback is covered at the execution-sink layer.

Open:

- Add a small generated-contract test client that can:
  - create a `PlanningRequirement`
  - read the resulting `Plan`
  - create an `ExecutionRequirement`
  - read the `ExecutionRun`
  - read `RequirementPlacement` records
  - push `StateUpdate` feedback
  - re-read the run and requirement achievement
- Keep the client useful both as a smoke test and as a reference for future
  Objectives/Tasks integrations.

### Slice 5: Objectives/Tasks Integration

Status: dependency.

- Wait for Objective and Task requirement protos owned by their components.
- Map Objective/Task requirements into AME `RequirementReference` and
  `PlanningGoal` values.
- Ensure Objectives/Tasks retain responsibility for objective/task quality,
  while AME reports plan quality, execution progress, and placement outcomes.
- Add cross-component tests once those protos and service bindings exist.

## Devenv Support Considerations

The current AME devenv is centred on ROS2/PCL workflows:

| Path | Current purpose |
|------|-----------------|
| `subprojects/AME/doc/guides/devenv_ros2_quickstart.md` | ROS2 and Foxglove devenv guide |
| `subprojects/AME/tools/devenv/start_devenv.bat` | Launches the ROS2-backed devenv |
| `subprojects/AME/tools/devenv/start_devenv_pcl.bat` | Launches the local PCL/Python-binding backend |

The new autonomy contract should be added as a third devenv capability rather
than replacing the current ROS2 or PCL modes.

### Devenv Goals

- Let developers exercise the new planning and execution requirement service
  contracts without writing custom C++.
- Make the retained products visible: requirements, plans, execution runs,
  placements, and state updates.
- Allow switching execution policy between `CommandOnly`,
  `PreferTypedPlacement`, and `RequireTypedPlacement`.
- Allow a small binding configuration to map AME actions to target component
  requirement services.
- Keep the dev loop local and deterministic before adding distributed transport.

### Proposed Devenv Shape

| Devenv addition | Description |
|-----------------|-------------|
| `--backend pyramid` | New backend mode that talks to an in-process `PyramidAutonomyBridge` |
| Contract client wrapper | Thin Python or subprocess wrapper around generated C++ JSON service dispatch |
| Requirement panel | Create/update/delete `PlanningRequirement` and `ExecutionRequirement` from editable JSON |
| State panel | Send `StateUpdate` messages and inspect world-state-driven progress |
| Plan panel | Create/update/delete plans through `Plan_Service` and show ordered `PlanStep` rows |
| Run panel | Read `Execution_Run_Service.ReadRun` and show state/achievement/progress |
| Placement panel | Read `Requirement_Placement_Service.ReadPlacement` and show bound target interactions |
| Binding editor | Load/save action-to-component binding examples for local scenarios |

Recommended launch shape:

```bat
subprojects\AME\tools\devenv\start_devenv_pyramid.bat ^
  --domain subprojects\AME\domains\uav_search\domain.pddl ^
  --problem subprojects\AME\domains\uav_search\problem.pddl ^
  --execution-policy PreferTypedPlacement ^
  --bindings subprojects\AME\tools\devenv\config\pyramid_bindings.example.json
```

This can initially run entirely in-process. A later transport-backed mode can
reuse the same UI panels once PYRAMID service transport is selected.

### Devenv Implementation Plan

1. Add a standalone C++ contract client or bridge runner that exposes the
   generated autonomy backend service handler through JSON request/response.
2. Add a Python adapter in `subprojects/AME/tools/devenv` that can call the
   runner and present the same operations as normal devenv backend methods.
3. Add a `start_devenv_pyramid.bat` launcher that verifies generated bindings
   and starts the UI with `--backend pyramid`.
4. Add example JSON fixtures for:
   - plan-only search
   - plan-and-execute search with state feedback
   - strict binding failure
   - command fallback
5. Add UI views for requirement, plan, run, and placement products.
6. Add smoke tests that run the pyramid backend without ROS2 and verify the UI
   adapter can complete a create/read/state-feedback/read cycle.

### Devenv Open Decisions

| Decision | Options |
|----------|---------|
| Local bridge process | Python extension binding, C++ subprocess with JSON stdin/stdout, or lightweight HTTP service |
| Binding config format | JSON fixture, command-line flags, or generated PYRAMID component registry |
| Transport alignment | Keep in-process for now, then map to the eventual PYRAMID runtime transport |
| Target component simulation | Mock generated service clients first, then swap in real component service clients |
| Scenario ownership | Keep examples under AME devenv until Objectives/Tasks scenario fixtures exist |

## Review Checklist

- AME is invoked by `CreatePlanningRequirement(PlanningRequirement)` followed by
  `CreateExecutionRequirement(ExecutionRequirement)` when execution is needed,
  not by session start or generic command.
- Objectives and Tasks remain upstream owners of their own requirement schemas.
- AME exposes `Plan` as a resource and exposes `ExecutionRun` /
  `RequirementPlacement` as products.
- Component behaviours are typed EntityActions against target component
  services.
- Repeated non-`one_shot` reads are idempotent.
- PDDL/BT/LAPKT details remain AME implementation details unless exposed as AME
  products such as `Plan.compiled_bt_xml`.
- AME can still be embedded as a command-oriented library through
  `CommandQueueExecutionSink`.
- PYRAMID deployments use `RequirementBindingExecutionSink` to project or invoke
  typed component requirement placements.
- Devenv support includes a local way to create requirements, push state
  feedback, and inspect plans/runs/placements before distributed transport is
  required.

## File Locations

| Component | Path |
|-----------|------|
| AME data model proto | `subprojects/PYRAMID/proto/pyramid/data_model/autonomy.proto` |
| AME provided services proto | `subprojects/PYRAMID/proto/pyramid/components/autonomy_backend/services/provided.proto` |
| Common data model proto | `subprojects/PYRAMID/proto/pyramid/data_model/common.proto` |
| Tactical Objects data model proto | `subprojects/PYRAMID/proto/pyramid/data_model/tactical.proto` |
| Tactical Objects provided proto | `subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/provided.proto` |
| Tactical Objects consumed proto | `subprojects/PYRAMID/proto/pyramid/components/tactical_objects/services/consumed.proto` |
| PYRAMID responsibilities | `subprojects/PYRAMID/doc/architecture/PYRAMID_COMPONENT_RESPONSIBILITIES.md` |
