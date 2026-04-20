# AME Nodes High-Level Requirements

Requirements for the AME ROS2 node layer, which adapts `ame_core` and PCL-backed components into deployable mission-planning and execution services.

This document covers the node layer as a system, then provides separate high-level requirements for each AME node: `WorldModelNode`, `PlannerNode`, `ExecutorNode`, `AgentDispatcherNode`, and `AmeLifecycleManager`.

The requirements are derived from the authoritative ROS2 architecture in `subprojects/AME/doc/architecture/06-ros2.md` and the node implementations under `subprojects/AME/ros2/include/ame_ros2/` and `subprojects/AME/ros2/src/nodes/`.

## Subject Matter

The subject matter of the AME node layer is lifecycle-managed deployment, middleware adaptation, inter-node communication, topology-independent composition, and node-level observability for the AME planning and execution pipeline.

**Exclusions:**

- Core planning, compilation, and execution algorithms implemented inside `ame_core`.
- PDDL language semantics and domain authoring.
- Foxglove bridge behaviour and visualisation transport.
- Perception algorithm internals beyond node-level ingestion and integration boundaries.
- Transport-agnostic container semantics already specified by PCL.

## Interfaces

| Interface | Type | Description |
| :--- | :--- | :--- |
| `~/get_fact` | Provided | World model fact query service exposed by `WorldModelNode`. |
| `~/set_fact` | Provided | World model fact mutation service exposed by `WorldModelNode`. |
| `~/query_state` | Provided | World model snapshot service exposed by `WorldModelNode`. |
| `~/load_domain` | Provided | Runtime PDDL domain/problem loading service exposed by `WorldModelNode` and `PlannerNode`. |
| `~/plan` | Provided | Plan action exposed by `PlannerNode`, returning plan metadata and compiled BT XML. |
| `~/bt_xml` / `/{agent_id}/executor/bt_xml` | Provided | Compiled Behaviour Tree XML stream consumed by `ExecutorNode`. |
| `/world_state` | Provided | Authoritative published world-state snapshot from `WorldModelNode`. |
| `/executor/bt_events` / `/{agent_id}/executor/bt_events` | Provided | Behaviour Tree event stream published by `ExecutorNode`. |
| `/executor/status` / `/{agent_id}/executor/status` | Provided | Durable execution status published by `ExecutorNode`. |
| `~/dispatch_goals` | Provided | Goal-dispatch service exposed by `AgentDispatcherNode` for multi-agent execution. |
| `change_state` / `get_state` | Consumed | Lifecycle services used by `AmeLifecycleManager` to orchestrate startup and shutdown. |

## Design Decisions

### D1 - ROS2 Nodes Are an Adapter Layer
Keep AME domain logic in `ame_core` and PCL-backed components. The ROS2 nodes are deployment adapters responsible for lifecycle, middleware bindings, and interface exposure.

**Why**: This preserves portability, testability, and the architectural boundary between autonomy logic and transport/runtime concerns.

### D2 - Lifecycle-Managed Composition
The operational nodes use managed lifecycle transitions so that interfaces appear only when dependencies are ready and are torn down in a controlled order.

**Why**: Planning and execution depend on deterministic startup and shutdown ordering.

### D3 - Canonical Shared State in WorldModelNode
The authoritative shared mission state is owned by `WorldModelNode`; other nodes read or mutate that state through either direct in-process wiring or explicit services.

**Why**: A single source of truth is required for auditability, replanning, and consistent execution.

### D4 - Planner/Executor Separation by BT XML
`PlannerNode` is responsible for solving and compiling a plan into BT XML; `ExecutorNode` is responsible for loading, ticking, and reporting execution of that tree.

**Why**: Separating planning from execution keeps the pipeline modular and supports replacement, scaling, and distributed deployment.

### D5 - Deployment Topology Agnostic
The same node contracts shall support in-process, distributed, multi-agent, and multi-planner deployment modes.

**Why**: AME must be usable from local demos through distributed operational systems without changing core behaviour.

### D6 - Explicit Namespaced Interfaces
Planner and executor interfaces use node-relative or agent-relative names so multiple instances can coexist without ambiguity.

**Why**: Multi-planner and multi-agent operation require unambiguous routing boundaries.

### D7 - Observability Is a First-Class Concern
The node layer exposes world-state publication, plan audit, BT event publication, and execution status as part of the deployed system behaviour.

**Why**: Mission planning and execution need traceability, diagnosis support, and operator-visible state.

### D8 - In-Process Optimisation Without Contract Changes
Where nodes run in a single executor, direct dependency injection may bypass service hops, but the external contracts remain available for distributed deployment.

**Why**: This allows efficient local execution without fragmenting the architecture.

### D9 - Coordinated Lifecycle Orchestration
A dedicated manager coordinates dependency-safe lifecycle transitions across the AME node set.

**Why**: Manual lifecycle sequencing is error-prone and difficult to scale to more complex deployments.

---

## Detailed Requirements

## System-Level Requirements

### AME.001 - Adapter-Layer Boundary
The AME node layer shall expose `ame_core` and PCL-backed components through ROS2 interfaces without relocating core planning, world-model, or Behaviour Tree logic into the nodes themselves.

**Rationale**: Preserves the ROS-agnostic architecture and keeps domain logic portable.

### AME.002 - Managed Operational Nodes
`WorldModelNode`, `PlannerNode`, `ExecutorNode`, and `AgentDispatcherNode` shall implement managed lifecycle behaviour for configure, activate, deactivate, cleanup, and shutdown transitions.

**Rationale**: Deterministic node availability depends on explicit operational states.

### AME.003 - Dependency-Safe Startup and Shutdown
The AME node layer shall support a startup sequence in which state services become available before planning and execution, and a shutdown sequence that tears down dependent nodes before their dependencies disappear.

**Rationale**: Planner and executor functionality depend on the world model and must not outlive it unexpectedly.

### AME.004 - Dual Access Mode to Shared State
The AME node layer shall support both in-process direct dependency injection and distributed service-backed interaction with the world model.

**Rationale**: Both local single-executor and distributed deployments are first-class modes.

### AME.005 - Deployment Mode Support
The AME node layer shall support single-process, distributed, multi-agent, and multi-planner deployments while preserving the same high-level role separation between state, planning, and execution.

**Rationale**: Deployment flexibility is a core architectural goal.

### AME.006 - Runtime Domain Loading Path
The AME node layer shall support both file-parameter-driven domain loading for deployment and runtime domain loading from PDDL strings for development, testing, and dynamic reconfiguration workflows.

**Rationale**: Operational and development environments require different loading modes.

### AME.007 - Explicit Node-Level Observability
The AME node layer shall expose node-level observability for world state, planning outcomes, Behaviour Tree execution, and lifecycle progress through ROS2-visible interfaces and audit outputs.

**Rationale**: Planning and execution must be externally diagnosable.

### AME.008 - Explicit Failure Reporting
Service, action, and execution interfaces shall report failure explicitly with machine-usable success/failure outcomes and human-readable error information where applicable.

**Rationale**: Silent failure is unacceptable in autonomy orchestration.

### AME.009 - Scalable Namespacing and Multi-Instance Isolation
The AME node layer shall use node-relative and agent-relative naming so that multiple planners and executors can coexist without endpoint collision.

**Rationale**: Instance isolation is required for multi-planner and multi-agent operation.

## WorldModelNode High-Level Requirements

### WMN.001 - Canonical World Model Ownership
`WorldModelNode` shall own the canonical deployed `WorldModel` instance for the AME node layer.

**Rationale**: A single authoritative state source is required for consistency and auditability.

### WMN.002 - Fact and State Services
`WorldModelNode` shall expose services for fact read, fact write, state query, and runtime domain loading.

**Rationale**: Other nodes and external writers need explicit interfaces to interact with the authoritative state.

### WMN.003 - Configure-Time Service Availability
`WorldModelNode` shall make its state-query and mutation services available after configuration, before activation.

**Rationale**: Dependent nodes may need access to state before publishers are active.

### WMN.004 - Published Authoritative State
When active, `WorldModelNode` shall publish authoritative world-state snapshots on `/world_state` using durable, reliable semantics suitable for late-joining consumers.

**Rationale**: State consumers need a coherent, discoverable current picture.

### WMN.005 - Domain Initialisation and Reload
`WorldModelNode` shall support initial world-model schema construction from deployment parameters and schema replacement from runtime-supplied PDDL domain and problem strings.

**Rationale**: The world model must support both fixed deployment configuration and dynamic development workflows.

### WMN.006 - Versioned Snapshot Semantics
`WorldModelNode` shall expose versioned world-state snapshots and versioned fact mutation results.

**Rationale**: Consumers need to reason about freshness and state progression.

### WMN.007 - Queued Mutation Application
`WorldModelNode` shall support queued fact mutations and publish updated state when queued mutations are applied or the authoritative state changes.

**Rationale**: State publication should reflect coherent updates rather than arbitrary partial mutation timing.

### WMN.008 - Auditability of State Change
`WorldModelNode` shall support audit logging of world-model changes through its underlying component configuration.

**Rationale**: State authority without traceability is insufficient for mission assurance.

### WMN.009 - Perception Ingestion Boundary
`WorldModelNode` shall support optional perception-driven fact ingestion, including configurable confidence thresholding and explicit handling of authority conflicts between perceived and planned state.

**Rationale**: The node is the correct system boundary for integrating external detections into authoritative state.

## PlannerNode High-Level Requirements

### PLN.001 - Active-State Plan Interface
`PlannerNode` shall expose a plan action interface only when activated.

**Rationale**: Planning requests must be rejected until the planner is operational.

### PLN.002 - Goal Validation
`PlannerNode` shall reject planning requests that do not specify any goal fluents.

**Rationale**: Solving without an explicit goal is not meaningful.

### PLN.003 - World-State Snapshot Before Solve
`PlannerNode` shall acquire a snapshot of current world state before solving, using either an in-process `WorldModel` reference or the world-model query service.

**Rationale**: Planning must be based on current authoritative state.

### PLN.004 - Non-Blocking Solve Execution
`PlannerNode` shall execute plan solving without blocking the ROS2 executor thread.

**Rationale**: Long-running solve operations must not starve the node runtime.

### PLN.005 - Plan Result Payload
`PlannerNode` shall return structured planning outcomes including success/failure, solve timing, search statistics, plan actions, and compiled BT XML when a plan is found.

**Rationale**: Callers need both the executable result and diagnostic metadata.

### PLN.006 - Compiled BT Publication
When configured to do so, `PlannerNode` shall publish the compiled BT XML produced by a successful planning episode for downstream execution.

**Rationale**: The planner-to-executor handoff is a core pipeline boundary.

### PLN.007 - Runtime Domain Loading Interface
`PlannerNode` shall expose a runtime domain-loading service so that domain and problem models can be loaded from PDDL strings without relying on filesystem paths.

**Rationale**: Development and dynamic deployment workflows require runtime model injection.

### PLN.008 - Multiple Planner Instance Support
`PlannerNode` shall support multiple simultaneous instances with distinct node names, action endpoints, and BT XML outputs.

**Rationale**: Different planners may serve different domains or agent roles in the same deployment.

### PLN.009 - Plan Audit Support
`PlannerNode` shall support plan-audit recording for each planning episode through its underlying planner component.

**Rationale**: Planning outcomes require traceability and post-run analysis.

## ExecutorNode High-Level Requirements

### EXN.001 - Behaviour Tree Runtime Ownership
`ExecutorNode` shall own the deployed Behaviour Tree factory and execution runtime.

**Rationale**: Execution should be encapsulated behind a dedicated node boundary.

### EXN.002 - BT Load and Execute Interface
`ExecutorNode` shall accept compiled BT XML for execution through its subscribed BT XML interface and an equivalent direct loading entry point.

**Rationale**: The node must support both message-driven and in-process usage.

### EXN.003 - Configurable Tick-Driven Execution
When active, `ExecutorNode` shall tick the loaded tree at a configurable rate.

**Rationale**: Behaviour Tree execution is periodic and mission-rate dependent.

### EXN.004 - Dual World-Model Access Modes
`ExecutorNode` shall support both direct in-process world-model access and service-backed world-model access for Behaviour Tree nodes.

**Rationale**: Execution must work in both local and distributed deployments.

### EXN.005 - Execution Status Publication
`ExecutorNode` shall publish durable execution status values representing at least idle, running, success, and failure states.

**Rationale**: Other nodes and operators need a simple authoritative execution state view.

### EXN.006 - BT Event Publication
`ExecutorNode` shall publish Behaviour Tree execution events for observability while the node is active.

**Rationale**: Event-level introspection is necessary for diagnosis and audit.

### EXN.007 - Extension Node Registration
`ExecutorNode` shall register the AME extension Behaviour Tree nodes needed for service invocation, hierarchical planning, and multi-agent delegation, and it shall allow additional action node registrations through its factory.

**Rationale**: The executor must host both core and extension execution behaviours.

### EXN.008 - Agent-Scoped Topics
`ExecutorNode` shall support optional `agent_id`-based namespacing of its BT XML, event, and status topics.

**Rationale**: Multi-agent deployments need per-agent execution channels.

### EXN.009 - Terminal Status Retention
`ExecutorNode` shall retain the final execution status after the tree halts so that completion state remains queryable after active ticking stops.

**Rationale**: Consumers often inspect outcome after execution has finished.

## AgentDispatcherNode High-Level Requirements

### ADN.001 - Multi-Agent Dispatch Interface
`AgentDispatcherNode` shall expose a goal-dispatch service for triggering multi-agent task allocation and execution dispatch.

**Rationale**: Multi-agent coordination requires an explicit system entry point.

### ADN.002 - Per-Agent BT Dispatch
`AgentDispatcherNode` shall publish compiled BT XML to each selected agent's executor command topic.

**Rationale**: Agent executors require concrete executable trees, not abstract goals.

### ADN.003 - Per-Agent Status Monitoring
`AgentDispatcherNode` shall subscribe to each participating agent executor's status topic and maintain the latest known status per agent.

**Rationale**: Dispatch coordination depends on observing agent execution state.

### ADN.004 - Dynamic Agent Interface Provisioning
`AgentDispatcherNode` shall create per-agent BT publishers and status subscriptions dynamically as agents are encountered.

**Rationale**: The active agent set may not be fixed at startup.

### ADN.005 - In-Process Dependency Injection
`AgentDispatcherNode` shall support injection of in-process world-model, planner, compiler, and action-registry dependencies.

**Rationale**: Multi-agent coordination must remain available in single-process deployments.

### ADN.006 - Dispatch Outcome Reporting
`AgentDispatcherNode` shall report which agents were successfully dispatched and shall fail the overall request if no agent dispatch succeeds.

**Rationale**: Multi-agent callers need explicit partial-success visibility.

## AmeLifecycleManager High-Level Requirements

### LCM.001 - Coordinated Startup Ordering
`AmeLifecycleManager` shall support coordinated startup that configures and then activates managed nodes in dependency-safe order.

**Rationale**: Dependent nodes must not activate before their prerequisites are ready.

### LCM.002 - Reverse-Order Shutdown
`AmeLifecycleManager` shall support coordinated shutdown that deactivates and cleans up managed nodes in reverse dependency order.

**Rationale**: Reverse teardown prevents dependents from calling disappearing services.

### LCM.003 - Configurable Managed Set
`AmeLifecycleManager` shall allow the managed node list to be configured.

**Rationale**: Different deployments may include different planner, executor, or dispatcher instances.

### LCM.004 - Configurable Transition Timeout
`AmeLifecycleManager` shall allow the maximum wait time for lifecycle service interactions to be configured.

**Rationale**: Deployment environments vary in discovery and transition latency.

### LCM.005 - Lifecycle Service-Based Control
`AmeLifecycleManager` shall control nodes through lifecycle state-change and state-query services rather than through private in-process assumptions.

**Rationale**: Service-mediated control preserves node boundary integrity.

### LCM.006 - Startup Failure Propagation
`AmeLifecycleManager` shall report startup failure when a required lifecycle service is unavailable, a transition times out, or a managed node rejects a required transition.

**Rationale**: Partial or silent startup is unsafe and difficult to diagnose.

---

## Design Decision Traceability

| Requirement | Design Decision |
| :--- | :--- |
| `AME.001` | `D1` |
| `AME.002` | `D2` |
| `AME.003` | `D2`, `D3`, `D9` |
| `AME.004` | `D3`, `D5`, `D8` |
| `AME.005` | `D5`, `D6` |
| `AME.006` | `D5` |
| `AME.007` | `D7` |
| `AME.008` | `D7` |
| `AME.009` | `D5`, `D6` |
| `WMN.001` | `D1`, `D3` |
| `WMN.002` | `D3` |
| `WMN.003` | `D2`, `D3` |
| `WMN.004` | `D3`, `D7` |
| `WMN.005` | `D3`, `D5` |
| `WMN.006` | `D3`, `D7` |
| `WMN.007` | `D3`, `D7` |
| `WMN.008` | `D3`, `D7` |
| `WMN.009` | `D3`, `D7` |
| `PLN.001` | `D2`, `D4` |
| `PLN.002` | `D4` |
| `PLN.003` | `D3`, `D4`, `D8` |
| `PLN.004` | `D4` |
| `PLN.005` | `D4`, `D7` |
| `PLN.006` | `D4`, `D6` |
| `PLN.007` | `D5` |
| `PLN.008` | `D5`, `D6` |
| `PLN.009` | `D7` |
| `EXN.001` | `D1`, `D4` |
| `EXN.002` | `D4`, `D8` |
| `EXN.003` | `D4` |
| `EXN.004` | `D3`, `D5`, `D8` |
| `EXN.005` | `D6`, `D7` |
| `EXN.006` | `D7` |
| `EXN.007` | `D4`, `D5` |
| `EXN.008` | `D5`, `D6` |
| `EXN.009` | `D7` |
| `ADN.001` | `D2`, `D5` |
| `ADN.002` | `D5`, `D6` |
| `ADN.003` | `D5`, `D7` |
| `ADN.004` | `D5`, `D6` |
| `ADN.005` | `D5`, `D8` |
| `ADN.006` | `D7` |
| `LCM.001` | `D2`, `D9` |
| `LCM.002` | `D2`, `D9` |
| `LCM.003` | `D5`, `D9` |
| `LCM.004` | `D9` |
| `LCM.005` | `D2`, `D9` |
| `LCM.006` | `D7`, `D9` |

## Implementation Notes

- Treat `WorldModelNode` as the authoritative deployment boundary for state mutation and state publication.
- Treat `PlannerNode` as the boundary between state snapshotting and executable plan generation.
- Treat `ExecutorNode` as the boundary between compiled BT XML and runtime task execution.
- Treat `AgentDispatcherNode` as the multi-agent coordination adapter, not as the authority for world state or planning algorithms.
- Treat `AmeLifecycleManager` as deployment orchestration infrastructure rather than mission logic.
- Keep node contracts stable across in-process and distributed deployments even when direct dependency injection is used as an optimisation.
