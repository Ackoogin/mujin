# AME Planning/Execution Pluggability Review

## Purpose

This review identifies architectural and interface changes that would improve pluggability of the AME planning and execution pipeline, with emphasis on:

- introducing a temporal planner backend (as evaluated in `../../research/AME/temporal_extension_research.md`)
- supporting alternative execution backends beyond the current BehaviourTree.CPP runtime
- clarifying what must change if the **entire decision-and-action loop** is swapped rather than just the planner or executor
- preserving current strengths: symbolic authority, observability, and safe fallback behaviour

Primary reference documents:

- `../architecture/01-overview.md`
- `../architecture/03-planning.md`
- `../architecture/04-execution.md`
- `../architecture/05-observability.md`
- `../architecture/07-extensions.md`
- `../../research/AME/temporal_extension_research.md`
- `../TODO.md`

## Executive Summary

AME is already close to being pluggable at a conceptual level, but several interfaces are still implicit or coupled to concrete implementations (LAPKT for planning, BT.CPP for execution).

More fundamentally, AME should be viewed not only as a "planner plus behaviour tree" stack, but as a **decision system** with this abstract loop:

`authoritative state -> decision process -> executable action commitments -> state transition feedback`

Today that loop is instantiated as:

- `WorldModel` as authoritative state
- LAPKT as the deliberation engine
- `PlanCompiler` + BT.CPP as the action realisation engine
- replan-on-failure as the control policy

If the project wants to support more radical substitutions, such as a reactive controller, HTN/task network system, temporal dispatcher, multi-agent allocator, or a human-supervised / learned advisory stack, then planner and executor interfaces are necessary but not sufficient. The architecture also needs a **system-level contract** above both of them.

The highest-impact improvements are:

1. **Define a decision-system contract** that makes state intake, decision production, action commitment, and feedback handling explicit.
2. **Stabilise planner and executor contracts** with explicit capability metadata.
3. **Introduce an internal intermediate representation (IR)** between planner output and execution compilation.
4. **Separate control policy from runtime engine** so replanning, retry, and mission abort logic remain backend-agnostic.
5. **Promote observability to a contract requirement**, not a backend-specific optional feature.

A phased approach can deliver this without destabilising the current system: first define system contracts and adapters, then migrate the existing LAPKT + BT flow onto those contracts, then add temporal and alternative runtime backends, and finally permit whole-loop substitution where justified.

## Reframing the Question

The key architecture question is not only "can we swap the planner?" or "can we swap the executor?" It is:

> What is the smallest set of invariants that makes AME still AME, even if the internal decision machinery is replaced?

That framing helps separate:

- **implementation choices**: LAPKT, BT.CPP, STN dispatcher, rule engine
- **system invariants**: authoritative state, governed action dispatch, auditability, fallback semantics

In that sense, the current planner/executor split is one concrete decomposition of a more general autonomy loop.

## System Invariants That Should Survive Any Swap

If the whole deliberation-and-action loop is replaceable, the following should remain stable:

1. **Authoritative state boundary**
   - `WorldModel` (or a future equivalent) remains the source of mission-relevant truth, versioned and auditable.
2. **Decision contract**
   - any backend must declare what kind of decisions it emits: full plan, partial plan, next-best action, schedule, delegated tasking, or advisory recommendation.
3. **Action contract**
   - emitted decisions must resolve to governed executable actions with explicit preconditions, ownership, timeout/cancel behaviour, and outcome reporting.
4. **Control policy**
   - retry, replan, abort, degrade, human-escalate, and backend-switch decisions should be explicit policy semantics rather than hidden engine quirks.
5. **Observability and assurance**
   - every decision proposal, commitment, execution transition, and world-state delta must remain reconstructable after the mission.

These invariants are what let AME absorb new decision technologies without becoming a different class of system.

## Current Coupling Points

### System-level coupling

Current documentation naturally describes a pipeline:

`WorldModel -> Planner -> PlanCompiler -> BT runtime`

That is clear and appropriate for the current baseline, but it also bakes in an assumption that deliberation always produces a plan and action realisation always means compiling into a behaviour tree.

Pluggability issues:

- the top-level product contract is implicit rather than named
- "decision" and "execution" are conflated with "plan" and "BT"
- recovery behaviour is described around replan-on-failure, not around a general control-policy abstraction
- action dispatch is implicitly tied to the BT node model

### Planning path

Current architecture indicates a direct flow from PDDL/WorldModel projection into LAPKT-oriented solve logic, then into a plan shape expected by the current PlanCompiler.

Pluggability issues:

- planner capabilities (classical vs temporal vs numeric) are not first-class handshake data
- plan result schema is implicitly aligned to sequential classical plans
- validation responsibilities (planner-native validation vs external VAL checks) are not normalised

### Execution path

Current execution is centred on compilation into BehaviourTree.CPP nodes and tick-based runtime semantics.

Pluggability issues:

- compiled artefact is effectively BT-specific
- runtime lifecycle semantics (start/tick/halt/recover) are not yet exposed as a generic executor interface
- compensation/recovery strategy is partly embedded in BT structure rather than an explicit policy layer

### Cross-cutting

Observability is rich, but some event schemas are naturally shaped by BT semantics and LAPKT episodes.

Pluggability issues:

- event taxonomy for "plan generated/validated/dispatched/failed" is not fully backend-neutral
- no explicit minimum telemetry contract for new planner/executor plugins

## Recommended Target Architecture

### 0) Decision System Contract Above Planner/Executor

Introduce an explicit `IDecisionSystem` or equivalent top-level contract that describes the autonomy loop as a whole.

Recommended responsibilities:

- ingest current authoritative state snapshot and mission intent
- choose a decision mode (full plan, partial plan, direct action, advisory-only)
- emit executable action commitments or a decision artefact plus dispatch instructions
- consume execution feedback and world-state deltas
- declare whether it supports replanning, continual policy updates, temporal dispatch, human confirmation gates, or advisory-only operation

Recommended request/response shape:

- `DecisionRequest`: mission id, intent/goals, state snapshot id, policy budget, environment metadata, safety constraints
- `DecisionResult`: status, decision artefact type, artefact payload, confidence/diagnostics, validation report, backend metadata

Recommended decision artefact classes:

- `PlanIR` for an ordered or partially ordered symbolic plan
- `ActionSequenceIR` for a short finite sequence
- `DirectActionIR` for immediate action selection
- `ScheduleIR` for temporal dispatch commitments
- `GoalDispatchIR` for agent-assignment / delegated tasking outputs
- `AdvisoryIR` for propose-verify-fallback flows where another authority approves execution

Why this matters:

- the existing planner/compiler/executor stack can implement this contract unchanged at first
- whole-loop swaps become deliberate architecture choices rather than ad hoc exceptions
- non-planning controllers can participate without pretending to be a planner

### Decision-System Archetypes Worth Supporting

Once the top-level contract exists, AME can support multiple classes of decision systems:

| Archetype | Decision output | Typical cadence | Example use |
|-----------|-----------------|-----------------|-------------|
| Classical planner + executor | full symbolic plan | episodic | current LAPKT + BT baseline |
| Temporal planner + dispatcher | scheduled plan with constraints | episodic + monitored | durative missions, concurrent activities |
| Reactive policy / rule engine | immediate next action | continuous | tight control loops, highly dynamic environments |
| HTN / task-network system | decomposed task graph | episodic + local repair | doctrine-driven missions |
| Workflow / DAG orchestrator | stage graph with retries/checkpoints | episodic | mission services / business-process style autonomy |
| Human-supervised adviser | recommendation only | on demand | operator approval gates |
| Learned or hybrid controller | action or ranking proposal | continuous or episodic | bounded neuro-symbolic assistance |

The important point is that these are not all "planner plugins". Some are alternative decision-system implementations with different artefact shapes and different feedback cadence.

### 1) Planner Backend Interface + Capability Model

Define a stable `IPlannerBackend` contract with explicit capability declaration.

Recommended capability fields:

- planning class: `classical`, `temporal`, `numeric`, `multi-agent`
- supported PDDL requirements (e.g. `:durative-actions`, `:fluents`)
- optimization objective support (makespan, action cost)
- deterministic mode availability
- validation support (native / external tool / none)

Recommended request/response shape:

- `PlanRequest`: domain/problem payload, world projection snapshot id, budget/timeouts, objective hints
- `PlanResult`: status, plan IR payload, diagnostics, validation report, backend metadata

Why this matters:

- temporal backends can be introduced without special-case branching
- classical LAPKT backend becomes one plugin among many
- mission-level selection/fallback policy can reason over capabilities instead of hardcoded backend names

### 2) Plan Intermediate Representation (Plan IR)

Insert a backend-neutral IR between planning and execution compilation.

Suggested IR levels:

- **Logical Plan IR**: actions, parameters, ordering constraints, causal links
- **Temporal Plan IR extension**: start/end times, duration bounds, invariant windows, resource mutexes
- **Execution Hints**: parallelisable groups, preferred retry boundaries, checkpoint annotations

Key rule:

- planner plugins emit Plan IR
- executor compilers consume Plan IR
- no planner writes directly into BT runtime objects

This decouples temporal planning adoption from BT-specific implementation details and enables alternative executors to reuse planning outputs.

### 3) Executor Backend Interface

Define an `IExecutionBackend` contract independent of BT.CPP.

Recommended lifecycle:

- `load(plan_ir, execution_context)`
- `start()`
- `tick_or_step()`
- `request_replan(reason)`
- `stop(mode)`
- `snapshot_state()`

Recommended semantics contract:

- action status model (`PENDING`, `RUNNING`, `SUCCEEDED`, `FAILED_TRANSIENT`, `FAILED_PERMANENT`)
- cancellation and timeout behaviour
- deterministic replay support where feasible
- bounded resource ownership handoff rules

Potential backend implementations:

- BT.CPP adapter (baseline)
- finite-state-machine runtime
- workflow/DAG engine runtime
- temporal dispatcher backed by STN schedule execution

If a decision backend emits `DirectActionIR` rather than `PlanIR`, the execution backend may become a much thinner dispatcher. That is acceptable, but it should still satisfy the same action lifecycle and observability contract.

### 4) Mission Control Policy Layer

Move replan/escalation decisions to a backend-agnostic control layer.

Policy concerns that should be engine-independent:

- retries vs local repair vs full replan
- backend switching on capability mismatch or repeated failure
- safety-mode transition and abort thresholds
- policy budgets (max replans, max wall-clock, critical action deadlines)
- human-confirmation gates for actions or plans above a risk threshold
- degrade-to-reactive or degrade-to-safe-hold behaviour when the preferred backend becomes unavailable

This enables consistent behaviour even when swapping planner/executor technologies.

### 5) Observability Contract for Pluggable Components

Define mandatory telemetry events and fields for all planner/executor plugins.

Minimum event families:

- planning request/response
- validation outcome
- dispatch and execution transitions
- failure classification and recovery decisions
- world-state assumption vs confirmation deltas

Minimum correlation fields:

- mission id, episode id, backend id, plan id, action instance id, timestamp, causality link ids

Outcome:

- preserve current auditability guarantees regardless of backend choice
- make A/B backend evaluation straightforward

## What Changes if the Whole Thing Is Swapped?

If the planner/compiler/executor bundle is replaced by a different control stack, the system should still answer the same questions:

1. **What state did the controller believe?**
   - snapshot id, relevant facts, confidence/authority tags if later introduced
2. **What decision did it make?**
   - plan, action, schedule, recommendation, or "no safe action"
3. **Why did it make that decision?**
   - goal linkage, policy rule, optimisation objective, heuristic ranking, human approval, or learned score
4. **What action was actually committed?**
   - executable instance id, parameters, resource owner, timeout/cancel semantics
5. **What happened next?**
   - success, transient failure, permanent failure, timeout, rejected by guard, superseded, or aborted

If a substitute architecture cannot answer those questions, it is not yet a drop-in autonomy backend for AME, even if it can produce actions.

## Boundary Recommendation: Swap the Decision Machinery, Not the Governance Surface

The safest way to think about whole-loop replacement is:

- allow the **decision machinery** to vary
- keep the **governance surface** stable

That stable governance surface should include:

- authoritative world-state access
- action schema / dispatch contract
- mission control policy
- observability / audit schema
- safety and assurance hooks

This keeps AME identifiable as an auditable autonomy framework even when the internal reasoning model changes dramatically.

## Proposed Whole-System Swap Surface

If the goal is to make the **entire current AME loop** replaceable without rewriting its internals, the cleanest approach is to wrap it as a black-box autonomy backend with a stable ingress and egress surface.

Conceptually:

`State/Intent Ingress -> AutonomyBackendShell -> Action/Outcome Egress`

Where `AutonomyBackendShell` may internally contain:

- the current `WorldModel`
- the current `Planner`
- the current `PlanCompiler`
- the current BT runtime / `MissionExecutor`

or a future alternative stack.

The key is that external systems no longer bind directly to those internals. They bind to the shell.

### Wrapper Intent

This surface should:

- wrap the **left side** of the autonomy loop: state snapshots, mission intent, constraints, operator commands
- wrap the **right side** of the autonomy loop: dispatched actions, execution outcomes, aborts, and telemetry
- leave current internal planning/execution code unchanged in the first migration

In other words, the initial implementation should be an adapter around the current stack, not a refactor of it.

### Suggested Top-Level Interface

One suitable shape is an `IAutonomyBackend` contract:

- `describeCapabilities()`
- `configure(mission_profile, policy_profile)`
- `start(session_request)`
- `pushState(state_update)`
- `pushIntent(intent_update)`
- `step()`
- `pullCommands()`
- `pushCommandResult(command_result)`
- `getStatus()`
- `requestStop(stop_mode)`
- `readSnapshot()`

This is intentionally broader than `IPlannerBackend` or `IExecutionBackend`. It treats the whole autonomy stack as a single service boundary.

### Left-Hand Wrapper: State and Intent Ingress

The ingress side should normalise everything the autonomy system needs to know, without exposing internal storage or planning details.

Recommended ingress objects:

- `StateSnapshot`
  - authoritative snapshot id
  - facts / numeric values / object identities as available
  - source tags and timestamp
- `MissionIntent`
  - goals, priorities, constraints, admissible action set
- `PolicyEnvelope`
  - retry budget, timing budget, safety mode, escalation policy
- `ExternalEvent`
  - perception update, operator override, subsystem fault, communication loss

For the current AME implementation, an adapter can translate these inputs into:

- `WorldModel` updates
- goal fluent updates
- existing replanning triggers

without changing how the planner or BT executor behave internally.

### Right-Hand Wrapper: Action and Outcome Egress

The egress side should normalise what the autonomy system commits to the outside world.

Recommended egress objects:

- `GoalDispatch`
  - agent assignment + delegated goals for a subordinate agent/backend
- `ActionCommand`
  - runtime command emitted by the executing backend, e.g. a PYRAMID service call
- `CommandBatch`
  - one or more commands ready for dispatch
- `CommandResult`
  - command id
  - terminal or intermediate status
  - observed effects / result payload
  - failure classification
- `DispatchResult`
  - completion/failure outcome for a delegated goal dispatch
- `DecisionRecord`
  - why the dispatch or command batch was issued
  - plan id / policy rule / backend id / confidence

For the current AME implementation, this can be realised by:

- emitting `GoalDispatch` artefacts when operating in leader/delegation mode
- emitting `ActionCommand` artefacts by intercepting live `InvokeService` / `IPyramidService` calls when operating in local execution mode

### Internal Mapping for the Current AME Stack

The current system can be wrapped with minimal intrusion:

- ingress adapter writes into `WorldModel` and mission-goal state
- existing planner produces its usual plan
- existing compiler builds its usual BT
- existing executor runs its usual tick loop
- egress adapter converts internal allocation and execution lifecycle into:
  - `GoalDispatch` / `DispatchResult`
  - `ActionCommand` / `CommandResult`
  - `DecisionRecord`

This means the first "whole-system swap surface" can be added **without changing planner, compiler, or executor internals**, only by introducing boundary adapters around them.

### Why This Surface Is Useful

This wrapper buys three forms of pluggability immediately:

1. **Current AME as one backend**
   - the present stack becomes one implementation behind the shell
2. **Alternative autonomy stack as another backend**
   - a reactive or temporal system can implement the same shell later
3. **Stable system integration boundary**
   - ROS2 nodes, operators, simulators, and external services can integrate once against the shell instead of binding to planner- or BT-specific APIs

### Non-Goals of the Wrapper

This surface should not try to standardise every internal abstraction on day one.

It should not require:

- immediate replacement of `WorldModel`
- immediate replacement of `MissionExecutor`
- forcing all backends to expose plans when they are naturally reactive
- premature unification of every internal action-node API

The wrapper is a **swap surface**, not an internal architecture rewrite.

### Recommended Naming and Layering

A practical layering would be:

- `IAutonomyBackend`
  - whole-system black-box contract
- `IStateIngress`
  - optional helper for state / intent normalisation
- `ICommandEgress`
  - optional helper for command dispatch / result return
- `CurrentAmeBackendAdapter`
  - wraps today's `WorldModel + Planner + PlanCompiler + MissionExecutor`

This keeps the planner/executor pluggability work useful, but makes it subordinate to a higher-level replaceable backend boundary.

### Recommended First Implementation

The lowest-risk first implementation is:

1. Introduce `CurrentAmeBackendAdapter` around the existing stack.
2. Feed it state snapshots and mission intent through ingress DTOs.
3. Expose emitted commands, command results, and telemetry through egress DTOs.
4. Leave internal solver, compiler, and BT execution semantics untouched.

That creates the system swap surface you asked for: a wrapper that spans both ends, preserves current internals, and still makes the whole autonomy loop replaceable later.

## Temporal Planner Adoption: Specific Tweaks

To align with the temporal extension research, prioritise:

1. **Capability-gated backend selection**
   - if domain requires `:durative-actions`, route to temporal-capable backend
2. **Temporal Plan IR schema additions**
   - explicit intervals, constraints, and invariants
3. **Plan validation abstraction**
   - backend-native validator + optional VAL post-check through unified interface
4. **Execution-time temporal monitor hook**
   - invariant checking and schedule conformance as reusable execution services
5. **Fallback strategy definition**
   - clear behaviour when temporal solve fails (e.g. fail-fast vs degrade to simplified classical surrogate if allowed)

## Alternative Execution Backend Adoption: Specific Tweaks

For non-BT execution engines, the key changes are:

1. **Action adapter boundary hardening**
   - keep ActionRegistry contracts independent of BT node APIs
2. **Control semantics normalisation**
   - retries, backoff, and preemption as policy-level primitives
3. **Compiler split**
   - `PlanIR -> BT`, `PlanIR -> FSM`, `PlanIR -> DAG` compilers as separate modules
4. **Conformance test suite**
    - any backend must pass the same scenario pack and produce equivalent mission outcomes

## Beyond Plan-Based Execution: Instantaneous and Continual Decisions

The current architecture is naturally plan-centric, but some missions may be better served by more immediate control styles:

- **next-action selection**
  - observe current state, choose one action, execute, re-observe, repeat
- **rolling-horizon planning**
  - produce a short horizon plan, commit only the near prefix, then update continuously
- **event-driven policy execution**
  - react to triggers and guards without materialising a full mission plan
- **mixed-mode control**
  - use classical planning for long-horizon structure and reactive policies for local contingencies

Architecturally, these modes suggest that `PlanIR` should not be the only legal decision artefact. The broader `DecisionResult` contract should permit both planned and more instantaneous outputs.

## Backward-Compatible Migration Plan

### Phase A: System Contract Definition (low risk)

- define the top-level decision-system contract and artefact taxonomy
- define backend-neutral observability fields for decision, action, and outcome records
- keep the current LAPKT + BT stack as the reference implementation of the contract

### Phase B: Planner/Executor Contract Definition (low risk)

- add `IPlannerBackend` + `IExecutionBackend` interfaces
- add capability descriptors and backend identifiers
- introduce Plan IR types (classical subset only at first)
- keep LAPKT + BT as default implementations behind adapters

### Phase C: Internal Rewiring (medium risk)

- refactor Planner and PlanCompiler to use interfaces/IR
- move recovery policy into mission control layer
- update observability schema with backend-neutral core fields

### Phase D: Temporal Pilot (medium/high risk)

- implement temporal backend plugin (e.g. OPTIC subprocess path)
- extend Plan IR with temporal constraints
- add temporal validation and runtime invariant monitoring

### Phase E: Alternate Executor Pilot (medium risk)

- implement one non-BT executor backend
- run parity and resilience test matrix
- compare observability completeness and recovery performance

### Phase F: Whole-Loop Alternate Controller Pilot (medium/high risk)

- implement one non-plan-centric decision backend, such as a reactive or rolling-horizon controller
- route it through the same governance surface: state snapshot, action contract, policy layer, observability
- compare mission outcomes, explainability, and fallback behaviour against the baseline stack

## Risks and Mitigations

- **Interface over-engineering risk**
  - mitigate by implementing adapters first and proving no-regression on existing demos/tests
- **Decision-contract ambiguity risk**
  - mitigate by explicitly enumerating legal decision artefact types and lifecycle states
- **Semantic drift across backends**
  - mitigate with conformance tests and invariant policy checks
- **Telemetry inconsistency**
  - mitigate with schema contract tests in CI
- **Temporal complexity leakage**
  - mitigate by isolating temporal semantics in IR extension + monitor services
- **Unbounded autonomy substitution risk**
  - mitigate by requiring all alternate controllers to use the same governance and audit surface before they are considered production-capable

## Suggested Acceptance Criteria

A pluggability milestone should be considered complete when:

1. Existing LAPKT + BT flow runs entirely through the new system, planner, and executor contracts with no behaviour regression.
2. Backend capability negotiation and decision artefact type are visible in logs and auditable post-mission.
3. At least one temporal planner plugin can generate and validate a temporal plan via Plan IR.
4. At least one alternative execution backend can execute a shared Plan IR scenario pack.
5. At least one alternate decision-system backend can consume the same world-state contract and emit governed actions without bypassing audit/policy hooks.
6. Recovery policy behaviour is consistent across backends for equivalent failures.

## Conclusion

AME can support temporal planning, alternate execution runtimes, and even whole-loop decision-system substitution with moderate refactoring, provided pluggability is treated as a **contract-and-semantics** problem rather than a direct component swap.

The critical architectural move is to establish a stable governance surface:

- authoritative state
- explicit decision artefacts
- governed action dispatch
- backend-agnostic control policy
- audit-grade observability

Within that frame, LAPKT + BT remains the current reference implementation, but it becomes one instance of a broader AME autonomy architecture rather than the only shape the system can ever take.
