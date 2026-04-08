# AME Planning/Execution Pluggability Review

## Purpose

This review identifies architectural and interface changes that would improve pluggability of the AME planning and execution pipeline, with emphasis on:

- introducing a temporal planner backend (as evaluated in `../research/temporal_extension_research.md`)
- supporting alternative execution backends beyond the current BehaviourTree.CPP runtime
- preserving current strengths: symbolic authority, observability, and safe fallback behaviour

Primary reference documents:

- `../architecture/01-overview.md`
- `../architecture/03-planning.md`
- `../architecture/04-execution.md`
- `../architecture/05-observability.md`
- `../architecture/07-extensions.md`
- `../research/temporal_extension_research.md`
- `../TODO.md`

## Executive Summary

AME is already close to being pluggable at a conceptual level, but several interfaces are still implicit or coupled to concrete implementations (LAPKT for planning, BT.CPP for execution).

The highest-impact improvements are:

1. **Stabilise planner and executor contracts** with explicit capability metadata.
2. **Introduce an internal intermediate representation (IR)** between planner output and execution compilation.
3. **Separate control policy from runtime engine** so replanning, retry, and mission abort logic remain backend-agnostic.
4. **Promote observability to a contract requirement**, not a backend-specific optional feature.

A phased approach can deliver this without destabilising the current system: first define interfaces and adapters, then migrate existing LAPKT + BT flow onto those interfaces, then add temporal and alternative runtime backends.

## Current Coupling Points

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

## 1) Planner Backend Interface + Capability Model

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

## 2) Plan Intermediate Representation (Plan IR)

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

## 3) Executor Backend Interface

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

## 4) Mission Control Policy Layer

Move replan/escalation decisions to a backend-agnostic control layer.

Policy concerns that should be engine-independent:

- retries vs local repair vs full replan
- backend switching on capability mismatch or repeated failure
- safety-mode transition and abort thresholds
- policy budgets (max replans, max wall-clock, critical action deadlines)

This enables consistent behaviour even when swapping planner/executor technologies.

## 5) Observability Contract for Pluggable Components

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

## Backward-Compatible Migration Plan

### Phase A: Contract Definition (low risk)

- add `IPlannerBackend` + `IExecutionBackend` interfaces
- add capability descriptors and backend identifiers
- introduce Plan IR types (classical subset only at first)
- keep LAPKT + BT as default implementations behind adapters

### Phase B: Internal Rewiring (medium risk)

- refactor Planner and PlanCompiler to use interfaces/IR
- move recovery policy into mission control layer
- update observability schema with backend-neutral core fields

### Phase C: Temporal Pilot (medium/high risk)

- implement temporal backend plugin (e.g. OPTIC subprocess path)
- extend Plan IR with temporal constraints
- add temporal validation and runtime invariant monitoring

### Phase D: Alternate Executor Pilot (medium risk)

- implement one non-BT executor backend
- run parity and resilience test matrix
- compare observability completeness and recovery performance

## Risks and Mitigations

- **Interface over-engineering risk**
  - mitigate by implementing adapters first and proving no-regression on existing demos/tests
- **Semantic drift across backends**
  - mitigate with conformance tests and invariant policy checks
- **Telemetry inconsistency**
  - mitigate with schema contract tests in CI
- **Temporal complexity leakage**
  - mitigate by isolating temporal semantics in IR extension + monitor services

## Suggested Acceptance Criteria

A pluggability milestone should be considered complete when:

1. Existing LAPKT + BT flow runs entirely through the new interfaces with no behaviour regression.
2. Backend capability negotiation is visible in logs and auditable post-mission.
3. At least one temporal planner plugin can generate and validate a temporal plan via Plan IR.
4. At least one alternative execution backend can execute a shared Plan IR scenario pack.
5. Recovery policy behaviour is consistent across backends for equivalent failures.

## Conclusion

AME can support temporal planning and alternate execution runtimes with moderate refactoring, provided pluggability is treated as a **contract-and-semantics** problem rather than a direct component swap.

The critical architectural move is to establish stable interfaces and a backend-neutral plan representation while preserving symbolic verification and audit-grade observability.
