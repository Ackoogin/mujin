# AME Core High-Level Requirements

Requirements for `ame_core`, the Autonomous Mission Engine planning-and-execution
library: a PDDL planning + Behaviour Tree execution pipeline built around a single
authoritative world model.

`ame_core` takes a formal mission description (PDDL domain + problem), solves it with
a classical STRIPS planner, compiles the resulting plan into an executable Behaviour
Tree, and runs that tree with automatic replan-on-failure — all behind a
middleware-agnostic C++ boundary. A sink-based observability stack provides full
causal traceability, and optional neuro-symbolic seams allow neural guidance without
weakening the symbolic guarantee.

This document is the core-engine companion to the node-layer
[`HLR.md`](HLR.md). That document specifies the ROS2 adapter nodes and explicitly
excludes the core planning, compilation, and execution algorithms; this document
specifies exactly those algorithms and the world-model, observability, and
neuro-symbolic seams they depend on. It is an **initial** baseline intended to
seed formalisation; requirement identifiers and groupings are expected to be
refined as formalisation proceeds.

## Subject Matter

The subject matter of `ame_core` is authoritative mission-state management, classical
planning over that state, deterministic compilation of plans into Behaviour Trees,
tick-driven execution with recovery, and observability of the whole pipeline.

**Exclusions:**

- Middleware, transport, and lifecycle deployment concerns (specified by PCL and the
  AME node layer — see [`HLR.md`](HLR.md)).
- PDDL language design; `ame_core` consumes STRIPS-level PDDL, it does not define the
  language.
- The internal search algorithm of the underlying LAPKT solver, treated here as a
  trusted dependency invoked through a stable projection.
- Domain authoring and the specific mission semantics of any one domain (e.g.
  `uav_search`, `vehicle_autonomy`, `mission_autonomy`).
- Perception algorithms; `ame_core` defines only the boundary at which perceived
  facts enter the world model.
- Visualisation transport (Foxglove WebSocket framing lives in the separate
  `ame_foxglove` library).

## PDDL Terminology Overview

PDDL (Planning Domain Definition Language) describes planning problems using a
symbolic vocabulary. In this document, the following terms are used in their
PDDL/STRIPS sense:

- **Domain**: the reusable planning model: types, predicates, and action schemas
  such as `move`, `search`, or `classify`.
- **Problem**: one concrete planning instance for a domain: the objects present,
  the initial state, and the goal state.
- **Type**: a category of object, such as `robot`, `location`, or `sector`, used
  to constrain which objects may bind to action and predicate parameters.
- **Object**: a named instance in the problem, such as `uav1`, `base`, or
  `sector_a`.
- **Predicate**: a boolean relation template, such as `(at ?robot ?location)`.
  Predicates define the vocabulary of possible state facts.
- **Grounding**: replacing typed parameters with concrete objects. Grounding
  `(at ?robot ?location)` over `uav1` and `base` produces `(at uav1 base)`.
- **Fluent**: one grounded boolean fact whose truth value may change over time,
  such as `(at uav1 base)` or `(searched sector_a)`. `ame_core` stores fluents
  in the `WorldModel` and assigns each one a stable numeric fluent index.
- **Initial facts**: the set of fluents that are true at the start of a problem.
  In `ame_core`, these load as `BELIEVED` state unless a caller explicitly
  supplies another authority.
- **Goal fluents**: the fluents the planner must make true for the mission to
  succeed.
- **Action schema**: a parameterised action definition containing preconditions
  and effects, such as a generic `move(?robot, ?from, ?to)`.
- **Ground action**: a concrete action instance produced from an action schema,
  such as `move(uav1,base,sector_a)`.
- **Precondition**: a fluent that must be true before an action can run.
- **Add effect / delete effect**: fluents made true or false by a completed
  action. In execution, plan effects are predictions and are written as
  `BELIEVED` state, while perception writes `CONFIRMED` state.

## Design Decisions

### D1 - Single Source of Truth
The `WorldModel` owns all mission state. The Behaviour Tree blackboard is a read-only
projection, and the planner receives an immutable snapshot projection. No component
holds an authoritative copy of state outside the `WorldModel`.

**Why**: Consistency, auditability, and correct replanning all require exactly one
authoritative state store.

### D2 - Middleware-Agnostic Core
`ame_core` depends only on BehaviorTree.CPP and the LAPKT core; it has no ROS2,
DDS, socket, or other transport dependency. Optional capabilities (Foxglove, neuro)
live in separate libraries guarded by compile flags.

**Why**: The same autonomy logic must run in local demos, embedded targets, and
distributed deployments without source changes.

### D3 - Planning Model and Execution Model Are Independently Replaceable
The planning model (PDDL predicates, types, action schemas) and the execution model
(BT node implementations) are bridged only through the `ActionRegistry`. Either side
can change without the other.

**Why**: Decoupling the symbolic model from its physical realisation keeps both
independently testable and reusable.

### D4 - Symbolic Determinism
Given identical world state and goals, the planner produces an identical plan, and
the compiler produces identical BT XML. Optional neural seams default to no-ops and
must not alter this baseline.

**Why**: Reproducibility is a precondition for assurance, audit, and regression
testing of safety-relevant autonomy.

### D5 - Fail-Closed Error Model
On action failure the engine does not apply the plan's predicted effects. Ground
truth comes only from perception writing to the `WorldModel`. Permission and safety
gates are modelled as domain preconditions surfaced as guard leaves, so they fail
closed by construction.

**Why**: The engine must never trust the plan's model of reality over observed
reality, and unsafe actions must be blocked unless explicitly authorised.

### D6 - Recovery by Replanning
An action failure triggers replanning from the current authoritative state, bounded
by a configurable retry policy — it does not abort the mission.

**Why**: Autonomous missions must tolerate local failures and adapt rather than
terminate.

### D7 - Sink-Based Observability
All observability is callback-driven, composable, and non-blocking. Multiple sinks
may attach simultaneously, and the streams are cross-correlatable into a single
causal timeline.

**Why**: Planning and execution must be fully diagnosable and auditable after the
fact without perturbing real-time execution.

### D8 - Eager Grounding with Stable Fluent Indices
Predicates are ground eagerly when objects are added, and fluent indices, once
assigned, never change.

**Why**: Stable indices make STRIPS projection a direct mapping and allow repeated
replanning without rebuilding the planning problem.

### D9 - Thread-Safe Authoritative State with Provenance
The `WorldModel` supports concurrent readers (BT ticks) and writers (perception)
through immutable snapshots and reader/writer locking, and every fact carries
authority and source provenance.

**Why**: Asynchronous perception must update state safely without breaking the
deterministic single-threaded tick, and consumers must be able to tell believed
state from confirmed state.

### D10 - Propose-Verify-Fallback Neural Integration
Neural guidance enters only through verified seams that fall back to the symbolic
baseline on rejection, timeout, or absence.

**Why**: Neural components may improve performance but must never be able to violate
the symbolic safety and determinism guarantees.

---

## Detailed Requirements

## Type System and Objects

### AMC.001 - Single-Inheritance Type Hierarchy
The engine shall maintain a single-inheritance type hierarchy mapping each type to a
single parent type up to a root type.

**Rationale**: PDDL `:typing` requires a hierarchy to validate action parameter
binding.

### AMC.002 - Object Registry
The engine shall maintain an object registry mapping each object name to exactly one
declared type, with type membership validated at grounding time.

**Rationale**: Grounded actions and predicates must bind only type-compatible
objects.

### AMC.003 - Predicate Registration
The engine shall allow registration of boolean predicates with an ordered list of
parameter types.

**Rationale**: Predicates define the state vocabulary over which planning operates.

## World Model State

### AMC.004 - Authoritative Boolean State Store
The `WorldModel` shall store all mission state as grounded boolean facts in a compact
bitset, and shall be the single authoritative owner of that state.

**Rationale**: A single compact source of truth is required for consistency,
efficiency, and auditability (D1).

### AMC.005 - Eager Grounding
When an object is added, the engine shall immediately ground every predicate that can
bind to the currently registered object set, assigning each newly discovered fact a
fluent index. For multi-argument predicates this grounding shall cover the complete
compatible object cross-product and shall skip duplicate fluent keys.

**Rationale**: Eager grounding makes STRIPS projection a direct mapping (D8).

### AMC.006 - Stable Fluent Indices
Once assigned, a fluent index shall never change for the lifetime of the world model.

**Rationale**: Stable indices allow repeated replanning without rebuilding the
problem (D8).

### AMC.007 - Fact Access by Index, Identifier, and Key
The engine shall support reading and writing a fact by fluent index, by
predicate/argument identifiers, and by string key (e.g. `"at(uav1,sectorA)"`), with a
stable bidirectional mapping between fluent index and string key.

**Rationale**: The planner projection, the action layer, and the BT nodes each
require a different addressing mode for the same underlying fact.

### AMC.008 - Monotonic Version Counter
The `WorldModel` shall expose a monotonic version counter that increments on every
state change.

**Rationale**: Consumers must be able to reason about state freshness and order
fact changes against BT events (D7).

## State Authority and Perception

### AMC.009 - Fact Authority Tagging
Every fact shall carry an authority of `BELIEVED` (default, model-loaded, or
plan-predicted state) or `CONFIRMED` (observed by perception), together with a
source tag and timestamp. Newly grounded facts and facts written without an explicit
authority shall default to `BELIEVED`.

**Rationale**: Believed and confirmed state must be distinguishable for correct
recovery and audit (D9).

### AMC.010 - Plan Effects Are Believed
Facts written by `SetWorldPredicate` execution nodes, by default `setFact` calls,
and by PDDL initial-state loading shall use `BELIEVED` authority unless a caller
explicitly supplies another authority. `SetWorldPredicate` writes shall be
attributed to the writing node.

**Rationale**: Plan-applied effects are predictions, not observations (D5, D9).

### AMC.011 - Perception Is Confirmed
Facts written through the perception ingestion boundary shall be recorded with
`CONFIRMED` authority and a perception source tag.

**Rationale**: Observed ground truth is the only authority allowed to override plan
predictions (D5).

### AMC.012 - Authority Conflict Detection
The engine shall expose a query that reports when a perceived value disagrees with
the currently believed value of a fact.

**Rationale**: Disagreement between prediction and observation must be surfaceable
for recovery and diagnosis (D5, D9).

### AMC.013 - Batched Perception Mutation Queue
The engine shall provide a mutation queue allowing external (perception) threads to
enqueue fact updates without taking the world-state write lock, and a batch-apply
operation that atomically commits all queued mutations.

**Rationale**: Asynchronous perception must update state without contending with or
disordering the deterministic tick (D9).

## Concurrency and Snapshots

### AMC.014 - Immutable Read Snapshots
The engine shall provide an immutable, reference-counted snapshot of world state for
lock-safe reads spanning a complete BT tick.

**Rationale**: BT nodes must observe a stable state for the duration of a tick even
while perception writes concurrently (D9).

### AMC.015 - Reader/Writer Concurrency Safety
Live world-state access shall be protected by reader/writer locking, and the
perception mutation queue shall be protected by a separate lock.

**Rationale**: Concurrent BT readers and perception writers must not corrupt state
or each other (D9).

### AMC.016 - Read-Only Blackboard Projection
The engine shall push world state to the BT blackboard as a one-way, read-only
projection; the blackboard shall never be an authoritative state source.

**Rationale**: Preserves the single-source-of-truth invariant while giving BT nodes
convenient state access (D1).

## PDDL Parsing and Domain Loading

### AMC.017 - STRIPS-Level PDDL Parsing
The engine shall parse STRIPS-level PDDL (`:typing`, `:strips`) domain and problem
inputs into a `WorldModel`, populating types, objects, predicates, initial facts, and
goal fluents.

**Rationale**: PDDL is the formal mission-description input to the pipeline (D3).

### AMC.018 - File and String Loading Paths
The engine shall support loading a domain and problem both from file paths and from
in-memory string content.

**Rationale**: File loading serves deployment; string loading serves
development, testing, and runtime model injection.

### AMC.019 - Reported Grounding Results
After parsing, the engine shall make the resulting grounded fluent and action counts
available to the caller.

**Rationale**: Callers need to confirm that a loaded model grounded as expected.

### AMC.020 - Subset-Domain Snapshotting
The engine shall support projecting only the predicates relevant to a given domain
when snapshotting world state for planning, so that a planner operating on a subset
domain ignores facts outside its vocabulary.

**Rationale**: Multi-domain deployments require each planner to plan over its own
predicate subset against a shared union state (D3).

## Planning

### AMC.021 - Stateless STRIPS Solve
The planner shall be stateless: it shall build a STRIPS problem by projecting the
current `WorldModel`, solve it, and hold no state between calls.

**Rationale**: Statelessness underpins determinism and safe repeated replanning
(D4).

### AMC.022 - Deterministic Plans
Given identical projected state, goals, and action ordering, the planner shall return
an identical plan.

**Rationale**: Reproducibility is required for assurance and regression testing
(D4).

### AMC.023 - Plan as Action Indices
The planner shall return plan steps as indices into the world model's grounded action
set.

**Rationale**: Index-based steps map directly back to grounded actions for
compilation.

### AMC.024 - Goal Requirement
The planner shall require at least one goal fluent and shall report failure for a
request with no goal.

**Rationale**: Solving without a goal is not meaningful.

### AMC.025 - Solve Metadata
The planner shall report, at minimum, success/failure, solve time, and the ordered
plan, and shall make solver/search metadata available for audit.

**Rationale**: Callers need both the executable result and diagnostic metadata (D7).

### AMC.026 - No-Solution Reporting
When no plan exists for the given state and goals, the planner shall report failure
explicitly rather than returning an empty or fabricated plan.

**Rationale**: Silent or fabricated success is unacceptable in autonomy planning
(D5).

## Action Registry

### AMC.027 - Action Name to Implementation Mapping
The engine shall map each PDDL action name to a Behaviour Tree implementation through
the `ActionRegistry`.

**Rationale**: The registry is the sole bridge between planning and execution models
(D3).

### AMC.028 - Three Implementation Levels
The registry shall support simple (single BT node type), template (parameterised
sub-tree XML with positional parameter substitution), and pre-authored (BT XML file)
action implementations.

**Rationale**: Action complexity ranges from a single node to a hand-crafted
reactive sub-tree.

### AMC.029 - Per-Action Reactivity Flag
The registry shall record, per action, whether the compiled action unit re-checks
preconditions every tick (reactive) or only once at start.

**Rationale**: Some actions require continuous precondition monitoring; others do
not.

### AMC.030 - Action Resolution
The registry shall resolve an action name plus ordered parameters into a concrete BT
implementation fragment.

**Rationale**: Compilation needs concrete BT fragments per grounded plan step.

## Plan Compilation

### AMC.031 - Causal Dependency Graph
The compiler shall build a causal dependency graph over plan steps, adding an edge
from an earlier step to a later step when an add-effect of the earlier is a
precondition of the later, when a delete-effect of the earlier removes a
precondition of the later, or when the later deletes a fluent added by the earlier.
These conflict edges shall prevent the affected steps from being emitted as
independent parallel flows.

**Rationale**: Ordering and parallelism must be derived from causal structure, not
assumed from plan order.

### AMC.032 - Parallel Execution Phase Extraction
The compiler shall topologically order the graph into execution phases. Each phase
shall contain the currently-ready steps whose causal predecessors have already been
scheduled.

**Rationale**: Independent ready steps can execute concurrently while conservative
phase barriers preserve causal order.

### AMC.033 - Action Unit Generation
Each plan step shall compile to an action unit that checks its preconditions,
executes the resolved BT fragment, and applies its add and delete effects as world
predicate writes, using a sequence or reactive sequence per the action's reactivity
flag.

**Rationale**: Precondition guarding and effect application make each step
self-contained and verifiable (D5).

### AMC.034 - Tree Composition
The compiler shall compose execution phases under a sequence. Single-step phases
shall be emitted directly, and multi-step phases shall be emitted under a parallel
node requiring all phase children to succeed.

**Rationale**: Correct composition preserves causal order while exposing available
parallelism.

### AMC.035 - Loadable BT XML Output
The compiler shall emit BT XML loadable by the Behaviour Tree factory, and shall
offer a sequential fallback composition for debugging.

**Rationale**: The compiler output is the planner-to-executor contract.

### AMC.036 - Deterministic Compilation
Given an identical plan and registry, the compiler shall emit identical BT XML.

**Rationale**: Determinism extends through the whole pipeline, not just planning
(D4).

## Behaviour Tree Nodes

### AMC.037 - World Predicate Check Node
The engine shall provide a condition node that reads a fact by key from the
`WorldModel` and returns success or failure accordingly.

**Rationale**: Compiled precondition guards depend on this node.

### AMC.038 - World Predicate Set Node
The engine shall provide an action node that writes a fact to the `WorldModel` with
`BELIEVED` authority attributed to the node.

**Rationale**: Compiled effect application depends on this node while preserving the
distinction between predicted and observed state (D1, D5, D9).

### AMC.039 - Hierarchical Phase Node
The engine shall provide a node that runs a full plan→compile→execute cycle for a
sub-goal set, enabling hierarchical decomposition within a parent tree, and that
links its planning episode to its parent episode for audit.

**Rationale**: Large missions decompose into nested planning phases with preserved
traceability (D7).

### AMC.040 - Service Invocation Node
The engine shall provide an asynchronous external-service invocation node behind a
SDK-agnostic interface, with request construction, polling to a terminal status,
cancellation, and a configurable timeout.

**Rationale**: Actions frequently delegate work to external services without
coupling the core to any one SDK (D2).

### AMC.041 - Agent Delegation Node
The engine shall provide a node that plans and runs an agent-scoped subtree, marking
the target agent unavailable for the duration and restoring availability on
completion or halt.

**Rationale**: Multi-agent missions require a leader to delegate scoped work (D3).

### AMC.042 - Guard and Dispatch Leaves
The engine shall provide registrable guard leaves (e.g. authorisation, geofence,
terrain-avoidance) and a dispatch leaf per registered verb that bridges execution to
an external execution sink.

**Rationale**: Permission and safety gates must be enforceable as tree leaves that
fail closed (D5).

### AMC.043 - World Model Injection
All world-model BT nodes shall obtain their `WorldModel` reference through the root
blackboard rather than owning state directly.

**Rationale**: Preserves single source of truth across the tree (D1).

## Execution and Replanning

### AMC.044 - Tick-Driven Execution
The executor shall load a compiled tree and tick it at a configurable rate, reporting
at minimum idle, running, success, and failure status.

**Rationale**: Behaviour Tree execution is periodic and mission-rate dependent.

### AMC.045 - Tree Reload
The executor shall support loading and swapping the compiled tree at runtime.

**Rationale**: Replanning produces a new tree that must replace the running one.

### AMC.046 - Failure Does Not Apply Effects
On action failure the engine shall not apply the failed action's predicted effects to
the `WorldModel`.

**Rationale**: The engine must never record believed state for work that did not
complete (D5).

### AMC.047 - Replan From Current State
When the running tree reports failure, the engine shall snapshot the current
authoritative state (which may have been updated by perception since the failure),
replan from that state, recompile, and resume execution.

**Rationale**: Recovery adapts to observed reality rather than aborting (D6).

### AMC.048 - Bounded Replanning
Replanning shall be bounded by a configurable maximum retry count, after which the
engine shall stop and report failure.

**Rationale**: Unbounded replanning could mask an unrecoverable situation (D6).

### AMC.049 - Status-Driven Recovery Boundary
Replanning shall be driven by observing the executor's published failure status, not
signalled from inside the tree by a dedicated node.

**Rationale**: Keeps the tree free of control-flow coupling to the recovery loop and
keeps the recovery policy in one place (D3).

## Contingency and Safety

### AMC.050 - Positive-Precondition Contingency Domains
The engine shall support contingency planning expressed as pure STRIPS domains with
positive preconditions only, modelling alternative action sets with relaxed
preconditions.

**Rationale**: Contingency ladders (e.g. redundancy fallback, task reallocation)
must be expressible within the symbolic model rather than hardcoded (D3, D5).

### AMC.051 - Safe-State Reachability Verification
The engine shall provide a tool that exhaustively verifies safe-state reachability
across all combinations of context (health) predicates by solving each combination
with the planner.

**Rationale**: Assurance requires demonstrating that a safe state is reachable from
every health configuration (D4, D5).

### AMC.052 - Dominance Pruning
The safe-state verification shall apply monotone dominance pruning to reduce the
number of solver calls while preserving exhaustiveness.

**Rationale**: Exhaustive verification over 2^N health states must remain tractable.

## Observability

### AMC.053 - World Model Audit Callback
Every fact change shall fire an audit callback carrying world-model version,
timestamp, fact key, new value, and source tag.

**Rationale**: Fact-level provenance is the basis of state auditability (D7).

### AMC.054 - Behaviour Tree Event Stream
The engine shall emit a structured event for every BT node state transition,
including timestamp, node identity and type, previous and new status, tree
identifier, and the world-model version at the transition.

**Rationale**: Correlating BT events to world-model versions yields full causal
traceability (D7).

### AMC.055 - Plan Audit Episodes
The engine shall record, per planning episode, a self-contained record containing the
episode and parent-episode identifiers, an optional phase label, the initial-state
snapshot, the goals, the solver and solve time, the ordered plan, and the compiled BT
XML.

**Rationale**: Each planning decision must be independently reconstructable, and
hierarchical missions must form a linked episode tree (D7).

### AMC.056 - Built-in BT Statistics and Tracing
The engine shall support per-node tick/success/failure statistics and exportable
execution traces through the Behaviour Tree library's logging infrastructure.

**Rationale**: Node-level performance and timing must be inspectable without custom
instrumentation (D7).

### AMC.057 - Composable Non-Blocking Sinks
Observability outputs shall be deliverable to multiple simultaneous sinks (e.g. file
and live consumer) through callbacks that do not block execution.

**Rationale**: Logging must never perturb the real-time tick, and multiple consumers
must be able to attach at once (D7).

### AMC.058 - Cross-Stream Correlation
The engine's audit streams shall share timestamps and identifiers sufficient to
reconstruct a single unified timeline across BT events, world-model changes, and
planning episodes.

**Rationale**: Diagnosis of any mission event requires correlating all streams on
one timeline (D7).

## Neuro-Symbolic Seams (Optional, `AME_NEURO`)

### AMC.059 - No-Op By Default
When no neural hook is attached, the engine's behaviour shall be identical to a build
without the neuro layer.

**Rationale**: The symbolic baseline must be preserved byte-for-byte unless a hook is
explicitly installed (D4, D10).

### AMC.060 - Heuristic Hook Seam
The planner shall consult an optional heuristic hook that returns per-action
preference scores, using them only to bias action traversal order; absence of the
hook shall be equivalent to the identity ordering.

**Rationale**: Neural guidance may reorder search but must not change the set of
admissible plans (D4, D10).

### AMC.061 - Repair Hook Seam
The executor shall consult an optional repair hook on failure before the baseline
failure path. The hook shall receive the failed-step index and current world model,
and may return compiled BT XML only after its proposal has been produced through the
registered symbolic planning, compilation, and verification boundary. A non-empty
repair XML result shall be loaded and executed; an empty result, timeout, exception,
or BT loading failure shall fall back to the baseline failure/replanning path without
converting the failure to success.

**Rationale**: Neural repair may shortcut a full replan but must always fall back to
the verified symbolic path (D6, D10).

### AMC.062 - Neural Provenance and Audit
When a neural seam influences a planning episode, the engine shall record the
provenance (e.g. heuristic/goal/repair source) on the plan audit record and emit one
audit record per neural advisory call capturing outcome, latency, retries, verifier
verdict, and evidence.

**Rationale**: Neural influence on a safety-relevant decision must be fully
auditable and verifiable (D7, D10).

## Portability and Boundaries

### AMC.063 - No Middleware Dependency in Core
`ame_core` shall depend only on the Behaviour Tree library and the planner core, with
no ROS2, DDS, socket, or other transport dependency.

**Rationale**: The core must run unchanged across local, embedded, and distributed
deployments (D2).

### AMC.064 - Optional Capabilities Are Separable
Visualisation and neuro-symbolic capabilities shall live in separate libraries
guarded by compile-time flags, leaving `ame_core` buildable and testable without
them.

**Rationale**: Optional, heavier dependencies must not be forced on minimal
deployments (D2).

---

## Design Decision Traceability

| Requirement | Design Decision |
| :--- | :--- |
| `AMC.001` | `D3` |
| `AMC.002` | `D1`, `D3` |
| `AMC.003` | `D3` |
| `AMC.004` | `D1` |
| `AMC.005` | `D8` |
| `AMC.006` | `D8` |
| `AMC.007` | `D1`, `D8` |
| `AMC.008` | `D7`, `D9` |
| `AMC.009` | `D9` |
| `AMC.010` | `D5`, `D9` |
| `AMC.011` | `D5`, `D9` |
| `AMC.012` | `D5`, `D9` |
| `AMC.013` | `D9` |
| `AMC.014` | `D9` |
| `AMC.015` | `D9` |
| `AMC.016` | `D1` |
| `AMC.017` | `D3` |
| `AMC.018` | `D2`, `D3` |
| `AMC.019` | `D7` |
| `AMC.020` | `D1`, `D3` |
| `AMC.021` | `D4` |
| `AMC.022` | `D4` |
| `AMC.023` | `D3`, `D4` |
| `AMC.024` | `D5` |
| `AMC.025` | `D7` |
| `AMC.026` | `D5` |
| `AMC.027` | `D3` |
| `AMC.028` | `D3` |
| `AMC.029` | `D3` |
| `AMC.030` | `D3` |
| `AMC.031` | `D4` |
| `AMC.032` | `D4` |
| `AMC.033` | `D5` |
| `AMC.034` | `D4` |
| `AMC.035` | `D3`, `D4` |
| `AMC.036` | `D4` |
| `AMC.037` | `D1` |
| `AMC.038` | `D1`, `D9` |
| `AMC.039` | `D3`, `D7` |
| `AMC.040` | `D2`, `D3` |
| `AMC.041` | `D3` |
| `AMC.042` | `D5` |
| `AMC.043` | `D1` |
| `AMC.044` | `D6` |
| `AMC.045` | `D6` |
| `AMC.046` | `D5` |
| `AMC.047` | `D5`, `D6` |
| `AMC.048` | `D6` |
| `AMC.049` | `D3`, `D6` |
| `AMC.050` | `D3`, `D5` |
| `AMC.051` | `D4`, `D5` |
| `AMC.052` | `D4` |
| `AMC.053` | `D7` |
| `AMC.054` | `D7` |
| `AMC.055` | `D7` |
| `AMC.056` | `D7` |
| `AMC.057` | `D7` |
| `AMC.058` | `D7` |
| `AMC.059` | `D4`, `D10` |
| `AMC.060` | `D4`, `D10` |
| `AMC.061` | `D6`, `D10` |
| `AMC.062` | `D7`, `D10` |
| `AMC.063` | `D2` |
| `AMC.064` | `D2` |

## Implementation Notes

- Treat the `WorldModel` as the single authoritative state boundary; every other
  component reads a projection or snapshot of it.
- Treat the `ActionRegistry` as the only sanctioned bridge between the PDDL planning
  model and the BT execution model — new behaviours are added there, not by branching
  on action names in the compiler or executor.
- Treat the planner and compiler as deterministic, stateless transforms; any source
  of non-determinism is a defect against `AMC.022` / `AMC.036`.
- Treat replanning as a recovery policy owned by the autonomy backend, observed from
  executor status rather than signalled from inside the tree.
- Treat every neuro-symbolic seam as additive: the symbolic baseline must remain the
  fallback and must be reproducible with the seam disabled.
- This baseline is expected to be decomposed into low-level requirements (LLR)
  during formalisation, mirroring the PCL `HLR.md` / `LLR.md` split.
