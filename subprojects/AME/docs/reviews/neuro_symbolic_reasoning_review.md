# Neuro-Symbolic Reasoning Review Report

## Purpose

This report reviews the neuro-symbolic reasoning approaches described in the project documentation and evaluates them against the repository's current architecture, assurance model, and roadmap maturity.

Primary source documents:

- `subprojects/AME/docs/research/neuro_symbolic_reasoning.md`
- `doc/concept.md`
- `doc/extensions.md`
- `subprojects/AME/docs/autonomy_assurance_plan.md`
- `README.md`
- `subprojects/AME/docs/stakeholder_summary.md`

## Executive Summary

The documentation presents a coherent neuro-symbolic strategy **if the symbolic stack remains authoritative** and neural components stay inside a strict **propose-verify-fallback** envelope.

The current repository is clearly documented as:

- symbolic-first
- deterministic in its planning core
- auditable through a 5-layer observability stack
- currently ML-free in the implemented baseline

Within that context, the documented neuro-symbolic options fall into three groups:

1. **Low-risk, near-term integrations**
   - **Option E: LLM Goal Interpreter**
   - **Option F: LLM Mission Analyst**

2. **Medium-risk, planner-adjacent integrations**
   - **Option A: LLM Heuristic Guide**
   - **Option B: LLM Plan Repair**

3. **Higher-burden or infrastructure-heavy integrations**
   - **Option D: Learned Heuristic**
   - **Option G: Neural Anomaly Detector**
   - **Option C: Neural PDDL Domain Authoring**

Overall judgement:

- The documentation is strongest when it treats neural components as **advisory helpers** and symbolic components as **the final arbiter of correctness**.
- The most credible adoption path is to start with **goal interpretation** and **post-mission analysis**, because both can deliver value without entering the hard real-time planning path.
- The main weaknesses are not in the core idea, but in documentation alignment and system pre-requisites: roadmap consistency, state-authority semantics, safe-state maturity, and validation/test depth for future ML components.

## Current Baseline the Review Assumes

Across the documentation, the implemented baseline is:

- `WorldModel` is the single authoritative state store
- planning is classical STRIPS search using LAPKT
- plans are compiled into executable Behaviour Trees
- execution supports replan-on-failure
- observability layers record planning episodes, BT events, and world-state changes

This matters because every neuro-symbolic proposal is constrained by those invariants. Any future neural capability that breaks determinism, traceability, or verifiability must be explicitly justified.

## Architecture Constraints Any Neural Extension Must Respect

The reviewed documents consistently imply the following design constraints:

### 1. Symbolic authority must remain intact

Neural components may propose:

- goal fluents
- action orderings
- repair candidates
- explanations
- anomaly alerts

But symbolic components must verify:

- whether goals exist in the grounded model
- whether actions satisfy PDDL preconditions and effects
- whether repair suffixes actually reach the goal
- whether operational fallbacks remain valid

### 2. Fallback to pure symbolic behaviour is mandatory

The strongest design pattern in the documentation is graceful degradation:

- if a neural component times out, fails, or is unavailable
- the system should continue with the existing symbolic path
- no neural component should become a single point of mission failure

### 3. Observability is a first-class dependency

The mission analyst, learned heuristic, and anomaly detector all depend on the project's existing audit trail. This is a strength: the repository already has the logging surface required to support later analysis and training.

### 4. The current state model is still boolean STRIPS-centric

Several future neural ideas assume richer operational context, but the documented implemented baseline is still centered on:

- boolean fluents
- static object sets
- non-temporal planning
- no numeric fluents
- no conditional effects

This limits what a neural component can sensibly infer or optimize without first extending the symbolic substrate.

## Review Criteria

Each option below is evaluated on:

- **Value**: likely operational or engineering payoff
- **Safety burden**: how much assurance overhead it introduces
- **Runtime criticality**: how close it sits to mission execution
- **Validation strength**: how clearly the docs describe a symbolic verification path
- **Readiness**: how well it fits the current codebase and roadmap

## Approach Review

### Option A - LLM Heuristic Guide

**Role**: Improve search efficiency by ranking actions or biasing planner expansion order while leaving LAPKT responsible for plan validity.

**Assessment**

- **Value**: High in larger grounded domains
- **Safety burden**: Moderate
- **Runtime criticality**: Medium to high, because it touches planning latency
- **Validation strength**: Strong, because symbolic search still checks action semantics
- **Readiness**: Medium

**Why it is promising**

- Preserves solver soundness
- Fits the existing planner abstraction cleanly
- Can be logged alongside the plan audit trail
- Degrades gracefully to default search ordering

**Main concerns**

- LLM latency may dominate planning in domains that currently solve quickly
- The document assumes a practical tie-breaking or heuristic injection point in the planner that still needs careful implementation
- Benefits are likely domain-sensitive rather than universal

**Review judgement**

This is a good medium-term neuro-symbolic option, but only after the project defines:

- strict timeout budgets
- deterministic fallback behaviour
- logging for all proposed scores and whether they affected search
- a benchmark harness showing clear win over pure symbolic planning

### Option B - LLM Plan Repair

**Role**: Suggest a repair suffix after execution failure instead of always replanning from scratch.

**Assessment**

- **Value**: Potentially high in large or failure-prone domains
- **Safety burden**: Moderate to high
- **Runtime criticality**: High during recovery
- **Validation strength**: Strong in principle, because forward simulation can reject invalid repairs
- **Readiness**: Medium to low

**Why it is promising**

- Uses the audit trail in a meaningful way
- May recover faster than full replanning
- Matches the architecture's replan-on-failure model

**Main concerns**

- Recovery time can get worse if the LLM repeatedly proposes unusable repairs
- Repair validation depends on a sufficiently faithful simulation of current symbolic state
- The docs do not yet fully resolve the state-authority question between BT-applied effects and perception-confirmed truth

**Review judgement**

Conceptually sound, but not yet a near-term first step. It should follow:

- clear semantics for when world-state changes are considered confirmed
- bounded retry policy before falling back to classical replanning
- explicit tests for adversarial or irrelevant repair suggestions

### Option C - Neural PDDL Domain Authoring

**Role**: Generate or extend formal planning models from natural-language mission descriptions.

**Assessment**

- **Value**: High for engineering productivity and mission setup
- **Safety burden**: High
- **Runtime criticality**: Low, because it is offline
- **Validation strength**: Partial
- **Readiness**: Low

**Why it is promising**

- Could reduce the barrier to creating new mission domains
- Aligns well with the existing parser and formal domain structure
- Is naturally separated from the execution path

**Main concerns**

- Syntax validation is not enough; the larger problem is semantic correctness
- Generated domains may appear plausible while encoding unsafe or incomplete behaviour
- Governance, review, and traceability requirements are much heavier here than for advisory runtime assistants

**Review judgement**

Useful as an offline authoring assistant, but it should be treated as a documentation and engineering tool, not an autonomous mission-authoring capability. This belongs late in the roadmap after validation workflows are mature.

### Option D - Learned Heuristic

**Role**: Replace or augment hand-selected planner heuristics with a compact trained model.

**Assessment**

- **Value**: High if search performance becomes a real bottleneck
- **Safety burden**: Moderate
- **Runtime criticality**: High for planning performance, lower than an online LLM from a latency perspective
- **Validation strength**: Moderate
- **Readiness**: Medium to low

**Why it is promising**

- Better runtime characteristics than cloud LLM calls
- Can exploit the existing plan audit trail as training data
- Fits the symbolic-authoritative pattern if used only to prioritize search

**Main concerns**

- Requires enough representative solved episodes to train on
- Risks domain overfitting
- Introduces ML lifecycle obligations: data provenance, versioning, monitoring, and drift management

**Review judgement**

This is likely the best production-grade optimization path once there is sufficient mission data. It is more operationally credible than putting a general-purpose LLM directly in the planner hot path.

### Option E - LLM Goal Interpreter

**Role**: Translate operator language into grounded symbolic goal fluents.

**Assessment**

- **Value**: High user-facing benefit
- **Safety burden**: Low to moderate
- **Runtime criticality**: Low to medium
- **Validation strength**: Very strong
- **Readiness**: High

**Why it is promising**

- The symbolic validation story is simple: goals either map to valid grounded fluents or they do not
- It improves usability without changing planner correctness
- Failure handling is straightforward: reject, clarify, or fall back to manual goal specification

**Main concerns**

- Ambiguous operator language can still produce the wrong valid goal set
- Governance is needed for goal authorization and allowed-goal filtering

**Review judgement**

This is the strongest near-term option in the documentation. It offers clear value with the smallest architectural and assurance disruption.

### Option F - LLM Mission Analyst

**Role**: Analyze observability outputs for summaries, explanations, anomaly review, and operator Q and A.

**Assessment**

- **Value**: High
- **Safety burden**: Low if kept out of closed-loop control
- **Runtime criticality**: Low in batch mode
- **Validation strength**: Moderate
- **Readiness**: High for batch, medium for streaming

**Why it is promising**

- Leverages documentation and logging that already exist
- Delivers immediate operator and post-incident value
- Can be deployed entirely outside the control loop

**Main concerns**

- Requires retrieval and windowing over large logs
- Explanations can sound convincing without being fully grounded unless evidence citation is enforced
- Real-time alerting is harder than post-mission analysis because of latency and context management

**Review judgement**

This is the best companion to Option E for an initial neuro-symbolic phase. It is high-value, low-disruption, and naturally aligned with the current observability stack.

### Option G - Neural Anomaly Detector

**Role**: Learn patterns in nominal BT event streams and flag unusual execution traces.

**Assessment**

- **Value**: Medium to high
- **Safety burden**: Moderate
- **Runtime criticality**: Medium
- **Validation strength**: Moderate
- **Readiness**: Medium to low

**Why it is promising**

- Real-time inference is feasible with compact models
- It complements, rather than replaces, symbolic execution controls
- It can improve monitoring before a full failure occurs

**Main concerns**

- Needs representative normal-operation data
- False positives may be frequent in novel but valid missions
- Without careful operator design, alerts may become noise rather than actionable safety signals

**Review judgement**

Worth pursuing after enough execution data exists and after the project has stronger runtime monitoring and alert-management conventions.

## Comparative Summary

| Option | Primary value | Risk level | Best deployment mode | Validation path | Recommended timing |
|--------|---------------|------------|----------------------|-----------------|-------------------|
| A: LLM Heuristic Guide | Faster planning in large domains | Medium | Planner-adjacent with timeout | Symbolic planner still validates | Phase 2 |
| B: LLM Plan Repair | Faster recovery after failure | Medium-High | Recovery path only | Forward simulation + fallback replan | Phase 2 |
| C: Domain Authoring | Faster domain creation | High | Offline, human-reviewed | Parser + semantic review | Phase 4 |
| D: Learned Heuristic | Production-speed search guidance | Medium | Embedded/runtime | Symbolic planner still validates | Phase 3 |
| E: Goal Interpreter | Better operator UX | Low | Front-end to planning | Grounded goal validation | Phase 1 |
| F: Mission Analyst | Better audit and operator insight | Low | Batch or interactive analysis | Evidence citation and source retrieval | Phase 1 |
| G: Anomaly Detector | Earlier issue detection | Medium | Runtime monitoring | Threshold tuning + operator workflow | Phase 3 |

## Cross-Document Gaps and Inconsistencies

The review found several documentation issues worth resolving even if no code changes are made yet.

### 1. Neuro-symbolic roadmap placement is underspecified

`subprojects/AME/docs/research/neuro_symbolic_reasoning.md` proposes an "Extension 8" for neuro-symbolic reasoning, but `doc/extensions.md` currently lists only seven extensions. The roadmap should either:

- add neuro-symbolic reasoning explicitly as the next extension, or
- fold it into the existing roadmap with clear dependencies and acceptance criteria

### 2. Goal decomposition is described too strongly in the assurance plan

The assurance documentation refers to autonomous goal decomposition as if it is a present capability, while the neuro-symbolic document frames goal interpretation as a future option. That wording should be aligned so future capabilities are not presented as already implemented.

### 3. Safe-state maturity appears ahead of evidence

The assurance plan assumes safe-state fallback concepts, but also documents missing safe-state integration evidence. That does not invalidate the architecture, but it does mean the maturity claim should stay clearly provisional.

### 4. World-state authority semantics need clarification

The documentation currently says:

- BT nodes write effects to the world model
- perception also writes authoritative fact updates
- the world model should not blindly trust the plan's model of reality

That is directionally sensible, but still leaves an ambiguity:

- are BT-applied effects authoritative facts
- optimistic model updates
- or provisional assertions awaiting perception confirmation

This matters directly for plan repair, anomaly analysis, and assurance claims.

## Recommended Adoption Sequence

The sequence proposed in `subprojects/AME/docs/research/neuro_symbolic_reasoning.md` is broadly correct. This review refines it as follows.

### Phase 1 - Start here

1. **Option E: Goal Interpreter**
2. **Option F: Mission Analyst (batch first)**

Why:

- no need to alter planner correctness
- simple fallback behaviour
- immediate operator value
- good fit for existing observability

### Phase 2 - Planner-adjacent but still symbolically bounded

3. **Option A: LLM Heuristic Guide**
4. **Option B: LLM Plan Repair**

Gate these on:

- timeout budgets
- deterministic fallback policy
- structured logging of every proposal and rejection
- benchmark evidence that the added complexity pays off

### Phase 3 - Data-driven operational tooling

5. **Option D: Learned Heuristic**
6. **Option G: Neural Anomaly Detector**

Gate these on:

- enough high-quality audit data
- model versioning and data lineage
- runtime monitoring for drift and false positives

### Phase 4 - Offline authoring support

7. **Option C: Domain Authoring**

Gate this on:

- human review workflow
- semantic domain checks
- governance for generated planning artifacts

## Recommended Pre-Requisites Before Any Implementation

Before introducing any neural component, the documentation should define:

1. **Neural component acceptance criteria**
   - latency budget
   - fallback path
   - minimum logging fields
   - operator visibility requirements

2. **State-authority semantics**
   - what is believed
   - what is observed
   - what is provisional

3. **Assurance trigger points**
   - when AMLAS becomes required
   - what evidence must exist before runtime deployment

4. **Test strategy**
   - prompt/output validation tests
   - adversarial and out-of-context cases
   - replay tests using audit logs
   - performance tests under timeout pressure

5. **Data and retrieval strategy**
   - how logs are indexed
   - how relevant windows are retrieved
   - how neural outputs cite evidence rather than free-form opinion

## Final Recommendation

The documented neuro-symbolic strategy is credible and well aligned with the repository's symbolic architecture, provided the project keeps to one rule:

> Neural components may assist, but the symbolic system must remain the authority for mission correctness and safety-critical state transitions.

On that basis:

- **Proceed first with Option E and Option F**
- **Treat Option A and Option B as bounded experiments, not default planning behaviour**
- **Adopt Option D and Option G only when enough audited mission data exists**
- **Keep Option C offline and human-reviewed**

If the project follows that path, the neuro-symbolic work can extend usability and scalability without sacrificing the determinism, auditability, and assurance posture that the current documentation makes central to the system.

