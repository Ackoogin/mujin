# Tooling Analysis: Planners and Verifiers for SACE

[Back to Index](00-index.md) | [Previous: E2E Example](12-e2e-example.md)

---

## Current Capability Baseline

The ame system uses LAPKT BRFS (Breadth-First Search) over eagerly-grounded STRIPS problems. This is the **execution planner** — it generates plans that the BT executor runs. It is fast, sound, and complete for STRIPS.

However, the SACE safety case requires analyses that go beyond goal-directed STRIPS planning. This document identifies the tooling gaps and recommends specific tools for each.

---

## Gap Analysis

### Gap 1: Trajectory Constraints (PDDL 3.0+)

**SACE Stages:** 3 (SOC constraints), 4 (decomposed constraints)

**What's needed:** Verify that plans satisfy global constraints over the entire trajectory, not just at individual action steps. Examples:

- `(always (not (collision ?r)))` — safety invariant
- `(sometime-before (at ?r base) (fuel-check-done ?r))` — ordering constraint
- `(at-most-once (airspace-enter ?r ?zone))` — visit constraints

**Why STRIPS can't do it:** STRIPS plans are sequences of state transitions. There is no mechanism to express or check properties over the entire sequence. Precondition gating (our current workaround) only works for state invariants that can be expressed as action preconditions — it cannot express temporal ordering, counting, or path-level properties.

**Recommended tools:**

| Tool | Capability | Licence | Integration Effort |
|------|-----------|---------|-------------------|
| **ENHSP** | Supports PDDL 3.0 constraints, numeric fluents, temporal preferences | LGPL | Medium — Java, CLI interface |
| **LPRPG-P** | Preferences and trajectory constraints | GPL | Medium — C++, CLI |
| **OPTIC** | Temporal + trajectory constraints | Academic | Medium — C++, CLI |

**Integration approach:** Offline analysis tool. Write constrained PDDL domain/problem files. Run external planner as a subprocess. Parse output for plan validity / constraint violation. The ame execution planner remains unchanged — the external tool is used only for safety case evidence generation.

---

### Gap 2: Model Checking / Reachability Analysis

**SACE Stages:** 2 (exhaustive hazard search), 7 (universal safe-state reachability), 8 (coverage metrics)

**What's needed:**

- "Does predicate `(collision ?r)` become true on **any** reachable path?" (existential reachability)
- "From **every** state where `(comms-lost)` is true, can the system reach `(at ?r base)`?" (universal reachability)
- "What fraction of the reachable state space has been explored?" (coverage)

**Why STRIPS can't do it:** A STRIPS planner searches for **one** path to **one** goal. It does not enumerate all reachable states, check universal properties, or report coverage. It answers "does a plan exist?" not "is a property true everywhere?"

**Recommended tools:**

| Tool | Capability | Licence | Notes |
|------|-----------|---------|-------|
| **SPIN** | LTL model checking, exhaustive state-space exploration, counterexample generation | BSD | Industry standard. Requires translation from PDDL to Promela |
| **NuSMV** | CTL + LTL model checking, BDD-based symbolic exploration | LGPL | Good for finite-state systems. Requires PDDL-to-SMV translation |
| **TorchLight** | PDDL-native dead-end detection and reachability analysis | Academic | Directly consumes PDDL; limited to reachability, not full temporal logic |
| **myND** | Non-deterministic PDDL planner, strong/weak plan analysis | Academic | Handles FOND (fully observable non-deterministic) domains — useful for "plan works under ALL possible failure outcomes" |
| **VAL** (Plan Validator) | Validates PDDL plans against domains, checks constraint satisfaction | BSD | Part of the IPC toolkit. Doesn't search, but validates given plans against PDDL 3.0 constraints |

**Integration approach — two tiers:**

**Tier 1 (near-term):** Use VAL to validate plan traces from the ame planner against PDDL 3.0 constrained domains. This is a lightweight check: the ame planner generates the plan, VAL checks it against the full constraint set. Catches constraint violations without changing the planner.

**Tier 2 (medium-term):** Build a PDDL-to-Promela translator for SPIN. This enables full LTL property checking over the PDDL state space. Properties like `AG (comms-lost -> EF (at uav1 base))` ("globally, if comms are lost, eventually the UAV can reach base") become directly checkable.

---

### Gap 3: Probabilistic / Stochastic Analysis

**SACE Stages:** 6 (failure probability), Residual Risk (quantitative risk assessment)

**What's needed:**

- "What is the probability of reaching a hazardous state given failure rate distributions?"
- "Does the system satisfy a safety property with probability ≥ p?"

**Why STRIPS can't do it:** STRIPS is purely Boolean and deterministic. No probability distributions, no stochastic effects.

**Recommended tools:**

| Tool | Capability | Licence | Notes |
|------|-----------|---------|-------|
| **PRISM** | Probabilistic model checking (DTMCs, MDPs, CTMCs). Properties in PCTL/CSL | GPL | Widely used for safety analysis. Requires PDDL-to-PRISM translation |
| **Storm** | High-performance probabilistic model checker | GPL | Faster than PRISM for large models. Same translation requirement |
| **PPDDL planners** (e.g. FF-Replan, FOND-SAT) | Plan under outcome uncertainty | Various | Less mature than dedicated probabilistic model checkers |

**Integration approach:** Build a PDDL+failure-rate-annotation → PRISM translator. Each PDDL action with a failure mode becomes a probabilistic choice in the PRISM model. Run PRISM with quantitative properties. This is a significant integration effort but provides the quantitative residual risk evidence that ISO 21448 requires.

---

### Gap 4: Temporal Planning

**SACE Stages:** 5 (timing constraints under degraded modes), 6 (failure-effect timing)

**What's needed:**

- Actions with durations (e.g. `move` takes time proportional to distance)
- Concurrent action scheduling with resource constraints
- Timing-based safety properties ("reach safe state within T seconds")

**Why STRIPS can't do it:** STRIPS actions are instantaneous and sequential. No durations, no concurrency, no real-time bounds.

**Recommended tools:**

| Tool | Capability | Licence | Notes |
|------|-----------|---------|-------|
| **OPTIC** | Temporal planning with PDDL 2.1 durative actions and trajectory constraints | Academic | Best combination of temporal + constraints |
| **TPSHE** | Temporal planning with soft/hard time windows | Academic | Good for deadline-constrained problems |
| **POPF** | Partial-order temporal planner | Academic | Handles temporal concurrency |

**Integration approach:** Create PDDL 2.1 domain variants with `:durative-actions` for timing analysis. Run externally. The ame execution planner continues to use the STRIPS version for actual execution; temporal analysis is offline evidence only.

---

### Gap 5: Automated Scenario Generation

**SACE Stages:** 2 (systematic hazard exploration), 8 (coverage), Residual Risk (unknown-unsafe exploration)

**What's needed:**

- Systematic generation of fault-injected problem files from a fault catalogue
- Combinatorial coverage of ISO 34502 risk factor combinations
- Randomised problem generation for unknown-unsafe discovery (ISO 34502 Annex K)

**Why manual file creation doesn't scale:** With *n* fault predicates, there are 2^*n* combinations. With 3 perception faults, 2 planning faults, and 2 control faults, that's 128 combinations. Each needs a problem file, planner run, and result classification.

**Recommended approach:** Build a **PDDL Problem Generator** within the ame toolchain:

```
Inputs:
  - Baseline domain file
  - Baseline problem file
  - Fault catalogue (list of predicates to toggle)
  - Coverage strategy (all-combinations / pairwise / random-N)

Outputs:
  - Generated problem files (one per fault combination)
  - Batch planner results (plan found / no plan / timeout)
  - Coverage report (which combinations tested, results)
```

This is a Python or C++ tool that reads the baseline PDDL, generates variants by toggling init predicates, runs the ame planner (or an external planner) on each, and collects results. This is the most immediately actionable tooling gap — it requires no new planner, just automation around the existing one.

---

### Gap 6: Plan Quality / Optimality

**SACE Stage:** 8 (verification — is the plan not just valid but good?)

**What's needed:**

- Cost-optimal plans (shortest, safest, most fuel-efficient)
- Plan comparison (is the constrained plan significantly worse than unconstrained?)
- Pareto analysis of safety vs. mission effectiveness

**Recommended tools:**

| Tool | Capability | Licence | Notes |
|------|-----------|---------|-------|
| **Fast Downward** | Optimal and satisficing STRIPS/SAS+ planning with many heuristics | GPL | Industry standard. Directly consumes PDDL. Drop-in replacement for analysis |
| **Scorpion** | Optimal planning with symbolic search | GPL | State of the art for optimal STRIPS |

**Integration approach:** Use Fast Downward as an offline analysis planner for plan quality evidence. The ame BRFS planner remains the execution planner. Fast Downward's cost-optimal plans provide evidence that the execution planner's satisficing plans are not unnecessarily far from optimal.

---

## Recommended Tooling Architecture

```
                    ┌---------------------------------------------┐
                    │           SACE Evidence Pipeline             │
                    │         (offline, safety case only)          │
                    ├---------------------------------------------┤
                    │                                             │
  PDDL Domain -----┤  ┌--------------┐   ┌------------------┐   │
  + Problem         │  │ Problem      │--▶│ Ame Planner    │   │--▶ Plan traces
  + Fault Catalogue │  │ Generator    │   │ (LAPKT BRFS)     │   │    (STRIPS scenarios)
                    │  └--------------┘   └------------------┘   │
                    │         │                                   │
                    │         │           ┌------------------┐   │
                    │         └----------▶│ VAL              │   │--▶ Constraint checks
                    │                     │ (Plan Validator)  │   │    (PDDL 3.0)
                    │                     └------------------┘   │
                    │                                             │
  PDDL 3.0         │  ┌--------------------------------------┐   │
  Constrained  -----┤  │ ENHSP / OPTIC                       │   │--▶ Constraint-
  Domain            │  │ (trajectory constraint planner)      │   │    satisfying plans
                    │  └--------------------------------------┘   │
                    │                                             │
  Promela /         │  ┌--------------------------------------┐   │
  SMV Model    -----┤  │ SPIN / NuSMV                         │   │--▶ Property proofs
  (translated)      │  │ (model checker)                      │   │    + counterexamples
                    │  └--------------------------------------┘   │
                    │                                             │
  PDDL +            │  ┌--------------------------------------┐   │
  Failure Rates ----┤  │ PRISM / Storm                        │   │--▶ Quantitative
                    │  │ (probabilistic model checker)        │   │    risk metrics
                    │  └--------------------------------------┘   │
                    │                                             │
  PDDL Domain -----┤  ┌--------------------------------------┐   │
                    │  │ Fast Downward                        │   │--▶ Optimal plan
                    │  │ (optimal STRIPS planner)             │   │    quality evidence
                    │  └--------------------------------------┘   │
                    └---------------------------------------------┘
                                         │
                                         ▼
                    ┌---------------------------------------------┐
                    │           Runtime Execution                  │
                    │         (unchanged — ame core)             │
                    ├---------------------------------------------┤
                    │  WorldModel --▶ LAPKT BRFS --▶ PlanCompiler │
                    │       --▶ BT.CPP Executor                   │
                    └---------------------------------------------┘
```

**Key architectural principle:** The ame execution planner is never replaced. External tools run offline, consuming the same PDDL domain files, and produce evidence artefacts that populate the GSN safety argument. The execution pipeline remains STRIPS-only and fast.

---

## Implementation Priority

| Priority | Tool / Capability | SACE Stages | Effort | Value |
|----------|------------------|-------------|--------|-------|
| **1 — Immediate** | PDDL Problem Generator (custom) | 2, 8, Residual | Low (Python script) | High — unblocks systematic fault injection |
| **2 — Near-term** | VAL (plan validator) | 3, 8 | Low (download + CLI) | High — validates plans against PDDL 3.0 constraints without changing planner |
| **3 — Near-term** | Fast Downward (optimal planner) | 8 | Low (download + CLI) | Medium — plan quality evidence |
| **4 — Medium-term** | ENHSP or OPTIC (constrained planner) | 3, 4 | Medium (CLI integration) | High — trajectory constraint satisfaction |
| **5 — Medium-term** | PDDL-to-Promela translator + SPIN | 2, 7, 8 | High (custom translator) | Very high — universal reachability, full LTL |
| **6 — Longer-term** | PRISM integration | 6, Residual | High (custom translator) | High — quantitative risk, ISO 21448 compliance |

---

## Tool-to-SACE-Stage Mapping

| Tool | Stage 1 | Stage 2 | Stage 3 | Stage 4 | Stage 5 | Stage 6 | Stage 7 | Stage 8 | Residual |
|------|---------|---------|---------|---------|---------|---------|---------|---------|----------|
| **Ame BRFS** | Plan generation | Fault scenarios (one-at-a-time) | Precondition-gated constraints | Sub-domain plans | Degraded-mode plans | Single failure variants | Per-state recovery | Plan traces | — |
| **Problem Generator** | — | Systematic fault combinations | — | — | Degraded config enumeration | Failure combinations | OOC state enumeration | Scenario coverage | Random scenarios |
| **VAL** | — | — | Constraint validation | Decomposition validation | — | — | — | Plan validation | — |
| **Fast Downward** | — | — | — | — | — | — | — | Optimal plans | — |
| **ENHSP / OPTIC** | — | — | Trajectory constraints | Constraint decomposition | Temporal constraints | — | — | Constrained plans | — |
| **SPIN / NuSMV** | — | Exhaustive reachability | — | Joint constraint implication | — | — | Universal recovery | Coverage metrics | Property proofs |
| **PRISM / Storm** | — | — | — | — | — | Failure probability | — | — | Quantitative risk |

---

## Key Design Decisions

### 1. Separation of Execution and Analysis

The execution planner (LAPKT BRFS) and the analysis tools serve different purposes:

- **Execution:** Fast, deterministic, STRIPS-only. Runs at mission time. Must be simple and auditable.
- **Analysis:** Comprehensive, potentially slow, uses richer PDDL semantics. Runs at design/assurance time. Produces evidence artefacts.

This separation is itself a safety argument: the execution planner's simplicity is an assurance property. The analysis tools verify the execution planner's domain, not replace it.

### 2. Shared PDDL Domain with Formal Subsumption

All tools consume the same baseline PDDL domain files, though analysis tools may use richer extensions (trajectory constraints, durative actions, probabilistic effects). This design relies on a **formal subsumption argument**: the STRIPS execution domain is a *restriction* of the richer analysis domains, meaning properties proven on the analysis domain hold on the execution domain provided the analysis domain only adds constraints or expressiveness that the STRIPS domain implicitly satisfies.

This subsumption relationship must be explicitly argued for each analysis tool integration. It ensures that:

- The model being analysed is traceable to the model being executed
- Traceability from evidence to execution is maintained through the subsumption argument
- Changes to the baseline domain automatically trigger re-analysis across all tool tiers

Note: "single source of truth" is used informally — the execution and analysis domains are not identical, but are formally related through subsumption

### 3. Incremental Adoption

The tooling can be adopted incrementally. Priority 1 (Problem Generator) requires no external dependencies. Priority 2 (VAL) is a single binary download. Each subsequent tool adds capability without changing prior results.

---

## Assumptions and Risks

| Assumption | Risk if Violated | Mitigation |
|-----------|-----------------|------------|
| PDDL domain is a faithful abstraction of the real system | Analysis evidence is irrelevant to real behaviour | Model-to-implementation traceability review; simulation correlation testing |
| External tools produce correct results | Unsound evidence in safety case | Use well-established, peer-reviewed tools; cross-validate with multiple tools |
| STRIPS execution domain is semantically compatible with richer analysis domains | Analysis results don't apply to execution domain | Formal subsumption argument: STRIPS domain is a restriction of the analysis domain |
| Fault catalogue covers relevant failure modes | Incomplete hazard coverage | Systematic derivation from ISO 34502 annexes; independent review |

---

[Back to Index](00-index.md)

