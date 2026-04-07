# End-to-End SACE Stage Application Example

[Back to Index](00-index.md)

---

## Purpose

This document walks through a concrete application of the SACE process to the UAV search domain, using the PDDL files in `subprojects/AME/domains/uav_search_sace/`. It demonstrates what each SACE stage produces in practice and identifies exactly where the current ame execution planner (LAPKT BRFS, STRIPS-only) is sufficient and where external safety-case tooling is required.

---

## Domain Files

| File | SACE Stage | Purpose |
|------|-----------|---------|
| `domain_baseline.pddl` | 1 | Baseline operating context with ODD gating predicates |
| `problem_nominal.pddl` | 1 | Nominal scenario — all ODD conditions satisfied |
| `problem_weather_fault.pddl` | 2 | Fault injection — weather degraded at target sector |
| `problem_comms_lost.pddl` | 2 | Fault injection — C2 link lost |
| `problem_ooc_recovery.pddl` | 7 | Out-of-context — can the UAV return to base? |
| `domain_degraded.pddl` | 5 | Design improvement — adds degraded-mode actions |
| `problem_ooc_recovery_degraded.pddl` | 5/7 | Recovery re-test with degraded-mode design |

---

## Stage-by-Stage Walkthrough

### Stage 1: Operating Context Assurance

**What we do:** Encode the ODD as PDDL predicates that gate action applicability.

The baseline domain (`domain_baseline.pddl`) adds three ODD predicates to the original UAV search domain:

```lisp
(airspace-clear ?l - location)    ; airspace authorisation
(weather-ok ?l - location)        ; within weather envelope
(comms-available)                 ; C2 link active
```

Every action that moves the UAV or performs a mission task requires these predicates. This means the planner **cannot** produce a plan that violates the ODD — the operating context is formally enforced.

**Can the ame planner do this?** Yes. This is pure STRIPS. Run:

```
domain_baseline.pddl + problem_nominal.pddl → expect: PLAN FOUND
```

The plan trace and the domain file are the Stage 1 evidence artefacts (GSN solution and context nodes respectively).

**Traceability check:** Each ODD requirement maps to a predicate:

| ODD Requirement | PDDL Predicate | Gated Actions |
|-----------------|---------------|---------------|
| Airspace authorisation | `(airspace-clear ?l)` | `move` |
| Weather envelope | `(weather-ok ?l)` | `move` |
| C2 link availability | `(comms-available)` | `move`, `search`, `classify` |

---

### Stage 2: Hazardous Scenario Identification

**What we do:** Create fault-injected problem variants and check whether hazardous states are reachable.

**Test 1 — Weather fault** (`problem_weather_fault.pddl`):

`(weather-ok sector_a)` is absent. Since the mission goal requires reaching sector_a, and `move` requires `(weather-ok ?to)`, the planner should return **NO PLAN**.

This is a *negative* result used as evidence: the ODD gating prevents the hazardous scenario "UAV enters degraded weather." If the planner *did* find a plan, that would be a counterexample revealing a hazard.

**Test 2 — Comms lost** (`problem_comms_lost.pddl`):

`(comms-available)` is absent. All mission actions require it, so the planner returns **NO PLAN** for the mission goal.

But we also need to check: *can the UAV still reach a safe state?* The `loiter-in-place` action has no comms precondition, so it remains applicable. This is Stage 7 evidence.

**Can the ame planner do this?** Yes, for this specific analysis. We run the planner and check:

- Plan found → hazardous state is reachable (counterexample trace)
- No plan found → gating prevents the hazardous scenario

**What the ame planner CANNOT do for Stage 2:**

| Analysis Need | Why It's Beyond STRIPS | What's Needed |
|--------------|----------------------|---------------|
| **Exhaustive state-space enumeration** | LAPKT BRFS explores reachable states but doesn't expose the search tree or report coverage metrics | A model checker (e.g. SPIN, NuSMV) or a planner that reports state-space statistics |
| **"Does hazard predicate X become true on ANY path?"** | This is a reachability/safety property check, not a goal-directed planning query | Model checker with CTL/LTL property: `EF (collision uav1)` |
| **Concurrent fault combinations** | Current approach requires one problem file per fault combination — combinatorial explosion | Automated problem generator + batch planner runner |

---

### Stage 3: Safe Operating Concept — Constraint Satisfaction

**What we do:** Express safety requirements as constraints and verify that plans satisfying mission goals also satisfy all constraints.

**The safety requirement:** "The UAV shall not enter a location where weather is not OK."

In PDDL 3.0+ this is a trajectory constraint:

```lisp
(:constraints
  (always (forall (?r - robot ?l - location)
    (implication (not (weather-ok ?l)) (not (at ?r ?l))))))
```

**Can the ame planner do this?** No. The ame planner supports STRIPS only — no trajectory constraints, no `always`, no `forall`, no `implication`.

**Workaround with STRIPS:** In this specific case, we achieved the same effect by encoding the constraint as action preconditions (Stage 1). The `move` action requires `(weather-ok ?to)`, which structurally prevents the constraint violation. This is a *design-time enforcement* rather than a *planning-time check*.

**When the workaround breaks down:**

| Constraint Type | Precondition Encoding? | Why Not |
|----------------|----------------------|---------|
| State invariant enforceable by preconditions | Yes | Directly maps to action gating |
| Trajectory constraint over intermediate states | No | STRIPS plans only check preconditions at each step, not global path properties |
| Constraint involving counting or numeric thresholds | No | No numeric fluents in STRIPS |
| Disjunctive constraints ("A or B must hold") | No | STRIPS preconditions are conjunctive only |
| Preference-weighted constraints ("prefer X") | No | No plan quality metrics |

For the full SACE Stage 3 analysis, an external planner with PDDL 3.0+ support is needed (see [Tooling Analysis](13-tooling-analysis.md)).

---

### Stage 4: Safety Requirements Decomposition

**What we do:** Decompose system-level constraints into sub-system PDDL sub-domains and verify that decomposed constraints jointly imply the system constraint.

For this example, the system constraint "maintain safe operation" decomposes to:

| Sub-system | Constraint | PDDL Encoding |
|-----------|-----------|---------------|
| Perception | Obstacle detection available | `(airspace-clear ?l)` predicate truthfulness |
| Planning | Only plan to safe locations | `(weather-ok ?to)` precondition on `move` |
| Control | Safe-state always reachable | `loiter-in-place` has no degraded preconditions |

**Can the ame planner do this?** Partially. It can verify each sub-domain independently (same STRIPS planning), but it cannot formally verify that the sub-domain constraints *jointly imply* the system constraint. That requires a theorem prover or model checker.

---

### Stage 5: Design Assurance — Degraded-Mode Actions

**What we do:** Add redundant action paths for degraded conditions and verify that safety constraints hold under degraded configurations.

The `domain_degraded.pddl` adds two emergency return actions:

```lisp
;; Works without comms, needs GPS
(:action emergency-return-gps
  :precondition (and (at ?r ?from) (airspace-clear ?to) (gps-available)))

;; Works without comms AND GPS, needs inertial nav only
(:action emergency-return-inertial
  :precondition (and (at ?r ?from) (inertial-nav-available)))
```

**Verification test:** Run `problem_ooc_recovery.pddl` against the baseline domain → **NO PLAN** (cannot return without comms). Then run `problem_ooc_recovery_degraded.pddl` against the degraded domain → **PLAN FOUND** (returns via `emergency-return-inertial`).

This pair of results is the Stage 5 evidence: the design improvement (adding degraded-mode actions) resolves the Stage 7 gap.

**Can the ame planner do this?** Yes — this is again pure STRIPS. The ame planner can verify each degraded configuration by running the appropriate problem file.

---

### Stage 6: Hazardous Failures Management

**What we do:** Model component failures as predicate states and check whether failures lead to unsafe plans.

Example: "What if the obstacle detector produces a false negative?"

A failure-mode domain variant would remove the `(airspace-clear ?to)` precondition from `move`, modelling a perception failure where the UAV doesn't know the airspace is blocked. If the planner then finds a plan that enters a blocked sector, that's a hazardous failure.

**Can the ame planner do this?** Yes, for the basic case — create a variant domain, run the planner, check if a hazardous goal state is reachable. But:

| Analysis Need | Ame Planner | External Tool |
|--------------|---------------|---------------|
| Single failure variant | Yes — edit domain, run planner | — |
| Combinatorial failure analysis | Manual — one domain per combination | Automated variant generator needed |
| Failure probability weighting | No — STRIPS is Boolean | Probabilistic planner (e.g. PPDDL + MDP solver) |
| Failure-effect temporal ordering | No — no durative actions | Temporal planner (e.g. OPTIC, TPSHE) |

---

### Stage 7: Out-of-Context Operation

**What we do:** Verify safe-state reachability from all out-of-context states.

The `problem_ooc_recovery.pddl` models the UAV at sector_a with no comms and degrading weather. With the baseline domain, no plan exists (demonstrating the gap). With the degraded domain, `emergency-return-inertial` provides a path home.

**The key question for Stage 7:** "From EVERY reachable out-of-context state, can the UAV reach a safe state?"

**Can the ame planner do this?** Only one state at a time. For each out-of-context configuration, you must write a problem file and run the planner. This does not scale.

**What's actually needed:** Universal reachability checking — "for all states where `(comms-lost)` is true, does there exist a plan reaching `(at uav1 base)`?" This is a model checking problem (see [Tooling Analysis](13-tooling-analysis.md)).

---

### Stage 8: Verification

**What we do:** Collect plan traces as verification evidence, compute coverage metrics.

For each problem file, the ame planner produces a plan trace (or NO_PLAN). Each trace is a plan validity certificate. The set of problem files exercises the scenario space.

**Evidence produced (within ame planner capability):**

| Evidence | Source | GSN Role |
|----------|--------|----------|
| Nominal plan trace | `problem_nominal.pddl` → planner | Solution node |
| Weather fault — no plan | `problem_weather_fault.pddl` → planner | Solution node (negative evidence) |
| Comms fault — no plan | `problem_comms_lost.pddl` → planner | Solution node (negative evidence) |
| OOC recovery — no plan (baseline) | `problem_ooc_recovery.pddl` → planner | Solution node (gap identification) |
| OOC recovery — plan found (degraded) | `problem_ooc_recovery_degraded.pddl` → planner | Solution node (design improvement proof) |

**What the ame planner CANNOT provide:**

| Verification Need | Why | What's Needed |
|-------------------|-----|---------------|
| State-space coverage percentage | BRFS doesn't report explored/total state counts | Model checker or custom BRFS instrumentation |
| Property verification (LTL/CTL) | STRIPS has no temporal logic support | Model checker (SPIN, NuSMV) |
| Randomised scenario generation | No problem generator built in | Custom PDDL problem generator + batch runner |
| Plan optimality evidence | BRFS returns first plan, not optimal | Cost-optimal planner (e.g. Fast Downward with A*) |

---

## Summary: What Works Now vs. What Needs Tooling

### The ame planner (LAPKT BRFS) can do

- Nominal plan generation for any STRIPS domain
- Fault-injected scenario testing (plan found / no plan)
- Degraded-mode verification (per-configuration)
- Plan trace generation for GSN solution nodes
- Single-scenario safe-state reachability checks

### The ame planner cannot do (external tooling required)

- Trajectory constraint verification (PDDL 3.0+)
- Universal reachability / safety property checking
- State-space coverage metrics
- Probabilistic failure analysis
- Temporal/durative action planning
- Plan quality optimisation
- Automated scenario generation at scale

See [Tooling Analysis](13-tooling-analysis.md) for recommended tools and integration approach.

---

[Next: Tooling Analysis](13-tooling-analysis.md)

