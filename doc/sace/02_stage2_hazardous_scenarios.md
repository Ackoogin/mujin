# SACE PDDL Contributions — Stage 2: Hazardous Scenarios Identification

*Part of the [SACE PDDL Contributions](00_standards_context.md) document set.*

**GSN Pattern: [I] — Hazardous Scenarios Identification**

---

## SACE Purpose

Identifies interactions between the AS and its environment, analyses every autonomous decision point, and enumerates hazardous scenarios that could lead to harm.

---

## PDDL Contribution

PDDL's search space is the formal enumeration of all decision points:

- By exploring the planner's **reachable state space**, engineers can systematically identify states where hazardous conditions hold.
- **Fault-injected domain variants** — where preconditions are deliberately relaxed or degradation predicates introduced — enable automated hazard discovery.
- Each counterexample trace transitions a scenario from unknown-unsafe to known-unsafe, operationalising the ISO 21448 four-quadrant model.

---

## Inputs

| Input | How it informs PDDL |
|-------|---------------------|
| PDDL domain and problem from Stage 1 | Baseline model for decision-point enumeration |
| Preliminary hazard list | Encoded as PDDL predicates (e.g. `collision`, `loss-of-comms`) |
| Interaction model | AS–environment interactions mapped to PDDL action effects |
| Fault and degradation modes | Modelled as predicate injections or precondition relaxations in fault-injected domain variants |

---

## Outputs

| Output | Description |
|--------|-------------|
| **Reachability analysis report** | Identifies which states are reachable and whether hazard predicates can become true |
| **Decision-point register** | Each applicable action in each reachable state, extracted from the planner's search tree |
| **Fault-injected domain variants** | Modified PDDL domains modelling specific degradation modes |
| **Counterexample plan traces** | Concrete action sequences leading to hazardous states |

---

## GSN Mapping — Pattern [I]

Pattern [I] argues that all hazardous scenarios have been identified and the identification is complete and correct.

| GSN Element | Content | PDDL Role |
|-------------|---------|-----------|
| **Goal** | All hazardous scenarios associated with AS operation have been identified | Target |
| **Strategy** | Systematic enumeration of decision points and reachable hazardous states using formal state-space exploration; decompose into sub-goals per hazard category | Formal exploration as strategy |
| **Context** | The PDDL domain defines the decision space; the preliminary hazard list defines which predicates constitute hazardous states; ISO 21448's four-quadrant classification scopes the argument | PDDL domain + hazard predicates as context |
| **Solution** | Reachability analysis report (which hazardous predicates are reachable); counterexample plan traces (concrete hazardous scenarios); decision-point register (completeness of decision analysis) | PDDL artefacts as solution nodes |
| **Justification** | ISO 34502 risk factor decomposition justifies generating fault-injected variants along perception, judgement, and control axes; ISO 21448's unknown-unsafe requirement justifies exhaustive state-space exploration | Standards as justification nodes |
| **Assumption** | The hazard predicate set is complete — all relevant hazardous conditions have been encoded; must be reviewed against the preliminary hazard analysis | Predicate completeness assumption |

---

## ISO 21448 (SOTIF) Impact

ISO 21448's four-quadrant model structures the argument:

- PDDL state-space search **addresses the unknown-unsafe quadrant** by exploring states not anticipated during manual hazard analysis.
- Each counterexample trace transitions a scenario from **unknown-unsafe → known-unsafe**.
- The completeness argument requires that the hazard predicate set covers all functional insufficiencies.

---

## ISO 34502 Impact

ISO 34502 annexes provide systematic templates for fault-injected domain variant generation:

| ISO 34502 Annex | Disturbance Category | PDDL Application |
|-----------------|---------------------|-----------------|
| **Annex B** | Traffic/interaction disturbances | Multi-agent interaction fault variants |
| **Annex C** | Perception disturbances | Sensor failure predicate injections |
| **Annex D** | Vehicle/control disturbances | Actuator degradation variants |

---

## Fault-Injected Domain Variant Pattern

A fault-injected domain variant relaxes or overrides specific predicates to model a component failure:

```pddl
;; Baseline: obstacle detection required before navigation
(:action navigate
  :precondition (and (at ?u ?from) (obstacle-detected ?to))
  ...)

;; Fault-injected variant: precondition relaxed to model sensor failure
;; (perception-failed sensor-camera) becomes true in initial state
(:action navigate
  :precondition (and (at ?u ?from))  ; obstacle check removed
  ...)
```

The planner running on the fault-injected variant discovers plans that reach `(collision ?uav ?obstacle)`, producing a counterexample trace.

---

## UAV Example

A fault-injected domain variant relaxes the `(obstacle-detected ?wp)` precondition to model camera sensor failure (ISO 34502 Annex C perception disturbance).

The planner discovers a plan reaching `(collision ?uav ?obstacle)`. This trace:
- Becomes a **GSN solution node** supporting the goal that hazardous scenario `HS-017` (collision due to perception failure) has been identified.
- The ISO 34502 Annex C perception disturbance category provides the **justification node** explaining why this class of fault injection is systematic.

---

## Evidence Checklist

- [ ] Preliminary hazard list encoded as PDDL predicates
- [ ] Reachability analysis run on baseline domain
- [ ] Fault-injected domain variants created for each degradation mode
- [ ] Counterexample traces produced for each identified hazardous scenario
- [ ] Decision-point register generated from planner search tree
- [ ] ISO 34502 Annex B, C, D disturbances addressed in variant set
- [ ] Hazard predicate completeness assumption documented and reviewed

---

*Previous: [Stage 1 — Operating Context Assurance](01_stage1_operating_context.md)*
*Next: [Stage 3 — Safe Operating Concept Assurance](03_stage3_safe_operating_concept.md)*
