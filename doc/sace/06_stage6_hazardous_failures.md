# SACE PDDL Contributions — Stage 6: Hazardous Failures Management

*Part of the [SACE PDDL Contributions](00_standards_context.md) document set.*

**GSN Pattern: [DD] — Hazardous Failures Management**

---

## SACE Purpose

Once the design for a tier is known, potential hazardous failures that could result from design decisions can be identified. Stage 6 addresses the identification and acceptable management of these hazardous failures. This includes:

- Failures introduced as side-effects of design choices
- Failures arising from component interactions
- Failures specific to the autonomous functions (perception, planning, actuation) that may not be apparent from the requirements or architecture alone

---

## PDDL Contribution

PDDL supports hazardous failure management in two principal ways:

1. **Systematic failure-mode exploration at the planning level.** By introducing failure predicates into the domain (representing component failures, data corruption, timing violations, or incorrect predicate grounding), the planner discovers whether such failures lead to unsafe plans or states.

2. **Interface contract verification.** PDDL defines the predicate interface between autonomous functions: perception must produce the predicates the planner consumes, and actuation must faithfully execute the actions the planner emits. Failures in meeting this contract are hazardous failures that Stage 6 must manage.

---

## Inputs

| Input | How it informs PDDL |
|-------|---------------------|
| PDDL domain from Stage 5 | Architecture-mapped domain defining the design at this tier |
| Design failure mode analysis | Hardware/software failure modes mapped to PDDL predicates representing failed states |
| Component interaction analysis | Emergent failures from component interactions modelled as unexpected predicate combinations |
| ML component safety cases (AMLAS) | Linked to PDDL predicate reliability assumptions, identifying where ML failures could corrupt planner inputs |

---

## Outputs

| Output | Description |
|--------|-------------|
| **Failure-mode PDDL variants** | Domain variants with failure predicates active, representing specific component or interaction failures |
| **Hazardous failure register** | Catalogue of identified failures with their PDDL representations and the unsafe states they can reach |
| **Mitigation strategy mapping** | For each hazardous failure: the design change, additional requirement, or operational restriction that manages it, with PDDL evidence that the mitigation is effective |
| **Residual failure analysis** | Identification of failures that cannot be fully mitigated, with associated risk assessment |

---

## GSN Mapping — Pattern [DD]

Pattern [DD] argues that potential hazardous failures introduced through design decisions have been identified and are acceptably managed.

| GSN Element | Content | PDDL Role |
|-------------|---------|-----------|
| **Goal** | Hazardous failures that may arise from the design at tier *n* are acceptably managed | Target |
| **Strategy** | Argue over each identified hazardous failure that it is either eliminated by design, mitigated by additional requirements, or that residual risk is acceptable; decompose into sub-goals per failure mode | Failure-mode planning as strategy |
| **Context** | The design at tier *n* from Stage 5; the failure mode analysis identifying potential hazardous failures; the PDDL domain structure mapping design components to actions and predicates | Domain structure as context |
| **Solution** | Failure-mode PDDL variants (consequences of each failure); mitigation strategy mapping (design changes or constraints prevent unsafe states); AMLAS safety cases for ML components (where applicable) | PDDL artefacts as solution nodes |
| **Assumption** | The failure mode analysis is sufficiently complete; the PDDL model faithfully represents the design's failure behaviour — both require confidence arguments | Two explicit assumptions requiring confidence arguments |
| **Justification** | ISO 21448 functional insufficiency analysis justifies examining failures arising from the gap between intended and actual component performance; ISO 34502 risk factor categories justify structuring the failure analysis along perception, judgement, and control axes | Standards as justification nodes |

---

## ISO 21448 (SOTIF) Impact

ISO 21448's functional insufficiency concept extends naturally to hazardous failure management:

- A **functional insufficiency** is effectively a failure of the intended functionality to perform as specified under certain conditions.
- PDDL failure-mode variants make these insufficiencies concrete: by activating failure predicates (such as incorrect predicate grounding from a perception insufficiency), the planner reveals whether the design can still achieve safe outcomes.
- Where it cannot, the failure is a **hazardous failure requiring management**.
- AMLAS provides the component-level evidence for ML-based functions where ML failures constitute functional insufficiencies.

---

## ISO 34502 Impact

ISO 34502 Annexes C and D directly inform the failure modes relevant to perception and control components:

| ISO 34502 Annex | Disturbance Category | PDDL Failure Predicate Examples |
|-----------------|---------------------|--------------------------------|
| **Annex C** | Perception disturbances | `(camera-degraded)`, `(lidar-occluded)`, `(gps-spoofed)` |
| **Annex D** | Control disturbances | `(actuator-saturated)`, `(control-loop-degraded)`, `(timing-violation)` |

Each annex provides a structured catalogue of disturbances that, when encoded as PDDL failure predicates, systematically populate the hazardous failure register.

---

## Failure-Mode Variant and Mitigation Pattern

```
Step 1: Identify failure
  Perception pipeline produces (obstacle-at ?wp)
  Hazardous failure: ML detector generates false negative under sun glare
  → Encode as: (perception-failed obstacle-detector) in initial state

Step 2: Analyse consequences
  Failure-mode variant: removes (obstacle-at ?wp) precondition from navigate
  Planner finds: plan reaching (collision ?uav ?obstacle)
  → Failure confirmed as hazardous

Step 3: Define mitigation
  Design change: LIDAR provides independent detection
  Updated domain: (obstacle-at ?wp) groundable by camera OR lidar

Step 4: Re-verify with mitigation
  New failure-mode variant: (camera-failed) only (LIDAR operational)
  Planner finds: valid constrained plan with no collision
  → Mitigation verified
```

---

## UAV Example

**Identified hazardous failure:** The ML-based obstacle detector produces a false negative under sun glare (ISO 34502 Annex C perception disturbance).

**Failure-mode PDDL variant:** Sets `(perception-failed obstacle-detector)` as true, removing the `(obstacle-at ?wp)` precondition from `navigate` actions.

**Planner result:** Finds a plan reaching `(collision ?uav ?obstacle)` — hazardous failure confirmed.

**Mitigation:** LIDAR provides independent detection. PDDL domain updated so `(obstacle-at ?wp)` can be grounded by either sensor.

**Re-verification:** Running the failure-mode variant with only `(camera-failed)` confirms safe plans are still achievable.

**GSN integration:**
- The re-analysis result is the **GSN solution node** for the sub-goal "collision hazard HS-017 is managed under camera failure."
- The AMLAS safety case for the ML detector is an **away goal** connected via an **assurance claim point** (the characteristic SACE marker requiring additional confidence argument).

---

## Interface Contract Failures

PDDL explicitly models the interface between autonomous functions:

| Interface | Predicate Contract | Failure Mode | Consequence |
|-----------|-------------------|--------------|-------------|
| Perception → Planner | `(obstacle-at ?wp)` must be correctly grounded | False negative (miss) | Plan navigates into obstacle |
| Perception → Planner | `(comms-available)` must reflect real state | False positive | Plan assumes connectivity; loses link |
| Planner → Actuator | Action effects must be achievable | Actuation saturation | Effect not realised; state diverges |
| Actuator → WorldModel | Executed action must update predicates | Missing feedback | WorldModel stale; replanning fails |

Each interface failure is a candidate for the hazardous failure register.

---

## Evidence Checklist

- [ ] Failure mode analysis produced for each design component
- [ ] Failure predicates defined and encoded in PDDL domain
- [ ] Failure-mode PDDL variants created per failure mode
- [ ] Planner run on each variant to identify consequences
- [ ] Hazardous failures identified and registered
- [ ] Mitigations designed and encoded in PDDL
- [ ] Mitigation re-verification completed (planner on mitigated domain)
- [ ] AMLAS safety cases linked for any ML components (via assurance claim points)
- [ ] ISO 34502 Annex C and D disturbances addressed in failure mode set
- [ ] Failure mode completeness assumption documented with confidence argument
- [ ] Residual failures documented with risk assessment

---

*Previous: [Stage 5 — Design Assurance](05_stage5_design_assurance.md)*
*Next: [Stage 7 — Out of Context Operation Assurance](07_stage7_out_of_context.md)*
