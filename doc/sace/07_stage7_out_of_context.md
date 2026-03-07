# SACE PDDL Contributions — Stage 7: Out of Context Operation Assurance

*Part of the [SACE PDDL Contributions](00_standards_context.md) document set.*

**GSN Pattern: [PP] — Out of Context Operation**

---

## SACE Purpose

Addresses safety when the AS operates outside its defined operating context, whether through planned handover or unplanned excursion. The goal is to demonstrate that the system remains sufficiently safe when its assured operating context boundaries are exceeded.

---

## PDDL Contribution

PDDL models the in-context / out-of-context boundary as state predicates:

- **Out-of-context predicates** (e.g. `(context-exceeded)`, `(comms-lost)`, `(gps-denied)`) mark states outside the assured ODM.
- **Fallback planning** from out-of-context initial states demonstrates whether safe fallback actions are always reachable.
- **Reachability analysis** over the space of out-of-context states proves — or disproves — that a safe state is reachable from every possible excursion.

---

## Inputs

| Input | How it informs PDDL |
|-------|---------------------|
| Operating context boundary predicates | Define when the AS has left its assured context |
| Safe state definitions | PDDL goal states for fallback planning (e.g. `(loitering ?uav)`, `(returned-to-base ?uav)`) |
| Fallback action set | Actions available during out-of-context operation (may be reduced set) |
| Transition trigger conditions | Predicates activating context-exit detection |

---

## Outputs

| Output | Description |
|--------|-------------|
| **Out-of-context reachability analysis** | For each out-of-context state combination, whether a safe state is reachable |
| **Fallback plan library** | Pre-computed plans for each identified out-of-context situation |
| **Unreachable safe-state report** | States from which recovery is impossible — triggers requirement for additional mitigation |
| **Context boundary validation** | Evidence that boundary predicates correctly partition in/out-of-context states |

---

## GSN Mapping — Pattern [PP]

Pattern [PP] argues that the AS remains sufficiently safe when operating outside its defined operating context.

| GSN Element | Content | PDDL Role |
|-------------|---------|-----------|
| **Goal** | The AS remains sufficiently safe when operating outside its defined operating context | Target |
| **Strategy** | Argue over each identified out-of-context condition that a safe state is reachable; decompose into sub-goals per out-of-context trigger combination | Fallback reachability as strategy |
| **Context** | The defined operating context boundary; the set of safe states | Boundary predicates as context |
| **Solution** | Out-of-context reachability analysis (formal proof of safe-state reachability); fallback plan library (concrete plans); unreachable safe-state report (residual gaps) | PDDL artefacts as solution nodes |
| **Justification** | ISO 21448 ODD excursion requirements justify the need for this argument; ISO 34502 combined risk factor analysis justifies modelling multi-factor out-of-context states | Standards as justification nodes |
| **Assumption** | The set of out-of-context trigger conditions is complete; the fallback action set is available under the degraded conditions modelled | Completeness and availability assumptions |

---

## ISO 21448 (SOTIF) Impact

ISO 21448 requires that ODD excursions be analysed and managed:

- Where PDDL proves **no safe fallback exists** from a particular out-of-context state, this identifies an ODD gap that ISO 21448 requires to be addressed.
- The resolution options are: (a) extend the ODD boundary (Stage 1 update), (b) add hardware/software to make a fallback available, or (c) add an operational restriction preventing that state from being reached.
- The unreachable safe-state report is the primary driver for ODD refinement.

---

## ISO 34502 Impact

ISO 34502 combined risk factor analysis justifies modelling multi-factor out-of-context states:

- Single-factor excursions (GPS only, comms only) may be manageable.
- Multi-factor combinations (GPS + comms simultaneously) may not be — and must be explicitly analysed.
- ISO 34502's combinatorial approach to risk factors justifies the strategy of testing all pairwise (and higher-order) combinations of out-of-context predicates.

---

## Out-of-Context Reachability Pattern

```pddl
;; Out-of-context state: both GPS and comms lost
(:init
  (at uav-1 waypoint-remote)
  (gps-denied)           ;; context boundary exceeded
  (comms-lost)           ;; context boundary exceeded
  (inertial-nav-calibrated)
  (fuel-level-adequate))

;; Safe state goal: returned to base
(:goal (returned-to-base uav-1))

;; Fallback action available under these conditions
(:action return-to-base-inertial
  :precondition (and (gps-denied) (inertial-nav-calibrated))
  :effect (returned-to-base ?u))
```

If the planner finds a valid plan: safe state reachable → **fallback plan** added to library.
If the planner finds no plan: → entry in **unreachable safe-state report**.

---

## UAV Example

**Out-of-context condition:** `(comms-lost)` and `(gps-denied)` simultaneously true.

**Fallback planning result:** The planner finds a return-to-base plan via inertial navigation actions (`navigate-inertial`, `return-to-base-inertial`).

**GSN integration:**
- The fallback plan is the **GSN solution node** for the sub-goal "safe state reachable under simultaneous GPS/comms loss."
- If no plan exists for a particular combination, the unreachable safe-state report becomes a **GSN context node** triggering a requirement for additional mitigation (e.g. a hardware beacon for home-direction finding).

**Context boundary validation:** The set of boundary predicates (`gps-denied`, `comms-lost`, `wind-exceeds-limits`, `battery-critical`) is reviewed against the ODM to confirm all out-of-context conditions are captured.

---

## Evidence Checklist

- [ ] Operating context boundary predicates defined and reviewed against ODM
- [ ] Safe state definitions formalised as PDDL goals
- [ ] Fallback action set defined and encoded in domain
- [ ] Reachability analysis run for each single-factor out-of-context condition
- [ ] Reachability analysis run for each multi-factor combination (pairwise minimum)
- [ ] Fallback plan library populated for all reachable cases
- [ ] Unreachable safe-state report produced and reviewed
- [ ] Each unreachable case has a mitigation action or accepted residual risk
- [ ] ISO 21448 ODD excursion requirements addressed
- [ ] ISO 34502 combined risk factor combinations addressed

---

*Previous: [Stage 6 — Hazardous Failures Management](06_stage6_hazardous_failures.md)*
*Next: [Stage 8 — Verification Assurance](08_stage8_verification.md)*
