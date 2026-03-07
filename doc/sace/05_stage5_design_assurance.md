# SACE PDDL Contributions — Stage 5: Design Assurance

*Part of the [SACE PDDL Contributions](00_standards_context.md) document set.*

**GSN Pattern: [X] — Design Assurance**

---

## SACE Purpose

Creates and justifies the AS design at each tier, including architecture decisions around redundancy, diversity, and fault tolerance.

---

## PDDL Contribution

The PDDL domain structure mirrors the system architecture:

- Design assurance uses PDDL to verify that the architecture satisfies all safety constraints under both **nominal and degraded configurations**.
- **Redundancy** is modelled by alternative action sets — if the primary action path is unavailable (due to a failure predicate), an alternative action achieves the same effect.
- **Degraded mode plans** demonstrate that safety constraints remain satisfiable even with reduced capabilities.

---

## Inputs

| Input | How it informs PDDL |
|-------|---------------------|
| Decomposed PDDL sub-domains from Stage 4 | Per-subsystem action and predicate scope |
| Architecture description (SysML IBDs/BDDs) | Mapped to PDDL domain structure |
| Redundancy and diversity strategy | Alternative PDDL action variants |
| Degraded mode definitions | Predicate states disabling primary actions |

---

## Outputs

| Output | Description |
|--------|-------------|
| **Architecture-mapped PDDL domain** | Domain structure traceable to SysML architecture |
| **Degraded-mode plan traces** | Safety constraints satisfied with reduced capabilities |
| **Redundancy verification evidence** | Plans achievable via alternative action paths |
| **Design justification records** | PDDL analysis results linked to architecture decisions |

---

## GSN Mapping — Pattern [X]

Pattern [X] argues that the design at tier *n* satisfies the allocated safety requirements, including under degraded conditions.

| GSN Element | Content | PDDL Role |
|-------------|---------|-----------|
| **Goal** | The design at tier *n* satisfies the allocated safety requirements, including under degraded conditions | Target |
| **Strategy** | Argue over nominal and degraded configurations that safety constraints remain satisfiable; decompose into sub-goals per redundancy path | Degraded-mode planning as strategy |
| **Context** | The SysML architecture; the degraded mode definitions | Architecture as context |
| **Solution** | Degraded-mode plan traces (constraint satisfaction under each degraded configuration); redundancy verification evidence (alternative action paths exist) | PDDL artefacts as solution nodes |
| **Justification** | ISO 21448 design improvement requirement justifies demonstrating constraint satisfaction under progressively degraded conditions; ISO 34502 Annex D control disturbances justify the degraded mode definitions | Standards as justification nodes |

---

## ISO 21448 (SOTIF) Impact

Each redundant action path **shrinks the unsafe scenario space**, providing ISO 21448 design improvement evidence:

- With one sensor/actuator path, certain failure states reach unsafe scenarios (known-unsafe).
- Adding a redundant path means the planner can reach safe states via the alternative, moving scenarios from known-unsafe to known-safe.
- The design improvement argument quantifies this reduction using PDDL reachability analysis before and after redundancy is added.

---

## ISO 34502 Impact

ISO 34502 Annex D control disturbances directly inform degraded-mode PDDL modelling:

| Annex D Disturbance | PDDL Degraded-Mode Predicate | Degraded Action Available |
|--------------------|------------------------------|--------------------------|
| GPS signal loss | `(gps-denied)` | `navigate-inertial-only` |
| Primary actuator failure | `(primary-actuator-failed)` | `navigate-backup-actuator` |
| Communication link degraded | `(comms-degraded)` | `execute-autonomous-plan` |
| Control authority exceeded | `(wind-exceeds-authority)` | `loiter` or `emergency-land` |

---

## Redundancy Modelling Pattern

Primary and backup action variants share effects but have mutually exclusive preconditions:

```pddl
;; Primary navigation (GPS-aided)
(:action navigate-gps
  :parameters (?u - uav ?from ?to - waypoint)
  :precondition (and (at ?u ?from) (gps-available) (obstacle-clear ?to))
  :effect (and (at ?u ?to) (not (at ?u ?from))))

;; Backup navigation (inertial only — available when GPS denied)
(:action navigate-inertial
  :parameters (?u - uav ?from ?to - waypoint)
  :precondition (and (at ?u ?from) (gps-denied) (obstacle-clear ?to)
                     (inertial-nav-calibrated))
  :effect (and (at ?u ?to) (not (at ?u ?from))))
```

The planner is run with `(gps-denied)` in the initial state. If a valid constrained plan is found via `navigate-inertial`, this is the **redundancy verification evidence**.

---

## UAV Example

**Degraded mode:** GPS-denied operation.

**PDDL initial state includes:** `(gps-denied)`

**Result:** The planner finds a return-to-base plan via `navigate-inertial` actions, satisfying:
- The position accuracy constraint (relaxed from 2m to 5m under inertial nav)
- The obstacle-avoidance constraint (maintained)
- The safe-state-reachability constraint

**GSN integration:**
- The degraded-mode plan trace is a **GSN solution node** for the design assurance sub-goal "design is safe under GPS denial."
- The GPS-denial degraded mode definition is a **GSN context node** referencing ISO 34502 Annex D.

---

## Evidence Checklist

- [ ] PDDL domain structure mapped to SysML architecture components
- [ ] Degraded mode predicates defined for each failure mode from FMEA
- [ ] Redundant action variants implemented in PDDL domain
- [ ] Degraded-mode plan traces produced for each degraded configuration
- [ ] Nominal plan traces produced for comparison
- [ ] ISO 34502 Annex D control disturbances addressed in degraded mode set
- [ ] Architecture-to-PDDL traceability documented

---

*Previous: [Stage 4 — Safety Requirements Assurance](04_stage4_safety_requirements.md)*
*Next: [Stage 6 — Hazardous Failures Management](06_stage6_hazardous_failures.md)*
