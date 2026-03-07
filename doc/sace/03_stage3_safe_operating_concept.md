# SACE PDDL Contributions — Stage 3: Safe Operating Concept Assurance

*Part of the [SACE PDDL Contributions](00_standards_context.md) document set.*

**GSN Pattern: [N] — SOC Assurance**

---

## SACE Purpose

Defines the Safe Operating Concept (SOC) — the system-level safety requirements specifying how the AS must behave to mitigate each hazardous scenario.

---

## PDDL Contribution

Safety requirements are encoded as PDDL trajectory constraints (PDDL 3.0 and later) or goal-state invariants:

- The planner only produces plans satisfying these constraints, providing **constructive proof** that the SOC is achievable.
- Where constraints conflict, this reveals **specification insufficiency** before implementation.
- The feasibility analysis demonstrates that the SOC does not make mission goals unachievable.

---

## Inputs

| Input | How it informs PDDL |
|-------|---------------------|
| Hazardous scenarios from Stage 2 | Each maps to a PDDL safety constraint |
| Tolerable risk criteria | Inform constraint strictness |
| System safety requirements (natural language) | Formalised as PDDL trajectory constraints |
| PDDL domain and problem files | Extended with a `:constraints` section |

---

## Outputs

| Output | Description |
|--------|-------------|
| **Constrained PDDL domain and problem** | Domain with trajectory constraints encoding safety requirements |
| **Feasibility analysis** | Planner output demonstrating constraints are satisfiable alongside mission goals |
| **Constraint conflict report** | Identifies requirements making planning infeasible |
| **SOC-to-PDDL traceability** | Maps each safety requirement to its formal constraint |

---

## GSN Mapping — Pattern [N]

Pattern [N] argues that the SOC sufficiently mitigates all identified hazardous scenarios.

| GSN Element | Content | PDDL Role |
|-------------|---------|-----------|
| **Goal** | The SOC provides sufficient mitigation for all identified hazardous scenarios | Target |
| **Strategy** | Argue over each hazardous scenario that its corresponding safety constraint is satisfiable and constrained plans achieve mission goals; decompose into a sub-goal per hazardous scenario | Constraint satisfaction as strategy |
| **Context** | Set of identified hazardous scenarios from Stage 2; definition of "sufficiently safe"; SOC-to-PDDL traceability mapping | Constraint set as context |
| **Solution** | Feasibility analysis (plans exist satisfying all constraints); constraint conflict report (specific requirement interactions resolved) | PDDL artefacts as solution nodes |
| **Justification** | ISO 21448 requires mitigation of known-unsafe scenarios; PDDL constraint satisfaction constitutes formal mitigation evidence; ISO 34502 safety test objectives map to specific constraints | Standards as justification nodes |
| **Assumption** | The PDDL trajectory constraints faithfully encode the natural-language safety requirements — validated through review of the SOC-to-PDDL traceability | Constraint fidelity assumption |

---

## ISO 21448 (SOTIF) Impact

PDDL constraint satisfaction provides direct evidence for ISO 21448's requirement to demonstrate that residual risk from known-unsafe scenarios is acceptable:

- Each satisfied constraint is formal evidence that the corresponding known-unsafe scenario is mitigated.
- Where constraints conflict, this reveals **specification insufficiency** — a gap in the intended functionality that ISO 21448 requires to be resolved before deployment.
- The constrained planning approach operationalises ISO 21448's mitigation evidence requirement.

---

## ISO 34502 Impact

ISO 34502 safety test objectives map to PDDL constraints, enabling automatic pass/fail evaluation of plan traces:

- Each ISO 34502 test objective is formalised as a trajectory constraint.
- Planner output constitutes automated test execution against the objective.
- Failed planning (infeasibility) corresponds to a test objective that cannot be met.

---

## PDDL Trajectory Constraint Pattern

PDDL 3.0 trajectory constraints express safety properties over the entire plan:

```pddl
(:constraints
  ;; UAV shall not enter an uncleared waypoint at any point in the plan
  (always
    (implication (not (obstacle-clear ?wp))
                 (not (at ?uav ?wp))))

  ;; UAV shall not leave the defined airspace zone
  (always
    (in-airspace ?uav defined-zone))

  ;; UAV shall maintain minimum separation from other agents
  (always
    (forall (?other - uav)
      (implication (not (= ?uav ?other))
                   (safe-separation ?uav ?other))))
)
```

Each constraint maps to a named safety requirement in the SOC-to-PDDL traceability matrix.

---

## UAV Example

**Safety requirement:** "The UAV shall not enter an uncleared waypoint."

**PDDL encoding:**
```pddl
(always (implication (not (obstacle-clear ?wp)) (not (at ?uav ?wp))))
```

**Feasibility analysis:** The planner produces a valid mission plan that:
- Reaches the mission goal (target waypoint)
- Satisfies the obstacle-clearance constraint throughout

This feasibility analysis becomes a **GSN solution node** supporting the SOC mitigation goal for the collision hazardous scenario.

**If constraint conflict arises:** The conflict report identifies that the clearance constraint and a time-critical delivery goal are mutually incompatible in certain weather states, revealing an ODD restriction that must be added to Stage 1.

---

## Evidence Checklist

- [ ] Each identified hazardous scenario from Stage 2 mapped to a PDDL trajectory constraint
- [ ] SOC-to-PDDL traceability matrix produced and reviewed
- [ ] Feasibility analysis run for each mission scenario
- [ ] Constraint conflict report produced (empty or resolved)
- [ ] ISO 21448 mitigation evidence documented for each known-unsafe scenario
- [ ] ISO 34502 safety test objectives formalised as PDDL constraints
- [ ] Constraint fidelity assumption documented and reviewed

---

*Previous: [Stage 2 — Hazardous Scenarios Identification](02_stage2_hazardous_scenarios.md)*
*Next: [Stage 4 — Safety Requirements Assurance](04_stage4_safety_requirements.md)*
