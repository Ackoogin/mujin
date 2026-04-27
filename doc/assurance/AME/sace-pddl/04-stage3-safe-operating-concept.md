# Stage 3: Safe Operating Concept Assurance

[Back to Index](00-index.md) | [Previous: Stage 2](03-stage2-hazardous-scenarios.md) | [Next: Stage 4](05-stage4-safety-requirements.md)

---

## SACE Purpose

Defines the Safe Operating Concept (SOC) -- the system-level safety requirements specifying how the AS must behave to mitigate each hazardous scenario.

## PDDL Contribution

Safety requirements are encoded as PDDL trajectory constraints (PDDL 3.0 and later) or goal-state invariants. The planner only produces plans satisfying these constraints, providing constructive evidence that the SOC is achievable within the model. This evidence is a proof of feasibility in the PDDL formalism, not a proof of real-world safety -- the gap between model and reality is captured in the assumption nodes below.

## Inputs

| Input | PDDL Mapping |
|-------|-------------|
| **Hazardous scenarios from Stage 2** | Each maps to a PDDL safety constraint |
| **Tolerable risk criteria** | Inform constraint strictness |
| **System safety requirements in natural language** | Formalised as PDDL trajectory constraints |
| **PDDL domain and problem files** | Extended with a `:constraints` section |

## Outputs

| Output | Description |
|--------|-------------|
| **Constrained PDDL domain and problem** | Domain with trajectory constraints encoding safety requirements |
| **Feasibility analysis** | Planner output demonstrating constraints are satisfiable alongside mission goals |
| **Constraint conflict report** | Identifies requirements making planning infeasible |
| **SOC-to-PDDL traceability** | Maps each safety requirement to its formal constraint |

## GSN Mapping -- Pattern [N]

Pattern [N] argues that the SOC sufficiently mitigates all identified hazardous scenarios.

| GSN Element | Content |
|-------------|---------|
| **Goal** | The SOC provides sufficient mitigation for all identified hazardous scenarios |
| **Strategy** | Argue over each hazardous scenario that its corresponding safety constraint is satisfiable and that constrained plans achieve mission goals. Decompose into a sub-goal per hazardous scenario |
| **Context** | The set of identified hazardous scenarios from Stage 2. The definition of "sufficiently safe". The SOC-to-PDDL traceability mapping |
| **Solution** | The feasibility analysis demonstrates that plans exist satisfying all constraints. The constraint conflict report (if any conflicts exist) provides evidence that specific requirement interactions have been resolved |
| **Justification** | ISO 21448 requires mitigation of known-unsafe scenarios; PDDL constraint satisfaction constitutes formal evidence of mitigation feasibility within the model. ISO 34502 safety test objectives map to specific PDDL constraints |
| **Assumption** | The PDDL trajectory constraints faithfully encode the natural-language safety requirements. Must be validated through review of the SOC-to-PDDL traceability |

## ISO 21448 (SOTIF) Impact

PDDL constraint satisfaction provides direct evidence for ISO 21448's requirement to demonstrate that residual risk from known-unsafe scenarios is acceptable. Where constraints conflict, this reveals specification insufficiency.

## ISO 34502 Impact

ISO 34502 safety test objectives map to PDDL constraints, enabling automatic pass/fail evaluation of plan traces.

## UAV Example

The constraint:

```pddl
(always (implication (not (obstacle-clear ?wp)) (not (at ?uav ?wp))))
```

encodes the requirement that the UAV shall not enter an uncleared waypoint. The planner proves missions are achievable under this constraint. The feasibility analysis becomes a GSN **solution** node supporting the SOC mitigation goal for the collision hazardous scenario.

---

[Next: Stage 4 -- Safety Requirements Assurance](05-stage4-safety-requirements.md)

