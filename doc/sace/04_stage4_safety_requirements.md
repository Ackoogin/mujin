# SACE PDDL Contributions — Stage 4: Safety Requirements Assurance

*Part of the [SACE PDDL Contributions](00_standards_context.md) document set.*

**GSN Pattern: [S] — Safety Requirements Assurance**

---

## SACE Purpose

Decomposes system-level safety requirements to sub-system requirements and validates completeness, consistency, and traceability across design tiers.

---

## PDDL Contribution

PDDL supports hierarchical decomposition via HTN extensions or multi-level domain modelling:

- Requirements at each tier are verified by checking that decomposed plans satisfy constraints from the tier above.
- **Sub-domains** per subsystem (perception, planning, control) each carry localised constraints that jointly imply the system-level constraints.
- The decomposition is formally verifiable: if each sub-domain's constraints are satisfiable and their conjunction implies the system constraint, decomposition is complete and consistent.

---

## Inputs

| Input | How it informs PDDL |
|-------|---------------------|
| Constrained PDDL domain from Stage 3 | Top-tier requirements as trajectory constraints |
| Sub-system architecture | Decomposition boundaries for PDDL sub-domains |
| Allocated safety requirements | Assigned to specific PDDL action groups |
| Interface definitions | Shared PDDL predicates between sub-domains |

---

## Outputs

| Output | Description |
|--------|-------------|
| **Decomposed PDDL sub-domains** | Per-subsystem domain files with localised constraints |
| **Requirement satisfaction proofs** | Plan traces at each tier demonstrating constraint satisfaction |
| **Completeness analysis** | Sub-domain constraints jointly imply system constraints |
| **Traceability matrix** | Links decomposed requirements to sub-domain predicates and constraints |

---

## GSN Mapping — Pattern [S]

Pattern [S] argues that safety requirements are correctly decomposed, complete, and traceable.

| GSN Element | Content | PDDL Role |
|-------------|---------|-----------|
| **Goal** | Safety requirements at tier *n* are correctly decomposed to tier *n+1* and the decomposition preserves safety intent | Target |
| **Strategy** | Argue that sub-domain constraints jointly imply system-level constraints; decompose into sub-goals for completeness, consistency, and traceability | Formal implication as strategy |
| **Context** | The system architecture defining decomposition boundaries; the PDDL sub-domain structure | Sub-domain structure as context |
| **Solution** | Completeness analysis (no system-level constraint lost in decomposition); requirement satisfaction proofs (plans at each tier satisfy local constraints); traceability matrix | PDDL artefacts as solution nodes |
| **Justification** | ISO 34502 three risk factor categories justify decomposition structure aligning with perception, judgement, and control; ISO 21448 functional insufficiency tracing justifies tracking which sub-system is responsible for each triggering condition | Standards as justification nodes |

---

## ISO 21448 (SOTIF) Impact

PDDL sub-domain decomposition makes functional insufficiency traceability explicit:

- Each sub-domain's **preconditions represent performance assumptions** that, if violated, are functional insufficiencies.
- The traceability matrix maps each functional insufficiency to the specific sub-system responsible for satisfying its precondition.
- This enables Stage 6 hazardous failure analysis to identify which component's failure introduces a specific insufficiency.

---

## ISO 34502 Impact

The three ISO 34502 risk factor categories provide a natural sub-domain decomposition structure:

| ISO 34502 Category | Corresponding Sub-domain | Key Predicates |
|-------------------|--------------------------|----------------|
| Perception disturbances | Perception sub-domain | `obstacle-detected`, `comms-available`, `gps-lock` |
| Judgement disturbances | Planning sub-domain | `plan-valid`, `goal-achievable`, `waypoint-spacing` |
| Control disturbances | Control sub-domain | `trajectory-within-bounds`, `actuator-responsive` |

---

## Multi-Tier Decomposition Pattern

System-level constraint decomposes to sub-domain constraints:

```
System level:
  (always (safe-separation ?uav ?other))

Decomposes to:
  Perception sub-domain:
    (always (detection-latency-within-bound ?sensor))
  Planning sub-domain:
    (always (minimum-waypoint-spacing ?plan))
  Control sub-domain:
    (always (tracking-error-within-bound ?uav))

Completeness check:
  {detection-latency ∧ min-spacing ∧ tracking-error} ⊨ safe-separation
```

The completeness analysis verifies this implication holds for all reachable states.

---

## UAV Example

**System constraint:** "The UAV shall always maintain safe separation from other agents."

**Decomposition:**

| Sub-domain | Allocated Constraint | PDDL Encoding |
|-----------|---------------------|---------------|
| Perception | Obstacle detection latency ≤ T ms | `(always (detection-latency-within-bound obstacle-detector))` |
| Planning | Minimum waypoint spacing ≥ D metres | `(always (min-separation-maintained ?plan))` |
| Control | Trajectory tracking error ≤ E metres | `(always (tracking-error-within-bound ?uav))` |

**GSN solution nodes:**
- Requirement satisfaction proofs at each sub-domain level (plan traces satisfying local constraints)
- Completeness analysis showing the conjunction implies the system constraint

**ISO 21448 functional insufficiency mapping:** A perception insufficiency (e.g. camera failure under glare) maps to a specific assumption node in the perception sub-domain argument, identifying which sub-system is responsible.

---

## Evidence Checklist

- [ ] System-level constraints from Stage 3 allocated to sub-domains
- [ ] PDDL sub-domain files created per subsystem
- [ ] Requirement satisfaction proofs produced at each tier
- [ ] Completeness analysis performed (sub-domain constraints ⊨ system constraints)
- [ ] Traceability matrix linking requirements to sub-domain predicates produced
- [ ] ISO 34502 risk factor categories addressed in sub-domain structure
- [ ] ISO 21448 functional insufficiency traceability documented per sub-domain

---

*Previous: [Stage 3 — Safe Operating Concept Assurance](03_stage3_safe_operating_concept.md)*
*Next: [Stage 5 — Design Assurance](05_stage5_design_assurance.md)*
