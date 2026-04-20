# Stage 4: Safety Requirements Assurance

[Back to Index](00-index.md) | [Previous: Stage 3](04-stage3-safe-operating-concept.md) | [Next: Stage 5](06-stage5-design-assurance.md)

---

## SACE Purpose

Decomposes system-level safety requirements to sub-system requirements and validates completeness, consistency, and traceability across design tiers.

## PDDL Contribution

PDDL supports hierarchical decomposition via multi-level domain modelling, where each sub-system gets its own domain file with localised constraints. Requirements at each tier are verified by checking that decomposed plans satisfy their local constraints. However, proving that sub-domain constraints *jointly imply* the system-level constraint requires additional tooling — a theorem prover or model checker (see [Tooling Analysis](13-tooling-analysis.md)). PDDL provides per-tier satisfaction evidence and traceability, not compositional correctness proofs.

## Inputs

| Input | PDDL Mapping |
|-------|-------------|
| **Constrained PDDL domain from Stage 3** | Top-tier requirements as trajectory constraints |
| **Sub-system architecture** | Decomposition boundaries for PDDL sub-domains |
| **Allocated safety requirements** | Assigned to specific PDDL action groups |
| **Interface definitions** | Shared PDDL predicates between sub-domains |

## Outputs

| Output | Description |
|--------|-------------|
| **Decomposed PDDL sub-domains** | Per-subsystem domain files with localised constraints |
| **Requirement satisfaction proofs** | Plan traces at each tier demonstrating constraint satisfaction |
| **Completeness analysis** | Traceability evidence that all system constraints have corresponding sub-domain constraints. Formal proof of joint implication requires a theorem prover or model checker |
| **Traceability matrix** | Links decomposed requirements to sub-domain predicates and constraints |

## GSN Mapping — Pattern [S]

Pattern [S] argues that safety requirements are correctly decomposed, complete, and traceable.

| GSN Element | Content |
|-------------|---------|
| **Goal** | Safety requirements at tier *n* are correctly decomposed to tier *n+1* and the decomposition preserves safety intent |
| **Strategy** | Argue that sub-domain constraints jointly imply system-level constraints. Decompose into sub-goals for completeness, consistency, and traceability |
| **Context** | The system architecture defining decomposition boundaries. The PDDL sub-domain structure |
| **Solution** | The completeness analysis provides traceability evidence that each system-level constraint maps to sub-domain constraints (formal proof of joint implication requires a model checker). Requirement satisfaction proofs demonstrate that plans at each tier satisfy their local constraints. The traceability matrix provides audit evidence |
| **Justification** | ISO 34502's three risk factor categories justify the decomposition structure, aligning sub-domains with perception, judgement, and control. ISO 21448 functional insufficiency tracing justifies the requirement to track which sub-system is responsible for each triggering condition |

## ISO 21448 (SOTIF) Impact

PDDL sub-domain decomposition makes functional insufficiency traceability explicit: each sub-domain's preconditions represent performance assumptions that, if violated, are functional insufficiencies.

## ISO 34502 Impact

The three risk factor categories provide a natural sub-domain decomposition structure.

## UAV Example

The system constraint "always maintain safe separation" decomposes to:

- **Perception** sub-domain: obstacle-detection latency constraints
- **Planning** sub-domain: minimum waypoint spacing constraints
- **Control** sub-domain: trajectory tracking error constraints

The GSN **solution** nodes are the requirement satisfaction proofs at each sub-domain level. An ISO 21448 functional insufficiency in perception maps to a specific **assumption** node in the perception sub-domain argument.

---

[Next: Stage 5 — Design Assurance](06-stage5-design-assurance.md)

