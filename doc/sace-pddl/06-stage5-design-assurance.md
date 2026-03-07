# Stage 5: Design Assurance

[Back to Index](00-index.md) | [Previous: Stage 4](05-stage4-safety-requirements.md) | [Next: Stage 6](07-stage6-hazardous-failures.md)

---

## SACE Purpose

Creates and justifies the AS design at each tier, including architecture decisions around redundancy, diversity, and fault tolerance.

## PDDL Contribution

The PDDL domain structure mirrors the system architecture. Design assurance uses PDDL to verify that the architecture satisfies all safety constraints under both nominal and degraded configurations. Redundancy is modelled by alternative action sets.

## Inputs

| Input | PDDL Mapping |
|-------|-------------|
| **Decomposed PDDL sub-domains from Stage 4** | Per-subsystem action and predicate scope |
| **Architecture description** | SysML IBDs and BDDs mapped to PDDL domain structure |
| **Redundancy and diversity strategy** | Alternative PDDL action variants |
| **Degraded mode definitions** | Predicate states disabling primary actions |

## Outputs

| Output | Description |
|--------|-------------|
| **Architecture-mapped PDDL domain** | Domain structure traceable to SysML architecture |
| **Degraded-mode plan traces** | Safety constraints satisfied with reduced capabilities |
| **Redundancy verification evidence** | Plans achievable via alternative action paths |
| **Design justification records** | PDDL analysis results linked to architecture decisions |

## GSN Mapping — Pattern [X]

| GSN Element | Content |
|-------------|---------|
| **Goal** | The design at tier *n* satisfies the allocated safety requirements, including under degraded conditions |
| **Strategy** | Argue over nominal and degraded configurations that safety constraints remain satisfiable. Decompose into sub-goals per redundancy path |
| **Context** | The SysML architecture. The degraded mode definitions |
| **Solution** | Degraded-mode plan traces demonstrate constraint satisfaction under each degraded configuration. Redundancy verification evidence demonstrates that alternative action paths exist |
| **Justification** | ISO 21448 design improvement requirement justifies demonstrating constraint satisfaction under progressively degraded conditions. ISO 34502 Annex D control disturbances justify the degraded mode definitions |

## ISO 21448 (SOTIF) Impact

Each redundant action path shrinks the unsafe scenario space, providing ISO 21448 design improvement evidence.

## ISO 34502 Impact

Annex D control disturbances directly inform degraded-mode PDDL modelling.

## UAV Example

Under `(gps-denied)` the planner proves safe-state reachability via inertial-only actions. The degraded-mode plan trace is a GSN **solution** node. The GPS-denial degraded mode definition is a GSN **context** node referencing ISO 34502 Annex D.

---

[Next: Stage 6 — Hazardous Failures Management](07-stage6-hazardous-failures.md)
