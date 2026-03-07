# SACE PDDL Contributions — Cross-Stage Summary

*Part of the [SACE PDDL Contributions](00_standards_context.md) document set.*

---

## Overview

This document summarises each stage's primary PDDL artefact, its GSN role, and its contribution to ISO 21448 (SOTIF) and ISO 34502. It serves as a quick-reference index across the full document set.

---

## Stage-by-Stage Summary

### Stage 1: Operating Context Assurance — Pattern [G]

| Dimension | Content |
|-----------|---------|
| **Primary PDDL artefact** | Domain file (capabilities) and problem files (scenarios) |
| **GSN role** | Context node (formal operating context); solution nodes (ODM traceability, scenario coverage report) |
| **ISO 21448 contribution** | ODD encoding; functional insufficiency visibility through precondition gaps |
| **ISO 34502 contribution** | Risk factor category coverage in domain structure |

*Detail: [01_stage1_operating_context.md](01_stage1_operating_context.md)*

---

### Stage 2: Hazardous Scenarios Identification — Pattern [I]

| Dimension | Content |
|-----------|---------|
| **Primary PDDL artefact** | Fault-injected domain variants; counterexample plan traces; reachability analysis report |
| **GSN role** | Solution nodes supporting hazard completeness goal (reachability analysis, counterexample traces, decision-point register) |
| **ISO 21448 contribution** | Unknown-unsafe scenario discovery through state-space exploration |
| **ISO 34502 contribution** | Annex B, C, D scenario generation as structured fault-injection framework |

*Detail: [02_stage2_hazardous_scenarios.md](02_stage2_hazardous_scenarios.md)*

---

### Stage 3: Safe Operating Concept Assurance — Pattern [N]

| Dimension | Content |
|-----------|---------|
| **Primary PDDL artefact** | Trajectory constraints; feasibility analysis; SOC-to-PDDL traceability |
| **GSN role** | Solution nodes (feasibility proofs) supporting the SOC mitigation goal |
| **ISO 21448 contribution** | Known-unsafe scenario mitigation proof through constrained planning |
| **ISO 34502 contribution** | Safety test objective encoding as PDDL constraints |

*Detail: [03_stage3_safe_operating_concept.md](03_stage3_safe_operating_concept.md)*

---

### Stage 4: Safety Requirements Assurance — Pattern [S]

| Dimension | Content |
|-----------|---------|
| **Primary PDDL artefact** | Decomposed sub-domains; requirement satisfaction proofs; completeness analysis |
| **GSN role** | Solution nodes (completeness analysis, satisfaction proofs) supporting the decomposition goal |
| **ISO 21448 contribution** | Functional insufficiency traceability to responsible sub-system |
| **ISO 34502 contribution** | Risk factor category decomposition alignment (perception/judgement/control sub-domains) |

*Detail: [04_stage4_safety_requirements.md](04_stage4_safety_requirements.md)*

---

### Stage 5: Design Assurance — Pattern [X]

| Dimension | Content |
|-----------|---------|
| **Primary PDDL artefact** | Architecture-mapped domain with alternative action variants; degraded-mode plan traces |
| **GSN role** | Solution nodes (degraded-mode traces, redundancy evidence) supporting the design goal |
| **ISO 21448 contribution** | Design improvement evidence through progressive degradation analysis |
| **ISO 34502 contribution** | Annex D control disturbance coverage in degraded mode definitions |

*Detail: [05_stage5_design_assurance.md](05_stage5_design_assurance.md)*

---

### Stage 6: Hazardous Failures Management — Pattern [DD]

| Dimension | Content |
|-----------|---------|
| **Primary PDDL artefact** | Failure-mode PDDL variants; hazardous failure register; mitigation re-verification evidence |
| **GSN role** | Solution nodes (failure-mode analyses, mitigation evidence); assumption nodes (failure mode completeness); away goals for AMLAS cases |
| **ISO 21448 contribution** | Functional insufficiency as hazardous failure; component-level SOTIF evidence; AMLAS linkage |
| **ISO 34502 contribution** | Annex C (perception) and Annex D (control) disturbances populating the failure register |

*Detail: [06_stage6_hazardous_failures.md](06_stage6_hazardous_failures.md)*

---

### Stage 7: Out of Context Operation — Pattern [PP]

| Dimension | Content |
|-----------|---------|
| **Primary PDDL artefact** | Fallback plan library; out-of-context reachability analysis; unreachable safe-state report |
| **GSN role** | Solution nodes (reachability proofs, fallback plans) supporting the safe-state goal |
| **ISO 21448 contribution** | ODD excursion handling — unreachable states drive ODD refinement |
| **ISO 34502 contribution** | Combined risk factor analysis for multi-factor out-of-context states |

*Detail: [07_stage7_out_of_context.md](07_stage7_out_of_context.md)*

---

### Stage 8: Verification Assurance — Pattern [UU]

| Dimension | Content |
|-----------|---------|
| **Primary PDDL artefact** | Plan validity certificates; state-space coverage reports; model-to-implementation traceability |
| **GSN role** | Solution nodes (certificates, coverage reports, property results); justification nodes (verification strategy rationale) |
| **ISO 21448 contribution** | Known-unsafe mitigation evidence; unknown-unsafe coverage bounding |
| **ISO 34502 contribution** | Annex-by-annex scenario verification coverage; Annex K randomised problem generation |

*Detail: [08_stage8_verification.md](08_stage8_verification.md)*

---

### Residual Risk Management (Cross-cutting)

| Dimension | Content |
|-----------|---------|
| **Primary PDDL artefact** | Coverage metrics; fault-injected variant results; misuse scenario analyses; response verification traces |
| **GSN role** | Solution nodes (coverage, response verification); assumption nodes (model fidelity, insufficiency completeness); context nodes (acceptance criteria) |
| **ISO 21448 contribution** | Core SOTIF activity: known-unsafe mitigation evidence + unknown-unsafe coverage bounding; misuse triggering conditions |
| **ISO 34502 contribution** | Risk factor completeness checklist; Annex K randomised exploration methodology |

*Detail: [09_residual_risk.md](09_residual_risk.md)*

---

## Key Relationships

### GSN ↔ PDDL

GSN is the structural backbone of the safety case. It defines what claims must be made and what evidence is required. PDDL generates that evidence in a formal, machine-checkable form.

```
GSN Goal nodes    ←— define —  what must be proven
GSN Strategy nodes ←— framed by — ISO standards
GSN Context nodes  ←— populated by — PDDL domain files
GSN Solution nodes ←— populated by — PDDL plan traces and analyses
GSN Assumption nodes ←— require — PDDL model fidelity confidence arguments
```

### ISO 21448 ↔ PDDL State Space

ISO 21448's four-quadrant model maps directly onto PDDL exploration:

```
Known-safe     = PDDL states satisfying all constraints (plan validity certificates)
Known-unsafe   = States reached by counterexample traces (Stage 2 fault injection)
Unknown-safe   = Explored PDDL states without counterexamples (coverage report)
Unknown-unsafe = Unexplored PDDL state space (residual bounded by coverage metrics)
```

The SACE process systematically shrinks the unknown-unsafe quadrant through:
1. Stage 2: Fault injection discovers new known-unsafe states
2. Stage 3: Constraint addition mitigates known-unsafe states → known-safe
3. Stage 8: Coverage exploration bounds the unknown-unsafe residual

### ISO 34502 ↔ PDDL Domain Structure

ISO 34502 provides the structured scenario derivation that ensures PDDL exploration is systematic:

```
Annex B → Multi-agent interaction predicates and fault variants
Annex C → Perception failure predicates and sensor degradation variants
Annex D → Control failure predicates and actuator degradation variants
Annex K → Randomised PDDL problem generation strategy
```

Its annexes provide the scenario templates encoded as PDDL problem file families.

---

## The Three-Layer Argument

Together, the three frameworks form a coherent, layered argument:

| Layer | Framework | Role |
|-------|-----------|------|
| **Structure** | SACE (GSN argument patterns) | Defines what must be argued and in what order |
| **Conceptual grounding** | ISO 21448 / ISO 34502 | Populates strategy, context, and justification nodes — explains *why* the argument decomposition is sound |
| **Formal evidence** | PDDL | Generates the formally verified evidence that populates solution nodes |

The result is a safety case that is:
- **Structured** — through SACE GSN argument patterns
- **Conceptually grounded** — through ISO 21448 and ISO 34502
- **Formally evidenced** — through PDDL planning analysis

---

## PDDL Artefact Register

The following table provides a complete register of PDDL artefacts and their GSN roles across all stages:

| Artefact | Produced in Stage | GSN Element Type | Pattern |
|----------|------------------|-----------------|---------|
| PDDL domain file | 1 | Context | [G] |
| PDDL problem files | 1 | Solution (coverage) | [G] |
| ODM-to-PDDL traceability matrix | 1 | Solution (completeness) | [G] |
| Reachability analysis report | 2 | Solution | [I] |
| Fault-injected domain variants | 2, 6 | Solution | [I], [DD] |
| Counterexample plan traces | 2 | Solution | [I] |
| Decision-point register | 2 | Solution | [I] |
| Trajectory constraints | 3 | Context | [N] |
| Feasibility analysis | 3 | Solution | [N] |
| SOC-to-PDDL traceability | 3 | Solution | [N] |
| Decomposed sub-domains | 4 | Context | [S] |
| Requirement satisfaction proofs | 4, 8 | Solution | [S], [UU] |
| Completeness analysis | 4 | Solution | [S] |
| Architecture-mapped domain | 5 | Context | [X] |
| Degraded-mode plan traces | 5 | Solution | [X] |
| Redundancy verification evidence | 5 | Solution | [X] |
| Failure-mode PDDL variants | 6 | Solution | [DD] |
| Hazardous failure register | 6 | Solution | [DD] |
| Mitigation re-verification evidence | 6 | Solution | [DD] |
| Out-of-context reachability analysis | 7 | Solution | [PP] |
| Fallback plan library | 7 | Solution | [PP] |
| Unreachable safe-state report | 7 | Solution/Context | [PP] |
| Plan validity certificates | 8 | Solution | [UU] |
| State-space coverage report | 8, RRM | Solution | [UU], top-level |
| Model-to-implementation traceability | 8 | Solution | [UU] |
| Misuse scenario analyses | RRM | Solution | top-level |
| Response verification traces | RRM | Solution | top-level |

*RRM = Residual Risk Management*

---

*Return to: [Standards Context and Document Index](00_standards_context.md)*
