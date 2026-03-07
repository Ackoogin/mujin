# SACE PDDL Contributions — 00: Standards Context & GSN Overview

**PDDL Contributions per SACE Stage with GSN Argument Mapping, ISO 21448 (SOTIF) and ISO 34502**
*Draft — March 2026*

---

## Overview

This document set maps the contributions of PDDL (Planning Domain Definition Language) formal models to each of the eight stages of the SACE methodology. For each stage it identifies the role PDDL plays, the inputs consumed, the outputs produced, how ISO 21448 (SOTIF) and ISO 34502 affect the modelling approach, and how the PDDL artefacts feed into the Goal Structuring Notation (GSN) argument patterns that SACE prescribes. Examples are oriented towards autonomous UAV systems.

### Document Index

| File | Contents |
|------|----------|
| `00_standards_context.md` | This file — standards overview, GSN elements, interaction model |
| `01_stage1_operating_context.md` | Stage 1: Operating Context Assurance |
| `02_stage2_hazardous_scenarios.md` | Stage 2: Hazardous Scenarios Identification |
| `03_stage3_safe_operating_concept.md` | Stage 3: Safe Operating Concept Assurance |
| `04_stage4_safety_requirements.md` | Stage 4: Safety Requirements Assurance |
| `05_stage5_design_assurance.md` | Stage 5: Design Assurance |
| `06_stage6_hazardous_failures.md` | Stage 6: Hazardous Failures Management |
| `07_stage7_out_of_context.md` | Stage 7: Out of Context Operation Assurance |
| `08_stage8_verification.md` | Stage 8: Verification Assurance |
| `09_residual_risk.md` | Residual Risk Management (cross-cutting) |
| `10_cross_stage_summary.md` | Cross-stage summary and key relationships |

---

## Standards Context

**SACE** (University of York) provides the overarching safety case structure for autonomous systems in complex environments, with eight iterative stages producing artefacts and GSN argument patterns.

**ISO 21448 (SOTIF)** addresses the safety of the intended functionality — hazards arising not from system faults but from functional insufficiencies and performance limitations, even when the system operates as designed. It introduces the four-quadrant scenario model (known-safe, known-unsafe, unknown-safe, unknown-unsafe) and requires systematic identification of triggering conditions.

**ISO 34502** provides a scenario-based safety evaluation framework that derives critical scenarios by systematically combining risk factors across three categories: perception disturbances, traffic/judgement disturbances, and vehicle control disturbances.

**GSN** (Goal Structuring Notation) is the graphical argumentation notation used throughout SACE to document safety cases. GSN captures goals (safety claims), strategies (reasoning steps), solutions (evidence), contexts (scoping information), assumptions, and justifications in a structured, auditable hierarchy.

---

## How These Standards and Notations Interact

SACE provides the safety case structure expressed as GSN argument patterns. ISO 21448 provides the conceptual framework for functional insufficiency analysis and the four-quadrant scenario classification. ISO 34502 provides the systematic scenario derivation methodology. PDDL provides the formal, executable modelling language that generates the evidence populating the GSN argument.

The relationship between PDDL and GSN is one of **evidence generation to argument structure**. PDDL artefacts do not replace GSN; rather, they provide the formally verified evidence that instantiates GSN solution nodes, while the ISO standards inform the strategy, context, and justification nodes that explain why the argument decomposition is sound.

```
ISO 21448 / ISO 34502
    │
    ▼ (inform strategy, context, justification nodes)
GSN Argument Patterns (SACE)
    │
    ▼ (populated by)
PDDL Artefacts (formal evidence → solution nodes)
```

---

## GSN Elements

GSN uses a small set of graphical elements to build structured safety arguments. Understanding these is essential for seeing how PDDL artefacts fit into the safety case.

| Element | Notation | Description | PDDL role |
|---------|----------|-------------|-----------|
| **Goal** | Rectangle | A claim about the system that must be shown to be true. Goals decompose into sub-goals. | Target of PDDL evidence |
| **Strategy** | Parallelogram | The reasoning step that explains how a goal is decomposed into sub-goals. | PDDL-based verification strategies |
| **Solution** | Circle | A reference to an item of evidence that directly supports a goal. | Primary role of PDDL artefacts |
| **Context** | Rounded rectangle | Information that scopes or qualifies a goal or strategy. | PDDL domain files as formal context |
| **Assumption** | Oval (A) | A statement taken to be true without proof. | PDDL abstraction assumptions |
| **Justification** | Oval (J) | The rationale for a strategy. | ISO standard references |
| **Assurance Claim Point** | Black square | Marks where additional confidence arguments are required (characteristic of SACE). | Points requiring PDDL model confidence arguments |

---

## SACE Argument Pattern Structure

SACE defines a top-level safety argument that decomposes into two principal claims: that the AS is safe within its defined operating context, and that it remains safe when outside that context. The first claim decomposes further through an argument that all hazardous scenarios have been identified and sufficiently mitigated.

The SACE argument patterns are:

| Pattern | Name | SACE Stage |
|---------|------|------------|
| **[G]** | Operating Context Assurance | Stage 1 |
| **[I]** | Hazardous Scenarios Identification | Stage 2 |
| **[N]** | SOC Assurance | Stage 3 |
| **[S]** | Safety Requirements Assurance | Stage 4 |
| **[X]** | Design Assurance | Stage 5 |
| **[DD]** | Hazardous Failures Management | Stage 6 |
| **[PP]** | Out of Context Operation | Stage 7 |
| **[UU]** | Verification Assurance | Stage 8 |

---

## How PDDL Artefacts Map to GSN Elements

### PDDL as Solution nodes
Plan traces, constraint satisfaction proofs, state-space coverage reports, and reachability analyses are all items of evidence that attach as solutions to specific goals in the SACE patterns.

### PDDL as Context nodes
The PDDL domain file itself serves as a context node, providing the formal definition of autonomous capabilities and operating context that scopes the argument.

### PDDL informing Strategy nodes
The choice to use formal planning-based verification as part of the safety argument is itself a strategy. This strategy node would be justified by reference to ISO 34502's scenario-based evaluation approach and ISO 21448's requirement for systematic scenario exploration.

### PDDL generating Assumption nodes
Every PDDL model involves abstraction assumptions: that the predicate set is sufficient, that action models are faithful, that the state space is adequately bounded. These must be explicitly captured as GSN assumption nodes, with associated confidence arguments explaining why each assumption is reasonable.

### ISO standards as Strategy/Context/Justification nodes
The ISO standards primarily populate strategy, context, and justification nodes:
- **ISO 21448's four-quadrant model** justifies the strategy of decomposing the hazard argument into known and unknown scenario sub-goals.
- **ISO 34502's risk factor categories** justify the strategy of decomposing scenario identification along perception, judgement, and control axes.

---

## ISO 21448 Four-Quadrant Model and PDDL

The ISO 21448 four-quadrant scenario model maps directly onto PDDL state-space exploration:

| ISO 21448 Quadrant | PDDL Correspondence | SACE Activity |
|-------------------|---------------------|---------------|
| **Known-safe** | PDDL states where all constraints are satisfied and no hazard predicate is true | Confirmed by plan validity certificates |
| **Known-unsafe** | States where counterexample traces have been found; constraints added | Stage 2 reachability analysis; Stage 3 constraint satisfaction |
| **Unknown-safe** | Explored PDDL states without counterexamples | State-space coverage report |
| **Unknown-unsafe** | Unexplored PDDL state space | Coverage metrics bound this residual |

The SACE process, expressed as GSN argument patterns and informed by ISO 21448, systematically shrinks the unknown-unsafe region through PDDL state-space exploration.

---

## ISO 34502 Risk Factor Categories and PDDL Domain Structure

ISO 34502 decomposes the operational environment into three risk factor categories. These map directly onto PDDL modelling:

| ISO 34502 Category | PDDL Mapping | Relevant Annexes |
|-------------------|--------------|-----------------|
| **Perception disturbances** | Predicate reliability qualifiers; fault-injected perception failure variants | Annex C |
| **Traffic/judgement disturbances** | Action precondition gaps; multi-agent interaction predicates | Annex B |
| **Vehicle/control disturbances** | Action effect uncertainty; degraded-mode action variants | Annex D |
| **Constrained random testing** | Randomised PDDL problem generation for unknown-unsafe exploration | Annex K |

---

## References

1. **SACE Guidance** — University of York, AAIP (2023). *Safety Assurance of Autonomous Systems in Complex Environments.*
2. **ISO 21448:2022** — *Road vehicles — Safety of the intended functionality (SOTIF).*
3. **ISO 34502:2022** — *Road vehicles — Test scenarios for automated driving systems — Scenario-based safety evaluation framework.*
4. **GSN Standard** — The Assurance Case Working Group. *Goal Structuring Notation Community Standard, Version 3.*
5. **AMLAS Guidance** — University of York, AAIP (2021). *Assurance of Machine Learning for use in Autonomous Systems.*
