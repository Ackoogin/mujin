# SACE Process — PDDL Contributions per Stage

With GSN Argument Mapping, ISO 21448 (SOTIF) and ISO 34502

**Inputs, Outputs & Formal Modelling Approach**

Draft — March 2026

---

## Overview

This document set maps the contributions of PDDL (Planning Domain Definition Language) formal models to each of the eight stages of the SACE (Safety Assurance of autonomous systems in Complex Environments) methodology. For each stage it identifies the role PDDL plays, the inputs consumed, the outputs produced, how ISO 21448 (SOTIF) and ISO 34502 affect the modelling approach, and how the PDDL artefacts feed into the Goal Structuring Notation (GSN) argument patterns that SACE prescribes. Examples are oriented towards autonomous UAV systems.

---

## Standards Context

| Standard / Notation | Source | Role |
|----------------------|--------|------|
| **SACE** | University of York | Overarching safety case structure for autonomous systems in complex environments, with eight iterative stages producing artefacts and GSN argument patterns |
| **ISO 21448 (SOTIF)** | ISO | Safety of the intended functionality — hazards arising not from system faults but from functional insufficiencies and performance limitations, even when the system operates as designed. Introduces the four-quadrant scenario model (known-safe, known-unsafe, unknown-safe, unknown-unsafe) and requires systematic identification of triggering conditions |
| **ISO 34502** | ISO | Scenario-based safety evaluation framework that derives critical scenarios by systematically combining risk factors across three categories: perception disturbances, traffic/judgement disturbances, and vehicle control disturbances |
| **GSN** | — | Graphical argumentation notation used throughout SACE to document safety cases. Captures goals, strategies, solutions, contexts, assumptions, and justifications in a structured, auditable hierarchy |

### How These Standards and Notations Interact

SACE provides the safety case structure expressed as GSN argument patterns. ISO 21448 provides the conceptual framework for functional insufficiency analysis and the four-quadrant scenario classification. ISO 34502 provides the systematic scenario derivation methodology. PDDL provides the formal, executable modelling language that generates the evidence populating the GSN argument.

The relationship between PDDL and GSN is one of **evidence generation to argument structure**. PDDL artefacts do not replace GSN; rather, they provide the formally verified evidence that instantiates GSN solution nodes, while the ISO standards inform the strategy, context, and justification nodes that explain why the argument decomposition is sound.

---

## Document Index

| File | Contents |
|------|----------|
| [00-index.md](00-index.md) | This file — overview, standards context, document map |
| [01-gsn-role.md](01-gsn-role.md) | GSN elements, SACE argument patterns, how PDDL artefacts map to GSN |
| [02-stage1-operating-context.md](02-stage1-operating-context.md) | Stage 1: Operating Context Assurance |
| [03-stage2-hazardous-scenarios.md](03-stage2-hazardous-scenarios.md) | Stage 2: Hazardous Scenarios Identification |
| [04-stage3-safe-operating-concept.md](04-stage3-safe-operating-concept.md) | Stage 3: Safe Operating Concept Assurance |
| [05-stage4-safety-requirements.md](05-stage4-safety-requirements.md) | Stage 4: Safety Requirements Assurance |
| [06-stage5-design-assurance.md](06-stage5-design-assurance.md) | Stage 5: Design Assurance |
| [07-stage6-hazardous-failures.md](07-stage6-hazardous-failures.md) | Stage 6: Hazardous Failures Management |
| [08-stage7-out-of-context.md](08-stage7-out-of-context.md) | Stage 7: Out of Context Operation Assurance |
| [09-stage8-verification.md](09-stage8-verification.md) | Stage 8: Verification Assurance |
| [10-residual-risk.md](10-residual-risk.md) | Residual Risk Management (cross-cutting) |
| [11-cross-stage-summary.md](11-cross-stage-summary.md) | Cross-Stage Summary and Key Relationships |
| [12-e2e-example.md](12-e2e-example.md) | End-to-End SACE Stage Application Example (with runnable PDDL) |
| [13-tooling-analysis.md](13-tooling-analysis.md) | Tooling Gap Analysis: Planners and Verifiers for SACE |
| [14-review-report.md](14-review-report.md) | Review of the current SACE/PDDL approach in this repository |

---

## Relationship to Existing Assurance Plan

This document set provides detailed PDDL-specific formal modelling guidance that complements the main [Autonomy Assurance Plan](../autonomy_assurance_plan.md). The assurance plan defines the overall SACE stage structure, hazard register, safety requirements, and DSTL cross-cuts. This document set adds:

- Formal PDDL inputs and outputs per stage
- GSN argument pattern mappings showing where PDDL evidence attaches
- ISO 21448 and ISO 34502 modelling impacts
- UAV-oriented worked examples
