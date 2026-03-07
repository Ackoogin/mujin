# Cross-Stage Summary

[Back to Index](00-index.md) | [Previous: Residual Risk Management](10-residual-risk.md)

---

## Per-Stage Summary

| Stage | PDDL Artefact | GSN Role | ISO 21448 Contribution | ISO 34502 Contribution |
|-------|---------------|----------|------------------------|------------------------|
| **1 — Operating Context** | Domain and problem files | Context node (formal operating context definition) + solution nodes (coverage report) | ODD encoding and insufficiency visibility | Risk factor category coverage |
| **2 — Hazardous Scenarios** | Fault-injected domain variants and counterexample traces | Solution nodes (reachability analysis, counterexample traces) supporting hazard completeness goal | Unknown-unsafe scenario discovery | Annex B, C, D scenario generation |
| **3 — Safe Operating Concept** | Trajectory constraints and feasibility analysis | Solution nodes (feasibility evidence within model) supporting mitigation goal | Known-unsafe mitigation evidence | Safety test objective encoding |
| **4 — Safety Requirements** | Decomposed sub-domains and per-tier satisfaction proofs | Solution nodes (traceability evidence, per-tier satisfaction proofs) supporting decomposition goal. Joint implication requires model checking | Insufficiency-to-subsystem tracing | Risk factor decomposition alignment |
| **5 — Design Assurance** | Architecture-mapped domain with action variants | Solution nodes (degraded-mode traces, redundancy evidence) supporting design goal | Design improvement evidence | Control disturbance coverage |
| **6 — Hazardous Failures Mgmt** | Failure-mode PDDL variants, hazardous failure register, and mitigation evidence | Solution nodes (failure-mode analyses, mitigation re-verification) + assumption nodes (failure mode completeness) | Functional insufficiency as hazardous failure; component-level SOTIF evidence | Perception and control disturbance coverage via Annex C and D |
| **7 — Out of Context** | Fallback plan library and per-scenario reachability analysis | Solution nodes (per-scenario reachability evidence, fallback plans) supporting safe-state goal. Universal reachability requires model checking | ODD excursion handling | Combined risk factor analysis |
| **8 — Verification** | Plan validity certificates and coverage reports | Solution nodes (certificates, coverage) + justification nodes (verification strategy rationale) | Residual risk evidence | Scenario-based evaluation evidence |
| **Residual Risk** | Coverage metrics, fault-injected variant results, misuse scenario analyses, response verification traces | Solution nodes (coverage, response verification) + assumption nodes (model fidelity, insufficiency completeness, coverage-to-risk linkage) + context nodes (acceptance criteria) | Core SOTIF: known-unsafe mitigation evidence and model coverage informing unknown-unsafe residual | Risk factor completeness checklist and Annex K randomised exploration methodology |

---

## Key Relationships

### GSN as Structural Backbone

GSN is the structural backbone of the safety case. It defines what claims must be made and what evidence is required. PDDL generates that evidence in a formal, machine-checkable form. The ISO standards inform the argument strategy — they explain *why* the decomposition of claims is sound and *why* the evidence is sufficient.

### ISO 21448 Four-Quadrant Model

ISO 21448's four-quadrant model provides the conceptual framework that PDDL state-space exploration operationalises:

| Quadrant | PDDL Correspondence |
|----------|---------------------|
| **Known-safe** | PDDL states where all constraints are satisfied |
| **Known-unsafe** | States where counterexample traces have been found and constraints added |
| **Unknown-safe** | Unexplored PDDL states that happen to be safe (discovered by exploration) |
| **Unknown-unsafe** | Unexplored PDDL state space (the region to be minimised). Note: this covers only hazards expressible in the model — real-world phenomena outside the model's representational capacity remain unknown-unsafe regardless of coverage |

The SACE process, expressed as GSN argument patterns and informed by ISO 21448, systematically shrinks the unknown-unsafe region through PDDL exploration.

### ISO 34502 Systematic Scenario Derivation

ISO 34502 provides the structured scenario derivation that ensures PDDL exploration is systematic. Its risk factor categories justify the GSN strategy nodes that decompose hazard identification and verification along perception, judgement, and control axes. Its annexes provide the scenario templates that are encoded as PDDL problem file families.

### Integration

Together:

- **SACE** provides the GSN safety case structure
- **ISO 21448 and ISO 34502** populate the strategy, context, and justification nodes
- **PDDL** generates the formal evidence that populates the solution nodes

The result is a safety case that is **structured** (GSN), **conceptually grounded** (ISO 21448/34502), and **formally evidenced** (PDDL).

---

[Back to Index](00-index.md)
