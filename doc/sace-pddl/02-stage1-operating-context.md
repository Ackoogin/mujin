# Stage 1: Operating Context Assurance

[Back to Index](00-index.md) | [Next: Stage 2](03-stage2-hazardous-scenarios.md)

---

## SACE Purpose

Defines what the AS can do autonomously, models the environment (Operational Domain Model), and specifies the operating scenarios. The goal is an assured operating context that bounds the autonomous operation.

## PDDL Contribution

PDDL provides the formal, executable definition of the operating context. The domain file encodes the system's autonomous capabilities as typed actions with preconditions and effects. PDDL types, predicates, and objects define the ODM. Problem files instantiate specific operating scenarios as initial states and goals.

## Inputs

| Input | PDDL Mapping |
|-------|-------------|
| **Concept of operations (ConOps)** | Informs the PDDL domain action set and type hierarchy |
| **Stakeholder requirements** | Constrain PDDL action preconditions (e.g. airspace rules, separation minima) |
| **Environmental characterisation data** | Mapped to PDDL types and objects (terrain classifications, weather envelopes, agent types) |
| **ODD description** | Becomes PDDL predicates that bound valid states, gating which actions are applicable |

## Outputs

| Output | Description |
|--------|-------------|
| **PDDL domain file** | Formal specification of autonomous capabilities — actions, types, and predicates |
| **PDDL problem file(s)** | Instantiated operating scenarios with initial states and goals representing specific missions |
| **ODM-to-PDDL traceability matrix** | Links each ODM element to its PDDL representation, ensuring completeness |
| **Scenario coverage report** | Demonstrates that the set of problem files covers the defined operating scenarios |

## GSN Mapping — Pattern [G]

Pattern [G] argues that the operating context has been sufficiently defined and is valid.

| GSN Element | Content |
|-------------|---------|
| **Goal** | The autonomous operating context for the AS is sufficiently defined |
| **Strategy** | Argue over completeness and validity of the ODM and operating scenarios, justified by systematic derivation from ConOps and ODD |
| **Context** | The PDDL domain file provides the formal definition of the operating context. The PDDL type hierarchy and predicate set define the ODM scope |
| **Solution** | The ODM-to-PDDL traceability matrix demonstrates that all ODM elements have PDDL representations. The scenario coverage report demonstrates that problem files cover the operating scenario space |
| **Assumption** | The PDDL abstraction is a faithful representation of the real operating context. This assumption must be justified (e.g. by review or by demonstrating correspondence between PDDL predicates and real sensor observables) |
| **Justification** | ISO 34502 risk factor categories (perception, judgement, control) justify structuring the ODM along these axes. ISO 21448 ODD definition requirements justify the completeness criteria applied |

## ISO 21448 (SOTIF) Impact

ISO 21448 requires explicit definition of the ODD and identification of functional insufficiencies within it. PDDL predicates can encode the ODD boundary conditions and action preconditions can capture known performance limitations. The PDDL domain becomes a formal specification of the intended functionality, making functional insufficiencies visible where preconditions cannot be fully assured by the real system.

## ISO 34502 Impact

ISO 34502 decomposes the operational environment into risk factors across perception, planning/judgement, and control. These three categories map directly onto PDDL modelling:

- **Perception** risk factors become predicate reliability qualifiers
- **Planning** risk factors map to action precondition gaps
- **Control** risk factors map to action effect uncertainty

Problem files should be structured to cover the ISO 34502 scenario space systematically.

## UAV Example

PDDL types include `uav`, `waypoint`, `airspace-zone`, and `threat-level`. Actions include `navigate`, `loiter`, and `return-to-base` with preconditions encoding airspace constraints. ISO 21448 ODD limits such as maximum wind speed and minimum visibility are encoded as predicates that gate action applicability.

- The GSN **context** node references the PDDL domain file as the formal operating context definition.
- The GSN **solution** node references the scenario coverage report showing that all identified operating scenarios have corresponding problem files.

---

[Next: Stage 2 — Hazardous Scenarios Identification](03-stage2-hazardous-scenarios.md)
