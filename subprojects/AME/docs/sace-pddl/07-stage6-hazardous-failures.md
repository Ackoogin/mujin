# Stage 6: Hazardous Failures Management

[Back to Index](00-index.md) | [Previous: Stage 5](06-stage5-design-assurance.md) | [Next: Stage 7](08-stage7-out-of-context.md)

---

## SACE Purpose

Once the design for a tier is known, potential hazardous failures that could result from design decisions can be identified. Stage 6 addresses the identification and acceptable management of these hazardous failures. This includes failures introduced as side-effects of design choices, failures arising from component interactions, and failures specific to the autonomous functions (perception, planning, actuation) that may not be apparent from the requirements or architecture alone.

## PDDL Contribution

PDDL supports hazardous failure management by enabling systematic failure-mode exploration at the planning level. By introducing failure predicates into the domain (representing component failures, data corruption, timing violations, or incorrect predicate grounding), the planner can discover whether such failures lead to unsafe plans or states.

PDDL also serves as the **interface contract** between autonomous functions: perception must produce the predicates the planner consumes, and actuation must faithfully execute the actions the planner emits. Failures in meeting this contract are hazardous failures that Stage 6 must manage.

## Inputs

| Input | PDDL Mapping |
|-------|-------------|
| **PDDL domain from Stage 5** | Architecture-mapped domain defining the design at this tier |
| **Design failure mode analysis** | Hardware and software failure modes mapped to PDDL predicates representing failed states |
| **Component interaction analysis** | Emergent failures from component interactions, modelled as unexpected predicate combinations |
| **ML component safety cases (AMLAS)** | Linked to PDDL predicate reliability assumptions, identifying where ML failures could corrupt planner inputs |

## Outputs

| Output | Description |
|--------|-------------|
| **Failure-mode PDDL variants** | Domain variants with failure predicates active, representing specific component or interaction failures |
| **Hazardous failure register** | Catalogue of identified failures with their PDDL representations and the unsafe states they can reach |
| **Mitigation strategy mapping** | For each hazardous failure: the design change, additional requirement, or operational restriction that manages it, with PDDL evidence that the mitigation is effective |
| **Residual failure analysis** | Identification of failures that cannot be fully mitigated, with associated risk assessment |

## GSN Mapping — Pattern [DD]

Pattern [DD] argues that potential hazardous failures introduced through design decisions have been identified and are acceptably managed.

| GSN Element | Content |
|-------------|---------|
| **Goal** | Hazardous failures that may arise from the design at tier *n* are acceptably managed |
| **Strategy** | Argue over each identified hazardous failure that it is either eliminated by design, mitigated by additional requirements, or that residual risk is acceptable. Decompose into sub-goals per failure mode |
| **Context** | The design at tier *n* from Stage 5. The failure mode analysis identifying potential hazardous failures. The PDDL domain structure mapping design components to actions and predicates |
| **Solution** | Failure-mode PDDL variants demonstrate the consequences of each failure. Mitigation strategy mapping provides evidence that design changes or additional constraints prevent unsafe states being reached. Where AMLAS safety cases exist for ML components, they provide evidence that perception or decision-making failures are managed |
| **Assumption** | The failure mode analysis is sufficiently complete. The PDDL model faithfully represents the design's failure behaviour. These assumptions require confidence arguments |
| **Justification** | ISO 21448 functional insufficiency analysis justifies examining failures arising from the gap between intended and actual component performance. ISO 34502 risk factor categories justify structuring the failure analysis along perception, judgement, and control disturbance axes |

## ISO 21448 (SOTIF) Impact

ISO 21448's functional insufficiency concept extends naturally to hazardous failure management. A functional insufficiency is effectively a failure of the intended functionality to perform as specified under certain conditions. PDDL failure-mode variants make these insufficiencies concrete: by activating failure predicates (such as incorrect predicate grounding from a perception insufficiency), the planner reveals whether the design can still achieve safe outcomes. Where it cannot, the failure is a hazardous failure requiring management. AMLAS provides the component-level evidence for ML-based functions.

## ISO 34502 Impact

ISO 34502 Annex C (perception-related critical scenarios) directly informs the failure modes relevant to perception components. Annex D (control-related critical scenarios) informs actuator and control failure modelling. Each annex provides a structured catalogue of disturbances that, when encoded as PDDL failure predicates, systematically populate the hazardous failure register. The annex structure ensures that failure identification is not ad-hoc but follows a recognised scenario derivation methodology.

## UAV Example

The planner requires predicate `(obstacle-at ?wp)` from the perception pipeline. A hazardous failure is identified: the ML-based obstacle detector produces a false negative under sun glare (an ISO 34502 Annex C perception disturbance).

1. A **failure-mode PDDL variant** sets `(perception-failed obstacle-detector)` as true, which removes the `(obstacle-at ?wp)` precondition from navigate actions.
2. The planner finds a plan reaching `(collision ?uav ?obstacle)`.
3. The **mitigation** is a design change: LIDAR provides independent detection, and the PDDL domain is updated so that `(obstacle-at ?wp)` can be grounded by either sensor.
4. Re-running the failure-mode variant with only camera failed confirms safe plans.
5. The GSN **solution** node references this re-analysis.
6. The AMLAS safety case for the ML detector is an away goal connected via an assurance claim point.

---

[Next: Stage 7 — Out of Context Operation Assurance](08-stage7-out-of-context.md)

