# Stage 7: Out of Context Operation Assurance

[Back to Index](00-index.md) | [Previous: Stage 6](07-stage6-hazardous-failures.md) | [Next: Stage 8](09-stage8-verification.md)

---

## SACE Purpose

Addresses safety when the AS operates outside its defined operating context, whether through planned handover or unplanned excursion.

## PDDL Contribution

PDDL models the in-context/out-of-context boundary as state predicates. Planning from out-of-context states demonstrates whether safe fallback actions are always reachable.

## Inputs

| Input | PDDL Mapping |
|-------|-------------|
| **Operating context boundary predicates** | Define when the AS has left its assured context |
| **Safe state definitions** | PDDL goal states for fallback planning |
| **Fallback action set** | Actions available during out-of-context operation |
| **Transition trigger conditions** | Predicates activating context-exit detection |

## Outputs

| Output | Description |
|--------|-------------|
| **Out-of-context reachability analysis** | Safe states reachable from all out-of-context states |
| **Fallback plan library** | Pre-computed plans for each out-of-context situation |
| **Unreachable safe-state report** | States from which recovery is impossible |
| **Context boundary validation** | Boundary predicates correctly partition in/out-of-context |

## GSN Mapping — Pattern [PP]

| GSN Element | Content |
|-------------|---------|
| **Goal** | The AS remains sufficiently safe when operating outside its defined operating context |
| **Strategy** | Argue over each identified out-of-context condition that a safe state is reachable. Decompose into sub-goals per out-of-context trigger combination |
| **Context** | The defined operating context boundary. The set of safe states |
| **Solution** | The out-of-context reachability analysis provides formal proof of safe-state reachability. The fallback plan library provides concrete plans. The unreachable safe-state report identifies residual gaps requiring additional mitigations |
| **Justification** | ISO 21448 ODD excursion requirements justify the need for this argument. ISO 34502 combined risk factor analysis justifies modelling multi-factor out-of-context states |
| **Assumption** | The set of out-of-context trigger conditions is complete. The fallback action set is available under the degraded conditions modelled |

## ISO 21448 (SOTIF) Impact

Where PDDL proves no safe fallback exists, this identifies ODD gaps that ISO 21448 requires to be addressed.

## ISO 34502 Impact

Combined risk factor states are modelled by setting multiple degradation predicates simultaneously.

## UAV Example

`(comms-lost)` and `(gps-denied)` simultaneously true. The planner finds a return-to-base plan via inertial navigation. The GSN **solution** is the fallback plan. If no plan exists for a particular combination, the unreachable safe-state report becomes a GSN **context** node triggering a requirement for additional mitigation.

---

[Next: Stage 8 — Verification Assurance](09-stage8-verification.md)
