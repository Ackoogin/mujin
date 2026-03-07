# Stage 2: Hazardous Scenarios Identification

[Back to Index](00-index.md) | [Previous: Stage 1](02-stage1-operating-context.md) | [Next: Stage 3](04-stage3-safe-operating-concept.md)

---

## SACE Purpose

Identifies interactions between the AS and its environment, analyses every autonomous decision point, and enumerates hazardous scenarios that could lead to harm.

## PDDL Contribution

PDDL's search space is the formal enumeration of all decision points. By exploring the planner's reachable state space, engineers can systematically identify states where hazardous conditions hold. Fault-injected domain variants, where preconditions are deliberately relaxed or degradation predicates introduced, enable automated hazard discovery.

## Inputs

| Input | PDDL Mapping |
|-------|-------------|
| **PDDL domain and problem from Stage 1** | Baseline model for decision-point enumeration |
| **Preliminary hazard list** | Encoded as PDDL predicates (e.g. `collision`, `loss-of-comms`) |
| **Interaction model** | AS-environment interactions mapped to PDDL action effects |
| **Fault and degradation modes** | Modelled as predicate injections or precondition relaxations in fault-injected domain variants |

## Outputs

| Output | Description |
|--------|-------------|
| **Reachability analysis report** | Identifies which states are reachable and whether hazard predicates can become true |
| **Decision-point register** | Each applicable action in each reachable state, extracted from the planner's search tree |
| **Fault-injected domain variants** | Modified PDDL domains modelling specific degradation modes |
| **Counterexample plan traces** | Concrete action sequences leading to hazardous states |

## GSN Mapping — Pattern [I]

Pattern [I] argues that all hazardous scenarios have been identified and the identification is complete and correct.

| GSN Element | Content |
|-------------|---------|
| **Goal** | All hazardous scenarios associated with AS operation have been identified |
| **Strategy** | Argue over systematic enumeration of decision points and reachable hazardous states, using formal state-space exploration. Decompose into sub-goals for each hazard category |
| **Context** | The PDDL domain defines the decision space. The preliminary hazard list defines which predicates constitute hazardous states. ISO 21448's four-quadrant classification scopes the argument |
| **Solution** | The reachability analysis report demonstrates which hazardous predicates are reachable. Counterexample plan traces provide concrete evidence of specific hazardous scenarios. The decision-point register demonstrates completeness of decision analysis |
| **Justification** | ISO 34502 risk factor decomposition (perception, judgement, control) justifies generating fault-injected variants along these three axes, providing systematic coverage. ISO 21448's requirement to address unknown-unsafe scenarios justifies exhaustive state-space exploration beyond manually identified hazards |
| **Assumption** | The hazard predicate set is complete — all relevant hazardous conditions have been encoded. Must be reviewed against the preliminary hazard analysis |

## ISO 21448 (SOTIF) Impact

ISO 21448's four-quadrant model structures the argument. PDDL state-space search addresses the **unknown-unsafe** quadrant *within the modelled state space* by exploring states not anticipated during manual hazard analysis. Each counterexample trace transitions a scenario from unknown-unsafe to known-unsafe. Note that PDDL exploration cannot discover hazards arising from real-world phenomena absent from the model (state space insufficiencies — see [Residual Risk](10-residual-risk.md)). The completeness of the model itself remains an assumption requiring independent review.

## ISO 34502 Impact

ISO 34502 annexes B, C, and D provide systematic templates for fault-injected domain variant generation across traffic, perception, and control disturbances respectively.

## UAV Example

A fault-injected domain variant relaxes the `(obstacle-detected ?wp)` precondition to model sensor failure. The planner discovers a plan reaching `(collision ?uav ?obstacle)`. This trace becomes a GSN **solution** node supporting the goal that hazardous scenario HS-017 (collision due to perception failure) has been identified. The ISO 34502 Annex C perception disturbance category provides a **justification** node explaining why this class of fault injection is systematic.

---

[Next: Stage 3 — Safe Operating Concept Assurance](04-stage3-safe-operating-concept.md)
