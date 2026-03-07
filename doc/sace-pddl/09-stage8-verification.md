# Stage 8: Verification Assurance

[Back to Index](00-index.md) | [Previous: Stage 7](08-stage7-out-of-context.md) | [Next: Residual Risk Management](10-residual-risk.md)

---

## SACE Purpose

Determines the verification strategy, executes verification, and builds the argument that safety requirements are met.

## PDDL Contribution

PDDL plan traces are formal verification evidence. Each valid plan is a constructive proof. Counterexample plans from fault-injected domain variants are negative evidence. Exhaustive state-space exploration provides coverage metrics.

## Inputs

| Input | PDDL Mapping |
|-------|-------------|
| **All PDDL artefacts from Stages 1–7** | Complete model set for verification |
| **Verification strategy** | Which properties require formal, simulation, or test evidence |
| **Coverage criteria** | State-space coverage targets |
| **Simulation and test correlation data** | Links PDDL plans to execution traces |

## Outputs

| Output | Description |
|--------|-------------|
| **Plan validity certificates** | Plan trace plus constraint satisfaction proof per scenario |
| **State-space coverage report** | Reachable states explored and uncovered regions |
| **Property verification results** | Per-constraint pass/fail with plan evidence |
| **Verification argument contribution** | PDDL evidence mapped to GSN verification argument pattern |
| **Model-to-implementation traceability** | PDDL abstractions linked to code and test artefacts |

## GSN Mapping — Pattern [UU]

| GSN Element | Content |
|-------------|---------|
| **Goal** | Verification evidence demonstrates that the AS satisfies its safety requirements |
| **Strategy** | Argue that formal verification (PDDL), simulation, and testing collectively provide sufficient evidence. Decompose into sub-goals per evidence type and per safety requirement |
| **Context** | The verification strategy defining evidence types. The coverage criteria. ISO 21448 validation targets |
| **Solution** | Plan validity certificates are solution nodes for formal verification sub-goals. State-space coverage reports support the completeness sub-goal. Property verification results are solution nodes for per-requirement sub-goals. Model-to-implementation traceability supports the argument that abstract PDDL evidence is relevant to the real system |
| **Justification** | ISO 21448's four-quadrant model justifies combining known-scenario verification with unknown-scenario exploration (coverage). ISO 34502 Annex K (constrained random testing) justifies randomised PDDL problem generation as a strategy for unknown-unsafe discovery |
| **Assumption** | PDDL state-space coverage is a meaningful proxy for real-world scenario coverage. The model-to-implementation traceability demonstrates that abstract plans correspond to executable behaviour. These assumptions require confidence arguments |

## ISO 21448 (SOTIF) Impact

Constrained plan traces prove mitigation of known-unsafe scenarios. State-space coverage metrics provide evidence about the unknown-unsafe residual.

## ISO 34502 Impact

Each ISO 34502 annex scenario maps to a PDDL verification task. Annex K corresponds to randomised PDDL problem generation.

## UAV Example

For each operating scenario, the planner produces a valid constrained plan. State-space analysis confirms a defined proportion of reachable states explored. Each plan validity certificate is a GSN **solution** node. The state-space coverage report is a GSN **solution** node supporting the completeness goal. The ISO 34502 annex coverage summary is a GSN **justification** node explaining why the verification scope is systematic.

---

[Next: Residual Risk Management](10-residual-risk.md)
