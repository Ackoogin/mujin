# SACE PDDL Contributions — Stage 8: Verification Assurance

*Part of the [SACE PDDL Contributions](00_standards_context.md) document set.*

**GSN Pattern: [UU] — Verification Assurance**

---

## SACE Purpose

Determines the verification strategy, executes verification, and builds the argument that safety requirements are met. This stage integrates evidence from all previous stages into a coherent, auditable verification argument.

---

## PDDL Contribution

PDDL plan traces are formal verification evidence:

- Each **valid constrained plan** is a constructive proof that the system can achieve its mission goal while satisfying all safety requirements.
- **Counterexample plans** from fault-injected domain variants are negative evidence demonstrating that unsafe scenarios are reachable (and therefore must be mitigated).
- **Exhaustive state-space exploration** provides coverage metrics that bound the unknown-unsafe residual.

---

## Inputs

| Input | How it informs PDDL |
|-------|---------------------|
| All PDDL artefacts from Stages 1 to 7 | Complete model set for verification |
| Verification strategy | Which properties require formal, simulation, or test evidence |
| Coverage criteria | State-space coverage targets |
| Simulation and test correlation data | Links PDDL plans to execution traces |

---

## Outputs

| Output | Description |
|--------|-------------|
| **Plan validity certificates** | Plan trace plus constraint satisfaction proof per scenario |
| **State-space coverage report** | Reachable states explored and uncovered regions |
| **Property verification results** | Per-constraint pass/fail with plan evidence |
| **Verification argument contribution** | PDDL evidence mapped to GSN verification argument pattern |
| **Model-to-implementation traceability** | PDDL abstractions linked to code and test artefacts |

---

## GSN Mapping — Pattern [UU]

Pattern [UU] argues that verification evidence demonstrates that the AS satisfies its safety requirements.

| GSN Element | Content | PDDL Role |
|-------------|---------|-----------|
| **Goal** | Verification evidence demonstrates that the AS satisfies its safety requirements | Target |
| **Strategy** | Argue that formal verification (PDDL), simulation, and testing collectively provide sufficient evidence; decompose into sub-goals per evidence type and per safety requirement | Multi-evidence strategy |
| **Context** | The verification strategy defining evidence types; the coverage criteria; ISO 21448 validation targets | Strategy as context |
| **Solution** | Plan validity certificates (formal verification sub-goals); state-space coverage reports (completeness sub-goal); property verification results (per-requirement sub-goals); model-to-implementation traceability (relevance of abstract evidence) | PDDL artefacts as solution nodes |
| **Justification** | ISO 21448's four-quadrant model justifies combining known-scenario verification with unknown-scenario exploration; ISO 34502 Annex K justifies randomised PDDL problem generation for unknown-unsafe discovery | Standards as justification nodes |
| **Assumption** | PDDL state-space coverage is a meaningful proxy for real-world scenario coverage; model-to-implementation traceability demonstrates abstract plans correspond to executable behaviour — both require confidence arguments | Model fidelity and coverage assumptions |

---

## ISO 21448 (SOTIF) Impact

PDDL verification addresses both dimensions of ISO 21448's validation requirement:

| ISO 21448 Target | PDDL Verification Method | Evidence Produced |
|-----------------|--------------------------|------------------|
| Known-unsafe scenarios mitigated | Constrained plan traces proving safe behaviour | Plan validity certificates |
| Unknown-unsafe residual bounded | State-space coverage metrics | Coverage report |
| ODD excursions managed | Fallback plan reachability (from Stage 7) | Fallback plan library |
| Functional insufficiencies managed | Failure-mode variant results (from Stage 6) | Failure register + mitigation evidence |

---

## ISO 34502 Impact

ISO 34502 provides two direct contributions to the verification strategy:

| ISO 34502 Element | PDDL Verification Application |
|-------------------|------------------------------|
| **Annexes B, C, D scenarios** | Each annex scenario maps to a PDDL verification task; the annex coverage summary is a GSN justification node |
| **Annex K (constrained random testing)** | Randomised PDDL problem generation for unknown-unsafe discovery — problems generated with randomly sampled initial states from the defined scenario space |

---

## Verification Evidence Types and GSN Mapping

```
Verification Assurance [UU]
│
├── Formal verification sub-goal
│   └── Solution: Plan validity certificates (one per scenario)
│
├── Completeness sub-goal
│   └── Solution: State-space coverage report
│
├── Per-requirement sub-goals (one per SR-xx)
│   └── Solution: Property verification results
│
├── Implementation relevance sub-goal
│   └── Solution: Model-to-implementation traceability
│
└── Unknown-unsafe exploration sub-goal
    └── Solution: Randomised problem generation results (Annex K)
```

---

## Plan Validity Certificate Structure

Each plan validity certificate records:

1. **Scenario identity** — problem file reference, scenario ID from scenario catalogue
2. **Initial state** — predicate set at planning start
3. **Goal** — target predicate set
4. **Plan** — ordered action sequence
5. **Constraint check** — per-constraint pass/fail for each trajectory constraint from Stage 3
6. **Planner metadata** — solver used, iterations, planning time
7. **Domain version** — hash of domain file used

---

## Model-to-Implementation Traceability

PDDL abstractions must be linked to concrete implementation artefacts:

| PDDL Element | Implementation Artefact | Test Evidence |
|-------------|------------------------|---------------|
| `navigate` action | `SetWorldPredicate` BT node + `navigate_bt_node.cpp` | `test_integration.cpp::NavigateAction` |
| `obstacle-at ?wp` predicate | `WorldModel::setFact("obstacle_at_wp")` | `test_world_model.cpp::SetAndGetFact` |
| `(always obstacle-clear)` constraint | `CheckWorldPredicate` BT node precondition | `test_integration.cpp::PreconditionBlocking` |
| Planner termination bound | `max_iterations` parameter in LAPKT config | `test_planner.cpp::TerminationBound` |

This table is the model-to-implementation traceability record, forming a GSN solution node supporting the implementation relevance sub-goal.

---

## UAV Example

**Verification scenario:** UAV search mission in nominal conditions.

**Plan validity certificate:**
- Scenario: `uav_search/problem_nominal.pddl`
- Plan: `[fly-to wp-1, scan wp-1, fly-to wp-2, scan wp-2, return-to-base]`
- Constraint checks: all 5 trajectory constraints PASS
- Planning time: 12ms (within SR-02 bound)

**State-space coverage report:** 94% of reachable states explored across all problem variants; remaining 6% are extreme multi-failure states entering immediate abort.

**GSN integration:**
- Each plan validity certificate is a **GSN solution node** for the formal verification sub-goal.
- The state-space coverage report is a **GSN solution node** supporting the completeness goal.
- The ISO 34502 annex coverage summary is a **GSN justification node** explaining why the verification scope is systematic.

---

## Evidence Checklist

- [ ] Plan validity certificates produced for each scenario in the scenario catalogue
- [ ] State-space coverage report generated with target coverage achieved
- [ ] Property verification results produced for each safety requirement (SR-xx)
- [ ] Model-to-implementation traceability table completed and reviewed
- [ ] ISO 34502 Annex B, C, D scenarios all have corresponding PDDL verification tasks
- [ ] Randomised problem generation implemented for Annex K exploration
- [ ] Coverage assumption documented with confidence argument
- [ ] Implementation relevance assumption documented with confidence argument

---

*Previous: [Stage 7 — Out of Context Operation Assurance](07_stage7_out_of_context.md)*
*Next: [Residual Risk Management](09_residual_risk.md)*
