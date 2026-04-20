# Autonomy Assurance Plan

Modular assurance plan for the PDDL-planning + Behaviour-Tree-execution autonomy module.
Structured around:

| Framework | Source | Scope |
|-----------|--------|-------|
| **SACE** (Safety Assurance of Autonomous Systems in Complex Environments) | University of York / AAIP | System-level safety case for AS in complex environments |
| **AMLAS** (Assurance of Machine Learning for Autonomous Systems) | University of York / AAIP | ML-component-specific assurance (if/when ML is introduced) |
| **DSTL Biscuit Book** (Assurance of AI & Autonomous Systems, 2021) | UK MOD / DSTL | Cross-cutting assurance dimensions — requirements, data, algorithms, integration, adversarial |
| **Defence Standard 00-56** | UK MOD | Safety Management Requirements for Defence Systems |
| **ISO 21448 (SOTIF)** | ISO | Safety of the intended functionality — functional insufficiency analysis, four-quadrant scenario classification |
| **ISO 34502** | ISO | Scenario-based safety evaluation — systematic risk factor derivation across perception, judgement, and control |

> [!NOTE]
> This plan is modular by design. Each numbered section maps to a SACE stage and can be instantiated, reviewed, and evidenced independently. DSTL "biscuit book" cross-cutting concerns are woven through the relevant stages. AMLAS stages are called out where ML components may be introduced.

### PDDL Formal Modelling & GSN Argument Mapping

Detailed documentation of how PDDL formal models contribute to each SACE stage, with GSN argument pattern mappings, ISO 21448/34502 impacts, and UAV worked examples, is provided in the **[SACE–PDDL Integration Guide](../../assurance/AME/sace-pddl/00-index.md)** (12 indexed files covering all 8 stages, residual risk management, and cross-stage summary).

---

## 0. System Overview (Non-SACE — reference context)

### 0.1 Architecture Summary

```
WorldModel --▶ LAPKT Planner --▶ Plan-to-BT Compiler --▶ BT.CPP Executor
    ▲                                                          │
    └------------ effects + perception ◀-----------------------┘
```

| Component | Role | Key Assurance Concern |
|-----------|------|----------------------|
| **WorldModel** | Single authoritative state store (grounded bitset) | State integrity, audit trail |
| **LAPKT Planner** | Stateless STRIPS solver (BFS/SIW) | Soundness, completeness, termination |
| **Plan-to-BT Compiler** | Causal-graph → parallel/sequence BT | Correctness of causal ordering |
| **BT.CPP Executor** | Ticks compiled tree; reactive or non-reactive | Timing, failure propagation, replan triggers |
| **ActionRegistry** | Maps PDDL actions → BT nodes / sub-trees | Mapping fidelity, coverage |
| **MissionExecutor** | Top-level tick loop with replan-on-failure | Convergence, livelock prevention |

### 0.2 Assurance-Relevant Properties

The core assurance question for this module is:

> **Does the autonomous planning and execution system achieve the mission goal safely, or fail gracefully, in all credible operating scenarios?**

This decomposes into:

1. **Correctness** — the planner produces valid plans; the BT executes them faithfully.
2. **Completeness** — if a valid plan exists, the planner finds one.
3. **Robustness** — the system detects and recovers from action failures.
4. **Transparency** — every decision is auditable post-hoc.
5. **Boundedness** — planning and execution are time-bounded.
6. **Graceful degradation** — the system enters a safe state when it cannot plan or execute.

---

## 1. Operating Context Assurance (SACE Stage 1)

> *Define and validate the operational boundaries within which the AS is deemed safe to operate.*

### 1.1 Operational Domain Model (ODM)

Document the environment the AS operates within:

| Dimension | What to specify | Source of truth |
|-----------|----------------|-----------------|
| **Physical environment** | Terrain, weather, electromagnetic, GPS availability | Domain PDDL + external config |
| **Other agents** | Friendly / adversarial / neutral entities, communication links | Problem PDDL + perception inputs |
| **Infrastructure** | Comms latency, C2 link reliability, compute constraints | System integration spec |
| **Temporal** | Mission duration bounds, replanning time budgets | `MissionExecutor` settings |
| **Regulatory** | Airspace restrictions, ROE, no-fly zones | Zone objects in WorldModel |

### 1.2 Autonomous Capabilities Specification

Enumerate the autonomous capabilities the planning module provides:

| Capability | Description | PDDL scope |
|------------|-------------|-------------|
| Autonomous goal decomposition | Converts high-level objectives into STRIPS goals | Goal fluent selection |
| Automated plan generation | Produces totally-ordered action sequences | LAPKT BFS(f)/SIW solver |
| Concurrent execution | Identifies and exploits plan parallelism | Causal-graph flow extraction |
| Reactive monitoring | Continuous precondition checking during execution | `ReactiveSequence` BT nodes |
| Autonomous replanning | Detects action failure → re-plans from current state | `MissionExecutor.replan()` |

### 1.3 Operating Scenarios

Define representative scenarios for assurance, e.g.:

- **Nominal** — plan executes to completion, all preconditions hold.
- **Partial failure** — single action fails, replanning succeeds.
- **Perception divergence** — external perception updates invalidate plan assumptions.
- **Planning failure** — no valid plan exists for the current world state.
- **Comms-degraded** — WorldModel cannot receive perception updates.
- **Resource-exhausted** — planning time budget exceeded.

### 1.4 Evidence Required

| Evidence artefact | Purpose |
|-------------------|---------|
| ODM document | Validates operating boundaries are fully specified |
| Capability–Domain matrix | Demonstrates every capability covered within ODM bounds |
| Scenario catalogue | Provides basis for SACE Stage 2 hazard identification |
| CONOPS diagram | Shows human-machine roles and authority levels |

### 1.5 DSTL Cross-cut: Assurance of Requirements

Per the DSTL biscuit book *"Assurance of Requirements"* dimension:

- [ ] Safety requirements traceable to hazard analysis (Stage 2)
- [ ] Functional requirements traceable to PDDL domain specification
- [ ] Non-functional requirements (timing, availability) captured and testable
- [ ] Requirements reviewed by independent party

---

## 2. AS Hazardous Scenarios Identification (SACE Stage 2)

> *Identify potential hazards arising from autonomy interactions.*

### 2.1 Hazard Analysis Techniques

| Technique | Application | Output |
|-----------|-------------|--------|
| **STPA** (System-Theoretic Process Analysis) | Control-loop analysis of Planner → Executor → WorldModel | Unsafe Control Actions (UCAs) |
| **HAZOP on PDDL domain** | Systematic guideword analysis of each PDDL action's preconditions/effects | Deviation scenarios |
| **FTA / ETA** | Fault and event trees for high-consequence failures | Probability estimation |
| **FMEA on BT nodes** | Each BT node type analysed for failure modes | Node-level mitigations |

### 2.2 Key Hazard Categories

| ID | Hazard Category | Example | Affected Component |
|----|----------------|---------|-------------------|
| H1 | **Incorrect plan** | Planner produces a plan that violates domain constraints | LAPKT Planner |
| H2 | **Incomplete plan** | Plan missing critical actions (solver terminated early) | LAPKT Planner |
| H3 | **Stale world model** | Plan based on perception data that no longer reflects reality | WorldModel |
| H4 | **Causal misordering** | Compiler produces BT with incorrect dependency structure | Plan-to-BT Compiler |
| H5 | **Effect misattribution** | BT node applies wrong PDDL effects to WorldModel | SetWorldPredicate nodes |
| H6 | **Replan livelock** | System cycles between failing plans without converging | MissionExecutor |
| H7 | **Timing violation** | Planning or BT tick exceeds real-time budget | Planner + Executor |
| H8 | **Action–BT mismatch** | ActionRegistry maps PDDL action to wrong BT implementation | ActionRegistry |
| H9 | **Unreachable safe state** | No safe-state behaviour defined for planning failure | MissionExecutor |

### 2.3 Evidence Required

| Evidence artefact | Purpose |
|-------------------|---------|
| STPA analysis tables (UCAs + loss scenarios) | Systematic identification of control-loop hazards |
| PDDL HAZOP worksheets | Action-level deviation analysis |
| FMEA on BT node types | Node-level failure mode coverage |
| Hazard log (live document) | Tracks hazards through to mitigation and closure |

---

## 3. Safe Operating Concept Assurance (SACE Stage 3)

> *Define the requirements and constraints for safe operation.*

### 3.1 Safe Operating Concept (SOC)

The SOC specifies the conditions under which the autonomy module is permitted to operate:

| Rule | Rationale | Implementation |
|------|-----------|----------------|
| **Human-on-the-loop (HOTL)** | Operator can observe and override mission execution | Observability stack (Layers 1–5) + override interface |
| **Bounded planning time** | Prevents unbounded compute; ensures timely replanning | `max_iterations` parameter in LAPKT solver |
| **Replan limit** | Prevents livelock | `MissionExecutor` replan counter + safe-state fallback |
| **Precondition gating** | Actions only execute when preconditions verified | `CheckWorldPredicate` BT nodes |
| **Perception freshness** | WorldModel state must be recent enough for planning | Perception timestamp + staleness threshold |
| **Safe-state fallback** | If no valid plan exists, system enters defined safe state | `ReplanOnFailure` → safe-state BT sub-tree |
| **Audit completeness** | All state changes and decisions are logged | WM audit log, BT event stream, Plan audit trail |

### 3.2 Authority and Control Framework

Mapped to UK MOD Joint Doctrine Note (JDN) autonomy levels:

| Autonomy Level | Description | When applicable |
|----------------|-------------|-----------------|
| **Level 2 — Human delegated** | System plans and executes; human can intervene at any point | Default operating mode |
| **Level 3 — Human supervisory** | System plans, executes, and self-corrects; human notified of key decisions | With replan-on-failure active |

### 3.3 Evidence Required

| Evidence artefact | Purpose |
|-------------------|---------|
| SOC document | Formal specification of operating constraints |
| Authority framework mapping | Demonstrates compliance with MOD autonomy governance |
| Safe-state specification | Defines what "safe" means for each operating scenario |

---

## 4. AS Safety Requirements Assurance (SACE Stage 4)

> *Derive traceable safety requirements that address identified hazards.*

### 4.1 Safety Requirements (derived from Stage 2 Hazard IL)

| Req ID | Requirement | Traces to Hazard | Verification Method |
|--------|-------------|------------------|---------------------|
| SR-01 | Planner SHALL only produce plans where every action's preconditions are satisfiable from the plan's causal chain | H1, H4 | Formal verification of Plan-to-BT compiler + unit tests |
| SR-02 | Planner SHALL terminate within N ms or return NO_PLAN | H2, H7 | Unit test with timeout; metric logged via PlanAuditLog |
| SR-03 | WorldModel SHALL reject `setFact()` calls with timestamps older than T seconds when freshness-checking is enabled | H3 | Unit test + integration test with stale perception |
| SR-04 | Causal graph extraction SHALL preserve all add-effect → precondition dependencies | H4 | Property-based test: compiled BT re-checked against original plan |
| SR-05 | `SetWorldPredicate` BT nodes SHALL only modify the facts specified in the PDDL action's effect list | H5 | Static analysis of action–node binding; integration test |
| SR-06 | `MissionExecutor` SHALL cease replanning after K consecutive failures and enter safe state | H6 | Unit test |
| SR-07 | BT tick period SHALL not exceed T ms (e.g. 20 ms for 50 Hz) | H7 | Performance test + runtime monitoring |
| SR-08 | `ActionRegistry.resolve()` SHALL fail loudly if no mapping exists for a PDDL action name | H8 | Unit test; compile-time assertion where possible |
| SR-09 | A safe-state BT sub-tree SHALL be loaded and tickable at all times | H9 | Integration test; boot-time sanity check |

### 4.2 DSTL Cross-cut: Assurance of Algorithms

Per the DSTL biscuit book *"Assurance of Algorithms"* dimension:

- [ ] LAPKT solver algorithm formally described (BFS(f), SIW) with known properties
- [ ] Soundness property: all produced plans are valid (precondition-satisfied sequences)
- [ ] Completeness bounds documented (BFS(f) is complete; SIW is not — document trade-off)
- [ ] Termination guaranteed by max-iteration bound
- [ ] Algorithm behaviour under degraded input documented (incomplete/contradictory WorldModel assertions)

### 4.3 Evidence Required

| Evidence artefact | Purpose |
|-------------------|---------|
| Safety Requirements Specification (SRS) | Formal list of all safety requirements |
| Traceability matrix (Hazard → Requirement → Test) | End-to-end traceability |
| Algorithm property documentation | Formal or semi-formal description of solver guarantees |

---

## 5. AS Design Assurance (SACE Stage 5)

> *Assure that the design adequately addresses safety requirements.*

### 5.1 Design Assurance Arguments

| Design decision | Assurance argument | Supporting evidence |
|-----------------|-------------------|---------------------|
| **Single authoritative WorldModel** | All components read from one truth source; eliminates state divergence | Architecture doc (`concept.md`) + unit tests showing single-instance |
| **Stateless planner** | Planner carries no hidden state between invocations; same input → same output | Pure-function design; deterministic test suite |
| **Compiler-generated BT** | No hand-authored trees in the execution path; systematic construction from plan | Compiler correctness tests; no manual BT XML in deployment |
| **Reactive vs. non-reactive per action** | Explicit per-action configuration prevents inappropriate continuous checking | ActionRegistry config; documented rationale per action |
| **Observability layers 1–5** | Every state change, every node transition, every planning episode is logged | Test coverage of audit log completeness |

### 5.2 DSTL Cross-cut: Assurance of Integration

Per the DSTL biscuit book *"Assurance of Integration"* dimension:

- [ ] Interface contracts between WorldModel ↔ Planner ↔ Compiler ↔ Executor defined and tested
- [ ] Data format consistency (fluent IDs, predicate strings) verified at integration boundaries
- [ ] Failure propagation across component boundaries tested (e.g. planner failure → executor response)
- [ ] ROS2 multi-node deployment tested for race conditions and message ordering

### 5.3 DSTL Cross-cut: Assurance of Data

Per the DSTL biscuit book *"Assurance of Data"* dimension:

- [ ] WorldModel data integrity: bitset consistency verified with version counter
- [ ] Perception input validation: type checking on incoming `setFact()` calls
- [ ] PDDL domain/problem files validated at load time (syntax + semantic checks)
- [ ] Audit log data completeness verified (no silent drops)

### 5.4 Evidence Required

| Evidence artefact | Purpose |
|-------------------|---------|
| Design Assurance Case (GSN or similar) | Structured argument linking design decisions to safety requirements |
| Interface control documents (ICDs) | Formal interface definitions between components |
| Integration test results | Demonstrate correct cross-component behaviour |

---

## 6. Hazardous Failures Management (SACE Stage 6)

> *Manage the consequences of failures within the AS.*

### 6.1 Failure Detection Mechanisms

| Failure type | Detection mechanism | Response |
|-------------|---------------------|----------|
| **Action execution failure** | BT node returns FAILURE status | `ReplanOnFailure` decorator triggers replan |
| **Planning failure** (no plan found) | Planner returns empty result | MissionExecutor enters safe-state |
| **Planning timeout** | Wall-clock timer on solver | Solver returns best-effort or NO_PLAN |
| **WorldModel corruption** | Version counter inconsistency / assertion failure | System halt + log dump |
| **Perception staleness** | Timestamp + freshness threshold | Planning blocked until fresh data received |
| **Communication loss** | Heartbeat timeout on ROS2 lifecycle | Node transitions to INACTIVE |
| **BT tick overrun** | Watchdog timer on tick duration | Log warning; escalate if sustained |

### 6.2 Safe States

| Safe state | Trigger | Behaviour |
|-----------|---------|-----------|
| **Loiter** | Planning failure, comms loss | Entity maintains current position/orbit |
| **Return to base** | Sustained failure, resource depletion | Pre-loaded BT sub-tree executed |
| **Hold** | Operator intervention | All execution paused; WorldModel frozen |

### 6.3 Common Cause Failure Analysis

Per SACE guidance, identify potential common-cause failures:

- [ ] Shared dependency on WorldModel — failure affects all downstream components
- [ ] Single-threaded execution — CPU starvation affects both BT ticking and perception processing
- [ ] PDDL domain errors — affect both planner soundness and BT correctness
- [ ] Version mismatch between BT.CPP and system — runtime behaviour assumptions change

### 6.4 Evidence Required

| Evidence artefact | Purpose |
|-------------------|---------|
| FMEA tables for each component | Systematic failure mode analysis |
| Safe-state test results | Demonstrate each safe state is reachable and correct |
| Common cause failure analysis | Identifies shared failure modes across components |

---

## 7. Out-of-Context Operation Assurance (SACE Stage 7)

> *Assure safety when the AS operates outside its defined operating context.*

### 7.1 Out-of-Context Detection

The system must detect when it is operating outside the bounds specified in Stage 1:

| Out-of-context condition | Detection method | Response |
|-------------------------|------------------|----------|
| Unknown object types in perception | WorldModel type system rejects unregistered types | Log + operator alert |
| Goals referencing undefined predicates | Planner pre-check fails | Reject goal + alert |
| WorldModel state exceeds tested scale | Fluent count exceeds validated threshold | Warning + graceful degradation |
| Operating in environment not in ODM | Operator-configured geofence / context check | Safe-state transition |

### 7.2 Handover Mechanisms

| Mechanism | Description |
|-----------|-------------|
| **Operator takeover** | Execution paused; operator assumes direct control |
| **Autonomous safe-state** | System transitions to pre-defined safe behaviour |
| **Graduated autonomy reduction** | System reduces autonomy level (Level 3 → Level 2 → Level 1) as confidence decreases |

### 7.3 Evidence Required

| Evidence artefact | Purpose |
|-------------------|---------|
| OOC detection test results | Demonstrate detection of out-of-context conditions |
| Handover procedure verification | Test that handover mechanisms work correctly |
| Boundary analysis | Document the limits of the operating context |

---

## 8. AS Verification Assurance (SACE Stage 8)

> *Verify through testing, analysis, and inspection that safety requirements are met.*

### 8.1 TEVV Strategy (DSTL Framework)

Per the DSTL biscuit book, assurance is underpinned by **Test, Evaluation, Verification, and Validation (TEVV)**:

| TEVV Activity | Scope | Tooling |
|---------------|-------|---------|
| **Unit Test** | Individual component correctness | Google Test (`tests/test_*.cpp`) |
| **Integration Test** | Cross-component interaction | Google Test (`tests/test_integration.cpp`) |
| **Property-Based Test** | Planner soundness, compiler correctness over random domains | Custom harness generating random PDDL |
| **Performance Test** | Planning time, BT tick rate, memory usage | Benchmark suite with thresholds |
| **Formal Verification** | Compiler causal-graph correctness, WorldModel invariants | Lightweight model checking / assertion proofs |
| **Simulation-Based Test** | End-to-end mission execution in representative scenarios | Sim harness with scenario injection |
| **Adversarial Test** | System behaviour under hostile perception inputs | Fault injection framework |
| **Runtime Monitoring** | Continuous online assertion checking in deployment | AmeBTLogger + WmAuditLog + watchdog |

### 8.2 Existing Test Coverage

| Test file | Covers | Status |
|-----------|--------|--------|
| `test_world_model.cpp` | WorldModel state management, eager grounding | ✅ Passing |
| `test_action_registry.cpp` | Action mapping, resolution | ✅ Passing |
| `test_plan_compiler.cpp` | Plan-to-BT compilation, causal graph | ✅ Passing |
| `test_pddl_parser.cpp` | PDDL file parsing | ✅ Passing |
| `test_integration.cpp` | End-to-end planning → execution | ✅ Passing |
| `test_observability.cpp` | Audit logging, BT event streaming | ✅ Passing |

### 8.3 Verification Gaps to Close

| Gap | Required evidence | Priority |
|-----|-------------------|----------|
| No property-based testing of planner | Random PDDL generation + plan validation suite | **High** |
| No adversarial testing of perception inputs | Fault injection into `setFact()` path | **High** |
| No explicit timing/performance test suite | Benchmark harness with CI thresholds | **Medium** |
| No safe-state integration test | Scenario test triggering each safe state | **High** |
| No formal argument for compiler correctness | Proof or exhaustive test of causal graph algorithm | **Medium** |
| No replan-livelock boundary test | Stress test with adversarial WorldModel changes | **Medium** |

### 8.4 Evidence Required

| Evidence artefact | Purpose |
|-------------------|---------|
| Test results (all suites) | Demonstrate requirement coverage |
| Coverage report | Show code/requirement coverage metrics |
| Performance benchmark results | Demonstrate timing requirements met |
| Verification summary report | Cross-reference test results to safety requirements |

---

## 9. DSTL Cross-cut: Assurance in the Presence of Adversaries

This dimension has no direct SACE stage but is critical for defence systems.

### 9.1 Threat Model

| Threat | Target | Mitigation |
|--------|--------|------------|
| **Perception spoofing** | WorldModel state via `setFact()` | Input validation, multi-source consistency checking, freshness gating |
| **PDDL domain injection** | Domain file loading | File integrity checking, signed domain files |
| **BT XML injection** | Compiler output | No external BT XML in execution path (compiler-generated only) |
| **Denial of planning** | LAPKT solver | Timeout bounds, fallback safe-state |
| **Adversarial goal manipulation** | Goal fluent specification | Goal validation against permitted set |

### 9.2 Evidence Required

| Evidence artefact | Purpose |
|-------------------|---------|
| Threat model document | Systematic adversarial analysis |
| Penetration test / red-team results | Demonstrate resilience under attack |
| Input validation test results | Demonstrate rejection of invalid inputs |

---

## 10. AMLAS Considerations (if/when ML components are introduced)

If ML components are introduced (e.g. learned heuristics for the planner, ML-based perception classifiers), the AMLAS 6-stage process applies:

| AMLAS Stage | Application to this system |
|-------------|---------------------------|
| 1. ML safety assurance scoping | Define which ML components exist and their safety role |
| 2. Safety requirements elicitation | Derive ML-specific safety requirements (e.g. classifier accuracy bounds) |
| 3. Data management | Training/validation data provenance, quality, bias analysis |
| 4. Model learning | Training process assurance, hyperparameter documentation |
| 5. Model verification | Out-of-distribution detection, robustness testing, formal guarantees |
| 6. Model deployment | Runtime monitoring, model versioning, graceful fallback to non-ML path |

> [!IMPORTANT]
> The current architecture is **ML-free** — the LAPKT planner uses classical deterministic search. AMLAS applies only if ML components are added in future (e.g. learned planning heuristics, perception classifiers). This section should be instantiated at that point.

---

## 11. Assurance Lifecycle & Governance

### 11.1 Living Assurance Documents

| Document | Update trigger | Owner |
|----------|---------------|-------|
| Hazard Log | New hazard identified, mitigation status change | Safety Engineer |
| Safety Requirements Specification | Hazard log update, design change | Systems Engineer |
| Traceability Matrix | Any SRS or test change | V&V Lead |
| Test Results | Code change, CI pipeline | Development Team |
| Audit Trail | Automatically updated at runtime | System (automated) |

### 11.2 Review Gates

| Gate | Condition | Reviewers |
|------|-----------|-----------|
| **Design Review** | Before implementation of safety-critical component | Independent safety assessor |
| **Code Review** | All changes to safety-critical components (`WorldModel`, `MissionExecutor`, `PlanCompiler`) | Peer + safety engineer |
| **Integration Review** | Before multi-component deployment | Systems integrator + safety |
| **Deployment Review** | Before operational use | Programme authority + safety board |

### 11.3 Runtime Assurance

| Activity | Mechanism | Already exists? |
|----------|-----------|-----------------|
| State change auditing | WM audit log (Layer 3) | ✅ Yes |
| Decision auditing | Plan audit trail (Layer 5) | ✅ Yes |
| Execution monitoring | BT event stream (Layer 2) | ✅ Yes |
| Performance monitoring | TreeObserver statistics (Layer 1) | ✅ Yes |
| Live visualization | Foxglove bridge (Layer 4) | ✅ Yes |
| Anomaly detection | **Gap** — not yet implemented | ❌ No |
| Runtime assertion checking | **Gap** — watchdog for invariants | ❌ No |

---

## 12. Summary: Assurance Modules & Maturity

| Stage | SACE Stage | Status | Key Gap |
|-------|-----------|--------|---------|
| 1 | Operating Context | 🟡 Partial (architecture documented, ODM not formalised) | Formal ODM + CONOPS document |
| 2 | Hazardous Scenarios | 🔴 Not started | STPA / HAZOP analysis needed |
| 3 | Safe Operating Concept | 🟡 Partial (safe-state design exists) | Formal SOC document |
| 4 | Safety Requirements | 🟡 Draft (this document) | Formal SRS + traceability matrix |
| 5 | Design Assurance | 🟢 Strong (modular, auditable architecture) | Formal safety argument (GSN) |
| 6 | Hazardous Failures | 🟡 Partial (replan-on-failure implemented) | FMEA + CCA completion |
| 7 | Out-of-Context | 🔴 Not started | OOC detection + handover design |
| 8 | Verification | 🟢 Strong (73+ tests, 5-layer observability) | Property-based + adversarial + perf tests |
| — | Adversarial (DSTL) | 🔴 Not started | Threat model + red-team plan |
| — | AMLAS | ⚪ N/A (no ML components) | Instantiate when ML introduced |

---

## References

1. **SACE Guidance** — University of York, Assuring Autonomy International Programme (2023). *Safety Assurance of Autonomous Systems in Complex Environments.* [york.ac.uk/assuring-autonomy](https://www.york.ac.uk/assuring-autonomy/)
2. **AMLAS Guidance** — University of York, AAIP (2021). *Assurance of Machine Learning for use in Autonomous Systems.*
3. **DSTL Biscuit Book** — UK MOD / DSTL (2021). *Assurance of Artificial Intelligence and Autonomous Systems.* [gov.uk](https://www.gov.uk/government/publications/assurance-of-artificial-intelligence-and-autonomous-systems)
4. **Defence Standard 00-56** — UK MOD. *Safety Management Requirements for Defence Systems.*
5. **STPA Handbook** — MIT. *System-Theoretic Process Analysis.*
6. **UK MOD JDN 3/22** — Joint Doctrine Note: Autonomy in Defence.
7. **ISO 21448:2022** — Road vehicles — Safety of the intended functionality (SOTIF).
8. **ISO 34502:2022** — Road vehicles — Test scenarios for automated driving systems — Scenario based safety evaluation framework.

