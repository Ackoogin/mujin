# Remaining Work

All vertical slice steps (1–8) and extensions 1–6 are complete. This document consolidates the roadmap for remaining work.

---

## Extensions 3 & 5: Perception & Thread Safety

These extensions are intertwined — a single authoritative `WorldModel` accessed by concurrent perception callbacks and a BT tick thread requires synchronisation.

### Versioned Snapshots
- Update `WorldModel` to maintain double-buffering or RCU (Read-Copy-Update) semantics
- BT executor reads from a consistent snapshot (captured at tick start)
- Apply queued `setFact()` mutations from perception between BT ticks

### Perception Integration
- Add dedicated interface in `WorldModelNode` to subscribe to `/detections` or `/telemetry` topics
- Map ROS messages into `setFact()` calls
- Enforce **State Authority Semantics**: mark perception-sourced facts as `CONFIRMED` vs plan-applied `BELIEVED` facts

---

## Extension 4: PYRAMID Service Nodes

- Implement `InvokeService` BT node inheriting from `BT::StatefulActionNode`
- Translate PDDL action bindings (from `ActionRegistry`) into async service requests
- Handle async wait returning `BT::NodeStatus::RUNNING` until service completes
- Implement timeout handling and graceful `FAILURE` if SDK service unreachable

---

## Extension 6: Hierarchical Planning

- Finalise `ExecutePhaseAction` BT node concept
- Integrate `PlannerNode` so `ExecutePhaseAction` can invoke ROS2 action call to planner
- Retrieve dynamically compiled child tree and tick using BT.CPP dynamic sub-tree features
- Update `PlanAuditLog` to capture causal links between parent phases and sub-trees

---

## Extension 7: Temporal Planning (not started)

PDDL 2.1 durative actions with STN conversion. See [`temporal_extension_research.md`](temporal_extension_research.md) for planner evaluation.

### Recommended Approach

- **Primary:** OPTIC (C++, PDDL 2.1 + 3.0, subprocess invocation)
- **Medium-term:** Evaluate Aries (Rust, built-in STN solver, hierarchical + temporal)
- **Fallback:** TFD (GPL, PlanSys2 ROS2 integration exists)
- **LAPKT retained** for STRIPS domains; temporal planner only for `:durative-actions`

### Implementation Phases

| Phase | Work Item | Effort |
|-------|-----------|--------|
| 7a | PDDL 2.1 parser (`:durative-action`, `:functions`) | Medium |
| 7b | WorldModel numeric fluent store + audit | Medium |
| 7c | `IPlannerBackend` abstraction + OPTIC subprocess | Medium |
| 7d | STN data structure + consistency check | Low |
| 7e | PlanCompiler temporal mode (STN → BT `Parallel`/`Timeout`) | High |
| 7f | VAL integration for temporal plan validation | Low |
| 7g | Temporal invariant monitoring (`ReactiveSequence`) | Low |
| 7h | End-to-end temporal tests | Medium |
| 7i | Aries evaluation + migration | Medium |

---

## Extension 8: Neuro-Symbolic Integration

Neural components assist, but the symbolic system remains authoritative. See [`neuro_symbolic_reasoning.md`](neuro_symbolic_reasoning.md) and [`neuro_symbolic_reasoning_review.md`](neuro_symbolic_reasoning_review.md).

### Pre-requisites
- **State-Authority Semantics:** Clarify `BELIEVED` (plan effects) vs `CONFIRMED` (perception) facts in WorldModel
- **Neural Acceptance Criteria:** Latency budget (500ms), fallback to LAPKT on timeout/error, audit logging

### Phase 1 (Low-Risk)
- **LLM Goal Interpreter:** Natural language → grounded PDDL goals; symbolic validation rejects invalid fluents
- **LLM Mission Analyst:** Offline analysis of audit logs; evidence-cited explanations

### Phase 2+ (Deferred)
- Planner-adjacent: LLM Heuristic Guide, Plan Repair
- Data-driven: Learned Heuristic, Anomaly Detector
- Authoring: Neural PDDL Domain Authoring

---

## Autonomy Assurance Gaps

From SACE Stage 8 analysis. See [`autonomy_assurance_plan.md`](autonomy_assurance_plan.md).

### High-Priority Verification

| Gap | Objective | Actions |
|-----|-----------|---------|
| Property-Based Planner Testing | Prove solver soundness over random valid domains | PDDL fuzzer + test harness asserting plan simulation |
| Adversarial Perception Testing | Reject malicious/stale/inconsistent data | Fault-injection middleware for `setFact()` ROS2 service |
| Safe-State Integration | Verify degradation to safe state | E2E tests: planning timeout, comms loss, unmapped actions → fallback BT |

### Medium-Priority

| Gap | Objective |
|-----|-----------|
| OOC Detection | Bounds-checking on WorldModel + `CheckContext` BT node for geofencing |
| Threat Modelling | `threat_model.md` for spoofing, PDDL injection, denial-of-planning |
| Performance Benchmarks | CI thresholds on LAPKT time; BT tick stress tests at 50Hz |
| Compiler Correctness | BT → linear action sequence round-trip verification |

---

## Hardening (production-readiness)

### Planner
- Solver portfolio: fast heuristic first, fall back to complete search
- Configurable time/node budget with timeout handling

### ActionRegistry
- Dynamic registration from config file (YAML/JSON)
- Type-checked parameter binding against PDDL schema
- Startup validation: all PDDL actions have registered implementations

### BT Nodes
- Failure taxonomy: TRANSIENT (retry), PERMANENT (replan + blacklist), FATAL (abort)
- `WaitForFact`, `Timeout` decorator, `RetryWithBackoff` nodes

### PlanCompiler
- Serialise compiled BT to file for inspection
- Emit DOT graph of causal structure
- Complex DAG join-point synchronisation

### MissionExecutor
- Progressive replan: retry → local replan → full replan → relax goal → abort
- Replan budget (max N replans)
- Pre-replan world model consistency checks

### PDDL Parser
- Structured error messages with line/column
- Schema validation: predicate arities, type consistency, unreachable goals

### Configuration & Packaging
- YAML config for domain paths, solver selection, action mappings, logging
- ROS2 lifecycle graceful shutdown with state persistence
- Debian/colcon package, Docker image, CI/CD pipeline

### Testing
- Property-based / fuzz tests for registration ordering
- Systematic fault injection
- Planning time vs. domain size benchmarks
- GitHub Actions CI: build matrix, ctest, coverage

---

## Post-Extension (future)

- Plan quality optimisation (anytime search, LPG-style local improvement)
- Advanced heuristics (landmark-based for larger domains)
- Multi-agent planning (MA-STRIPS or task allocation)
- Dynamic object lifecycle (creation/destruction mid-mission)
- Resource-aware flow scheduling (mutex on shared resources)
- Gazebo/Isaac Sim integration
