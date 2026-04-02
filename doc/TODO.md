# Remaining Work

All vertical slice steps (1–8) and extensions 1–6 are complete. This document consolidates the roadmap for remaining work.

---

## Extensions 3 & 5: Perception & Thread Safety ✓

**Status: COMPLETE**

These extensions are intertwined — a single authoritative `WorldModel` accessed by concurrent perception callbacks and a BT tick thread requires synchronisation.

### Versioned Snapshots ✓
- [x] `WorldModel` maintains RCU semantics via `captureSnapshot()` returning immutable `WorldStateSnapshot`
- [x] BT executor can capture snapshot at tick start for consistent reads
- [x] `enqueueMutation()` / `applyQueuedMutations()` for batching perception updates between ticks
- [x] Thread-safe access with `std::shared_mutex` (readers) and `std::mutex` (mutation queue)

### Perception Integration ✓
- [x] `WorldModelNode` subscribes to `/detections` topic (`ame_ros2::msg::Detection`)
- [x] Maps detection properties to `setFact()` calls with `FactAuthority::CONFIRMED`
- [x] Configurable confidence threshold (`perception.confidence_threshold` parameter)
- [x] Authority conflict detection: `hasAuthorityConflict()` warns when perception disagrees with plan

### State Authority Semantics ✓
- [x] `FactAuthority` enum: `BELIEVED` (plan effects) vs `CONFIRMED` (perception)
- [x] `FactMetadata` struct: authority, timestamp, source for each fact
- [x] `getFactMetadata()` API for querying fact provenance

### Test Coverage
- [x] Unit tests for authority semantics, snapshots, mutation queue
- [x] Thread safety stress tests (concurrent reads, read/write, mutation queue)

---

## Extension 4: PYRAMID Service Nodes ✓

**Status: COMPLETE**

- [x] `InvokeService` BT node converted from `BT::SyncActionNode` to `BT::StatefulActionNode`
- [x] `IPyramidService` extended with async API: `callAsync()`, `pollResult()`, `cancelCall()`
- [x] `AsyncCallStatus` enum: `PENDING`, `SUCCESS`, `FAILURE`, `CANCELLED`
- [x] PDDL action param auto-mapping via `param_names`/`param_values` ports (strips `?` prefix)
- [x] Async lifecycle: `onStart()` initiates call, `onRunning()` polls, `onHalted()` cancels
- [x] Configurable timeout via `timeout_ms` port (default 5000ms, 0 = no timeout)
- [x] `MockPyramidService` updated for async support (immediate completion on first poll)
- [x] Tests: async RUNNING→SUCCESS, timeout→FAILURE, halt→cancel, param mapping, merge with request_json

---

## Extension 6: Hierarchical Planning ✓

**Status: COMPLETE**

- [x] `ExecutePhaseAction` BT node: plan–compile–execute cycle for sub-goal sets
- [x] `PlannerComponent` integration: `ExecutePhaseAction` detects `PlannerComponent*` on blackboard, enabling distributed planning via ROS2 action server
- [x] Dynamic sub-tree: compiled BT XML loaded via `BT::BehaviorTreeFactory::createTreeFromText()` with shared blackboard
- [x] `PlanAuditLog` causal links: `episode_id`, `parent_episode_id`, `phase_name` fields track parent–child phase relationships
- [x] Sequential phases: `parent_episode_id` propagated via blackboard for causal chaining
- [x] Tests: audit trail recording, causal links for sequential phases, failed planning episodes, PlannerComponent path

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

## Multi-Agent Planning ✓

**Status: COMPLETE (Initial Implementation)**

Leader-delegation pattern for multi-agent coordination. See [`multi_agent_implementation_plan.md`](multi_agent_implementation_plan.md).

### Agent Registry ✓
- [x] `AgentInfo` struct: id, type, availability tracking
- [x] `WorldModel::registerAgent()`, `getAgent()`, `agentIds()`, `availableAgentIds()`
- [x] Agent state copied correctly in WorldModel copy constructor/assignment

### Goal Allocation ✓
- [x] `GoalAllocator` class: sector-based goal grouping, round-robin agent assignment
- [x] `extractSector()` utility for parsing goal fluents
- [x] `AgentGoalAssignment` struct for per-agent goal sets

### Agent Delegation BT Node ✓
- [x] `DelegateToAgent` BT node: plans and executes goals scoped to a specific agent
- [x] Agent availability management: marks agent busy during execution, restores on completion/halt
- [x] Agent context injection via blackboard (`current_agent_id`, `executing_agent`)

### PlanCompiler Agent Context ✓
- [x] `compile(plan, wm, registry, agent_id)` overload injects `SetBlackboard` node
- [x] Agent ID available to all child action nodes via `executing_agent` blackboard key

### Multi-Agent PDDL Domain ✓
- [x] `domains/multi_uav_search/domain.pddl`: typed agents, parameterized actions
- [x] `domains/multi_uav_search/problem.pddl`: 2 UAVs, 4 sectors

### Test Coverage ✓
- [x] Agent registry: registration, lookup, availability, copy semantics
- [x] GoalAllocator: round-robin, sector grouping, edge cases
- [x] Multi-agent planning: joint plans, leader-delegation pattern, plan extraction
- [x] DelegateToAgent: agent busy marking, non-existent agent, unavailable agent
- [x] Complex scenario: 2 agents, 4 sectors, full mission (22 tests passing)

### Distributed Execution ✓
- [x] `ExecutorNode` supports `agent_id` parameter for topic namespacing
- [x] Agent-scoped topics: `/{agent_id}/executor/bt_xml`, `/{agent_id}/executor/status`
- [x] `AgentDispatcher` PCL component for coordinating multi-agent plan dispatch
- [x] `AgentDispatcherNode` ROS2 wrapper with transport callbacks and `~/dispatch_goals` service
- [x] `ame_multi_agent.launch.py` for spawning multiple agent executors + dispatcher

### Future Extensions
- Dynamic re-allocation on agent failure
- Coordination constraints (mutex resources, sync barriers)

---

## ROS2 Extension Wiring ✓

**Status: COMPLETE**

Gap analysis revealed that extensions 3-6 and multi-agent delegation were implemented in `ame_core` but incompletely wired into the ROS2 layer. See `ros2_extensions_gap_fix_plan.md` for the original plan.

- [x] `DispatchGoals.srv` and `AgentDispatcherNode` service wiring
- [x] `InvokeService` registration + `IPyramidService*` injection in `ExecutorNode`
- [x] `ExecutePhaseAction` registration + blackboard wiring (planner/compiler/registry/audit-log)
- [x] `DelegateToAgent` registration (shares blackboard keys with `ExecutePhaseAction`)
- [x] Perception mutation queue fix (`enqueueMutation` + `applyQueuedMutations`)
- [x] `PlannerNode` accessor exposure (`planner()`, `compiler()`, `planAuditLog()`)
- [x] `combined_main.cpp` demo updated with all extension dependencies
- [x] Documentation updates (`06-ros2.md`, `07-extensions.md`, `TODO.md`)
- [x] `AgentDispatcherNode` standalone executable and multi-agent launch update

---

## Post-Extension (future)

- Plan quality optimisation (anytime search, LPG-style local improvement)
- Advanced heuristics (landmark-based for larger domains)
- Dynamic object lifecycle (creation/destruction mid-mission)
- Resource-aware flow scheduling (mutex on shared resources)
- Gazebo/Isaac Sim integration
