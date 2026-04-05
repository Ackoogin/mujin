# Multi-Agent Planning Extension

**Status: Phase 1 IMPLEMENTED** ✓

This document reviews approaches for extending the AME PDDL planning system to support multi-agent problems, and outlines a phased implementation plan.

> **Implementation Note:** The recommended "PDDL + Agent Parameter" approach with leader-delegation has been implemented. See `tests/test_multi_agent.cpp` for test coverage.

---

## 1. Current System Overview

The AME pipeline currently uses:
- **LAPKT** with BRFS for single-agent STRIPS planning
- **WorldModel** as the shared state store (predicates, types, objects, grounded actions)
- **PlanCompiler** converting linear plans → sequential BehaviorTrees
- **ActionRegistry** mapping PDDL actions → BT node implementations

The current domain (`uav-search`) models a single robot with `move`, `search`, and `classify` actions. The planner produces a single linear plan executed by one agent.

---

## 2. Multi-Agent Planning Paradigms

### 2.1 Centralised vs. Distributed

| Approach | Description | Pros | Cons |
|----------|-------------|------|------|
| **Centralised** | Single planner solves for all agents | Optimal coordination, simpler WorldModel | Scalability, single point of failure |
| **Distributed** | Each agent plans locally, coordination protocol | Scalable, resilient | Coordination overhead, suboptimal |
| **Hybrid** | Centralised high-level + distributed low-level | Balance of both | Complexity |

**Recommendation:** Start with **centralised MA-STRIPS** (joint planning) given the existing LAPKT architecture. This is the simplest extension and maintains plan optimality for small teams (2–6 agents).

### 2.2 Multi-Agent PDDL Formalisms

| Formalism | Description | Tooling |
|-----------|-------------|---------|
| **MA-STRIPS** | Joint actions across agents; single centralised problem | FMAP, CMAP, ADP |
| **MA-PDDL** | Extended PDDL with `:multi-agent` requirement, agent types | Limited planners |
| **PDDL + Agent Parameter** | Standard PDDL with `?agent` parameter on all actions | Any STRIPS planner |
| **Task Allocation + Single-Agent** | Decompose goals, allocate, plan per-agent | CBBA, auction-based |

**Recommendation:** Use **PDDL + Agent Parameter** for near-term (works with LAPKT), then evaluate **MA-STRIPS planners** (FMAP) for concurrent action support.

---

## 3. Architecture Impact Analysis

### 3.1 WorldModel Changes

| Component | Current | Multi-Agent Extension |
|-----------|---------|----------------------|
| Agent tracking | Single implicit agent | Explicit `agent` type with instances |
| Agent state | Single `(at ?r ?l)` | Per-agent fluents: `(at uav1 sector_a)`, `(at uav2 sector_b)` |
| Goal distribution | Single goal set | Goal assignment per agent or joint goals |
| Action ownership | Implicit | Explicit agent parameter on all actions |

**Required changes:**
- Add `agent` as a first-class type in the type system
- Support agent-scoped state queries: `wm.getAgentFacts(agent_id)`
- Track which agent is executing which action during BT execution

### 3.2 Planner Changes

| Approach | Implementation | Effort |
|----------|----------------|--------|
| **A. Agent-parameterised STRIPS** | Extend domain with `?agent` param; LAPKT solves joint problem | Low |
| **B. External MA-STRIPS planner** | Integrate FMAP/CMAP via subprocess (like OPTIC pattern) | Medium |
| **C. Task allocation + per-agent planning** | Goal decomposition → N separate LAPKT calls | Medium |

**Approach A (recommended for Phase 1):**
```pddl
(:action move
  :parameters (?agent - robot ?from - location ?to - location)
  :precondition (at ?agent ?from)
  :effect (and (at ?agent ?to) (not (at ?agent ?from))))
```
LAPKT interleaves actions across agents. Output: `[move(uav1, A, B), move(uav2, C, D), search(uav1, B), ...]`

### 3.3 PlanCompiler Changes

The compiler must handle **concurrent execution** across agents:

| Plan Structure | Current BT | Multi-Agent BT |
|----------------|------------|----------------|
| Sequential (single agent) | `Sequence` | `Sequence` (unchanged) |
| Interleaved (multi-agent) | N/A | **Agent-parallel execution** |

**Options for multi-agent BT compilation:**

1. **Per-Agent Sub-Trees:** Compile separate BT per agent, run in parallel
   ```
   ParallelNode
   ├── Agent1_Sequence: [move(uav1,...), search(uav1,...)]
   └── Agent2_Sequence: [move(uav2,...), classify(uav2,...)]
   ```

2. **Synchronisation Points:** Detect causal dependencies, insert sync barriers
   ```
   ParallelNode
   ├── Agent1: move(uav1, A, B)
   └── Agent2: move(uav2, C, D)
   SyncBarrier
   ParallelNode
   ├── Agent1: search(uav1, B)
   └── Agent2: search(uav2, D)
   ```

3. **Partial-Order Compilation:** Use STN-like structure (from temporal extension) for action ordering

**Recommendation:** Option 1 (per-agent sub-trees) for Phase 1; Option 2 for Phase 2 when coordination constraints exist.

### 3.4 ActionRegistry Changes

- Actions must carry agent context: `ActionImpl::agent_id`
- BT nodes receive agent ID via blackboard: `getInput<std::string>("agent_id")`
- ROS2: per-agent namespaces or action servers: `/uav1/move`, `/uav2/move`

### 3.5 Execution Changes

| Component | Change |
|-----------|--------|
| **MissionExecutor** | Tick multiple per-agent BTs or single parallel BT |
| **Replanning** | Per-agent replan or global replan on failure |
| **Observability** | Agent-tagged events in audit logs |

---

## 4. Multi-Agent Domain Modelling

### 4.1 Example Multi-Agent Domain

```pddl
(define (domain multi-uav-search)
  (:requirements :strips :typing)

  (:types
    location - object
    sector - location
    agent - object
    uav - agent
  )

  (:predicates
    (at ?a - agent ?l - location)
    (searched ?s - sector)
    (classified ?s - sector)
    (assigned ?a - agent ?s - sector)  ; task allocation
    (free ?a - agent)                   ; agent availability
  )

  (:action move
    :parameters (?a - agent ?from - location ?to - location)
    :precondition (and (at ?a ?from) (free ?a))
    :effect (and (at ?a ?to) (not (at ?a ?from))))

  (:action search
    :parameters (?a - agent ?s - sector)
    :precondition (and (at ?a ?s) (assigned ?a ?s))
    :effect (searched ?s))

  (:action classify
    :parameters (?a - agent ?s - sector)
    :precondition (and (at ?a ?s) (searched ?s))
    :effect (classified ?s))

  (:action assign-task
    :parameters (?a - agent ?s - sector)
    :precondition (free ?a)
    :effect (and (assigned ?a ?s) (not (free ?a))))

  (:action complete-task
    :parameters (?a - agent ?s - sector)
    :precondition (and (assigned ?a ?s) (classified ?s))
    :effect (and (free ?a) (not (assigned ?a ?s))))
)
```

### 4.2 Concurrency Constraints

For true concurrent execution, need **mutex predicates** or **resource constraints**:

```pddl
(:predicates
  (comm-channel-free)  ; shared resource
)

(:action transmit
  :parameters (?a - agent ?data - object)
  :precondition (and (has ?a ?data) (comm-channel-free))
  :effect (and (transmitted ?data) (not (comm-channel-free))))
```

---

## 5. Planner Options Evaluation

### 5.1 Centralised Joint Planning (LAPKT)

**Approach:** Single STRIPS problem with all agents; planner interleaves actions.

| Pros | Cons |
|------|------|
| No new planner needed | Exponential blowup in action space |
| Consistent with existing architecture | No native concurrency semantics |
| Optimal for small teams | Plan extraction requires post-processing |

**Scalability:** O(|A|^n) where |A| = actions per agent, n = agents. Practical for ≤6 agents.

### 5.2 FMAP (Forward Multi-Agent Planner)

**Approach:** Distributed forward search with privacy-preserving coordination.

| Pros | Cons |
|------|------|
| Native MA-STRIPS support | Additional dependency |
| Concurrent action semantics | Java-based (subprocess integration) |
| Handles larger teams | Learning curve |

**Integration:** Subprocess pattern (like OPTIC); parse plan output.

### 5.3 Task Allocation + Per-Agent Planning

**Approach:** Decompose joint goal into per-agent goals, solve independently.

| Pros | Cons |
|------|------|
| Scalable | May miss coordination opportunities |
| Fault-tolerant | Goal decomposition heuristic needed |
| Per-agent replanning | Suboptimal |

**Algorithms:** CBBA (Consensus-Based Bundle Algorithm), market-based auction.

---

## 6. Implementation Plan

### Phase 1: Agent-Parameterised STRIPS (Low Effort)

**Goal:** Multi-agent planning using existing LAPKT with agent parameters.

| Step | Work Item | Effort | Dependencies |
|------|-----------|--------|--------------|
| 1.1 | Add `agent` type to domain; update `uav-search` domain | Low | None |
| 1.2 | Update `PddlParser` to handle agent-typed parameters | Low | 1.1 |
| 1.3 | Extend `WorldModel` with agent registry | Low | 1.2 |
| 1.4 | Plan post-processor: extract per-agent action sequences | Medium | 1.3 |
| 1.5 | `PlanCompiler`: generate per-agent sub-trees under `ParallelNode` | Medium | 1.4 |
| 1.6 | Update `ActionRegistry` with agent context propagation | Low | 1.5 |
| 1.7 | BT nodes: add `agent_id` input port | Low | 1.6 |
| 1.8 | Agent-scoped observability (audit logs, Foxglove) | Low | 1.7 |
| 1.9 | Multi-agent domain tests + integration tests | Medium | 1.8 |

**Deliverable:** Multi-UAV search mission with 2–4 agents executing in parallel.

### Phase 2: Coordination Constraints (Medium Effort)

**Goal:** Handle resource conflicts and synchronisation.

| Step | Work Item | Effort |
|------|-----------|--------|
| 2.1 | Mutex predicate detection in plan compiler | Medium |
| 2.2 | Sync barrier insertion for causal dependencies | Medium |
| 2.3 | `SyncBarrier` BT node (blocks until all agents reach barrier) | Low |
| 2.4 | Resource-aware WorldModel (mutex fluents, capacity constraints) | Medium |
| 2.5 | Deadlock detection in compiled BT | Low |

### Phase 3: External MA-STRIPS Planner (Medium Effort)

**Goal:** Integrate dedicated MA planner for better scalability.

| Step | Work Item | Effort |
|------|-----------|--------|
| 3.1 | `IMultiAgentPlanner` abstraction (similar to `IPlannerBackend`) | Low |
| 3.2 | FMAP subprocess integration | Medium |
| 3.3 | MA-plan parser (concurrent action sets) | Medium |
| 3.4 | `PlanCompiler` concurrent action layer compilation | High |
| 3.5 | Fallback: LAPKT for single-agent, FMAP for multi-agent | Low |

### Phase 4: Distributed Execution (High Effort)

**Goal:** Per-agent BT execution with coordination protocol.

| Step | Work Item | Effort |
|------|-----------|--------|
| 4.1 | Per-agent `ExecutorNode` with local WorldModel view | High |
| 4.2 | WorldModel synchronisation protocol (ROS2 services) | High |
| 4.3 | Distributed replanning (local replan vs. global replan) | High |
| 4.4 | Agent failure handling (reassign tasks) | Medium |
| 4.5 | Communication-aware planning (comms constraints) | Medium |

---

## 7. Testing Strategy

| Test Type | Scope |
|-----------|-------|
| **Unit** | Agent registry, per-agent plan extraction, agent-scoped facts |
| **Integration** | Multi-agent plan → parallel BT compilation → execution |
| **E2E** | 2-UAV, 4-UAV search missions; timing, concurrency |
| **Stress** | Scaling: 2, 4, 8, 16 agents; plan time vs. agent count |
| **Failure** | Single agent failure → replan; communication loss |

---

## 8. Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Exponential plan time | High | Limit to ≤6 agents in Phase 1; external planner in Phase 3 |
| Concurrent BT complexity | Medium | Per-agent sub-trees simplify execution model |
| WorldModel race conditions | High | Existing thread-safety (RCU snapshots) applies |
| Goal decomposition errors | Medium | Validate per-agent goals are achievable |
| Distributed state divergence | High | Authoritative WorldModel with sync protocol |

---

## 9. Dependencies

| Dependency | Status | Notes |
|------------|--------|-------|
| Temporal planning (Extension 7) | Future | Concurrent actions benefit from STN infrastructure |
| Thread safety (Extension 5) | ✓ Complete | RCU snapshots support concurrent agent execution |
| Hierarchical planning (Extension 6) | ✓ Complete | Per-agent phases can use `ExecutePhaseAction` |

---

## 10. Success Criteria

### Phase 1 (MVP)
- [ ] 2+ UAVs execute parallel search missions
- [ ] Plans generated in <1s for 4 agents, 10 sectors
- [ ] Per-agent audit trails in observability stack
- [ ] No regression in single-agent performance

### Phase 2
- [ ] Resource mutex correctly enforced (no conflicts)
- [ ] Sync barriers work for dependent actions

### Phase 3
- [ ] External planner integration functional
- [ ] 8+ agents supported with reasonable plan times

---

## 11. References

- Brafman, R. I., & Domshlak, C. (2008). *From one to many: Planning for loosely coupled multi-agent systems.* ICAPS.
- Nissim, R., & Brafman, R. I. (2014). *Distributed heuristic forward search for multi-agent planning.* JAIR.
- FMAP: https://github.com/rpglab/FMAP
- MA-PDDL specification: Kovacs, D. L. (2012). *A multi-agent extension of PDDL3.1.*
