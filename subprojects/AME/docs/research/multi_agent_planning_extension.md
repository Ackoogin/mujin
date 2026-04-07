# Multi-Agent Planning Extension

**Status: Phase 1 IMPLEMENTED** ✓

This document reviews approaches for extending the AME PDDL planning system to support multi-agent problems, and outlines remaining phases.

> **Implementation Note:** The recommended "PDDL + Agent Parameter" approach with leader-delegation has been implemented. See `tests/test_multi_agent.cpp` for test coverage.

### Phase 1 Summary (Complete)

Phase 1 delivered agent-parameterised STRIPS planning using existing LAPKT with leader-delegation:
- Agent registry in WorldModel (`registerAgent`, `availableAgentIds`)
- `GoalAllocator` with sector-based grouping and round-robin assignment
- `DelegateToAgent` BT node for per-agent planning and execution
- `PlanCompiler` agent context injection via blackboard
- Multi-agent PDDL domain (`subprojects/AME/domains/multi_uav_search/`)
- `AgentDispatcher` + `AgentDispatcherNode` for distributed ROS2 execution
- 22 tests passing, <500ms for 2 agents / 6 sectors

---

## Remaining Phases

### Phase 2: Coordination Constraints (Medium Effort)

Handle resource conflicts and synchronisation between agents.

| Step | Work Item | Effort |
|------|-----------|--------|
| 2.1 | Mutex predicate detection in plan compiler | Medium |
| 2.2 | Sync barrier insertion for causal dependencies | Medium |
| 2.3 | `SyncBarrier` BT node (blocks until all agents reach barrier) | Low |
| 2.4 | Resource-aware WorldModel (mutex fluents, capacity constraints) | Medium |
| 2.5 | Deadlock detection in compiled BT | Low |

### Phase 3: External MA-STRIPS Planner (Medium Effort)

Integrate dedicated MA planner for better scalability (>6 agents).

| Step | Work Item | Effort |
|------|-----------|--------|
| 3.1 | `IMultiAgentPlanner` abstraction (similar to `IPlannerBackend`) | Low |
| 3.2 | FMAP subprocess integration | Medium |
| 3.3 | MA-plan parser (concurrent action sets) | Medium |
| 3.4 | `PlanCompiler` concurrent action layer compilation | High |
| 3.5 | Fallback: LAPKT for single-agent, FMAP for multi-agent | Low |

### Phase 4: Distributed Execution (High Effort)

Per-agent BT execution with coordination protocol.

| Step | Work Item | Effort |
|------|-----------|--------|
| 4.1 | Per-agent `ExecutorNode` with local WorldModel view | High |
| 4.2 | WorldModel synchronisation protocol (ROS2 services) | High |
| 4.3 | Distributed replanning (local replan vs. global replan) | High |
| 4.4 | Agent failure handling (reassign tasks) | Medium |
| 4.5 | Communication-aware planning (comms constraints) | Medium |

---

## Dependencies

| Dependency | Status | Notes |
|------------|--------|-------|
| Temporal planning (Extension 7) | Future | Concurrent actions benefit from STN infrastructure |
| Thread safety (Extension 5) | ✓ Complete | RCU snapshots support concurrent agent execution |
| Hierarchical planning (Extension 6) | ✓ Complete | Per-agent phases can use `ExecutePhaseAction` |

---

## References

- Brafman, R. I., & Domshlak, C. (2008). *From one to many: Planning for loosely coupled multi-agent systems.* ICAPS.
- Nissim, R., & Brafman, R. I. (2014). *Distributed heuristic forward search for multi-agent planning.* JAIR.
- FMAP: https://github.com/rpglab/FMAP
- MA-PDDL specification: Kovacs, D. L. (2012). *A multi-agent extension of PDDL3.1.*

