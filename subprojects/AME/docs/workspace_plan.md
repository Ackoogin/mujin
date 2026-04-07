# Implementation Reference

**Status: IMPLEMENTED** — Vertical slice steps 1–8 complete. Extensions 1–6 complete.

This document is a brief reference for the implemented architecture. See `subprojects/AME/docs/architecture/` for the full technical reference; see `TODO.md` for remaining work.

---

## Implemented Steps

| Step | Description |
|------|-------------|
| 1 | Project scaffolding (CMake, GoogleTest, directory structure) |
| 2 | TypeSystem and WorldModel core (eager grounding, bitset storage) |
| 3 | PDDL projection (WorldModel → LAPKT STRIPS_Problem) |
| 4 | PDDL parser integration (FF-parser, UAV example domain) |
| 5 | ActionRegistry (PDDL action → BT implementation mapping) |
| 6 | BT node types (CheckWorldPredicate, SetWorldPredicate) |
| 7 | Plan-to-BT compiler (CausalGraph, flow extraction, Parallel/Sequence) |
| 8 | End-to-end vertical slice (UAV search-and-classify) |

## Implemented Extensions

| Extension | Description |
|-----------|-------------|
| 1 | Observability stack (5 layers: TreeObserver → BT Logger → WM Audit → Foxglove → Plan Audit) |
| 2 | ROS2 integration (WorldModelNode, PlannerNode, ExecutorNode, lifecycle management) |
| 3 | Perception integration (confidence thresholds, authority conflict detection) |
| 4 | PYRAMID service nodes (InvokeService async BT node, IPyramidService interface) |
| 5 | Thread safety (RCU snapshots, mutation queue, shared_mutex) |
| 6 | Hierarchical planning (ExecutePhaseAction, causal episode linking) |
| — | Multi-agent planning (AgentRegistry, GoalAllocator, DelegateToAgent, AgentDispatcher) |
| — | ROS2 extension wiring (all extensions wired into ROS2 layer) |

## Dependencies

| Dependency | Source | Purpose |
|------------|--------|---------|
| BehaviorTree.CPP 4.6.2 | FetchContent | BT execution + SQLite logging |
| LAPKT Devel2.0 (core) | FetchContent | STRIPS model, BRFS search |
| Google Test 1.14 | FetchContent | Unit/integration testing (73 tests) |
| SQLite3 | System | BT.CPP SQLite logging backend |
| websocketpp 0.8.2 | FetchContent | Foxglove WebSocket server |
| Standalone Asio 1.28 | FetchContent | Async I/O for websocketpp |

