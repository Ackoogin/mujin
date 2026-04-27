# System Overview

PDDL planning + behaviour tree execution with a shared world model, full observability, and optional ROS2 deployment.

## Data Flow

```
PDDL Domain/Problem
        |
        v
   WorldModel  --> Planner (LAPKT BRFS) --> PlanCompiler --> BT XML
   (facts +                                                       |
    actions)                                                      v
        |                                                  BT.CPP Executor
        +--> WmAuditLog (Layer 3)                          (tickWhileRunning)
                                                                  |
                                              +-------------------+
                                              |                   |
                                        TreeObserver        AmeBTLogger
                                         (Layer 1)           (Layer 2)
                                                                  |
                                                         FoxgloveBridge (Layer 4)
                                                         ws://localhost:8765

PlanAuditLog (Layer 5) -- records each planning episode independently
```

## Components

| Component | Role | Header |
|-----------|------|--------|
| **WorldModel** | Single authoritative state store -- typed objects + boolean predicates as a grounded bitset | `world_model.h` |
| **TypeSystem** | Object/type hierarchy with single inheritance | `type_system.h` |
| **Planner** | Stateless STRIPS solver (LAPKT BRFS) -- same input always produces the same plan | `planner.h` |
| **PddlParser** | Loads PDDL domain + problem files into WorldModel | `pddl_parser.h` |
| **ActionRegistry** | Maps PDDL action names to BT node types, sub-tree templates, or pre-authored sub-trees | `action_registry.h` |
| **PlanCompiler** | Builds a causal dependency graph from the plan, extracts parallel flows, emits BT XML | `plan_compiler.h` |
| **BT.CPP Executor** | Ticks the compiled behaviour tree; BT nodes read/write world state directly | `bt_nodes/` |
| **MissionExecutor** | Top-level tick loop with replan-on-failure | `main.cpp` |
| **Observability** | 5-layer audit stack: TreeObserver, AmeBTLogger, WmAuditLog, FoxgloveBridge, PlanAuditLog | see [05-observability.md](05-observability.md) |

## Design Principles

- **Single source of truth** -- WorldModel owns all state. The BT blackboard is a read-only view. LAPKT gets a snapshot projection.
- **ROS-agnostic core** -- `ame_core` has no ROS2 dependency. ROS2 integration is a separate adapter layer.
- **Separation of concerns** -- planning model (PDDL) and execution model (BT) are independently replaceable via ActionRegistry.
- **Sink-based observability** -- all logging is callback-driven, composable, and non-blocking.
- **Automatic recovery** -- action failures trigger replanning from current state, not mission abort.

## Library Boundaries

| Library | Contents | Dependencies |
|---------|----------|-------------|
| `ame_core` | WorldModel, Planner, PlanCompiler, ActionRegistry, PddlParser, BT nodes, all loggers | BT.CPP, lapkt_core |
| `ame_foxglove` | FoxgloveBridge WebSocket server | ame_core, websocketpp, asio |
| `ame_test_app` | Demo executable (`src/ame/apps/main.cpp`) | ame_core, optionally ame_foxglove |
| `ame_ros2_lib` | WorldModelNode, PlannerNode, ExecutorNode, RosWmBridge, LifecycleManager | ame_core, rclcpp, rclcpp_action, rclcpp_lifecycle |

`ame_foxglove` is a separate static library so `ame_core` stays dependency-free. Guard any Foxglove code with `#if defined(AME_FOXGLOVE)`.

## Dependencies

| Dependency | Source | Purpose |
|------------|--------|---------|
| BehaviorTree.CPP 4.6.2 | FetchContent | BT execution + SQLite logging |
| LAPKT Devel2.0 (core) | FetchContent | STRIPS model, BRFS search |
| Google Test 1.14 | FetchContent | Unit/integration testing (73 tests) |
| SQLite3 | System | BT.CPP SQLite logging backend |
| websocketpp 0.8.2 | FetchContent | Foxglove WebSocket server (optional) |
| Standalone Asio 1.28 | FetchContent | Async I/O for websocketpp (optional) |

All dependencies are fetched automatically via CMake FetchContent.

