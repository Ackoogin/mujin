# AME

`subprojects/AME` contains the Autonomous Mission Engine libraries, tests, PDDL domains, ROS2 package, and AME-owned developer tooling.

It depends on the `PCL` and `PYRAMID` subprojects and is structured as a future extraction boundary for the planning/execution stack.

## What It Builds

| Target | Purpose |
|--------|---------|
| `ame_core` | World model, PDDL parser, planner, plan compiler, BT nodes, audit logs, PCL/PYRAMID adapters |
| `ame_foxglove` | Optional Foxglove WebSocket bridge, enabled when `AME_FOXGLOVE=ON` |
| `ame_test_app` | Standalone UAV search-and-classify demo |
| `ame_pcl_main` | PCL-hosted AME component executable |
| `ame_ros2_lib` | ROS2 lifecycle wrappers under `ros2/` |

## Build And Test

From the workspace root:

```bat
cmake --preset default
cmake --build build --config Release -j%NUMBER_OF_PROCESSORS%
ctest --test-dir build --output-on-failure -C Release -R "WorldModel|Planner|PlanCompiler|PddlParser|E2EPipeline|Ame|PclIntegration"
```

Or use the helper:

```bat
subprojects\AME\scripts\build.bat
```

## Directory Map

| Path | Contents |
|------|----------|
| `include/ame/` | Public AME C++ interfaces |
| `src/lib/` | Core implementation |
| `src/nodes/` | BehaviorTree.CPP nodes |
| `src/apps/` | Standalone/demo executables |
| `tests/` | AME unit and integration tests |
| `domains/` | PDDL domain/problem fixtures |
| `ros2/` | ROS2 lifecycle package, messages, services, launch files, and tests |
| `tools/devenv/` | Local developer environment tooling |
| `doc/architecture/` | AME architecture reference |
| `doc/guides/` | Build, runtime, and operator guides |
| `doc/requirements/` | AME requirements |

## Documentation

| Document | Purpose |
|----------|---------|
| [Architecture overview](doc/architecture/01-overview.md) | System data flow, library boundaries, and design principles |
| [World model](doc/architecture/02-world-model.md) | Authoritative state model, type system, and grounding |
| [Planning](doc/architecture/03-planning.md) | PDDL parser, LAPKT integration, and action mapping |
| [Execution](doc/architecture/04-execution.md) | Plan compiler, BT nodes, mission executor, and replanning |
| [Observability](doc/architecture/05-observability.md) | BT events, world-model audit, Foxglove, and plan audit logs |
| [ROS2](doc/architecture/06-ros2.md) | Lifecycle nodes and deployment modes |
| [Quick start](doc/guides/quickstart.md) | Build, run, tests, and Foxglove setup |
| [Planning and execution guide](doc/guides/planning_execution_user_guide.md) | Mission flow, PDDL terms, BT nodes, and PYRAMID integration |
| [Remaining work](../../doc/todo/AME/TODO.md) | Open AME roadmap items |
