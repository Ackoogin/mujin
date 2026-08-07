# AME

`subprojects/AME` contains the Autonomous Mission Engine libraries, tests, PDDL domains, ROS2 package, and AME-owned developer tooling.

It depends on the `PCL` and `PYRAMID` subprojects and is structured as a future extraction boundary for the planning/execution stack.

## What It Builds

### Libraries

| Target | Build | Purpose |
|--------|-------|---------|
| `ame_core` | always | World model, PDDL parser, planner, plan compiler, BT nodes, audit logs, PCL/PYRAMID adapters |
| `ame_foxglove` | `AME_FOXGLOVE` (default ON) | Foxglove WebSocket bridge for live monitoring |
| `ame_neuro` | `AME_NEURO` (default OFF) | Neuro-symbolic core infrastructure (propose–verify–fallback backends, verifiers, neuro audit log); also compiles the `ame_core` seam via `AME_NEURO=1` |
| `_ame_py` | `AME_BUILD_PYTHON` (default OFF) | pybind11 Python module wrapping `ame_core`; copied to `build/python/` and consumed by the AutoMTK Python AME backend |

### Executables

| Target | Build | Purpose |
|--------|-------|---------|
| `ame_test_app` | always | Standalone UAV search-and-classify demo (emits `ame_bt_events.jsonl`, `ame_wm_audit.jsonl`, `ame_plan_audit.jsonl`) |
| `ame_pcl_main` | always | PCL-hosted AME component executable (no ROS2 dependency) |
| `contingency_verifier` | always | Exhaustive safe-state reachability / contingency-domain verifier tool |
| `ame_authoring_tool` | `AME_BUILD_AUTHORING` (default OFF) | Graphical PDDL/scenario authoring tool (requires SDL2/OpenGL) |

### ROS2 package (`ros2/`, built under ament)

| Target | Build | Purpose |
|--------|-------|---------|
| `ame_ros2_lib` | ament build | Shared library of lifecycle wrappers (WorldModelNode, PlannerNode, ExecutorNode, RosWmBridge, LifecycleManager) |
| `world_model_node`, `planner_node`, `executor_node`, `agent_dispatcher_node` | ament build | Per-component ROS2 lifecycle node executables |
| `ame_combined` | ament build | Single-process executable hosting all AME lifecycle nodes |
| `lifecycle_manager` | ament build | Lifecycle orchestration executable |
| `foxglove_bridge` | ament build, `AME_FOXGLOVE` | ROS2 Foxglove bridge node executable |

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
| `src/lib/neuro/` | Neuro-symbolic infrastructure (`ame_neuro`, optional) |
| `src/nodes/` | BehaviorTree.CPP nodes |
| `src/apps/` | Standalone/demo executables (`ame_test_app`, `ame_pcl_main`, `contingency_verifier`) |
| `src/bindings/` | pybind11 source for the `_ame_py` Python module (optional) |
| `src/authoring/` | Optional native graphical authoring tool |
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
| [Authoring tool user guide](doc/guides/authoring_tool_user_guide.md) | Graphical domain/scenario authoring, validation, preview, and import/export workflow |
