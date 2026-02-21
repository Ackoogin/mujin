# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

This project uses MSVC on Windows via CMake. All dependencies (BehaviorTree.CPP, LAPKT, websocketpp, asio, googletest) are fetched automatically by CMake on first configure — no Conan or manual installs needed.

**Configure and build (Windows, VS 2022):**
```bat
cmake --preset default
cmake --build build --config Release -j%NUMBER_OF_PROCESSORS%
```

**Or using the helper script:**
```bat
build.bat
```

**Debug build:**
```bat
cmake --preset default
cmake --build build --config Debug -j%NUMBER_OF_PROCESSORS%
```

**Without Foxglove bridge (removes websocketpp/asio):**
```bat
cmake --preset default -DMUJIN_FOXGLOVE=OFF
cmake --build build --config Release
```

The preset (`CMakePresets.json`) sets generator to "Visual Studio 17 2022", arch x64, binaryDir = `build/`.

## Tests

```bat
ctest --test-dir build --output-on-failure -C Release
```

**Run a single test binary:**
```bat
build\tests\Release\test_world_model.exe
build\tests\Release\test_observability.exe
```

**Run a specific test case by name:**
```bat
build\tests\Release\test_world_model.exe --gtest_filter=WorldModel.SetAndGetFact
```

All 73 tests should pass across: WorldModel, TypeSystem, ActionRegistry, PlanCompiler, Planner, PddlParser, integration (BT nodes), end-to-end pipeline, and observability (Layers 1–5).

## Run the demo

```bat
build\src\Release\mujin_test_app.exe
```

Produces three JSONL output files in the working directory: `mujin_bt_events.jsonl`, `mujin_wm_audit.jsonl`, `mujin_plan_audit.jsonl`.

## Architecture

The system is a **PDDL planning + BehaviorTree execution pipeline** with full observability. The core library (`mujin_core`) is deliberately ROS-agnostic.

### Data flow

```
PDDL Domain/Problem
        |
        v
   WorldModel  ──► Planner (LAPKT BRFS) ──► PlanCompiler ──► BT XML
   (facts +                                                       |
    actions)                                                      v
        |                                                  BT.CPP Executor
        └──► WmAuditLog (Layer 3)                          (tickWhileRunning)
                                                                  |
                                              ┌───────────────────┤
                                              │                   │
                                        TreeObserver        MujinBTLogger
                                         (Layer 1)           (Layer 2)
                                                                  |
                                                         FoxgloveBridge (Layer 4)
                                                         ws://localhost:8765

PlanAuditLog (Layer 5) — records each planning episode independently
```

### Key design decisions

**WorldModel** (`include/mujin/world_model.h`) is the central shared state. It eagerly grounds all fluents when predicates/objects are registered, storing them as a bitset (`vector<uint64_t>`). All components access it by reference. Fact changes fire an `AuditCallback` for observability.

**ActionRegistry** (`include/mujin/action_registry.h`) bridges PDDL action names to BT.CPP node types or XML subtree templates. `PlanCompiler` calls `registry.resolve(name, params)` to emit action XML during compilation.

**BT nodes** (`include/mujin/bt_nodes/`) — `CheckWorldPredicate` reads a fluent, `SetWorldPredicate` writes one. They get the `WorldModel*` from the BT blackboard. New PDDL actions need corresponding BT node implementations registered here plus an `ActionRegistry` mapping.

**Observability** is sink-based and layered:
- Layer 1: `TreeObserver` (BT.CPP built-in, in-memory stats)
- Layer 2: `MujinBTLogger` — callback + file sinks, emits JSONL
- Layer 3: `WmAuditLog` — hooked into `WorldModel::setAuditCallback()`
- Layer 4: `FoxgloveBridge` — WebSocket server (port 8765), receives callbacks from Layers 2–3
- Layer 5: `PlanAuditLog` — records full planning episodes (init state, goals, plan, BT XML)

**LAPKT integration**: `Planner::solve()` calls `WorldModel::projectToSTRIPS()` to build an `aptk::STRIPS_Problem`, runs BRFS, and returns plan steps as indices into `WorldModel::groundActions()`. LAPKT is built from source as `lapkt_core` static library (not its own CMake); MSVC compat shims are in `cmake/compat/`.

**`mujin_foxglove`** is a separate static library so `mujin_core` stays dependency-free. Guard any Foxglove code with `#if defined(MUJIN_FOXGLOVE)`.

### Library boundaries

| Library | Contents | Dependencies |
|---------|----------|-------------|
| `mujin_core` | WorldModel, Planner, PlanCompiler, ActionRegistry, PddlParser, BT nodes, all loggers | BT.CPP, lapkt_core |
| `mujin_foxglove` | FoxgloveBridge WebSocket server | mujin_core, websocketpp, asio |
| `mujin_test_app` | Demo executable (src/main.cpp) | mujin_core, optionally mujin_foxglove |
| `mujin_ros2_lib` | WorldModelNode, PlannerNode, ExecutorNode, RosWmBridge, LifecycleManager | mujin_core, rclcpp, rclcpp_action, rclcpp_lifecycle |

### Adding a new PDDL action with BT execution

1. Implement a BT node (subclass `BT::SyncActionNode` or `BT::StatefulActionNode`) in `include/mujin/bt_nodes/` + `src/bt_nodes/`
2. Add it to `mujin_core` in `src/CMakeLists.txt`
3. Register in `ActionRegistry` via `registerAction(pddl_name, bt_node_type)` or `registerActionSubTree(...)`
4. Register the BT node type with the BT.CPP factory: `factory.registerNodeType<MyNode>("MyNode")`
5. In ROS2 mode: also register on `ExecutorNode::factory()` in `combined_main.cpp`

### ROS2 Node Wrappers (`ros2/`)

The `ros2/` directory is a separate `ament_cmake` package (ROS2 Jazzy, `D:\Dev\ros2-windows`). It wraps `mujin_core` with three lifecycle nodes without modifying the core.

**Build sequence:**
```bat
:: 1. Install mujin_core (adds cmake/mujin_coreConfig.cmake.in install rules)
cmake --preset default -DCMAKE_INSTALL_PREFIX=build/install
cmake --build build --config Release -j%NUMBER_OF_PROCESSORS%
cmake --install build --config Release

:: 2. Build the ROS2 package
call D:\Dev\ros2-windows\setup.bat
colcon build --packages-select mujin_ros2 --base-paths ros2 ^
  --cmake-args -DCMAKE_PREFIX_PATH="D:/Dev/ros2-windows;D:/Dev/repo/mujin/build/install"
```

**Run tests:**
```bat
colcon test --packages-select mujin_ros2
colcon test-result --verbose
```

**In-process demo (all nodes on one executor):**
```bat
call install\setup.bat
ros2 launch mujin_ros2 mujin_inprocess.launch.py ^
  pddl_file:=domains/uav_search/domain.pddl ^
  problem_file:=domains/uav_search/problem.pddl
```

**ROS2 node roles:**
- `WorldModelNode` — owns canonical `WorldModel`; services `~/get_fact`, `~/set_fact`, `~/query_state`; publisher `/world_state` (reliable, transient_local)
- `PlannerNode` — `rclcpp_action::Server` at `/mujin/plan`; BRFS runs on dedicated thread; returns `bt_xml` in result
- `ExecutorNode` — ticks BT at 50 Hz via `rclcpp::Timer`; `factory()` exposes BT.CPP factory for registering action nodes; publishes `/executor/bt_events` (JSON lines)
- In-process mode: `setInProcessWorldModel(WorldModel*)` skips service calls entirely (use `SingleThreadedExecutor` to avoid WorldModel race conditions)
- Distributed mode: BT nodes use `RosCheckWorldPredicate`/`RosSetWorldPredicate` which call services with 500ms timeout

### Extension roadmap

See `doc/extensions.md`. Completed: Observability (all 5 layers), ROS2 Node Wrappers. Next: Perception Integration (Extension 3, requires ROS2 services already in place).
