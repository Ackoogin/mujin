# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.
## Coding standards 
See `doc/CODING_STYLE.md` for coding standards.


## Project Overview

AME is a **PDDL planning + BehaviorTree execution pipeline** for autonomous mission planning and execution. It takes formal mission descriptions (PDDL), automatically plans using LAPKT classical AI search, compiles plans into executable behaviour trees, and runs them with replan-on-failure. A 5-layer observability stack provides full auditability.

See `README.md` for a comprehensive project overview, and `doc/reports/AME/stakeholder_summary.md` for a non-technical summary aimed at programme managers and other stakeholders.

## Documentation

| Document | Purpose |
|----------|---------|
| `README.md` | Project overview, quick start, architecture summary |
| `doc/reports/AME/stakeholder_summary.md` | High-level approach and status for non-technical stakeholders |
| `subprojects/AME/doc/architecture/` | **Consolidated architecture reference** (see below) |
| `subprojects/AME/doc/guides/quickstart.md` | Getting started, build, run, Foxglove Studio setup |
| `doc/todo/AME/TODO.md` | Consolidated remaining work: temporal planning, hardening, future |
| `doc/research/AME/temporal_extension_research.md` | **Temporal planner evaluation**: OPTIC, POPF, TFD, Aries, STN, BT.CPP integration |
| `doc/plans/AME/autonomy_assurance_plan.md` | SACE/AMLAS/DSTL safety assurance framework |
| `doc/research/AME/neuro_symbolic_reasoning.md` | Neural/LLM integration options and architecture |

### Architecture Reference (`subprojects/AME/doc/architecture/`)

| File | Contents |
|------|----------|
| `01-overview.md` | System overview, data flow, design principles, library boundaries, dependencies |
| `02-world-model.md` | WorldModel API, TypeSystem, eager grounding, authoritative state |
| `03-planning.md` | PDDL parser, Planner (LAPKT), ActionRegistry API, adding new actions |
| `04-execution.md` | PlanCompiler algorithm, BT node types, MissionExecutor, replanning |
| `05-observability.md` | 5-layer observability stack (TreeObserver → FoxgloveBridge → PlanAuditLog) |
| `06-ros2.md` | ROS2 lifecycle nodes, deployment modes, build/run instructions |
| `07-extensions.md` | Extensions roadmap (future/cross-cutting: temporal + neuro-symbolic) |

## Build

This project uses CMake, with MSVC/Visual Studio 2022 the primary Windows toolchain. Core dependencies (BehaviorTree.CPP, LAPKT, websocketpp, asio, googletest, and optional PYRAMID transport dependencies) are fetched automatically by CMake on first configure -- no Conan or manual installs needed.

`CMakePresets.json` lives at the repository root and defines:

| Preset type | Names | Purpose |
|-------------|-------|---------|
| Configure | `default` | Standard workspace build in `build/` using project default options |
| Configure | `all-on` | Optional dependencies enabled in `build-all-enabled/`: Foxglove, FlatBuffers, gRPC, Protobuf, ROS2 |
| Configure | `all-off` | Optional dependencies disabled in `build-all-off/`: Foxglove, FlatBuffers, gRPC, Protobuf, ROS2 |
| Build | `release`, `debug` | Release/Debug builds using the `default` configure preset |
| Build | `all-on-release` | Release build using `all-on` |
| Build | `all-off-release`, `all-off-debug` | Release/Debug builds using `all-off` |
| Test | `all-on-release`, `all-off-release`, `all-off-debug` | CTest runs with `outputOnFailure` enabled |

The presets currently do not force a generator or architecture. On Windows, use a Visual Studio 2022 x64 developer environment or select the Visual Studio 2022 x64 kit/generator in your CMake frontend.

**Configure and build (Windows, VS 2022):**
```bat
cmake --preset default
cmake --build --preset release --parallel %NUMBER_OF_PROCESSORS%
```

**Or using the helper script:**
```bat
subprojects\AME\scripts\build.bat
```

**Debug build:**
```bat
cmake --preset default
cmake --build --preset debug --parallel %NUMBER_OF_PROCESSORS%
```

**All optional dependencies disabled:**
```bat
cmake --preset all-off
cmake --build --preset all-off-release
```

**Build Ada E2E test binaries (requires GNAT/gprbuild; not built by default):**
```bat
cmake --build --preset release --target pyramid_ada_all
```
Ada targets (`ada_tobj_client_build`, `ada_active_find_e2e_build`, `pyramid_bridge_ada_build`, `ada_test_generated_bindings_build`) are excluded from the default `ALL` build so that routine `cmake --build` and VS Code pre-test builds remain fast. Run the target above before running Ada E2E CTest tests. Ada tests SKIP gracefully when the binaries are absent.

**VS Code / CMake Tools:**

1. Open `D:\Dev\repo\mujin` as the workspace folder.
2. Run `CMake: Enable CMake Presets integration` from the command palette.
3. Use `CMake: Select Configure Preset`, `CMake: Select Build Preset`, and `CMake: Select Test Preset`.
4. If the picker only shows `Release`, `Debug`, or `[Default]`, presets integration is not active yet.

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
build\subprojects\AME\src\Release\ame_test_app.exe
```

Produces three JSONL output files in the working directory: `ame_bt_events.jsonl`, `ame_wm_audit.jsonl`, `ame_plan_audit.jsonl`.

## Architecture

The full architecture is documented in `subprojects/AME/doc/architecture/` (7 numbered files). Key points for development:

- **WorldModel** (`subprojects/AME/include/ame/world_model.h`) is the central shared state with eager grounding and audit callbacks. See `subprojects/AME/doc/architecture/02-world-model.md`.
- **LAPKT integration**: `Planner::solve()` calls `WorldModel::projectToSTRIPS()`. LAPKT is built from source as `lapkt_core` static library; MSVC compat shims in `cmake/compat/`.
- **`ame_foxglove`** is a separate static library. Guard Foxglove code with `#if defined(AME_FOXGLOVE)`.
- **Library boundaries**, **adding new PDDL actions**, and **ROS2 build/run** are all in the architecture docs.

| Library | Contents | Dependencies |
|---------|----------|-------------|
| `ame_core` | WorldModel, Planner, PlanCompiler, ActionRegistry, PddlParser, BT nodes, all loggers | BT.CPP, lapkt_core |
| `ame_foxglove` | FoxgloveBridge WebSocket server | ame_core, websocketpp, asio |
| `ame_test_app` | Demo executable (`subprojects/AME/src/apps/main.cpp`) | ame_core, optionally ame_foxglove |
| `ame_ros2_lib` | WorldModelNode, PlannerNode, ExecutorNode, RosWmBridge, LifecycleManager | ame_core, rclcpp, rclcpp_action, rclcpp_lifecycle |
