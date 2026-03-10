# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MUJIN is a **PDDL planning + BehaviorTree execution pipeline** for autonomous mission planning and execution. It takes formal mission descriptions (PDDL), automatically plans using LAPKT classical AI search, compiles plans into executable behaviour trees, and runs them with replan-on-failure. A 5-layer observability stack provides full auditability.

See `README.md` for a comprehensive project overview, and `doc/stakeholder_summary.md` for a non-technical summary aimed at programme managers and other stakeholders.

## Documentation

| Document | Purpose |
|----------|---------|
| `README.md` | Project overview, quick start, architecture summary |
| `doc/stakeholder_summary.md` | High-level approach and status for non-technical stakeholders |
| `doc/architecture/` | **Consolidated architecture reference** (see below) |
| `doc/quickstart.md` | Getting started, build, run, Foxglove Studio setup |
| `doc/plan.md` | Implementation plan and gap analysis (steps 1–8 done) |
| `doc/TODO.md` | Consolidated remaining work: temporal planning, hardening, future |
| `doc/temporal_extension_research.md` | **Temporal planner evaluation**: OPTIC, POPF, TFD, Aries, STN, BT.CPP integration |
| `doc/autonomy_assurance_plan.md` | SACE/AMLAS/DSTL safety assurance framework |
| `doc/neuro_symbolic_reasoning.md` | Neural/LLM integration options and architecture |

### Architecture Reference (`doc/architecture/`)

| File | Contents |
|------|----------|
| `01-overview.md` | System overview, data flow, design principles, library boundaries, dependencies |
| `02-world-model.md` | WorldModel API, TypeSystem, eager grounding, authoritative state |
| `03-planning.md` | PDDL parser, Planner (LAPKT), ActionRegistry API, adding new actions |
| `04-execution.md` | PlanCompiler algorithm, BT node types, MissionExecutor, replanning |
| `05-observability.md` | 5-layer observability stack (TreeObserver → FoxgloveBridge → PlanAuditLog) |
| `06-ros2.md` | ROS2 lifecycle nodes, deployment modes, build/run instructions |
| `07-extensions.md` | Perception bridge, PYRAMID service, thread safety, hierarchical planning, temporal planning |

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

The full architecture is documented in `doc/architecture/` (7 numbered files). Key points for development:

- **WorldModel** (`include/mujin/world_model.h`) is the central shared state with eager grounding and audit callbacks. See `doc/architecture/02-world-model.md`.
- **LAPKT integration**: `Planner::solve()` calls `WorldModel::projectToSTRIPS()`. LAPKT is built from source as `lapkt_core` static library; MSVC compat shims in `cmake/compat/`.
- **`mujin_foxglove`** is a separate static library. Guard Foxglove code with `#if defined(MUJIN_FOXGLOVE)`.
- **Library boundaries**, **adding new PDDL actions**, and **ROS2 build/run** are all in the architecture docs.

| Library | Contents | Dependencies |
|---------|----------|-------------|
| `mujin_core` | WorldModel, Planner, PlanCompiler, ActionRegistry, PddlParser, BT nodes, all loggers | BT.CPP, lapkt_core |
| `mujin_foxglove` | FoxgloveBridge WebSocket server | mujin_core, websocketpp, asio |
| `mujin_test_app` | Demo executable (src/main.cpp) | mujin_core, optionally mujin_foxglove |
| `mujin_ros2_lib` | WorldModelNode, PlannerNode, ExecutorNode, RosWmBridge, LifecycleManager | mujin_core, rclcpp, rclcpp_action, rclcpp_lifecycle |
