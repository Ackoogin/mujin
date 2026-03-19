# Autonomous Mission Engine (AME)

A **PDDL planning + Behaviour Tree execution** pipeline for autonomous mission planning and execution, with full observability and audit trail support.

The system takes a formal mission description (PDDL domain and problem files), automatically generates a plan using classical AI planning, compiles that plan into an executable behaviour tree, and runs it — with replanning on failure. Every decision and state change is logged for post-hoc analysis and real-time monitoring.

## Key Capabilities

- **Automated planning** — LAPKT breadth-first search solver finds action sequences to achieve mission goals
- **Parallel execution** — causal analysis identifies independent action flows and executes them concurrently
- **Reactive monitoring** — configurable per-action precondition checking during execution
- **Replan on failure** — detects action failures, snapshots current state, replans, and resumes
- **Full observability** — 5-layer audit stack covering BT execution, world state changes, and planning episodes
- **Live monitoring** — Foxglove Studio integration via WebSocket for real-time visualization
- **ROS2 integration** — optional lifecycle node wrappers for distributed robotic deployments

## Architecture Overview

```
PDDL Domain/Problem
        |
        v
   WorldModel  --> Planner (LAPKT BRFS) --> PlanCompiler --> BT.CPP Executor
   (facts +                                                       |
    actions)                                                      v
        ^                                                  Tick loop with
        |                                                  replan-on-failure
        +---- effects + perception updates -----------------------+
```

| Component | Role |
|-----------|------|
| **WorldModel** | Single authoritative state store — typed objects + boolean predicates as a grounded bitset |
| **Planner** | Stateless STRIPS solver (LAPKT BRFS) — same input always produces the same plan |
| **PlanCompiler** | Builds a causal dependency graph from the plan, extracts parallel flows, emits BT XML |
| **BT.CPP Executor** | Ticks the compiled behaviour tree; BT nodes read/write world state directly |
| **ActionRegistry** | Maps PDDL action names to BT node types, sub-tree templates, or pre-authored sub-trees |
| **Observability Stack** | 5 layers: TreeObserver stats, structured BT events, WM audit log, Foxglove bridge, plan audit trail |

For full architecture details, see `doc/architecture/` (7 numbered files covering WorldModel, planning, execution, observability, ROS2, and extensions).

## Quick Start

### Prerequisites

- CMake 3.21+
- C++17 compiler (MSVC 2019+, GCC 9+, Clang 10+)
- Git (for automatic dependency fetching)

All dependencies (BehaviorTree.CPP, LAPKT, websocketpp, asio, GoogleTest) are fetched automatically by CMake.

### Build

```bash
# Configure and build
cmake --preset default
cmake --build build --config Release -j$(nproc)

# Or on Windows with the helper script
build.bat
```

To disable the Foxglove bridge (removes websocketpp/asio dependencies):

```bash
cmake --preset default -DAME_FOXGLOVE=OFF
cmake --build build --config Release
```

### Run Tests

```bash
ctest --test-dir build --output-on-failure -C Release
```

All 73 tests cover: WorldModel, TypeSystem, ActionRegistry, PlanCompiler, Planner, PDDL Parser, BT integration, end-to-end pipeline, and observability (Layers 1-5).

### Run the Demo

```bash
# Linux/macOS
./build/src/ame_test_app

# Windows
build\src\Release\ame_test_app.exe
```

Runs the UAV search-and-classify example end-to-end and produces three JSONL output files:

| File | Contents |
|------|----------|
| `ame_bt_events.jsonl` | BT node state transitions (Layer 2) |
| `ame_wm_audit.jsonl` | World model fact changes with source attribution (Layer 3) |
| `ame_plan_audit.jsonl` | Full planning episode audit trail (Layer 5) |

## Observability

The system replaces the paid Groot2 monitoring tool with an open, layered observability stack:

| Layer | Component | What It Captures |
|-------|-----------|-----------------|
| 1 | TreeObserver | Per-node tick counts, success/failure rates, timing |
| 2 | AmeBTLogger | Structured JSON events for every BT node state transition |
| 3 | WmAuditLog | Every world model fact change with source tag and timestamp |
| 4 | FoxgloveBridge | WebSocket server for live Foxglove Studio visualization |
| 5 | PlanAuditLog | Full planning episodes: initial state, goals, solver, plan, compiled BT XML |

Connect Foxglove Studio to `ws://localhost:8765` for real-time monitoring. See `doc/quickstart.md` for setup details.

## ROS2 Integration

The core library (`ame_core`) is ROS-agnostic. The `ros2/` directory contains an `ament_cmake` package with lifecycle node wrappers:

- **WorldModelNode** — owns state, exposes get/set/query services, publishes `/world_state`
- **PlannerNode** — stateless action server at `/ame/plan`
- **ExecutorNode** — ticks BT at 50 Hz, publishes `/executor/bt_events`

Supports both in-process (single executor) and distributed (service-backed) deployment modes from the same code. See `doc/quickstart.md` and `CLAUDE.md` for build instructions.

## Project Structure

```
include/ame/           Core library headers
  world_model.h          Authoritative state store
  type_system.h          Object/type hierarchy
  action_registry.h      PDDL-to-BT action mapping
  plan_compiler.h        Plan-to-BT compilation
  planner.h              LAPKT solver wrapper
  pddl_parser.h          PDDL file loading
  bt_logger.h            Layer 2: structured BT event stream
  wm_audit_log.h         Layer 3: world model audit log
  plan_audit_log.h       Layer 5: plan audit trail
  foxglove_bridge.h      Layer 4: Foxglove WebSocket bridge
  bt_nodes/              BT node implementations
src/                     Core library source
tests/                   73 unit, integration, and e2e tests
domains/                 PDDL domain and problem files
  uav_search/            UAV search-and-classify example domain
ros2/                    ROS2 lifecycle node wrappers (ament_cmake)
doc/                     Documentation
  architecture/          Consolidated architecture reference (7 numbered files)
  quickstart.md          Getting started and Foxglove setup
  TODO.md                Remaining work: temporal planning, hardening, future
  plan.md                Implementation plan and gap analysis (historical)
  autonomy_assurance_plan.md   SACE/AMLAS/DSTL safety assurance plan
  neuro_symbolic_reasoning.md  Neural integration options
  neuro_symbolic_reasoning_review.md  Review of documented neuro-symbolic approaches
```

## Documentation

| Document | Audience | Contents |
|----------|----------|----------|
| [Stakeholder Summary](doc/stakeholder_summary.md) | Programme managers, non-technical stakeholders | High-level approach, benefits, and status |
| [Architecture](doc/architecture/) | Engineers | Full technical architecture (7 numbered files) |
| [Quick Start](doc/quickstart.md) | Developers | Build, run, test, and Foxglove setup |
| [Remaining Work](doc/TODO.md) | Engineers, programme leads | Temporal planning, hardening, future work |
| [Assurance Plan](doc/autonomy_assurance_plan.md) | Safety engineers, assessors | SACE/AMLAS/DSTL autonomy assurance framework |
| [Neuro-Symbolic Integration](doc/neuro_symbolic_reasoning.md) | Engineers, architects | Options for adding AI/ML capabilities |
| [Neuro-Symbolic Review Report](doc/neuro_symbolic_reasoning_review.md) | Engineers, architects, programme leads | Review of documented neuro-symbolic approaches, risks, and adoption order |

## Dependencies

| Dependency | Purpose |
|------------|---------|
| BehaviorTree.CPP 4.6.2 | Behaviour tree execution engine |
| LAPKT Devel2.0 | Classical AI planning (STRIPS model + search) |
| Google Test 1.14 | Unit and integration testing |
| websocketpp 0.8.2 | Foxglove WebSocket server (optional) |
| Standalone Asio 1.28 | Async I/O for WebSocket (optional) |

All dependencies are fetched automatically via CMake FetchContent.

## License

See project licence file for details.
