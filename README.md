# Mujin Workspace

This repository is now organised as a workspace containing three subprojects that can be split into separate repositories later with much less churn:

- `subprojects/PCL` — the low-level PYRAMID Container Library runtime and C/C++ wrappers
- `subprojects/PYRAMID` — PYRAMID core and tactical-objects components built on top of PCL
- `subprojects/AME` — the Autonomous Mission Engine planning/execution stack built on top of PCL and PYRAMID

AME remains the headline application in the workspace: a **PDDL planning + Behaviour Tree execution** pipeline for autonomous mission planning and execution, with full observability and audit trail support.

The system takes a formal mission description (PDDL domain and problem files), automatically generates a plan using classical AI planning, compiles that plan into an executable behaviour tree, and runs it — with replanning on failure. Every decision and state change is logged for post-hoc analysis and real-time monitoring.

## Key Capabilities

- **Automated planning** — LAPKT breadth-first search solver finds action sequences to achieve mission goals
- **Parallel execution** — causal analysis identifies independent action flows and executes them concurrently
- **Hierarchical planning** — `ExecutePhaseAction` decomposes missions into sub-phases, each planned and executed as a dynamic BT sub-tree
- **External service integration** — `InvokeService` node maps PDDL actions to async PYRAMID service calls with timeout and cancellation
- **Reactive monitoring** — configurable per-action precondition checking during execution
- **Replan on failure** — detects action failures, snapshots current state, replans, and resumes
- **Full observability** — 5-layer audit stack covering BT execution, world state changes, and planning episodes with hierarchical causal links
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
| **ExecutePhaseAction** | Hierarchical planning node — plans, compiles, and ticks a sub-tree for a phase goal set |
| **InvokeService** | Async PYRAMID service node — maps PDDL actions to external service calls with timeout/cancel |
| **Observability Stack** | 5 layers: TreeObserver stats, structured BT events, WM audit log, Foxglove bridge, plan audit trail |

For full architecture details, see `subprojects/AME/docs/architecture/` and the component-specific docs under `subprojects/PCL/docs/` and `subprojects/PYRAMID/docs/`.

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
subprojects\AME\scripts\build.bat
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
./build/subprojects/AME/src/ame_test_app

# Windows
build\subprojects\AME\src\Release\ame_test_app.exe
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
| 5 | PlanAuditLog | Full planning episodes with hierarchical causal links: initial state, goals, solver, plan, compiled BT XML, parent phase |

Connect Foxglove Studio to `ws://localhost:8765` for real-time monitoring. See `subprojects/AME/docs/guides/quickstart.md` for setup details.

## ROS2 Integration

The core library (`ame_core`) is ROS-agnostic. The `subprojects/AME/ros2/` directory contains an `ament_cmake` package with lifecycle node wrappers:

- **WorldModelNode** — owns state, exposes get/set/query/load_domain services, publishes `/world_state`
- **PlannerNode** — action server at `~/plan`, domain loading via `~/load_domain` service
- **ExecutorNode** — ticks BT at 50 Hz, publishes `/executor/bt_events`

Supports in-process, distributed, multi-agent, and multi-planner deployment modes. Multiple PlannerNode instances can run with different domain models; domains can be loaded from files (deployment) or pushed via service calls (devenv/testing). See `subprojects/AME/docs/architecture/06-ros2.md` for details.

## Workspace Structure

```
subprojects/
  PCL/
    include/pcl/         Public PCL headers
    src/                 PCL C runtime and transport sources
    tests/               PCL-focused unit and integration tests
    examples/            Minimal PCL examples
    docs/                PCL design and component docs
  PYRAMID/
    core/                PYRAMID shared runtime/services layer
    tactical_objects/    Tactical objects libraries and docs
    tests/               Tactical objects and codec/bridge tests
    examples/            Ada/C++/codec/codegen examples
    proto/               Protocol and data-model definitions
    pim/                 Model/code generation tooling
    docs/                Service/codegen/tactical-object docs
  AME/
    include/ame/         Public AME headers
    src/                 AME libraries, nodes, app, bindings
    tests/               AME-focused unit and integration tests
    domains/             PDDL domains and problems
    ros2/                ROS2 package and tests
    docs/                AME architecture, guides, research, assurance docs
doc/                     Workspace-level coding style and licence docs
cmake/                   Shared CMake support files
```

## Documentation

| Document | Audience | Contents |
|----------|----------|----------|
| [Stakeholder Summary](subprojects/AME/docs/stakeholder_summary.md) | Programme managers, non-technical stakeholders | High-level approach, benefits, and status |
| [Architecture](subprojects/AME/docs/architecture/) | Engineers | AME architecture and runtime structure |
| [Quick Start](subprojects/AME/docs/guides/quickstart.md) | Developers | Build, run, test, and Foxglove setup |
| [Planning & Execution User Guide](subprojects/AME/docs/guides/planning_execution_user_guide.md) | Operators, integrators, developers | Mission flow, PDDL terms, BT nodes, and PYRAMID integration |
| [Remaining Work](subprojects/AME/docs/TODO.md) | Engineers, programme leads | Temporal planning, hardening, future work |
| [Assurance Plan](subprojects/AME/docs/autonomy_assurance_plan.md) | Safety engineers, assessors | SACE/AMLAS/DSTL autonomy assurance framework |
| [Service Binding Codegen](subprojects/PYRAMID/docs/service_binding_codegen.md) | Engineers | PYRAMID service/codegen pipeline and generated bindings |
| [PCL Component Design](subprojects/PCL/docs/component_container_design.md) | Engineers | PCL component/container integration design |

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
