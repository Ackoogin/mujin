# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.
## Coding standards 
See `doc/CODING_STYLE.md` for coding standards.

## Writing style (docs, comments, summaries)

Documentation here is read by software engineers of varying experience
levels, often new to the project, and some readers are not native English
speakers. Write for them:

- Use plain language and complete sentences; avoid fragment/arrow/
  abbreviation chains.
- Do not coin compressed jargon when a plain phrase says the same thing
  (write "how much smaller the generated set gets if this root is
  dropped", not "per-root marginal").
- Watch for field-specific terms that mean something different to a
  typical software reader or nothing at all ("closure", "drop",
  "pinned", "vendored"). Recurring repository terms are defined in
  [`doc/GLOSSARY.md`](doc/GLOSSARY.md) — link there or give the
  plain-words meaning in one sentence at first use.
- Assume the reader has not been following the work session that
  produced the document.


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
| `subprojects/AME/doc/guides/contingency_verifier.md` | **Contingency verifier tool**: exhaustive safe-state reachability, domains, pruning. The search itself is `ame_core`'s `ContingencySearch`, shared with the authoring tool |
| `doc/todo/AME/TODO.md` | Consolidated remaining work: temporal planning, hardening, future |
| `doc/roadmap/` + `tools/roadmap/README.md` | **Roadmap dashboards**: roadmap data files, the generator that turns them into standalone HTML pages, and the update flow |
| `doc/research/AME/temporal_extension_research.md` | **Temporal planner evaluation**: OPTIC, POPF, TFD, Aries, STN, BT.CPP integration |
| `doc/plans/AME/autonomy_assurance_plan.md` | SACE/AMLAS/DSTL safety assurance framework |
| `doc/plans/AME/authoring_tool_plan.md` | **Authoring tool plan**: what the graphical authoring tool does today, what is left to build (workstream D — the tool accepts less PDDL than `PddlParser` does), and the non-specialist walkthrough that gates a release |
| `subprojects/AME/doc/guides/authoring_tool_user_guide.md` | How to build, launch and use the graphical authoring tool |
| `doc/research/AME/neuro_symbolic_reasoning.md` | Neural/LLM integration options and architecture |
| `doc/plans/AME/neuro_symbolic_infrastructure_plan.md` | **Neuro-symbolic core infrastructure work plan**: propose-verify-fallback envelope, backend/verifier abstractions, audit, policy, test harness |

### Architecture Reference (`subprojects/AME/doc/architecture/`)

| File | Contents |
|------|----------|
| `01-overview.md` | System overview, data flow, design principles, library boundaries, dependencies |
| `02-world-model.md` | WorldModel API, TypeSystem, eager grounding, authoritative state |
| `03-planning.md` | PDDL parser, Planner (LAPKT), ActionRegistry API, contingency domains, contingency verifier |
| `04-execution.md` | PlanCompiler algorithm, BT node types, MissionExecutor, replanning |
| `05-observability.md` | 6-layer observability stack (TreeObserver -> FoxgloveBridge -> PlanAuditLog -> NeuroAuditLog) |
| `06-ros2.md` | ROS2 lifecycle nodes, deployment modes, build/run instructions |
| `07-extensions.md` | Extensions roadmap (future/cross-cutting: temporal + neuro-symbolic) |
| `08-neuro-symbolic.md` | **Neuro-symbolic infrastructure**: PVF envelope, backend layer, Advisor, verifiers, codecs, config, seams, test patterns |

## Build

This project uses CMake, with MSVC/Visual Studio 2022 the primary Windows toolchain. No Conan or
manual installs are needed.

AME's dependencies (BehaviorTree.CPP, LAPKT, googletest, nlohmann/json, Dear ImGui, SDL2 and the
rest) are **checked into the repository** under
[`subprojects/AME/external`](subprojects/AME/external/README.md), so a build needs no network
access. Anything missing from that directory is downloaded instead, unless
`AME_REQUIRE_VENDORED_DEPENDENCIES=ON` is set, which turns a missing copy into an error — set it
for air-gapped builds. PYRAMID's optional transport dependencies (FlatBuffers, gRPC, Protobuf) are
still fetched at configure time.

`CMakePresets.json` lives at the repository root and defines:

| Preset type | Names | Purpose |
|-------------|-------|---------|
| Configure | `default` | Standard workspace build in `build/` using project default options |
| Configure | `all-on` | Optional dependencies enabled in `build-all-enabled/`: Foxglove, FlatBuffers, gRPC, Protobuf, ROS2 |
| Configure | `all-off` | Optional dependencies disabled in `build-all-off/`: Foxglove, FlatBuffers, gRPC, Protobuf, ROS2 |
| Configure | `authoring` | Graphical authoring tool enabled (`AME_BUILD_AUTHORING=ON`) in `build-authoring/` |
| Build | `release`, `debug` | Release/Debug builds using the `default` configure preset |
| Build | `all-on-release` | Release build using `all-on` |
| Build | `all-off-release`, `all-off-debug` | Release/Debug builds using `all-off` |
| Build | `authoring-release`, `authoring-debug` | Release/Debug builds using `authoring`. Builds `ame_authoring_tool` and `ame_mission_cli` |
| Test | `all-on-release`, `all-off-release`, `all-off-debug` | CTest runs with `outputOnFailure` enabled |
| Test | `authoring-release`, `authoring-debug` | CTest runs covering the authoring tool's own test suites |

The `authoring` preset has its own build directory on purpose. `AME_BUILD_AUTHORING` is a cached
CMake option, so if the authoring tool were configured into the shared `build/` tree it would stay
enabled there even after switching back to the `default` preset.

### Building AME on its own

`subprojects/AME` is also a complete CMake project with its own `CMakePresets.json`, so it can be
configured directly — from the command line, or by opening that folder in VS Code. It needs neither
PCL nor PYRAMID on disk:

```bat
cd subprojects\AME
cmake --preset default
cmake --build --preset release --parallel %NUMBER_OF_PROCESSORS%
ctest --preset release
```

| Preset | Purpose |
|--------|---------|
| `default` | Core engine, authoring tool, `ame_mission_cli` and the contingency verifier, in `build/` |
| `no-authoring` | The same without the graphical tool, in `build-engine/`. Leaves SDL2, OpenGL and Dear ImGui out, which is what a headless build agent wants |
| `networked` | As `default`, but downloads a dependency missing from `external/` instead of failing. For adding a dependency, never for an air-gapped build |

What the standalone build leaves out is the PCL component layer (`ame_pcl_components`,
`ame_pcl_main`) and the PYRAMID autonomy bridge, which are the only parts of AME that need those
subprojects, together with the tests covering them. The workspace build is unchanged and still
builds everything. `AME_BUILD_PCL_COMPONENTS` is worked out from whether the `pcl_core` target
exists rather than being an option, because it is not a preference.

### Contract tree

PYRAMID generates AME's C++ bindings from a single contract tree named by `PYRAMID_PROTO_DIR`.
AME owns the contracts it needs in `subprojects/AME/contracts/proto`, so AME can be built with
PYRAMID's proof layer switched off (`PYRAMID_BUILD_PROOFS=OFF`), which drops roughly 40% of the
build's targets. With the proof layer on, its contracts are used instead, and they are a superset.
The `AmeContractTree` tests check that the files common to both copies have not drifted apart. See
[`subprojects/AME/contracts/README.md`](subprojects/AME/contracts/README.md).

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

1. Open `D:\Dev\repo\unmanned` as the workspace folder.
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

The full CTest suite should pass. It spans PCL, PYRAMID, Tactical Objects, AME core (WorldModel, TypeSystem, ActionRegistry, PlanCompiler, Planner, PddlParser), integration (BT nodes), end-to-end pipeline, observability (Layers 1-5), generated bindings, and transport adapters -- 1452 registered tests in the workspace `authoring` build at last count. AME's standalone build runs the subset that does not need PCL or PYRAMID: 451 with the graphical tool, 260 without it. Use `ctest --test-dir build -N -C Release` to list the exact set in your build tree.

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
