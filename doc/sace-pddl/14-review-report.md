# SACE PDDL Approach Review Report

Date: 2026-03-07

## Scope and review method

This review covers the current SACE/PDDL approach implemented and documented in this repository, with emphasis on:

- `doc/sace-pddl/`
- `domains/uav_search_sace/`
- `src/plan_compiler.cpp`
- `src/planner.cpp`
- `ros2/src/planner_node.cpp`
- `ros2/src/executor_node.cpp`
- `ros2/src/combined_main.cpp`

The review was primarily static. I did not run the C++/ROS2 test suite in this environment because the checked-in preset is Windows/MSVC-specific (`CMakePresets.json:5-9`).

## Findings

### 1. High: Unmapped PDDL actions can still mutate the world model during BT execution

**Evidence**

- `src/plan_compiler.cpp:65-80` only emits the concrete BT action node when `registry.hasAction(name)` is true, but it always emits `SetWorldPredicate` nodes for the action effects.
- `doc/autonomy_assurance_plan.md:209` defines `SR-08`: `ActionRegistry.resolve()` shall fail loudly if no mapping exists.
- `tests/test_action_registry.cpp:53-56` verifies that `ActionRegistry::resolve()` throws for unknown actions, but there is no matching `PlanCompiler` test for this path.

**Why this matters**

This is the most important implementation defect in the current approach. If an action is present in the PDDL plan but missing from the `ActionRegistry`, the compiled BT can still:

1. check the action preconditions, then
2. apply the PDDL add/delete effects directly to the `WorldModel`,

without ever executing a real action node.

That creates false evidence of successful execution. In a SACE context, this is especially dangerous because Stage 5 and Stage 7 arguments rely on the system actually performing degraded-mode or safe-state actions rather than only simulating their effects in the internal state.

**SACE-specific impact**

- `domains/uav_search_sace/domain_baseline.pddl:77-80` defines `loiter-in-place`.
- `domains/uav_search_sace/domain_degraded.pddl:79-109` defines `emergency-return-gps`, `emergency-return-inertial`, and `loiter-in-place`.
- The demo/runtime wiring only registers `move`, `search`, and `classify` in `src/main.cpp:162-165` and `ros2/src/combined_main.cpp:93-95`.

So the SACE-specific actions most relevant to degraded recovery are currently the ones most likely to hit this fail-open behavior.

**Recommendation**

Make `PlanCompiler` fail hard when any planned action is absent from the registry, and add a unit test that verifies compilation rejects a plan containing an unregistered action.

### 2. High: The ROS2 runtime does not contain a production path from plan result to executor start

**Evidence**

- `ros2/src/planner_node.cpp:162-196` compiles `bt_xml` and returns it in the action result.
- `ros2/src/executor_node.cpp:85-133` only starts execution when `loadAndExecute(bt_xml)` is called.
- `ros2/test/test_full_pipeline.cpp:150-159` performs that call manually in the test harness.
- `ros2/src/combined_main.cpp:13-18` states that after sending a `/mujin/plan` goal, the executor will tick the compiled BT automatically, but no such handoff exists in the production code shown above.

**Why this matters**

The planning and execution pieces exist, but the live orchestration step between them is missing. As written, a user can obtain a plan result containing `bt_xml`, but sending the ROS2 action goal alone does not cause mission execution to start.

This is a material gap for an assurance story that depends on traceability from:

`PDDL problem -> plan -> compiled BT -> executed behavior`

Without a production orchestrator, the system cannot demonstrate that the reviewed SACE planning artefacts are the ones actually executed in deployment.

**Recommendation**

Add a runtime orchestrator that consumes `Plan.action` results and calls `ExecutorNode::loadAndExecute()`, or move that orchestration into a `MissionExecutor`-style component and update the docs to match the implemented behavior.

### 3. Medium: Replanning and safe-state fallback are treated as core safety controls in the docs, but are not implemented in code

**Evidence**

- `doc/autonomy_assurance_plan.md:169-173` lists replan limit and safe-state fallback as Safe Operating Concept controls.
- `doc/autonomy_assurance_plan.md:207-210` defines `SR-06` and `SR-09` around replanning limits and always-available safe-state behavior.
- `doc/concept.md:31` describes a `MissionExecutor` with replan-on-failure.
- `ros2/action/Plan.action:3` exposes a `replan` flag.
- `ros2/src/planner_node.cpp:79-197` never reads `goal->replan`.
- Repository search found no implementation of `MissionExecutor` or `ReplanOnFailure` in `src/` or `ros2/`.

**Why this matters**

Several of the SACE claims depend on a supervisory component that can:

- detect execution failure,
- decide whether replanning is allowed,
- bound repeated replans, and
- enter or load a safe-state behavior when recovery fails.

Today those behaviors are design intent, not implemented controls. That weakens the current assurance argument for hazards H6 and H9 because the mitigations are not yet in the runtime path.

**Recommendation**

Either implement the supervisory executor path now, or explicitly downgrade these items in the assurance documentation from "implemented control" to "planned control".

### 4. Medium: The code does not implement the bounded planning-time control claimed by the assurance docs

**Evidence**

- `doc/autonomy_assurance_plan.md:168` says bounded planning time is implemented via a `max_iterations` parameter in the LAPKT solver.
- `doc/autonomy_assurance_plan.md:203` defines `SR-02`: planner shall terminate within N ms or return `NO_PLAN`.
- `src/planner.cpp:16-65` exposes no timeout, iteration bound, or node-expansion budget.
- `include/mujin/planner.h:21-25` also exposes no configuration surface for such a bound.

**Why this matters**

This is an assurance/code mismatch rather than a pure documentation nit. In the current implementation, BRFS runs until it finds a plan or exhausts search. For larger SACE scenario sets, especially degraded or fault-injected variants, there is no mechanism that enforces a mission-time planning budget.

That leaves Stage 3 and Stage 4 arguments about bounded planning time unsupported by the implementation.

**Recommendation**

Add an explicit planning budget API and plumb it through the ROS2 node parameters, or revise the assurance text to state that bounded solve time is a requirement not yet implemented.

### 5. Medium: The SACE domain family is not exercised by automated tests

**Evidence**

- `tests/test_pddl_parser.cpp:183-193` only parses `domains/uav_search/domain.pddl` and `domains/uav_search/problem.pddl`.
- Repository search found no references to `uav_search_sace` under `tests/` or `ros2/test/`.
- The core Stage 5 and Stage 7 examples in `doc/sace-pddl/12-e2e-example.md:144-156` and `doc/sace-pddl/12-e2e-example.md:183-211` therefore are not backed by CI-visible tests.

**Why this matters**

The repository contains a strong set of SACE artefacts, but the actual SACE example domains are currently documentary examples rather than regression-tested fixtures. That increases the risk that:

- parser behavior changes break the SACE domains,
- planner behavior no longer matches the documented "PLAN FOUND" / "NO PLAN" expectations,
- degraded-mode recovery examples drift from the runtime implementation.

**Recommendation**

Add targeted tests for:

- baseline nominal scenario: `PLAN FOUND`
- weather fault: `NO_PLAN`
- comms lost: expected result made explicit and checked
- baseline OOC recovery: `NO_PLAN`
- degraded OOC recovery: `PLAN FOUND`

At least one ROS2 integration test should also verify that any SACE-specific action appearing in the plan is present in the `ActionRegistry`.

## Strengths

The approach already has several strong foundations:

- The separation between runtime execution and offline assurance tooling is clear and well-motivated in `doc/sace-pddl/13-tooling-analysis.md:170-219`.
- The SACE document set is concrete, not abstract; it ties each stage to specific PDDL artefacts in `domains/uav_search_sace/`.
- The runtime pipeline itself is straightforward and auditable: `PDDL -> WorldModel -> BRFS planner -> PlanCompiler -> BT XML`.

## Overall assessment

The repository presents a credible **SACE-oriented PDDL assurance concept**, but it is not yet a complete **SACE-capable execution stack**.

The strongest parts today are:

- the staged documentation,
- the concrete SACE PDDL artefacts,
- the simple STRIPS execution pipeline.

The weakest parts are the places where the assurance story assumes runtime controls that are either incomplete or absent:

- missing-action handling,
- plan-to-executor orchestration,
- replanning/safe-state supervision,
- bounded planning-time enforcement,
- SACE-specific regression coverage.

## Recommended next steps

1. Fix the `PlanCompiler` fail-open behavior for unregistered actions.
2. Add a real production handoff from planner result to executor.
3. Decide whether `MissionExecutor` is being implemented now or deferred, and align docs accordingly.
4. Add planner budget controls if bounded planning time is a required safety claim.
5. Turn the `uav_search_sace` scenarios into automated tests so the assurance examples become maintained evidence.
