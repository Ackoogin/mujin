# AME Dev Environment -- ROS2 Quickstart

## Do I need a new application?

No. The dev environment (`subprojects/AME/tools/devenv`) connects directly to the existing `ame_ros2` nodes. No extra bridge app is required.

This guide covers the existing ROS2/Foxglove devenv path. The new PYRAMID
autonomy backend contract is being tracked separately as a planned devenv mode
so developers can create separate `PlanningRequirement` and
`ExecutionRequirement` entities, push `StateUpdate` feedback, and inspect
`Plan` resources plus `ExecutionRun` and `RequirementPlacement` products
locally. See `doc/todo/AME/ALIGNMENT.md`
for the current contract progress and devenv support plan.

It uses two parallel connections:

| Connection | Purpose | When active |
|------------|---------|-------------|
| **rclpy** (direct ROS2) | World state subscription, `set_fact`/`get_fact`/`query_state`/`load_domain` services, `~/plan` action | Requires ROS2 + sourced workspace |
| **Foxglove WebSocket** `ws://localhost:8765` | BT event streaming, WM audit streaming | Requires `foxglove_bridge` running alongside |

---

## Prerequisites

- ROS2 Jazzy installed via pixi at `D:\Dev\ros2-windows`
- `ame_ros2` built (run `subprojects\AME\scripts\build_ros2.bat` from `D:\Dev\repo\unmanned` -- see `subprojects/AME/doc/architecture/06-ros2.md`)
- `install\setup.bat` sourced (produced by the colcon build at `D:\Dev\repo\unmanned\install\setup.bat`)
- Dev environment venv set up (`subprojects\AME\tools\devenv\setup_venv.bat`)

---

## Step 1 -- Make rclpy available to the devenv

The devenv venv needs rclpy on `PYTHONPATH`. Set it in the shell before launching:

```bat
call D:\Dev\ros2-windows\setup.bat
call D:\Dev\repo\unmanned\install\setup.bat
set PYTHONPATH=%PYTHONPATH%;D:\Dev\ros2-windows\.pixi\envs\default\Lib\site-packages
```

Then launch the devenv in the same shell (Step 4).

---

## Step 2 -- Start the ame_ros2 stack

The quickest path is the helper script, which handles all PATH and environment setup:

```bat
D:\Dev\repo\unmanned\subprojects\AME\scripts\run_ros2.bat
```

Or with custom PDDL files:

```bat
subprojects\AME\scripts\run_ros2.bat D:\Dev\repo\unmanned\subprojects\AME\domains\uav_search\domain.pddl ^
                                      D:\Dev\repo\unmanned\subprojects\AME\domains\uav_search\problem.pddl
```

**Alternatively, launch directly:**

```bat
call D:\Dev\ros2-windows\setup.bat
call D:\Dev\repo\unmanned\install\setup.bat

REM In-process (single node, recommended for development):
ros2 run ame_ros2 ame_combined ^
  --ros-args ^
  -p domain.pddl_file:=D:/Dev/repo/unmanned/subprojects/AME/domains/uav_search/domain.pddl ^
  -p domain.problem_file:=D:/Dev/repo/unmanned/subprojects/AME/domains/uav_search/problem.pddl

REM Or via launch file (equivalent):
ros2 launch ame_ros2 ame_inprocess.launch.py ^
  pddl_file:=D:/Dev/repo/unmanned/subprojects/AME/domains/uav_search/domain.pddl ^
  problem_file:=D:/Dev/repo/unmanned/subprojects/AME/domains/uav_search/problem.pddl
```

The `AmeLifecycleManager` inside the combined node automatically configures and activates
`world_model_node` -> `planner_node` -> `executor_node` in order.

For the distributed mode (separate processes):

```bat
ros2 launch ame_ros2 ame_distributed.launch.py ^
  pddl_file:=D:/Dev/repo/unmanned/subprojects/AME/domains/uav_search/domain.pddl ^
  problem_file:=D:/Dev/repo/unmanned/subprojects/AME/domains/uav_search/problem.pddl
```

---

## Step 3 -- (Optional) Start the Foxglove bridge

The Observability and Execution tabs in the devenv receive live BT events and WM audit entries
via Foxglove WebSocket. Install `foxglove_bridge` then run:

```bat
call D:\Dev\ros2-windows\setup.bat
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
```

This bridges the `/executor/bt_events` and `/wm_audit` ROS2 topics to the WebSocket
the devenv expects at `ws://localhost:8765`.

---

## Step 4 -- Launch the dev environment

In the same shell where you set `PYTHONPATH` in Step 1:

```bat
cd D:\Dev\repo\unmanned
subprojects\AME\tools\devenv\start_devenv.bat
```

The status bar at the bottom will show:

- `ROS2: Connected` -- rclpy found the `world_model_node` services
- `Foxglove: Connected` -- WebSocket bridge is live (BT/WM streams active)
- `WM: v<n>` -- world model version, updating on every fact change

> Run with `--no-ros2` for offline/replay mode (no ROS2 required).

---

## What each tab does with ROS2

| Tab | ROS2 interaction |
|-----|-----------------|
| **World Model** | Displays live `/world_state` facts; Set Fact button calls `world_model_node/set_fact` service |
| **Planning** | Sends goal to `/planner_node/plan` action; streams feedback (nodes expanded, elapsed) |
| **Execution** | Displays BT node status transitions from Foxglove `/bt_events` stream |
| **Observability** | Time-series plots of BT events + WM audit entries from Foxglove streams |
| **PDDL Editor** | Offline only -- edit and validate domain/problem files locally |

---

## Topics and services reference

| Name | Type | Direction | Description |
|------|------|-----------|-------------|
| `/world_state` | `ame_ros2/msg/WorldState` | subscribe | Full fact snapshot, published on change |
| `/world_model_node/set_fact` | `ame_ros2/srv/SetFact` | call | Write a predicate value |
| `/world_model_node/get_fact` | `ame_ros2/srv/GetFact` | call | Read a single predicate |
| `/world_model_node/query_state` | `ame_ros2/srv/QueryState` | call | Bulk read (empty keys = all true facts) |
| `/planner_node/plan` | `ame_ros2/action/Plan` | action goal | Run LAPKT planner; returns BT XML on success |
| `/planner_node/load_domain` | `ame_ros2/srv/LoadDomain` | call | Load domain from PDDL strings (devenv/testing) |
| `/world_model_node/load_domain` | `ame_ros2/srv/LoadDomain` | call | Reload union domain from PDDL strings (devenv/testing) |
| `/executor/bt_events` | `std_msgs/String` (JSON) | Foxglove stream | Per-tick BT node status transitions |
| `/detections` | `ame_ros2/msg/Detection` | publish | Inject perception updates into world model |

---

## Quick smoke test (no devenv)

```bat
REM Confirm world model is live
ros2 service call /world_model_node/query_state ame_ros2/srv/QueryState "{keys: []}"

REM Inject a fact
ros2 service call /world_model_node/set_fact ame_ros2/srv/SetFact ^
  "{key: 'uav_at_base', value: true, source: 'manual'}"

REM Trigger planning
ros2 action send_goal /planner_node/plan ame_ros2/action/Plan ^
  "{goal_fluents: ['(searched sector_a)'], replan: false}"

REM Load domain at runtime (devenv/testing -- bypasses file paths)
ros2 service call /world_model_node/load_domain ame_ros2/srv/LoadDomain ^
  "{domain_id: 'test', domain_pddl: '...', problem_pddl: '...'}"
ros2 service call /planner_node/load_domain ame_ros2/srv/LoadDomain ^
  "{domain_id: 'test', domain_pddl: '...', problem_pddl: '...'}"
```
