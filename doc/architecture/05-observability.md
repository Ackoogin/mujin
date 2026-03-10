# Observability Stack

A 5-layer audit and monitoring system that replaces the paid Groot2 tool with open, composable, sink-based logging.

## Layer Summary

| Layer | Component | What It Captures | Output |
|-------|-----------|-----------------|--------|
| 1 | TreeObserver + SqliteLogger + MinitraceLogger | Per-node tick counts, success/failure rates, timing | In-memory stats, SQLite DB, Chrome Tracing JSON |
| 2 | MujinBTLogger | Structured JSON events for every BT node state transition | JSONL file + callback sinks |
| 3 | WmAuditLog | Every world model fact change with source tag and timestamp | JSONL file + callback sinks |
| 4 | FoxgloveBridge | WebSocket server for live Foxglove Studio visualization | `ws://localhost:8765` |
| 5 | PlanAuditLog | Full planning episodes: initial state, goals, solver, timing, plan, compiled BT XML | JSONL file |

## Layer 1: Built-in BT.CPP Loggers

Zero custom code — uses BT.CPP's free logging infrastructure:

- **SqliteLogger** — persistent, queryable record of every node state transition with timestamps. Enabled via `BTCPP_SQLITE_LOGGING=ON`.
- **MinitraceLogger** — Chrome Tracing format (`.json`), viewable in `chrome://tracing` or Perfetto UI for flame chart analysis.
- **TreeObserver** — per-node statistics (tick counts, success/failure rates, duration). Queryable in-memory.

## Layer 2: Structured Event Stream (MujinBTLogger)

**Files:** `include/mujin/bt_logger.h`, `src/bt_logger.cpp`

Custom `StatusChangeLogger` subclass that emits structured JSON events:

```json
{
  "ts_us": 1702345678000,
  "node": "search_sector_uav1_sectorA",
  "type": "ReactiveSequence",
  "prev": "IDLE",
  "status": "RUNNING",
  "tree_id": "MissionPlan",
  "wm_version": 42
}
```

Sinks (configurable, multiple simultaneously):
- **File** — JSONL (`mujin_bt_events.jsonl`)
- **Callback** — for live consumers (FoxgloveBridge, ROS2 topic publisher)

Captures world model version at each transition, linking BT execution to world model state history.

## Layer 3: World Model Audit Log (WmAuditLog)

**Files:** `include/mujin/wm_audit_log.h`, `src/wm_audit_log.cpp`

Hooked into `WorldModel::setAuditCallback()`. Emits a structured entry on every state change:

```json
{
  "wm_version": 43,
  "ts_us": 1702345679000,
  "fact": "at(uav1,sectorA)",
  "value": true,
  "source": "SetWorldPredicate:search_sector_uav1_sectorA"
}
```

Sources are one of:
- A BT node name (from `SetWorldPredicate`)
- `"perception"` or `"perception:<subtag>"` (external update)
- `"planner_init"` (initial state sync)

Combined with Layer 2, this gives full causal traceability: which BT node changed which fact at which world model version.

## Layer 4: Foxglove Bridge

**Files:** `include/mujin/foxglove_bridge.h`, `src/foxglove_bridge.cpp`
**Library:** `mujin_foxglove` (separate static library, optional via `MUJIN_FOXGLOVE` CMake option)
**Dependencies:** websocketpp 0.8.2, standalone Asio 1.28

Implements the [Foxglove WebSocket protocol](https://github.com/foxglove/ws-protocol) directly, without requiring ROS2:

- WebSocket server on configurable port (default 8765)
- Two channels: `/bt_events` (schema `mujin.BTEvent`) and `/wm_audit` (schema `mujin.WMFactChange`)
- Streams live BT state transitions and WM fact changes as binary `MessageData` frames
- Integrates as callback sinks for MujinBTLogger and WmAuditLog

Connect Foxglove Studio to `ws://localhost:8765`. See the [quickstart guide](../quickstart.md) for panel setup.

### Programmatic Usage

```cpp
#include "mujin/foxglove_bridge.h"
#include "mujin/bt_logger.h"

mujin::FoxgloveBridge bridge({8765, "my_app"});
bridge.start();

// Wire BT events to Foxglove
mujin::MujinBTLogger bt_logger(tree, "MyPlan", &world_model);
bt_logger.addCallbackSink(bridge.btEventSink());

// Wire WM events to Foxglove
auto wm_sink = bridge.wmEventSink();
world_model.setAuditCallback([&wm_sink](uint64_t ver, uint64_t ts,
                                         const std::string& fact, bool val,
                                         const std::string& src) {
    std::string json = /* ... */;
    wm_sink(json);
});

// ... tick tree ...
bridge.stop();
```

## Layer 5: Plan Audit Trail (PlanAuditLog)

**Files:** `include/mujin/plan_audit_log.h`, `src/plan_audit_log.cpp`

For each planning episode, logs a self-contained JSON object:
- World model snapshot (init state) that triggered planning
- Goal fluents
- LAPKT solver used and solve time (`PlanResult::solve_time_ms`)
- Ordered action list
- Compiled BT XML

Output: `mujin_plan_audit.jsonl`. Forms a complete mission audit trail.

## What Replaces Groot2

| Groot2 Feature | Replacement | Layer |
|----------------|------------|-------|
| Live tree visualization | Foxglove Studio via FoxgloveBridge | 4 |
| Real-time node status | MujinBTLogger → Foxglove `/bt_events` | 2, 4 |
| Blackboard / state inspector | WmAuditLog → Foxglove `/wm_audit` | 3, 4 |
| Log replay | SQLite queries + Perfetto/Chrome tracing + JSONL | 1, 5 |
| Node statistics | TreeObserver queryable in-memory | 1 |
| Plan provenance | PlanAuditLog JSONL | 5 |
| Tree editor | Not needed — trees are compiler-generated | — |
