# Observability Stack

A 6-layer audit and monitoring system (Layer 6 requires `AME_NEURO=ON`) that replaces the paid Groot2 tool with open, composable, sink-based logging.

## Layer Summary

| Layer | Component | What It Captures | Output |
|-------|-----------|-----------------|--------|
| 1 | TreeObserver + SqliteLogger + MinitraceLogger | Per-node tick counts, success/failure rates, timing | In-memory stats, SQLite DB, Chrome Tracing JSON |
| 2 | AmeBTLogger | Structured JSON events for every BT node state transition | JSONL file + callback sinks |
| 3 | WmAuditLog | Every world model fact change with source tag and timestamp | JSONL file + callback sinks |
| 4 | FoxgloveBridge | WebSocket server for live Foxglove Studio visualization | `ws://localhost:8765` |
| 5 | PlanAuditLog | Full planning episodes: initial state, goals, solver, timing, plan, BT XML, neural provenance | JSONL file |
| 6 | NeuroAuditLog (opt.) | One record per neural `Advisor` call: outcome, latency, retries, verifier verdict, evidence | JSONL file |

## Layer 1: Built-in BT.CPP Loggers

Zero custom code -- uses BT.CPP's free logging infrastructure:

- **SqliteLogger** -- persistent, queryable record of every node state transition with timestamps. Enabled via `BTCPP_SQLITE_LOGGING=ON`.
- **MinitraceLogger** -- Chrome Tracing format (`.json`), viewable in `chrome://tracing` or Perfetto UI for flame chart analysis.
- **TreeObserver** -- per-node statistics (tick counts, success/failure rates, duration). Queryable in-memory.

## Layer 2: Structured Event Stream (AmeBTLogger)

**Files:** `include/ame/bt_logger.h`, `src/bt_logger.cpp`

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
- **File** -- JSONL (`ame_bt_events.jsonl`)
- **Callback** -- for live consumers (FoxgloveBridge, ROS2 topic publisher)

Captures world model version at each transition, linking BT execution to world model state history.

## Layer 3: World Model Audit Log (WmAuditLog)

**Files:** `include/ame/wm_audit_log.h`, `src/wm_audit_log.cpp`

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

**Files:** `include/ame/foxglove_bridge.h`, `src/foxglove_bridge.cpp`
**Library:** `ame_foxglove` (separate static library, optional via `AME_FOXGLOVE` CMake option)
**Dependencies:** websocketpp 0.8.2, standalone Asio 1.28

Implements the [Foxglove WebSocket protocol](https://github.com/foxglove/ws-protocol) directly, without requiring ROS2:

- WebSocket server on configurable port (default 8765)
- Two channels: `/bt_events` (schema `ame.BTEvent`) and `/wm_audit` (schema `ame.WMFactChange`)
- Streams live BT state transitions and WM fact changes as binary `MessageData` frames
- Integrates as callback sinks for AmeBTLogger and WmAuditLog

Connect Foxglove Studio to `ws://localhost:8765`. See the [quickstart guide](../guides/quickstart.md) for panel setup.

### Programmatic Usage

```cpp
#include "ame/foxglove_bridge.h"
#include "ame/bt_logger.h"

ame::FoxgloveBridge bridge({8765, "my_app"});
bridge.start();

// Wire BT events to Foxglove
ame::AmeBTLogger bt_logger(tree, "MyPlan", &world_model);
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

**Files:** `include/ame/plan_audit_log.h`, `src/plan_audit_log.cpp`

For each planning episode, logs a self-contained JSON object:
- `episode_id` -- unique auto-assigned ID for this episode
- `parent_episode_id` -- causal link to parent phase (0 for top-level episodes)
- `phase_name` -- human-readable label (set by `ExecutePhaseAction`)
- World model snapshot (init state) that triggered planning
- Goal fluents
- LAPKT solver used and solve time (`PlanResult::solve_time_ms`)
- Ordered action list
- Compiled BT XML

Hierarchical missions produce a tree of episodes linked by `parent_episode_id`, enabling full traceability from top-level mission phases down to individual sub-plans:

```json
{"episode_id":1,"parent_episode_id":0,"phase_name":"recon_phase","solver":"BRFS",...}
{"episode_id":2,"parent_episode_id":1,"phase_name":"search_sub","solver":"BRFS",...}
```

Output: `ame_plan_audit.jsonl`. Forms a complete mission audit trail.

When `AME_NEURO=ON`, four provenance fields are appended to each episode record:

| Field | Default | Meaning |
|-------|---------|---------|
| `heuristic_source` | `"symbolic"` | `"neural_hook"` when HeuristicHook biased action ordering |
| `goal_source` | `"symbolic"` | Set by goal-interpretation integrations |
| `repair_source` | `"symbolic"` | Set by repair integrations |
| `neuro_record_ids` | `[]` | Layer 6 `NeuroAuditRecord.id` values for cross-referencing |

## Layer 6: Neural Advisor Audit Trail (NeuroAuditLog) — `AME_NEURO=ON`

**Files:** `include/ame/neuro/neuro_audit_log.h`, `src/lib/neuro/neuro_audit_log.cpp`

One `NeuroAuditRecord` is emitted per `Advisor::advise()` call, regardless of outcome:

```json
{
  "id": 1,
  "ts_us": 1702345679000000,
  "integration_id": "heuristic_hook",
  "backend_id": "onnx_heuristic",
  "outcome": "Accepted",
  "latency_ms": 12.4,
  "retries": 0,
  "verify_accepted": true,
  "reason": "plan_verified",
  "evidence": ["believed:42"],
  "request_digest": "{\"fluents\":[\"(at uav1 base)\"]",
  "proposal_digest": "[{\"ground_action_id\":3,\"score\":0.91}]"
}
```

Output: `neuro_audit.jsonl`. IDs are monotonically assigned and globally unique per log.

## AuditIndex: Cross-Stream Query (`AME_NEURO=ON`)

**Files:** `include/ame/neuro/audit_index.h`, `src/lib/neuro/audit_index.cpp`

Loads all JSONL streams and builds time + entity indices for correlated analysis:

```cpp
AuditIndex idx;
idx.load("bt_events.jsonl",   "bt");    // Layer 2
idx.load("wm_audit.jsonl",    "wm");    // Layer 3
idx.load("plan_audit.jsonl",  "plan");  // Layer 5
idx.load("neuro_audit.jsonl", "neuro"); // Layer 6

// What happened in a 100ms window?
auto records = idx.window(t0_us, t0_us + 100'000);

// Context around a specific entity (episode, fact, node)
auto context = idx.around("episode_42", /*k=*/10, /*max_bytes=*/65536);

// Export plan records as training samples
auto samples = idx.training_export();   // init_facts, goal_fluents, plan_actions

// Human-readable provenance
auto ids = idx.cite("uav1_sectorA");
```

This gives a single unified timeline across BT ticks, WM fact changes, plan episodes,
and neural advisor calls — enabling full causal traceability of any mission event.

## What Replaces Groot2

| Groot2 Feature | Replacement | Layer |
|----------------|------------|-------|
| Live tree visualization | Foxglove Studio via FoxgloveBridge | 4 |
| Real-time node status | AmeBTLogger -> Foxglove `/bt_events` | 2, 4 |
| Blackboard / state inspector | WmAuditLog -> Foxglove `/wm_audit` | 3, 4 |
| Log replay | SQLite queries + Perfetto/Chrome tracing + JSONL | 1, 5 |
| Node statistics | TreeObserver queryable in-memory | 1 |
| Plan provenance | PlanAuditLog JSONL | 5 |
| Tree editor | Not needed -- trees are compiler-generated | -- |

