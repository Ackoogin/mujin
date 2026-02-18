# Extensions Plan

Post-vertical-slice features, ordered by priority and dependency.

---

## 1. Observability (Groot Monitoring Replacement)

Groot2 is BehaviorTree.CPP's paid monitoring/visualization tool. We replace it with a layered observability stack built on BT.CPP's free logging infrastructure.

### What Groot2 provides (paid)

- Live tree visualization with real-time node status
- Blackboard inspector
- Log replay
- Node performance statistics
- Tree editor GUI

### What BT.CPP provides for free

BT.CPP v4 ships several logger backends that are independent of Groot:

| Class | What it does | Output |
|-------|-------------|--------|
| `StatusChangeLogger` | Abstract base. Fires on every node state transition. | Subclass it |
| `FileLogger2` | Binary `.btlog` of all state transitions | File (Groot2 needed to replay) |
| `SqliteLogger` | Node transitions + timestamps into SQLite | `.db` file |
| `MinitraceLogger` | Chrome Tracing format | `.json` viewable in `chrome://tracing` or Perfetto |
| `TreeObserver` | Per-node statistics: tick count, success/failure counts, duration | In-memory, queryable |

### Implementation plan

#### Layer 1: Enable built-in loggers (zero custom code)

Turn on what already exists in BT.CPP:

1. **Enable `BTCPP_SQLITE_LOGGING`** in CMakeLists.txt. Attach a `SqliteLogger` to every tree. This gives a persistent, queryable record of every node state transition with timestamps.

2. **Attach `MinitraceLogger`** for any execution session where timing analysis is needed. Open the resulting `.json` in `chrome://tracing` or Perfetto UI — free, zero-install flame chart of the entire BT execution.

3. **Use `TreeObserver`** to collect per-node statistics (tick counts, success/failure rates, average duration). Expose via a query API.

These three give us logging, replay (via SQL queries), and performance profiling with no custom code.

#### Layer 2: Structured event stream (custom `StatusChangeLogger`)

Write a `MujinBTLogger` subclass of `StatusChangeLogger` that emits structured JSON events:

```cpp
class MujinBTLogger : public BT::StatusChangeLogger {
public:
    // Callback fired on every node status change
    void callback(BT::Duration timestamp,
                  const BT::TreeNode& node,
                  BT::NodeStatus prev,
                  BT::NodeStatus status) override;

    // Flush to configured sink(s)
    void flush() override;
};
```

Each event:

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
- **File** — JSONL for post-hoc analysis
- **Unix socket / WebSocket** — for live consumers (dashboard, ROS2 bridge)
- **ROS2 topic** — when ROS2 integration lands (Layer 4)

This logger also captures world model version at each transition, linking BT execution to world model state history.

#### Layer 3: World model audit log

The `WorldModel::setFact()` path already increments a version counter. Extend it to emit a structured log entry on every state change:

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
- `"perception"` (external update via `setFact()` API)
- `"planner_init"` (initial state sync)

Combined with the BT event stream from Layer 2, this gives full causal traceability: which BT node changed which fact at which world model version.

Storage: same SQLite database as Layer 1 (separate table), or JSONL file.

#### Layer 4: Live dashboard

A lightweight web UI for real-time monitoring. Two options depending on deployment context:

**Option A: Standalone (no ROS2)**

Embed a minimal WebSocket server (e.g., `uWebSockets` or `libwebsockets`) in the executor process. Serve a single-page HTML/JS dashboard that:

- Renders the BT as a tree diagram (D3.js or similar) with live node status coloring
- Shows a world model state table with live updates
- Displays per-node statistics from `TreeObserver`
- Provides a timeline/waterfall view of execution history

The dashboard is a static HTML file; no build toolchain needed. The WebSocket carries the same JSON events from Layer 2.

**Option B: ROS2 / Foxglove (with ROS2 integration)**

Publish BT state transitions and world model changes as ROS2 topics. Use Foxglove Studio (free, open-source) as the visualization frontend:

- Custom Foxglove panel for BT tree rendering
- Built-in timeline, log, and plot panels for world model state
- Record/replay via `rosbag2`

This is the preferred path once ROS2 integration lands, since Foxglove already handles the hard parts (WebSocket transport, time sync, recording).

#### Layer 5: Plan audit trail

For each planning episode, log:
- The world model snapshot (init state) that triggered planning
- The goal fluents
- The LAPKT solver used and solve time
- The plan (ordered action list)
- The causal graph (adjacency list)
- The compiled BT XML

This forms a mission audit trail: for any point in execution, you can reconstruct why the system chose the actions it did.

Storage: SQLite (separate table) or structured log files alongside the BT logs.

### Summary: what replaces what

| Groot2 feature | Our replacement | Layer |
|----------------|-----------------|-------|
| Live tree visualization | Web dashboard (D3.js) or Foxglove | 4 |
| Real-time node status | `MujinBTLogger` → WebSocket stream | 2 |
| Blackboard / state inspector | World model audit log + dashboard | 3, 4 |
| Log replay | SQLite queries + Perfetto/Chrome tracing | 1 |
| Node statistics | `TreeObserver` exposed via dashboard | 1, 4 |
| Tree editor | Not needed — trees are compiler-generated | — |

### Build order

Layers 1–3 are pre-requisites for Layer 4 and have no external dependencies beyond what we already vendor. Layer 5 is independent and can be built in parallel.

| Layer | Dependency | Effort |
|-------|-----------|--------|
| 1: Built-in loggers | None (BT.CPP already available) | CMake flag + wiring |
| 2: MujinBTLogger | Layer 1 (for SQLite sink) | ~200 LOC |
| 3: WM audit log | None | ~100 LOC in WorldModel |
| 4: Dashboard | Layers 2 + 3 | HTML/JS page + WebSocket server |
| 5: Plan audit trail | None | ~150 LOC in PlanToBTCompiler |

---

## 2. ROS2 Node Wrappers

Thin adapter nodes wrapping the core C++ library. The core remains ROS-agnostic.

### Nodes

- **WorldModel Node** — wraps `WorldModel` with ROS2 services (`get_fact`, `set_fact`, `query_state`) and publishes `/world_state` topic on change. Perception nodes are independent clients.
- **Planner Node** — stateless action server. Receives STRIPS_Problem snapshot + goal, returns plan.
- **Executor Node** — owns BT runtime. Ticks tree, calls WorldModel services for precondition/effect operations.

### Design constraints

- All nodes share a lifecycle manager
- Single-node (in-process) and multi-node (distributed) configurations from the same code
- WorldModel is the natural service boundary — everything else is a client

### Depends on

Core vertical slice complete. Observability Layer 2 (event stream) should land first so ROS2 topics can be a sink.

---

## 3. Perception Integration

External systems update world model state. The `WorldModel::setFact()` API already supports this — what's needed is the ROS2 service wrapper (from extension 2) and conventions for source tagging in the audit log (extension 1, Layer 3).

No new core code required beyond what extensions 1 and 2 provide.

---

## 4. PYRAMID Service Nodes

`InvokeService` BT node that maps PDDL actions to PYRAMID SDK service calls.

```cpp
class InvokeService : public BT::StatefulActionNode {
    static PortsList providedPorts() {
        return { InputPort<std::string>("service_name"),
                 InputPort<std::string>("operation"),
                 BidirectionalPort<ServiceMessage>("request"),
                 OutputPort<ServiceMessage>("response") };
    }
};
```

### Depends on

PYRAMID SDK availability. ActionRegistry already supports mapping PDDL actions to arbitrary BT node types, so integration is primarily about implementing the `InvokeService` node.

---

## 5. Thread Safety

Versioned snapshots for concurrent WorldModel access. Needed for ROS2 multi-node deployment where the BT tick thread and perception update threads access the world model concurrently.

Approach: the BT reads from a consistent snapshot; perception writes to a pending buffer; snapshots swap at defined sync points (between ticks).

### Depends on

ROS2 integration (extension 2). Not needed for single-threaded vertical slice.

---

## 6. Hierarchical Planning

`ExecutePhaseAction` BT node that triggers sub-planners for decomposed mission phases.

```cpp
class ExecutePhaseAction : public BT::StatefulActionNode {
    BT::NodeStatus onStart() override {
        auto sub_goals = mission_model_.getPhaseGoals(phase);
        auto sub_plan = planner_.solve(world_model_, sub_goals);
        sub_tree_ = compiler_.compileAndCreate(sub_plan, config().blackboard);
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override {
        return sub_tree_->tickOnce();
    }
};
```

### Depends on

Core vertical slice complete. Plan audit trail (extension 1, Layer 5) should capture sub-planning episodes.

---

## 7. Temporal Planning

PDDL 2.1 durative actions with STN (Simple Temporal Network) conversion. Extends the planner and plan-to-BT compiler to handle time-bounded actions.

### Depends on

Core vertical slice + hierarchical planning. Requires a temporal planner backend (LAPKT's temporal extensions or an external solver like OPTIC).

---

## Priority Order

1. **Observability Layers 1–3** — immediate, no blockers
2. **ROS2 Node Wrappers** — after vertical slice
3. **Observability Layers 4–5** — after Layer 2 + ROS2
4. **Perception Integration** — after ROS2
5. **PYRAMID Service Nodes** — when SDK available
6. **Thread Safety** — when multi-node deployment needed
7. **Hierarchical Planning** — post-core hardening
8. **Temporal Planning** — last, most complex
