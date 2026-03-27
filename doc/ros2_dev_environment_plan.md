# ROS2 Planning Dev Environment Plan

## Goal

A single, cross-platform GUI application that lets a developer:

1. **Define problems** — load/edit PDDL domain+problem files, manage type hierarchies, add objects
2. **Manipulate facts** — set/get/browse world model state, inject perception, set goals
3. **Trigger planning** — invoke the planner, inspect plan steps, view compiled BT XML
4. **Monitor execution** — live BT tick visualisation, replan events, node status
5. **Observe everything** — unified view of all 5 observability layers (BT events, WM audit, plan audit, traces, statistics)

All through a polished, unified interface that works on Linux, macOS, and Windows.

---

## Architecture Decision: Python + Dear PyGui

### Options Evaluated

| Option | Pros | Cons | Verdict |
|--------|------|------|---------|
| **Foxglove Studio** (current) | Already integrated, good for raw message viewing | Read-only — no fact setting, no PDDL editing, no plan triggering; panel layout is generic, not domain-aware; Electron-heavy; custom panels require their own React build toolchain | **Keep as optional companion, not primary** |
| **ImGui (C++)** | Fast, GPU-rendered, immediate-mode | Requires C++ build integration, harder to iterate, text editing is weak, no native file dialogs without platform libs | Rejected |
| **Dear PyGui** (Python imgui) | GPU-rendered, 60fps, immediate-mode but Pythonic; built-in plots, node editor, tables, tree views, file dialogs; single `pip install`; cross-platform | Smaller ecosystem than Qt; less "native" look | **Selected — primary UI** |
| **PyQt / PySide6** | Mature, native look, rich widgets | Heavy dependency, GPL/commercial licensing concerns (PyQt), slower for real-time streaming, retained-mode adds complexity | Rejected |
| **Streamlit / Gradio** | Fastest to prototype | Web-based, high latency, poor for real-time streaming, not suitable for mission-critical tooling | Rejected |
| **Custom web (React + WebSocket)** | Flexible, cross-platform via browser | Significant frontend effort, two codebases to maintain, latency | Rejected |

### Why Dear PyGui

- **Zero build complexity** — `pip install dearpygui`, runs immediately
- **Real-time capable** — GPU-rendered at 60fps, handles streaming BT events without lag
- **Rich built-in widgets** — plots (time series), tables, tree views, tab bars, node editor (for BT visualisation), colour themes, file dialogs
- **Cross-platform** — Windows, Linux, macOS from same codebase
- **Python ecosystem** — direct access to `rclpy` for ROS2, `websocket-client` for Foxglove bridge, `json` for JSONL parsing, `subprocess` for CLI tools
- **Theming** — built-in dark/light themes with full colour customisation for a polished look

---

## Communication Layer

The dev environment connects to the running ROS2 system via two parallel channels:

```
┌─────────────────────────────────────────────────────┐
│                  Dev Environment UI                  │
│                  (Python / Dear PyGui)                │
├──────────────────────┬──────────────────────────────┤
│   ROS2 Services      │   Foxglove WebSocket         │
│   (rclpy)            │   (websocket-client)         │
│                      │                              │
│  • ~/set_fact        │  • ws://localhost:8765       │
│  • ~/get_fact        │    ├─ /bt_events  (live)     │
│  • ~/query_state     │    └─ /wm_audit   (live)     │
│  • /ame/plan (action)│                              │
│  • /world_state (sub)│                              │
└──────────┬───────────┴──────────────┬───────────────┘
           │                          │
           ▼                          ▼
   ROS2 Nodes (C++)           FoxgloveBridge (C++)
   WorldModelNode             ws://localhost:8765
   PlannerNode
   ExecutorNode
```

### Why both channels?

| Channel | Purpose | Strengths |
|---------|---------|-----------|
| **rclpy** (ROS2) | Commands: set facts, trigger plans, query state | Request/response, typed services, action feedback |
| **Foxglove WS** | Streaming observability: BT events, WM audit | High-throughput, already implemented, no extra ROS2 topics needed |

### Offline / Standalone mode

When no ROS2 system is running, the UI can:
- Load and replay JSONL audit files (`ame_bt_events.jsonl`, `ame_wm_audit.jsonl`, `ame_plan_audit.jsonl`)
- Edit PDDL files locally
- Connect to just the Foxglove bridge (non-ROS2 `ame_test_app` demo)

---

## UI Layout

Single window, tab-based layout with a persistent status bar:

```
┌─────────────────────────────────────────────────────────────────┐
│  AME Dev Environment                              [Connected ●] │
├────────┬──────────┬───────────┬────────────┬────────────────────┤
│ World  │ Planning │ Execution │ Observ-    │ PDDL              │
│ Model  │          │           │ ability    │ Editor             │
├────────┴──────────┴───────────┴────────────┴────────────────────┤
│                                                                 │
│  ┌─────────────────────────┐  ┌──────────────────────────────┐  │
│  │                         │  │                              │  │
│  │    (active tab content) │  │   (context panel / details)  │  │
│  │                         │  │                              │  │
│  └─────────────────────────┘  └──────────────────────────────┘  │
│                                                                 │
├─────────────────────────────────────────────────────────────────┤
│ Status: WorldModel v42 │ 3 goals │ Plan: 7 steps │ BT: RUNNING │
└─────────────────────────────────────────────────────────────────┘
```

### Tab 1: World Model

| Panel | Content |
|-------|---------|
| **Type Hierarchy** | Tree view of types (`object → vehicle → uav`, etc.) |
| **Objects** | Table: name, type — with add/remove buttons |
| **Predicates** | Table: name, parameter types |
| **Facts** | Filterable table: fluent key, value (checkbox), authority, source, timestamp. Click to edit. Bulk set/clear. |
| **Goals** | List of goal fluent keys with add/remove. "Set Goal" button calls `WorldModel::setGoal()` via service. |
| **Inject Fact** | Quick-entry: pick predicate + objects from dropdowns, set value, source tag, authority. Calls `~/set_fact`. |

### Tab 2: Planning

| Panel | Content |
|-------|---------|
| **Goal Editor** | Select goal fluents from registered predicates |
| **Plan Button** | Sends `/ame/plan` action goal. Shows progress feedback (nodes expanded, elapsed time) in real-time |
| **Plan Result** | Ordered list of plan steps with action name + parameters. Solve time, nodes expanded/generated, cost. |
| **BT XML** | Syntax-highlighted view of compiled BT XML |
| **Plan History** | Table of past planning episodes from PlanAuditLog (Layer 5) |

### Tab 3: Execution

| Panel | Content |
|-------|---------|
| **BT Visualisation** | Tree layout of current behaviour tree nodes. Colour-coded by status: green=SUCCESS, yellow=RUNNING, red=FAILURE, grey=IDLE. Updates live from `/bt_events` stream. |
| **Node Inspector** | Click a BT node to see: tick count, success/failure rate, average duration, last status, ports/parameters |
| **Execution Controls** | Start/pause/halt execution (via ExecutorNode). Trigger manual replan. |
| **Timeline** | Scrollable time-series plot: x=time, y=node status transitions. Zoom/pan. Powered by Dear PyGui's plot widget. |

### Tab 4: Observability

| Panel | Content |
|-------|---------|
| **Live Event Stream** | Combined, colour-coded feed of BT events + WM audit entries. Filterable by source, node, fact key. Auto-scroll with pause button. |
| **WM Version Plot** | Time-series of world model version — shows rate of state change |
| **Fact History** | Select a fluent key → see all historical changes (value, source, timestamp) as a table and boolean step-plot |
| **Plan Audit** | Expandable list of planning episodes: initial state, goals, plan, solve stats, compiled BT |
| **JSONL Replay** | Load `.jsonl` files for offline analysis. Scrub through events with a time slider. |

### Tab 5: PDDL Editor

| Panel | Content |
|-------|---------|
| **File Browser** | Browse `domains/` directory, open domain.pddl + problem.pddl |
| **Text Editor** | Multi-line text input with basic PDDL syntax highlighting (keyword colouring via Dear PyGui's colour text API) |
| **Validation** | "Parse" button runs PddlParser and reports errors inline |
| **Load to WM** | "Load" button parses PDDL and populates the running WorldModel via ROS2 services |

---

## Implementation Plan

### Phase 1: Scaffold + ROS2 Connection (Week 1)

**Deliverables:** Running application shell with ROS2 connectivity.

```
tools/
  devenv/
    __init__.py
    main.py              # Entry point, Dear PyGui setup, theme, window layout
    ros2_client.py       # rclpy service/action/subscription wrappers
    foxglove_client.py   # WebSocket client for FoxgloveBridge
    config.py            # Connection settings, defaults
    requirements.txt     # dearpygui, websocket-client, rclpy (from ROS2)
```

Tasks:
1. Dear PyGui window with tab bar, status bar, dark theme
2. `ros2_client.py`: async wrapper around `rclpy` — `set_fact()`, `get_fact()`, `query_state()`, `plan()` (action client with feedback)
3. `foxglove_client.py`: WebSocket client that connects to `ws://localhost:8765`, subscribes to `/bt_events` and `/wm_audit`, queues events for UI consumption
4. Status bar showing connection state, WM version (from `/world_state` subscription)
5. Graceful degradation: if ROS2 unavailable, disable command tabs; if Foxglove unavailable, disable live streaming

### Phase 2: World Model Tab (Week 2)

Tasks:
1. On tab open, call `~/query_state` to populate facts table
2. Filterable, sortable facts table (Dear PyGui `table` widget)
3. Inline editing: click a fact's value checkbox → calls `~/set_fact`
4. "Inject Fact" panel with predicate/object dropdowns
5. Goal management panel
6. Subscribe to `/world_state` for live updates (highlight changed rows)
7. Type hierarchy tree view, objects table (read from initial PDDL parse or query)

### Phase 3: Planning Tab (Week 3)

Tasks:
1. Goal selection UI (checkboxes from registered predicates)
2. "Plan" button → `/ame/plan` action call with live feedback display
3. Plan result display: steps table, statistics
4. BT XML viewer with keyword colouring
5. Plan history table (load from `ame_plan_audit.jsonl` or accumulate from live sessions)

### Phase 4: Execution + Observability Tabs (Week 4)

Tasks:
1. Live BT event stream (from Foxglove WS) rendered as colour-coded log
2. BT tree visualisation — parse BT XML, render as indented tree with status colours
3. WM audit stream alongside BT events
4. Time-series plots (Dear PyGui `plot` widget): WM version over time, node status transitions
5. Fact history drill-down: select fluent → plot boolean value over time
6. JSONL replay: file picker, load events, time slider scrub

### Phase 5: PDDL Editor + Polish (Week 5)

Tasks:
1. File browser for `domains/` directory
2. Multi-line text editor with PDDL keyword colouring
3. Parse validation button (shell out to existing parser or use rclpy service)
4. "Load to WM" button
5. UI polish: consistent spacing, keyboard shortcuts, window save/restore
6. Cross-platform testing (Linux, Windows, macOS)

---

## Key Technical Details

### Threading Model

```
Main Thread (Dear PyGui)          Background Threads
┌──────────────────────┐    ┌─────────────────────────┐
│  UI render loop      │    │  rclpy spin thread       │
│  60fps tick          │◄───│  (services, actions,     │
│  reads from queues   │    │   subscriptions)         │
│                      │    ├─────────────────────────┤
│                      │◄───│  Foxglove WS thread      │
│                      │    │  (event streaming)       │
└──────────────────────┘    └─────────────────────────┘
         │
    thread-safe queues (collections.deque or queue.Queue)
```

- **rclpy** runs in a background thread (`executor.spin()`)
- **Foxglove WS** runs in a background thread (websocket-client `run_forever()`)
- Both push events into thread-safe queues
- Main thread drains queues each frame and updates UI state
- Service calls are async: UI shows spinner, callback updates result

### Dear PyGui Theming

```python
with dpg.theme() as global_theme:
    with dpg.theme_component(dpg.mvAll):
        dpg.add_theme_color(dpg.mvThemeCol_WindowBg,       (30, 30, 35))
        dpg.add_theme_color(dpg.mvThemeCol_TitleBgActive,  (45, 90, 160))
        dpg.add_theme_color(dpg.mvThemeCol_Tab,            (50, 50, 60))
        dpg.add_theme_color(dpg.mvThemeCol_TabActive,      (45, 90, 160))
        dpg.add_theme_color(dpg.mvThemeCol_FrameBg,        (40, 40, 50))
        dpg.add_theme_style(dpg.mvStyleVar_FrameRounding,  4)
        dpg.add_theme_style(dpg.mvStyleVar_WindowRounding,  6)
```

BT node status colours:
- `SUCCESS` → `(80, 200, 80)` (green)
- `RUNNING` → `(220, 180, 40)` (amber)
- `FAILURE` → `(220, 60, 60)` (red)
- `IDLE` → `(120, 120, 130)` (grey)

### ROS2 Client API (Python)

```python
class AmeRos2Client:
    """Wraps rclpy calls for the dev environment."""

    def __init__(self, node_name: str = "ame_devenv"):
        self._node = rclpy.create_node(node_name)
        # Service clients
        self._set_fact = self._node.create_client(SetFact, '/world_model_node/set_fact')
        self._get_fact = self._node.create_client(GetFact, '/world_model_node/get_fact')
        self._query    = self._node.create_client(QueryState, '/world_model_node/query_state')
        # Action client
        self._plan     = ActionClient(self._node, Plan, '/ame/plan')
        # Subscription
        self._wm_sub   = self._node.create_subscription(
            WorldState, '/world_state', self._on_world_state, 10)

    def set_fact(self, key: str, value: bool, source: str = "devenv") -> Future:
        ...

    def query_state(self, keys: list[str] | None = None) -> Future:
        ...

    def plan(self, goal_fluents: list[str], on_feedback=None) -> Future:
        ...
```

### Foxglove WebSocket Client

```python
class FoxgloveClient:
    """Connects to AME FoxgloveBridge for live event streaming."""

    def __init__(self, url: str = "ws://localhost:8765"):
        self._url = url
        self._bt_queue: deque[dict] = deque(maxlen=10000)
        self._wm_queue: deque[dict] = deque(maxlen=10000)

    def start(self):
        """Connect and subscribe to /bt_events and /wm_audit."""
        ...

    def drain_bt_events(self) -> list[dict]:
        """Non-blocking drain of queued BT events for UI consumption."""
        ...

    def drain_wm_events(self) -> list[dict]:
        """Non-blocking drain of queued WM audit events."""
        ...
```

---

## Dependencies

```
# requirements.txt
dearpygui>=1.11       # UI framework — pip install, no build needed
websocket-client>=1.6 # Foxglove WS connection

# ROS2 (from system, not pip):
# rclpy, ame_ros2 interfaces — available when sourced into a ROS2 workspace
```

### Cross-Platform Notes

| Platform | Dear PyGui | rclpy | Foxglove WS |
|----------|-----------|-------|-------------|
| **Linux** (Ubuntu 22/24) | pip install | `apt install ros-jazzy-*` + source workspace | Works |
| **Windows** | pip install | ROS2 Jazzy Windows binaries + source workspace | Works |
| **macOS** | pip install | ROS2 via robostack-humble conda | Works |
| **No ROS2** | pip install | Disabled (offline/replay mode only) | Works (connects to `ame_test_app`) |

---

## Standalone Mode (No ROS2)

For developers who want to use the dev environment without a full ROS2 installation:

1. **Replay mode** — load JSONL files from previous runs, scrub through events
2. **Direct bridge mode** — connect to `ame_test_app` (non-ROS2 demo) via Foxglove WS for live observability
3. **PDDL editing** — edit domain/problem files locally, validate syntax via bundled Python PDDL parser

This ensures the tool is useful even during early development before the full ROS2 deployment is running.

---

## Directory Structure

```
tools/devenv/
├── main.py                  # Entry point
├── config.py                # Settings, connection defaults, theme colours
├── requirements.txt
│
├── comms/
│   ├── __init__.py
│   ├── ros2_client.py       # rclpy service/action wrappers
│   └── foxglove_client.py   # WebSocket streaming client
│
├── ui/
│   ├── __init__.py
│   ├── app.py               # Top-level window, tab bar, status bar
│   ├── theme.py             # Dear PyGui theme definition
│   ├── world_model_tab.py   # Facts, objects, types, goals
│   ├── planning_tab.py      # Goal editor, plan trigger, results
│   ├── execution_tab.py     # BT visualisation, controls
│   ├── observability_tab.py # Live streams, plots, replay
│   └── pddl_editor_tab.py  # File browser, editor, validation
│
├── models/
│   ├── __init__.py
│   ├── events.py            # BT event / WM audit data classes
│   └── replay.py            # JSONL file loader + time scrubber
│
└── assets/
    └── icon.png             # App icon
```

---

## Future Extensions

| Extension | Description |
|-----------|-------------|
| **3D Scene View** | Integrate Open3D or PyVista for spatial visualisation of objects/locations |
| **PDDL Autocomplete** | Use registered predicates/types to provide autocomplete in the PDDL editor |
| **Mission Templates** | Save/load common goal configurations as named templates |
| **Multi-Robot Dashboard** | Split views per robot, show per-agent BT and fact subsets |
| **Recording/Playback** | Record a full session (all channels) to a single file for deterministic replay |
| **Remote Connection** | Connect to ROS2 nodes on a remote machine via DDS discovery or Zenoh bridge |

---

## Summary

**Choice:** Python + Dear PyGui for the primary dev environment UI.

**Rationale:** Zero build complexity (`pip install`), GPU-rendered 60fps for real-time streaming, rich widget set (tables, plots, trees), cross-platform, and direct access to `rclpy` for ROS2 service calls and `websocket-client` for the existing Foxglove bridge.

**Foxglove Studio** remains available as a complementary tool for ad-hoc inspection but is not the primary interface — it cannot set facts, trigger plans, or provide a domain-aware workflow.

**Timeline:** 5 phases over ~5 weeks, from scaffold to polished cross-platform tool.
