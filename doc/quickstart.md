# Quickstart Guide

Build, run, and test the MUJIN planning + execution pipeline with live observability.

---

## Prerequisites

- CMake 3.21+
- C++17 compiler (GCC 9+, Clang 10+, MSVC 2019+)
- SQLite3 development headers (`libsqlite3-dev` on Debian/Ubuntu)
- Git (for FetchContent downloads)

Optional:

- [Foxglove Studio](https://foxglove.dev/download) for live BT/WM visualization

## Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
```

To disable the Foxglove bridge (removes websocketpp/asio dependencies):

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug -DMUJIN_FOXGLOVE=OFF
```

## Run tests

```bash
ctest --test-dir build --output-on-failure
```

All 73 tests should pass, covering: WorldModel, TypeSystem, ActionRegistry, PlanCompiler, Planner, PDDL Parser, BT integration, end-to-end pipeline, and observability (Layers 1-5).

## Run the demo

```bash
./build/src/mujin_test_app
```

This runs the UAV search-and-classify example end-to-end:

1. Builds a WorldModel with types, objects, predicates, and actions
2. Plans with LAPKT BRFS solver
3. Compiles the plan to BT XML
4. Executes the BT to completion
5. Verifies goal state

Output files produced in the current working directory:

| File | Contents |
|------|----------|
| `mujin_bt_events.jsonl` | BT node state transitions (Layer 2) |
| `mujin_wm_audit.jsonl` | World model fact changes (Layer 3) |
| `mujin_plan_audit.jsonl` | Planning episode audit trail (Layer 5) |

---

## Inspecting output files

All output files use JSONL (one JSON object per line), viewable with standard tools:

```bash
# Pretty-print BT events
cat mujin_bt_events.jsonl | python3 -m json.tool --json-lines

# Show all WM fact changes
cat mujin_wm_audit.jsonl | python3 -m json.tool --json-lines

# Inspect the plan audit trail (init state, goals, solver, plan, BT XML)
cat mujin_plan_audit.jsonl | python3 -m json.tool

# Query specific fields with jq
cat mujin_bt_events.jsonl | jq 'select(.status == "SUCCESS")'
cat mujin_wm_audit.jsonl | jq 'select(.source | startswith("SetWorldPredicate"))'
cat mujin_plan_audit.jsonl | jq '.plan_actions'
```

---

## Foxglove Studio integration

The `FoxgloveBridge` implements the [Foxglove WebSocket protocol](https://github.com/foxglove/ws-protocol), enabling Foxglove Studio to connect for live monitoring without ROS2.

### Setup

1. **Download Foxglove Studio** from [foxglove.dev/download](https://foxglove.dev/download) (free, cross-platform).

2. **Run the demo app** — it starts a WebSocket server automatically:

   ```bash
   ./build/src/mujin_test_app
   ```

   You should see:

   ```
   [Foxglove] WebSocket server listening on ws://localhost:8765
   ```

3. **Connect Foxglove Studio:**

   - Open Foxglove Studio
   - Click **"Open connection"**
   - Select **"Foxglove WebSocket"**
   - Enter URL: `ws://localhost:8765`
   - Click **"Open"**

### Available channels

Once connected, Foxglove discovers two channels:

| Channel | Schema | Description |
|---------|--------|-------------|
| `/bt_events` | `mujin.BTEvent` | BT node state transitions: timestamp, node name, node type, previous status, new status, tree ID, WM version |
| `/wm_audit` | `mujin.WMFactChange` | World model fact changes: WM version, timestamp, fact key, new value, source tag |

### Recommended panels

Add these panels in Foxglove Studio for a useful monitoring layout:

- **Raw Messages** — subscribe to `/bt_events` to see live BT transitions
- **Raw Messages** (second panel) — subscribe to `/wm_audit` to see WM changes
- **Log** — subscribe to either channel for a scrolling event log
- **Plot** — plot `wm_version` from `/bt_events` to visualize WM version progression over time

### Testing the live connection

Since the demo app runs to completion quickly, the Foxglove connection is most useful when you:

1. **Add a sleep** in main.cpp before `tree.tickWhileRunning()` to give time to connect
2. **Build a longer-running scenario** with more planning steps or async actions
3. **Use the API programmatically** in your own code:

```cpp
#include "mujin/foxglove_bridge.h"
#include "mujin/bt_logger.h"

// Start the bridge
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
    std::string json = "{\"wm_version\":" + std::to_string(ver)
        + ",\"ts_us\":" + std::to_string(ts)
        + ",\"fact\":\"" + fact + "\""
        + ",\"value\":" + (val ? "true" : "false")
        + ",\"source\":\"" + src + "\"}";
    wm_sink(json);
});

// ... tick tree, do work ...

bridge.stop();
```

### Disabling Foxglove

To build without Foxglove (removes websocketpp/asio dependencies):

```bash
cmake -B build -DMUJIN_FOXGLOVE=OFF
```

The `#if defined(MUJIN_FOXGLOVE)` guards in `main.cpp` ensure the code compiles cleanly either way.

---

## Architecture overview

```
                PDDL Domain
                    |
                    v
    WorldModel  <-- Planner (LAPKT BRFS)
    (facts +        |
     actions)       v
        |       PlanCompiler --> BT XML
        |                          |
        v                          v
    WmAuditLog              BT.CPP Executor
    (Layer 3)                  |       |
        |               TreeObserver  MujinBTLogger
        |               (Layer 1)     (Layer 2)
        |                                |
        +----------+---------------------+
                   |
                   v
            FoxgloveBridge (Layer 4)
            ws://localhost:8765
                   |
                   v
            Foxglove Studio

    PlanAuditLog (Layer 5) -- records planning episodes
```

See `doc/concept.md` for full architecture details and `doc/extensions.md` for the extension roadmap.
