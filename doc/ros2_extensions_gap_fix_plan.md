# ROS2 Extensions Gap-Fix Plan

**Status: NOT STARTED**

Gap analysis revealed that extensions 3-6 and multi-agent delegation are implemented in `ame_core` but incompletely wired into the ROS2 layer. This plan addresses all missing/conflicting elements identified in the review.

---

## Summary of Gaps

| # | Gap | Severity | Effort |
|---|-----|----------|--------|
| G1 | PYRAMID `InvokeService` not registered in `ExecutorNode`; no ROS2 adapter | High | Medium |
| G2 | `ExecutePhaseAction` not registered; blackboard keys not wired | High | Medium |
| G3 | `DelegateToAgent` not registered; no bridge to `AgentDispatcherNode` transport | High | Medium |
| G4 | `AgentDispatcherNode` missing `~/dispatch_goals` service; `DispatchGoals.srv` absent | High | Low |
| G5 | Perception handler uses `setFact()` directly instead of mutation queue | Low | Low |
| G6 | `06-ros2.md` says "three lifecycle nodes" but there are four; dispatcher undocumented | Medium | Low |
| G7 | `07-extensions.md` does not document `DelegateToAgent` | Medium | Low |
| G8 | Multi-agent launch missing `AgentDispatcherNode`; lifecycle manager default excludes it | Medium | Low |

---

## Step 1: Create `DispatchGoals.srv` (G4)

**Files:** `ros2/srv/DispatchGoals.srv` (new)

The `AgentDispatcherNode` header documents a `~/dispatch_goals` service, but the `.srv` file does not exist and the service is never created. Define the service interface and wire it.

```
# DispatchGoals.srv — trigger multi-agent goal dispatch

# Request
string[] goal_fluents          # Top-level mission goals
string[] agent_ids             # Agents to dispatch to (empty = all available)
---
# Response
bool success
string error_msg
string[] dispatched_agents     # Agents that received sub-plans
```

**Implementation:**

1. Create `ros2/srv/DispatchGoals.srv` with the definition above.
2. Add to `ros2/CMakeLists.txt` in the `rosidl_generate_interfaces` call.
3. In `AgentDispatcherNode::on_activate()`, create the service:
   ```cpp
   srv_dispatch_ = create_service<ame_ros2::srv::DispatchGoals>(
     "~/dispatch_goals",
     [this](auto req, auto res) { handleDispatchGoals(req, res); });
   ```
4. Implement `handleDispatchGoals()`: call `component_.dispatch(goals, agents)`, iterate results, call `sendBTToAgent()` for each, populate response.

**Effort:** ~80 LOC, low complexity.

---

## Step 2: Wire `InvokeService` into `ExecutorNode` (G1)

**Files:** `ros2/src/executor_node.cpp`, `ros2/include/ame_ros2/executor_node.hpp`

The core `InvokeService` BT node requires an `IPyramidService*` on the blackboard under key `"pyramid_service"`. The executor must register the node type and inject the service pointer.

### 2a: Register `InvokeService` in executor factory

In `ExecutorNode::registerCoreNodes()`:

```cpp
#include <ame/bt_nodes/invoke_service.h>

// After CheckWorldPredicate / SetWorldPredicate registration:
component_.factory().registerNodeType<ame::InvokeService>("InvokeService");
```

This is unconditional — the same `InvokeService` node works in both in-process and distributed modes because it talks to the PYRAMID SDK, not the WorldModel.

### 2b: Add `IPyramidService*` injection point

Add a setter on `ExecutorNode`:

```cpp
// executor_node.hpp
void setPyramidService(ame::IPyramidService* svc) { pyramid_service_ = svc; }

// executor_node.hpp (private)
ame::IPyramidService* pyramid_service_ = nullptr;
```

Wire into the blackboard initializer (in `on_configure`, inside the lambda):

```cpp
if (pyramid_service_) {
    blackboard->set<ame::IPyramidService*>("pyramid_service", pyramid_service_);
}
```

### 2c: Create `RosPyramidServiceAdapter` (optional, future)

A full ROS2 adapter wrapping PYRAMID SDK calls as ROS2 service clients is a larger piece of work that depends on the PYRAMID SDK deployment. For now:

- The injection point allows any `IPyramidService*` to be passed in (including `MockPyramidService` for testing).
- `combined_main.cpp` can demonstrate wiring a `MockPyramidService` instead of stub actions.
- A real `RosPyramidServiceAdapter` can be added later without changing `ExecutorNode`.

**Effort:** ~30 LOC for 2a+2b; adapter (2c) deferred.

---

## Step 3: Wire `ExecutePhaseAction` into `ExecutorNode` (G2)

**Files:** `ros2/src/executor_node.cpp`, `ros2/include/ame_ros2/executor_node.hpp`

`ExecutePhaseAction` requires several blackboard keys. The wiring differs between in-process and distributed modes.

### 3a: Register the BT node

In `ExecutorNode::registerCoreNodes()`:

```cpp
#include <ame/bt_nodes/execute_phase_action.h>

component_.factory().registerNodeType<ame::ExecutePhaseAction>("ExecutePhaseAction");
```

### 3b: In-process blackboard wiring

Add injection setters for the dependencies `ExecutePhaseAction` needs:

```cpp
// executor_node.hpp
void setPlanner(ame::Planner* p) { planner_ = p; }
void setPlanCompiler(ame::PlanCompiler* c) { plan_compiler_ = c; }
void setActionRegistry(ame::ActionRegistry* r) { action_registry_ = r; }
void setPlanAuditLog(ame::PlanAuditLog* l) { plan_audit_log_ = l; }
```

In the blackboard initializer:

```cpp
blackboard->set<BT::BehaviorTreeFactory*>("bt_factory", &component_.factory());

if (inprocess_wm_) {
    blackboard->set<ame::WorldModel*>("world_model", inprocess_wm_);
}
if (planner_) {
    blackboard->set<ame::Planner*>("planner", planner_);
}
if (plan_compiler_) {
    blackboard->set<ame::PlanCompiler*>("plan_compiler", plan_compiler_);
}
if (action_registry_) {
    blackboard->set<ame::ActionRegistry*>("action_registry", action_registry_);
}
if (plan_audit_log_) {
    blackboard->set<ame::PlanAuditLog*>("plan_audit_log", plan_audit_log_);
}
```

### 3c: Distributed blackboard wiring (PlannerComponent path)

When running distributed, `ExecutePhaseAction` can use `PlannerComponent*` instead of direct planner/compiler/registry. Add:

```cpp
void setPlannerComponent(ame::PlannerComponent* pc) { planner_component_ = pc; }
```

And in the blackboard initializer:

```cpp
if (planner_component_) {
    blackboard->set<ame::PlannerComponent*>("planner_component", planner_component_);
}
```

### 3d: Wire in `combined_main.cpp`

```cpp
ex_node->setPlanner(&pl_node->planner());       // expose via PlannerNode
ex_node->setPlanCompiler(&pl_node->compiler());
ex_node->setActionRegistry(&pl_node->actionRegistry());
```

This requires exposing `planner()` and `compiler()` accessors on `PlannerNode`, mirroring the existing `actionRegistry()`.

**Effort:** ~60 LOC across header/source, ~10 LOC in combined_main.

---

## Step 4: Wire `DelegateToAgent` into `ExecutorNode` (G3)

**Files:** `ros2/src/executor_node.cpp`

`DelegateToAgent` uses the same blackboard keys as `ExecutePhaseAction` (world_model, planner, plan_compiler, action_registry, bt_factory, plan_audit_log). Since Step 3 already wires those, this step only needs registration.

### 4a: Register the BT node

In `ExecutorNode::registerCoreNodes()`:

```cpp
#include <ame/bt_nodes/delegate_to_agent.h>

component_.factory().registerNodeType<ame::DelegateToAgent>("DelegateToAgent");
```

### 4b: Bridge to `AgentDispatcherNode` transport (future)

Currently `DelegateToAgent` does in-process sub-planning and sub-tree execution. For true distributed delegation (where sub-plans are sent to remote executors), the node would need to:

1. Detect an `AgentDispatcher*` on the blackboard.
2. Use the dispatcher's `sendBTToAgent()` instead of local sub-tree creation.
3. Poll `queryAgentStatus()` in `onRunning()`.

This is a larger architectural change that should be addressed when distributed multi-agent execution is needed. For now, registering the node enables the in-process leader-delegation pattern documented in `multi_agent_implementation_plan.md`.

**Effort:** ~5 LOC for registration; distributed bridge deferred.

---

## Step 5: Fix perception handler to use mutation queue (G5)

**Files:** `ros2/src/world_model_node.cpp`

The doc (Extension 5, `07-extensions.md`) describes two perception paths: the ROS2 `/detections` topic and the in-process mutation queue. Currently `handleDetection()` calls `wm.setFact()` directly, which acquires an exclusive write lock. This is correct for single-threaded mode, but the documented pattern recommends `enqueueMutation()` + `applyQueuedMutations()` for the perception path.

### Change

Replace the direct `setFact()` call in `handleDetection()` with:

```cpp
// Before (current):
wm.setFact(fact_key, true, source, ame::FactAuthority::CONFIRMED);

// After:
wm.enqueueMutation(fluent_id, true, source, ame::FactAuthority::CONFIRMED);
```

Add a periodic flush in the publish timer callback (which already fires at `publish_rate_hz`):

```cpp
publish_timer_ = create_wall_timer(period, [this]() {
    size_t applied = component_.worldModel().applyQueuedMutations();
    if (applied > 0 || component_.consumeStateDirty()) {
        publishWorldState();
    }
});
```

This batches perception updates and applies them atomically between publish cycles, matching the documented architecture and reducing lock contention.

### Authority conflict check

The conflict check currently happens before `setFact()`. Move it before `enqueueMutation()` — it only reads state, so it can use the shared read lock:

```cpp
if (wm.hasAuthorityConflict(fluent_id, perceived_value)) {
    RCLCPP_WARN(get_logger(), "Authority conflict: ...");
}
wm.enqueueMutation(fluent_id, true, source, ame::FactAuthority::CONFIRMED);
```

**Effort:** ~15 LOC change, low risk.

---

## Step 6: Update `AgentDispatcherNode` and multi-agent launch (G4, G8)

**Files:** `ros2/src/agent_dispatcher_node.cpp`, `ros2/launch/ame_multi_agent.launch.py`, `ros2/include/ame_ros2/lifecycle_manager.hpp`

### 6a: Wire `~/dispatch_goals` service in `on_activate()`

After Step 1 defines the `.srv`, implement the service handler:

```cpp
void AgentDispatcherNode::on_activate(...) {
    // ... existing code ...

    srv_dispatch_ = create_service<ame_ros2::srv::DispatchGoals>(
        "~/dispatch_goals",
        [this](auto req, auto res) { handleDispatchGoals(req, res); });
}
```

The handler delegates to the `AgentDispatcher` component, which already has goal allocation and plan compilation logic.

### 6b: Add `AgentDispatcherNode` to multi-agent launch

In `ame_multi_agent.launch.py`, add:

```python
LifecycleNode(
    package='ame_ros2',
    executable='agent_dispatcher_node',
    name='agent_dispatcher_node',
    output='screen',
    parameters=[{
        'world_model_node': 'world_model_node',
    }],
),
```

Add `'agent_dispatcher_node'` to the lifecycle manager's `managed_nodes` list (after `planner_node`, before executor nodes).

### 6c: Create `agent_dispatcher_node_main.cpp`

A standalone main for the dispatcher, mirroring the pattern of the other `*_main.cpp` files:

```cpp
#include <ame_ros2/agent_dispatcher_node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ame_ros2::AgentDispatcherNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
```

Add to `ros2/CMakeLists.txt` as a new executable target.

**Effort:** ~60 LOC across files.

---

## Step 7: Expose `PlannerNode` internals for in-process wiring (G2 support)

**Files:** `ros2/include/ame_ros2/planner_node.hpp`

Step 3 requires `combined_main.cpp` to inject planner/compiler/registry into the executor. `PlannerNode` already exposes `actionRegistry()`. Add:

```cpp
/// \brief Expose Planner for in-process hierarchical planning.
ame::Planner& planner() { return component_.planner(); }

/// \brief Expose PlanCompiler for in-process hierarchical planning.
ame::PlanCompiler& compiler() { return component_.compiler(); }

/// \brief Expose PlanAuditLog for in-process audit trail.
ame::PlanAuditLog* planAuditLog() { return component_.planAuditLog(); }
```

This requires corresponding accessors on `PlannerComponent`. Check whether they already exist; if not, add them.

**Effort:** ~10 LOC.

---

## Step 8: Update `combined_main.cpp` demo (G1, G2, G3)

**Files:** `ros2/src/combined_main.cpp`

Replace or augment the stub actions to demonstrate extension wiring:

```cpp
#include <ame/bt_nodes/invoke_service.h>
#include <ame/bt_nodes/execute_phase_action.h>
#include <ame/bt_nodes/delegate_to_agent.h>
#include <ame/pyramid_service.h>

// ...

// Wire PYRAMID service (mock for demo)
ame::MockPyramidService mock_pyramid;
ex_node->setPyramidService(&mock_pyramid);

// Wire hierarchical planning dependencies
ex_node->setPlanner(&pl_node->planner());
ex_node->setPlanCompiler(&pl_node->compiler());
ex_node->setActionRegistry(&pl_node->actionRegistry());
ex_node->setPlanAuditLog(pl_node->planAuditLog());
```

Keep the stub actions registered alongside — they're still needed for the UAV search demo domain. The `InvokeService` / `ExecutePhaseAction` / `DelegateToAgent` nodes are additional capabilities, not replacements.

**Effort:** ~20 LOC.

---

## Step 9: Update documentation (G6, G7)

### 9a: Update `doc/architecture/06-ros2.md`

1. Change "three lifecycle nodes" to "four lifecycle nodes" in the opening paragraph.
2. Add `AgentDispatcherNode` section after ExecutorNode:
   ```markdown
   - **AgentDispatcherNode** (`ros2/src/agent_dispatcher_node.cpp`) — wraps
     `AgentDispatcher` PCL component for multi-agent coordination. Exposes
     `~/dispatch_goals` service. Dynamically creates per-agent publishers
     (`/{agent_id}/executor/bt_xml`) and status subscriptions.
   ```
3. Update the ASCII node diagram to include the dispatcher.
4. Add a "Multi-Agent" deployment mode section describing the `ame_multi_agent.launch.py` topology.

### 9b: Update `doc/architecture/07-extensions.md`

1. Add a "Multi-Agent Delegation" section documenting the `DelegateToAgent` BT node, its ports, blackboard requirements, and leader-delegation pattern. Reference `multi_agent_implementation_plan.md` for the full architecture.
2. In the PYRAMID Service section, add a note about ROS2 wiring:
   ```markdown
   ### ROS2 Integration
   `ExecutorNode` registers `InvokeService` and injects `IPyramidService*` via
   `setPyramidService()`. For in-process demos, `MockPyramidService` is used.
   Production deployments should provide a concrete adapter.
   ```
3. In the Hierarchical Planning section, add:
   ```markdown
   ### ROS2 Integration
   `ExecutorNode` exposes setters for planner/compiler/registry/audit-log.
   In-process mode: `combined_main.cpp` wires these from `PlannerNode`.
   Distributed mode: inject a `PlannerComponent*` instead.
   ```

### 9c: Update `doc/TODO.md`

Add a section for this gap-fix work:

```markdown
## ROS2 Extension Wiring (Gap Fix)

**Status: NOT STARTED**

- [ ] `DispatchGoals.srv` and `AgentDispatcherNode` service wiring (Steps 1, 6)
- [ ] `InvokeService` registration + `IPyramidService*` injection (Step 2)
- [ ] `ExecutePhaseAction` registration + blackboard wiring (Step 3)
- [ ] `DelegateToAgent` registration (Step 4)
- [ ] Perception mutation queue fix (Step 5)
- [ ] `PlannerNode` accessor exposure (Step 7)
- [ ] `combined_main.cpp` demo update (Step 8)
- [ ] Documentation updates (Step 9)
- [ ] Tests (Step 10)
```

**Effort:** ~100 lines of documentation changes.

---

## Step 10: Tests

### 10a: Unit test — `InvokeService` via `ExecutorNode`

In `ros2/test/test_executor_node.cpp`, add a test that:

1. Creates an `ExecutorNode` with `MockPyramidService` injected.
2. Loads a BT XML containing an `<InvokeService>` node.
3. Ticks until completion.
4. Asserts `SUCCESS`.

### 10b: Unit test — `ExecutePhaseAction` via `ExecutorNode`

1. Creates an `ExecutorNode` with in-process planner/compiler/registry wired.
2. Loads a BT XML containing `<ExecutePhaseAction phase_goals="..." phase_name="test"/>`.
3. Ticks until completion.
4. Asserts `SUCCESS` and verifies audit log recorded the sub-episode.

### 10c: Unit test — `DelegateToAgent` via `ExecutorNode`

1. Registers agents in WorldModel.
2. Loads BT XML with `<DelegateToAgent agent_id="uav1" agent_goals="..."/>`.
3. Ticks until completion.
4. Asserts agent marked busy then available again.

### 10d: Unit test — `DispatchGoals` service

1. Spins up `WorldModelNode` + `PlannerNode` + `AgentDispatcherNode`.
2. Calls `~/dispatch_goals` with goal fluents and agent IDs.
3. Asserts success response and agents listed in `dispatched_agents`.

### 10e: Integration test — perception mutation queue

1. Sends detections via `/detections` topic.
2. Verifies facts are not immediately visible (queued).
3. After publish timer fires, verifies facts are applied and `/world_state` updated.

**Effort:** ~200 LOC of tests.

---

## Implementation Order

```
Step 1  ──────────────────────────────────────────▶  DispatchGoals.srv
Step 7  ──────────────────────────────────────────▶  PlannerNode accessors
Step 2  ─── depends on nothing ───────────────────▶  InvokeService wiring
Step 3  ─── depends on Step 7 ────────────────────▶  ExecutePhaseAction wiring
Step 4  ─── depends on Step 3 (shared BB keys) ──▶  DelegateToAgent registration
Step 5  ─── independent ─────────────────────────▶  Perception queue fix
Step 6  ─── depends on Step 1 ───────────────────▶  Dispatcher node + launch
Step 8  ─── depends on Steps 2-4, 7 ────────────▶  combined_main.cpp update
Step 9  ─── after implementation ────────────────▶  Documentation
Step 10 ─── after implementation ────────────────▶  Tests
```

Parallelisable: Steps 1+7 together, then 2+5 together, then 3 -> 4, then 6+8, then 9+10.

**Total estimated effort:** ~500 LOC of implementation + ~200 LOC of tests + ~100 lines of documentation.
