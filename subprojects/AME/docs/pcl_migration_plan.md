# AME ROS2 Node PCL Migration Plan

## Problem Statement

The four AME ROS2 nodes (`WorldModelNode`, `PlannerNode`, `ExecutorNode`, `AgentDispatcherNode`) inherit from `pcl::Component` but only use its lifecycle state machine and parameter store. All communication (pub/sub, services, timers) is wired directly through ROS2 primitives, bypassing PCL's port system, executor, transport abstraction, and thread-safety guarantees.

This defeats the purpose of PCL:

- **No transport portability** -- components cannot run over PCL sockets or any non-ROS2 transport without rewriting the nodes.
- **Broken thread safety** -- `PlannerNode` spawns detached `std::thread` for planning; `WorldModelNode::handleDetection` mutates world model state from the ROS2 callback thread.  PCL's single-threaded executor guarantee is bypassed.
- **Duplicated wiring** -- every ROS2 node manually creates publishers, subscribers, services, and timers that PCL's port system was designed to abstract.
- **Unusable PCL executor** -- since no ports are registered on any component, the PCL executor has nothing to dispatch.

### Reference implementation

The PYRAMID `StandardBridge` and `TacticalObjectsComponent` (`subprojects/PYRAMID/tactical_objects/`) demonstrate correct PCL usage: ports created in `on_configure()`, executor manages all dispatch, transport adapter provides socket connectivity.

---

## Scope

| In scope | Out of scope |
|----------|-------------|
| Migrate all four AME components to use PCL ports | ROS2 transport adapter implementation (Phase 4 -- separate work) |
| Move tick logic into `on_tick()` | Changing PDDL/BT business logic |
| Use `pcl_executor_post_incoming()` for thread-safe ingress | Adding new features or components |
| Create a PCL executor-based `combined_main` | Modifying PCL itself |
| Preserve distributed (ROS2) and in-process deployment modes | |

---

## Architecture Target

```
                     +-----------------------+
                     |    pcl::Executor      |   single-threaded tick loop
                     |  (deterministic)      |
                     +-----------+-----------+
                                 |
            +--------------------+--------------------+
            |                    |                    |
  +---------v--------+  +-------v--------+  +--------v---------+
  | WorldModelComponent|  | PlannerComponent |  | ExecutorComponent  |
  |  ports:            |  |  ports:          |  |  ports:            |
  |   pub: world_state |  |   sub: plan_req  |  |   sub: bt_xml      |
  |   sub: detections  |  |   pub: bt_xml    |  |   pub: bt_events   |
  |   svc: get_fact    |  |   svc: load_dom  |  |   pub: status      |
  |   svc: set_fact    |  |                  |  |   on_tick(): BT     |
  |   svc: query_state |  |                  |  |                    |
  |   svc: load_domain |  |                  |  |                    |
  |   on_tick(): pub   |  |                  |  |                    |
  +--------------------+  +------------------+  +--------------------+
                                 |
                     +-----------+-----------+
                     |   pcl_transport_t     |   pluggable: intra-process,
                     |   (vtable)            |   socket, or ROS2 adapter
                     +-----------------------+
```

---

## Migration Phases

### Phase 1: WorldModelComponent -- ports and on_tick

**Goal:** Move all pub/sub/service wiring from `WorldModelNode` into `WorldModelComponent` using PCL ports.

#### 1.1 Define a serialisation convention

PCL ports exchange `pcl_msg_t` (opaque byte buffer + type name).  Choose a wire format for AME messages.  The simplest option is JSON (consistent with existing JSONL audit logs and PYRAMID's `"application/json"` convention).

Create `subprojects/AME/include/ame/pcl_msg_json.h`:

```cpp
// Helper: serialise/deserialise AME structs to/from pcl_msg_t via JSON.
// Keeps the dependency on nlohmann/json inside ame_core, not in PCL.

pcl_msg_t ame_pack_world_state(const WorldStateSnapshot& snap);
WorldStateSnapshot ame_unpack_world_state(const pcl_msg_t* msg);

pcl_msg_t ame_pack_get_fact_request(const std::string& key);
GetFactResult ame_unpack_get_fact_response(const pcl_msg_t* msg);
// ... etc. for each service pair
```

#### 1.2 Add ports in WorldModelComponent::on_configure

```cpp
pcl_status_t WorldModelComponent::on_configure() {
  // ... existing domain load + audit log setup ...

  // Publisher (periodic world state)
  pub_world_state_ = addPublisher("world_state", "ame/WorldState");

  // Subscriber (perception detections)
  addSubscriber("detections", "ame/Detection",
                onDetectionTrampoline, this);

  // Services
  addService("get_fact",     "ame/GetFact",     handleGetFactCb,     this);
  addService("set_fact",     "ame/SetFact",     handleSetFactCb,     this);
  addService("query_state",  "ame/QueryState",  handleQueryStateCb,  this);
  addService("load_domain",  "ame/LoadDomain",  handleLoadDomainCb,  this);

  return PCL_OK;
}
```

#### 1.3 Move periodic publishing into on_tick

```cpp
pcl_status_t WorldModelComponent::on_tick(double /*dt*/) {
  size_t applied = wm_.applyQueuedMutations();
  if (applied > 0 || consumeStateDirty()) {
    auto snap = buildSnapshot({});
    auto msg = ame_pack_world_state(snap);
    pcl_port_publish(pub_world_state_, &msg);
  }
  return PCL_OK;
}
```

Set tick rate during configure:

```cpp
setTickRateHz(paramF64("publish_rate_hz", 10.0));
```

#### 1.4 Move detection handler to PCL subscriber callback

```cpp
static void onDetectionTrampoline(pcl_container_t*, const pcl_msg_t* msg,
                                   void* user_data) {
  auto* self = static_cast<WorldModelComponent*>(user_data);
  auto detection = ame_unpack_detection(msg);
  self->handleDetection(detection);
}
```

This callback runs on the PCL executor thread, so `wm_.enqueueMutation()` is now single-threaded -- no more cross-thread mutation.

#### 1.5 Move service handlers to PCL service callbacks

```cpp
static pcl_status_t handleGetFactCb(pcl_container_t*,
                                     const pcl_msg_t* request,
                                     pcl_msg_t* response,
                                     pcl_svc_context_t*,
                                     void* user_data) {
  auto* self = static_cast<WorldModelComponent*>(user_data);
  auto req = ame_unpack_get_fact_request(request);
  auto result = self->getFact(req.key);
  *response = ame_pack_get_fact_response(result);
  return PCL_OK;
}
```

#### 1.6 Thin out WorldModelNode

After migration, `WorldModelNode` becomes a thin lifecycle forwarder with no ports of its own:

```cpp
class WorldModelNode : public rclcpp_lifecycle::LifecycleNode {
  ame::WorldModelComponent component_;

  CallbackReturn on_configure(...) override {
    // Forward ROS2 params to PCL params
    component_.setParam("domain.pddl_file", ...);
    return component_.configure() == PCL_OK
           ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
  }
  CallbackReturn on_activate(...) override {
    return component_.activate() == PCL_OK
           ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
  }
  // ... deactivate, cleanup, shutdown same pattern
  // NO publishers, NO subscribers, NO services, NO timers
};
```

The node exists only to bridge ROS2 lifecycle transitions and parameter declaration.  All communication flows through PCL ports.

#### 1.7 Tests

- Existing unit tests (`test_world_model`) should pass unchanged (they test the WorldModel, not the component).
- Add a PCL-level integration test: create an executor, add WorldModelComponent, configure+activate, invoke `get_fact` service via `pcl_executor_invoke_service()`, verify response.
- Verify the detached-thread detection handler no longer exists.

---

### Phase 2: ExecutorComponent -- on_tick and ports

**Goal:** Replace ROS2 wall timers and pub/sub with PCL equivalents.

#### 2.1 Add ports in ExecutorComponent::on_configure

```cpp
pcl_status_t ExecutorComponent::on_configure() {
  pub_bt_events_ = addPublisher("executor/bt_events", "ame/BTEvent");
  pub_status_    = addPublisher("executor/status",    "ame/Status");

  addSubscriber("executor/bt_xml", "ame/BTXML",
                onBTXmlTrampoline, this);

  // ... existing BT factory setup ...
  return PCL_OK;
}
```

#### 2.2 Move BT ticking into on_tick

```cpp
pcl_status_t ExecutorComponent::on_tick(double /*dt*/) {
  if (!executing_) return PCL_OK;

  tickOnce();

  if (last_status_ == BT::NodeStatus::SUCCESS ||
      last_status_ == BT::NodeStatus::FAILURE) {
    haltExecution();
    auto msg = ame_pack_status(last_status_);
    pcl_port_publish(pub_status_, &msg);
  }
  return PCL_OK;
}
```

Set tick rate from parameter:

```cpp
setTickRateHz(paramF64("tick_rate_hz", 50.0));
```

#### 2.3 Replace event sink with PCL publish

Currently `setEventSink()` accepts a `std::function` that the ROS2 node sets to publish on a ROS2 topic.  Replace with:

```cpp
// Inside ExecutorComponent (no external sink needed)
void emitEvent(const std::string& json_line) {
  pcl_msg_t msg;
  msg.data = json_line.data();
  msg.size = static_cast<uint32_t>(json_line.size());
  msg.type_name = "ame/BTEvent";
  pcl_port_publish(pub_bt_events_, &msg);
}
```

#### 2.4 Thin out ExecutorNode

Remove: `tick_timer_`, `sub_bt_xml_`, `pub_bt_events_`, `pub_status_`, `create_wall_timer`, `setEventSink` lambda.  The node becomes lifecycle-only.

#### 2.5 Tests

- Existing BT integration tests should pass unchanged.
- Add PCL integration test: executor with `ExecutorComponent`, post a BT XML message via `pcl_executor_post_incoming()`, spin until status is SUCCESS.

---

### Phase 3: PlannerComponent -- services and async planning

**Goal:** Move service/action handling into PCL while preserving async planning.

#### 3.1 Add ports in PlannerComponent::on_configure

```cpp
pcl_status_t PlannerComponent::on_configure() {
  pub_bt_xml_ = addPublisher("bt_xml", "ame/BTXML");

  addService("load_domain", "ame/LoadDomain",
             handleLoadDomainCb, this);

  addService("plan", "ame/Plan",
             handlePlanCb, this);

  // ... existing setup ...
  return PCL_OK;
}
```

#### 3.2 Replace the ROS2 action server

The current `PlannerNode` uses a ROS2 action server (`rclcpp_action`) with feedback.  PCL does not have a native action concept, but the planning request/response can be modelled as a synchronous PCL service.

**Option A -- Synchronous service (recommended for Phase 3):**  Planning runs on the executor thread inside the service handler.  This blocks the executor during planning but is deterministic and simple.  Acceptable if solve times are bounded (typically <100ms for LAPKT BRFS on AME-scale problems).

```cpp
static pcl_status_t handlePlanCb(pcl_container_t*,
                                  const pcl_msg_t* request,
                                  pcl_msg_t* response,
                                  pcl_svc_context_t*,
                                  void* user_data) {
  auto* self = static_cast<PlannerComponent*>(user_data);
  auto goal = ame_unpack_plan_request(request);
  auto result = self->solveGoal(goal.goal_fluents);

  if (result.success && self->pub_bt_xml_) {
    auto bt_msg = ame_pack_bt_xml(result.bt_xml);
    pcl_port_publish(self->pub_bt_xml_, &bt_msg);
  }

  *response = ame_pack_plan_result(result);
  return PCL_OK;
}
```

**Option B -- Deferred response (future, if needed):**  Use `pcl_svc_context_t*` to defer the response and schedule planning on a background I/O thread, posting the result back via `pcl_executor_post_response_cb()`.  Only needed if solve times become unpredictable (e.g. temporal planning).

#### 3.3 Replace the QueryState client

Currently `PlannerNode` holds a ROS2 service client for distributed-mode world state queries.  Replace with PCL service invocation:

```cpp
WorldStateSnapshot PlannerComponent::queryWorldStateViaService() {
  auto req = ame_pack_query_state_request({});
  pcl_msg_t response;
  pcl_status_t rc = pcl_container_invoke_async(
      handle(), "query_state", &req, onQueryStateResponse, this);
  // ...
}
```

In-process mode continues to use the direct `WorldModel*` pointer (unchanged).

#### 3.4 Eliminate the detached thread

The current `PlannerNode::handleAccepted()` spawns `std::thread(...).detach()`.  With synchronous PCL service handling, planning runs on the executor thread.  No detached thread, no race conditions.

#### 3.5 Thin out PlannerNode

Remove: `action_server_`, `pub_bt_xml_`, `client_query_state_`, `srv_load_domain_`, `handleGoal/handleCancel/handleAccepted/executePlan`.

#### 3.6 Tests

- Verify planning still works end-to-end via PCL service invocation.
- Verify no `std::thread` is spawned anywhere in `PlannerComponent` or `PlannerNode`.
- Benchmark solve time to confirm synchronous execution is acceptable.

---

### Phase 4: AgentDispatcher -- dynamic ports

**Goal:** Replace the callback-injection pattern with PCL ports.

#### 4.1 Current design

`AgentDispatcher` uses injected `std::function` callbacks (`setBTSender`, `setStatusQuery`) for transport.  The ROS2 node creates per-agent publishers/subscribers dynamically.

#### 4.2 PCL port approach

PCL requires port creation during `on_configure()` (immutable topology).  Dynamic per-agent publishers are incompatible with this constraint.

**Options:**

**A. Fixed agent roster (recommended):** Declare agent IDs via parameter at configure time.  Create one publisher per agent:

```cpp
pcl_status_t AgentDispatcher::on_configure() {
  auto agents = parseAgentList(paramStr("agent_ids", ""));
  for (const auto& id : agents) {
    std::string topic = id + "/executor/bt_xml";
    agent_pubs_[id] = addPublisher(topic.c_str(), "ame/BTXML");

    std::string status_topic = id + "/executor/status";
    addSubscriber(status_topic.c_str(), "ame/Status",
                  onAgentStatusTrampoline, this);
  }

  addService("dispatch_goals", "ame/DispatchGoals",
             handleDispatchGoalsCb, this);

  return PCL_OK;
}
```

This is consistent with PCL's immutable-topology design and with the fact that agent deployment is typically known at launch time in mission systems.

**B. Executor-level publish (fallback):** For genuinely dynamic agents, use `pcl_executor_publish()` which publishes to any topic without pre-registered ports.  Less type-safe but supports open topologies.

#### 4.3 Remove callback injection

Delete `setBTSender()` and `setStatusQuery()`.  Replace with direct PCL port publish/subscribe inside the component.

#### 4.4 Thin out AgentDispatcherNode

Remove: `agent_bt_pubs_`, `agent_status_subs_`, `agent_statuses_`, `srv_dispatch_`, `ensureAgentPublisher`, `ensureAgentStatusSubscription`, `sendBTToAgent`, `queryAgentStatus`.

---

### Phase 5: PCL executor entry point

**Goal:** Create a proper PCL executor-based main that can run all components without ROS2.

#### 5.1 New `ame_pcl_main.cpp`

```cpp
#include <pcl/executor.hpp>
#include <ame/world_model_component.h>
#include <ame/planner_component.h>
#include <ame/executor_component.h>
#include <ame/agent_dispatcher.h>

int main() {
  pcl::Executor executor;

  ame::WorldModelComponent wm_comp;
  ame::PlannerComponent    pl_comp;
  ame::ExecutorComponent   ex_comp;
  ame::AgentDispatcher     ad_comp;

  // Wire in-process pointers
  pl_comp.setInProcessWorldModel(&wm_comp.worldModel());
  ex_comp.setInProcessWorldModel(&wm_comp.worldModel());
  // ... etc.

  // Set parameters
  wm_comp.setParam("domain.pddl_file", "logistics.pddl");
  // ...

  // Add to executor
  executor.add(wm_comp);
  executor.add(pl_comp);
  executor.add(ex_comp);
  executor.add(ad_comp);

  // Configure + activate all
  wm_comp.configure(); wm_comp.activate();
  pl_comp.configure(); pl_comp.activate();
  ex_comp.configure(); ex_comp.activate();
  ad_comp.configure(); ad_comp.activate();

  // Deterministic single-threaded spin
  executor.spin();
}
```

#### 5.2 Optional: socket transport for distributed deployment

```cpp
#include <pcl/pcl_transport_socket.h>

auto* transport = pcl_socket_transport_create_server(9100, executor.handle());
executor.setTransport(pcl_socket_transport_get_transport(transport));
executor.spin();
```

This enables remote clients (e.g., a ROS2 bridge, a web UI, or an Ada controller) to interact with the same components over TCP -- no code changes to the components.

#### 5.3 Update combined_main.cpp

Refactor `combined_main.cpp` to use PCL executor internally, with the ROS2 nodes as thin lifecycle shims only.  The ROS2 `SingleThreadedExecutor` manages lifecycle transitions; the PCL executor manages all component ticking and message dispatch.

---

### Phase 6 (future): ROS2 transport adapter

**Goal:** Write a `pcl_transport_t` implementation backed by ROS2 DDS, so PCL ports map to ROS2 topics/services automatically.

This is a larger piece of work and can be done independently.  It would allow the thin ROS2 node wrappers to be eliminated entirely -- components would register PCL ports, and the transport adapter would create corresponding ROS2 publishers/subscribers/services under the hood.

**Sketch:**

```cpp
class Ros2Transport {
  pcl_transport_t vtable_;
  rclcpp::Node::SharedPtr node_;

  static pcl_status_t publish_impl(void* ctx, const char* topic,
                                    const pcl_msg_t* msg) {
    auto* self = static_cast<Ros2Transport*>(ctx);
    // Look up or lazily create a rclcpp publisher for `topic`
    // Wrap msg bytes in std_msgs::msg::UInt8MultiArray or custom msg
    // Publish via ROS2
  }
  // ... subscribe_impl, serve_impl, invoke_async_impl ...
};
```

This is **not required** for the earlier phases to deliver value.  Phases 1-5 give full PCL portability for non-ROS2 deployments (socket transport, in-process, Ada bindings) while keeping ROS2 as a supported deployment option via the thin node wrappers.

---

## Migration Order and Dependencies

```
Phase 1 (WorldModelComponent)
    |
    +---> Phase 2 (ExecutorComponent)     -- depends on msg format from Phase 1
    |         |
    |         +---> Phase 5 (PCL main)    -- needs phases 1-4 complete
    |
    +---> Phase 3 (PlannerComponent)      -- depends on WorldModel ports from Phase 1
    |
    +---> Phase 4 (AgentDispatcher)       -- depends on executor ports from Phase 2

Phase 6 (ROS2 transport adapter)          -- independent, can start any time after Phase 1
```

**Recommended execution order:** 1 -> 2 -> 3 -> 4 -> 5 -> 6

Each phase is independently testable and deployable.  After each phase, the migrated component works with both the PCL executor and the existing ROS2 node wrapper.

---

## Risk Mitigations

| Risk | Mitigation |
|------|-----------|
| JSON serialisation overhead | Profile after Phase 1.  If too slow, switch to flatbuffers or raw memcpy for known-layout structs.  The `pcl_msg_t` interface is format-agnostic. |
| Synchronous planning blocks executor | Benchmark LAPKT BRFS solve times.  If >50ms, use deferred PCL service response (Option B in Phase 3). |
| Fixed agent roster too rigid | Start with parameter-based roster (Phase 4A).  Fall back to `pcl_executor_publish()` if dynamic agents are needed. |
| ROS2 action feedback lost | Replace with periodic status publishes from PlannerComponent via `on_tick()` (status topic).  ROS2 wrapper can subscribe and bridge to action feedback if needed. |
| Breaking existing ROS2 launch files | Node names and lifecycle behaviour are unchanged.  Only internal wiring changes.  ROS2 topics/services continue to exist (published by thin wrapper or future ROS2 transport adapter). |

---

## Files Changed Per Phase

### Phase 1
| File | Change |
|------|--------|
| `include/ame/pcl_msg_json.h` | **New** -- serialisation helpers |
| `src/lib/pcl_msg_json.cpp` | **New** -- serialisation implementation |
| `include/ame/world_model_component.h` | Add port members, `on_tick()` override, static service handlers |
| `src/lib/world_model_component.cpp` | Move pub/sub/service logic from node, add `on_tick()` |
| `ros2/include/ame_ros2/world_model_node.hpp` | Remove all ROS2 pub/sub/service/timer members |
| `ros2/src/nodes/world_model_node.cpp` | Strip to lifecycle forwarding only |
| `tests/` | Add PCL integration test for WorldModelComponent |

### Phase 2
| File | Change |
|------|--------|
| `include/ame/executor_component.h` | Add port members, `on_tick()` override, remove `EventSink` |
| `src/lib/executor_component.cpp` | Move tick/pub/sub logic from node |
| `ros2/include/ame_ros2/executor_node.hpp` | Remove all ROS2 pub/sub/timer members |
| `ros2/src/nodes/executor_node.cpp` | Strip to lifecycle forwarding only |

### Phase 3
| File | Change |
|------|--------|
| `include/ame/planner_component.h` | Add port members, service handler |
| `src/lib/planner_component.cpp` | Move plan/service logic from node, remove thread spawn |
| `ros2/include/ame_ros2/planner_node.hpp` | Remove action server, pub, service client |
| `ros2/src/nodes/planner_node.cpp` | Strip to lifecycle forwarding only |

### Phase 4
| File | Change |
|------|--------|
| `include/ame/agent_dispatcher.h` | Add port members, remove callback injection |
| `src/lib/agent_dispatcher.cpp` | Use PCL ports for dispatch |
| `ros2/include/ame_ros2/agent_dispatcher_node.hpp` | Remove dynamic pubs/subs/service |
| `ros2/src/nodes/agent_dispatcher_node.cpp` | Strip to lifecycle forwarding only |

### Phase 5
| File | Change |
|------|--------|
| `src/apps/ame_pcl_main.cpp` | **New** -- PCL executor entry point |
| `src/CMakeLists.txt` | Add `ame_pcl_main` target |
| `ros2/src/apps/combined_main.cpp` | Refactor to use PCL executor internally |

---

## Definition of Done

- [ ] All four components create their ports in `on_configure()` and use `pcl_port_publish()` for output.
- [ ] No `rclcpp::Publisher`, `rclcpp::Subscription`, `rclcpp::Service`, `rclcpp::Client`, or `rclcpp::TimerBase` exists inside any `*Component` class.
- [ ] No `std::thread` is spawned by any component or node.
- [ ] `ExecutorComponent::on_tick()` drives BT ticking.
- [ ] `WorldModelComponent::on_tick()` drives periodic state publishing.
- [ ] Detection ingress runs on the PCL executor thread (no cross-thread mutation of WorldModel).
- [ ] `ame_pcl_main` runs the full pipeline without ROS2.
- [ ] All 73 existing tests pass.
- [ ] At least one new PCL-level integration test per component.
- [ ] ROS2 `combined_main` still works (thin node wrappers forward lifecycle only).
