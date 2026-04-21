# AME ROS2 Node PCL Migration Status

## Current Status (2026-04-21)

The AME component-layer migration to PCL is complete. The remaining work is no
longer the original port migration; it is the ROS2 wrapper compliance gap left
by the temporary devenv reconnection bridges.

## Completed

All four AME components now own their PCL communication surface:

| Component | Completed PCL surface | Notes |
|-----------|-----------------------|-------|
| `WorldModelComponent` | `pub world_state`, `sub detections`, `svc get_fact / set_fact / query_state / load_domain`, `on_tick()` state publishing | Uses queued perception mutations and applies them from the PCL tick path |
| `PlannerComponent` | `pub bt_xml`, `svc plan / load_domain` | Planning service runs synchronously on the PCL executor thread |
| `ExecutorComponent` | `sub executor/bt_xml`, `pub executor/bt_events / executor/status`, `on_tick()` BT ticking | PCL subscriber path loads BT XML and ticks to terminal status |
| `AgentDispatcher` | Fixed-roster per-agent BT XML/status ports plus `svc dispatch_goals` | ROS2 wrapper is already lifecycle/parameter forwarding only |

The non-ROS2 PCL path is also in place:

- `subprojects/AME/src/apps/ame_pcl_main.cpp` runs the AME pipeline without
  ROS2.
- `subprojects/AME/ros2/src/apps/combined_main.cpp` runs the PCL executor in a
  background thread alongside ROS2 lifecycle management.
- `subprojects/AME/include/ame/pcl_msg_json.h` and
  `subprojects/AME/src/lib/pcl_msg_json.cpp` provide the current AME PCL JSON
  message convention.
- `subprojects/AME/tests/test_pcl_integration.cpp` covers PCL service,
  subscriber, and tick paths for WorldModel, Planner, Executor, and dispatch
  message helpers.

The original migration phases for component ports, `on_tick()` dispatch,
`ame_pcl_main`, and AgentDispatcher fixed-roster ports are therefore closed and
should not be tracked as remaining work.

## Remaining Problem

The ROS2 nodes still expose direct ROS2 compatibility bridges for the Python
devenv and other ROS2-only callers. Those bridges preserve usability, but they
bypass PCL ingress and keep part of the system outside the executor's
single-threaded dispatch model.

| Node | Current bypass | Risk |
|------|----------------|------|
| `WorldModelNode` | ROS2 services call `component_->getFact()`, `setFact()`, `queryState()`, and `loadDomainFromStrings()` directly | `setFact()` is mutex-protected but still bypasses the PCL mutation queue/single-writer model |
| `WorldModelNode` | `setWmPublishCallback()` republishes PCL snapshots to ROS2 | Outbound-only bridge; acceptable until a ROS2 PCL transport exists |
| `PlannerNode` | ROS2 `~/plan` action calls `component_->solveGoal()` from a detached `std::thread` | Keeps the pre-migration detached-thread model and bypasses the PCL `plan` service |
| `PlannerNode` | ROS2 `~/load_domain` calls `component_->loadDomainFromStrings()` directly | Bypasses the PCL `load_domain` service |
| `ExecutorNode` | ROS2 `~/load_bt` calls `component_->loadAndExecute()` directly | Data race risk with PCL `on_tick()`/subscriber execution; `exec_mutex_` does not cover the component tick path |
| `ExecutorNode` | ROS2 `~/stop_execution` calls `component_->haltExecution()` directly | Same race risk as `load_bt` |
| `ExecutorNode` | `setEventSink()` republishes BT events/status to ROS2 | Outbound-only bridge; acceptable until a ROS2 PCL transport exists |
| `AgentDispatcherNode` | None observed | Wrapper is already thin |

## Architectural Target

ROS2 should reach AME through a PCL transport adapter, not by calling component
methods from ROS2 callbacks:

```text
ROS2 caller / devenv
  -> ROS2 PCL transport adapter
  -> PCL service/topic ingress
  -> pcl::Executor
  -> AME components
```

PYRAMID now has reusable groundwork for this mapping under:

- `subprojects/PYRAMID/bindings/cpp/generated/ros2/cpp`
- `subprojects/PYRAMID/ros2`
- `subprojects/PYRAMID/doc/architecture/ros2_transport_semantics.md`

That work defines canonical topic/service/stream naming, a generic ROS2
envelope package, and executor-thread ingress rules. AME still needs to adopt a
ROS2 PCL transport path for its current PCL ports before the ROS2 wrappers can
be reduced to pure lifecycle/parameter shims.

## Remaining Plan

### 1. Fix ExecutorNode Ingress Race

Add a pending-command queue to `ExecutorComponent` and make ROS2 service
handlers enqueue commands instead of calling `loadAndExecute()` or
`haltExecution()` directly.

Required changes:

- Add `enqueueBtXml()` and `enqueueHalt()` to `ExecutorComponent`.
- Drain pending commands at the start of `ExecutorComponent::on_tick()`.
- Change `ExecutorNode` `~/load_bt` and `~/stop_execution` handlers to call the
  enqueue methods.
- Remove `ExecutorNode::exec_mutex_` once direct component mutation is gone.
- Add a focused test that proves service-thread enqueue plus PCL ticking is
  serialized through `on_tick()`.

This is the urgent correctness fix because the current direct executor calls
can overlap with BT ticking on the PCL executor thread.

### 2. Route ROS2 World-Model Writes Through PCL Ingress

`WorldModelComponent::setFact()` currently calls `wm_.setFact()` directly under
`wm_mutex_`. That is data-race resistant, but it still sidesteps the
single-writer executor model.

Required changes:

- Introduce an enqueue path for external fact writes or route the ROS2 service
  through the PCL `set_fact` service.
- Apply queued external writes from `on_tick()`.
- Keep direct read-only methods (`getFact()`, `queryState()`) only where they
  are intentionally snapshot reads protected by `wm_mutex_`.

### 3. Remove PlannerNode Detached Planning Thread

`PlannerComponent` already exposes a PCL `plan` service. The ROS2 action bridge
should stop invoking `solveGoal()` directly from a detached thread.

Acceptable implementations:

- Preferred: route the ROS2 action goal to the PCL `plan` service and complete
  the action from the PCL response.
- Future option: use a ROS2 PCL transport adapter so the action bridge becomes
  a small compatibility layer over the generated/PCL service path.

### 4. Add AME ROS2 PCL Transport

Implement or adopt a `pcl_transport_t` backed by ROS2 DDS so AME PCL ports
surface as ROS2 topics/services automatically.

Once this exists, remove from the ROS2 node layer:

- `svc_*` service servers in `WorldModelNode`, `PlannerNode`, and
  `ExecutorNode`
- `PlannerNode` action-to-direct-call implementation
- `WorldModelNode::setWmPublishCallback()` ROS2 publisher bridge
- `ExecutorNode::setEventSink()` ROS2 publisher bridge
- ROS2 publisher members used only to mirror PCL output

The final wrapper target is lifecycle and parameter forwarding only.

### 5. Keep Compatibility Tests Focused

Retain the existing PCL integration coverage and add tests only for remaining
risks:

- Executor pending-command queue serialization
- ROS2 compatibility bridge over PCL ingress
- AME ROS2 PCL transport service/topic mapping once implemented
- `combined_main` smoke test after direct-call bridges are removed

## Definition Of Done

- [x] All AME components create their PCL ports in `on_configure()`.
- [x] Component output uses PCL publishers.
- [x] `WorldModelComponent::on_tick()` publishes state and drains queued
  perception mutations.
- [x] `ExecutorComponent::on_tick()` drives BT execution.
- [x] `PlannerComponent` exposes synchronous PCL planning and load-domain
  services.
- [x] `AgentDispatcher` uses fixed-roster PCL ports and a PCL dispatch service.
- [x] `ame_pcl_main` runs the pipeline without ROS2.
- [x] PCL-level integration tests exist.
- [ ] `ExecutorNode` no longer calls `loadAndExecute()` / `haltExecution()`
  directly from ROS2 service callbacks.
- [ ] ROS2 world-model writes preserve the PCL single-writer model.
- [ ] `PlannerNode` no longer uses a detached thread for direct planning calls.
- [ ] AME has a ROS2 PCL transport path for its PCL ports.
- [ ] Direct ROS2 bypass services/publishers are removed or reduced to thin
  compatibility shims over PCL ingress.
