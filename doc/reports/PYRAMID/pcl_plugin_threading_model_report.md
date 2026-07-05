# PCL Plugin Threading Model Report

Status: 2026-07-05.  All planned rework landed: the shared-memory service/stream
discovery poll moved onto the transport worker, the ROS2 coupled plugin now
queues consumed unary/stream egress onto a worker, the gRPC coupled plugin
cancels in-flight client RPCs at teardown, and a reusable threading conformance
suite (groups A-D) covers the in-tree transports.  See "Current status" and the
Resolved table for what was verified where.

This report records the threading-model review for PCL transports and PYRAMID
runtime plugins.  It focuses on the rule that PCL component business logic runs
only on the PCL executor thread, and that the executor thread must not block on
outgoing data or remote-service waits.  Codec and transport plugins must
therefore queue ingress and egress safely across thread boundaries.

## Required contract

The runtime contract should be made explicit in `pcl/pcl_transport.h`,
`pcl/pcl_executor.h`, and the plugin architecture documentation:

1. Ingress from foreign threads must deep-copy borrowed payloads and post work to
   executor-owned queues.  Component callbacks, service handlers, stream
   callbacks, and response callbacks must run only while the executor drains
   those queues.
2. Egress entry points called by the executor (`publish`, `invoke_async`, and
   `invoke_stream`) must not perform blocking I/O, remote waits, discovery
   polling, or unbounded lock waits.  They may validate, copy payloads, allocate
   correlation state, enqueue work to transport-owned workers, and return.
3. Transport workers may block on sockets, ROS2, gRPC, shared-memory wait
   primitives, or discovery, but completions must return to the executor through
   `pcl_executor_post_response_msg`, `pcl_executor_post_remote_incoming`, or an
   equivalent executor queue.
4. Codec plugin `encode` and `decode` functions remain synchronous marshaling
   functions.  They must not own business logic or call transport stacks.  The
   coupled ROS2/gRPC plugins are transport+codec modules, so the transport half
   must still obey the egress queue rule.

## Current status

The executor ingress model is sound: `pcl_executor_post_incoming`,
`pcl_executor_post_remote_incoming`, `pcl_executor_post_response_msg`, and
`pcl_executor_post_service_request` deep-copy into queues, and
`pcl_executor_spin_once` drains them on the executor thread.

The socket transport and transport-template implementation already follow the
egress model.  `publish` and `invoke_async` clone outbound frames into FIFO
queues, and dedicated send/receive workers own blocking wire operations.

The remaining known residuals are:

| Area | Residual | Notes |
|------|----------|-------|
| Shared-memory transport | `respond`, `stream_send`, `stream_end`, and `stream_cancel` still take the bus lock synchronously on the executor-facing path. | The bus lock is a short, bounded critical section (not a discovery poll or backpressure wait), so this does not block the executor indefinitely; the genuinely-blocking discovery poll was the conformance violation and is now worker-owned.  Moving these off the executor as well would need to serialise them through the FIFO worker and interacts with the delicate server-side stream-target lifecycle (abort-on-destroy) -- deferred as a lower-value hardening. |
| gRPC coupled plugin | wedged-peer teardown is now cancellable but not E2E-verified in this tree. | `test_grpc_coupled_plugin_e2e` still does not build here because generated `pyramid_codec_plugin_test_paths.hpp` is missing; the cancellation mechanism is unit-covered only by the plugin load test. |

Resolved paths:

| Area | Resolution | Verification |
|------|------------|--------------|
| UDP transport | `publish` now formats the datagram and enqueues it to a transport-owned send thread; only that worker calls `sendto`.  Shutdown stops egress and future publishes fail with `PCL_ERR_STATE`. | Built `test_pcl_udp_transport`; ran `ctest --test-dir build -C Release -R "PclUdpTransport\\." --output-on-failure` |
| Shared-memory topic backpressure | non-zero topic backpressure configuration is accepted again, but configured publishes copy into a transport-owned egress queue and return promptly.  The egress worker owns the mailbox-capacity wait/retry loop and preserves all-or-nothing fan-out.  Clearing a policy restores the default fail-fast publish path. | Built `test_pcl_shared_memory_transport`; ran `ctest --test-dir build -C Release -R "PclSharedMemoryTransport\\." --output-on-failure` |
| gRPC consumed egress | client-mode `invoke_async` and `invoke_stream` now copy requests into a plugin-owned worker queue and return promptly.  The worker owns blocking generated gRPC calls and posts unary/stream callbacks back through the PCL executor response queue. | Built `pyramid_grpc_coupled_plugin` in `build-all-enabled`; built and ran `ctest --test-dir build-all-enabled -C Release -R "GrpcCoupledPlugin" --output-on-failure`.  `test_grpc_coupled_plugin_e2e` did not build in this tree because generated `pyramid_codec_plugin_test_paths.hpp` is missing. |
| Shared-memory service/stream discovery | `invoke_async` and `invoke_stream` now register pending state and enqueue an `SVC_REQ`/`STREAM_OPEN` item to the transport-owned egress worker, returning promptly.  The worker owns the (bounded) provider-discovery poll and the request-send bus lock; on discovery failure it delivers a terminal (empty unary response / `NOT_FOUND` stream end) on the executor thread instead of blocking or failing the caller synchronously.  The worker-resolved provider id is recorded on the pending-stream node so a later cancel can address it without racing the caller handle. | `subprojects/PCL/src/pcl_transport_shared_memory.c`.  Built and ran `ctest --test-dir build -C Release -R "PclSharedMemoryTransport\\." --output-on-failure` (33/33), and the full `Pcl` suite (454/454). |
| ROS2 coupled plugin consumed egress | `ros2InvokeAsync`/`ros2InvokeStream` now copy the request and enqueue it to a plugin-owned worker; the worker runs the (bounded, ~5s) `invokeRemoteUnary`/`invokeRemoteStream` calls with a thunk callback that re-posts each result via `pcl_executor_post_response_msg`, so user callbacks run on the executor thread, never inline or on the worker/spin thread.  Teardown stops the worker before the spin thread so an in-flight bounded call can still complete. | `subprojects/PYRAMID/plugins/pyramid_ros2_coupled_plugin.cpp`.  Built the ament package via `scripts/build_ros2_transport.sh`; ran `test_ros2_coupled_plugin_load` (4/4), `test_rclcpp_runtime_adapter` (9/9), `test_ros2_codec_roundtrip` (8/8).  Also fixed a GCC-11 `= {}` aggregate-default-argument miscompile in `rclcpp_runtime_adapter.hpp` and a missing `<rclcpp/serialization.hpp>` include in the adapter test that blocked the whole ROS2 build on this toolchain. |
| gRPC teardown cancellation | the generated aggregator keeps a registry of in-flight client `grpc::ClientContext`s and exposes `pyramid_grpc_plugin_client_cancel_all()`; the plugin calls it in `stopClientWorker` before joining, so `TryCancel()` makes a wedged unary call / stream `Read()` return promptly instead of stranding the worker at unload. | `subprojects/PYRAMID/pim/backends/grpc_backend.py`, `subprojects/PYRAMID/plugins/pyramid_grpc_coupled_plugin.cpp`.  Regenerated bindings via `cmake --preset all-on`; rebuilt `pyramid_grpc_coupled_plugin` and ran `GrpcCoupledPlugin` load tests (4/4).  Wedged-peer behaviour not E2E-verified (see residuals). |
| Threading conformance suite | reusable helpers `expectIngressRunsOnExecutorThread` (group A) and `expectResponseCallbackOnExecutorNotInline` (groups A+C) added to `pcl_transport_conformance.hpp`; a new `test_pcl_transport_threading.cpp` wires them plus gated-send egress-promptness (group B) and destroy-wakes-worker (group D) cases across the template, shared-memory, UDP, and socket transports. | Built and ran `test_pcl_transport_threading` (11/11, stable over repeated runs). |

## Fix plan

### 1. Document and enforce the transport ABI rule

Update public headers and architecture docs before implementation so future
plugins have a single source of truth.

- Add `/// \brief` documentation to `pcl_transport_t` stating that executor
  entry points are non-blocking queue APIs.
- Document that `invoke_async` callbacks are delivered on the executor thread,
  not on transport worker threads and not inline before `invoke_async` returns.
- Document that `invoke_stream` may return a stream handle after queueing the
  open request; frames and terminal status are delivered on the executor thread.
- Clarify that transport workers own blocking I/O, connection setup retries,
  service discovery waits, and backpressure waits.

### 2. Add a reusable egress worker primitive

Avoid duplicating the socket transport's FIFO pattern in each plugin.

- Introduce a small PCL-internal egress queue helper, or factor the existing
  `pcl_transport_template` send-queue pattern into a reusable C helper.
- Queue nodes should own:
  - operation kind: publish, unary request, stream open, stream frame, stream
    cancel, shutdown
  - copied topic/service names
  - copied `pcl_msg_t` payload and type name
  - sequence/correlation id
  - callback and user data
- Worker shutdown should stop accepting new items, wake the worker, drain or
  fail pending calls deterministically, and join before plugin unload.

### 3. Fix UDP egress

Convert UDP from inline `sendto` to queued sending.

- Add a send queue, send event/condition variable, and send thread to
  `pcl_udp_transport_t`.
- Change `udp_publish` to build or clone the datagram and enqueue it only.
- Move `sendto` into the send thread.
- Ensure `pcl_udp_transport_destroy` wakes and joins both receive and send
  workers.
- Preserve current ingress behavior: receive thread posts via
  `pcl_executor_post_remote_incoming`.

### 4. Fix shared-memory egress semantics

Shared memory should not block the executor even though the medium is local.

- Completed: topic backpressure was moved off the executor-facing publish path.
  Non-zero backpressure configuration now queues configured publishes to a
  transport-owned egress worker; the worker owns the bounded mailbox-capacity
  wait/retry loop.  Zero-timeout clearing restores the default fail-fast path.
- Completed: the egress item was generalised (`PCL_SHM_EGRESS_PUBLISH` /
  `SVC_REQ` / `STREAM_OPEN`) and the worker dispatches by kind.  `invoke_async`
  and `invoke_stream` now register pending state, enqueue an item, and return
  promptly; the worker owns the provider-discovery poll and the request-send bus
  lock.  On discovery failure the worker delivers a terminal empty unary
  response / `NOT_FOUND` stream end on the executor thread (async, via
  `pcl_executor_post_response_msg` / the stream trampoline) rather than blocking
  or failing the call synchronously.  The worker-resolved provider id is stored
  on the pending-stream node so `stream_cancel` can address it without racing
  the caller handle.
- Deferred: `respond`, `stream_send`, `stream_end`, and `stream_cancel` still
  take the (bounded) bus lock synchronously.  A try-lock is not sufficient
  because the receive thread legitimately holds the lock during local
  service/subscription synchronization; routing these through the FIFO worker
  interacts with the server-side stream-target lifecycle and is a lower-value
  hardening (see residuals).
- Async responses and stream frames continue to be delivered through executor
  queues, as before.

### 5. Fix ROS2 coupled plugin

Make the ROS2 transport vtable queue work into plugin-owned workers.

- Completed: `Ros2TransportContext` owns a consumed-side work queue and worker
  thread (`ros2WorkerMain`), mirroring the gRPC coupled plugin.
- Completed: `ros2InvokeAsync` copies the request (and its content type) into a
  unary work item, enqueues it, and returns.  The worker runs the bounded
  `invokeRemoteUnary` with a thunk that re-posts the response via
  `pcl_executor_post_response_msg`; on a bounded ROS2 failure it still posts a
  terminal empty response so the client callback fires on the executor thread.
- Completed: `ros2InvokeStream` copies the request, enqueues a stream work item,
  and returns.  The worker runs `invokeRemoteStream` with a thunk that re-posts
  each frame (and a synthesised terminal if the helper returns without one)
  through the executor stream-event trampoline.
- Completed: user callbacks are never invoked from the ROS2 spin thread or the
  worker -- everything is re-posted to the executor.  Teardown stops the worker
  before the spin thread so an in-flight bounded call can complete first.
- Not queued: `ros2Publish` remains direct because rclcpp publish is a
  non-blocking DDS enqueue (no remote wait/discovery/backpressure on the
  executor path).
- Unchanged: ROS2 subscription/service ingress helpers still post to executor
  queues; threading peer id through for remote ingress remains a separate
  routing fix.
- Toolchain fix required to build the ament package on GCC 11: rewrote the
  `RclcppRuntimeAdapter` `Options options = {}` default argument as two
  overloads (GCC 11 miscompiles `= {}` for an aggregate with a default member
  initializer; fixed in GCC 12) and added the missing `<rclcpp/serialization.hpp>`
  include to the adapter test.

### 6. Fix gRPC coupled plugin

Make the gRPC consumed-side vtable asynchronous from the executor's point of
view.

- Completed: `GrpcTransportContext` owns a client work queue and worker thread.
- Completed: `grpcClientInvokeAsync` copies request bytes, enqueues unary work,
  and returns without invoking the callback inline.
- Completed: the worker calls `pyramid_grpc_plugin_client_invoke_unary`, wraps
  the returned bytes in a `pcl_msg_t`, and posts completion with
  `pcl_executor_post_response_msg`.
- Completed: `grpcClientInvokeStream` enqueues stream work; the worker reads the
  blocking gRPC stream and posts each item/terminal event to the executor.
- Generated server ingress may continue to block gRPC handler threads while it
  waits for executor service completion, because those are not executor
  threads.  Add bounded timeout/failure behavior so a stopped executor does not
  strand gRPC server threads indefinitely.
- Completed: the generated aggregator now registers each in-flight client
  `grpc::ClientContext` (`ClientContextGuard`) and exposes
  `pyramid_grpc_plugin_client_cancel_all()`.  The plugin calls it in
  `stopClientWorker` before joining, so `TryCancel()` unblocks a wedged unary
  call or stream `Read()` at teardown -- no artificial per-call deadline, so
  legitimate long-lived streams keep working.  Behaviour is unit-covered by the
  plugin load test; the wedged-peer E2E remains unbuildable in this tree.

## Conformance tests

Add a transport conformance suite that can be reused by built-in transports and
runtime plugins.  The goal is not just successful delivery; it must prove
threading behavior.

Status (2026-07-05): implemented for the in-tree transports as
`subprojects/PCL/tests/test_pcl_transport_threading.cpp` (suite
`PclTransportThreading`), backed by two reusable helpers in
`pcl_transport_conformance.hpp`:
`expectIngressRunsOnExecutorThread` (group A) and
`expectResponseCallbackOnExecutorNotInline` (groups A+C).  The file records the
executor/spin thread id and asserts every business-logic callback runs on it,
uses a gated template send hook to prove egress returns promptly while the send
path is wedged (group B), and drives destroy against a blocked send worker
(group D).  Coverage: template (A/B/C/D), shared memory (A/C + backpressure B),
UDP (A), socket (A/C).  11/11 passing.  The `PyramidPluginThreading.Ros2*` /
`Grpc*` cases named below still need the all-enabled / ament build harness and
remain to be written as plugin-level tests; the ROS2 and gRPC plugins already
satisfy the underlying contract (see the Resolved table).

### Test group A: executor-thread ownership

1. Create a test component whose subscriber, service handler, response callback,
   and stream callback record `std::this_thread::get_id()`.
2. Drive ingress from a foreign worker thread through each transport.
3. Spin the PCL executor on a known thread.
4. Assert every business-logic callback ran on the executor thread and never on
   the transport worker, ROS2 spin thread, gRPC handler thread, or test helper
   thread.

Expected tests:

- `PclTransportThreading.SocketIngressRunsOnExecutor`
- `PclTransportThreading.SharedMemoryIngressRunsOnExecutor`
- `PclTransportThreading.UdpIngressRunsOnExecutor`
- `PyramidPluginThreading.Ros2IngressRunsOnExecutor`
- `PyramidPluginThreading.GrpcIngressRunsOnExecutor`

### Test group B: egress returns promptly

1. Install a transport or plugin whose real send path blocks behind a latch.
2. Call `pcl_executor_publish`, generated facade publish, `invoke_async`, and
   `invoke_stream` from executor-owned callbacks.
3. Assert the call returns within a tight budget such as 10-25 ms while the
   worker remains blocked.
4. Release the latch and assert the queued operation completes normally.

Expected tests:

- `PclTransportThreading.SocketPublishQueuesWhenSendBlocks`
- `PclTransportThreading.TemplateInvokeQueuesWhenSendBlocks`
- `PclTransportThreading.UdpPublishQueuesWhenSendBlocks`
- `PclTransportThreading.SharedMemoryPublishDoesNotSleepForBackpressure`
- `PyramidPluginThreading.Ros2InvokeAsyncDoesNotWaitForRemoteService`
- `PyramidPluginThreading.Ros2InvokeStreamDoesNotWaitForStreamCompletion`
- `PyramidPluginThreading.GrpcInvokeAsyncDoesNotBlockOnUnaryRpc`
- `PyramidPluginThreading.GrpcInvokeStreamDoesNotBlockOnReader`

### Test group C: callbacks are not inline

`invoke_async` returning after firing the callback inline violates the queue
contract because user code re-enters business logic before the executor drains a
completion queue.

1. Call `pcl_executor_invoke_async` from a component tick or service handler.
2. Record whether the response callback fires before `invoke_async` returns.
3. Assert the callback has not fired inline.
4. Spin the executor again and assert the callback fires on that spin.

Expected tests:

- `PclTransportThreading.SocketResponseCallbackNotInline`
- `PclTransportThreading.SharedMemoryResponseCallbackNotInline`
- `PyramidPluginThreading.Ros2ResponseCallbackNotInline`
- `PyramidPluginThreading.GrpcResponseCallbackNotInline`

### Test group D: shutdown and unload safety

1. Queue publish, unary, and stream work while the worker is blocked.
2. Destroy or unload the transport plugin.
3. Assert worker threads are woken and joined before the plugin is unloaded.
4. Assert pending callbacks receive a terminal error on the executor thread, or
   are explicitly cancelled before executor destruction.

Expected tests:

- `PclTransportThreading.UdpDestroyWakesBlockedSendWorker`
- `PclTransportThreading.SharedMemoryDestroyFailsPendingQueuedWork`
- `PyramidPluginThreading.Ros2PluginUnloadJoinsWorkers`
- `PyramidPluginThreading.GrpcPluginUnloadJoinsWorkers`

## Suggested implementation order

1. Add the conformance harness using the existing socket/template transports as
   the first passing examples.
2. Add failing coverage for UDP inline `sendto`.
3. Implement the UDP send worker and make the UDP tests pass.
4. Complete shared-memory by adding a worker-owned egress queue for the
   remaining bus-lock and discovery waits.
5. Add test doubles for ROS2/gRPC adapters that can block deterministically
   without requiring network timing.
6. Refactor the ROS2 coupled plugin around worker queues and add gRPC
   cancellation/deadline support for unload safety.
7. Run the plugin E2E matrix again:
   - `test_pcl_udp_transport`
   - `test_pcl_shared_memory_transport`
   - `test_pcl_socket_transport`
   - `test_pcl_template_transport`
   - `test_grpc_coupled_plugin_e2e`
   - `test_ros2_coupled_plugin_load`
   - Tactical Objects socket/shared-memory/gRPC/ROS2 E2E tests where available.

## Acceptance criteria

The threading model is conformant when:

- every non-test transport/plugin passes the executor-thread ownership suite
  -- met for the in-tree transports (template, shared memory, UDP, socket) by
  `PclTransportThreading`; the ROS2 and gRPC plugins satisfy the contract in
  code but still lack a plugin-level `PyramidPluginThreading` harness.
- every egress vtable call is proven to return promptly while the real I/O path
  is blocked -- met for the transports with an injectable block (template gated
  send, shared-memory backpressure); the plugin cases are pending the harness.
- no transport/plugin invokes user callbacks inline from `invoke_async` or from
  non-executor worker threads -- met; the shared-memory, ROS2, and gRPC consumed
  paths now re-post every callback through the executor queue.
- plugin unload joins all plugin-owned threads before `dlclose`/`FreeLibrary`
  -- met; the ROS2 worker joins before the spin thread, and the gRPC worker is
  cancelled (`TryCancel`) before join so a wedged peer cannot block unload.
- documentation states the contract clearly enough for third-party transport
  plugin authors to implement it without reading socket transport internals
  -- met by the `pcl_transport_t` vtable docs.

Remaining before "fully conformant": author the `PyramidPluginThreading.Ros2*`
and `Grpc*` cases against the all-enabled/ament harness, restore the gRPC E2E
build (missing `pyramid_codec_plugin_test_paths.hpp`), and -- if desired --
route the shared-memory `respond`/`stream_send`/`stream_end`/`stream_cancel`
bus-lock operations through the egress worker.
