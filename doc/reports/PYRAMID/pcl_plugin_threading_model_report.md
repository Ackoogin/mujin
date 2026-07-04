# PCL Plugin Threading Model Report

Status: 2026-07-04.  Updated after the UDP egress fix and shared-memory
worker-owned backpressure fix.

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

The non-conforming paths are:

| Area | Issue | Primary files |
|------|-------|---------------|
| Shared-memory transport | non-backpressured publish, service invoke, stream invoke, stream send/end/cancel, and respond still take the bus lock synchronously; service and stream invoke paths poll discovery. | `subprojects/PCL/src/pcl_transport_shared_memory.c`, `subprojects/PCL/include/pcl/pcl_transport_shared_memory.h` |
| ROS2 coupled plugin | consumed unary/stream calls synchronously wait on ROS2 services and stream completion from the executor call path. | `subprojects/PYRAMID/plugins/pyramid_ros2_coupled_plugin.cpp`, `subprojects/PYRAMID/ros2/src/rclcpp_runtime_adapter.cpp`, `subprojects/PYRAMID/pim/backends/ros2_backend.py` |
| gRPC coupled plugin | client worker shutdown still waits for an in-flight blocking generated gRPC call to return; add bounded/cancellable RPC support before treating plugin unload as fully conformant under a wedged peer. | `subprojects/PYRAMID/plugins/pyramid_grpc_coupled_plugin.cpp`, `subprojects/PYRAMID/pim/backends/grpc_backend.py` |

Resolved paths:

| Area | Resolution | Verification |
|------|------------|--------------|
| UDP transport | `publish` now formats the datagram and enqueues it to a transport-owned send thread; only that worker calls `sendto`.  Shutdown stops egress and future publishes fail with `PCL_ERR_STATE`. | Built `test_pcl_udp_transport`; ran `ctest --test-dir build -C Release -R "PclUdpTransport\\." --output-on-failure` |
| Shared-memory topic backpressure | non-zero topic backpressure configuration is accepted again, but configured publishes copy into a transport-owned egress queue and return promptly.  The egress worker owns the mailbox-capacity wait/retry loop and preserves all-or-nothing fan-out.  Clearing a policy restores the default fail-fast publish path. | Built `test_pcl_shared_memory_transport`; ran `ctest --test-dir build -C Release -R "PclSharedMemoryTransport\\." --output-on-failure` |
| gRPC consumed egress | client-mode `invoke_async` and `invoke_stream` now copy requests into a plugin-owned worker queue and return promptly.  The worker owns blocking generated gRPC calls and posts unary/stream callbacks back through the PCL executor response queue. | Built `pyramid_grpc_coupled_plugin` in `build-all-enabled`; built and ran `ctest --test-dir build-all-enabled -C Release -R "GrpcCoupledPlugin" --output-on-failure`.  `test_grpc_coupled_plugin_e2e` did not build in this tree because generated `pyramid_codec_plugin_test_paths.hpp` is missing. |

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
- Replace the indefinite bus lock with a non-blocking or bounded try-lock path
  only if normal peer polling cannot race it.  A direct try-lock on the shared
  bus is not sufficient because the receive thread legitimately holds the lock
  during local service/subscription synchronization.
- Preferred remaining fix: move shared-memory publish, service invoke, stream
  open, stream frame, and cancel operations into a transport-owned egress queue.
  The worker can block on the bus lock and discovery while the executor only
  copies payloads and enqueues work.
- Move service/stream provider discovery retries to a worker queue, or change
  executor-facing `invoke_async`/`invoke_stream` to fail fast when no provider is
  visible immediately.
- Deliver async responses and stream frames through executor queues, as today.

### 5. Fix ROS2 coupled plugin

Make the ROS2 transport vtable queue work into plugin-owned workers.

- Add an outbound work queue to `Ros2TransportContext`.
- Change `ros2Publish` to copy the envelope and enqueue publish work.  The
  worker calls `publishOutboundEnvelope`.
- Change `ros2InvokeAsync` to allocate pending state, copy the request, enqueue
  a unary call, and return.  The worker calls the ROS2 adapter, then posts the
  response with `pcl_executor_post_response_msg`.
- Change `ros2InvokeStream` to allocate a stream handle and pending state,
  enqueue the open request, and return.  The worker opens the ROS2 stream and
  posts each frame/terminal event back to the executor.
- Do not invoke user callbacks directly from ROS2 spin threads or transport
  workers.
- Keep ROS2 subscription/service ingress helpers posting to executor queues; in
  a separate routing fix, thread peer id through and use the remote-aware post
  APIs for remote ingress.

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
- Remaining: add deadlines/cancellation to generated client calls so plugin
  teardown does not wait indefinitely on a wedged peer.

## Conformance tests

Add a transport conformance suite that can be reused by built-in transports and
runtime plugins.  The goal is not just successful delivery; it must prove
threading behavior.

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
- every egress vtable call is proven to return promptly while the real I/O path
  is blocked
- no transport/plugin invokes user callbacks inline from `invoke_async` or from
  non-executor worker threads
- plugin unload joins all plugin-owned threads before `dlclose`/`FreeLibrary`
- documentation states the contract clearly enough for third-party transport
  plugin authors to implement it without reading socket transport internals
