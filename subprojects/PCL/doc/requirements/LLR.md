# PCL Low-Level Requirements

Low-level requirements for the PYRAMID Composition Library (PCL), derived from the high-level requirements in `HLR.md`.

Each LLR maps to at least one unit or robustness test. Tests use requirement tags in the form `///< REQ_PCL_NNN: ...`.

---

## 1. Container Creation and Destruction

### REQ_PCL_001 - Container Creation Returns Valid Handle
`pcl_container_create()` shall return a non-NULL handle in the UNCONFIGURED state when given a valid name.

**Traces**: PCL.001, PCL.005

**Verification**: `test_pcl_lifecycle.cpp::CreateDestroy` creates a container and asserts non-NULL handle, UNCONFIGURED state, and correct name.

### REQ_PCL_002 - Container Creation With NULL Name Returns NULL
`pcl_container_create()` shall return NULL when the name argument is NULL.

**Traces**: PCL.045

**Verification**: `test_pcl_lifecycle.cpp::NullHandlesReturnError` passes NULL name and asserts NULL return.

### REQ_PCL_003 - Container Destroy Frees Resources
`pcl_container_destroy()` shall free all resources associated with the container, including parameter storage and ports.

**Traces**: PCL.001

**Verification**: `test_pcl_lifecycle.cpp::CreateDestroy` and all tests that call `pcl_container_destroy()` verify no memory leaks (via test harness).

---

## 2. Lifecycle Transitions

### REQ_PCL_004 - Full Lifecycle Cycle
The container shall support the full transition cycle: UNCONFIGURED -> CONFIGURED -> ACTIVE -> CONFIGURED -> UNCONFIGURED -> FINALIZED, invoking the corresponding callback at each step.

**Traces**: PCL.002, PCL.003

**Verification**: `test_pcl_lifecycle.cpp::FullTransitionCycle` exercises every valid transition and verifies callback invocation counts and resulting states.

### REQ_PCL_005 - Invalid Transitions Rejected
The container shall reject invalid transitions with `PCL_ERR_STATE` and remain in the current state.

**Traces**: PCL.002

**Verification**: `test_pcl_lifecycle.cpp::InvalidTransitionsRejected` attempts activate from UNCONFIGURED, deactivate from UNCONFIGURED, cleanup from UNCONFIGURED, configure from CONFIGURED, and deactivate from CONFIGURED.

### REQ_PCL_006 - Shutdown From Any State
`pcl_container_shutdown()` shall succeed from any state except FINALIZED, transitioning to FINALIZED.

**Traces**: PCL.002

**Verification**: `test_pcl_lifecycle.cpp::ShutdownFromAnyState` shuts down from ACTIVE and verifies FINALIZED, then verifies shutdown from FINALIZED returns ERR_STATE.

### REQ_PCL_007 - Callback Failure Aborts Transition
If a lifecycle callback returns non-OK, the transition shall be aborted and the container shall remain in its previous state.

**Traces**: PCL.004

**Verification**: `test_pcl_lifecycle.cpp::CallbackFailureAbortsTransition` supplies an on_configure that returns ERR_CALLBACK and verifies the container remains UNCONFIGURED.

### REQ_PCL_008 - Null Callbacks Are No-Ops
When callbacks are NULL, lifecycle transitions shall succeed as if the callback returned PCL_OK.

**Traces**: PCL.003

**Verification**: `test_pcl_lifecycle.cpp::NullCallbacksAreNoOp` creates a container with no callbacks and successfully completes all transitions.

### REQ_PCL_009 - Deactivate Callback Failure Keeps Active State
If `on_deactivate` returns non-OK, the container shall remain ACTIVE.

**Traces**: PCL.004

**Verification**: `test_pcl_robustness.cpp::DeactivateCallbackFailureKeepsActiveState`.

### REQ_PCL_010 - Cleanup Callback Failure Keeps Configured State
If `on_cleanup` returns non-OK, the container shall remain CONFIGURED.

**Traces**: PCL.004

**Verification**: `test_pcl_robustness.cpp::CleanupCallbackFailureKeepsConfiguredState`.

### REQ_PCL_011 - Activate Callback Failure Keeps Configured State
If `on_activate` returns non-OK, the container shall remain CONFIGURED.

**Traces**: PCL.004

**Verification**: `test_pcl_robustness.cpp::ActivateCallbackFailureKeepsConfiguredState`.

### REQ_PCL_012 - Cleanup Resets Port Count
`pcl_container_cleanup()` shall reset the port count to zero, allowing ports to be re-created in a subsequent configure cycle.

**Traces**: PCL.007

**Verification**: `test_pcl_robustness.cpp::CleanupResetsPortCount`.

---

## 3. Parameters

### REQ_PCL_013 - String Parameter Round-Trip
String parameters shall be stored and retrieved with exact fidelity. Missing keys shall return the caller-supplied default.

**Traces**: PCL.012, PCL.014

**Verification**: `test_pcl_lifecycle.cpp::StringRoundTrip` sets, gets, and tests default.

### REQ_PCL_014 - Numeric and Boolean Parameter Types
Double, int64, and boolean parameters shall be stored and retrieved correctly.

**Traces**: PCL.012

**Verification**: `test_pcl_lifecycle.cpp::NumericTypes` sets and gets f64, i64, and bool parameters.

### REQ_PCL_015 - Parameter Overwrite
Setting a parameter with an existing key shall overwrite the previous value.

**Traces**: PCL.012

**Verification**: `test_pcl_lifecycle.cpp::OverwriteExisting` sets a key twice and verifies the latest value.

### REQ_PCL_016 - Parameter Overflow
Exceeding `PCL_MAX_PARAMS` (128) shall return `PCL_ERR_NOMEM` for all parameter types.

**Traces**: PCL.013, PCL.046

**Verification**: `test_pcl_robustness.cpp::ParamOverflowReturnsNomem` fills 128 slots and verifies the 129th returns ERR_NOMEM for str, f64, i64, and bool.

### REQ_PCL_017 - Parameter Type Mismatch Returns Default
Retrieving a parameter with a different type than stored shall return the default value.

**Traces**: PCL.014

**Verification**: `test_pcl_robustness.cpp::ParamTypeMismatchReturnsDefault` stores as f64, retrieves as str/i64/bool, and vice versa.

### REQ_PCL_018 - Parameter Null Argument Safety
Parameter setters and getters shall handle NULL container, NULL key, and NULL value arguments gracefully.

**Traces**: PCL.045

**Verification**: `test_pcl_robustness.cpp::ParamNullArgs` and `ParamGetNullArgs`.

---

## 4. Port Creation

### REQ_PCL_019 - Port Creation During Configure
Ports created during `on_configure` shall succeed and return non-NULL handles.

**Traces**: PCL.006, PCL.007

**Verification**: `test_pcl_lifecycle.cpp::CreatedDuringConfigure` adds a publisher in on_configure and verifies success.

### REQ_PCL_020 - Port Creation Rejected Outside Configure
Port creation outside `on_configure` shall fail and return NULL.

**Traces**: PCL.007

**Verification**: `test_pcl_lifecycle.cpp::RejectedOutsideConfigure` attempts to add a publisher before configure.

### REQ_PCL_021 - Port Overflow Returns NULL
Exceeding `PCL_MAX_PORTS` (64) shall return NULL for publisher, subscriber, and service ports.

**Traces**: PCL.008, PCL.046

**Verification**: `test_pcl_robustness.cpp::PortOverflowReturnNull` fills 64 ports and verifies the 65th returns NULL for all three port types.

### REQ_PCL_022 - Port Creation Null Arguments
Port creation with NULL mandatory arguments (topic, type, callback, handler) shall return NULL.

**Traces**: PCL.006, PCL.045

**Verification**: `test_pcl_robustness.cpp::PortCreationNullArgs` tests all NULL combinations for publisher, subscriber, and service.

---

## 5. Publishing

### REQ_PCL_023 - Publish On Active Container Succeeds
`pcl_port_publish()` shall succeed when the container is ACTIVE.

**Traces**: PCL.009

**Verification**: `test_pcl_robustness.cpp::PublishSuccessOnActiveContainer`.

### REQ_PCL_024 - Publish On Inactive Container Returns Port Closed
`pcl_port_publish()` shall return `PCL_ERR_PORT_CLOSED` when the container is not ACTIVE.

**Traces**: PCL.009

**Verification**: `test_pcl_robustness.cpp::PublishOnInactiveContainerReturnsClosed`.

### REQ_PCL_025 - Publish Null Safety
`pcl_port_publish()` shall return `PCL_ERR_INVALID` when the port or message is NULL.

**Traces**: PCL.045

**Verification**: `test_pcl_robustness.cpp::PublishNullPortOrMsg`.

### REQ_PCL_026 - Publish On Wrong Port Type Returns Invalid
Publishing on a subscriber port shall return `PCL_ERR_INVALID`.

**Traces**: PCL.009

**Verification**: `test_pcl_robustness.cpp::PublishOnSubscriberPortReturnsInvalid`.

### REQ_PCL_027 - Publish With Executor Routes To Subscribers
When a container publishes and is managed by an executor, the message shall be routed to matching subscribers.

**Traces**: PCL.009, PCL.022

**Verification**: `test_pcl_robustness.cpp::PublishWithExecutorRoutes`.

---

## 6. Tick Rate

### REQ_PCL_028 - Default Tick Rate Is 100 Hz
A newly created container shall have a default tick rate of 100 Hz.

**Traces**: PCL.015

**Verification**: `test_pcl_lifecycle.cpp::DefaultIs100Hz`.

### REQ_PCL_029 - Tick Rate Set And Get
`pcl_container_set_tick_rate_hz()` shall update the rate and `pcl_container_get_tick_rate_hz()` shall return it.

**Traces**: PCL.015

**Verification**: `test_pcl_lifecycle.cpp::SetAndGet` sets 50 Hz and verifies.

### REQ_PCL_030 - Invalid Tick Rate Rejected
Zero and negative tick rates shall be rejected with `PCL_ERR_INVALID`.

**Traces**: PCL.016

**Verification**: `test_pcl_lifecycle.cpp::SetAndGet` verifies -1.0 and 0.0 return ERR_INVALID.

---

## 7. Executor Core

### REQ_PCL_031 - Executor Create And Destroy
`pcl_executor_create()` shall return a non-NULL handle. `pcl_executor_destroy()` shall free resources.

**Traces**: PCL.018

**Verification**: `test_pcl_executor.cpp::CreateDestroy`.

### REQ_PCL_032 - Spin Once Ticks Active Container
`pcl_executor_spin_once()` shall invoke `on_tick()` for each ACTIVE container whose tick period has elapsed.

**Traces**: PCL.018, PCL.019

**Verification**: `test_pcl_executor.cpp::SpinOnceTicksActiveContainer` runs 10 spin_once calls and verifies tick_count > 0.

### REQ_PCL_033 - Inactive Container Not Ticked
Containers not in the ACTIVE state shall not have their `on_tick()` invoked by the executor.

**Traces**: PCL.018

**Verification**: `test_pcl_executor.cpp::InactiveContainerNotTicked`.

### REQ_PCL_034 - Multiple Containers Ticked
The executor shall tick multiple containers at their individual rates.

**Traces**: PCL.018

**Verification**: `test_pcl_executor.cpp::MultipleContainers` adds two containers at 10 Hz and 50 Hz and verifies both are ticked.

### REQ_PCL_035 - Request Shutdown Stops Spin
`pcl_executor_request_shutdown()` shall cause `pcl_executor_spin()` to return.

**Traces**: PCL.020

**Verification**: `test_pcl_executor.cpp::RequestShutdownStopsSpin` spins in a background thread and requests shutdown.

### REQ_PCL_036 - Graceful Shutdown Finalizes Containers
`pcl_executor_shutdown_graceful()` shall transition each ACTIVE container to FINALIZED.

**Traces**: PCL.021

**Verification**: `test_pcl_executor.cpp::GracefulShutdownFinalizesContainers` verifies deactivate is called and state becomes FINALIZED.

### REQ_PCL_037 - Container Overflow Returns NOMEM
Adding more than the executor's container capacity shall return `PCL_ERR_NOMEM`.

**Traces**: PCL.024, PCL.046

**Verification**: `test_pcl_robustness.cpp::ContainerOverflowReturnsNomem`.

### REQ_PCL_038 - Add Null Container Returns Invalid
`pcl_executor_add(e, NULL)` shall return `PCL_ERR_INVALID`.

**Traces**: PCL.045

**Verification**: `test_pcl_robustness.cpp::AddNullContainerReturnsInvalid`.

### REQ_PCL_039 - Remove Container
`pcl_executor_remove()` shall remove a container from the executor without destroying it.

**Traces**: PCL.024

**Verification**: `test_pcl_robustness.cpp::RemoveContainerSuccess`.

### REQ_PCL_040 - Invoke Service Found
`pcl_executor_invoke_service()` shall dispatch to the registered handler and return PCL_OK.

**Traces**: PCL.011, PCL.023

**Verification**: `test_pcl_robustness.cpp::InvokeServiceFound`.

### REQ_PCL_041 - Graceful Shutdown Timeout
If a container's deactivate exceeds the timeout, graceful shutdown shall return `PCL_ERR_TIMEOUT`.

**Traces**: PCL.021

**Verification**: `test_pcl_robustness.cpp::GracefulShutdownTimeout`.

### REQ_PCL_042 - Graceful Shutdown Already Finalized
`pcl_executor_shutdown_graceful()` shall skip containers already in FINALIZED state.

**Traces**: PCL.021

**Verification**: `test_pcl_robustness.cpp::GracefulShutdownAlreadyFinalized`.

### REQ_PCL_043 - Spin Once First Call Uses Default Delta
The first `spin_once()` call shall use a reasonable default delta time.

**Traces**: PCL.017

**Verification**: `test_pcl_robustness.cpp::SpinOnceFirstCallUsesDefaultDt`.

---

## 8. Intra-Process Dispatch

### REQ_PCL_044 - Intra-Process Pub/Sub
`pcl_executor_dispatch_incoming()` shall deliver a message to the subscriber callback matching the topic.

**Traces**: PCL.010, PCL.022, PCL.030

**Verification**: `test_pcl_executor.cpp::IntraProcessPubSub` dispatches a message and verifies receipt.

### REQ_PCL_045 - Dispatch To Unknown Topic Returns Not Found
Dispatching to a topic with no subscriber shall return `PCL_ERR_NOT_FOUND`.

**Traces**: PCL.022

**Verification**: `test_pcl_executor.cpp::DispatchToUnknownTopicReturnsNotFound`.

### REQ_PCL_046 - Dispatch Incoming Null Arguments
Null executor or message arguments shall return `PCL_ERR_INVALID`.

**Traces**: PCL.045

**Verification**: `test_pcl_robustness.cpp::DispatchIncomingNullArgs`.

### REQ_PCL_047 - Publish No Transport Dispatches To Subscriber
`pcl_executor_publish()` without a transport adapter shall dispatch to matching subscribers.

**Traces**: PCL.022

**Verification**: `test_pcl_robustness.cpp::PublishNoTransportDispatchesToSubscriber`.

### REQ_PCL_048 - Publish With Transport Calls Transport Publish
`pcl_executor_publish()` with a transport adapter shall call the adapter's publish function.

**Traces**: PCL.028, PCL.029

**Verification**: `test_pcl_robustness.cpp::PublishWithTransportCallsTransportPublish`.

---

## 9. Cross-Thread Ingress

### REQ_PCL_049 - External Thread Posts Copied Message
`pcl_executor_post_incoming()` shall deep-copy the message so the producer may release buffers immediately. The callback shall execute on the executor thread.

**Traces**: PCL.025

**Verification**: `test_pcl_executor.cpp::ExternalThreadPostsCopiedMessageToExecutorThread` overwrites the source buffer after posting and verifies the subscriber receives the original value, and that the callback runs on the executor thread.

### REQ_PCL_050 - Post Incoming Bad Inputs
`pcl_executor_post_incoming()` shall handle NULL executor, NULL topic, and NULL message gracefully.

**Traces**: PCL.025, PCL.045

**Verification**: `test_pcl_robustness.cpp::PostIncomingBadInputs`.

### REQ_PCL_051 - Post Incoming Zero Size No Data
`pcl_executor_post_incoming()` shall accept a zero-size message without data.

**Traces**: PCL.025

**Verification**: `test_pcl_robustness.cpp::PostIncomingZeroSizeNoData`.

### REQ_PCL_052 - Destroy With Pending Messages Frees
Destroying an executor with pending queued messages shall free the queued messages without crashing.

**Traces**: PCL.025, PCL.046

**Verification**: `test_pcl_robustness.cpp::DestroyWithPendingMessagesFrees`.

### REQ_PCL_053 - Concurrent Post From Many Threads
Multiple threads posting simultaneously via `pcl_executor_post_incoming()` shall not corrupt state or lose messages.

**Traces**: PCL.025

**Verification**: `test_pcl_robustness.cpp::ConcurrentPostFromManyThreads` uses 8 threads each posting 100 messages.

### REQ_PCL_054 - Producer Consumer With Subscriber
A producer thread posting messages while the executor spins shall deliver all messages to the subscriber callback.

**Traces**: PCL.025, PCL.026

**Verification**: `test_pcl_robustness.cpp::ProducerConsumerWithSubscriber`.

### REQ_PCL_055 - Spin In Background Shutdown From Foreground
Posting messages to a spinning executor from another thread, then requesting shutdown, shall complete cleanly.

**Traces**: PCL.020, PCL.025

**Verification**: `test_pcl_robustness.cpp::SpinInBackgroundShutdownFromForeground`.

### REQ_PCL_056 - Post During Active Spin No Corruption
High-rate posting from multiple threads during active spin shall not corrupt the ingress queue.

**Traces**: PCL.025

**Verification**: `test_pcl_robustness.cpp::PostDuringActiveSpinNoCorruption`.

---

## 10. Async Response Delivery

### REQ_PCL_057 - Post Response Callback With Data
`pcl_executor_post_response_cb()` shall deep-copy the data and fire the callback on the next spin.

**Traces**: PCL.027

**Verification**: `test_pcl_robustness.cpp::PostResponseCbWithDataFiresOnSpin`.

### REQ_PCL_058 - Post Response Callback No Data
`pcl_executor_post_response_cb()` with zero size and NULL data shall fire the callback.

**Traces**: PCL.027

**Verification**: `test_pcl_robustness.cpp::PostResponseCbNoData`.

### REQ_PCL_059 - Post Response Callback Multiple Queued
Multiple queued response callbacks shall all fire on the next spin.

**Traces**: PCL.027

**Verification**: `test_pcl_robustness.cpp::PostResponseCbMultipleOnQueue`.

### REQ_PCL_060 - Post Response Callback Null Safety
Null executor or callback arguments shall return `PCL_ERR_INVALID`.

**Traces**: PCL.027, PCL.045

**Verification**: `test_pcl_robustness.cpp::PostResponseCbNullSafety`.

### REQ_PCL_061 - Destroy With Queued Response Callbacks
Destroying an executor with queued response callbacks (including those with data) shall free all resources.

**Traces**: PCL.027, PCL.046

**Verification**: `test_pcl_robustness.cpp::DestroyWithQueuedRespCbWithData`.

---

## 11. Transport Adapter

### REQ_PCL_062 - Set Transport And Clear
`pcl_executor_set_transport()` shall accept a transport adapter. Passing NULL shall revert to direct dispatch.

**Traces**: PCL.029

**Verification**: `test_pcl_robustness.cpp::SetTransportAndClear`.

### REQ_PCL_063 - Set Transport Null Executor
`pcl_executor_set_transport()` with a NULL executor shall return `PCL_ERR_INVALID`.

**Traces**: PCL.029, PCL.045

**Verification**: `test_pcl_robustness.cpp::SetTransportNullExecutorReturnsInvalid`.

### REQ_PCL_164 - Invoke Async Routes Through Transport
`pcl_executor_invoke_async()` shall route the service request through the transport adapter's `invoke_async` function pointer when a transport is set.

**Traces**: PCL.011a, PCL.030a

**Verification**: `test_pcl_robustness.cpp::InvokeAsyncRoutesViaTransport`.

### REQ_PCL_165 - Invoke Async Intra-Process Fallback
`pcl_executor_invoke_async()` shall fall back to intra-process synchronous dispatch when no transport is set or when the transport's `invoke_async` is NULL. The service handler shall be invoked immediately and the callback fired before returning.

**Traces**: PCL.011a, PCL.022, PCL.030a

**Verification**: `test_pcl_robustness.cpp::InvokeAsyncIntraProcessFallback`.

### REQ_PCL_166 - Invoke Async Null Arguments
`pcl_executor_invoke_async()` shall return `PCL_ERR_INVALID` when the executor, service name, request, or callback is NULL.

**Traces**: PCL.030a, PCL.045

**Verification**: `test_pcl_robustness.cpp::InvokeAsyncNullArgsReturnsInvalid`.

---

## 12. Logging

### REQ_PCL_064 - Custom Handler Receives Messages
A custom log handler shall receive formatted messages with the correct level.

**Traces**: PCL.037, PCL.038

**Verification**: `test_pcl_log.cpp::CustomHandlerReceivesMessages`.

### REQ_PCL_065 - Container Name Passed Through
The log handler shall receive the container name when a container context is provided.

**Traces**: PCL.037

**Verification**: `test_pcl_log.cpp::ContainerNamePassedThrough`.

### REQ_PCL_066 - Log Level Filtering
Messages below the configured minimum level shall not be delivered to the handler.

**Traces**: PCL.039

**Verification**: `test_pcl_log.cpp::LevelFiltering` sets WARN level and verifies only WARN and ERROR are delivered.

### REQ_PCL_067 - Printf Format String
`pcl_log()` shall support printf-style format strings with variadic arguments.

**Traces**: PCL.037

**Verification**: `test_pcl_log.cpp::FormatString` verifies formatted output with int and float arguments.

### REQ_PCL_068 - Revert To Default Handler
Passing NULL to `pcl_log_set_handler()` shall revert to the default stderr handler without crashing.

**Traces**: PCL.038

**Verification**: `test_pcl_log.cpp::RevertToDefaultHandler`.

### REQ_PCL_069 - All Log Levels Reach Handler
Messages at all five levels (DEBUG, INFO, WARN, ERROR, FATAL) shall reach the handler when the minimum level is DEBUG.

**Traces**: PCL.040

**Verification**: `test_pcl_robustness.cpp::AllLevelsReachHandler`.

### REQ_PCL_070 - Filter Blocks All Below Fatal
Setting minimum level to FATAL shall block DEBUG, INFO, WARN, and ERROR messages.

**Traces**: PCL.039

**Verification**: `test_pcl_robustness.cpp::FilterBlocksAllBelowFatal`.

### REQ_PCL_071 - Long Format String Truncated
Format strings producing output longer than the internal buffer shall be truncated without crashing.

**Traces**: PCL.037

**Verification**: `test_pcl_robustness.cpp::LongFormatStringTruncated`.

### REQ_PCL_072 - Handler User Data Passed Through
The user_data pointer passed to `pcl_log_set_handler()` shall be forwarded to every handler invocation.

**Traces**: PCL.038

**Verification**: `test_pcl_robustness.cpp::HandlerUserDataPassedThrough`.

### REQ_PCL_073 - Default Handler No Container Name
The default handler shall not crash when the container context is NULL.

**Traces**: PCL.037

**Verification**: `test_pcl_robustness.cpp::DefaultHandlerNoContainerName`.

### REQ_PCL_074 - Default Handler With Container Name
The default handler shall include the container name when a container context is provided.

**Traces**: PCL.037

**Verification**: `test_pcl_robustness.cpp::DefaultHandlerWithContainerName`.

---

## 13. Bridge

### REQ_PCL_075 - Bridge Create And Destroy
`pcl_bridge_create()` shall return a non-NULL handle with a valid internal container. `pcl_bridge_destroy()` shall free all bridge-owned resources.

**Traces**: PCL.041

**Verification**: `test_pcl_dining.cpp::CreateAndDestroy`; also `test_pcl_bridge.cpp::PclBridge.CreateDestroy` in isolation from the dining integration.

### REQ_PCL_076 - Bridge Null Arguments Return NULL
`pcl_bridge_create()` shall return NULL when any mandatory argument is NULL.

**Traces**: PCL.044

**Verification**: `test_pcl_dining.cpp::CreateNullArgsReturnNull` tests all NULL argument combinations; also `test_pcl_bridge.cpp::PclBridge.CreateNullArgs`.

### REQ_PCL_077 - Bridge Container Null Returns NULL
`pcl_bridge_container(NULL)` shall return NULL.

**Traces**: PCL.044

**Verification**: `test_pcl_dining.cpp::ContainerNullBridgeReturnsNull`; also `test_pcl_bridge.cpp::PclBridge.ContainerNullBridge`.

### REQ_PCL_078 - Bridge Destroy Null Is No-Op
`pcl_bridge_destroy(NULL)` shall be a no-op.

**Traces**: PCL.044

**Verification**: `test_pcl_dining.cpp::DestroyNullIsNoOp`; also `test_pcl_bridge.cpp::PclBridge.DestroyNullBridge`.

### REQ_PCL_079 - Bridge Unit Conversion
A bridge with a transform function shall convert messages (e.g., float m/s -> int32 km/h) and dispatch to the output topic.

**Traces**: PCL.042

**Verification**: `test_pcl_dining.cpp::SpeedUnitConversion` dispatches 10.0 m/s and verifies 36 km/h, then 27.78 m/s and verifies 100 km/h; also `test_pcl_bridge.cpp::PclBridge.TransformSuccess` for a generic pass-through transform.

### REQ_PCL_080 - Bridge Fail Transform Suppresses Forward
When the transform function returns non-OK, no message shall be dispatched to the output topic.

**Traces**: PCL.043

**Verification**: `test_pcl_dining.cpp::FailTransformSuppressesForward`; also `test_pcl_bridge.cpp::PclBridge.TransformSuppressed`.

### REQ_PCL_081 - Bridge Inactive Does Not Forward
An inactive bridge (CONFIGURED but not ACTIVE) shall not forward messages.

**Traces**: PCL.041

**Verification**: `test_pcl_dining.cpp::BridgeInactiveDoesNotForward`.

### REQ_PCL_082 - Bridge Configure Fails When Ports Full
If the bridge's internal container has no free port slots, configure shall fail with `PCL_ERR_NOMEM`.

**Traces**: PCL.041, PCL.046

**Verification**: `test_pcl_dining.cpp::ConfigureFailsWhenPortsFull` forces the overflow; `test_pcl_bridge.cpp::PclBridge.ConfigurePortOverflow` exercises the same create/configure/destroy path as a smoke test.

### REQ_PCL_083 - Bridge Dispatch Not Found Logs Debug
When the bridge's output topic has no subscriber, the bridge shall log a DEBUG message and not crash.

**Traces**: PCL.041

**Verification**: `test_pcl_dining.cpp::DispatchNotFoundLogsDebug`; also `test_pcl_bridge.cpp::PclBridge.NoDownstreamSubscriber`.

### REQ_PCL_328 - Bridge Transform User Data Forwarded
The `user_data` pointer supplied to `pcl_bridge_create()` shall be forwarded to the transform function on every dispatch.

**Traces**: PCL.041

**Verification**: `test_pcl_bridge.cpp::PclBridge.TransformWithUserData`.

---

## 14. Dining Philosophers Integration

### REQ_PCL_084 - Multi-Container Bridge Integration
A graph of 5 philosopher containers, 5 bridges, and 1 monitor container shall execute without mutual exclusion violations and with all philosophers completing the required meals.

**Traces**: PCL.018, PCL.042

**Verification**: `test_pcl_dining.cpp::FivePhilosophersWithBridges` asserts no mutual exclusion violation and that each philosopher completes >= 3 meals.

---

## 15. Out-of-Memory Handling

### REQ_PCL_085 - Post Incoming Pending Alloc Failure
If the pending message allocation fails, `pcl_executor_post_incoming()` shall return `PCL_ERR_NOMEM`.

**Traces**: PCL.046

**Verification**: `test_pcl_oom.cpp::PostIncomingPendingAllocFails`.

### REQ_PCL_086 - Post Incoming Topic Strdup Failure
If the topic string duplication fails, `pcl_executor_post_incoming()` shall free partial allocations and return `PCL_ERR_NOMEM`.

**Traces**: PCL.046

**Verification**: `test_pcl_oom.cpp::PostIncomingTopicStrdupFails`.

### REQ_PCL_087 - Post Incoming Type Name Strdup Failure
If the type name string duplication fails, `pcl_executor_post_incoming()` shall free partial allocations and return `PCL_ERR_NOMEM`.

**Traces**: PCL.046

**Verification**: `test_pcl_oom.cpp::PostIncomingTypeNameStrdupFails`.

### REQ_PCL_088 - Post Incoming Data Malloc Failure
If the data payload allocation fails, `pcl_executor_post_incoming()` shall free partial allocations and return `PCL_ERR_NOMEM`.

**Traces**: PCL.046

**Verification**: `test_pcl_oom.cpp::PostIncomingDataMallocFails`.

### REQ_PCL_089 - Bridge Create Container Alloc Failure
If the internal container allocation fails during bridge creation, the bridge struct shall be freed and NULL returned.

**Traces**: PCL.046

**Verification**: `test_pcl_oom.cpp::BridgeCreateContainerAllocFails`.

### REQ_PCL_090 - Post Response Callback Data Malloc Failure
If the data buffer allocation fails in `pcl_executor_post_response_cb()`, the pending node shall be freed and `PCL_ERR_NOMEM` returned.

**Traces**: PCL.046

**Verification**: `test_pcl_oom.cpp::PostResponseCbDataMallocFails`.

---

## 16. Throughput and Performance

### REQ_PCL_091 - Spin Once Burst Dispatch
The executor shall handle a burst of dispatched messages within a single spin_once cycle.

**Traces**: PCL.018

**Verification**: `test_pcl_robustness.cpp::SpinOnceBurstDispatch`.

### REQ_PCL_092 - High Frequency Tick Rate
The executor shall support tick rates up to 1000 Hz without missing ticks over a short interval.

**Traces**: PCL.015

**Verification**: `test_pcl_robustness.cpp::HighFrequencyTickRate`.

### REQ_PCL_093 - Multi Container Different Rates
Multiple containers at different tick rates shall each be ticked at approximately their configured rate.

**Traces**: PCL.018

**Verification**: `test_pcl_robustness.cpp::MultiContainerDifferentRates`.

---

## 17. Service Bindings (Proto/Generated)

### REQ_PCL_094 - Provided Service Wire Names
The provided service binding constants shall match the proto-defined wire names for matching_objects, object_of_interest, and specific_object_detail services.

**Traces**: PCL.050

**Verification**: `test_pcl_proto_bindings.cpp::WireNames` (ProtoBindingsProvided).

### REQ_PCL_095 - Provided Topic Names
The provided topic constants shall match: `standard.entity_matches` and `standard.evidence_requirements`.

**Traces**: PCL.051

**Verification**: `test_pcl_proto_bindings.cpp::TopicNames` (ProtoBindingsProvided).

### REQ_PCL_096 - Consumed Service Wire Names
The consumed service binding constants shall match the proto-defined wire names for object_evidence, object_solution_evidence, and object_source_capability services.

**Traces**: PCL.050

**Verification**: `test_pcl_proto_bindings.cpp::WireNames` (ProtoBindingsConsumed).

### REQ_PCL_097 - Consumed Topic Names
The consumed topic constant shall match: `standard.object_evidence`.

**Traces**: PCL.051

**Verification**: `test_pcl_proto_bindings.cpp::TopicNames` (ProtoBindingsConsumed).

### REQ_PCL_098 - Provided msgToString Utility
The provided binding's `msgToString()` shall convert a data/size pair to a std::string.

**Traces**: PCL.053

**Verification**: `test_pcl_proto_bindings.cpp::MsgToString` and `MsgToStringEmpty` (ProtoBindingsProvided).

### REQ_PCL_099 - Consumed msgToString Utility
The consumed binding's `msgToString()` shall convert a data/size pair to a std::string.

**Traces**: PCL.053

**Verification**: `test_pcl_proto_bindings.cpp::MsgToString` (ProtoBindingsConsumed).

### REQ_PCL_100 - Build Standard Requirement JSON
`buildStandardRequirementJson()` shall produce valid JSON with policy, identity, dimension, and lat/lon bounds fields.

**Traces**: PCL.053

**Verification**: `test_pcl_proto_bindings.cpp::BuildRequirementJsonKeys` and `BuildRequirementJsonNoDimension`.

### REQ_PCL_101 - Build Standard Evidence JSON
`buildStandardEvidenceJson()` shall produce valid JSON with identity, dimension, lat/lon, confidence, and observed_at fields.

**Traces**: PCL.053

**Verification**: `test_pcl_proto_bindings.cpp::BuildEvidenceJsonKeys` and `BuildEvidenceJsonDefaultObservedAt`.

### REQ_PCL_102 - Provided Service Handler Stubs
The provided `ServiceHandler` default stubs shall return empty collections or success acknowledgements.

**Traces**: PCL.052

**Verification**: `test_pcl_proto_bindings.cpp::HandlerStubsReturnEmpty` (ProtoBindingsProvided).

### REQ_PCL_103 - Consumed Service Handler Stubs
The consumed `ServiceHandler` default stubs shall return empty collections or success acknowledgements.

**Traces**: PCL.052

**Verification**: `test_pcl_proto_bindings.cpp::HandlerStubsReturnEmpty` (ProtoBindingsConsumed).

### REQ_PCL_104 - Provided Subscribe Registration
`subscribeEntityMatches()` and `subscribeEvidenceRequirements()` shall register PCL subscriber ports successfully during on_configure.

**Traces**: PCL.054

**Verification**: `test_pcl_proto_bindings.cpp::SubscribeRegistrationSucceeds` (ProtoBindingsProvided).

### REQ_PCL_105 - Consumed Subscribe Registration
`subscribeObjectEvidence()` shall register a PCL subscriber port successfully during on_configure.

**Traces**: PCL.054

**Verification**: `test_pcl_proto_bindings.cpp::SubscribeRegistrationSucceeds` (ProtoBindingsConsumed).

### REQ_PCL_106 - Consumed Publish When Inactive
`publishObjectEvidence()` shall return a non-OK status when the container is not ACTIVE.

**Traces**: PCL.009

**Verification**: `test_pcl_proto_bindings.cpp::PublishReturnsBadStatusWhenInactive`.

### REQ_PCL_107 - Consumed Publish When Active
`publishObjectEvidence()` shall succeed when the container is ACTIVE.

**Traces**: PCL.009

**Verification**: `test_pcl_proto_bindings.cpp::PublishSucceedsWhenActive`.

### REQ_PCL_108 - Provided Dispatch All Channels
The provided binding's `dispatch()` function shall handle all `ServiceChannel` enum values without crashing.

**Traces**: PCL.055

**Verification**: `test_pcl_proto_bindings.cpp::DispatchAllChannelsNoCrash` (ProtoBindingsProvided).

### REQ_PCL_109 - Consumed Dispatch All Channels
The consumed binding's `dispatch()` function shall handle all `ServiceChannel` enum values without crashing.

**Traces**: PCL.055

**Verification**: `test_pcl_proto_bindings.cpp::DispatchAllChannelsNoCrash` (ProtoBindingsConsumed).

### REQ_PCL_110 - Domain Type Constants
The Ack constants (`kAckOk`, `kAckFail`) shall have correct success values. Query, Identifier, and ObjectDetail types shall have sensible defaults.

**Traces**: PCL.052

**Verification**: `test_pcl_proto_bindings.cpp::AckConstants`, `QueryDefault`, `IdentifierIsString`, `ObjectDetailDefaults`.

---

## 18. Null Safety (API-Wide)

### REQ_PCL_111 - Container Null Handle Safety
All container API functions shall return appropriate error codes or safe defaults when passed NULL handles.

**Traces**: PCL.045

**Verification**: `test_pcl_lifecycle.cpp::NullHandlesReturnError`.

### REQ_PCL_112 - Executor Null Handle Safety
All executor API functions shall return appropriate error codes when passed NULL handles.

**Traces**: PCL.045

**Verification**: `test_pcl_executor.cpp::NullSafety`.

### REQ_PCL_113 - Request Shutdown Null Safe
`pcl_executor_request_shutdown(NULL)` shall be a no-op.

**Traces**: PCL.045

**Verification**: `test_pcl_robustness.cpp::RequestShutdownNullSafe`.

### REQ_PCL_114 - Container Long Name Truncated
A container name exceeding the internal buffer shall be truncated without crashing.

**Traces**: PCL.005, PCL.045

**Verification**: `test_pcl_robustness.cpp::LongNameTruncated`.

---

## 16. TCP Socket Transport

### REQ_PCL_115 - Server Creation Returns Valid Transport
`pcl_socket_transport_create_server()` shall return a non-NULL handle with a valid transport vtable (publish, subscribe, shutdown all non-NULL).

**Traces**: PCL.031

**Verification**: `test_pcl_socket_transport.cpp::ServerCreationAndDestroy`.

### REQ_PCL_116 - Server Null Executor Returns NULL
`pcl_socket_transport_create_server()` shall return NULL when the executor argument is NULL.

**Traces**: PCL.031, PCL.045

**Verification**: `test_pcl_socket_transport.cpp::ServerNullExecutorReturnsNull`.

### REQ_PCL_117 - Client Creation Returns Valid Transport
`pcl_socket_transport_create_client()` shall return a non-NULL handle with a valid transport vtable when connecting to a listening server.

**Traces**: PCL.032

**Verification**: `test_pcl_socket_transport.cpp::ClientCreationAndDestroy`.

### REQ_PCL_118 - Client Null Arguments Return NULL
`pcl_socket_transport_create_client()` shall return NULL when the host or executor argument is NULL.

**Traces**: PCL.032, PCL.045

**Verification**: `test_pcl_socket_transport.cpp::ClientNullArgsReturnNull`.

### REQ_PCL_119 - Client Connect To Nonexistent Server Fails
`pcl_socket_transport_create_client()` shall return NULL when no server is listening on the target port.

**Traces**: PCL.032

**Verification**: `test_pcl_socket_transport.cpp::ClientConnectToNonexistentServerFails`.

### REQ_PCL_120 - Publish Server To Client Delivered
A message published via the server transport shall be received by a matching subscriber on the client executor.

**Traces**: PCL.033

**Verification**: `test_pcl_socket_transport.cpp::PublishServerToClientDelivered`.

### REQ_PCL_121 - Publish Client To Server Delivered
A message published via the client transport shall be received by a matching subscriber on the server executor.

**Traces**: PCL.033

**Verification**: `test_pcl_socket_transport.cpp::PublishClientToServerDelivered`.

### REQ_PCL_122 - Gateway Container Exists And Is Configurable
`pcl_socket_transport_gateway_container()` on a server transport shall return a non-NULL container that supports full lifecycle transitions.

**Traces**: PCL.034

**Verification**: `test_pcl_socket_transport.cpp::GatewayContainerExists`.

### REQ_PCL_123 - Gateway Null Transport Returns NULL
`pcl_socket_transport_gateway_container(NULL)` shall return NULL.

**Traces**: PCL.034, PCL.045

**Verification**: `test_pcl_socket_transport.cpp::GatewayContainerNullReturnsNull`.

### REQ_PCL_124 - Client Has No Gateway
`pcl_socket_transport_gateway_container()` on a client transport shall return NULL (gateway is server-only).

**Traces**: PCL.034

**Verification**: `test_pcl_socket_transport.cpp::ClientHasNoGateway`.

### REQ_PCL_125 - Publish Is Non-Blocking
Publishing 100 messages in rapid succession via the transport shall complete in under 1 second, confirming no blocking I/O on the publish path.

**Traces**: PCL.035

**Verification**: `test_pcl_socket_transport.cpp::PublishIsNonBlocking`.

### REQ_PCL_126 - Async Remote Service Round Trip
`pcl_socket_transport_invoke_remote_async()` shall send a service request from the client, invoke the registered handler on the server, and deliver the response back to the client callback.

**Traces**: PCL.036

**Verification**: `test_pcl_socket_transport.cpp::AsyncRemoteServiceRoundTrip`.

### REQ_PCL_127 - Invoke Remote Async Null Arguments
`pcl_socket_transport_invoke_remote_async()` shall return `PCL_ERR_INVALID` when any required argument (transport, service name, request, callback) is NULL.

**Traces**: PCL.036, PCL.045

**Verification**: `test_pcl_socket_transport.cpp::InvokeRemoteAsyncNullArgs`.

### REQ_PCL_128 - Invoke Remote Async On Server Returns Invalid
`pcl_socket_transport_invoke_remote_async()` shall return `PCL_ERR_INVALID` when called on a server transport (only clients may invoke remote services).

**Traces**: PCL.036

**Verification**: `test_pcl_socket_transport.cpp::InvokeRemoteAsyncOnServerReturnsInvalid`.

### REQ_PCL_129 - Get Transport Null Returns NULL
`pcl_socket_transport_get_transport(NULL)` shall return NULL.

**Traces**: PCL.045

**Verification**: `test_pcl_socket_transport.cpp::GetTransportNullReturnsNull`.

### REQ_PCL_130 - Destroy Null Is No-Op
`pcl_socket_transport_destroy(NULL)` shall be a no-op (no crash).

**Traces**: PCL.045

**Verification**: `test_pcl_socket_transport.cpp::DestroyNullIsNoOp`.

### REQ_PCL_158 - Subscribe Vtable Is Callable
The transport vtable `subscribe` function shall be callable and return `PCL_OK` (interest registration is a no-op in the socket transport).

**Traces**: PCL.031

**Verification**: `test_pcl_socket_transport.cpp::SubscribeVtableCallSucceeds`.

### REQ_PCL_159 - Shutdown Vtable Sets Stop Flag
The transport vtable `shutdown` function shall set the recv_stop flag, signalling the receive thread to exit.

**Traces**: PCL.031

**Verification**: `test_pcl_socket_transport.cpp::ShutdownVtableCallSucceeds`.

### REQ_PCL_160 - Destroy With Pending Async Calls Frees Resources
`pcl_socket_transport_destroy()` shall free all un-responded pending async service records without leak.

**Traces**: PCL.036

**Verification**: `test_pcl_socket_transport.cpp::DestroyWithPendingAsyncCallNoLeak`.

### REQ_PCL_161 - Destroy With Unsent Frames Frees Resources
`pcl_socket_transport_destroy()` shall drain and free all queued outbound frames without leak.

**Traces**: PCL.035

**Verification**: `test_pcl_socket_transport.cpp::DestroyWithUnsentFramesNoLeak`.

### REQ_PCL_162 - Invoke Remote Async Oversized Payload Returns NOMEM
`pcl_socket_transport_invoke_remote_async()` shall return `PCL_ERR_NOMEM` when the total payload exceeds `PCL_SOCKET_MAX_PAYLOAD`.

**Traces**: PCL.036

**Verification**: `test_pcl_socket_transport.cpp::InvokeRemoteAsyncOversizedPayloadReturnsNomem`.

### REQ_PCL_163 - Gateway Service Dispatch With No Match
When the gateway receives a service request for which no handler is registered, it shall still send a response (with empty data) back to the client.

**Traces**: PCL.034, PCL.036

**Verification**: `test_pcl_socket_transport.cpp::GatewayServiceDispatchNoMatch`.

### REQ_PCL_190 - Client Extended Create With Null Opts Preserves Single-Shot Semantics
`pcl_socket_transport_create_client_ex()` with all-zero `pcl_socket_client_opts_t` (or NULL) shall perform a single connection attempt and return NULL immediately when the server is not listening, matching legacy `create_client` behaviour.

**Traces**: PCL.032, PCL.036e

**Verification**: `test_pcl_socket_transport.cpp::ClientExSingleShotFailsFast`.

### REQ_PCL_191 - Client Extended Create Retries Connect With Backoff
`pcl_socket_transport_create_client_ex()` shall retry the TCP connect with exponential backoff (starting at 100 ms, doubling up to a 2 s cap) until it succeeds, `max_retries` is exceeded, or `connect_timeout_ms` elapses. A server that starts after the client begins connecting shall be reachable within the deadline.

**Traces**: PCL.032, PCL.036e

**Verification**: `test_pcl_socket_transport.cpp::ClientExRetryConnectsToDelayedServer`.

### REQ_PCL_192 - Client Extended Create Honours Connect Timeout
When `connect_timeout_ms` is set and no server responds, `pcl_socket_transport_create_client_ex()` shall return NULL within a bounded window close to the configured deadline and fire the state callback with `DISCONNECTED`.

**Traces**: PCL.032, PCL.036e, PCL.045

**Verification**: `test_pcl_socket_transport.cpp::ClientExRetryHonoursTimeout`.

### REQ_PCL_193 - Connection State Accessor
`pcl_socket_transport_get_state()` shall return the current `pcl_socket_state_t` of a live transport and `PCL_SOCKET_STATE_DISCONNECTED` for a NULL handle.

**Traces**: PCL.031, PCL.032, PCL.036e, PCL.045

**Verification**: `test_pcl_socket_transport.cpp::GetStateReportsConnected`.

### REQ_PCL_194 - State Callback Fires On Initial Connect
When `pcl_socket_client_opts_t::state_cb` is non-NULL, it shall be invoked with `CONNECTING` before the first connect attempt and with `CONNECTED` after a successful attempt.

**Traces**: PCL.032, PCL.036e

**Verification**: `test_pcl_socket_transport.cpp::StateCallbackFiresOnInitialConnect`.

### REQ_PCL_195 - Client Auto-Reconnects After Server Restart
When `pcl_socket_client_opts_t::auto_reconnect` is non-zero, the receive thread shall detect a dropped connection, fire the state callback with `DISCONNECTED`, then attempt to reconnect with the same backoff policy. A server restarted on the same port shall cause `get_state()` to return `CONNECTED` again without the caller re-creating the transport.

**Traces**: PCL.031, PCL.032, PCL.036e

**Verification**: `test_pcl_socket_transport.cpp::AutoReconnectAfterServerRestart`.

### REQ_PCL_196 - Client Resolves Hosts Via getaddrinfo
`pcl_socket_transport_create_client_ex()` shall resolve the `host` argument via `getaddrinfo` (thread-safe, IPv6-aware) and attempt each returned address in order until one succeeds.

**Traces**: PCL.032, PCL.036e

**Verification**: `test_pcl_socket_transport.cpp::ClientCreationAndDestroy` and `test_pcl_socket_transport.cpp::ClientExRetryConnectsToDelayedServer` exercise the getaddrinfo resolution path against `"127.0.0.1"`.

### REQ_PCL_197 - TCP Keepalive Enabled On Connected Sockets
Every connected TCP socket (client-connect and server-accept) shall have `SO_KEEPALIVE` enabled. On Linux, `TCP_KEEPIDLE`, `TCP_KEEPINTVL`, and `TCP_KEEPCNT` shall be tuned so silent peer death is detected within approximately eight seconds.

**Traces**: PCL.031, PCL.032, PCL.036e

**Verification**: `test_pcl_socket_transport.cpp::AutoReconnectAfterServerRestart` depends on timely peer-death detection via the keepalive settings.

### REQ_PCL_198 - UDP Transport Creation
`pcl_udp_transport_create(local_port, remote_host, remote_port, executor)` shall bind a UDP socket on `local_port` (0 = ephemeral), resolve `remote_host:remote_port`, spawn a receive thread, and expose a `pcl_transport_t` vtable with `publish`, `subscribe`, and `shutdown` populated. It shall return NULL for NULL `remote_host` or NULL `executor`.

**Traces**: PCL.036f, PCL.045

**Verification**: `test_pcl_udp_transport.cpp::CreateAndDestroy`, `NullArgsReturnNull`, `DestroyNullIsNoOp`.

### REQ_PCL_199 - UDP Transport Peer Identity Configurable
`pcl_udp_transport_set_peer_id()` shall set the logical peer ID used when posting inbound datagrams to the executor. It shall reject NULL or empty peer IDs with `PCL_ERR_INVALID`.

**Traces**: PCL.036f, PCL.036a

**Verification**: `test_pcl_udp_transport.cpp::SetPeerIdRoundTrip`.

### REQ_PCL_200 - UDP Publish Round-Trip Delivers Message To Subscriber
When the sender's transport publishes a `pcl_msg_t`, the receiver's subscriber (with `PCL_ROUTE_REMOTE` and the sender's peer ID in its allow list) shall receive the message with byte-identical payload and topic after at most a few publish attempts (UDP is best-effort).

**Traces**: PCL.036f

**Verification**: `test_pcl_udp_transport.cpp::PublishDeliveredToSubscriber`.

### REQ_PCL_201 - UDP Subscriber Peer Allow-List Enforced
Inbound datagrams shall be posted via `pcl_executor_post_remote_incoming()` with the configured peer ID, so subscriber peer allow-lists drop traffic from foreign peers.

**Traces**: PCL.036f, PCL.030d

**Verification**: `test_pcl_udp_transport.cpp::SubscriberPeerFilterDropsForeignIngress`.

### REQ_PCL_202 - UDP Publish Rejects Oversized Payloads
Publishing a message whose serialised payload exceeds `PCL_UDP_MAX_PAYLOAD` (1400 bytes) shall return `PCL_ERR_NOMEM` without sending a datagram, preventing silent truncation or IP-layer fragmentation.

**Traces**: PCL.036f

**Verification**: `test_pcl_udp_transport.cpp::OversizedPublishReturnsNomem`.

### REQ_PCL_203 - UDP Transport Does Not Expose Service RPC Or Streaming Vtable Entries
The UDP transport's `pcl_transport_t` shall leave `invoke_async`, `respond`, `serve`, and `invoke_stream` NULL so the executor returns `PCL_ERR_NOT_FOUND` for unsupported request/reply or streaming calls rather than silently accepting unreliable RPC.

**Traces**: PCL.036f

**Verification**: `test_pcl_udp_transport.cpp::NoServiceRpcSupport`.

### REQ_PCL_204 - Non-Reconnecting Client Reports Disconnected After Peer Drop
When a TCP client created with `auto_reconnect == 0` observes its peer closing the socket, `pcl_socket_transport_get_state()` shall transition to `PCL_SOCKET_STATE_DISCONNECTED` and the registered state callback shall fire with that value, regardless of whether the transport will attempt to re-establish the connection.

**Traces**: PCL.036e

**Verification**: `test_pcl_socket_transport.cpp::NonReconnectingClientReportsDisconnectedAfterServerClose`.

### REQ_PCL_205 - Connect Attempts Bounded By Connect Timeout Budget
Each TCP connect attempt performed by `pcl_socket_transport_create_client_ex()` shall be bounded by the remaining `connect_timeout_ms` budget (or a 2-second slice when no total timeout is set), so a single SYN to a blackholed host cannot exceed the caller's deadline by more than one per-attempt slice. When the total budget is exhausted the function shall return NULL.

**Traces**: PCL.036e

**Verification**: `test_pcl_socket_transport.cpp::ClientExRespectsTimeoutOnBlackholedHost`.

---

## 17. C++ Component Wrapper

### REQ_PCL_131 - Component Construction Produces Valid Handle
Constructing a `pcl::Component` subclass shall produce a non-NULL `pcl_container_t` handle in UNCONFIGURED state with the given name.

**Traces**: PCL.048

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppComponent.ConstructionProducesValidHandle`.

### REQ_PCL_132 - Component Destructor Frees Resources
The `pcl::Component` destructor shall call `pcl_container_destroy()`, freeing all associated resources without leak or crash.

**Traces**: PCL.048

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppComponent.DestructorFreesResources`.

### REQ_PCL_133 - Component Lifecycle Callbacks Invoked
Virtual methods `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`, and `on_shutdown` shall be invoked via the C trampoline on the corresponding lifecycle transitions.

**Traces**: PCL.048

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppComponent.LifecycleCallbacksInvoked`.

### REQ_PCL_134 - Component addPublisher During Configure
`addPublisher()` shall create a publisher port during `on_configure` and return a non-NULL port handle.

**Traces**: PCL.048

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppComponent.AddPublisherDuringConfigure`.

### REQ_PCL_135 - Component addSubscriber During Configure
`addSubscriber()` shall create a subscriber port during `on_configure` and return a non-NULL port handle.

**Traces**: PCL.048

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppComponent.AddSubscriberDuringConfigure`.

### REQ_PCL_136 - Component addService During Configure
`addService()` shall create a service port during `on_configure` and return a non-NULL port handle.

**Traces**: PCL.048

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppComponent.AddServiceDuringConfigure`.

### REQ_PCL_137 - Component Parameter String Round-Trip
`setParam(key, string)` followed by `paramStr(key)` shall return the stored string value; missing keys shall return the default.

**Traces**: PCL.048

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppComponent.ParamStringRoundTrip`.

### REQ_PCL_138 - Component Parameter Double Round-Trip
`setParam(key, double)` followed by `paramF64(key)` shall return the stored value; missing keys shall return the default.

**Traces**: PCL.048

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppComponent.ParamDoubleRoundTrip`.

### REQ_PCL_139 - Component Parameter Int64 Round-Trip
`setParam(key, int64_t)` followed by `paramI64(key)` shall return the stored value; missing keys shall return the default.

**Traces**: PCL.048

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppComponent.ParamInt64RoundTrip`.

### REQ_PCL_140 - Component Parameter Bool Round-Trip
`setParam(key, bool)` followed by `paramBool(key)` shall return the stored value; missing keys shall return the default.

**Traces**: PCL.048

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppComponent.ParamBoolRoundTrip`.

### REQ_PCL_141 - Component Tick Rate Round-Trip
`setTickRateHz()` followed by `tickRateHz()` shall return the stored rate; the default shall be 100 Hz.

**Traces**: PCL.048

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppComponent.TickRateRoundTrip`.

### REQ_PCL_142 - Component Logging Does Not Crash
`logDebug`, `logInfo`, `logWarn`, and `logError` shall forward to `pcl_log()` without crash.

**Traces**: PCL.048

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppComponent.LoggingDoesNotCrash`.

### REQ_PCL_143 - Component Move Constructor Transfers Ownership
The move constructor shall transfer the underlying `pcl_container_t` handle, leaving the source with a NULL handle.

**Traces**: PCL.048

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppComponent.MoveConstructor`.

### REQ_PCL_144 - Component Move Assignment Transfers Ownership
The move assignment operator shall transfer the underlying `pcl_container_t` handle, destroying any prior handle held by the target.

**Traces**: PCL.048

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppComponent.MoveAssignment`.

---

## 18. C++ Executor Wrapper

### REQ_PCL_145 - Executor Construction Produces Valid Handle
Constructing a `pcl::Executor` shall produce a non-NULL `pcl_executor_t` handle.

**Traces**: PCL.049

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppExecutor.ConstructionProducesValidHandle`.

### REQ_PCL_146 - Executor Destructor Frees Resources
The `pcl::Executor` destructor shall call `pcl_executor_destroy()`, freeing all associated resources without leak or crash.

**Traces**: PCL.049

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppExecutor.DestructorFreesResources`.

### REQ_PCL_147 - Executor Add Component
`add(Component&)` shall add a `pcl::Component` to the executor and return `PCL_OK`.

**Traces**: PCL.049

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppExecutor.AddComponent`.

### REQ_PCL_148 - Executor Add Raw Container
`add(pcl_container_t*)` shall add a raw C container to the executor and return `PCL_OK`.

**Traces**: PCL.049

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppExecutor.AddRawContainer`.

### REQ_PCL_149 - Executor SpinOnce Ticks Component
`spinOnce()` shall tick all ACTIVE components added to the executor, incrementing their tick count.

**Traces**: PCL.049

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppExecutor.SpinOnceTicksComponent`.

### REQ_PCL_150 - Executor RequestShutdown Stops Spin
`requestShutdown()` shall cause a running `spin()` loop to return.

**Traces**: PCL.049

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppExecutor.RequestShutdownStopsSpin`.

### REQ_PCL_151 - Executor ShutdownGraceful Finalizes Components
`shutdownGraceful()` shall deactivate and finalize all added components, returning `PCL_OK`.

**Traces**: PCL.049

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppExecutor.ShutdownGracefulFinalizesComponents`.

### REQ_PCL_152 - Executor SetTransport Wires Adapter
`setTransport()` shall wire a transport adapter to the executor; passing NULL shall clear it.

**Traces**: PCL.049

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppExecutor.SetTransport`.

### REQ_PCL_153 - Executor DispatchIncoming Routes To Subscribers
`dispatchIncoming()` shall synchronously route a message to all matching subscribers.

**Traces**: PCL.049

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppExecutor.DispatchIncoming`.

### REQ_PCL_154 - Executor PostIncoming Queues From External Thread
`postIncoming()` shall deep-copy and enqueue a message for dispatch on the next `spinOnce()`.

**Traces**: PCL.049

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppExecutor.PostIncoming`.

### REQ_PCL_155 - Executor Move Constructor Transfers Ownership
The move constructor shall transfer the underlying `pcl_executor_t` handle, leaving the source with a NULL handle.

**Traces**: PCL.049

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppExecutor.MoveConstructor`.

### REQ_PCL_156 - Executor Move Assignment Transfers Ownership
The move assignment operator shall transfer the underlying `pcl_executor_t` handle, destroying any prior handle held by the target.

**Traces**: PCL.049

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppExecutor.MoveAssignment`.

### REQ_PCL_157 - Component Lifecycle Via Executor Integration
A `pcl::Component` added to a `pcl::Executor` shall receive ticks when active and reach FINALIZED state after `shutdownGraceful()`.

**Traces**: PCL.048, PCL.049

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppIntegration.ComponentLifecycleViaExecutor`.

---

## Traceability Summary

| LLR | PCL (HLR) | Test File |
|-----|-----------|-----------|
| 001 | 001, 005 | test_pcl_lifecycle |
| 002 | 045 | test_pcl_lifecycle |
| 003 | 001 | test_pcl_lifecycle |
| 004 | 002, 003 | test_pcl_lifecycle |
| 005 | 002 | test_pcl_lifecycle |
| 006 | 002 | test_pcl_lifecycle |
| 007 | 004 | test_pcl_lifecycle |
| 008 | 003 | test_pcl_lifecycle |
| 009 | 004 | test_pcl_robustness |
| 010 | 004 | test_pcl_robustness |
| 011 | 004 | test_pcl_robustness |
| 012 | 007 | test_pcl_robustness |
| 013 | 012, 014 | test_pcl_lifecycle |
| 014 | 012 | test_pcl_lifecycle |
| 015 | 012 | test_pcl_lifecycle |
| 016 | 013, 046 | test_pcl_robustness |
| 017 | 014 | test_pcl_robustness |
| 018 | 045 | test_pcl_robustness |
| 019 | 007 | test_pcl_lifecycle |
| 020 | 007 | test_pcl_lifecycle |
| 021 | 008, 046 | test_pcl_robustness |
| 022 | 045 | test_pcl_robustness |
| 023 | 009 | test_pcl_robustness |
| 024 | 009 | test_pcl_robustness |
| 025 | 045 | test_pcl_robustness |
| 026 | 009 | test_pcl_robustness |
| 027 | 009, 022 | test_pcl_robustness |
| 028 | 015 | test_pcl_lifecycle |
| 029 | 015 | test_pcl_lifecycle |
| 030 | 016 | test_pcl_lifecycle |
| 031 | 018 | test_pcl_executor |
| 032 | 018, 019 | test_pcl_executor |
| 033 | 018 | test_pcl_executor |
| 034 | 018 | test_pcl_executor |
| 035 | 020 | test_pcl_executor |
| 036 | 021 | test_pcl_executor |
| 037 | 024, 046 | test_pcl_robustness |
| 038 | 045 | test_pcl_robustness |
| 039 | 024 | test_pcl_robustness |
| 040 | 023 | test_pcl_robustness |
| 041 | 021 | test_pcl_robustness |
| 042 | 021 | test_pcl_robustness |
| 043 | 017 | test_pcl_robustness |
| 044 | 022, 010 | test_pcl_executor |
| 045 | 022 | test_pcl_executor |
| 046 | 045 | test_pcl_robustness |
| 047 | 022 | test_pcl_robustness |
| 048 | 028, 029 | test_pcl_robustness |
| 049 | 025 | test_pcl_executor |
| 050 | 025, 045 | test_pcl_robustness |
| 051 | 025 | test_pcl_robustness |
| 052 | 025, 046 | test_pcl_robustness |
| 053 | 025 | test_pcl_robustness |
| 054 | 025, 026 | test_pcl_robustness |
| 055 | 020, 025 | test_pcl_robustness |
| 056 | 025 | test_pcl_robustness |
| 057 | 027 | test_pcl_robustness |
| 058 | 027 | test_pcl_robustness |
| 059 | 027 | test_pcl_robustness |
| 060 | 027, 045 | test_pcl_robustness |
| 061 | 027, 046 | test_pcl_robustness |
| 062 | 029 | test_pcl_robustness |
| 063 | 029, 045 | test_pcl_robustness |
| 064 | 037, 038 | test_pcl_log |
| 065 | 037 | test_pcl_log |
| 066 | 039 | test_pcl_log |
| 067 | 037 | test_pcl_log |
| 068 | 038 | test_pcl_log |
| 069 | 040 | test_pcl_robustness |
| 070 | 039 | test_pcl_robustness |
| 071 | 037 | test_pcl_robustness |
| 072 | 038 | test_pcl_robustness |
| 073 | 037 | test_pcl_robustness |
| 074 | 037 | test_pcl_robustness |
| 075 | 041 | test_pcl_dining |
| 076 | 044 | test_pcl_dining |
| 077 | 044 | test_pcl_dining |
| 078 | 044 | test_pcl_dining |
| 079 | 042 | test_pcl_dining |
| 080 | 043 | test_pcl_dining |
| 081 | 041 | test_pcl_dining |
| 082 | 041, 046 | test_pcl_dining |
| 083 | 041 | test_pcl_dining |
| 084 | 018, 042 | test_pcl_dining |
| 085 | 046 | test_pcl_oom |
| 086 | 046 | test_pcl_oom |
| 087 | 046 | test_pcl_oom |
| 088 | 046 | test_pcl_oom |
| 089 | 046 | test_pcl_oom |
| 090 | 046 | test_pcl_oom |
| 091 | 018 | test_pcl_robustness |
| 092 | 015 | test_pcl_robustness |
| 093 | 018 | test_pcl_robustness |
| 094 | 050 | test_pcl_proto_bindings |
| 095 | 051 | test_pcl_proto_bindings |
| 096 | 050 | test_pcl_proto_bindings |
| 097 | 051 | test_pcl_proto_bindings |
| 098 | 053 | test_pcl_proto_bindings |
| 099 | 053 | test_pcl_proto_bindings |
| 100 | 053 | test_pcl_proto_bindings |
| 101 | 053 | test_pcl_proto_bindings |
| 102 | 052 | test_pcl_proto_bindings |
| 103 | 052 | test_pcl_proto_bindings |
| 104 | 054 | test_pcl_proto_bindings |
| 105 | 054 | test_pcl_proto_bindings |
| 106 | 009 | test_pcl_proto_bindings |
| 107 | 009 | test_pcl_proto_bindings |
| 108 | 055 | test_pcl_proto_bindings |
| 109 | 055 | test_pcl_proto_bindings |
| 110 | 052 | test_pcl_proto_bindings |
| 111 | 045 | test_pcl_lifecycle |
| 112 | 045 | test_pcl_executor |
| 113 | 045 | test_pcl_robustness |
| 114 | 005, 045 | test_pcl_robustness |
| 115 | 031 | test_pcl_socket_transport |
| 116 | 031, 045 | test_pcl_socket_transport |
| 117 | 032 | test_pcl_socket_transport |
| 118 | 032, 045 | test_pcl_socket_transport |
| 119 | 032 | test_pcl_socket_transport |
| 120 | 033 | test_pcl_socket_transport |
| 121 | 033 | test_pcl_socket_transport |
| 122 | 034 | test_pcl_socket_transport |
| 123 | 034, 045 | test_pcl_socket_transport |
| 124 | 034 | test_pcl_socket_transport |
| 125 | 035 | test_pcl_socket_transport |
| 126 | 036 | test_pcl_socket_transport |
| 127 | 036, 045 | test_pcl_socket_transport |
| 128 | 036 | test_pcl_socket_transport |
| 129 | 045 | test_pcl_socket_transport |
| 130 | 045 | test_pcl_socket_transport |
| 131 | 048 | test_pcl_cpp_wrappers |
| 132 | 048 | test_pcl_cpp_wrappers |
| 133 | 048 | test_pcl_cpp_wrappers |
| 134 | 048 | test_pcl_cpp_wrappers |
| 135 | 048 | test_pcl_cpp_wrappers |
| 136 | 048 | test_pcl_cpp_wrappers |
| 137 | 048 | test_pcl_cpp_wrappers |
| 138 | 048 | test_pcl_cpp_wrappers |
| 139 | 048 | test_pcl_cpp_wrappers |
| 140 | 048 | test_pcl_cpp_wrappers |
| 141 | 048 | test_pcl_cpp_wrappers |
| 142 | 048 | test_pcl_cpp_wrappers |
| 143 | 048 | test_pcl_cpp_wrappers |
| 144 | 048 | test_pcl_cpp_wrappers |
| 145 | 049 | test_pcl_cpp_wrappers |
| 146 | 049 | test_pcl_cpp_wrappers |
| 147 | 049 | test_pcl_cpp_wrappers |
| 148 | 049 | test_pcl_cpp_wrappers |
| 149 | 049 | test_pcl_cpp_wrappers |
| 150 | 049 | test_pcl_cpp_wrappers |
| 151 | 049 | test_pcl_cpp_wrappers |
| 152 | 049 | test_pcl_cpp_wrappers |
| 153 | 049 | test_pcl_cpp_wrappers |
| 154 | 049 | test_pcl_cpp_wrappers |
| 155 | 049 | test_pcl_cpp_wrappers |
| 156 | 049 | test_pcl_cpp_wrappers |
| 157 | 048, 049 | test_pcl_cpp_wrappers |
| 158 | 031 | test_pcl_socket_transport |
| 159 | 031 | test_pcl_socket_transport |
| 160 | 036 | test_pcl_socket_transport |
| 161 | 035 | test_pcl_socket_transport |
| 162 | 036 | test_pcl_socket_transport |
| 163 | 034, 036 | test_pcl_socket_transport |
| 164 | 030a | test_pcl_robustness |
| 165 | 030a, 045 | test_pcl_robustness |
| 166 | 030a, 045 | test_pcl_robustness |
| 173 | 030b, 030d | test_pcl_executor |
| 174 | 030b, 030c | test_pcl_executor |
| 175 | 030b, 030c | test_pcl_executor |
| 176 | 033, 036a | test_pcl_socket_transport |
| 177 | 034, 036, 036a | test_pcl_socket_transport |
| 178 | 030c | test_pcl_executor |
| 179 | 036a, 056 | test_pcl_socket_transport |
| 186 | 036b, 036c | test_pcl_shared_memory_transport |
| 187 | 036b, 036c | test_pcl_shared_memory_transport |
| 188 | 036b, 036d | test_pcl_shared_memory_transport |
| 189 | 036d, 045 | test_pcl_shared_memory_transport |
| 211 | 036c, 036g | test_pcl_shared_memory_transport |
| 212 | 036g, 045 | test_pcl_shared_memory_transport |
| 167 | 011c | test_pcl_streaming |
| 168 | 011c | test_pcl_streaming |
| 169 | 011c | test_pcl_streaming |
| 170 | 011c, 045 | test_pcl_streaming |
| 171 | 011c | test_pcl_streaming |
| 172 | 011c | test_pcl_streaming |
| 213 | 074 | test_pcl_alloc |
| 214 | 074 | test_pcl_alloc |
| 215 | 074 | test_pcl_alloc |
| 216 | 074, 045 | test_pcl_alloc |
| 217 | 048 | test_pcl_cpp_wrappers |
| 275 | 070 | test_pcl_transport_routing |
| 286 | 071 | test_pcl_template_transport |
| 288 | 071 | test_pcl_template_transport |
| 290 | 071 | test_pcl_template_transport |
| 292 | 071 | test_pcl_template_transport |
| 293 | 071 | test_pcl_template_transport |
| 294 | 071 | test_pcl_template_transport |
| 295 | 071 | test_pcl_template_transport |
| 297 | 073 | test_pcl_apos_transport |
| 298 | 073, 045 | test_pcl_apos_transport |
| 300 | 073 | test_pcl_apos_transport |
| 301 | 036f | test_pcl_udp_transport |
| 302 | 036f, 045 | test_pcl_udp_transport |
| 303 | 036f | test_pcl_udp_transport |
| 304 | 036f | test_pcl_udp_transport |
| 305 | 036b | test_pcl_shared_memory_transport |
| 306 | 030d, 036b | test_pcl_shared_memory_transport |
| 307 | 036b | test_pcl_shared_memory_transport |
| 308 | 036d | test_pcl_shared_memory_transport |
| 309 | 036d, 046 | test_pcl_shared_memory_transport |
| 310 | 036b, 046 | test_pcl_shared_memory_transport |
| 311 | 030d, 011c | test_pcl_shared_memory_transport |
| 312 | 011c | test_pcl_shared_memory_transport |
| 313 | 036b | test_pcl_shared_memory_transport |
| 314 | 030d, 036b | test_pcl_shared_memory_transport |
| 315 | 036b | test_pcl_shared_memory_transport |
| 316 | 036b | test_pcl_shared_memory_transport |
| 317 | 069, 045 | test_pcl_transport_routing |
| 318 | 070, 046 | test_pcl_transport_routing |
| 319 | 070, 046 | test_pcl_transport_routing |
| 320 | 071 | test_pcl_template_transport |
| 321 | 071 | test_pcl_template_transport |
| 322 | 073 | test_pcl_apos_transport |
| 323 | 073 | test_pcl_apos_transport |
| 324 | 073 | test_pcl_apos_transport |
| 325 | 073 | test_pcl_apos_transport |
| 326 | 036b | test_pcl_shared_memory_transport |
| 327 | 030d, 036b | test_pcl_shared_memory_transport |
| 328 | 041 | test_pcl_bridge |
| 337 | 059 | test_pcl_codec_registry |
| 338 | 059 | test_pcl_codec_registry |
| 339 | 059 | test_pcl_codec_registry |
| 340 | 059, 045 | test_pcl_codec_registry |
| 341 | 059 | test_pcl_codec_registry |
| 342 | 059 | test_pcl_codec_registry |
| 343 | 059 | test_pcl_codec_registry |
| 344 | 059 | test_pcl_codec_registry |
| 345 | 059 | test_pcl_codec_registry |
| 346 | 058 | test_pcl_codec_registry |
| 347 | 060, 045 | test_pcl_capabilities |
| 348 | 060 | test_pcl_capabilities |
| 349 | 060 | test_pcl_capabilities |
| 350 | 060 | test_pcl_capabilities |
| 351 | 060 | test_pcl_capabilities |
| 352 | 060 | test_pcl_capabilities |
| 353 | 065, 045 | test_pcl_capabilities |
| 354 | 060, 064 | test_pcl_capabilities |
| 355 | 060, 064 | test_pcl_capabilities |
| 356 | 068 | test_pcl_capabilities |
| 357 | 068 | test_pcl_capabilities |
| 358 | 068 | test_pcl_capabilities |
| 359 | 068, 062 | test_pcl_capabilities |
| 360 | 068, 062 | test_pcl_capabilities |
| 361 | 068, 062 | test_pcl_capabilities |
| 362 | 065, 045 | test_pcl_capabilities |
| 363 | 061 | test_pcl_capabilities |
| 364 | 060 | test_pcl_capabilities |
| 365 | 063 | test_pcl_capabilities |
| 366 | 063 | test_pcl_capabilities |
| 367 | 063 | test_pcl_capabilities |
| 368 | 063 | test_pcl_capabilities |
| 369 | 063 | test_pcl_capabilities |
| 370 | 062 | test_pcl_capabilities |
| 371 | 063, 062 | test_pcl_capabilities |
| 372 | 063, 062 | test_pcl_capabilities |
| 373 | 063, 062 | test_pcl_capabilities |
| 374 | 063, 062 | test_pcl_capabilities |
| 375 | 061 | test_pcl_capabilities |
| 376 | 062 | test_pcl_capabilities |
| 377 | 062, 045 | test_pcl_capabilities |
| 378 | 062, 045 | test_pcl_capabilities |
| 379 | 063 | test_pcl_capabilities |
| 380 | 063 | test_pcl_capabilities |
| 381 | 067, 045 | test_pcl_plugin_loader |
| 382 | 067 | test_pcl_plugin_loader |
| 383 | 064 | test_pcl_plugin_loader |
| 384 | 064, 029 | test_pcl_plugin_loader |
| 385 | 066, 059 | test_pcl_plugin_loader |
| 386 | 065 | test_pcl_plugin_loader |
| 387 | 068 | test_pcl_plugin_loader |
| 388 | 068, 045 | test_pcl_plugin_loader |
| 389 | 068 | test_pcl_plugin_loader |
| 390 | 068, 045 | test_pcl_plugin_loader |
| 391 | 066 | test_pcl_plugin_loader |
| 392 | 066 | test_pcl_plugin_loader |
| 393 | 065 | test_pcl_plugin_loader |
| 394 | 065 | test_pcl_plugin_loader |
| 395 | 065, 059 | test_pcl_plugin_loader |
| 396 | 065 | test_pcl_plugin_loader |
| 397 | 065, 064 | test_pcl_plugin_loader |
| 398 | 065 | test_pcl_plugin_loader |
| 399 | 066, 045 | test_pcl_plugin_loader |
| 400 | 066 | test_pcl_plugin_loader |
| 401 | 066, 045 | test_pcl_plugin_loader |
| 402 | 065 | test_pcl_plugin_loader |
| 403 | 068, 067 | test_pcl_plugin_loader |
| 404 | 068, 045 | test_pcl_plugin_loader |
| 405 | 066 | test_pcl_plugin_loader |
| 406 | 068, 045 | test_pcl_plugin_loader |
| 407 | 067, 068 | test_pcl_plugin_loader |
| 408 | 068, 045 | test_pcl_plugin_loader |
| 409 | 034, 046 | test_pcl_socket_faults |
| 410 | 036, 046 | test_pcl_socket_faults |
| 411 | 036, 046 | test_pcl_socket_faults |
| 412 | 033 | test_pcl_socket_faults |
| 413 | 033, 046 | test_pcl_socket_faults |
| 414 | 036e | test_pcl_socket_faults |
| 415 | 036e | test_pcl_socket_faults |
| 416 | 069, 070 | test_pcl_transport_routing |
| 417 | 069 | test_pcl_transport_routing |
| 418 | 069 | test_pcl_transport_routing |
| 419 | 069 | test_pcl_transport_routing |
| 420 | 070, 063 | test_pcl_transport_routing |
| 421 | 070 | test_pcl_transport_routing |
| 422 | 070 | test_pcl_transport_routing |
| 423 | 070 | test_pcl_transport_routing |
| 424 | 070 | test_pcl_transport_routing |
| 425 | 069, 070 | test_pcl_transport_routing |
| 426 | 070, 062 | test_pcl_transport_routing |
| 427 | 070 | test_pcl_transport_routing |
| 428 | 070 | test_pcl_transport_routing |
| 429 | 071, 045 | test_pcl_template_transport |
| 430 | 071 | test_pcl_template_transport |
| 431 | 071, 045 | test_pcl_template_transport |
| 432 | 071, 045 | test_pcl_template_transport |
| 433 | 072 | test_pcl_template_transport |
| 434 | 071 | test_pcl_template_transport |
| 435 | 071 | test_pcl_template_transport |
| 436 | 071 | test_pcl_template_transport |
| 437 | 071 | test_pcl_template_transport |
| 438 | 071, 030d | test_pcl_template_transport |
| 439 | 071 | test_pcl_template_transport |
| 440 | 071 | test_pcl_template_transport |
| 441 | 072 | test_pcl_template_transport |
| 442 | 072 | test_pcl_template_transport |
| 443 | 072 | test_pcl_template_transport |
| 444 | 073 | test_pcl_apos_transport |
| 445 | 073, 072 | test_pcl_apos_transport |
| 446 | 073, 072 | test_pcl_apos_transport |
| 447 | 073, 072 | test_pcl_apos_transport |

---

## 18. Endpoint Routing

### REQ_PCL_173 - Remote Ingress Honors Subscriber Peer Route
`pcl_executor_post_remote_incoming()` shall deliver a queued remote message only to subscriber ports whose route allows remote delivery and whose configured peer allow-list matches the source peer.

**Traces**: PCL.030b, PCL.030d

**Verification**: `test_pcl_executor.cpp::RemoteIngressHonorsSubscriberPeerRoute`.

### REQ_PCL_174 - Publisher Route Can Fan Out Local And Remote
`pcl_port_publish()` on a publisher configured for local-plus-remote routing shall dispatch to matching local subscribers and publish to each configured remote peer transport.

**Traces**: PCL.030b, PCL.030c

**Verification**: `test_pcl_executor.cpp::PublisherRouteCanBeLocalAndRemote`.

### REQ_PCL_175 - Consumed Service Route Uses Named Peer Transport
`pcl_executor_invoke_async()` shall use the named peer transport selected by a consumed-service endpoint route when the route is configured as remote.

**Traces**: PCL.030b, PCL.030c

**Verification**: `test_pcl_executor.cpp::ConsumedServiceRouteUsesNamedPeerTransport`.

### REQ_PCL_176 - Socket Publish Uses Peer Identity For Remote Delivery
The socket transport shall tag inbound published messages with its configured peer identity so remote subscriber routes can be filtered by peer.

**Traces**: PCL.033, PCL.036a

**Verification**: `test_pcl_socket_transport.cpp::PublishServerToClientDelivered`, `test_pcl_socket_transport.cpp::PublishClientToServerDelivered`.

### REQ_PCL_177 - Remote Service Round Trip Respects Peer Routes
The socket transport gateway shall dispatch inbound service requests through remote-aware executor service lookup so that remote provided-service routes and peer allow-lists are enforced.

**Traces**: PCL.034, PCL.036, PCL.036a

**Verification**: `test_pcl_socket_transport.cpp::AsyncRemoteServiceRoundTrip`.

### REQ_PCL_178 - Named Transport Registration Selects Correct Peer
`pcl_executor_register_transport()` shall register a transport under a peer id and make that transport available to endpoint route resolution for remote publish and invoke operations.

**Traces**: PCL.030c

**Verification**: `test_pcl_executor.cpp::PublisherRouteCanBeLocalAndRemote`, `test_pcl_executor.cpp::ConsumedServiceRouteUsesNamedPeerTransport`.

### REQ_PCL_179 - Socket Transport Peer Identifier Is Configurable
`pcl_socket_transport_set_peer_id()` shall store a logical peer identifier used for remote ingress and remote service dispatch decisions.

**Traces**: PCL.036a, PCL.056

**Verification**: `test_pcl_socket_transport.cpp::PublishServerToClientDelivered`, `test_pcl_socket_transport.cpp::PublishClientToServerDelivered`, `test_pcl_socket_transport.cpp::AsyncRemoteServiceRoundTrip`.

---

## 19. Shared Memory Transport

### REQ_PCL_186 - Inter-Process Publish Parent To Child Delivered
The shared-memory transport shall deliver a published message from one process to a subscriber hosted in a different process on the same named bus.

**Traces**: PCL.036b, PCL.036c

**Verification**: `test_pcl_shared_memory_transport.cpp::InterProcessPublishParentToChildDelivered`.

### REQ_PCL_187 - Inter-Process Publish Child To Parent Delivered
The shared-memory transport shall deliver a published message from a helper child process to a subscriber hosted in the parent test process, preserving the source participant identity for peer filtering.

**Traces**: PCL.036b, PCL.036c

**Verification**: `test_pcl_shared_memory_transport.cpp::InterProcessPublishChildToParentDelivered`.

### REQ_PCL_188 - Inter-Process Async Service Round Trip
The shared-memory transport shall route an async remote service request across processes to the unique advertised provider and deliver the response callback on the caller's executor thread.

**Traces**: PCL.036b, PCL.036d

**Verification**: `test_pcl_shared_memory_transport.cpp::InterProcessAsyncServiceRoundTrip`.

### REQ_PCL_189 - Shared Memory Invoke Async Without Provider Returns Not Found
`pcl_executor_invoke_async()` shall return `PCL_ERR_NOT_FOUND` when the shared-memory bus does not advertise any provider for the requested remote service.

**Traces**: PCL.036d, PCL.045

**Verification**: `test_pcl_shared_memory_transport.cpp::InvokeAsyncReturnsNotFoundWithoutProvider`.

### REQ_PCL_211 - Shared Memory Publish Fan-Out Is Atomic
When a shared-memory publish targets multiple participants, the transport shall pre-check capacity in every target mailbox while holding the bus lock. If any target mailbox is full, the publish shall return an error and shall not enqueue that frame to any target mailbox.

**Traces**: PCL.036c, PCL.036g

**Verification**: `test_pcl_shared_memory_transport.cpp::PublishFanoutIsAtomicWhenAnyMailboxIsFull`.

### REQ_PCL_212 - Shared Memory Topic Backpressure Waits For Capacity
`pcl_shared_memory_transport_set_topic_backpressure()` shall configure a per-topic bounded wait policy. Publishes on configured topics shall wait for all target mailboxes to have capacity until the timeout expires, while topics without a policy shall remain on the generic non-blocking output path. The API shall reject NULL transport handles or NULL/empty topic names.

**Traces**: PCL.036g, PCL.045

**Verification**: `test_pcl_shared_memory_transport.cpp::TopicBackpressureWaitsForMailboxCapacity`.

---

## 20. Streaming Services

### REQ_PCL_167 - Streaming Service Send and End
`pcl_stream_send()` shall deliver messages to the client callback with `end=false`. `pcl_stream_end()` shall deliver a final callback with `end=true` and `status=PCL_OK`.

**Traces**: PCL.011c

**Verification**: `test_pcl_streaming.cpp::PclStreaming.BasicStreamingSendEnd`, `PclStreaming.TransportStreamSendEnd`, `PclStreaming.TransportInvokeStream`.

### REQ_PCL_168 - Client Stream Cancellation
`pcl_stream_cancel()` shall set the cancelled flag. `pcl_stream_is_cancelled()` shall return true after cancellation. `pcl_stream_send()` shall return `PCL_ERR_CANCELLED` when cancelled.

**Traces**: PCL.011c

**Verification**: `test_pcl_streaming.cpp::PclStreaming.ClientCancellation`, `PclStreaming.TransportStreamCancel`.

### REQ_PCL_169 - Server Stream Abort
`pcl_stream_abort()` shall deliver a final callback with `end=true` and the specified error status.

**Traces**: PCL.011c

**Verification**: `test_pcl_streaming.cpp::PclStreaming.ServerAbort`, `PclStreaming.TransportStreamAbort`.

### REQ_PCL_170 - Stream Service Not Found
`pcl_executor_invoke_stream()` shall return `PCL_ERR_NOT_FOUND` when no streaming service matches the name.

**Traces**: PCL.011c, PCL.045

**Verification**: `test_pcl_streaming.cpp::PclStreaming.StreamNotFound`, `PclStreaming.HandlerReturnsError`.

### REQ_PCL_171 - Stream API Null Safety
All streaming API functions shall return `PCL_ERR_INVALID` or safe defaults when passed NULL arguments.

**Traces**: PCL.011c

**Verification**: `test_pcl_streaming.cpp::PclStreaming.StreamNullSafety`.

### REQ_PCL_172 - Add Stream Service During Configure
`pcl_container_add_stream_service()` shall create a streaming service port during `on_configure` and return a non-NULL port handle.

**Traces**: PCL.011c

**Verification**: `test_pcl_streaming.cpp::PclStreaming.AddStreamServiceDuringConfigure`.

---

## 17. Cross-Thread Local Service Request Queuing

### REQ_PCL_180 - Post Service Request Deep Copies Request
`pcl_executor_post_service_request()` shall deep-copy the service name, type name, and request payload bytes before returning so the caller may release or reuse its buffers immediately.

**Traces**: PCL.057

**Verification**: `test_pcl_robustness.cpp::PostServiceRequestDeepCopiesRequest` posts a request then overwrites the source buffer and verifies the handler receives the original bytes.

### REQ_PCL_181 - Post Service Request Fires Handler On Executor Thread
The service handler and response callback enqueued by `pcl_executor_post_service_request()` shall both execute on the executor thread during the next `pcl_executor_spin_once` or `pcl_executor_spin` cycle.

**Traces**: PCL.057

**Verification**: `test_pcl_robustness.cpp::PostServiceRequestFiresHandlerOnExecutorThread` posts from a background thread and records the thread ID in the handler; asserts it matches the spin thread.

### REQ_PCL_182 - Post Service Request Service Not Found Fires Empty Callback
When the named service is not registered, `pcl_executor_post_service_request()` shall still enqueue the request. During drain, the response callback shall be fired with an empty (zero-size, NULL-data) message so the caller is not silently abandoned.

**Traces**: PCL.057

**Verification**: `test_pcl_robustness.cpp::PostServiceRequestNotFoundFiresEmptyCallback`.

### REQ_PCL_183 - Post Service Request Multiple Queued Fire In Order
Multiple requests queued via `pcl_executor_post_service_request()` before a spin shall all be drained and their callbacks fired in FIFO order during that spin.

**Traces**: PCL.057

**Verification**: `test_pcl_robustness.cpp::PostServiceRequestMultipleQueuedFireInOrder`.

### REQ_PCL_184 - Post Service Request Null Safety
`pcl_executor_post_service_request()` shall return `PCL_ERR_INVALID` for NULL executor, NULL service name, NULL request, or NULL callback.

**Traces**: PCL.057, PCL.045

**Verification**: `test_pcl_robustness.cpp::PostServiceRequestNullSafety`.

### REQ_PCL_185 - Destroy With Pending Service Requests Frees Memory
Destroying an executor that has queued service requests (not yet drained) shall free all pending nodes without crashing.

**Traces**: PCL.057, PCL.046

**Verification**: `test_pcl_robustness.cpp::DestroyWithPendingServiceRequestsFrees`.

---

## 21. Portable Allocator

### REQ_PCL_213 - Alloc And Free Round Trip
`pcl_alloc()` shall return usable memory for a nonzero size and NULL for a zero size; `pcl_free()` shall release it.

**Traces**: PCL.074

**Verification**: `test_pcl_alloc.cpp::PclAlloc.AllocAndFreeRoundTrip`.

### REQ_PCL_214 - Calloc Zeroes And Rejects Overflow
`pcl_calloc()` shall zero-initialize the returned memory and reject zero counts and an `nmemb * size` product that overflows `size_t` with NULL.

**Traces**: PCL.074

**Verification**: `test_pcl_alloc.cpp::PclAlloc.CallocZeroesAndRejectsOverflow`.

### REQ_PCL_215 - Realloc Semantics
`pcl_realloc(NULL, size)` shall behave as `pcl_alloc()`; growing an allocation shall preserve its contents; `pcl_realloc(ptr, 0)` shall free `ptr` and return NULL.

**Traces**: PCL.074

**Verification**: `test_pcl_alloc.cpp::PclAlloc.ReallocSemantics`.

### REQ_PCL_216 - Free Null Is No-Op
`pcl_free(NULL)` shall be a safe no-op.

**Traces**: PCL.074, PCL.045

**Verification**: `test_pcl_alloc.cpp::PclAlloc.FreeNullIsNoOp`.

### REQ_PCL_217 - Component Default Callbacks Are OK No-Ops
A `pcl::Component` subclass overriding none of the virtual lifecycle methods shall complete the full lifecycle and receive ticks, since every default returns `PCL_OK`.

**Traces**: PCL.048

**Verification**: `test_pcl_cpp_wrappers.cpp::PclCppComponent.DefaultCallbacksAreOkNoOps`.

---

## 22. Transport Template Scaffold

### REQ_PCL_286 - Set Peer Id Idempotent For Same Value
Re-registering the transport template's current peer id shall succeed without creating a duplicate alias entry.

**Traces**: PCL.071

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.SetPeerIdSameValueIsIdempotent`.

### REQ_PCL_288 - Destroy Clears Default Transport Slot When Self
Destroying a template transport that is installed as the executor's default transport shall clear the default slot.

**Traces**: PCL.071

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.DestroyClearsDefaultTransportWhenSelf`.

### REQ_PCL_290 - Destroy Drains Queued Frames Behind A Slow Send Hook
Frames queued behind a blocked `send_blocking` hook shall accumulate in FIFO order; destroying the transport shall drain the backlog without leaking.

**Traces**: PCL.071

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.DestroyDrainsQueuedFramesBehindSlowSend`.

### REQ_PCL_292 - Send Hook Failure Drops The Frame
When `send_blocking` returns a non-OK status, the template shall log a warning, drop the frame, and continue servicing later frames.

**Traces**: PCL.071

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.SendHookFailureDropsFrame`.

### REQ_PCL_293 - Recv Hook Hard Failure Retries Until Destroy
When `recv_blocking` returns a hard error (not a timeout), the template shall log and retry rather than exit, and shall still tear down cleanly on destroy.

**Traces**: PCL.071

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.RecvHookHardFailureRetriesUntilDestroy`.

### REQ_PCL_294 - Malformed Inbound Frames Ignored
Inbound frames with an unrecognised kind, or an SVC_RESP whose sequence id matches no pending request, shall be ignored without crashing the receive thread.

**Traces**: PCL.071

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.MalformedInboundFramesIgnored`.

### REQ_PCL_295 - Subscribe Vtable Is A Wire-Agnostic No-Op
The template's `subscribe` vtable entry shall return `PCL_OK` without action, since the engineer's `recv_blocking` already surfaces every inbound frame.

**Traces**: PCL.071

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.SubscribeVtableIsNoOp`.

### REQ_PCL_320 - Publish And Invoke Fail After Vtable Shutdown
After the vtable `shutdown` function has stopped the send thread, `publish` and `invoke_async` shall return `PCL_ERR_STATE`, and any request already registered shall be rolled back rather than orphaned.

**Traces**: PCL.071

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.PublishAndInvokeFailAfterVtableShutdown`.

### REQ_PCL_321 - Destroy Frees Unanswered Pending Async Requests
Destroying a transport with async requests still awaiting a response shall free every pending correlation entry without leaking or crashing.

**Traces**: PCL.071

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.DestroyFreesUnansweredPendingRequests`.

---

## 23. APOS Transport

### REQ_PCL_297 - Apos Stub Blocking Wrappers And Delay Function
The APOS stub's blocking `sendMessage`/`receiveMessage` wrappers shall delegate to their non-blocking counterparts with a timeout, `setupApos` shall alias `setupAPOS`, `logEvent` shall be a safe no-op, and an installed delay callback shall fire once per non-blocking send.

**Traces**: PCL.073

**Verification**: `test_pcl_apos_transport.cpp::AposStub.BlockingWrappersAndDelayFunction`.

### REQ_PCL_298 - Apos Transport Create Without Executor Returns Null
`pcl_apos_transport_create()` shall return NULL when the executor argument is NULL; `pcl_apos_transport_destroy(NULL)` shall be a safe no-op.

**Traces**: PCL.073, PCL.045

**Verification**: `test_pcl_apos_transport.cpp::PclTransportApos.CreateWithoutExecutorReturnsNull`.

### REQ_PCL_300 - Oversized Topic Rejected At Encode
A frame whose topic, service name, or type name exceeds the 16-bit wire length limit shall be rejected at encode time on the send thread and dropped, rather than truncated onto the wire.

**Traces**: PCL.073

**Verification**: `test_pcl_apos_transport.cpp::PclTransportApos.OversizedTopicRejectedAtEncode`.

### REQ_PCL_322 - Apos Stub Non-Blocking Receive Edge Cases
`receiveMessageNonBlocking()` shall report `RS_RESOURCE` for an empty channel and `RS_ERROR` (with the true size reported) for a message larger than the caller's buffer, leaving the message queued for a subsequent adequately-sized read.

**Traces**: PCL.073

**Verification**: `test_pcl_apos_transport.cpp::AposStub.NonBlockingReceiveEdgeCases`.

### REQ_PCL_323 - Apos Stub Infinite Wait Woken By Producer
`waitOnMultiChannel()` with a NULL timeout shall block until another thread queues data on one of the watched channels, and shall reject NULL channel arrays, a non-positive channel count, or a NULL output array with `TM_ERROR`.

**Traces**: PCL.073

**Verification**: `test_pcl_apos_transport.cpp::AposStub.InfiniteWaitWokenByProducerThread`.

### REQ_PCL_324 - Apos Stub Blocking Receive Undersized Buffer Is Error
A blocking `receiveMessage()` whose message exceeds the caller's buffer shall report `TM_ERROR` (mapped from the non-blocking `RS_ERROR`) with the true size reported, leaving the message available for a retry with an adequate buffer.

**Traces**: PCL.073

**Verification**: `test_pcl_apos_transport.cpp::AposStub.BlockingReceiveUndersizedBufferIsError`.

### REQ_PCL_325 - Malformed Inbound Apos Messages Dropped
Malformed inbound APOS messages -- zero-length, an unrecognised frame kind, truncated headers, or a topic/type length pointing past the message end -- shall be dropped by the receive thread without crashing.

**Traces**: PCL.073

**Verification**: `test_pcl_apos_transport.cpp::PclTransportApos.MalformedInboundAposMessagesDropped`.

---

## 24. UDP Transport Edge Cases

### REQ_PCL_301 - Udp Subscribe And Shutdown Vtable
The UDP transport's `subscribe` vtable entry shall return `PCL_OK` as a no-op; `shutdown` shall stop the receive thread and be NULL-safe.

**Traces**: PCL.036f

**Verification**: `test_pcl_udp_transport.cpp::PclUdpTransport.SubscribeAndShutdownVtable`.

### REQ_PCL_302 - Unresolvable Remote Host Returns Null
`pcl_udp_transport_create()` shall return NULL when the remote host cannot be resolved.

**Traces**: PCL.036f, PCL.045

**Verification**: `test_pcl_udp_transport.cpp::PclUdpTransport.UnresolvableRemoteHostReturnsNull`.

### REQ_PCL_303 - Malformed Datagrams Dropped
Malformed inbound datagrams -- an unrecognised message type, or a topic/type/payload length exceeding the datagram size -- shall be dropped by the receive thread without crashing or wedging.

**Traces**: PCL.036f

**Verification**: `test_pcl_udp_transport.cpp::PclUdpTransport.MalformedDatagramsDropped`.

### REQ_PCL_304 - Destroy Clears Default Transport Slot When Self
Destroying a UDP transport installed as the executor's default transport shall clear the default slot.

**Traces**: PCL.036f

**Verification**: `test_pcl_udp_transport.cpp::PclUdpTransport.DestroyClearsDefaultTransportWhenSelf`.

---

## 25. Shared Memory Transport Edge Cases

### REQ_PCL_305 - Shm Subscribe And Shutdown Vtable Are No-Ops
The shared-memory transport's `subscribe` and `shutdown` vtable entries shall be no-ops, since bus interest and teardown are managed through the bus region itself.

**Traces**: PCL.036b

**Verification**: `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.SubscribeAndShutdownVtableAreNoOps`.

### REQ_PCL_306 - Remote Service Request From Disallowed Peer Yields Empty Response
A remote unary service request from a peer outside the provider's route allow-list shall be answered with an empty response instead of dispatching the service handler.

**Traces**: PCL.030d, PCL.036b

**Verification**: `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.ServicePeerFilterYieldsEmptyResponse`.

### REQ_PCL_307 - Remote Service Handler Error Yields Empty Response
When a remote unary service handler returns a non-OK, non-pending status, the gateway shall send an empty response rather than leaving the caller waiting.

**Traces**: PCL.036b

**Verification**: `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.ServiceHandlerErrorYieldsEmptyResponse`.

### REQ_PCL_308 - Ambiguous Provider Fails Closed
When two participants advertise the same service name, both unary invocation (`pcl_executor_invoke_async()`) and streaming invocation (`invoke_stream`) shall fail closed with `PCL_ERR_INVALID` rather than picking an arbitrary provider.

**Traces**: PCL.036d

**Verification**: `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.AmbiguousProviderFailsClosed`.

### REQ_PCL_309 - Destroy With Pending Requests Cancels Them
Destroying a participant with un-answered unary and stream requests shall free the pending unary correlation entries and shall fire the stream callback with `end=true, status=PCL_ERR_CANCELLED` for any pending stream.

**Traces**: PCL.036d, PCL.046

**Verification**: `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.DestroyWithPendingRequestsCancelsThem`.

### REQ_PCL_310 - Participant Slots Exhausted
The shared-memory bus shall support at most 8 participants; `pcl_shared_memory_transport_create()` shall return NULL when the bus is full.

**Traces**: PCL.036b, PCL.046

**Verification**: `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.ParticipantSlotsExhausted`.

### REQ_PCL_311 - Stream Request From Disallowed Peer Ends With Error
A streaming service request from a peer outside the provider's route allow-list shall be terminated with an error END frame instead of dispatching the stream handler.

**Traces**: PCL.030d, PCL.011c

**Verification**: `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.StreamPeerFilterEndsWithError`.

### REQ_PCL_312 - Stream Handler Error Ends Stream
When a stream handler returns an error instead of `PCL_STREAMING`, the gateway shall send an END frame carrying that error status.

**Traces**: PCL.011c

**Verification**: `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.StreamHandlerErrorEndsStream`.

### REQ_PCL_313 - Service Advertisement Table Dedupes And Caps
The per-participant service advertisement table shall deduplicate repeated service names and cap at 16 entries; a local-only service shall never be advertised.

**Traces**: PCL.036b

**Verification**: `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.ServiceTableDedupesAndCaps`.

### REQ_PCL_314 - Stale Advertisement Refused At Dispatch
A request that arrives after the provider re-routed its service to local-only shall be refused at dispatch time: unary requests receive an empty response, stream requests receive an error END frame, and the handler is never invoked.

**Traces**: PCL.030d, PCL.036b

**Verification**: `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.StaleAdvertisementRefusedAtDispatch`.

### REQ_PCL_315 - Service Without Route Uses Default Modes
A service port with no explicit route configuration shall fall back to the legacy default (advertised remotely when a default transport is attached), and the round trip shall complete normally.

**Traces**: PCL.036b

**Verification**: `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.ServiceWithoutRouteUsesDefaultModes`.

### REQ_PCL_316 - Service Without Route Or Transport Stays Local
A service port with no explicit route and no default transport attached shall stay local-only and never be advertised on the bus.

**Traces**: PCL.036b

**Verification**: `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.ServiceWithoutRouteOrTransportStaysLocal`.

### REQ_PCL_326 - Publisher Without Route Uses Default Modes
A publisher port with no explicit endpoint route shall fall back to the legacy default routing mode when publishing through the shared-memory transport.

**Traces**: PCL.036b

**Verification**: `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.PublishWithoutRoutesUsesDefaultModes`.

### REQ_PCL_327 - Remote Ingress Default Route Modes
A subscriber port with no explicit route shall accept remote ingress when a default transport is attached to its executor, exercising the legacy route-mode fallback alongside an unrelated route-table entry.

**Traces**: PCL.030d, PCL.036b

**Verification**: `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.RemoteIngressDefaultRouteModes`.

---

## 26. Manifest Routing Edge Cases

### REQ_PCL_275 - Manifest Malformed Shapes Fail Closed With Precise Diagnostics
Every malformed manifest shape -- a transport line missing its plugin path, a transport plugin path that does not exist, a route line missing its peer list, an unknown reliability token, an oversized peer list, and an unknown directive -- shall fail closed with a diagnostic naming the specific defect.

**Traces**: PCL.070

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.FailsClosedOnEachMalformedManifestShape`.

### REQ_PCL_317 - Routing Null Handle Accessors Are Safe
`pcl_transport_routing_transport_count(NULL)` shall return 0, `pcl_transport_routing_destroy(NULL)` shall be a no-op, and `pcl_transport_routing_load()` shall reject NULL executor, manifest path, or output-handle arguments with `PCL_ERR_INVALID`.

**Traces**: PCL.069, PCL.045

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.NullHandleAccessorsAreSafe`.

### REQ_PCL_318 - Manifest Load Fails Closed When Transport Slots Exhausted
When the manifest's transports would exceed the executor's fixed transport table, the load shall fail closed with a diagnostic naming the register failure, and any transports already registered by the failed load shall be rolled back.

**Traces**: PCL.070, PCL.046

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.FailsClosedWhenExecutorTransportSlotsExhausted`.

### REQ_PCL_319 - Manifest Load Fails Closed When Route Table Exhausted
When the manifest's routes would exceed the executor's fixed endpoint-route table, the load shall fail closed with a diagnostic naming the route-set failure.

**Traces**: PCL.070, PCL.046

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.FailsClosedWhenRouteTableExhausted`.

---

## 27. Codec Registry

### REQ_PCL_337 - Register And Get Return Same Pointer
A codec vtable registered under a content_type shall be returned unmodified by a subsequent `pcl_codec_registry_get()`.

**Traces**: PCL.059

**Verification**: `test_pcl_codec_registry.cpp::PclCodecRegistry.RegisterAndGetReturnsSamePointer`.

### REQ_PCL_338 - Get Unregistered Returns Null
`pcl_codec_registry_get()` for a content_type with no registration shall return NULL.

**Traces**: PCL.059

**Verification**: `test_pcl_codec_registry.cpp::PclCodecRegistry.GetUnregisteredReturnsNull`.

### REQ_PCL_339 - Bad Abi Version Rejected With State
Registering a codec whose `abi_version` does not equal `PCL_CODEC_ABI_VERSION` shall be rejected with `PCL_ERR_STATE` and shall not be counted.

**Traces**: PCL.059

**Verification**: `test_pcl_codec_registry.cpp::PclCodecRegistry.BadAbiVersionRejectedWithState`.

### REQ_PCL_340 - Null Arguments Rejected With Invalid
`pcl_codec_registry_register()` shall reject a NULL registry, NULL codec, or a codec with a NULL `content_type` with `PCL_ERR_INVALID`; `pcl_codec_registry_get()` shall return NULL for a NULL registry or NULL content_type.

**Traces**: PCL.059, PCL.045

**Verification**: `test_pcl_codec_registry.cpp::PclCodecRegistry.NullArgsRejectedWithInvalid`.

### REQ_PCL_341 - Multiple Codecs Per Content Type Allowed, First Wins For Get
Multiple codecs may register under the same content_type (e.g. per-component plugins sharing a bridge process); `pcl_codec_registry_get()` shall return the first registered.

**Traces**: PCL.059

**Verification**: `test_pcl_codec_registry.cpp::PclCodecRegistry.MultipleCodecsPerContentTypeAllowedFirstWinsForGet`.

### REQ_PCL_342 - Same Codec Pointer Rejected With State
Re-registering the identical codec vtable pointer shall be rejected with `PCL_ERR_STATE` and shall not increase the registered count.

**Traces**: PCL.059

**Verification**: `test_pcl_codec_registry.cpp::PclCodecRegistry.SameCodecPointerRejectedWithState`.

### REQ_PCL_343 - Get At Iterates Codecs In Registration Order
`pcl_codec_registry_get_at()` shall iterate same-content_type codecs in registration order, returning NULL once the index exceeds the registered count or for a NULL registry/content_type.

**Traces**: PCL.059

**Verification**: `test_pcl_codec_registry.cpp::PclCodecRegistry.GetAtIteratesCodecsForContentTypeInRegistrationOrder`.

### REQ_PCL_344 - Count And Clear
`pcl_codec_registry_count()` shall reflect the number of registrations; `pcl_codec_registry_clear()` shall remove all registrations (without destroying the borrowed codec vtables) so subsequent lookups return NULL.

**Traces**: PCL.059

**Verification**: `test_pcl_codec_registry.cpp::PclCodecRegistry.CountAndClearWork`.

### REQ_PCL_345 - Default Registry Is Lazily Created And Stable
`pcl_codec_registry_default()` shall create the process-global registry lazily and return the same pointer on every subsequent call.

**Traces**: PCL.059

**Verification**: `test_pcl_codec_registry.cpp::PclCodecRegistry.DefaultReturnsStableNonNullPointer`.

### REQ_PCL_346 - Codec Vtable Round Trip
A codec vtable's `encode`/`decode`/`free_msg` functions shall round-trip a typed value through a `pcl_msg_t`, and `free_msg` shall release the encoded message.

**Traces**: PCL.058

**Verification**: `test_pcl_codec_registry.cpp::PclCodecRegistry.RoundTripEncodeDecodeFreeMsgThroughStubCodec`.

---

## 28. Transport Capabilities and QoS

### REQ_PCL_347 - Null Vtable Has No Capabilities
`pcl_transport_caps_from_vtable(NULL)` shall return `PCL_CAP_NONE`.

**Traces**: PCL.060, PCL.045

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.NullVtableHasNoCaps`.

### REQ_PCL_348 - Empty Vtable Has No Capabilities
A vtable with every slot NULL shall derive `PCL_CAP_NONE`.

**Traces**: PCL.060

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.EmptyVtableHasNoCaps`.

### REQ_PCL_349 - Publish Or Subscribe Implies Pubsub
A non-NULL `publish` or `subscribe` vtable slot shall derive `PCL_CAP_PUBSUB`.

**Traces**: PCL.060

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.PublishOrSubscribeImpliesPubsub`.

### REQ_PCL_350 - Serve Or Invoke Async Implies Unary
A non-NULL `serve` or `invoke_async` vtable slot shall derive `PCL_CAP_RPC_UNARY`.

**Traces**: PCL.060

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.ServeOrInvokeAsyncImpliesUnary`.

### REQ_PCL_351 - Invoke Stream Or Stream Send Implies Stream
A non-NULL `invoke_stream` or `stream_send` vtable slot shall derive `PCL_CAP_RPC_STREAM`.

**Traces**: PCL.060

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.InvokeStreamOrStreamSendImpliesStream`.

### REQ_PCL_352 - Action Is Never Derived
`PCL_CAP_RPC_ACTION` shall never be derived from a vtable, regardless of which other slots are non-NULL, since it has no corresponding vtable slot.

**Traces**: PCL.060

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.ActionIsNeverDerived`.

### REQ_PCL_353 - Loader Caps Query Null Handle Fails Closed
`pcl_plugin_transport_caps()` shall return `PCL_ERR_INVALID` for a NULL handle.

**Traces**: PCL.065, PCL.045

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.LoaderNullHandleFailsClosed`.

### REQ_PCL_354 - Loader Prefers Explicit Caps Symbol
When a loaded transport plugin exports `pcl_transport_plugin_caps`, `pcl_plugin_transport_caps()` shall return the declared mask rather than one derived from the vtable.

**Traces**: PCL.060, PCL.064

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.LoaderUsesExplicitCapsSymbol`.

### REQ_PCL_355 - Loader Derives Capabilities When Symbol Absent
When a loaded transport plugin exports no caps symbol, `pcl_plugin_transport_caps()` shall fall back to deriving the mask from the vtable.

**Traces**: PCL.060, PCL.064

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.LoaderDerivesWhenSymbolAbsent`.

### REQ_PCL_356 - Udp Plugin Declares Pubsub
The reference UDP transport plugin shall declare `PCL_CAP_PUBSUB` and no other capability.

**Traces**: PCL.068

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.UdpPluginDeclaresPubsub`.

### REQ_PCL_357 - Socket Plugin Declares Pubsub Unary
The reference socket transport plugin shall declare `PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY`.

**Traces**: PCL.068

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.SocketPluginDeclaresPubsubUnary`.

### REQ_PCL_358 - Shm Plugin Declares Pubsub Unary Stream
The reference shared-memory transport plugin shall declare `PCL_CAP_PUBSUB | PCL_CAP_RPC_UNARY | PCL_CAP_RPC_STREAM`.

**Traces**: PCL.068

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.ShmPluginDeclaresPubsubUnaryStream`.

### REQ_PCL_359 - Udp Plugin Offers Best Effort
The reference UDP transport plugin shall declare `PCL_QOS_RELIABILITY_BEST_EFFORT`.

**Traces**: PCL.068, PCL.062

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.UdpPluginOffersBestEffort`.

### REQ_PCL_360 - Socket Plugin Offers Reliable
The reference socket transport plugin shall declare `PCL_QOS_RELIABILITY_RELIABLE`.

**Traces**: PCL.068, PCL.062

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.SocketPluginOffersReliable`.

### REQ_PCL_361 - Shm Plugin Offers Reliable
The reference shared-memory transport plugin shall declare `PCL_QOS_RELIABILITY_RELIABLE`.

**Traces**: PCL.068, PCL.062

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.ShmPluginOffersReliable`.

### REQ_PCL_362 - Loader Qos Query Null Out-Param Fails Closed
`pcl_plugin_transport_qos()` shall return `PCL_ERR_INVALID` when the output QoS pointer is NULL.

**Traces**: PCL.065, PCL.045

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.LoaderQosUnspecifiedWhenSymbolAbsent`.

### REQ_PCL_363 - Endpoint Required Capabilities Mapping
`pcl_endpoint_required_caps()` shall map publisher/subscriber to `PCL_CAP_PUBSUB`, provided/consumed to `PCL_CAP_RPC_UNARY`, and stream-provided to `PCL_CAP_RPC_STREAM`.

**Traces**: PCL.061

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.EndpointRequiredCaps`.

### REQ_PCL_364 - Caps Supports Check
`pcl_transport_caps_supports()` shall return true only when a capability mask includes every bit of the required mask, vacuously true for `PCL_CAP_NONE` required.

**Traces**: PCL.060

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.CapsSupports`.

### REQ_PCL_365 - Validate Route Passes When Transport Has Capability
`pcl_executor_validate_endpoint_route()` shall succeed when the named peer transport's declared capabilities satisfy the endpoint's requirement.

**Traces**: PCL.063

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.ValidateRoutePassesWhenTransportHasCap`.

### REQ_PCL_366 - Validate Route Fails Closed On Missing Capability
`pcl_executor_validate_endpoint_route()` shall return `PCL_ERR_STATE` with a diagnostic naming the missing capability when the named transport lacks it.

**Traces**: PCL.063

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.ValidateRouteFailsClosedOnMissingCap`.

### REQ_PCL_367 - Validate Route Fails Closed On Missing Peer
`pcl_executor_validate_endpoint_route()` shall return `PCL_ERR_NOT_FOUND` with a diagnostic naming the peer when a route names an unregistered transport peer.

**Traces**: PCL.063

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.ValidateRouteFailsClosedOnMissingPeer`.

### REQ_PCL_368 - Validate Local-Only Route Needs No Transport
A route configured local-only shall validate successfully without any transport registered.

**Traces**: PCL.063

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.ValidateLocalOnlyRouteNeedsNoTransport`.

### REQ_PCL_369 - Validate Pubsub Endpoint Over Pubsub Transport
A subscriber endpoint routed to a pub/sub-capable named transport shall validate successfully.

**Traces**: PCL.063

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.ValidatePubsubEndpointOverPubsubTransport`.

### REQ_PCL_370 - Qos Satisfies Is Ordered Reliability
`pcl_qos_satisfies()` shall implement the ordered reliability check (unspecified < best_effort < reliable): an offer satisfies a floor only when its reliability is greater than or equal to the floor's.

**Traces**: PCL.062

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.QosSatisfiesIsOrderedReliability`.

### REQ_PCL_371 - Validate Route Passes When Transport Meets Qos Floor
A route shall validate successfully when the named transport's declared QoS meets the endpoint's QoS floor.

**Traces**: PCL.063, PCL.062

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.ValidateRoutePassesWhenTransportMeetsQosFloor`.

### REQ_PCL_372 - Validate Route Fails Closed When Qos Floor Unmet
A route shall fail closed with a diagnostic naming both the required and offered reliability when the named transport's declared QoS does not meet the endpoint's floor.

**Traces**: PCL.063, PCL.062

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.ValidateRouteFailsClosedWhenQosFloorUnmet`.

### REQ_PCL_373 - Validate Route Fails Closed When Qos Undeclared
A route with a declared QoS floor shall fail closed when the named transport's reliability is undeclared (unspecified).

**Traces**: PCL.063, PCL.062

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.ValidateRouteFailsClosedWhenQosUndeclared`.

### REQ_PCL_374 - Validate Route Ignores Qos When No Floor
A route with no declared QoS floor shall validate on capabilities alone, ignoring the transport's offered QoS.

**Traces**: PCL.063, PCL.062

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.ValidateRouteIgnoresQosWhenNoFloor`.

### REQ_PCL_375 - Endpoint Required Capabilities Unknown Kind Is None
`pcl_endpoint_required_caps()` shall return `PCL_CAP_NONE` for an unrecognised endpoint kind.

**Traces**: PCL.061

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.EndpointRequiredCapsUnknownKindIsNone`.

### REQ_PCL_376 - Qos Reliability Names
`pcl_qos_reliability_name()` shall return the correct human-readable name for each declared reliability level and default to `"unspecified"` for an unrecognised value.

**Traces**: PCL.062

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.QosReliabilityNames`.

### REQ_PCL_377 - Set Transport Qos On Default Transport
`pcl_executor_set_transport_qos()` shall set the QoS of the executor's default transport slot and reject a NULL executor with `PCL_ERR_INVALID`.

**Traces**: PCL.062, PCL.045

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.SetTransportQosOnDefaultTransport`.

### REQ_PCL_378 - Register Transport Qos Unknown Peer Not Found
`pcl_executor_register_transport_qos()` shall reject NULL executor/peer_id arguments with `PCL_ERR_INVALID` and return `PCL_ERR_NOT_FOUND` for a peer id that was never registered.

**Traces**: PCL.062, PCL.045

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.RegisterTransportQosUnknownPeerNotFound`.

### REQ_PCL_379 - Validate Route Against Default Transport
A remote route with no named peers shall validate against the executor's default transport slot: failing closed with `PCL_ERR_NOT_FOUND` when absent, and against its capabilities when present.

**Traces**: PCL.063

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.ValidateRouteAgainstDefaultTransport`.

### REQ_PCL_380 - Validate Publisher Route Fails Closed Without Pubsub Capability
A publisher endpoint routed to a transport lacking `PCL_CAP_PUBSUB` shall fail closed with a diagnostic naming the missing capability.

**Traces**: PCL.063

**Verification**: `test_pcl_capabilities.cpp::PclCapabilities.ValidatePublisherRouteFailsClosedWithoutPubsubCap`.

---

## 29. Plugin Loader

### REQ_PCL_381 - Unload Transport Null Handle Fails Closed
`pcl_plugin_unload_transport(NULL, ...)` shall return `PCL_ERR_INVALID`.

**Traces**: PCL.067, PCL.045

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.UnloadTransportNullHandleFailsClosed`.

### REQ_PCL_382 - Unload Transport Degrades Without Teardown Symbol
`pcl_plugin_unload_transport()` shall degrade to a plain unload for a plugin exporting no teardown symbol.

**Traces**: PCL.067

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.UnloadTransportDegradesToUnloadWithoutTeardownSymbol`.

### REQ_PCL_383 - Loaded Transport Plugin Vtable Is Callable
A transport plugin's vtable returned by `pcl_plugin_load_transport()` shall be directly callable, and its additional exported symbols resolvable via `pcl_plugin_symbol()`.

**Traces**: PCL.064

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.LoadTransportPluginAndCallVtable`.

### REQ_PCL_384 - Loaded Transport Installable On Executor
A transport plugin's vtable shall be installable as an executor's default transport via `pcl_executor_set_transport()`.

**Traces**: PCL.064, PCL.029

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.InstallLoadedTransportOnExecutor`.

### REQ_PCL_385 - Load Codec Plugin Registers By Content Type
`pcl_plugin_load_codec()` shall register the plugin's codec vtable under its declared content_type and thread `config_json` into the plugin's entry point.

**Traces**: PCL.066, PCL.059

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.LoadCodecPluginRegistersByContentType`.

### REQ_PCL_386 - Load Codec Missing File Fails Closed
`pcl_plugin_load_codec()` with a nonexistent path shall return `PCL_ERR_NOT_FOUND` and register nothing.

**Traces**: PCL.065

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.MissingFileFailsClosed`.

### REQ_PCL_387 - Shared Memory Transport Plugin Constructs Working Transport
The shared-memory transport plugin shall construct a fully functional transport (with gateway and destroy hooks) from its JSON configuration.

**Traces**: PCL.068

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.LoadSharedMemoryTransportPluginViaConfig`.

### REQ_PCL_388 - Shared Memory Transport Plugin Null Config Fails Closed
The shared-memory transport plugin's entry point shall return NULL (mapped to `PCL_ERR_STATE`) for a NULL configuration string.

**Traces**: PCL.068, PCL.045

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.SharedMemoryTransportPluginNullConfigFailsClosed`.

### REQ_PCL_389 - Udp Transport Plugin Constructs Working Transport
The UDP transport plugin shall construct a fully functional transport (with a destroy hook) from its JSON configuration.

**Traces**: PCL.068

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.LoadUdpTransportPluginViaConfig`.

### REQ_PCL_390 - Udp Transport Plugin Null Config Fails Closed
The UDP transport plugin's entry point shall return NULL (mapped to `PCL_ERR_STATE`) for a NULL configuration string.

**Traces**: PCL.068, PCL.045

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.UdpTransportPluginNullConfigFailsClosed`.

### REQ_PCL_391 - Load Codec Plugins From Manifest
`pcl_codec_registry_load_plugins_from_manifest()` shall load every codec plugin listed in a manifest file, ignoring blank lines and `#` comments and skipping a listed path that is not a codec plugin.

**Traces**: PCL.066

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.LoadCodecPluginsFromManifest`.

### REQ_PCL_392 - Missing Manifest Fails Closed
`pcl_codec_registry_load_plugins_from_manifest()` with a nonexistent manifest path shall return `PCL_ERR_NOT_FOUND`.

**Traces**: PCL.066

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.MissingManifestFailsClosed`.

### REQ_PCL_393 - Codec Plugin Bad Abi Fails Closed
`pcl_plugin_load_codec()` shall return `PCL_ERR_STATE` for a codec plugin whose declared ABI version does not match `PCL_CODEC_ABI_VERSION`, registering nothing.

**Traces**: PCL.065

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.BadAbiFailsClosed`.

### REQ_PCL_394 - Codec Entry Returning Null Fails Closed
`pcl_plugin_load_codec()` shall return `PCL_ERR_STATE` when the plugin's entry point returns NULL.

**Traces**: PCL.065

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.CodecEntryReturningNullFailsClosed`.

### REQ_PCL_395 - Reloading Same Codec Vtable Fails Closed
Loading the same codec plugin twice in one process yields the identical static vtable pointer; the registry's duplicate-pointer rejection (`PCL_ERR_STATE`) shall propagate through `pcl_plugin_load_codec()`, and the loader shall unload the redundant library handle.

**Traces**: PCL.065, PCL.059

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.ReloadingSameCodecVtableFailsClosed`.

### REQ_PCL_396 - Transport Plugin Without Abi Symbol Fails Closed
`pcl_plugin_load_transport()` shall return `PCL_ERR_NOT_FOUND` when the library exports no ABI-version symbol.

**Traces**: PCL.065

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.TransportPluginWithoutAbiSymbolFailsClosed`.

### REQ_PCL_397 - Transport Plugin With Wrong Abi Fails Closed
`pcl_plugin_load_transport()` shall return `PCL_ERR_STATE` when the plugin's declared ABI version does not equal `PCL_TRANSPORT_ABI_VERSION`.

**Traces**: PCL.065, PCL.064

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.TransportPluginWithWrongAbiFailsClosed`.

### REQ_PCL_398 - Transport Plugin Without Entry Symbol Fails Closed
`pcl_plugin_load_transport()` shall return `PCL_ERR_NOT_FOUND` when the library exports no entry-point symbol.

**Traces**: PCL.065

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.TransportPluginWithoutEntrySymbolFailsClosed`.

### REQ_PCL_399 - Load Codec Plugins From Paths Skips Bad Entries
`pcl_codec_registry_load_plugins_from_paths()` shall skip NULL, empty, missing, and non-codec path entries while still loading valid ones, and shall reject a NULL registry or a NULL path array with a nonzero count with `PCL_ERR_INVALID`.

**Traces**: PCL.066, PCL.045

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.LoadCodecPluginsFromPathsSkipsBadEntries`.

### REQ_PCL_400 - Load Codec Plugins From Environment Variable
`pcl_codec_registry_load_plugins_from_env()` shall split the named environment variable's value on the platform path-list separator, skip a missing plugin path, and load the rest; an unset or empty variable shall be `PCL_OK` with nothing loaded.

**Traces**: PCL.066

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.LoadCodecPluginsFromEnv`.

### REQ_PCL_401 - Manifest Load Null Arguments Fail Closed
`pcl_codec_registry_load_plugins_from_manifest()` shall reject a NULL registry, a NULL manifest path, or an empty manifest path with `PCL_ERR_INVALID`.

**Traces**: PCL.066, PCL.045

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.ManifestNullArgsFailClosed`.

### REQ_PCL_402 - Raw Plugin Open Symbol Unload
`pcl_plugin_open()`/`pcl_plugin_symbol()`/`pcl_plugin_unload()` shall provide a portable wrapper over the platform dynamic loader for libraries with no PCL entry-point contract, safely handling NULL paths, unresolvable libraries, unresolvable symbols, and NULL handles.

**Traces**: PCL.065

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.OpenSymbolUnloadRawLibrary`.

### REQ_PCL_403 - Socket Plugin Server Client Round Trip Via Entry
The socket transport plugin's server and client roles, constructed purely through the plugin ABI, shall interoperate end-to-end -- including gateway container access on the server and safe teardown-then-unload for both.

**Traces**: PCL.068, PCL.067

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.SocketPluginServerClientRoundTripViaEntry`.

### REQ_PCL_404 - Socket Plugin Rejects Bad Configs
The socket transport plugin's entry point shall return NULL (mapped to `PCL_ERR_STATE`) for a NULL configuration string, a missing or unknown `role`, an out-of-range or missing `port`, or a missing `executor`.

**Traces**: PCL.068, PCL.045

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.SocketPluginRejectsBadConfigs`.

### REQ_PCL_405 - Independent Codec Batch Loads Each Retain Their Handle
Two independent calls to `pcl_codec_registry_load_plugins_from_paths()` for the same codec plugin, into two different registries in one process, shall each retain a resident handle and register the codec in its own registry.

**Traces**: PCL.066

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.LoadCodecPluginsFromPathsTwiceRetainsBothHandles`.

### REQ_PCL_406 - Shm Plugin Rejects Incomplete Configs
The shared-memory transport plugin's entry point shall return NULL (mapped to `PCL_ERR_STATE`) when `bus_name`, `participant_id`, or `executor` is missing from its configuration.

**Traces**: PCL.068, PCL.045

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.ShmPluginRejectsIncompleteConfigs`.

### REQ_PCL_407 - Shm Plugin Teardown Then Unload
The shared-memory transport plugin's teardown symbol, invoked via `pcl_plugin_unload_transport()`, shall release its live transport safely before the library is unloaded.

**Traces**: PCL.067, PCL.068

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.ShmPluginTeardownThenUnload`.

### REQ_PCL_408 - Udp Plugin Rejects Incomplete Configs
The UDP transport plugin's entry point shall return NULL (mapped to `PCL_ERR_STATE`) when `remote_host` is missing, `remote_port` is missing, `local_port` is out of range, or `executor` is missing from its configuration.

**Traces**: PCL.068, PCL.045

**Verification**: `test_pcl_plugin_loader.cpp::PclPluginLoader.UdpPluginRejectsIncompleteConfigs`.

---

## 30. Socket Transport Fault Injection

### REQ_PCL_409 - Gateway Configure Fails When Ports Already Full
The socket transport's gateway container's `on_configure` shall fail with `PCL_ERR_NOMEM` when its port table is already at capacity.

**Traces**: PCL.034, PCL.046

**Verification**: `test_pcl_socket_faults.cpp::PclSocketFaults.GatewayConfigureFailsWhenPortsAlreadyFull`.

### REQ_PCL_410 - Invoke Remote Async Frame Allocation Failure
`pcl_socket_transport_invoke_remote_async()` shall return `PCL_ERR_NOMEM` and release any partial allocations when the pending-record or wire-frame allocation fails.

**Traces**: PCL.036, PCL.046

**Verification**: `test_pcl_socket_faults.cpp::PclSocketFaults.InvokeRemoteAsyncFrameMallocFails`.

### REQ_PCL_411 - Invoke Remote Async Enqueue Failure Rolls Back Pending Record
A mid-enqueue allocation failure in `pcl_socket_transport_invoke_remote_async()` shall roll back the pending record registered earlier in the same call, returning `PCL_ERR_NOMEM`.

**Traces**: PCL.036, PCL.046

**Verification**: `test_pcl_socket_faults.cpp::PclSocketFaults.InvokeRemoteAsyncEnqueueDataMallocFailsRollsBackPending`.

### REQ_PCL_412 - Recv Thread Drops Malformed Frames
The client receive thread shall drop a malformed PUBLISH frame, a malformed SVC_RESP frame, and a frame with an unrecognised type byte without crashing or wedging.

**Traces**: PCL.033

**Verification**: `test_pcl_socket_faults.cpp::PclSocketFaults.RecvThreadDropsMalformedPublishResponseAndUnknownFrames`.

### REQ_PCL_413 - Recv Thread Handles Response Type Allocation Failure
The client receive thread shall handle an allocation failure while decoding a service response's type-name string without crashing or leaking partial state.

**Traces**: PCL.033, PCL.046

**Verification**: `test_pcl_socket_faults.cpp::PclSocketFaults.RecvThreadHandlesResponseTypeAllocationFailure`.

### REQ_PCL_414 - Client Retry Covers Bad Host And Backoff Sleep
`pcl_socket_transport_create_client_ex()` with bounded retries shall exercise the exponential backoff sleep path and fail closed (return NULL) against an unresolvable host and against a refused connection on a live loopback port.

**Traces**: PCL.036e

**Verification**: `test_pcl_socket_faults.cpp::PclSocketFaults.ClientRetryCoversBadHostAndBackoffSleep`.

### REQ_PCL_415 - Auto-Reconnect Backoff Runs While Server Stays Down
With `auto_reconnect` enabled, the client's reconnect backoff loop shall continue running while the server remains down, and `pcl_socket_transport_get_state()` shall report a non-CONNECTED state throughout.

**Traces**: PCL.036e

**Verification**: `test_pcl_socket_faults.cpp::PclSocketFaults.AutoReconnectBackoffRunsWhileServerStaysDown`.

---

## 31. Manifest Routing Core Behaviour

### REQ_PCL_416 - Missing Manifest Fails Closed With Diagnostic
`pcl_transport_routing_load()` with a nonexistent manifest path shall return `PCL_ERR_NOT_FOUND` with a diagnostic naming the missing file.

**Traces**: PCL.069, PCL.070

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.MissingManifestFailsClosed`.

### REQ_PCL_417 - Empty Manifest Is Ok With No Transports
An empty or comment-only manifest shall load successfully with an empty routing handle reporting zero transports.

**Traces**: PCL.069

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.EmptyManifestIsOkWithNoTransports`.

### REQ_PCL_418 - Mixed Middleware Loads And Validates
A single manifest shall compose heterogeneous transports -- a unary-capable recorder for a service and UDP for a topic -- registering each as a distinct named peer and validating both routes.

**Traces**: PCL.069

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.MixedMiddlewareLoadsAndValidates`.

### REQ_PCL_419 - Real Traffic Flows Through Routed Transport
A publish issued through a manifest-registered named transport shall be carried by that transport, verified via the transport's own recorded state.

**Traces**: PCL.069

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.RealTrafficFlowsThroughRoutedTransport`.

### REQ_PCL_420 - Fails Closed When Transport Lacks Required Capability
A manifest route to a transport lacking the endpoint's required capability shall fail closed with a diagnostic naming the missing capability, leaving nothing installed.

**Traces**: PCL.070, PCL.063

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.FailsClosedWhenTransportLacksRequiredCap`.

### REQ_PCL_421 - Rolls Back Routes When Later Line Fails
A route installed by an earlier manifest line shall be rolled back when a later line in the same manifest fails, leaving the executor's endpoint-route table empty.

**Traces**: PCL.070

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.RollsBackRoutesWhenLaterLineFails`.

### REQ_PCL_422 - Preserves Pre-Existing Route On Duplicate
A manifest route line duplicating a pre-existing endpoint route (e.g. a programmatic default) shall fail closed without overwriting the original route.

**Traces**: PCL.070

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.PreservesPreExistingRouteOnDuplicate`.

### REQ_PCL_423 - Rejects Duplicate Transport Peer
A manifest transport line reusing a peer id already registered on the executor shall fail closed, leaving the original transport untouched.

**Traces**: PCL.070

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.RejectsDuplicateTransportPeer`.

### REQ_PCL_424 - Preserves Default Transport On Named Peer Teardown
Destroying a manifest-owned named transport shall not clear an unrelated default transport installed by another owner.

**Traces**: PCL.070

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.PreservesDefaultTransportOnNamedPeerTeardown`.

### REQ_PCL_425 - Reuses Transport Slots Across Load Destroy Cycles
Repeated load/destroy cycles of the same manifest on one executor shall free the named-transport slot each time, never leaking it toward `PCL_MAX_TRANSPORTS`.

**Traces**: PCL.069, PCL.070

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.ReusesTransportSlotsAcrossLoadDestroyCycles`.

### REQ_PCL_426 - Fails Closed When Qos Floor Unmet
A manifest route demanding a QoS floor the named transport does not meet shall fail closed with a diagnostic naming the reliability mismatch.

**Traces**: PCL.070, PCL.062

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.FailsClosedWhenQosFloorUnmet`.

### REQ_PCL_427 - Fails Closed On Unknown Peer
A manifest route naming a peer with no matching transport line shall fail closed with `PCL_ERR_NOT_FOUND` and a diagnostic naming the peer.

**Traces**: PCL.070

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.FailsClosedOnUnknownPeer`.

### REQ_PCL_428 - Fails Closed On Malformed Line
A manifest route line with an unrecognised endpoint kind shall fail closed with `PCL_ERR_INVALID` and a diagnostic naming the bad kind.

**Traces**: PCL.070

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.FailsClosedOnMalformedLine`.

---

## 32. Transport Template Core Behaviour

### REQ_PCL_429 - Create Rejects Missing Hooks
`pcl_transport_template_create()` shall return NULL for a NULL hooks pointer, a NULL executor, or hooks missing `send_blocking` or `recv_blocking`.

**Traces**: PCL.071, PCL.045

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.RejectsMissingHooks`.

### REQ_PCL_430 - Open Failure Aborts Create
A failing `hooks.open` shall abort `pcl_transport_template_create()`, returning NULL without invoking `hooks.close` (which only runs after a successful open).

**Traces**: PCL.071

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.OpenFailureAbortsCreate`.

### REQ_PCL_431 - Destroy Null Is No-Op
`pcl_transport_template_destroy(NULL)` shall be a safe no-op.

**Traces**: PCL.071, PCL.045

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.DestroyNullIsNoOp`.

### REQ_PCL_432 - Set Peer Id Validation
`pcl_transport_template_set_peer_id()` shall reject a NULL context, a NULL peer id, or an empty peer id with `PCL_ERR_INVALID`.

**Traces**: PCL.071, PCL.045

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.SetPeerIdValidation`.

### REQ_PCL_433 - Vtable Shape
The template transport's vtable shall expose the full pub/sub plus unary-RPC shape expected of a conforming transport.

**Traces**: PCL.072

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.VtableShape`.

### REQ_PCL_434 - Close Runs Exactly Once After Destroy
`hooks.close` shall fire exactly once when the transport is destroyed.

**Traces**: PCL.071

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.CloseRunsAfterDestroy`.

### REQ_PCL_435 - Destroy Clears Every Alias, Including Renamed Ones
Destroying a transport shall unregister every peer id it was ever known by from the executor, not just its current peer id at the time of destruction.

**Traces**: PCL.071

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.DestroyClearsRenamedPeerAlias`.

### REQ_PCL_436 - Recv Publish Normalises Null Type Name
An inbound PUBLISH frame whose `type_name` is NULL shall be normalised to an empty string before being posted as remote ingress, so the executor does not silently reject it.

**Traces**: PCL.071

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.RecvPublishNormalisesNullTypeName`.

### REQ_PCL_437 - Invoke Async Fails When Send Stopped
`invoke_async()` shall return `PCL_ERR_STATE` without firing the caller's callback once the send thread has been signalled to stop.

**Traces**: PCL.071

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.InvokeAsyncFailsWhenSendStopped`.

### REQ_PCL_438 - Remote Svc Req Honours Local-Only Exposure
An inbound SVC_REQ frame for a service configured local-only shall be dispatched through the remote-aware executor ingress path: the handler shall never be invoked, and the remote caller shall still receive an empty response rather than being left waiting.

**Traces**: PCL.071, PCL.030d

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.RemoteSvcReqHonoursLocalOnlyExposure`.

### REQ_PCL_439 - Destroy Clears Long Peer Id Alias
A peer id exceeding the executor's fixed peer-id buffer shall be tracked by the transport in its executor-truncated form, so `destroy()` still locates and clears the correct executor slot.

**Traces**: PCL.071

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.DestroyClearsLongPeerIdAlias`.

### REQ_PCL_440 - Destroy Preserves Unrelated Default Transport
Destroying a template transport instance that is registered only as a named peer (not the executor's default) shall not clear the executor's unrelated default transport slot.

**Traces**: PCL.071

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate.DestroyPreservesUnrelatedDefaultTransport`.

### REQ_PCL_441 - Conformance Publish Round Trip
The template transport shall pass the shared publish-round-trip conformance case.

**Traces**: PCL.072

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate_Conformance.PublishRoundTrip`.

### REQ_PCL_442 - Conformance Service Round Trip
The template transport shall pass the shared service-round-trip conformance case.

**Traces**: PCL.072

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate_Conformance.ServiceRoundTrip`.

### REQ_PCL_443 - Conformance Multiple Publishes Preserve Order
The template transport shall preserve publish ordering across multiple messages on the same topic.

**Traces**: PCL.072

**Verification**: `test_pcl_template_transport.cpp::PclTransportTemplate_Conformance.MultiplePublishesPreserveOrder`.

---

## 33. APOS Transport Core Behaviour

### REQ_PCL_444 - Apos Stub Fifo And Wait On Multi Channel
The APOS stub shall deliver messages FIFO per channel, and `waitOnMultiChannel()` shall report which watched channel has data ready.

**Traces**: PCL.073

**Verification**: `test_pcl_apos_transport.cpp::AposStub.FifoAndWaitOnMultiChannel`.

### REQ_PCL_445 - Apos Transport Vtable Shape
The APOS transport's vtable shall expose the full pub/sub plus unary-RPC shape expected of a conforming transport.

**Traces**: PCL.073, PCL.072

**Verification**: `test_pcl_apos_transport.cpp::PclTransportApos.VtableShape`.

### REQ_PCL_446 - Apos Conformance Publish Round Trip
The APOS transport shall pass the shared publish-round-trip conformance case.

**Traces**: PCL.073, PCL.072

**Verification**: `test_pcl_apos_transport.cpp::PclTransportApos_Conformance.PublishRoundTrip`.

### REQ_PCL_447 - Apos Conformance Service Round Trip
The APOS transport shall pass the shared service-round-trip conformance case.

**Traces**: PCL.073, PCL.072

**Verification**: `test_pcl_apos_transport.cpp::PclTransportApos_Conformance.ServiceRoundTrip`.

---

## 34. Transport Threading-Model Conformance

These requirements verify the transport threading-model contract (PCL.075) via the
reusable threading conformance suite (PCL.076). Two reusable helpers in
`pcl_transport_conformance.hpp` back most cases:
`expectIngressRunsOnExecutorThread` (records the executor/spin thread id and
asserts the subscriber callback ran on it) and
`expectResponseCallbackOnExecutorNotInline` (asserts the async response callback
did not fire before `invoke_async` returned and then ran on the executor thread).
The template transport additionally uses a gated send hook to prove egress
promptness and destroy-wakes-worker behaviour deterministically.

### REQ_PCL_448 - Template Ingress Runs On Executor Thread
The template transport shall deep-copy inbound wire traffic into an executor queue so the subscriber callback runs on the executor thread, never inline on the transport receive thread.

**Traces**: PCL.075, PCL.076

**Verification**: `test_pcl_transport_threading.cpp::PclTransportThreading.TemplateIngressRunsOnExecutor`.

### REQ_PCL_449 - Shared-Memory Ingress Runs On Executor Thread
The shared-memory transport shall dispatch subscriber ingress on the executor thread, never inline on the bus receive thread.

**Traces**: PCL.075, PCL.076

**Verification**: `test_pcl_transport_threading.cpp::PclTransportThreading.SharedMemoryIngressRunsOnExecutor`.

### REQ_PCL_450 - UDP Ingress Runs On Executor Thread
The UDP transport shall dispatch subscriber ingress on the executor thread, never inline on the datagram receive thread.

**Traces**: PCL.075, PCL.076

**Verification**: `test_pcl_transport_threading.cpp::PclTransportThreading.UdpIngressRunsOnExecutor`.

### REQ_PCL_451 - Socket Ingress Runs On Executor Thread
The socket transport shall dispatch subscriber ingress on the executor thread, never inline on the socket receive thread.

**Traces**: PCL.075, PCL.076

**Verification**: `test_pcl_transport_threading.cpp::PclTransportThreading.SocketIngressRunsOnExecutor`.

### REQ_PCL_452 - Template Async Response Callback Not Inline
The template transport's `invoke_async` shall not fire the response callback before it returns; the callback shall be delivered on a later executor spin, on the executor thread.

**Traces**: PCL.075, PCL.076

**Verification**: `test_pcl_transport_threading.cpp::PclTransportThreading.TemplateResponseCallbackNotInline`.

### REQ_PCL_453 - Shared-Memory Async Response Callback Not Inline
The shared-memory transport's `invoke_async` shall not fire the response callback inline; the callback shall be delivered on the executor thread.

**Traces**: PCL.075, PCL.076

**Verification**: `test_pcl_transport_threading.cpp::PclTransportThreading.SharedMemoryResponseCallbackNotInline`.

### REQ_PCL_454 - Socket Async Response Callback Not Inline
The socket transport's `invoke_async` shall not fire the response callback inline; the callback shall be delivered on the executor thread.

**Traces**: PCL.075, PCL.076

**Verification**: `test_pcl_transport_threading.cpp::PclTransportThreading.SocketResponseCallbackNotInline`.

### REQ_PCL_455 - Template Publish Returns Promptly While Send Blocked
With the template send hook wedged behind a latch, `publish` shall copy and enqueue the frame and return promptly (well under the blocked-send budget), not block on the send worker.

**Traces**: PCL.075, PCL.076

**Verification**: `test_pcl_transport_threading.cpp::PclTransportThreading.TemplatePublishQueuesWhenSendBlocks`.

### REQ_PCL_456 - Template Invoke-Async Returns Promptly While Send Blocked
With the template send hook wedged, `invoke_async` shall register correlation state, enqueue the request, and return promptly without firing the callback inline.

**Traces**: PCL.075, PCL.076

**Verification**: `test_pcl_transport_threading.cpp::PclTransportThreading.TemplateInvokeQueuesWhenSendBlocks`.

### REQ_PCL_457 - Backpressured Shared-Memory Publish Does Not Block Executor
When a shared-memory topic has a non-zero backpressure budget, the executor-facing `publish` shall copy and enqueue the frame to the egress worker and return promptly; the mailbox-capacity wait shall be owned by the worker, not the executor thread.

**Traces**: PCL.075, PCL.076, PCL.036g

**Verification**: `test_pcl_transport_threading.cpp::PclTransportThreading.SharedMemoryPublishDoesNotSleepForBackpressure`.

### REQ_PCL_458 - Template Destroy Wakes And Joins Blocked Send Worker
With a frame queued behind a wedged send hook, `pcl_transport_template_destroy()` shall wake the blocked send worker (via the `wake` hook) and join it within a bounded deadline rather than hanging.

**Traces**: PCL.075, PCL.076

**Verification**: `test_pcl_transport_threading.cpp::PclTransportThreading.TemplateDestroyWakesBlockedSendWorker`.

## 35. Deferred Service Responses

### REQ_PCL_459 - Deferred Service Response Round Trip
A service handler may save its `pcl_svc_context_t` and return `PCL_PENDING`; a later call to `pcl_service_respond()` with that context shall deliver the response to the original caller's callback.

**Traces**: PCL.011b

**Verification**: `test_pcl_robustness.cpp::DeferredServiceResponse`, `test_pcl_executor.cpp::InvokeAsyncIntraProcessPendingResponse`.

### REQ_PCL_460 - Deferred Response Null Safety And Context Release
`pcl_service_respond()` shall return `PCL_ERR_INVALID` for a NULL context or NULL response, and the saved service context shall be releasable without leaking when the handler never responds.

**Traces**: PCL.011b, PCL.045

**Verification**: `test_pcl_robustness.cpp::ServiceRespondNullArgs`, `test_pcl_robustness.cpp::ServiceContextFree`.

### REQ_PCL_461 - Deferred Response Through Transport
When the request arrived through a transport with a `respond` vtable slot, `pcl_service_respond()` shall route the deferred response back through that transport.

**Traces**: PCL.011b

**Verification**: `test_pcl_robustness.cpp::ServiceRespondWithTransport`.

## 36. Status Code Discrimination

### REQ_PCL_462 - Status Codes Discriminate Failure Modes
Each documented status code shall be returned by at least one exercised public API path: `PCL_OK` (any successful call), `PCL_ERR_INVALID` (NULL/invalid arguments), `PCL_ERR_STATE` (invalid lifecycle transition), `PCL_ERR_TIMEOUT` (graceful-shutdown timeout), `PCL_ERR_CALLBACK` (lifecycle callback failure), `PCL_ERR_NOMEM` (allocation failure or capacity overflow), `PCL_ERR_NOT_FOUND` (unknown topic/service/peer), and `PCL_ERR_PORT_CLOSED` (publish while inactive).

**Traces**: PCL.047

**Verification**: `test_pcl_lifecycle.cpp::InvalidTransitionsRejected` (ERR_STATE), `test_pcl_robustness.cpp::PublishOnInactiveContainerReturnsClosed` (ERR_PORT_CLOSED), `test_pcl_robustness.cpp::ParamOverflowReturnsNomem` (ERR_NOMEM), `test_pcl_robustness.cpp::DispatchToUnknownTopicReturnsNotFound` (ERR_NOT_FOUND), `test_pcl_robustness.cpp::GracefulShutdownTimeout` (ERR_TIMEOUT), `test_pcl_lifecycle.cpp::CallbackFailureAbortsTransition` (ERR_CALLBACK), `test_pcl_lifecycle.cpp::NullHandlesReturnError` (ERR_INVALID).

## 37. Manifest Routing Exclusivity

Two-sided `exclusive <group_name> <side_a_endpoints> <side_b_endpoints>` groups
(PCL.077): mutual exclusivity is between the two named *sides* of one logical
leg, not between individual members -- any number of same-side endpoints
route together freely; the violation is routing anything from *both* sides.
See design decision D5 ("Compose-time exclusivity") in the retired
`rpc_pubsub_interchangeability_plan.md` (design intent summarised in
`doc/plans/PYRAMID/README.md`; full text in git history).

### REQ_PCL_463 - Exclusive Group Two-Sided Conflict Fails Closed
A `route` line that completes both sides of a declared `exclusive` group (one side-A endpoint and one side-B endpoint both routed) shall fail closed with `PCL_ERR_STATE` and a diagnostic naming the group and both conflicting endpoints.

**Traces**: PCL.077

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.ExclusiveGroupTwoSidedConflictFailsClosed`.

### REQ_PCL_464 - Exclusive Group Multiple Same-Side Endpoints Route Together
Any number of same-side endpoints of one declared `exclusive` group (e.g. all three rpc command endpoints of a request leg) shall route together successfully with no side-B member routed.

**Traces**: PCL.077

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.ExclusiveGroupMultipleSameSideEndpointsRouteTogether`.

### REQ_PCL_465 - Exclusive Group Leaves Ungrouped Endpoint Unaffected
An endpoint named in no `exclusive` group shall be unaffected by exclusivity checking, even when routed alongside a group whose declared members are also routed.

**Traces**: PCL.077

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.ExclusiveGroupLeavesUngroupedEndpointUnaffected`.

### REQ_PCL_466 - Exclusive Group Fails Closed On Malformed Line
An `exclusive` line missing a clause (group name, side A, or side B) shall fail closed with `PCL_ERR_INVALID` and a diagnostic naming the missing clause.

**Traces**: PCL.077, PCL.070

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.ExclusiveGroupFailsClosedOnMalformedLine`.

### REQ_PCL_467 - Exclusive Group Fails Closed On Empty List Member
An `exclusive` line with an empty endpoint name inside a side's comma-separated list (a doubled comma) shall fail closed with `PCL_ERR_INVALID` and a diagnostic naming the group and the affected side.

**Traces**: PCL.077, PCL.070

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.ExclusiveGroupFailsClosedOnEmptyListMember`.

### REQ_PCL_468 - Exclusive Group Routes Rolled Back When Later Line Fails
Routes installed for a declared `exclusive` group's side shall be rolled back, along with everything else this manifest installed, when a later manifest line fails for an unrelated reason.

**Traces**: PCL.077, PCL.070

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.ExclusiveGroupRoutesRolledBackWhenLaterLineFails`.

### REQ_PCL_469 - Exclusive Group Conflict Detected Regardless Of Declaration Order
A two-sided `exclusive` group conflict shall be detected and fail closed with `PCL_ERR_STATE` regardless of whether the conflicting `route` lines appear before or after the group's `exclusive` declaration in the manifest file.

**Traces**: PCL.077

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.ExclusiveGroupConflictDetectedRegardlessOfDeclarationOrder`.

### REQ_PCL_474 - Exclusive Group Conflict Detected Against Pre-Existing Route
A two-sided `exclusive` group conflict shall be detected and fail closed with `PCL_ERR_STATE` even when one side's routed member was installed before the current `pcl_transport_routing_load()` call began -- by an earlier manifest load into the same executor, or by a direct programmatic `pcl_executor_set_endpoint_route()` call -- and the current manifest only routes the opposite side, never mentioning the pre-existing side's endpoint at all.

**Traces**: PCL.077

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.ExclusiveGroupConflictDetectedAgainstPreExistingRoute`. Implementation note: `validate_exclusivity()` queries the executor's live route table via `pcl_executor_endpoint_route_exists_any_kind()` (kind-agnostic, since an exclusive group's side members are plain endpoint names with no fixed kind), not merely the routes the current manifest load installed.

### REQ_PCL_475 - Exclusive Group Conflict Detected Against Concrete Port Route
A two-sided `exclusive` group conflict shall be detected and fail closed with `PCL_ERR_STATE` even when one side's routed member is a concrete port routed directly via `pcl_port_set_route()` (the mechanism `pcl::Port::routeRemote()` et al. use, typically applied to a publisher/provided port at bind() time) rather than through `pcl_executor_set_endpoint_route()`'s endpoint route table -- the two are distinct, independently-queryable live route stores, and a name routed through either one counts as "routed" for exclusivity purposes.

**Traces**: PCL.077

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.ExclusiveGroupConflictDetectedAgainstConcretePortRoute`. Implementation note: `pcl_executor_endpoint_route_exists_any_kind()` covers both live route stores -- the executor endpoint route table and every container's `route_configured` ports -- with the single query `validate_exclusivity()` calls.

## Manifest-Driven Remote Streaming Invoke and Gateway Discovery

These requirements cover routing `provided`/`stream_provided`/`stream_consumed`-kind
endpoints through `pcl_transport_routing_load()` and carrying real
cross-process RPC traffic over the routed transport, beyond the pub/sub-only
routing the earlier manifest requirements exercise.

### REQ_PCL_470 - Manifest-Routed Streaming Invoke Dispatches Remotely
`pcl_executor_invoke_stream()` shall dispatch a streaming service invocation through the named peer transport a `stream_consumed`-kind route table entry designates, before considering the legacy single executor-wide transport or the intra-process fallback -- mirroring `pcl_executor_invoke_async()`'s existing two-tier (per-endpoint route, then legacy transport) structure, but keyed on `PCL_ENDPOINT_STREAM_CONSUMED` rather than the unary `PCL_ENDPOINT_CONSUMED` (see REQ_PCL_472).

**Traces**: PCL.078

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.StreamingInvokeDispatchesThroughRoutedTransport`. Also exercised end-to-end at `pim/test_harness/agra_seam_interchange_test.cpp` (run1: request leg AND requirement leg both rpc-realized, cross-process over shared memory, JSON and FlatBuffers).

### REQ_PCL_472 - Stream-Consumed Routes Require RPC_STREAM Capability
A manifest `route` line of kind `stream_consumed` shall fail closed with `PCL_ERR_STATE` and a diagnostic naming `RPC_STREAM` when routed to a transport that does not declare `PCL_CAP_RPC_STREAM`, even when that transport declares `PCL_CAP_RPC_UNARY` (i.e. is unary-rpc-capable but not streaming-capable) -- `stream_consumed` and `consumed` are distinct endpoint kinds precisely so `pcl_endpoint_required_caps()` can require the correct capability for each, rather than a streaming client route being satisfied by `PCL_CAP_RPC_UNARY` alone and only failing once actually invoked.

**Traces**: PCL.078

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.StreamingRouteRejectsStreamIncapableTransport` (routes against the unary-only `pcl_transport_nocaps_plugin` -- publish + invoke_async only, no invoke_stream); `PclCapabilities.EndpointRequiredCaps` (`PCL_ENDPOINT_STREAM_CONSUMED` maps to `PCL_CAP_RPC_STREAM`, not `PCL_CAP_RPC_UNARY`). Downstream consumers of the `PCL_ENDPOINT_STREAM_CONSUMED` kind (the generated `ConsumedService` route helpers and Ada `Configure_Consumed_Transport` select it for server-streaming rpcs; `contract_routing_manifest.py` emits `stream_consumed` route lines with the endpoint's reliability floor) are pinned by `pim/test_stream_consumed_routing.py`.

### REQ_PCL_473 - Explicit Local Route Bypasses Legacy Transport Fallback
An endpoint with an explicit `PCL_ROUTE_LOCAL` route entry shall be dispatched intra-process by `pcl_executor_invoke_async()`/`pcl_executor_invoke_stream()` without ever consulting the legacy single executor-wide transport (`e->has_transport`/`e->transport`), even when such a default transport is also registered on the executor. A `PCL_ROUTE_LOCAL` route is a deliberate override, not merely the absence of a remote route, so it must not fall through to a fallback that exists only for endpoints with no route table entry at all.

**Traces**: PCL.078

**Verification**: `test_pcl_executor.cpp::PclExecutor.InvokeAsyncExplicitLocalRouteBypassesDefaultTransport`, `test_pcl_streaming.cpp::PclStreaming.InvokeStreamExplicitLocalRouteBypassesDefaultTransport`. Implementation note: both `pcl_executor_invoke_async()` and `pcl_executor_invoke_stream()` gate the legacy transport fallback on the route not forcing `PCL_ROUTE_LOCAL`.

### REQ_PCL_471 - Routing-Manifest Gateway Container Discovery
`pcl_transport_routing_get_gateway(routing, peer_id, out_gateway)` shall return `PCL_OK` with the named peer's transport's gateway container (if its plugin exports one of the known optional gateway-accessor symbols) or `PCL_OK` with `*out_gateway = NULL` (if it exports none -- not an error), and `PCL_ERR_NOT_FOUND` when no transport was loaded for `peer_id`. The routing manifest loader itself shall not add this container to the executor; a component providing a `provided`/`stream_provided` endpoint over such a transport is responsible for retrieving, configuring, activating, and adding it.

**Traces**: PCL.078

**Verification**: `test_pcl_transport_routing.cpp::PclTransportRouting.GetGatewayReturnsUsableContainerForShmPeer`, `.GetGatewayReturnsNullForPluginWithoutGateway`, `.GetGatewayFailsClosedOnUnknownPeerAndBadArgs`. Also exercised for real end-to-end at `pim/test_harness/agra_seam_interchange_test.cpp` (the provider/MA role of every rpc-realized-request-leg scenario: run1 and run3, JSON and FlatBuffers) -- before this fix, inbound `SVC_REQ`/`STREAM_REQ` frames arriving over the shared-memory bus at a manifest-routed provider were silently dropped (nothing was registered to receive the internal gateway-subscriber topic those frames are re-posted to), so the provider process never observed the command at all and the consumer's pending RPC call hung until timeout.

## 39. Standalone Process Runtime

### REQ_PCL_476 - Process Runtime Creation And Null Safety
`pcl_process_runtime_create()` shall create a runtime-owned executor and return it through `pcl_process_runtime_executor()`. Process-runtime functions shall reject required null or empty arguments with `PCL_ERR_INVALID`; null accessors shall return their documented null value; and null shutdown or destroy calls shall be no-ops.

**Traces**: PCL.079, PCL.045

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.CreatesExecutorAndHandlesNullArguments`.

### REQ_PCL_477 - Process Runtime Codec Loading
`pcl_process_runtime_load_codec()` shall load a valid codec plugin into the process-wide registry and retain its plugin handle for the runtime lifetime. A missing or invalid plugin shall fail closed and record a diagnostic naming the codec-load operation.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.LoadsCodecAndReportsPluginFailure`.

### REQ_PCL_478 - Complete Per-Port Deployment Load
`pcl_process_runtime_load_ports_file()` shall accept one entry for every supplied logical-port descriptor, translate the selected RPC or publish/subscribe realization and every supported endpoint kind into a routing manifest, reuse identical peer configurations, atomically install the routes, and reject a second deployment-file load on the same runtime with `PCL_ERR_STATE`.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.LoadsCompletePortsFileAndRejectsSecondLoad`.

### REQ_PCL_479 - Process Runtime Gateway Lifecycle
After a deployment manifest loads, `pcl_process_runtime_load_ports_file()` shall discover each selected transport's optional gateway container, configure it, activate it, and add it to the runtime executor. `pcl_process_runtime_destroy()` shall remove, deactivate, and clean up every gateway before destroying its routing handle and executor.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.ActivatesAndCleansUpSharedMemoryGateway`.

### REQ_PCL_480 - Logical-Port Descriptor Name Validation
`pcl_process_runtime_load_ports_file()` shall reject an empty logical-port name, duplicate logical-port names, or a non-zero endpoint count paired with a null endpoint array before it opens or installs deployment data.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.RejectsInvalidDeploymentDescriptors`.

### REQ_PCL_481 - Deployment-File Entry Validation
`pcl_process_runtime_load_ports_file()` shall reject malformed lines, unknown directives, unknown realization modes, missing plugin configuration, unknown logical-port names, duplicate logical-port entries, omitted logical ports, and conflicting plugin configurations for the same peer. Each rejection shall record a diagnostic and leave no routing handle installed.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.RejectsInvalidPortsFileEntries`.

### REQ_PCL_482 - Process Runtime Peer Bound
`pcl_process_runtime_load_ports_file()` shall accept at most 64 distinct transport peers and shall reject a deployment requiring a sixty-fifth peer with `PCL_ERR_NOMEM` before installing the generated routing manifest.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.RejectsMoreThanMaximumPeers`.

### REQ_PCL_483 - Endpoint Descriptor Validation
`pcl_process_runtime_load_ports_file()` shall validate every generated endpoint descriptor before writing it to a temporary routing manifest. A null endpoint name or an endpoint kind outside the supported publisher, subscriber, provided, consumed, stream-provided, and stream-consumed kinds shall return `PCL_ERR_INVALID` with a diagnostic instead of dereferencing or formatting the invalid value.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.RejectsUnsupportedEndpointDescriptors`.

### REQ_PCL_484 - Generated Routing Failure Is Atomic
If the generated manifest cannot load a selected transport or validate its routes, `pcl_process_runtime_load_ports_file()` shall return the routing failure, include the routing diagnostic in its process-runtime error, and leave no installed routing handle.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.ReportsGeneratedRoutingFailure`.

### REQ_PCL_485 - Temporary Manifest Failure
If a temporary routing manifest cannot be created, `pcl_process_runtime_load_ports_file()` shall return `PCL_ERR_INVALID`, record a diagnostic, and leave no installed routing handle.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.ReportsTemporaryManifestCreationFailure`.

### REQ_PCL_486 - Normal Process Runtime Lifecycle
`pcl_process_runtime_run()` shall configure and activate the component, add it to the runtime executor, spin until shutdown is requested, then remove, deactivate, and clean up the component. A successful run shall leave the component in `PCL_STATE_UNCONFIGURED`.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.RunsComponentUntilShutdownRequest`.

### REQ_PCL_487 - Signal-Requested Process Shutdown
While `pcl_process_runtime_run()` is active, `SIGINT` or `SIGTERM` shall request graceful shutdown. The runtime shall restore the process's previous handlers before returning.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.SignalRequestsGracefulShutdown`.

### REQ_PCL_488 - Process Runtime Lifecycle Failure Unwind
If component configure, activate, deactivate, or cleanup fails, `pcl_process_runtime_run()` shall return the failing status, record a diagnostic naming the failed component operation, and execute every cleanup step made reachable by the completed lifecycle transitions.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.ReportsLifecycleFailures`.

### REQ_PCL_489 - Process Runtime Executor Add Failure
If the component cannot be added because the runtime executor has reached its container bound, `pcl_process_runtime_run()` shall return `PCL_ERR_NOMEM` and shall still deactivate and clean up the component.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.ReportsExecutorAddFailure`.

### REQ_PCL_490 - Missing Deployment File
If the requested per-port deployment file cannot be opened, `pcl_process_runtime_load_ports_file()` shall return `PCL_ERR_NOT_FOUND`, record a diagnostic naming the failed open operation, and install no routing handle.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.RejectsMissingPortsFile`.

### REQ_PCL_491 - Duration-Limited Process Run
When the runtime was created with a non-zero duration, `pcl_process_runtime_run()` shall stop after that many seconds without requiring a signal or explicit shutdown request, then perform the normal component cleanup sequence.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.DurationLimitStopsRun`.

### REQ_PCL_492 - Process Runtime Codec Bound
`pcl_process_runtime_load_codec()` shall retain at most 32 codec-plugin handles and shall reject a thirty-third successful-load attempt with `PCL_ERR_NOMEM` before opening the plugin.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.RejectsMoreThanMaximumCodecs`.

### REQ_PCL_493 - Deployment File Read Failure
If an opened per-port deployment file cannot be read, `pcl_process_runtime_load_ports_file()` shall return `PCL_ERR_INVALID`, record a diagnostic naming the failed read operation, and install no routing handle.

**Traces**: PCL.079

**Verification**: `test_pcl_process_runtime.cpp::PclProcessRuntime.ReportsPortsFileReadFailure`.

### REQ_PCL_494 - Process Runtime Allocation Failure
`pcl_process_runtime_create()` shall return `PCL_ERR_NOMEM` without leaking when allocation of either the runtime object or its executor fails.

**Traces**: PCL.079, PCL.046

**Verification**: `test_pcl_oom.cpp::PclOom.ProcessRuntimeAllocationFails`, `test_pcl_oom.cpp::PclOom.ProcessRuntimeExecutorAllocationFails`.

## 40. Transport Flow Control and Monitoring

### REQ_PCL_495 - Remote Subscriber Registration During Container Add
When `pcl_executor_add()` adds a remote subscriber, it shall call `subscribe` with the port's topic and type on each already-registered named transport selected by the port, or on the default transport when the port selects no named peer. It shall not register local-only ports or call transports that the port did not select.

**Traces**: PCL.080

**Verification**: `test_pcl_executor.cpp::PclExecutor.SubscriberPortRegistersWithNamedTransportDuringSetup`, `test_pcl_executor.cpp::PclExecutor.DefaultTransportRegistersUnboundRemoteSubscriberOnly`.

### REQ_PCL_496 - Remote Subscriber Registration During Late Transport Add
When `pcl_executor_register_transport()` registers a named transport after containers have been added, it shall call `subscribe` for every existing remote subscriber that selects that peer and shall not call it for subscribers that select another peer or the default transport. When `pcl_executor_set_transport()` installs a default transport after containers have been added, it shall register every remote subscriber that selects no named peer.

**Traces**: PCL.080

**Verification**: `test_pcl_executor.cpp::PclExecutor.LateNamedTransportRegistersExistingSubscriberPort`, `test_pcl_executor.cpp::PclExecutor.LateDefaultTransportRegistersUnboundRemoteSubscriber`.

### REQ_PCL_497 - Unsupported Or Rejected Subscription Fails Closed
If a selected transport has no `subscribe` operation, subscriber registration shall return `PCL_ERR_NOT_FOUND` with a diagnostic. If the transport's `subscribe` operation returns an error, the executor operation performing registration shall return that error without adding the container or retaining the named transport registration.

**Traces**: PCL.080, PCL.045

**Verification**: `test_pcl_executor.cpp::PclExecutor.SelectedTransportWithoutSubscribeFailsClosed`, `test_pcl_executor.cpp::PclExecutor.RejectedLateSubscriptionRollsBackTransport`.

### REQ_PCL_498 - Executor Incoming Queue Limit
`pcl_executor_set_incoming_queue_limit()` shall set the incoming publish/subscribe queue bound and shall reject a null executor with `PCL_ERR_INVALID`. A zero bound shall permit more than the formerly configured non-zero bound. When a post reaches a non-zero bound, it shall return `PCL_ERR_NOMEM` without increasing the queue depth or taking ownership of the caller's message.

**Traces**: PCL.081

**Verification**: `test_pcl_executor.cpp::PclExecutor.IncomingQueueLimitRejectsExcessMessages`.

### REQ_PCL_499 - Executor Incoming Queue Depth
`pcl_executor_get_incoming_queue_depth()` shall report the number of queued incoming publish/subscribe messages, shall decrease as executor spins drain messages, and shall return zero for a null executor.

**Traces**: PCL.081

**Verification**: `test_pcl_executor.cpp::PclExecutor.IncomingQueueLimitRejectsExcessMessages`.

### REQ_PCL_500 - Shared-Memory Subscription Interest
The shared-memory transport `subscribe` operation shall record a valid topic once per participant, return `PCL_OK` for a duplicate, reject an empty or overlong topic, and return `PCL_ERR_NOMEM` after 16 distinct topics. Publish shall target only participants that advertised the topic and shall return `PCL_ERR_NOT_FOUND` when none are interested.

**Traces**: PCL.082

**Verification**: `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.PublishTargetsOnlyInterestedParticipants`, `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.SubscriptionValidationAndCapacity`.

### REQ_PCL_501 - Shared-Memory Unary Service Congestion
When a discovered unary service provider's mailbox is full, the shared-memory egress worker shall retry enqueueing the request for no longer than its configured bounded retry interval. If retry succeeds, the normal response shall be delivered; otherwise the pending call shall complete on the executor thread with an empty terminal response.

**Traces**: PCL.082, PCL.075

**Verification**: `test_pcl_shared_memory_transport.cpp::PclSharedMemoryTransport.UnaryServiceCongestionRetriesThenCompletesEmpty`.

### REQ_PCL_502 - Reliable-Socket Outbound Queue Bound
The reliable socket transport shall reject an outbound frame with `PCL_ERR_NOMEM` when accepting it would make the queued byte count exceed 16 MiB. The rejected frame shall be freed and shall increment `pcl_socket_transport_dropped_publishes()` exactly once; the getter shall be thread-safe and shall return zero for a null transport.

**Traces**: PCL.083

**Verification**: `test_pcl_socket_transport.cpp::PclSocketTransport.OutboundQueueLimitReportsDrops`.

### REQ_PCL_503 - Reliable-Socket Teardown Drains Accepted Frames
`pcl_socket_transport_destroy()` shall signal the send worker to drain every frame accepted before teardown, join the worker, and reclaim any frame remaining after a send failure.

**Traces**: PCL.083, PCL.075

**Verification**: `test_pcl_socket_transport.cpp::PclSocketTransport.DestroyWithUnsentFramesNoLeak`.

### REQ_PCL_504 - UDP Received-Datagram Counter
`pcl_udp_transport_received_datagrams()` shall count every datagram returned by the receive socket, including malformed and unsupported datagrams, and shall return zero for a null transport.

**Traces**: PCL.084

**Verification**: `test_pcl_udp_transport.cpp::PclUdpTransport.MalformedDatagramsIncreaseReceivedCounter`.

### REQ_PCL_505 - UDP Per-Source Sequence-Gap Counter
For each source address and port, the UDP transport shall establish a sequence baseline from the first valid publish datagram and add each later forward sequence gap to `pcl_udp_transport_dropped_datagrams()`. Duplicate, reordered, or stale sequence values shall not increase the count. The getter shall return zero for a null transport.

**Traces**: PCL.084

**Verification**: `test_pcl_udp_transport.cpp::PclUdpTransport.SequenceGapsAreCountedPerSource`.
