# PCL Low-Level Requirements

Low-level requirements for the PYRAMID Container Library (PCL), derived from the high-level requirements in `HLR.md`.

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
The container shall support the full transition cycle: UNCONFIGURED → CONFIGURED → ACTIVE → CONFIGURED → UNCONFIGURED → FINALIZED, invoking the corresponding callback at each step.

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

**Traces**: PCL.007

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

**Traces**: PCL.045

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

**Traces**: PCL.023

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

**Traces**: PCL.022, PCL.010

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

**Traces**: PCL.030a

**Verification**: `test_pcl_robustness.cpp::InvokeAsyncRoutesViaTransport`.

### REQ_PCL_165 - Invoke Async Intra-Process Fallback
`pcl_executor_invoke_async()` shall fall back to intra-process synchronous dispatch when no transport is set or when the transport's `invoke_async` is NULL. The service handler shall be invoked immediately and the callback fired before returning.

**Traces**: PCL.030a, PCL.022

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

**Verification**: `test_pcl_dining.cpp::CreateAndDestroy`.

### REQ_PCL_076 - Bridge Null Arguments Return NULL
`pcl_bridge_create()` shall return NULL when any mandatory argument is NULL.

**Traces**: PCL.044

**Verification**: `test_pcl_dining.cpp::CreateNullArgsReturnNull` tests all NULL argument combinations.

### REQ_PCL_077 - Bridge Container Null Returns NULL
`pcl_bridge_container(NULL)` shall return NULL.

**Traces**: PCL.044

**Verification**: `test_pcl_dining.cpp::ContainerNullBridgeReturnsNull`.

### REQ_PCL_078 - Bridge Destroy Null Is No-Op
`pcl_bridge_destroy(NULL)` shall be a no-op.

**Traces**: PCL.044

**Verification**: `test_pcl_dining.cpp::DestroyNullIsNoOp`.

### REQ_PCL_079 - Bridge Unit Conversion
A bridge with a transform function shall convert messages (e.g., float m/s → int32 km/h) and dispatch to the output topic.

**Traces**: PCL.042

**Verification**: `test_pcl_dining.cpp::SpeedUnitConversion` dispatches 10.0 m/s and verifies 36 km/h, then 27.78 m/s and verifies 100 km/h.

### REQ_PCL_080 - Bridge Fail Transform Suppresses Forward
When the transform function returns non-OK, no message shall be dispatched to the output topic.

**Traces**: PCL.043

**Verification**: `test_pcl_dining.cpp::FailTransformSuppressesForward`.

### REQ_PCL_081 - Bridge Inactive Does Not Forward
An inactive bridge (CONFIGURED but not ACTIVE) shall not forward messages.

**Traces**: PCL.041

**Verification**: `test_pcl_dining.cpp::BridgeInactiveDoesNotForward`.

### REQ_PCL_082 - Bridge Configure Fails When Ports Full
If the bridge's internal container has no free port slots, configure shall fail with `PCL_ERR_NOMEM`.

**Traces**: PCL.041, PCL.046

**Verification**: `test_pcl_dining.cpp::ConfigureFailsWhenPortsFull`.

### REQ_PCL_083 - Bridge Dispatch Not Found Logs Debug
When the bridge's output topic has no subscriber, the bridge shall log a DEBUG message and not crash.

**Traces**: PCL.041

**Verification**: `test_pcl_dining.cpp::DispatchNotFoundLogsDebug`.

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

**Verification**: Existing client connect tests against `"127.0.0.1"` exercise the getaddrinfo path via `ClientCreationAndDestroy` and `ClientExRetryConnectsToDelayedServer`.

### REQ_PCL_197 - TCP Keepalive Enabled On Connected Sockets
Every connected TCP socket (client-connect and server-accept) shall have `SO_KEEPALIVE` enabled. On Linux, `TCP_KEEPIDLE`, `TCP_KEEPINTVL`, and `TCP_KEEPCNT` shall be tuned so silent peer death is detected within approximately eight seconds.

**Traces**: PCL.031, PCL.032, PCL.036e

**Verification**: Implicit — covered by `AutoReconnectAfterServerRestart` which depends on timely peer-death detection.

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
| 167 | 011c | test_pcl_streaming |
| 168 | 011c | test_pcl_streaming |
| 169 | 011c | test_pcl_streaming |
| 170 | 011c, 045 | test_pcl_streaming |
| 171 | 011c | test_pcl_streaming |
| 172 | 011c | test_pcl_streaming |

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
