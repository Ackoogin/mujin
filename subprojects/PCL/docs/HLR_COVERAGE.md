# HLR Requirements Coverage

End-to-end traceability: **Test → LLR → HLR** for all PCL requirements (PCL.001–PCL.056).

Each test traces to one or more LLRs (REQ_PCL_NNN). Each LLR traces to one or more HLRs (PCL.0XX). This document confirms the chain is complete.

## Container Lifecycle Coverage

| PCL (HLR) | Description | LLR(s) | Test File(s) | Test(s) |
|------------|-------------|--------|-------------|---------|
| PCL.001 | Lifecycle state machine | 001, 003 | test_pcl_lifecycle | CreateDestroy, FullTransitionCycle |
| PCL.002 | Lifecycle transitions | 004, 005, 006 | test_pcl_lifecycle | FullTransitionCycle, InvalidTransitionsRejected, ShutdownFromAnyState |
| PCL.003 | Transition callback invocation | 004, 008 | test_pcl_lifecycle | FullTransitionCycle, NullCallbacksAreNoOp |
| PCL.004 | Callback failure aborts transition | 007, 009, 010, 011 | test_pcl_lifecycle, test_pcl_robustness | CallbackFailureAbortsTransition, DeactivateCallbackFailure*, ActivateCallbackFailure*, CleanupCallbackFailure* |
| PCL.005 | Container identity | 001, 114 | test_pcl_lifecycle, test_pcl_robustness | CreateDestroy, LongNameTruncated |

## Port Management Coverage

| PCL (HLR) | Description | LLR(s) | Test File(s) | Test(s) |
|------------|-------------|--------|-------------|---------|
| PCL.006 | Port types | 019, 022 | test_pcl_lifecycle, test_pcl_robustness | CreatedDuringConfigure, AddServiceSuccessPath, PortCreationNullArgs |
| PCL.007 | Port creation during configure only | 012, 019, 020 | test_pcl_lifecycle, test_pcl_robustness | CreatedDuringConfigure, RejectedOutsideConfigure, CleanupResetsPortCount |
| PCL.008 | Port capacity limit (64) | 021 | test_pcl_robustness | PortOverflowReturnNull |
| PCL.009 | Publish constraint (ACTIVE only) | 023, 024, 026, 027, 106, 107 | test_pcl_robustness, test_pcl_proto_bindings | PublishOnInactiveContainerReturnsClosed, PublishSuccessOnActiveContainer, PublishOnSubscriberPortReturnsInvalid, PublishReturnsBadStatusWhenInactive, PublishSucceedsWhenActive |
| PCL.010 | Subscriber callback dispatch | 044 | test_pcl_executor | IntraProcessPubSub |
| PCL.011 | Service handler dispatch | 040 | test_pcl_robustness | InvokeServiceFound |

## Parameter Coverage

| PCL (HLR) | Description | LLR(s) | Test File(s) | Test(s) |
|------------|-------------|--------|-------------|---------|
| PCL.012 | Typed parameter storage | 013, 014, 015 | test_pcl_lifecycle | StringRoundTrip, NumericTypes, OverwriteExisting |
| PCL.013 | Parameter capacity limit (128) | 016 | test_pcl_robustness | ParamOverflowReturnsNomem |
| PCL.014 | Parameter default values | 013, 017 | test_pcl_lifecycle, test_pcl_robustness | StringRoundTrip, ParamTypeMismatchReturnsDefault |

## Tick Rate Coverage

| PCL (HLR) | Description | LLR(s) | Test File(s) | Test(s) |
|------------|-------------|--------|-------------|---------|
| PCL.015 | Configurable tick rate | 028, 029, 092 | test_pcl_lifecycle, test_pcl_robustness | DefaultIs100Hz, SetAndGet, HighFrequencyTickRate |
| PCL.016 | Tick rate validation | 030 | test_pcl_lifecycle | SetAndGet (negative, zero rejected) |
| PCL.017 | Delta time reporting | 043 | test_pcl_robustness | SpinOnceFirstCallUsesDefaultDt |

## Executor Coverage

| PCL (HLR) | Description | LLR(s) | Test File(s) | Test(s) |
|------------|-------------|--------|-------------|---------|
| PCL.018 | Multi-container execution | 031, 032, 033, 034, 084, 091, 093 | test_pcl_executor, test_pcl_dining, test_pcl_robustness | SpinOnceTicksActiveContainer, InactiveContainerNotTicked, MultipleContainers, FivePhilosophersWithBridges, SpinOnceBurstDispatch, MultiContainerDifferentRates |
| PCL.019 | Spin and spin-once | 032 | test_pcl_executor | SpinOnceTicksActiveContainer |
| PCL.020 | Shutdown request | 035, 055 | test_pcl_executor, test_pcl_robustness | RequestShutdownStopsSpin, SpinInBackgroundShutdownFromForeground, RequestShutdownNullSafe |
| PCL.021 | Graceful shutdown | 036, 041, 042 | test_pcl_executor, test_pcl_robustness | GracefulShutdownFinalizesContainers, GracefulShutdownTimeout, GracefulShutdownAlreadyFinalized |
| PCL.022 | Intra-process direct dispatch | 027, 044, 045, 047 | test_pcl_executor, test_pcl_robustness | IntraProcessPubSub, DispatchToUnknownTopicReturnsNotFound, PublishNoTransportDispatchesToSubscriber, PublishWithExecutorRoutes |
| PCL.023 | Service invocation | 040 | test_pcl_robustness | InvokeServiceFound |
| PCL.024 | Container add and remove | 037, 038, 039 | test_pcl_robustness | ContainerOverflowReturnsNomem, AddNullContainerReturnsInvalid, RemoveContainerSuccess |

## Cross-Thread Ingress Coverage

| PCL (HLR) | Description | LLR(s) | Test File(s) | Test(s) |
|------------|-------------|--------|-------------|---------|
| PCL.025 | Cross-thread message posting | 049, 050, 051, 052, 053, 055, 056 | test_pcl_executor, test_pcl_robustness | ExternalThreadPostsCopiedMessageToExecutorThread, PostIncomingBadInputs, PostIncomingZeroSizeNoData, DestroyWithPendingMessagesFrees, ConcurrentPostFromManyThreads, PostDuringActiveSpinNoCorruption |
| PCL.026 | Ingress queue drain | 054 | test_pcl_robustness | ProducerConsumerWithSubscriber |
| PCL.027 | Async response delivery | 057, 058, 059, 060, 061 | test_pcl_robustness | PostResponseCbWithDataFiresOnSpin, PostResponseCbNoData, PostResponseCbMultipleOnQueue, PostResponseCbNullSafety, DestroyWithQueuedRespCbWithData |
| PCL.057 | Cross-thread local service request queuing | 180, 181, 182, 183, 184, 185 | test_pcl_robustness | PostServiceRequestDeepCopiesRequest, PostServiceRequestFiresHandlerOnExecutorThread, PostServiceRequestNotFoundFiresEmptyCallback, PostServiceRequestMultipleQueuedFireInOrder, PostServiceRequestNullSafety, DestroyWithPendingServiceRequestsFrees |

## Transport Adapter Coverage

| PCL (HLR) | Description | LLR(s) | Test File(s) | Test(s) |
|------------|-------------|--------|-------------|---------|
| PCL.028 | Transport adapter interface | 048 | test_pcl_robustness | PublishWithTransportCallsTransportPublish |
| PCL.029 | Transport wiring | 048, 062, 063 | test_pcl_robustness | SetTransportAndClear, SetTransportNullExecutorReturnsInvalid |
| PCL.030 | Dispatch incoming from transport | 044 | test_pcl_executor | IntraProcessPubSub |
| PCL.030a | Transport client service invocation | 164, 165, 166 | test_pcl_robustness | InvokeAsyncRoutesViaTransport, InvokeAsyncIntraProcessFallback, InvokeAsyncNullArgsReturnsInvalid |
| PCL.030b | Endpoint routing configuration | 173, 174, 175 | test_pcl_executor | RemoteIngressHonorsSubscriberPeerRoute, PublisherRouteCanBeLocalAndRemote, ConsumedServiceRouteUsesNamedPeerTransport |
| PCL.030c | Named peer transport registration | 174, 175, 178 | test_pcl_executor | PublisherRouteCanBeLocalAndRemote, ConsumedServiceRouteUsesNamedPeerTransport |
| PCL.030d | Remote ingress filtering | 173 | test_pcl_executor | RemoteIngressHonorsSubscriberPeerRoute |

## TCP Socket Transport Coverage

| PCL (HLR) | Description | LLR(s) | Test File(s) | Test(s) |
|------------|-------------|--------|-------------|---------|
| PCL.031 | Server mode | 115, 116, 158, 159 | test_pcl_socket_transport | ServerCreationAndDestroy, ServerNullExecutorReturnsNull, SubscribeVtableCallSucceeds, ShutdownVtableCallSucceeds |
| PCL.032 | Client mode | 117, 118, 119 | test_pcl_socket_transport | ClientCreationAndDestroy, ClientNullArgsReturnNull, ClientConnectToNonexistentServerFails |
| PCL.033 | Wire protocol | 120, 121 | test_pcl_socket_transport | PublishServerToClientDelivered, PublishClientToServerDelivered |
| PCL.034 | Gateway container | 122, 123, 124, 163 | test_pcl_socket_transport | GatewayContainerExists, GatewayContainerNullReturnsNull, ClientHasNoGateway, GatewayServiceDispatchNoMatch |
| PCL.035 | Non-blocking send | 125, 161 | test_pcl_socket_transport | PublishIsNonBlocking, DestroyWithUnsentFramesNoLeak |
| PCL.036 | Async remote service invocation | 126, 127, 128, 160, 162, 163 | test_pcl_socket_transport | AsyncRemoteServiceRoundTrip, InvokeRemoteAsyncNullArgs, InvokeRemoteAsyncOnServerReturnsInvalid, DestroyWithPendingAsyncCallNoLeak, InvokeRemoteAsyncOversizedPayloadReturnsNomem, GatewayServiceDispatchNoMatch |
| PCL.036a | Remote peer identity | 176, 177, 179 | test_pcl_socket_transport | PublishServerToClientDelivered, PublishClientToServerDelivered, AsyncRemoteServiceRoundTrip |

## Routing API Coverage

| PCL (HLR) | Description | LLR(s) | Test File(s) | Test(s) |
|------------|-------------|--------|-------------|---------|
| PCL.056 | Public route configuration API | 179 | test_pcl_socket_transport | PublishServerToClientDelivered, PublishClientToServerDelivered, AsyncRemoteServiceRoundTrip |

## Logging Coverage

| PCL (HLR) | Description | LLR(s) | Test File(s) | Test(s) |
|------------|-------------|--------|-------------|---------|
| PCL.037 | Printf-style logging | 064, 065, 067, 071, 073, 074 | test_pcl_log, test_pcl_robustness | CustomHandlerReceivesMessages, ContainerNamePassedThrough, FormatString, LongFormatStringTruncated, DefaultHandlerNoContainerName, DefaultHandlerWithContainerName |
| PCL.038 | Pluggable log handler | 064, 068, 072 | test_pcl_log, test_pcl_robustness | CustomHandlerReceivesMessages, RevertToDefaultHandler, HandlerUserDataPassedThrough |
| PCL.039 | Log level filtering | 066, 070 | test_pcl_log, test_pcl_robustness | LevelFiltering, FilterBlocksAllBelowFatal |
| PCL.040 | Log levels (5 levels) | 069 | test_pcl_robustness | AllLevelsReachHandler |

## Bridge Coverage

| PCL (HLR) | Description | LLR(s) | Test File(s) | Test(s) |
|------------|-------------|--------|-------------|---------|
| PCL.041 | Bridge creation | 075, 081, 082, 083 | test_pcl_dining | CreateAndDestroy, BridgeInactiveDoesNotForward, ConfigureFailsWhenPortsFull, DispatchNotFoundLogsDebug |
| PCL.042 | Bridge transform dispatch | 079, 084 | test_pcl_dining | SpeedUnitConversion, FivePhilosophersWithBridges |
| PCL.043 | Bridge message suppression | 080 | test_pcl_dining | FailTransformSuppressesForward |
| PCL.044 | Bridge null safety | 076, 077, 078 | test_pcl_dining | CreateNullArgsReturnNull, ContainerNullBridgeReturnsNull, DestroyNullIsNoOp |

## Robustness and Error Handling Coverage

| PCL (HLR) | Description | LLR(s) | Test File(s) | Test(s) |
|------------|-------------|--------|-------------|---------|
| PCL.045 | Null handle safety | 002, 018, 022, 025, 038, 046, 050, 060, 063, 111, 112, 113, 114, 116, 118, 123, 127, 129, 130 | test_pcl_lifecycle, test_pcl_executor, test_pcl_robustness, test_pcl_socket_transport | NullHandlesReturnError, NullSafety, ParamNullArgs, ParamGetNullArgs, PortCreationNullArgs, PublishNullPortOrMsg, PostIncomingBadInputs, DispatchIncomingNullArgs, AddNullContainerReturnsInvalid, PostResponseCbNullSafety, SetTransportNullExecutorReturnsInvalid, RequestShutdownNullSafe, LongNameTruncated, ServerNullExecutorReturnsNull, ClientNullArgsReturnNull, GatewayContainerNullReturnsNull, InvokeRemoteAsyncNullArgs, GetTransportNullReturnsNull, DestroyNullIsNoOp |
| PCL.046 | Allocation failure handling | 016, 021, 037, 052, 061, 082, 085, 086, 087, 088, 089, 090 | test_pcl_oom, test_pcl_robustness, test_pcl_dining | PostIncomingPendingAllocFails, PostIncomingTopicStrdupFails, PostIncomingTypeNameStrdupFails, PostIncomingDataMallocFails, BridgeCreateContainerAllocFails, PostResponseCbDataMallocFails, ParamOverflowReturnsNomem, PortOverflowReturnNull, ContainerOverflowReturnsNomem, DestroyWithPendingMessagesFrees, DestroyWithQueuedRespCbWithData |
| PCL.047 | Status codes | All | All test files | (Verified implicitly by all tests checking pcl_status_t return values) |

## C++ Wrapper Coverage

| PCL (HLR) | Description | LLR(s) | Test File(s) | Test(s) |
|------------|-------------|--------|-------------|---------|
| PCL.048 | Component base class | 131–144, 157 | test_pcl_cpp_wrappers | ConstructionProducesValidHandle, DestructorFreesResources, LifecycleCallbacksInvoked, AddPublisherDuringConfigure, AddSubscriberDuringConfigure, AddServiceDuringConfigure, ParamStringRoundTrip, ParamDoubleRoundTrip, ParamInt64RoundTrip, ParamBoolRoundTrip, TickRateRoundTrip, LoggingDoesNotCrash, MoveConstructor, MoveAssignment, ComponentLifecycleViaExecutor |
| PCL.049 | Executor wrapper | 145–157 | test_pcl_cpp_wrappers | ConstructionProducesValidHandle, DestructorFreesResources, AddComponent, AddRawContainer, SpinOnceTicksComponent, RequestShutdownStopsSpin, ShutdownGracefulFinalizesComponents, SetTransport, DispatchIncoming, PostIncoming, MoveConstructor, MoveAssignment, ComponentLifecycleViaExecutor |

## Service Bindings Coverage

| PCL (HLR) | Description | LLR(s) | Test File(s) | Test(s) |
|------------|-------------|--------|-------------|---------|
| PCL.050 | Wire-name constants | 094, 096 | test_pcl_proto_bindings | WireNames (Provided), WireNames (Consumed) |
| PCL.051 | Topic constants | 095, 097 | test_pcl_proto_bindings | TopicNames (Provided), TopicNames (Consumed) |
| PCL.052 | Service handler base class | 102, 103, 110 | test_pcl_proto_bindings | HandlerStubsReturnEmpty (Provided), HandlerStubsReturnEmpty (Consumed), AckConstants, QueryDefault, IdentifierIsString, ObjectDetailDefaults |
| PCL.053 | JSON builder functions | 098, 099, 100, 101 | test_pcl_proto_bindings | BuildRequirementJsonKeys, BuildRequirementJsonNoDimension, BuildEvidenceJsonKeys, BuildEvidenceJsonDefaultObservedAt, MsgToString (Provided), MsgToString (Consumed), MsgToStringEmpty |
| PCL.054 | Subscribe wrappers | 104, 105 | test_pcl_proto_bindings | SubscribeRegistrationSucceeds (Provided), SubscribeRegistrationSucceeds (Consumed) |
| PCL.055 | Dispatch function | 108, 109 | test_pcl_proto_bindings | DispatchAllChannelsNoCrash (Provided), DispatchAllChannelsNoCrash (Consumed) |

---

## Coverage Summary

| Category | HLRs | LLRs | Test Files |
|----------|-------|------|------------|
| Container Lifecycle | 5 (001–005) | 14 (001–012, 111, 114) | test_pcl_lifecycle, test_pcl_robustness |
| Port Management | 6 (006–011) | 9 (019–027) | test_pcl_lifecycle, test_pcl_robustness |
| Parameters | 3 (012–014) | 6 (013–018) | test_pcl_lifecycle, test_pcl_robustness |
| Tick Rate | 3 (015–017) | 3 (028–030) | test_pcl_lifecycle, test_pcl_robustness |
| Executor | 11 (018–024, 030a–030d) | 20 (031–043, 164–166, 173–175, 178) | test_pcl_executor, test_pcl_robustness |
| Cross-Thread Ingress | 3 (025–027) | 13 (049–061) | test_pcl_executor, test_pcl_robustness |
| Transport Adapter | 3 (028–030) | 2 (062–063) | test_pcl_robustness |
| TCP Socket Transport | 7 (031–036a) | 25 (115–130, 158–163, 176, 177, 179) | test_pcl_socket_transport |
| Logging | 4 (037–040) | 11 (064–074) | test_pcl_log, test_pcl_robustness |
| Bridge | 4 (041–044) | 9 (075–083) | test_pcl_dining |
| Integration | — | 1 (084) | test_pcl_dining |
| Robustness / OOM | 3 (045–047) | 6 (085–090) | test_pcl_oom, test_pcl_robustness |
| Throughput | — | 3 (091–093) | test_pcl_robustness |
| C++ Component Wrapper | 1 (048) | 14 (131–144) | test_pcl_cpp_wrappers |
| C++ Executor Wrapper | 1 (049) | 12 (145–156) | test_pcl_cpp_wrappers |
| C++ Integration | — | 1 (157) | test_pcl_cpp_wrappers |
| Service Bindings | 6 (050–055) | 17 (094–110) | test_pcl_proto_bindings |
| Routing API | 1 (056) | 1 (179) | test_pcl_socket_transport |
| Null Safety | — | 4 (111–114) | test_pcl_lifecycle, test_pcl_executor, test_pcl_robustness |
| **Total** | **60** | **179** | **9 test files** |

## Test File Registry

| File | Purpose | LLRs | Test Count |
|------|---------|------|------------|
| `test_pcl_lifecycle.cpp` | Container lifecycle, parameters, ports, tick rate, null safety | 001–008, 013–015, 019–020, 028–030, 111 | 13 |
| `test_pcl_executor.cpp` | Executor spin, dispatch, multi-container, shutdown, cross-thread ingress, endpoint routing | 031–036, 044–045, 049, 055, 112, 173–175, 178 | 13 |
| `test_pcl_log.cpp` | Logging handler, level filtering, format strings | 064–068 | 5 |
| `test_pcl_dining.cpp` | Bridge unit tests + dining philosophers integration | 075–084 | 11 |
| `test_pcl_oom.cpp` | Out-of-memory injection for allocation failure branches | 085–090 | 6 |
| `test_pcl_robustness.cpp` | 100% code coverage: overflow, threading, throughput, edge cases | 009–012, 016–018, 021–027, 037–043, 046–048, 050–054, 056–063, 069–074, 091–093, 113–114 | 55 |
| `test_pcl_socket_transport.cpp` | TCP socket transport: server/client, wire protocol, gateway, async service | 115–130, 158–163 | 22 |
| `test_pcl_cpp_wrappers.cpp` | C++ wrappers: pcl::Component lifecycle/params/ports, pcl::Executor spin/shutdown | 131–157 | 27 |
| `test_pcl_proto_bindings.cpp` | C++ service bindings: wire names, handlers, dispatch, JSON builders | 094–110 | 19 |

## Trace Tag Format

Tests use LLR requirement tags in comments, with the parent HLR noted for traceability:

```
///< REQ_PCL_NNN: Brief description. PCL.0XX.
```

Each LLR in `LLR.md` has a **Traces** field listing the parent HLR(s), completing the chain:

```
Test (code comment) → LLR (LLR.md) → HLR (HLR.md)
```

Run tests with filter to verify a specific requirement area:

```bat
build\tests\Release\test_pcl_robustness.exe --gtest_filter=PclContainerRobust*
build\tests\Release\test_pcl_lifecycle.exe --gtest_filter=PclLifecycle*
build\tests\Release\test_pcl_proto_bindings.exe --gtest_filter=ProtoBindings*
build\tests\Release\test_pcl_socket_transport.exe --gtest_filter=PclSocketTransport*
build\tests\Release\test_pcl_cpp_wrappers.exe --gtest_filter=PclCppComponent*
```

## Requirements Coverage Status

All 60 HLRs (PCL.001–PCL.056) are covered by 179 LLRs across 9 test files (165 tests total). Every test traces to at least one LLR, and every LLR traces to at least one HLR, providing complete end-to-end traceability.

Statement coverage: **96.3%** line coverage, **100%** function coverage across all PCL source files. See `coverage_pcl/summary.txt` for per-file details.
