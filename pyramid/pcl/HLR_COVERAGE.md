# HLR Requirements Coverage

Traceability of HLR requirements (PCL.001–PCL.055) to tests across all PCL test suites.

## Container Lifecycle Coverage

| PCL | Description | Test File(s) | Test(s) |
|-----|-------------|-------------|---------|
| PCL.001 | Lifecycle state machine | test_pcl_lifecycle | CreateDestroy, FullTransitionCycle |
| PCL.002 | Lifecycle transitions | test_pcl_lifecycle | FullTransitionCycle, InvalidTransitionsRejected, ShutdownFromAnyState |
| PCL.003 | Transition callback invocation | test_pcl_lifecycle | FullTransitionCycle, NullCallbacksAreNoOp |
| PCL.004 | Callback failure aborts transition | test_pcl_lifecycle, test_pcl_robustness | CallbackFailureAbortsTransition, DeactivateCallbackFailure*, ActivateCallbackFailure*, CleanupCallbackFailure* |
| PCL.005 | Container identity | test_pcl_lifecycle, test_pcl_robustness | CreateDestroy, LongNameTruncated |

## Port Management Coverage

| PCL | Description | Test File(s) | Test(s) |
|-----|-------------|-------------|---------|
| PCL.006 | Port types | test_pcl_lifecycle, test_pcl_robustness | CreatedDuringConfigure, AddServiceSuccessPath, PortCreationNullArgs |
| PCL.007 | Port creation during configure only | test_pcl_lifecycle, test_pcl_robustness | CreatedDuringConfigure, RejectedOutsideConfigure, CleanupResetsPortCount |
| PCL.008 | Port capacity limit (64) | test_pcl_robustness | PortOverflowReturnNull |
| PCL.009 | Publish constraint (ACTIVE only) | test_pcl_robustness, test_pcl_proto_bindings | PublishOnInactiveContainerReturnsClosed, PublishSuccessOnActiveContainer, PublishOnSubscriberPortReturnsInvalid, PublishReturnsBadStatusWhenInactive, PublishSucceedsWhenActive |
| PCL.010 | Subscriber callback dispatch | test_pcl_executor | IntraProcessPubSub |
| PCL.011 | Service handler dispatch | test_pcl_robustness | InvokeServiceFound |

## Parameter Coverage

| PCL | Description | Test File(s) | Test(s) |
|-----|-------------|-------------|---------|
| PCL.012 | Typed parameter storage | test_pcl_lifecycle | StringRoundTrip, NumericTypes, OverwriteExisting |
| PCL.013 | Parameter capacity limit (128) | test_pcl_robustness | ParamOverflowReturnsNomem |
| PCL.014 | Parameter default values | test_pcl_lifecycle, test_pcl_robustness | StringRoundTrip, ParamTypeMismatchReturnsDefault |

## Tick Rate Coverage

| PCL | Description | Test File(s) | Test(s) |
|-----|-------------|-------------|---------|
| PCL.015 | Configurable tick rate | test_pcl_lifecycle, test_pcl_robustness | DefaultIs100Hz, SetAndGet, HighFrequencyTickRate |
| PCL.016 | Tick rate validation | test_pcl_lifecycle | SetAndGet (negative, zero rejected) |
| PCL.017 | Delta time reporting | test_pcl_robustness | SpinOnceFirstCallUsesDefaultDt |

## Executor Coverage

| PCL | Description | Test File(s) | Test(s) |
|-----|-------------|-------------|---------|
| PCL.018 | Multi-container execution | test_pcl_executor, test_pcl_dining, test_pcl_robustness | SpinOnceTicksActiveContainer, InactiveContainerNotTicked, MultipleContainers, FivePhilosophersWithBridges, SpinOnceBurstDispatch, MultiContainerDifferentRates |
| PCL.019 | Spin and spin-once | test_pcl_executor | SpinOnceTicksActiveContainer |
| PCL.020 | Shutdown request | test_pcl_executor, test_pcl_robustness | RequestShutdownStopsSpin, SpinInBackgroundShutdownFromForeground, RequestShutdownNullSafe |
| PCL.021 | Graceful shutdown | test_pcl_executor, test_pcl_robustness | GracefulShutdownFinalizesContainers, GracefulShutdownTimeout, GracefulShutdownAlreadyFinalized |
| PCL.022 | Intra-process direct dispatch | test_pcl_executor, test_pcl_robustness | IntraProcessPubSub, DispatchToUnknownTopicReturnsNotFound, PublishNoTransportDispatchesToSubscriber, PublishWithExecutorRoutes |
| PCL.023 | Service invocation | test_pcl_robustness | InvokeServiceFound |
| PCL.024 | Container add and remove | test_pcl_robustness | ContainerOverflowReturnsNomem, AddNullContainerReturnsInvalid, RemoveContainerSuccess |

## Cross-Thread Ingress Coverage

| PCL | Description | Test File(s) | Test(s) |
|-----|-------------|-------------|---------|
| PCL.025 | Cross-thread message posting | test_pcl_executor, test_pcl_robustness | ExternalThreadPostsCopiedMessageToExecutorThread, PostIncomingBadInputs, PostIncomingZeroSizeNoData, DestroyWithPendingMessagesFrees, ConcurrentPostFromManyThreads, PostDuringActiveSpinNoCorruption |
| PCL.026 | Ingress queue drain | test_pcl_robustness | ProducerConsumerWithSubscriber |
| PCL.027 | Async response delivery | test_pcl_robustness | PostResponseCbWithDataFiresOnSpin, PostResponseCbNoData, PostResponseCbMultipleOnQueue, PostResponseCbNullSafety, DestroyWithQueuedRespCbWithData |

## Transport Adapter Coverage

| PCL | Description | Test File(s) | Test(s) |
|-----|-------------|-------------|---------|
| PCL.028 | Transport adapter interface | test_pcl_robustness | PublishWithTransportCallsTransportPublish |
| PCL.029 | Transport wiring | test_pcl_robustness | SetTransportAndClear, SetTransportNullExecutorReturnsInvalid |
| PCL.030 | Dispatch incoming from transport | test_pcl_executor | IntraProcessPubSub |

## TCP Socket Transport Coverage

| PCL | Description | Test File(s) | Test(s) |
|-----|-------------|-------------|---------|
| PCL.031 | Server mode | — | Covered by integration tests (not in unit suite) |
| PCL.032 | Client mode | — | Covered by integration tests (not in unit suite) |
| PCL.033 | Wire protocol | — | Covered by integration tests (not in unit suite) |
| PCL.034 | Gateway container | — | Covered by integration tests (not in unit suite) |
| PCL.035 | Non-blocking send | — | Covered by integration tests (not in unit suite) |
| PCL.036 | Async remote service invocation | — | Covered by integration tests (not in unit suite) |

## Logging Coverage

| PCL | Description | Test File(s) | Test(s) |
|-----|-------------|-------------|---------|
| PCL.037 | Printf-style logging | test_pcl_log, test_pcl_robustness | CustomHandlerReceivesMessages, ContainerNamePassedThrough, FormatString, LongFormatStringTruncated, DefaultHandlerNoContainerName, DefaultHandlerWithContainerName |
| PCL.038 | Pluggable log handler | test_pcl_log, test_pcl_robustness | CustomHandlerReceivesMessages, RevertToDefaultHandler, HandlerUserDataPassedThrough |
| PCL.039 | Log level filtering | test_pcl_log, test_pcl_robustness | LevelFiltering, FilterBlocksAllBelowFatal |
| PCL.040 | Log levels (5 levels) | test_pcl_robustness | AllLevelsReachHandler |

## Bridge Coverage

| PCL | Description | Test File(s) | Test(s) |
|-----|-------------|-------------|---------|
| PCL.041 | Bridge creation | test_pcl_dining | CreateAndDestroy, BridgeInactiveDoesNotForward, ConfigureFailsWhenPortsFull, DispatchNotFoundLogsDebug |
| PCL.042 | Bridge transform dispatch | test_pcl_dining | SpeedUnitConversion, FivePhilosophersWithBridges |
| PCL.043 | Bridge message suppression | test_pcl_dining | FailTransformSuppressesForward |
| PCL.044 | Bridge null safety | test_pcl_dining | CreateNullArgsReturnNull, ContainerNullBridgeReturnsNull, DestroyNullIsNoOp |

## Robustness and Error Handling Coverage

| PCL | Description | Test File(s) | Test(s) |
|-----|-------------|-------------|---------|
| PCL.045 | Null handle safety | test_pcl_lifecycle, test_pcl_executor, test_pcl_robustness | NullHandlesReturnError, NullSafety, ParamNullArgs, ParamGetNullArgs, PortCreationNullArgs, PublishNullPortOrMsg, PostIncomingBadInputs, DispatchIncomingNullArgs, AddNullContainerReturnsInvalid, PostResponseCbNullSafety, SetTransportNullExecutorReturnsInvalid, RequestShutdownNullSafe, LongNameTruncated |
| PCL.046 | Allocation failure handling | test_pcl_oom, test_pcl_robustness | PostIncomingPendingAllocFails, PostIncomingTopicStrdupFails, PostIncomingTypeNameStrdupFails, PostIncomingDataMallocFails, BridgeCreateContainerAllocFails, PostResponseCbDataMallocFails, ParamOverflowReturnsNomem, PortOverflowReturnNull, ContainerOverflowReturnsNomem, DestroyWithPendingMessagesFrees, DestroyWithQueuedRespCbWithData |
| PCL.047 | Status codes | All test files | (Verified implicitly by all tests checking pcl_status_t return values) |

## C++ Wrapper Coverage

| PCL | Description | Test File(s) | Test(s) |
|-----|-------------|-------------|---------|
| PCL.048 | Component base class | — | Verified via pcl::Component usage in tactical_objects and integration tests |
| PCL.049 | Executor wrapper | — | Verified via pcl::Executor usage in tactical_objects and integration tests |

## Service Bindings Coverage

| PCL | Description | Test File(s) | Test(s) |
|-----|-------------|-------------|---------|
| PCL.050 | Wire-name constants | test_pcl_proto_bindings | WireNames (Provided), WireNames (Consumed) |
| PCL.051 | Topic constants | test_pcl_proto_bindings | TopicNames (Provided), TopicNames (Consumed) |
| PCL.052 | Service handler base class | test_pcl_proto_bindings | HandlerStubsReturnEmpty (Provided), HandlerStubsReturnEmpty (Consumed), AckConstants, QueryDefault, IdentifierIsString, ObjectDetailDefaults |
| PCL.053 | JSON builder functions | test_pcl_proto_bindings | BuildRequirementJsonKeys, BuildRequirementJsonNoDimension, BuildEvidenceJsonKeys, BuildEvidenceJsonDefaultObservedAt, MsgToString (Provided), MsgToString (Consumed), MsgToStringEmpty |
| PCL.054 | Subscribe wrappers | test_pcl_proto_bindings | SubscribeRegistrationSucceeds (Provided), SubscribeRegistrationSucceeds (Consumed) |
| PCL.055 | Dispatch function | test_pcl_proto_bindings | DispatchAllChannelsNoCrash (Provided), DispatchAllChannelsNoCrash (Consumed) |

---

## Coverage Summary

| Category | HLRs | LLRs | Test Files |
|----------|-------|------|------------|
| Container Lifecycle | 5 (001–005) | 14 (001–012, 111, 114) | test_pcl_lifecycle, test_pcl_robustness |
| Port Management | 6 (006–011) | 9 (019–027) | test_pcl_lifecycle, test_pcl_robustness |
| Parameters | 3 (012–014) | 6 (013–018) | test_pcl_lifecycle, test_pcl_robustness |
| Tick Rate | 3 (015–017) | 3 (028–030) | test_pcl_lifecycle, test_pcl_robustness |
| Executor | 7 (018–024) | 13 (031–043) | test_pcl_executor, test_pcl_robustness |
| Cross-Thread Ingress | 3 (025–027) | 13 (049–061) | test_pcl_executor, test_pcl_robustness |
| Transport Adapter | 3 (028–030) | 2 (062–063) | test_pcl_robustness |
| TCP Socket Transport | 6 (031–036) | 0 | (Integration tests) |
| Logging | 4 (037–040) | 11 (064–074) | test_pcl_log, test_pcl_robustness |
| Bridge | 4 (041–044) | 9 (075–083) | test_pcl_dining |
| Integration | — | 1 (084) | test_pcl_dining |
| Robustness / OOM | 3 (045–047) | 6 (085–090) | test_pcl_oom, test_pcl_robustness |
| Throughput | — | 3 (091–093) | test_pcl_robustness |
| C++ Wrappers | 2 (048–049) | 0 | (Component-level tests) |
| Service Bindings | 6 (050–055) | 17 (094–110) | test_pcl_proto_bindings |
| Null Safety | — | 4 (111–114) | test_pcl_lifecycle, test_pcl_executor, test_pcl_robustness |
| **Total** | **55** | **114** | **7 test files** |

## Test File Registry

| File | Purpose | Test Count |
|------|---------|------------|
| `test_pcl_lifecycle.cpp` | Container lifecycle, parameters, ports, tick rate, null safety | 13 |
| `test_pcl_executor.cpp` | Executor spin, dispatch, multi-container, shutdown, cross-thread ingress | 10 |
| `test_pcl_log.cpp` | Logging handler, level filtering, format strings | 5 |
| `test_pcl_dining.cpp` | Bridge unit tests + dining philosophers integration | 11 |
| `test_pcl_oom.cpp` | Out-of-memory injection for allocation failure branches | 6 |
| `test_pcl_robustness.cpp` | 100% code coverage: overflow, threading, throughput, edge cases | 55 |
| `test_pcl_proto_bindings.cpp` | C++ service bindings: wire names, handlers, dispatch, JSON builders | 19 |

## Trace Tag Format

Tests use requirement tags in comments:

```
///< REQ_PCL_NNN: Brief description.
///< PCL.0XX: HLR description.
```

Run tests with filter to verify a specific requirement area:

```bat
build\tests\Release\test_pcl_robustness.exe --gtest_filter=PclContainerRobust*
build\tests\Release\test_pcl_lifecycle.exe --gtest_filter=PclLifecycle*
build\tests\Release\test_pcl_proto_bindings.exe --gtest_filter=ProtoBindings*
```

## Requirements Not Yet Covered by Unit Tests

The following HLRs are implemented in `pcl_transport_socket.c` but are currently verified only by integration tests (not in the unit test suite):

| PCL | Description | Status |
|-----|-------------|--------|
| PCL.031 | Server mode | Implemented, needs unit test |
| PCL.032 | Client mode | Implemented, needs unit test |
| PCL.033 | Wire protocol | Implemented, needs unit test |
| PCL.034 | Gateway container | Implemented, needs unit test |
| PCL.035 | Non-blocking send | Implemented, needs unit test |
| PCL.036 | Async remote service invocation | Implemented, needs unit test |

The C++ wrappers (PCL.048, PCL.049) are exercised indirectly through component-level tests (e.g., `tactical_objects/tests/`) but do not have dedicated PCL-level unit tests.
