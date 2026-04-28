# PYRAMID Binding Performance Report

Date: 2026-04-28

> Note: the benchmark harness has since been corrected so the local, shared-memory,
> socket, and gRPC transport cases all use the same unary
> `object_of_interest.create_requirement` request/response shape
> (`ObjectInterestRequirement -> Identifier`). The numeric results below were
> gathered before that correction and should be rerun before making cross-transport
> comparisons.

## Purpose

This report captures the current performance baseline for PYRAMID generated
bindings across codec and transport combinations, plus the backend fixes needed
to make those measurements meaningful.

Use this page to answer:

- what the current wire-format encode/decode costs are
- what the current end-to-end transport latencies are
- why shared memory was previously reporting implausibly bad latency
- what changed in the executor and shared-memory idle paths

## Scope

The measurements in this report were taken from:

- [`subprojects/PYRAMID/tests/test_binding_performance.cpp`](../../../subprojects/PYRAMID/tests/test_binding_performance.cpp)
- [`subprojects/PCL/src/pcl_executor.c`](../../../subprojects/PCL/src/pcl_executor.c)
- [`subprojects/PCL/src/pcl_transport_shared_memory.c`](../../../subprojects/PCL/src/pcl_transport_shared_memory.c)

The benchmark was rerun after:

1. adding a codec-only microbenchmark for wire-format cost
2. fixing `pcl_executor_spin_once(..., timeout_ms)` so it no longer ignores
   `timeout_ms`
3. replacing accidental Windows `Sleep(1)` behavior in short-idle paths with a
   yield-style wait
4. reducing shared-memory poll cadence from `5 ms` to `1 ms`

## Measurement Notes

### Payloads

- Local, shared-memory, and socket transport rows use a non-trivial
  `ObjectDetail` payload.
- gRPC uses a unary protobuf request/response round-trip for
  `ObjectInterestRequirement -> Identifier`.

### Metric meaning

- `wall`: end-to-end elapsed time
- `thread cpu`: CPU consumed by the benchmark thread only
- `cycles`: thread cycle count, used only in the codec microbenchmark

### Important caveat

The codec microbenchmark is the best measure of wire-format cost.

The transport-side `thread cpu` column is still useful, but it does not include
helper-thread work in the transport runtime, and on Windows many small samples
quantize to `0.0 us`.

## Results

### Codec Cost

These numbers isolate encode/decode work from transport and scheduling.

| Codec op | Bytes | Wall ns/op | CPU ns/op | Cycles/op |
|----------|------:|-----------:|----------:|----------:|
| JSON encode | 263 | 7356.5 | 6250.0 | 24636.6 |
| JSON decode | 263 | 9589.3 | 9375.0 | 36215.0 |
| FlatBuffers encode | 168 | 367.5 | 781.2 | 1387.9 |
| FlatBuffers decode | 168 | 264.1 | 0.0 | 999.4 |
| Protobuf encode | 94 | 698.2 | 781.2 | 2638.3 |
| Protobuf decode | 94 | 725.0 | 781.2 | 2748.4 |

### End-to-End Transport

These numbers include codec work, transport runtime behavior, queue handoff,
and scheduling.

| Combo | Bytes | Avg wall us | Min wall us | P99 wall us | Avg thread cpu us |
|-------|------:|------------:|------------:|------------:|------------------:|
| local / json | 263 | 8.3 | 6.2 | 15.0 | 0.0 |
| local / flatbuffers | 168 | 0.9 | 0.9 | 1.0 | 0.0 |
| local / protobuf | 94 | 1.3 | 1.0 | 2.0 | 0.0 |
| shmem / json | 263 | 68.8 | 39.1 | 127.0 | 31.2 |
| shmem / flatbuffers | 168 | 48.2 | 26.5 | 89.0 | 0.0 |
| shmem / protobuf | 94 | 52.5 | 30.3 | 112.8 | 0.0 |
| socket / json | 263 | 51.4 | 33.8 | 142.4 | 31.2 |
| socket / flatbuffers | 168 | 39.2 | 26.0 | 94.9 | 31.2 |
| socket / protobuf | 94 | 36.1 | 23.6 | 106.7 | 31.2 |
| grpc / tcp | 73 | 319.6 | 196.7 | 466.1 | 234.4 |

## Interpretation

### Codec ranking

For pure wire-format cost:

1. FlatBuffers is fastest.
2. Protobuf is somewhat heavier than FlatBuffers but still very light.
3. JSON is much more expensive than both binary codecs.

That holds for both encode and decode, with JSON decode the most expensive path
in this benchmark.

### Transport ranking

For end-to-end latency:

1. local in-process dispatch is predictably best
2. socket and shared-memory are now in the same rough class
3. gRPC is materially slower because it measures a full unary RPC stack

### Shared-memory conclusion

The earlier shared-memory results in the tens of milliseconds were not showing
serialization cost and were not caused by recreating the shared-memory region
per message.

The main problem was short idle waits being implemented as real `Sleep(1)`-like
behavior on Windows, which often means "sleep until the next scheduler tick"
rather than "wait about one millisecond".

After fixing that, shared-memory moved from implausible multi-millisecond
latency into a believable tens-of-microseconds range.

## Root Cause and Fixes

### `timeout_ms` bug

Before this work, `pcl_executor_spin_once(..., timeout_ms)` ignored
`timeout_ms` entirely.

That meant:

- call sites that thought they were doing bounded idle waits were not
- benchmark loops could accidentally become busy-spin loops or rely on ad hoc
  sleeps outside the executor

The fix now:

- drains incoming, service, and response queues
- only idles when no work was drained
- uses a yield-style wait for the `<= 1 ms` case on Windows

Relevant code:

- [`subprojects/PCL/src/pcl_executor.c`](../../../subprojects/PCL/src/pcl_executor.c)

### Shared-memory idle path

The shared-memory transport also had a short-poll problem.

The receive thread and service-discovery loop used `PCL_SHM_POLL_MS`, but on
Windows the helper implementing that poll delay would call `Sleep(ms)`
directly. With `ms == 1`, that could still become a scheduler-quantum sleep.

The fix now:

- sets `PCL_SHM_POLL_MS` to `1u`
- uses a yield-style wait for the `<= 1 ms` case on Windows

Relevant code:

- [`subprojects/PCL/src/pcl_transport_shared_memory.c`](../../../subprojects/PCL/src/pcl_transport_shared_memory.c)

## Verification

The benchmark target was rebuilt and rerun one case at a time after the fixes:

```bat
cmake --build --preset all-on-release --target test_binding_performance --parallel 4
build-all-enabled\subprojects\PYRAMID\tests\Release\test_binding_performance.exe --gtest_filter=BindingPerformanceTest.Codec_Json
build-all-enabled\subprojects\PYRAMID\tests\Release\test_binding_performance.exe --gtest_filter=BindingPerformanceTest.Codec_FlatBuffers
build-all-enabled\subprojects\PYRAMID\tests\Release\test_binding_performance.exe --gtest_filter=BindingPerformanceTest.Codec_Protobuf
build-all-enabled\subprojects\PYRAMID\tests\Release\test_binding_performance.exe --gtest_filter=BindingPerformanceTest.Local_Json
build-all-enabled\subprojects\PYRAMID\tests\Release\test_binding_performance.exe --gtest_filter=BindingPerformanceTest.Local_FlatBuffers
build-all-enabled\subprojects\PYRAMID\tests\Release\test_binding_performance.exe --gtest_filter=BindingPerformanceTest.Local_Protobuf
build-all-enabled\subprojects\PYRAMID\tests\Release\test_binding_performance.exe --gtest_filter=BindingPerformanceTest.Shmem_Json
build-all-enabled\subprojects\PYRAMID\tests\Release\test_binding_performance.exe --gtest_filter=BindingPerformanceTest.Shmem_FlatBuffers
build-all-enabled\subprojects\PYRAMID\tests\Release\test_binding_performance.exe --gtest_filter=BindingPerformanceTest.Shmem_Protobuf
build-all-enabled\subprojects\PYRAMID\tests\Release\test_binding_performance.exe --gtest_filter=BindingPerformanceTest.Socket_Json
build-all-enabled\subprojects\PYRAMID\tests\Release\test_binding_performance.exe --gtest_filter=BindingPerformanceTest.Socket_FlatBuffers
build-all-enabled\subprojects\PYRAMID\tests\Release\test_binding_performance.exe --gtest_filter=BindingPerformanceTest.Socket_Protobuf
build-all-enabled\subprojects\PYRAMID\tests\Release\test_binding_performance.exe --gtest_filter=BindingPerformanceTest.Grpc_Tcp
```

## Recommended Next Steps

1. Keep the codec microbenchmark as the primary comparison for wire-format
   decisions.
2. Treat transport `thread cpu` as advisory rather than authoritative for total
   runtime cost.
3. If a stronger CPU metric is needed for transports, add explicit transport
   runtime instrumentation rather than relying on the caller thread alone.
