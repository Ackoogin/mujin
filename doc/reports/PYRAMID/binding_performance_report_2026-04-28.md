# PYRAMID Binding Performance Report

Date: 2026-04-28

Updated: 2026-04-29

## Purpose

This report captures the current performance baseline for PYRAMID generated
bindings across codec and transport combinations, plus the backend fixes needed
to make those measurements meaningful.

Use this page to answer:

- what the current wire-format encode/decode costs are
- what the current end-to-end transport latencies are
- what changed to make the transport comparison fair
- why shared memory was previously reporting implausibly bad latency
- what changed in the executor, shared-memory, and socket benchmark paths

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
5. changing local, shared-memory, and socket rows to use the same unary
   `object_of_interest.create_requirement` request/response shape as gRPC
6. adding the required remote gateway container for shared-memory and socket
   provided services in the benchmark harness
7. fixing the socket benchmark setup so the server/client connection path is
   deterministic on Windows
8. replacing the C++ protobuf service codec JSON shim with direct
   protobuf/domain-model mapping and native varint-delimited protobuf array
   framing

## Measurement Notes

### Payloads

- All transport rows now use the same unary RPC shape:
  `ObjectInterestRequirement -> Identifier`.
- The request payload includes a populated identifier/source, source enum,
  policy enum, two battle-dimension entries, and a geographic point.
- The response payload is the created `Identifier`.

### Metric meaning

- `wall`: end-to-end elapsed time
- `thread cpu`: CPU consumed by the benchmark thread only
- `cycles`: thread cycle count, used only in the codec microbenchmark

### Important caveat

The codec microbenchmark is the best measure of wire-format cost.

The transport-side `thread cpu` column is still useful, but it does not include
helper-thread work in the transport runtime, and on Windows many small samples
quantize to `0.0 us`.

## Improving Accuracy of Results

The earlier version of this report was not a fair cross-transport comparison.

The main issue was payload mismatch:

- local, shared-memory, and socket were measuring a pub/sub-style
  `ObjectDetail` payload
- gRPC was measuring a unary RPC
  `ObjectInterestRequirement -> Identifier` round-trip

That meant the benchmark was mixing two different things at once:

1. transport/runtime cost
2. materially different message shapes and serialization work

The harness now uses the same unary request/response path across local,
shared-memory, socket, and gRPC, so the transport rows are finally comparing
like with like.

There were also two transport-harness issues that affected correctness rather
than just absolute latency:

- the shared-memory and socket provider participants were missing the remote
  gateway container required for inbound unary service dispatch
- the socket benchmark had a server/client setup race on Windows around
  ephemeral-port publication versus the server actually entering `listen()`

Those are now fixed in the benchmark harness, so the current numbers should be
treated as the first trustworthy baseline for cross-transport comparison.

There was also a protobuf-specific accuracy problem in the generated C++ codec
layer.

The earlier protobuf rows were still paying a JSON transcode cost inside the
C++ service codec:

1. domain model to JSON
2. JSON shim to protobuf
3. protobuf parse back to JSON
4. JSON back to domain model

That path is now removed for the C++ tactical-object service codec. The JSON
shim remains appropriate for Ada interop, but the C++ benchmark path is now
measuring direct protobuf serialization and direct domain-model mapping.

## Results

### Codec Cost

These numbers isolate encode/decode work from transport and scheduling.

The JSON and FlatBuffers rows below come from the earlier full-suite rerun.
The Protobuf rows were refreshed after the direct C++ protobuf codec fix on
2026-04-29.

| Codec op | Bytes | Wall ns/op | CPU ns/op | Cycles/op |
|----------|------:|-----------:|----------:|----------:|
| JSON encode | 378 | 15822.4 | 15625.0 | 59971.2 |
| JSON decode | 378 | 20605.4 | 21093.8 | 78035.7 |
| FlatBuffers encode | 216 | 729.8 | 781.2 | 2760.3 |
| FlatBuffers decode | 216 | 566.6 | 0.0 | 2149.8 |
| Protobuf encode | 102 | 1344.0 | 1562.5 | 4912.2 |
| Protobuf decode | 102 | 1228.5 | 781.2 | 4583.5 |

### End-to-End Transport

These numbers include codec work, transport runtime behavior, queue handoff,
and scheduling.

The JSON and FlatBuffers rows below are still from the earlier full-suite rerun.
The Protobuf rows, plus the gRPC reference row, were refreshed after the direct
protobuf codec fix on 2026-04-29.

| Combo | Bytes | Avg wall us | Min wall us | P99 wall us | Avg thread cpu us |
|-------|------:|------------:|------------:|------------:|------------------:|
| local / json | 378 | 39.2 | 38.3 | 45.7 | 62.5 |
| local / flatbuffers | 216 | 2.3 | 2.2 | 3.4 | 0.0 |
| local / protobuf | 102 | 4.2 | 3.4 | 9.8 | 0.0 |
| shmem / json | 378 | 170.0 | 104.4 | 276.5 | 93.8 |
| shmem / flatbuffers | 216 | 101.7 | 60.4 | 231.7 | 93.8 |
| shmem / protobuf | 102 | 144.2 | 76.5 | 417.1 | 125.0 |
| socket / json | 378 | 124.0 | 103.8 | 222.1 | 125.0 |
| socket / flatbuffers | 216 | 80.1 | 53.1 | 201.4 | 62.5 |
| socket / protobuf | 102 | 174.9 | 49.7 | 1218.2 | 125.0 |
| grpc / tcp | 73 | 339.9 | 211.6 | 507.7 | 156.2 |

## Interpretation

### Codec ranking

For pure wire-format cost:

1. FlatBuffers is fastest.
2. Protobuf is still heavier than FlatBuffers, but only by a small constant
   factor in the direct C++ path.
3. JSON is much more expensive than both binary codecs.

That holds for both encode and decode, with JSON decode the most expensive path
in this benchmark.

The previous protobuf numbers were dominated by an implementation artifact, not
protobuf itself. After removing the C++ JSON shim, protobuf moved from roughly
`28 us/op` down to roughly `1.2-1.3 us/op` for this payload while also staying
the smallest on the wire.

### Transport ranking

For end-to-end latency:

1. local in-process dispatch is predictably best
2. local protobuf is now in the same low-single-digit-microsecond range as the
   other binary codecs
3. shared-memory and socket are still in the same broad order of magnitude
4. gRPC is materially slower because it measures a full unary RPC stack

The remote protobuf rows still show scheduler and transport noise, but they are
now much closer to what the wire-format microbenchmark suggests.

### Shared-memory conclusion

The earlier shared-memory results in the tens of milliseconds were not showing
serialization cost and were not caused by recreating the shared-memory region
per message.

The main problem was short idle waits being implemented as real `Sleep(1)`-like
behavior on Windows, which often means "sleep until the next scheduler tick"
rather than "wait about one millisecond".

After fixing that, shared-memory moved from implausible multi-millisecond
latency into a believable sub-millisecond range for this unary benchmark.

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

### Benchmark RPC fairness fixes

The benchmark harness itself also needed correction.

For shared-memory and socket:

- the server side now configures and adds the required remote gateway container
- the benchmark invokes the same unary generated binding used by the gRPC case

For socket specifically:

- the benchmark now picks a loopback port up front
- starts the server on that fixed port
- connects the client with retry-enabled `create_client_ex(...)`
- sets explicit peer IDs and a consumed-endpoint route for the unary service

Relevant code:

- [`subprojects/PYRAMID/tests/test_binding_performance.cpp`](../../../subprojects/PYRAMID/tests/test_binding_performance.cpp)

### Protobuf codec fix

The generated C++ protobuf service codec previously depended on JSON shims for
service-layer domain-model encode/decode. That was appropriate for Ada-facing
interop helpers, but it materially distorted the C++ benchmark path.

The fix now:

- maps the singular C++ service types directly between domain-model structs and
  protobuf messages
- serializes protobuf arrays using native varint-delimited message framing
- removes the JSON shim from the C++ tactical-object service codec hot path

Relevant code:

- [`subprojects/PYRAMID/bindings/protobuf/cpp/pyramid_services_tactical_objects_protobuf_codec.cpp`](../../../subprojects/PYRAMID/bindings/protobuf/cpp/pyramid_services_tactical_objects_protobuf_codec.cpp)

## Verification

The benchmark target was rebuilt and rerun one case at a time after the
transport-harness fixes:

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

The full suite was then rerun successfully with:

```bat
build-all-enabled\subprojects\PYRAMID\tests\Release\test_binding_performance.exe --gtest_filter=BindingPerformanceTest.*
```

After the direct protobuf codec fix, the protobuf support library and affected
tests were rebuilt and rerun with:

```bat
cmake --build --preset all-on-release --target pyramid_protobuf_support test_pcl_proto_bindings test_binding_performance --parallel 4
build-all-enabled\subprojects\PYRAMID\tests\Release\test_pcl_proto_bindings.exe
build-all-enabled\subprojects\PYRAMID\tests\Release\test_binding_performance.exe --gtest_filter=BindingPerformanceTest.Codec_Protobuf:BindingPerformanceTest.Local_Protobuf:BindingPerformanceTest.Shmem_Protobuf:BindingPerformanceTest.Socket_Protobuf:BindingPerformanceTest.Grpc_Tcp
```

That targeted rerun passed, and the report now reflects those refreshed
protobuf rows.

## Recommended Next Steps

1. Keep the codec microbenchmark as the primary comparison for wire-format
   decisions.
2. Treat transport `thread cpu` as advisory rather than authoritative for total
   runtime cost.
3. If a stronger CPU metric is needed for transports, add explicit transport
   runtime instrumentation rather than relying on the caller thread alone.
4. If this report will be used for architecture decisions, rerun each transport
   row multiple times and publish median plus variance, not just a single pass.
