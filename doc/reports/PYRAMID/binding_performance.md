# PYRAMID binding performance

Last measured: 2026-07-15

## Summary

This report is the maintained performance baseline for PYRAMID generated
bindings. It replaces the removed point-in-time report from April 2026.

The benchmark now records the start immediately before an invocation or
publish, and records completion in the final application callback. The test
thread waits on a condition variable only after the completion timestamp has
been recorded. No timed result includes a test loop that polls an executor or
sleeps between polls.

The main conclusions on this host are:

- FlatBuffers has the lowest isolated codec cost. Protobuf encode takes about
  1.6 times as long and Protobuf decode takes about 2.0 times as long, while
  making the smallest request payload. Both binary codecs remain much faster
  than JSON for this payload.
- A raw generated-protobuf gRPC service averages `44.1 us` round trip. The
  same generated gRPC service backed by a PYRAMID port averages `50.4 us`.
  The full binding path therefore adds `6.3 us`, or about 14 percent.
- Across five Fast DDS repetitions, the mean of the plain typed ROS2 run
  averages is `7.5 us` one way. The mean for delivery through the generated
  PYRAMID ROS2 ingress and a PCL subscriber port is `10.4 us`. The added
  binding path costs `2.9 us`, or about 39 percent, on this host.
- UDP averages `4.9-7.0 us` for raw, pre-encoded payload delivery and
  `6.9-14.0 us` for the full typed PYRAMID path. The difference depends mainly
  on the codec.
- The current Linux shared-memory implementation is dominated by an internal
  `nanosleep(1 ms)` after each mailbox scan. That is an implementation
  artifact, not an intrinsic shared-memory cost and not benchmark executor
  polling. It remains in the full-system result because deployed code pays it.

## What is measured

There are three kinds of result. They must not be mixed into one ranking.

1. **Unary round trip** measures request start to the response callback. Local,
   shared memory, TCP socket, and gRPC use this metric.
2. **One-way application delivery** measures publish start to the subscriber
   callback. UDP, LA-CAL, and ROS2 over Fast DDS use this metric.
3. **Codec cost** measures encode or decode only, with no transport.

The local, shared-memory, and socket unary rows use the same typed operation:
`ObjectInterestRequirement -> Identifier`. The UDP rows use the same
`ObjectMatch[1]` message. Each raw/full pair carries the same encoded bytes.

The gRPC comparison is also matched: both rows use the same generated protobuf
service and messages. The raw row terminates in an ordinary generated gRPC
service implementation. The full row enters through generated gRPC code,
crosses the PYRAMID-to-PCL adapter, dispatches a PYRAMID port, and maps the
response back to the generated protobuf type.

The Fast DDS comparison uses the same generated ROS2 message and typed CDR wire
format in both rows. The raw baseline is plain `rclcpp` pub/sub with
`rmw_fastrtps_cpp`. It is not a direct use of the native Fast DDS API. The full
row adds the generated PYRAMID ROS2 ingress, typed-message deserialization,
executor handoff, and final PCL subscriber callback.

### Raw and full path definitions

| Label | Included work |
|---|---|
| `raw-pcl` | PCL port and selected transport using an already encoded payload; no generated typed facade or codec work |
| `raw-grpc` | Generated protobuf mapping and the ordinary synchronous gRPC client/server path; no PYRAMID or PCL port |
| `raw rclcpp / Fast DDS` | Generated typed ROS2 message through `rclcpp` and Fast DDS to a plain ROS2 callback |
| `pyramid` or `pyramid-port` | Generated typed binding, codec where applicable, transport adapter, PCL dispatch, and typed response or subscriber handling |

## Timing method

Each PCL transport executor runs independently. Invocation commands are posted
to the client executor thread. That thread records the start and invokes the
operation. The response or subscriber callback records completion with
`std::chrono::steady_clock`. Only then does it notify the benchmark thread.

This design avoids the old pattern in which the benchmark repeatedly called
`spin_once()` until it noticed completion. There is no fixed sleep or polling
interval between the two timestamps. Background executors may call
`spin_once(..., 0)` to dispatch queued work, but that is normal execution and
does not add an artificial timed wait.

The gRPC client is synchronous, so its interval is the ordinary call start to
call return. ROS2 uses an event-driven `SingleThreadedExecutor::spin()`. Its PCL
subscriber callback records the full-path completion timestamp.

All transport tests run 50 untimed warm-up operations. PCL and pub/sub rows
then collect 500 sequential samples; gRPC collects 200. Codec rows run 20,000
operations. The Fast DDS pair was repeated five times to show variation
between short ROS2/DDS runs. The benchmark reports average, minimum, median,
and 99th percentile latency. Calls are sequential, with one operation in
flight, so these results measure latency rather than throughput.

## Test environment

| Item | Value |
|---|---|
| Host | Linux 6.8, x86-64 |
| CPU | Intel Core i9-12950HX, 16 cores / 24 logical CPUs |
| Compiler | GCC 11.4.0 |
| Build | CMake `Release`; FlatBuffers, Protobuf, gRPC, and OWP enabled |
| gRPC | v1.72.0, repository CMake dependency |
| ROS2 | Humble |
| Fast DDS | 2.6.11 through `rmw_fastrtps_cpp` 6.2.10 |
| Network | Same-host loopback |

The host was not isolated, CPU frequency was not fixed, and threads were not
pinned. Treat small differences, especially noisy 99th percentiles, as
indicative rather than exact capacity limits.

## Results

### Codec cost

The first six rows use the unary `ObjectInterestRequirement`. The OMS JSON rows
use the UCI Starter `PositionReport` C ABI and include the runtime plugin call
and its output allocation/free. Because the message types differ, compare the
OMS row as a cost and size baseline, not as a strict codec contest.

| Codec operation | Bytes | Wall ns/op | CPU ns/op |
|---|---:|---:|---:|
| JSON encode | 378 | 6,827.2 | 6,825.8 |
| JSON decode | 378 | 10,383.8 | 10,382.2 |
| FlatBuffers encode | 216 | 330.6 | 330.6 |
| FlatBuffers decode | 216 | 281.4 | 281.4 |
| Protobuf encode | 102 | 524.2 | 524.2 |
| Protobuf decode | 102 | 549.3 | 548.9 |
| OMS JSON plugin encode | 549 | 6,255.4 | 6,253.5 |
| OMS JSON plugin decode | 549 | 6,469.0 | 6,468.3 |

FlatBuffers is fastest for this message. Protobuf encode is `193.6 ns` slower,
or about 1.6 times the FlatBuffers encode cost. Protobuf decode is `267.9 ns`
slower, or about 2.0 times the FlatBuffers decode cost. A complete encode and
decode pair is about `0.46 us` slower with Protobuf, while reducing the request
from 216 to 102 bytes. JSON encode plus decode costs about `17.2 us`, which
explains most of the full binding overhead in the local and socket JSON rows.

### Unary request/response latency

All rows below measure round-trip time in microseconds.

| Path | Bytes | Average | Minimum | P50 | P99 |
|---|---:|---:|---:|---:|---:|
| Full PYRAMID / local / JSON | 378 | 21.7 | 20.6 | 21.2 | 27.4 |
| Raw PCL / local / JSON | 378 | 0.1 | 0.0 | 0.1 | 0.1 |
| Full PYRAMID / local / FlatBuffers | 216 | 1.6 | 1.6 | 1.6 | 1.6 |
| Raw PCL / local / FlatBuffers | 216 | 0.0 | 0.0 | 0.0 | 0.1 |
| Full PYRAMID / local / Protobuf | 102 | 2.2 | 2.2 | 2.2 | 2.3 |
| Raw PCL / local / Protobuf | 102 | 0.0 | 0.0 | 0.0 | 0.1 |
| Full PYRAMID / shared memory / JSON | 378 | 1,146.4 | 934.6 | 1,051.8 | 2,116.8 |
| Raw PCL / shared memory / JSON | 378 | 1,084.1 | 951.7 | 1,051.7 | 2,111.0 |
| Full PYRAMID / shared memory / FlatBuffers | 216 | 1,073.3 | 951.7 | 1,051.4 | 2,110.0 |
| Raw PCL / shared memory / FlatBuffers | 216 | 1,084.9 | 249.7 | 1,047.2 | 2,103.8 |
| Full PYRAMID / shared memory / Protobuf | 102 | 1,107.2 | 951.5 | 1,051.2 | 2,109.0 |
| Raw PCL / shared memory / Protobuf | 102 | 1,062.2 | 927.4 | 1,053.2 | 2,108.5 |
| Full PYRAMID / TCP socket / JSON | 378 | 38.5 | 34.6 | 36.4 | 97.1 |
| Raw PCL / TCP socket / JSON | 378 | 14.5 | 13.4 | 13.9 | 21.0 |
| Full PYRAMID / TCP socket / FlatBuffers | 216 | 17.1 | 15.8 | 16.3 | 24.5 |
| Raw PCL / TCP socket / FlatBuffers | 216 | 16.8 | 14.9 | 15.5 | 90.3 |
| Full PYRAMID / TCP socket / Protobuf | 102 | 17.8 | 16.1 | 17.5 | 22.7 |
| Raw PCL / TCP socket / Protobuf | 102 | 16.3 | 13.3 | 15.5 | 65.3 |
| PYRAMID port / gRPC / Protobuf | 73 | 50.4 | 42.9 | 47.0 | 99.8 |
| Raw generated gRPC / Protobuf | 73 | 44.1 | 39.4 | 42.3 | 65.8 |

The binary-codec binding overhead is small beside TCP and gRPC. JSON remains
visible because its encode and decode work is much larger. Shared memory does
not show a stable codec difference because the transport poll cadence is about
two orders of magnitude larger than binary codec work. These shared-memory
rows are therefore an as-implemented latency regression marker, not a measure
of the underlying shared-memory primitive. The receive loop needs an
event-driven, cross-process mailbox notification before this transport can
provide a meaningful low-latency baseline without busy-spinning.

The raw local rows round below the report's `0.1 us` precision. They are useful
as a control showing that the local full rows mainly measure typed conversion
and codec work, not transport scheduling.

### One-way pub/sub latency

These results are one-way application delivery times in microseconds. They are
not directly comparable with the unary round trips above.

| Path | Payload | Bytes | Average | Minimum | P50 | P99 |
|---|---|---:|---:|---:|---:|---:|
| Full PYRAMID / UDP / JSON | `ObjectMatch[1]` | 99 | 14.0 | 12.6 | 13.2 | 26.8 |
| Raw PCL / UDP / JSON | `ObjectMatch[1]` | 99 | 7.0 | 5.6 | 7.1 | 7.8 |
| Full PYRAMID / UDP / FlatBuffers | `ObjectMatch[1]` | 132 | 7.1 | 6.2 | 6.6 | 12.6 |
| Raw PCL / UDP / FlatBuffers | `ObjectMatch[1]` | 132 | 6.0 | 4.8 | 5.1 | 71.1 |
| Full PYRAMID / UDP / Protobuf | `ObjectMatch[1]` | 67 | 6.9 | 5.8 | 6.1 | 35.3 |
| Raw PCL / UDP / Protobuf | `ObjectMatch[1]` | 67 | 4.9 | 4.6 | 4.9 | 6.0 |
| Plain rclcpp / Fast DDS / typed CDR | `ObjectDetail` | typed | 6.1-9.4 | 5.3-7.7 | 5.8-9.6 | 8.9-14.3 |
| PYRAMID port / Fast DDS / typed CDR | `ObjectDetail` | typed | 10.2-10.8 | 7.2-9.0 | 9.0-10.2 | 15.1-26.8 |
| Raw PCL / LA-CAL OWP / OMS JSON | `PositionReport`-shaped test body | 522 | 107.5 | 19.4 | 22.1 | 118.6 |

The Fast DDS cells show the range across five independent runs of 500 samples,
not a confidence interval. The raw run average varied more than the full path,
which is why a single sub-microsecond comparison would be misleading.

The LA-CAL result uses the real PYRAMID LA-CAL transport plugin and an
in-process OWP routing broker. It captures WebSocket framing, broker fan-out,
transport ingress, executor handoff, and the final PCL callback. The mean is
affected by a small number of scheduler outliers; the median of `22.1 us` is a
better description of the usual same-host result. This is a raw transport row:
the separate OMS JSON codec result above supplies the codec cost. The generated
LA-CAL UCI seam remains covered by functional tests, but it does not yet have a
same-message raw/full performance pair.

## Coverage and deliberate exclusions

| Transport or codec | Quantitative coverage | Notes |
|---|---|---|
| JSON, FlatBuffers, Protobuf | Codec-only and full local/socket/shared-memory/UDP paths | Matched raw/full payloads |
| gRPC/Protobuf | Raw generated gRPC and full PYRAMID-port round trip | Same proto, service, request, and response |
| ROS2/Fast DDS typed CDR | Plain rclcpp baseline and full PYRAMID ingress | Same generated message; forced `rmw_fastrtps_cpp` |
| UDP | Raw PCL and full typed PYRAMID one-way delivery | Best-effort pub/sub |
| LA-CAL OWP | Raw PCL transport plus separate OMS JSON plugin cost | Full generated UCI seam is functional-only today |
| Shared memory | Raw PCL and full typed PYRAMID unary round trip | Current 1 ms internal poll cadence dominates |
| TCP socket | Raw PCL and full typed PYRAMID unary round trip | Same-host loopback |
| APOS | Not reported | The repository target uses the development APOS stub; a number would measure the stub rather than the deployment platform |
| ROS2 envelope fallback | Not reported | Typed CDR is the default production wire and the matched Fast DDS comparison |

This table prevents an unavailable or synthetic platform from appearing as a
real transport comparison. APOS should be added when the benchmark can run on
the target APOS virtual channel implementation.

## Reproducing the measurements

Configure an optimized non-ROS2 benchmark build with all ordinary codecs and
transports:

```sh
cmake -S . -B build-performance \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYRAMID_ENABLE_FLATBUFFERS=ON \
  -DPYRAMID_ENABLE_PROTOBUF=ON \
  -DPYRAMID_ENABLE_GRPC=ON \
  -DPYRAMID_ENABLE_OWP=ON \
  -DPYRAMID_ENABLE_ROS2=OFF
cmake --build build-performance --target \
  test_binding_performance test_lacal_transport_plugin --parallel
build-performance/subprojects/PYRAMID/tests/test_binding_performance
build-performance/subprojects/PYRAMID/tests/test_lacal_transport_plugin \
  --gtest_filter=LacalPlugin.IdentityFromInfoAndBrokerRoundTrip
```

Build and run the Fast DDS comparison in the ROS2 environment:

```sh
subprojects/PYRAMID/scripts/build_ros2_transport.sh --jobs 4
source /opt/ros/humble/setup.bash
source build-ros2-ament/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
build-ros2-ament/build/pyramid_ros2_transport/test/test_rclcpp_runtime_adapter \
  --gtest_filter='RuntimeAdapterFixture.FastDds_*'
```

The executable output is the source of truth. This page records one named-host
baseline so regressions are visible, but it should be refreshed after material
changes to a codec, transport, executor, generated facade, or dependency.

## Limitations

- Results are same-process or same-host loopback; they do not predict network
  latency between deployed hosts.
- Sequential calls measure latency, not maximum throughput or behavior with
  many operations in flight.
- The test does not isolate CPU cores, lock CPU frequency, or apply real-time
  scheduling.
- A single run can contain scheduler outliers. Compare medians and repeat the
  suite before treating a small change as a regression.
- UDP is best effort. This run delivered all 500 measured messages, but the
  transport does not promise that result under congestion.
- Fast DDS is measured through the ROS2 RMW layer because that is PYRAMID's
  production integration. A native Fast DDS API benchmark would answer a
  different question.
