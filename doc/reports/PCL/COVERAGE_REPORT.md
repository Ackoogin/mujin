# PCL Statement Coverage Report

Generated: 2026-07-06

Toolchain: `gcovr 8.6` with GCC 13.3.0 (`--coverage -O0 -g -fprofile-abs-path`), Linux x86_64.

This snapshot covers **all 17 current PCL production sources**, including the
plugin/routing-era files (`pcl_alloc.c`, `pcl_capabilities.c`,
`pcl_codec_registry.c`, `pcl_plugin_loader.c`, `pcl_transport_routing.c`, and
the three reference transport plugin shims) that were missing from the
2026-05-21 report. The coverage harness
(`cmake/pcl_coverage/CMakeLists.txt`) was extended in this pass to build the
full `pcl_core` source list, the reference transport plugins, the loader/
routing/capabilities test plugins, and all 21 PCL test binaries.

## Result Summary

| Metric | Value |
|--------|-------|
| Statement (line) coverage | **100%** (4300 / 4300 lines) |
| Branch coverage (informational, DAL B planning input) | 73% (2429 / 3285 branches) |
| Test binaries run | 21 |
| Tests passed / failed | **440 / 0** |

Statement coverage with no unexplained gaps is the DO-178C Table A-7
structural-coverage objective for DAL C. Branch coverage is reported as an
approximation of the decision-coverage objective that applies at DAL B; the
73% figure quantifies the DAL B delta (see `DO178C_GAP_ANALYSIS.md`, GAP-B-01).

## Per-File Line Coverage

| Source File | Lines | Executed | Coverage |
|-------------|-------|----------|----------|
| `pcl_alloc.c` | 19 | 19 | **100%** |
| `pcl_bridge.c` | 47 | 47 | **100%** |
| `pcl_capabilities.c` | 30 | 30 | **100%** |
| `pcl_codec_registry.c` | 57 | 57 | **100%** |
| `pcl_container.c` | 325 | 325 | **100%** |
| `pcl_executor.c` | 765 | 765 | **100%** |
| `pcl_log.c` | 28 | 28 | **100%** |
| `pcl_plugin_loader.c` | 151 | 151 | **100%** |
| `pcl_transport_apos.c` | 195 | 195 | **100%** |
| `pcl_transport_routing.c` | 189 | 189 | **100%** |
| `pcl_transport_shared_memory.c` | 1148 | 1148 | **100%** |
| `pcl_transport_shared_memory_plugin.c` | 72 | 72 | **100%** |
| `pcl_transport_socket.c` | 569 | 569 | **100%** |
| `pcl_transport_socket_plugin.c` | 80 | 80 | **100%** |
| `pcl_transport_template.c` | 323 | 323 | **100%** |
| `pcl_transport_udp.c` | 226 | 226 | **100%** |
| `pcl_transport_udp_plugin.c` | 76 | 76 | **100%** |
| **Total** | **4300** | **4300** | **100%** |

The residual statement misses documented in the 2026-05-21 report
(socket/pthread fault paths, SHM malformed-frame parsing, template-transport
OOM paths, APOS stub error paths) have since been closed by the fault-injection
and threading-conformance work (see commit `37cbfb9`, "restore 100% statement
coverage"); this run confirms them closed on the current baseline.

## Per-File Branch Coverage (Informational)

| Source File | Branches | Taken | Coverage |
|-------------|----------|-------|----------|
| `pcl_alloc.c` | 16 | 16 | 100% |
| `pcl_bridge.c` | 28 | 27 | 96% |
| `pcl_capabilities.c` | 21 | 21 | 100% |
| `pcl_codec_registry.c` | 50 | 39 | 78% |
| `pcl_container.c` | 296 | 250 | 84% |
| `pcl_executor.c` | 621 | 522 | 84% |
| `pcl_log.c` | 16 | 15 | 93% |
| `pcl_plugin_loader.c` | 138 | 115 | 83% |
| `pcl_transport_apos.c` | 132 | 85 | 64% |
| `pcl_transport_routing.c` | 158 | 128 | 81% |
| `pcl_transport_shared_memory.c` | 887 | 552 | 62% |
| `pcl_transport_shared_memory_plugin.c` | 70 | 44 | 62% |
| `pcl_transport_socket.c` | 354 | 263 | 74% |
| `pcl_transport_socket_plugin.c` | 80 | 57 | 71% |
| `pcl_transport_template.c` | 196 | 143 | 73% |
| `pcl_transport_udp.c` | 144 | 101 | 70% |
| `pcl_transport_udp_plugin.c` | 78 | 51 | 65% |
| **Total** | **3285** | **2429** | **73%** |

GCC branch counters include compiler-synthesised branches (e.g. short-circuit
sub-expressions); a qualified decision-coverage tool will report different
figures. Treat this table as a prioritisation aid for DAL B planning, not as
decision-coverage evidence.

## Coverage Harness Composition

The harness in `cmake/pcl_coverage` builds the complete PCL core (all nine
`pcl_core` sources), all production transports (socket, UDP, shared-memory,
template, APOS), the three reference transport plugins, the six loader/
capabilities/routing test plugins, and runs every PCL test binary against
them.

| Test Binary | Tests | Purpose |
|-------------|-------|---------|
| `test_pcl_alloc` | 4 | Portable allocator: overflow rejection, realloc semantics |
| `test_pcl_lifecycle` | 14 | Container lifecycle, parameters, ports, tick rate |
| `test_pcl_executor` | 32 | Executor spin, dispatch, routing, shutdown, async invoke |
| `test_pcl_log` | 5 | Logging handler, level filtering, formatting |
| `test_pcl_robustness` | 90 | Edge cases, threading, throughput, route filtering, deferred responses |
| `test_pcl_streaming` | 12 | Streaming service/client paths |
| `test_pcl_bridge` | 9 | Bridge unit tests and port overflow |
| `test_pcl_dining` | 10 | Dining philosophers bridge/executor integration |
| `test_pcl_socket_transport` | 33 | TCP socket round trips, gateway, reconnect, keepalive |
| `test_pcl_socket_faults` | 7 | GCC-only socket fault injection, malformed wire frames |
| `test_pcl_udp_transport` | 12 | UDP datagram pub/sub, peer filtering, RPC absence |
| `test_pcl_shared_memory_transport` | 33 | SHM bus pub/sub/service/streaming, inter-process IPC |
| `test_pcl_template_transport` | 24 | Engineer-extensible scaffold + delivery conformance suite |
| `test_pcl_apos_transport` | 11 | APOS Local Virtual Channel transport |
| `test_pcl_transport_threading` | 11 | Threading-model conformance suite (PCL.075/076) |
| `test_pcl_codec_registry` | 10 | Codec vtable registration, lookup, iteration |
| `test_pcl_plugin_loader` | 28 | Plugin ABI gating, fail-closed loading, teardown-then-unload |
| `test_pcl_transport_routing` | 17 | Manifest loading, atomic rollback, fail-closed diagnostics |
| `test_pcl_capabilities` | 34 | Capability masks, QoS floors, compose-time validation |
| `test_pcl_cpp_wrappers` | 32 | C++ `Component` and `Executor` wrappers |
| `test_pcl_oom` | 12 | GCC-only allocation-failure injection |
| **Total** | **440** | |

## Known Measurement Artifacts

- gcovr reports `Ignoring negative hits` on `pcl_executor.c:1365` and
  `pcl_transport_shared_memory.c:762` (branch counters only). This is the
  known GCC profile-runtime counter underflow when one object file is
  exercised by many test binaries; the lines themselves are covered.
  `--gcov-ignore-parse-errors negative_hits.warn` is set so the report is not
  poisoned.
- `test_pcl_oom` links a separate `pcl_core_oom` copy of the core sources so
  its `--wrap=malloc` counters do not corrupt the shared `pcl_core` counters
  (GCC bug #68080). Both copies contribute coverage.

## Build and Run Instructions

```bash
# Configure coverage harness (out of tree)
cmake -S cmake/pcl_coverage -B build-coverage-pcl \
      -G "Unix Makefiles" \
      -DCMAKE_C_COMPILER=gcc \
      -DCMAKE_CXX_COMPILER=g++

# Build
cmake --build build-coverage-pcl -j"$(nproc)"

# Clear stale counters before a clean run
find build-coverage-pcl -name '*.gcda' -delete

# Run every coverage binary
for exe in test_pcl_alloc test_pcl_lifecycle test_pcl_executor test_pcl_log \
           test_pcl_robustness test_pcl_streaming test_pcl_bridge \
           test_pcl_dining test_pcl_socket_transport test_pcl_socket_faults \
           test_pcl_udp_transport test_pcl_shared_memory_transport \
           test_pcl_template_transport test_pcl_apos_transport \
           test_pcl_transport_threading test_pcl_codec_registry \
           test_pcl_plugin_loader test_pcl_transport_routing \
           test_pcl_capabilities test_pcl_cpp_wrappers test_pcl_oom; do
  ./build-coverage-pcl/${exe} || exit 1
done

# Line coverage (DAL C objective)
gcovr --root . --filter "subprojects/PCL/src/" \
      --gcov-executable gcov \
      --gcov-ignore-errors source_not_found \
      --gcov-ignore-errors no_working_dir_found \
      --gcov-ignore-parse-errors negative_hits.warn \
      --txt=coverage_pcl_current/summary.txt \
      --html-details=coverage_pcl_current/index.html \
      build-coverage-pcl

# Branch coverage view (informational)
gcovr --root . --filter "subprojects/PCL/src/" \
      --gcov-executable gcov \
      --gcov-ignore-errors source_not_found \
      --gcov-ignore-errors no_working_dir_found \
      --gcov-ignore-parse-errors negative_hits.warn \
      --branches --txt=coverage_pcl_current/branches.txt \
      build-coverage-pcl
```

When configuring the harness from outside the repository root, pass absolute
paths to `--filter` (gcovr resolves the filter against `--root`).

## Mutex Audit Cross-Reference

See `MUTEX_AUDIT.md` (same directory) for the companion lock-hierarchy and
deadlock-risk review across the executor and all transports. It is the seed
input for the data/control-coupling analysis required by DO-178C Table A-7
objective 8 (see `DO178C_GAP_ANALYSIS.md`, GAP-C-14).
