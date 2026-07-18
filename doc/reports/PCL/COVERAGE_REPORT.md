# PCL Statement Coverage Report

Generated: 2026-07-18

Toolchain: `gcovr 8.6` with GCC 13.3.0 (`--coverage -O0 -g -fprofile-abs-path`), Linux x86_64.

This snapshot covers **all 18 current PCL production sources** built by the
coverage harness (`cmake/pcl_coverage/CMakeLists.txt`), including
`pcl_process_runtime.c`, which was added to the library after the 2026-07-06
report. It supersedes that report. The measurement was taken on a clean build
of the completed requirements baseline (444 LLRs, function-level code-to-LLR
tags in every production source) with all counters deleted before a single
full-suite run — no isolated re-runs and no accumulated counters from earlier
runs contribute to the result.

## Result Summary

| Metric | Value |
|--------|-------|
| Statement (line) coverage | **100%** (5,072 / 5,072 lines) |
| Branch coverage (informational, DAL B planning input) | 76% (2,930 / 3,846 branches) |
| Test binaries run | 22 |
| Tests passed / failed | **499 / 0** |

Statement coverage with no unexplained gaps is the DO-178C Table A-7
structural-coverage objective for DAL C. Branch coverage is reported as an
approximation of the decision-coverage objective that applies at DAL B; the
76% figure quantifies the DAL B delta (see `DO178C_GAP_ANALYSIS.md`, GAP-B-01).

The five residual locations reported by the previous in-progress run are all
resolved on this baseline:

- `pcl_container.c` (formerly line 505): closed by test input added in the
  requirements checkpoint; the file measures 100% with no exclusion.
- `pcl_executor.c` (formerly line 243): the proposed exclusion was **rejected
  during review** — the line is reachable through the public API (installing a
  default transport while a peer-selected remote subscriber exists). It is now
  covered by the test
  `test_pcl_executor.cpp::LateDefaultTransportSkipsPeerSelectedSubscriber`.
- `pcl_transport_routing.c` (formerly line 280): carries a reviewed exclusion
  for the empty-token-list arm of the exclusive-group splitter, whose caller
  tokenises the line before invoking it.
- `pcl_transport_socket.c` (formerly lines 874-875 and 888, the client
  auto-reconnect backoff): covered in this single full-suite run by
  `PclSocketTransport.AutoReconnectAfterServerRestart` and
  `PclSocketFaults.AutoReconnectBackoffRunsWhileServerStaysDown`; no isolated
  execution or test-order dependence was needed.

## Per-File Line Coverage

| Source File | Lines | Executed | Coverage |
|-------------|-------|----------|----------|
| `pcl_alloc.c` | 19 | 19 | **100%** |
| `pcl_bridge.c` | 47 | 47 | **100%** |
| `pcl_capabilities.c` | 30 | 30 | **100%** |
| `pcl_codec_registry.c` | 83 | 83 | **100%** |
| `pcl_container.c` | 360 | 360 | **100%** |
| `pcl_executor.c` | 925 | 925 | **100%** |
| `pcl_log.c` | 28 | 28 | **100%** |
| `pcl_plugin_loader.c` | 169 | 169 | **100%** |
| `pcl_process_runtime.c` | 306 | 306 | **100%** |
| `pcl_transport_apos.c` | 195 | 195 | **100%** |
| `pcl_transport_routing.c` | 280 | 280 | **100%** |
| `pcl_transport_shared_memory.c` | 1230 | 1230 | **100%** |
| `pcl_transport_shared_memory_plugin.c` | 72 | 72 | **100%** |
| `pcl_transport_socket.c` | 585 | 585 | **100%** |
| `pcl_transport_socket_plugin.c` | 83 | 83 | **100%** |
| `pcl_transport_template.c` | 323 | 323 | **100%** |
| `pcl_transport_udp.c` | 261 | 261 | **100%** |
| `pcl_transport_udp_plugin.c` | 76 | 76 | **100%** |
| **Total** | **5,072** | **5,072** | **100%** |

## Per-File Branch Coverage (Informational)

| Source File | Branches | Taken | Coverage |
|-------------|----------|-------|----------|
| `pcl_alloc.c` | 16 | 16 | 100% |
| `pcl_bridge.c` | 28 | 27 | 96% |
| `pcl_capabilities.c` | 21 | 21 | 100% |
| `pcl_codec_registry.c` | 64 | 49 | 76% |
| `pcl_container.c` | 274 | 243 | 88% |
| `pcl_executor.c` | 745 | 635 | 85% |
| `pcl_log.c` | 16 | 15 | 93% |
| `pcl_plugin_loader.c` | 142 | 125 | 88% |
| `pcl_process_runtime.c` | 235 | 202 | 86% |
| `pcl_transport_apos.c` | 132 | 85 | 64% |
| `pcl_transport_routing.c` | 234 | 196 | 83% |
| `pcl_transport_shared_memory.c` | 977 | 622 | 63% |
| `pcl_transport_shared_memory_plugin.c` | 70 | 44 | 62% |
| `pcl_transport_socket.c` | 360 | 269 | 74% |
| `pcl_transport_socket_plugin.c` | 84 | 60 | 71% |
| `pcl_transport_template.c` | 196 | 143 | 73% |
| `pcl_transport_udp.c` | 174 | 128 | 73% |
| `pcl_transport_udp_plugin.c` | 78 | 50 | 64% |
| **Total** | **3,846** | **2,930** | **76%** |

GCC branch counters include compiler-synthesised branches (e.g. short-circuit
sub-expressions); a qualified decision-coverage tool will report different
figures. Treat this table as a prioritisation aid for DAL B planning, not as
decision-coverage evidence.

## Coverage Exclusions and Their Review

The sources contain 104 `GCOVR_EXCL_START`/`GCOVR_EXCL_LINE` sites. Each
states its claimed invariant inline. All sites were reviewed in this pass
against the rule that a path reachable through the public API must be covered
by a test, not excluded. One exclusion failed that review and was replaced by
a test (the `pcl_executor.c` default-transport case described above). The
remaining sites fall into six categories:

1. **Operating-system resource faults** that cannot be injected from a test
   (thread, semaphore, socket, or event creation failing; `sem_init`;
   `shm_open`/`mmap`).
2. **Heap exhaustion on paths the OOM harness cannot reach.** The
   allocation-failure harness (`test_pcl_oom`, `--wrap=malloc/calloc`) covers
   the paths it links; these sites sit in binaries where wrapping would
   corrupt shared coverage counters (GCC bug #68080) or on transport worker
   threads the wrap cannot target deterministically.
3. **Defensive guards against corrupted shared state or misbehaving peers**
   (unterminated names in a shared-memory region, unknown frame kinds,
   response frames with no matching pending entry). These reject inputs a
   conforming peer can never produce; the region is written only by this
   transport under the bus lock.
4. **Teardown reclaim of items stranded in flight** — drain loops that only
   execute when a message or stream is abandoned at exactly the wrong
   instant relative to shutdown.
5. **Internal preconditions proven at every call site** (for example the
   routing exclusive-group splitter, whose caller tokenises first; the
   shared-memory frame enqueue, whose callers resolve a valid slot under the
   bus lock).
6. **Sustained-congestion retry arms** whose timing (a mailbox staying full
   across a bounded retry window) cannot be produced deterministically; the
   externally visible outcomes on both sides of the retry are covered.

These dispositions are engineering analysis, not yet independently reviewed
evidence: the formal review cycle with named reviewers is tracked as
GAP-C-07 in `DO178C_GAP_ANALYSIS.md`.

## Coverage Harness Composition

The harness in `cmake/pcl_coverage` builds the complete PCL core (all ten
`pcl_core` sources), all production transports (socket, UDP, shared-memory,
template, APOS), the three reference transport plugins, the seven loader/
capabilities/routing/codec test plugins, and runs every PCL test binary
against them. The harness now calls `enable_testing()`, so
`ctest --test-dir build-coverage-pcl` discovers and runs the same 499 tests
as the explicit loop below.

| Test Binary | Tests | Purpose |
|-------------|-------|---------|
| `test_pcl_alloc` | 4 | Portable allocator: overflow rejection, realloc semantics |
| `test_pcl_lifecycle` | 14 | Container lifecycle, parameters, ports, tick rate |
| `test_pcl_executor` | 42 | Executor spin, dispatch, routing, shutdown, async invoke |
| `test_pcl_log` | 8 | Logging handler, level filtering, formatting |
| `test_pcl_robustness` | 91 | Edge cases, threading, throughput, route filtering, deferred responses |
| `test_pcl_streaming` | 14 | Streaming service/client paths |
| `test_pcl_bridge` | 9 | Bridge unit tests and port overflow |
| `test_pcl_dining` | 10 | Dining philosophers bridge/executor integration |
| `test_pcl_socket_transport` | 34 | TCP socket round trips, gateway, reconnect, keepalive |
| `test_pcl_socket_faults` | 8 | GCC-only socket fault injection, malformed wire frames |
| `test_pcl_udp_transport` | 13 | UDP datagram pub/sub, peer filtering, RPC absence |
| `test_pcl_shared_memory_transport` | 36 | SHM bus pub/sub/service/streaming, inter-process IPC |
| `test_pcl_template_transport` | 24 | Engineer-extensible scaffold + delivery conformance suite |
| `test_pcl_apos_transport` | 11 | APOS Local Virtual Channel transport |
| `test_pcl_transport_threading` | 11 | Threading-model conformance suite (PCL.075/076) |
| `test_pcl_codec_registry` | 10 | Codec vtable registration, lookup, iteration |
| `test_pcl_plugin_loader` | 29 | Plugin ABI gating, fail-closed loading, teardown-then-unload |
| `test_pcl_transport_routing` | 33 | Manifest loading, atomic rollback, fail-closed diagnostics |
| `test_pcl_capabilities` | 34 | Capability masks, QoS floors, compose-time validation |
| `test_pcl_process_runtime` | 18 | Deployment-file loading, plugin wiring, runtime lifecycle |
| `test_pcl_cpp_wrappers` | 32 | C++ `Component` and `Executor` wrappers |
| `test_pcl_oom` | 14 | GCC-only allocation-failure injection |
| **Total** | **499** | |

## Known Measurement Artifacts

- On some runs gcovr reports `Ignoring negative hits` on branch counters in
  `pcl_executor.c` and `pcl_transport_shared_memory.c` (branch counters
  only). This is the known GCC profile-runtime counter underflow when one
  object file is exercised by many test binaries; the lines themselves are
  covered. `--gcov-ignore-parse-errors negative_hits.warn` is set so the
  report is not poisoned. The final evidence run for this report produced
  no such warnings on the line-coverage pass.
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

# Run every coverage binary (or: ctest --test-dir build-coverage-pcl)
for exe in test_pcl_alloc test_pcl_lifecycle test_pcl_executor test_pcl_log \
           test_pcl_robustness test_pcl_streaming test_pcl_bridge \
           test_pcl_dining test_pcl_socket_transport test_pcl_socket_faults \
           test_pcl_udp_transport test_pcl_shared_memory_transport \
           test_pcl_template_transport test_pcl_apos_transport \
           test_pcl_transport_threading test_pcl_codec_registry \
           test_pcl_plugin_loader test_pcl_transport_routing \
           test_pcl_capabilities test_pcl_process_runtime \
           test_pcl_cpp_wrappers test_pcl_oom; do
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
