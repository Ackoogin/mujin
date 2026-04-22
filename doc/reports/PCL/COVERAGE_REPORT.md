# PCL Statement Coverage Report

Generated: 2026-04-22

Toolchain: `gcovr 8.6` with GCC 10.3.1 (`--coverage -O0 -g -fprofile-abs-path`)

Coverage artifacts:

- HTML: `coverage_pcl_current/index.html`
- Text summary: `coverage_pcl_current/summary.txt`

## Per-File Line Coverage

| Source File | Lines | Executed | Coverage | Status |
|-------------|-------|----------|----------|--------|
| `pcl_bridge.c` | 47 | 47 | **100%** | Full |
| `pcl_container.c` | 323 | 323 | **100%** | Full |
| `pcl_executor.c` | 621 | 621 | **100%** | Full |
| `pcl_log.c` | 28 | 28 | **100%** | Full |
| `pcl_transport_socket.c` | 585 | 566 | **96%** | See analysis |
| **Total** | **1604** | **1585** | **98.8%** | |

## Coverage Harness Composition

The source coverage harness in `cmake/pcl_coverage` builds the PCL core plus TCP socket transport coverage targets. The UDP and shared-memory transport tests remain in the normal PCL test CMake but are not part of this source coverage report yet.

| Test Binary | Tests | Purpose |
|-------------|-------|---------|
| `test_pcl_lifecycle` | 14 | Container lifecycle, parameters, ports, tick rate |
| `test_pcl_executor` | 26 | Executor spin, dispatch, routing, shutdown, async invoke |
| `test_pcl_log` | 5 | Logging handler, level filtering, formatting |
| `test_pcl_robustness` | 90 | Edge cases, threading, throughput, route filtering |
| `test_pcl_streaming` | 12 | Streaming service/client paths |
| `test_pcl_bridge` | 9 | Bridge unit tests and port overflow |
| `test_pcl_dining` | 10 | Dining philosophers bridge/executor integration |
| `test_pcl_socket_transport` | 31 | TCP socket transport round trips, gateway, reconnect, keepalive |
| `test_pcl_socket_faults` | 7 | GCC-only socket fault injection and malformed wire frames |
| `test_pcl_cpp_wrappers` | 27 | C++ `Component` and `Executor` wrappers |
| `test_pcl_oom` | 11 | GCC-only allocation failure injection |
| **Total** | **242** | |

## Changes Since Previous Report

| Change | Detail |
|--------|--------|
| Executor OOM gaps closed | Added allocation-failure tests for `pcl_executor_post_response_msg`, `pcl_executor_post_service_request`, and the queued service request context allocation path. `pcl_executor.c` is now 100%. |
| Socket fault coverage added | Added `test_pcl_socket_faults.cpp`, a GCC-only suite using linker-wrapped allocation plus raw loopback frames to cover malformed publish/response frames, unknown message types, response type allocation failure, invoke rollback after enqueue failure, gateway port overflow, retry sleep, and reconnect backoff. |
| Coverage harness expanded | Added `test_pcl_socket_faults` to both `cmake/pcl_coverage/CMakeLists.txt` and the normal PCL test CMake under `if(NOT MSVC)`. |
| Report command hardened | `gcovr` is now run against `build-coverage-pcl-current` explicitly so stale counters from other coverage builds are not merged into the report. |

## Uncovered Lines Analysis - `pcl_transport_socket.c`

19 lines remain uncovered in `pcl_transport_socket.c`.

### OS Socket API Failure Paths (10 lines)

| Lines | Function | Description |
|-------|----------|-------------|
| 246 | `connect_with_timeout` | `getsockopt(SO_ERROR)` itself fails after `select()` |
| 926-927 | `pcl_socket_transport_create_server_ex` | `socket()` returns invalid |
| 938-939 | `pcl_socket_transport_create_server_ex` | `bind()` fails |
| 957-958 | `pcl_socket_transport_create_server_ex` | `listen()` fails |
| 963-964 | `pcl_socket_transport_create_server_ex` | `accept()` fails |

**Why uncovered**: These require OS/Winsock API fault injection. GCC `--wrap` is reliable for allocator calls in this harness, but not for Winsock import-thunk calls in the GNAT/MinGW environment used for this run.

### Defensive / Unreachable Paths (9 lines)

| Lines | Function | Description |
|-------|----------|-------------|
| 287 | `try_connect_addrinfo` | Legacy `total_timeout_ms == 0` branch; no current caller passes zero |
| 802-804 | `recv_thread_main` | Second close in auto-reconnect loop; the preceding disconnect block normally closes and invalidates the socket first |
| 1281-1282 | `pcl_socket_transport_destroy` | Destroy closes `listen_sock`; reachable only when server creation fails before the normal accept/close sequence |
| 1328-1331 | `pcl_socket_transport_destroy` | Drain unsent frames after send thread join; the send thread drains the queue before exiting in normal operation |

## Path to 100%

1. Add a platform shim for Winsock calls (`socket`, `bind`, `listen`, `accept`, `getsockopt`) instead of relying on linker wrapping. This would cover the server creation and `connect_with_timeout` failure branches.
2. Remove or refactor the now-unused `total_timeout_ms == 0` branch in `try_connect_addrinfo` if the current bounded-call contract is retained.
3. Review the defensive destroy-time queue drain and reconnect double-close paths. If they are retained as safety code, they are acceptable residual misses; if they are proven unreachable, remove or narrow them.

## Build and Run Instructions

```bash
# Configure coverage harness (Windows, GCC via GNAT)
cmake -S cmake/pcl_coverage -B build-coverage-pcl-current -G "Unix Makefiles" \
      -DCMAKE_C_COMPILER=gcc \
      -DCMAKE_CXX_COMPILER=g++

# Build
cmake --build build-coverage-pcl-current -j4

# Clear stale counters before a clean run
find build-coverage-pcl-current -name '*.gcda' -delete

# Run coverage binaries individually
for exe in test_pcl_lifecycle test_pcl_executor test_pcl_log \
           test_pcl_robustness test_pcl_streaming test_pcl_bridge \
           test_pcl_dining test_pcl_socket_transport test_pcl_socket_faults \
           test_pcl_cpp_wrappers test_pcl_oom; do
  ./build-coverage-pcl-current/${exe}.exe
done

# Generate report
gcovr --root . \
      --filter "subprojects/PCL/src/" \
      --gcov-executable gcov \
      --gcov-ignore-errors source_not_found \
      --gcov-ignore-errors no_working_dir_found \
      --gcov-ignore-parse-errors negative_hits.warn \
      --html-details=coverage_pcl_current/index.html \
      --txt=coverage_pcl_current/summary.txt \
      build-coverage-pcl-current
```

On this Windows/GNAT setup, `test_pcl_bridge` and `test_pcl_cpp_wrappers` occasionally tripped a gcov profile-runtime access violation when run immediately after other coverage binaries, then passed on immediate isolated rerun. The final report uses counters from passing isolated reruns.
