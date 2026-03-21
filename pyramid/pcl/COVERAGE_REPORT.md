# PCL Statement Coverage Report

Generated: 2026-03-21

Tool: `gcovr` 8.6 with GCC 10.3.1 (`--coverage -O0 -g`)

## Per-File Line Coverage

| Source File | Lines | Executed | Coverage | Status |
|-------------|-------|----------|----------|--------|
| `pcl_bridge.c` | 47 | 47 | **100%** | Full |
| `pcl_container.c` | 195 | 195 | **100%** | Full |
| `pcl_executor.c` | 281 | 281 | **100%** | Full |
| `pcl_log.c` | 28 | 28 | **100%** | Full |
| `pcl_transport_socket.c` | 387 | 352 | **91%** | See analysis |
| **Total** | **938** | **903** | **96.3%** | |

## Function Coverage

| Metric | Count | Coverage |
|--------|-------|----------|
| Total functions | 84 | **100%** |
| Executed | 84 | |

All 84 functions across all PCL source files are exercised by the test suite.

## Branch Coverage

| Metric | Count | Coverage |
|--------|-------|----------|
| Total branches | 620 | **82.9%** |
| Taken | 514 | |

Branch coverage is lower than line coverage because many error-handling branches (malloc failures, socket API errors) are only reachable through fault injection.

## Test Suite Composition

| Test File | Tests | LLRs | Purpose |
|-----------|-------|------|---------|
| `test_pcl_lifecycle.cpp` | 14 | 001–008, 013–015, 019–020, 028–030, 111 | Container lifecycle, parameters, ports, tick rate |
| `test_pcl_executor.cpp` | 10 | 031–036, 044–045, 049, 055, 112 | Executor spin, dispatch, multi-container, shutdown |
| `test_pcl_log.cpp` | 5 | 064–068 | Logging handler, level filtering |
| `test_pcl_robustness.cpp` | 55 | 009–012, 016–027, 037–043, 046–048, 050–054, 056–063, 069–074, 091–093, 113–114 | Edge cases, threading, throughput |
| `test_pcl_dining.cpp` | 10 | 075–084 | Bridge unit tests + dining philosophers |
| `test_pcl_oom.cpp` | 6 | 085–090 | Out-of-memory injection (Linux only) |
| `test_pcl_socket_transport.cpp` | 22 | 115–130, 158–163 | TCP socket transport |
| `test_pcl_cpp_wrappers.cpp` | 27 | 131–157 | C++ Component and Executor wrappers |
| `test_pcl_proto_bindings.cpp` | 19 | 094–110 | Generated service bindings |
| **Total** | **168** | **163** | |

## Uncovered Lines Analysis — `pcl_transport_socket.c`

35 lines remain uncovered (91% coverage). All fall into categories that require fault injection or are platform-specific dead code on the current build target.

### Category 1: Allocation Failure Paths (4 lines)

| Lines | Function | Description |
|-------|----------|-------------|
| 174–175 | `enqueue_outbound_frame` | `malloc(size)` for frame data fails |
| 763–764 | `invoke_remote_async` | `malloc(frame)` for outbound frame fails |

**Why uncovered**: Requires `--wrap=malloc` linker injection (as used by `test_pcl_oom` for `pcl_core`). The transport library is a separate link unit.

### Category 2: Socket API Failure Paths (12 lines)

| Lines | Function | Description |
|-------|----------|-------------|
| 594–595 | `create_server` | `socket()` returns INVALID_SOCKET |
| 606–607 | `create_server` | `bind()` fails |
| 621–622 | `create_server` | `listen()` fails |
| 627–628 | `create_server` | `accept()` returns INVALID_SOCKET |
| 691 | `create_client` | `socket()` returns INVALID_SOCKET |
| 853–854 | `destroy` | `listen_sock` still open at destroy time |

**Why uncovered**: These require OS-level fault injection or resource exhaustion. The `bind()` failure case was attempted but Windows `SO_REUSEADDR` semantics prevent reliable triggering in a unit test.

### Category 3: Wire Protocol Edge Cases (4 lines)

| Lines | Function | Description |
|-------|----------|-------------|
| 430 | `recv_thread` | `type_len` bounds check fails (malformed packet) |
| 513 | `recv_thread` | Unknown message type byte |

**Why uncovered**: Requires sending raw malformed TCP data to the transport socket, bypassing the transport API. Would need a dedicated wire-fuzzing test harness.

### Category 4: Dead Code (2 lines)

| Lines | Function | Description |
|-------|----------|-------------|
| 406–407 | `recv_thread` | `payload_len < 1` check after `payload_len == 0` already exits at line 395 |

**Why uncovered**: Defensive check is unreachable. The `payload_len == 0` condition at line 395 exits the loop before reaching line 405. Could be removed, but serves as belt-and-suspenders safety.

### Category 5: Ephemeral Port Path (3 lines)

| Lines | Function | Description |
|-------|----------|-------------|
| 612 | `create_server` | `int len` declaration for `getsockname` (Windows path) |
| 616–617 | `create_server` | `getsockname()` + `ntohs()` to read assigned port |

**Why uncovered**: Only runs when `port == 0` is passed to `create_server`. The test harness pre-selects an ephemeral port to avoid race conditions, so `port` is always non-zero.

### Category 6: Enqueue Failure Rollback (6 lines)

| Lines | Function | Description |
|-------|----------|-------------|
| 807 | `invoke_remote_async` | Enter pending_lock after enqueue failure |
| 811–816 | `invoke_remote_async` | Walk pending list to remove failed record |
| 820 | `invoke_remote_async` | Leave pending_lock |

**Why uncovered**: Requires `enqueue_outbound_frame` to fail (which requires malloc failure) inside `invoke_remote_async` after the pending record is registered.

### Category 7: Cleanup Drain Loops (4 lines)

| Lines | Function | Description |
|-------|----------|-------------|
| 900–903 | `destroy` | Drain unsent outbound frames from send queue |

**Why uncovered**: The send thread drains and sends queued frames before `destroy` can join it, leaving `send_head` empty. A burst-publish test was added (`DestroyWithUnsentFramesNoLeak`) but the send thread is fast enough to process all frames before teardown.

## Path to 100%

Achieving 100% line coverage for `pcl_transport_socket.c` would require:

1. **Malloc interception** (`--wrap=malloc`) for allocation failure paths (6 lines) — feasible, similar to existing `test_pcl_oom` approach, but needs a separate `pcl_transport_socket_oom` link unit to avoid gcda counter overflow
2. **Wire-level fuzzing harness** for malformed protocol data (4 lines) — requires sending raw bytes on the socket
3. **Platform-specific socket mocking** for socket API failures (12 lines) — requires wrapping `socket()`, `bind()`, `listen()`, `accept()`
4. **Remove dead code** at lines 406–407 (2 lines) — defensive check is unreachable
5. **Alternative test architecture** for the ephemeral port path (3 lines) — pass `port=0` directly to `create_server`

Items 1 and 5 are practical additions; items 2–3 are significant infrastructure investments. Item 4 is a code cleanup.

## Build and Run Instructions

```bash
# Configure
cmake -S cmake/pcl_coverage -B build-coverage-pcl -G "Unix Makefiles" \
      -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++

# Build
cmake --build build-coverage-pcl -j

# Run all tests
for test in test_pcl_lifecycle test_pcl_executor test_pcl_log \
            test_pcl_robustness test_pcl_dining test_pcl_oom \
            test_pcl_socket_transport test_pcl_cpp_wrappers; do
  ./build-coverage-pcl/${test}.exe
done

# Generate report
gcovr --root . --filter "src/pcl/" \
      --gcov-ignore-parse-errors=negative_hits.warn_once_per_file \
      --print-summary \
      --txt coverage_pcl/summary.txt \
      --html-details coverage_pcl/index.html \
      build-coverage-pcl
```
