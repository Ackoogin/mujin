# PCL Statement Coverage Report

Generated: 2026-04-12

Tool: `gcovr` with GCC 10.3.1 (`--coverage -O0 -g -fprofile-abs-path`)

## Per-File Line Coverage

| Source File | Lines | Executed | Coverage | Status |
|-------------|-------|----------|----------|--------|
| `pcl_bridge.c` | 47 | 47 | **100%** | Full |
| `pcl_container.c` | 323 | 323 | **100%** | Full |
| `pcl_executor.c` | 621 | 607 | **97%** | See analysis |
| `pcl_log.c` | 28 | 28 | **100%** | Full |
| `pcl_transport_socket.c` | 448 | 414 | **92%** | See analysis |
| **Total** | **1467** | **1419** | **96.7%** | |

## Test Suite Composition

| Test File | Tests | LLRs | Purpose |
|-----------|-------|------|---------|
| `test_pcl_lifecycle.cpp` | 14 | 001–008, 013–015, 019–020, 028–030, 111 | Container lifecycle, parameters, ports, tick rate |
| `test_pcl_executor.cpp` | 26 | 031–036, 044–045, 049, 055, 112 | Executor spin, dispatch, multi-container, shutdown, invoke\_async |
| `test_pcl_log.cpp` | 5 | 064–068 | Logging handler, level filtering |
| `test_pcl_robustness.cpp` | 90 | 009–012, 016–027, 037–043, 046–048, 050–054, 056–063, 069–074, 091–093, 113–114 | Edge cases, threading, throughput |
| `test_pcl_streaming.cpp` | 12 | — | Streaming port tests |
| `test_pcl_bridge.cpp` | 9 | 075–084 | Bridge unit tests |
| `test_pcl_dining.cpp` | 10 | — | Dining philosophers (bridge + executor integration) |
| `test_pcl_oom.cpp` | 6 | 085–090 | Out-of-memory injection (GCC `--wrap` only) |
| `test_pcl_socket_transport.cpp` | 23 | 115–130, 158–164 | TCP socket transport |
| `test_pcl_cpp_wrappers.cpp` | — | 131–157 | C++ Component and Executor wrappers |
| **Total** | **195+** | | |

## Uncovered Lines Analysis — `pcl_executor.c`

14 lines remain uncovered (97% coverage). All are OOM cleanup paths.

### Allocation Failure Paths (14 lines)

| Lines | Function | Description |
|-------|----------|-------------|
| 512–513 | `drain_svc_req_queue` | `malloc` for queued service request fails |
| 1073–1075 | `post_response_msg` | `malloc` for response message node fails |
| 1168–1169 | `post_service_request` | `malloc` for pending record fails |
| 1175–1177 | `post_service_request` | `malloc` for request data copy fails |
| 1184–1187 | `post_service_request` | `malloc` for service request queue node fails |

**Why uncovered**: Requires `--wrap=malloc`/`--wrap=calloc` linker injection. The OOM test binary (`test_pcl_oom`) uses a separate `pcl_core_oom` link unit to avoid gcda counter overflow (GCC bug #68080); extending it to executor internals is feasible but was not needed to demonstrate functional correctness.

## Uncovered Lines Analysis — `pcl_transport_socket.c`

34 lines remain uncovered (92% coverage). All fall into categories that require fault injection.

### Category 1: Allocation Failure Paths (12 lines)

| Lines | Function | Description |
|-------|----------|-------------|
| 175–176 | `enqueue_outbound_frame` | `malloc` for frame data fails |
| 545–546 | `recv_thread` | `malloc` for receive payload fails |
| 608 | `recv_thread` | `malloc` for service request queue node fails |
| 783 | `create_client` | `malloc` for pending record fails |
| 864–865 | `invoke_remote_async` | `malloc` for frame buffer fails after pending registered |
| 913 | `invoke_remote_async` | `enqueue_outbound_frame` failed — enter lock to remove pending |
| 917–922 | `invoke_remote_async` | Walk pending list to remove failed record |
| 926 | `invoke_remote_async` | Leave lock after pending rollback |

**Why uncovered**: Requires `--wrap=malloc` linker injection into the transport link unit.

### Category 2: Socket API Failure Paths (8 lines)

| Lines | Function | Description |
|-------|----------|-------------|
| 672–673 | `create_server_ex` | `socket()` returns INVALID_SOCKET |
| 684–685 | `create_server_ex` | `bind()` fails |
| 703–704 | `create_server_ex` | `listen()` fails |
| 709–710 | `create_server_ex` | `accept()` returns INVALID_SOCKET |

**Why uncovered**: Requires OS-level fault injection (`socket()`, `bind()`, `listen()`, `accept()` returning errors). The `bind()` failure case is additionally unreliable on Windows because `SO_REUSEADDR` permits re-binding in-use ports.

### Category 3: Wire Protocol Edge Cases (6 lines)

| Lines | Function | Description |
|-------|----------|-------------|
| 460 | `recv_thread` | `type_len` exceeds remaining payload (malformed PUBLISH) |
| 534–535 | `recv_thread` | Unknown message type byte |
| 587 | `recv_thread` | `svc_name_len` exceeds remaining payload (malformed SVC_REQ) |

**Why uncovered**: Requires injecting raw malformed bytes directly onto the socket, bypassing the transport API.

### Category 4: Defensive Safety Code (6 lines)

| Lines | Function | Description |
|-------|----------|-------------|
| 960–961 | `destroy` | Close `listen_sock` still open at destroy time |
| 1007–1010 | `destroy` | Drain unsent outbound frames remaining in send queue after thread join |

**Why uncovered**: `listen_sock` is closed immediately after `accept()` succeeds in the normal path; it is only still open if `accept()` itself fails (which requires OS fault injection — see Category 2). The frame drain loop requires frames to be enqueued between the send thread's final dequeue check and `join()` returning — a genuine data race that cannot be reliably forced from a test.

## Changes from Previous Report

| Change | Detail |
|--------|--------|
| Dead code removed | `payload_len < 1u` check in `recv_thread` (formerly lines 440–442) was unreachable — prior `payload_len == 0u` check at line 430 already exits. Removed from source. |
| Ephemeral port path covered | Added `pcl_socket_transport_create_server_ex()` (exposes bound port before blocking on `accept()`) and `pcl_socket_transport_get_port()` accessor. New test `ServerEphemeralPortAssigned` covers lines 693–700 (getsockname branch for `port == 0`). |
| Executor invoke_async covered | Added 5 new tests covering `set_transport` null clear, endpoint route update, intra-process immediate response, intra-process deferred response, and publisher remote route mode. |
| `pcl_bridge.c` 0% → 100% | Build system was missing `${PCL_ROOT}/src` include path for `test_pcl_bridge`, so the binary was never built. Fixed in `cmake/pcl_coverage/CMakeLists.txt`. |
| `pcl_container.c` coverage increased | Additional lifecycle and robustness tests raised coverage from prior level to 100%. |

## Path to 100%

Remaining gaps and the effort required to close them:

1. **OOM injection for executor and transport** — create `pcl_executor_oom` and `pcl_transport_socket_oom` link units (separate `.a` files) to avoid gcda counter overflow (GCC bug #68080), then use `--wrap=malloc,--wrap=calloc` as done by `test_pcl_oom`. Covers ~26 lines.
2. **Socket API wrapping** — `--wrap=socket,--wrap=bind,--wrap=listen,--wrap=accept` in a dedicated link unit. Covers 8 lines.
3. **Wire-level fuzzing harness** — inject raw malformed bytes via a raw loopback socket to exercise protocol parser edge cases. Covers 6 lines.
4. **Defensive safety code** — lines 960–961 and 1007–1010 are reachable only via OS fault injection or data races; they can be left as accepted misses or removed if proven truly unreachable.

Items 1 and 2 are practical additions with moderate effort. Items 3 and 4 have diminishing returns.

## Build and Run Instructions

```bash
# Configure (Windows, GCC via GNAT)
cmake -S cmake/pcl_coverage -B build-coverage-pcl-current -G "Unix Makefiles" \
      -DCMAKE_C_COMPILER=/c/GNAT/2021/bin/gcc \
      -DCMAKE_CXX_COMPILER=/c/GNAT/2021/bin/g++

# Build
cmake --build build-coverage-pcl-current -j4

# Run all tests (GNAT DLLs must be on PATH)
export PATH="/c/GNAT/2021/bin:$PATH"
cd build-coverage-pcl-current
for exe in test_pcl_lifecycle test_pcl_executor test_pcl_log \
           test_pcl_robustness test_pcl_streaming test_pcl_bridge \
           test_pcl_dining test_pcl_socket_transport \
           test_pcl_cpp_wrappers test_pcl_oom; do
  ./${exe}.exe
done

# Generate report
cd ..
gcovr --root subprojects/PCL/src \
      --filter 'subprojects/PCL/src/' \
      --gcov-executable gcov \
      --gcov-ignore-parse-errors=negative_hits.warn_once_per_file \
      D:/Dev/repo/mujin/build-coverage-pcl-current
```
