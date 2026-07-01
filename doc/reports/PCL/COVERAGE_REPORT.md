# PCL Statement Coverage Report

Generated: 2026-05-21

> **Scope note:** this snapshot predates the plugin/routing-era PCL sources
> added in late June 2026 (`pcl_codec_registry.c`, `pcl_plugin_loader.c`,
> `pcl_transport_routing.c`, `pcl_capabilities.c`, `pcl_alloc.c`), which are
> not in the per-file table below. Regenerate with
> `subprojects/PCL/scripts/coverage_pcl.{sh,bat}` for current figures.

Toolchain: `gcovr 8.6` with GCC 13.3.0 (`--coverage -O0 -g -fprofile-abs-path`)

Coverage artifacts:

- HTML: `coverage_pcl_current/index.html`
- Text summary: `coverage_pcl_current/summary.txt`

## Per-File Line Coverage

| Source File | Lines | Executed | Coverage | Status |
|-------------|-------|----------|----------|--------|
| `pcl_bridge.c` | 47 | 47 | **100%** | Full |
| `pcl_container.c` | 325 | 325 | **100%** | Full |
| `pcl_executor.c` | 683 | 683 | **100%** | Full |
| `pcl_log.c` | 28 | 28 | **100%** | Full |
| `pcl_transport_apos.c` | 202 | 183 | **90%** | See analysis |
| `pcl_transport_shared_memory.c` | 977 | 839 | **85%** | See analysis |
| `pcl_transport_socket.c` | 596 | 567 | **95%** | See analysis |
| `pcl_transport_template.c` | 367 | 306 | **83%** | See analysis |
| `pcl_transport_udp.c` | 166 | 146 | **88%** | See analysis |
| **Total** | **3391** | **3124** | **92%** | |

## Coverage Harness Composition

The source coverage harness in `cmake/pcl_coverage` builds the PCL core plus
all production transports (socket, UDP, shared-memory, template, APOS) and
runs every PCL test binary against them. This is a deliberate expansion since
the previous report -- the new transports added between 2026-04-22 and
2026-05-21 (template, APOS, shared-memory streaming) were not in the harness
when this work started.

| Test Binary | Tests | Purpose |
|-------------|-------|---------|
| `test_pcl_lifecycle` | 14 | Container lifecycle, parameters, ports, tick rate |
| `test_pcl_executor` | 32 | Executor spin, dispatch, routing, shutdown, async invoke, named-peer transport accessors, remote SVC drain |
| `test_pcl_log` | 5 | Logging handler, level filtering, formatting |
| `test_pcl_robustness` | 90 | Edge cases, threading, throughput, route filtering |
| `test_pcl_streaming` | 12 | Streaming service/client paths |
| `test_pcl_bridge` | 9 | Bridge unit tests and port overflow |
| `test_pcl_dining` | 10 | Dining philosophers bridge/executor integration |
| `test_pcl_socket_transport` | 31 | TCP socket transport round trips, gateway, reconnect, keepalive |
| `test_pcl_socket_faults` | 7 | GCC-only socket fault injection and malformed wire frames |
| `test_pcl_udp_transport` | 8 | UDP datagram pub/sub, peer filtering, RPC absence |
| `test_pcl_shared_memory_transport` | 13 | SHM bus pub/sub/service round-trip, inter-process IPC, **streaming** |
| `test_pcl_template_transport` | 15 | Engineer-extensible scaffold + transport conformance suite |
| `test_pcl_apos_transport` | 4 | APOS Local Virtual Channel transport on top of the template |
| `test_pcl_cpp_wrappers` | 31 | C++ `Component` and `Executor` wrappers |
| `test_pcl_oom` | 12 | GCC-only allocation failure injection (incl. remote service request strdup) |
| **Total** | **293** | |

## Changes Since Previous Report (2026-04-22)

| Change | Detail |
|--------|--------|
| Harness extended to all transports | `pcl_transport_udp`, `pcl_transport_shared_memory`, `pcl_transport_template`, and `pcl_transport_apos` are now compiled with coverage and exercised. `test_pcl_udp_transport`, `test_pcl_shared_memory_transport`, `test_pcl_template_transport`, and `test_pcl_apos_transport` were added to `cmake/pcl_coverage/CMakeLists.txt`. |
| Executor regression closed | `pcl_executor.c` had regressed from 100% to 95% after the metrics/shm-fix work. Four new tests cover `pcl_executor_get_transport`, `pcl_executor_get_transport_for_peer` (added by the alias-rebinding fix), the remote SVC dispatch path through `pcl_executor_post_service_request_remote`, the idle-wait branch in `pcl_executor_spin_once`, and the public `pcl_executor_containers_lock/unlock` helpers. |
| OOM coverage for remote SVC strdup | `test_pcl_oom` gained `PostServiceRequestRemoteSourcePeerIdStrdupFails`, closing the new five-line OOM rollback branch in `enqueue_svc_req`. `pcl_executor.c` is back at 100%. |
| SHM streaming coverage added | The shared-memory stream support added in `f83e04a` was completely untested. Four new tests cover the round-trip, missing-provider, handler abort, and destroy-during-active-stream paths, taking `pcl_transport_shared_memory.c` from 57% to 85%. |
| Negative-hit GCC artifact | gcovr reports `Ignoring negative hits in: branch 1 taken -1` on a single line (`pcl_executor.c:1184`). This is a GCC profile-runtime counter underflow when the same shared object file is exercised by many test binaries; the line itself is covered. `--gcov-ignore-parse-errors negative_hits.warn` is set so it does not poison the report. |

## Uncovered Lines Analysis

### `pcl_transport_socket.c` (29 uncovered lines, 95%)

| Lines | Function | Description |
|-------|----------|-------------|
| 258 | `connect_with_timeout` | `getsockopt(SO_ERROR)` itself fails after `select()` |
| 299 | `try_connect_addrinfo` | Legacy `total_timeout_ms == 0` branch; no current caller passes zero |
| 637-638 | `recv_thread_main` | Peer closes mid-payload between header and body recv |
| 816-818 | `recv_thread_main` | Second close in auto-reconnect loop |
| 943-944, 949-950, 961-962, 980-981, 986-987 | `pcl_socket_transport_create_server_ex` | `calloc()`, `socket()`, `bind()`, `listen()`, `accept()` failure paths |
| 1003, 1006 | server create | `pthread_create()` failure for recv/send threads |
| 1055-1056 | client create | `socket_transport_create_common()` failure path |
| 1133, 1136 | client create | `pthread_create()` failure for recv/send threads |
| 1307-1308 | destroy | Listen socket close when create failed before accept |
| 1356-1359 | destroy | Drain unsent send queue when send thread exits without draining |

These remain the same residual misses documented previously: socket/pthread fault paths require either a platform shim or a `--wrap` on Winsock/pthread calls. The GCC `--wrap` used for allocators is unreliable for these symbols on the Windows GNAT/MinGW environment.

### `pcl_transport_shared_memory.c` (138 uncovered lines, 85%)

The biggest residual categories are:

| Category | Approx lines | Description |
|----------|--------------|-------------|
| Unicode/long-name guard rails | 18 | `PCL_SHM_MAX_ID` truncation, magic-byte mismatch, OOM during open |
| Inbound frame parsing errors | 36 | Malformed `STREAM_REQ`/`SVC_REQ` wire frames (truncated length fields, OOM during decode) |
| Stream gateway error paths | 28 | Handler refuses request, handler returns error, response target alloc failure |
| Destroy-time stream tear-down | 14 | Streams already ended/cancelled by the time abort runs |
| Bus discovery retry loop | 9 | Provider slot races during discovery retries |
| Wire reflection on long bus names | 4 | `pcl_shm_copy_token` truncation cases |

Most of these are defensive/error-path code that the in-process loopback tests cannot directly trigger without injecting malformed wire bytes. A targeted bytestream-fuzz test would close them.

### `pcl_transport_template.c` (61 uncovered lines, 83%)

Almost all remaining gaps are init/teardown OOM paths:

| Category | Approx lines | Description |
|----------|--------------|-------------|
| `pthread_mutex_init` / `pthread_cond_init` failure | 14 | Lines 787-799 -- only triggered by an OS-level resource exhaustion |
| `pthread_create` failure | 22 | Lines 852-868 -- send/recv thread creation failure cleanup |
| `tpl_alias_remember("default")` OOM in `create` | 4 | Lines 880-881 -- alias seed strdup failure |
| `tpl_send_enqueue` racing destroy | 6 | Lines 281-283, 339-341 -- enqueue/drain after `send_stop` |
| Recv-thread log paths | 8 | Unknown SVC_RESP seq, malformed frame kind |

Closing these would need a template-transport OOM harness analogous to `test_pcl_oom`.

### `pcl_transport_apos.c` (19 uncovered lines, 90%)

| Category | Lines | Description |
|----------|-------|-------------|
| Encode/decode oversize guards | 75, 90-91, 171, 187-193 | Frames exceeding `PCL_APOS_MAX_FRAME` or with truncated headers |
| LVC open/wait failure paths | 242, 253-254, 257-258, 264, 297-298 | APOS process_id/LVC handle creation failures |

These need an APOS stub error-injection mode.

### `pcl_transport_udp.c` (20 uncovered lines, 88%)

| Category | Lines | Description |
|----------|-------|-------------|
| `socket()` / `setsockopt` / `bind` failure | 146, 150, 153-156, 204-205, 217-218 | OS-level socket failures during create |
| Oversize send guard | 298-299, 304-305 | Frames exceeding the UDP datagram cap |
| `recvfrom` non-EAGAIN error | 316-318 | Errors other than would-block |
| `pthread_create` failure | 356-358 | Recv thread creation failure |

## Mutex Audit Cross-Reference

A companion review of every mutex acquisition order across `pcl_executor.c`,
`pcl_transport_socket.c`, `pcl_transport_shared_memory.c`,
`pcl_transport_template.c`, and `pcl_transport_udp.c` was performed alongside
this coverage pass. See `MUTEX_AUDIT.md` (same directory) for the lock
hierarchy, deadlock-risk analysis, and contract observations. No
lock-order-inversion deadlocks were identified; the audit highlights two
unprotected fields (`e->transports[]`, `e->endpoint_routes[]`) that rely on
the documented single-threaded-mutator contract.

## Build and Run Instructions

```bash
# Configure coverage harness
cmake -S cmake/pcl_coverage -B build-coverage-pcl-current \
      -G "Unix Makefiles" \
      -DCMAKE_C_COMPILER=gcc \
      -DCMAKE_CXX_COMPILER=g++

# Build
cmake --build build-coverage-pcl-current -j"$(nproc)"

# Clear stale counters before a clean run
find build-coverage-pcl-current -name '*.gcda' -delete

# Run coverage binaries individually
for exe in test_pcl_lifecycle test_pcl_executor test_pcl_log \
           test_pcl_robustness test_pcl_streaming test_pcl_bridge \
           test_pcl_dining test_pcl_socket_transport test_pcl_socket_faults \
           test_pcl_udp_transport test_pcl_shared_memory_transport \
           test_pcl_template_transport test_pcl_apos_transport \
           test_pcl_cpp_wrappers test_pcl_oom; do
  ./build-coverage-pcl-current/${exe}
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

The Windows/GNAT runtime occasionally tripped a gcov profile-runtime access
violation on `test_pcl_bridge` and `test_pcl_cpp_wrappers` when run
immediately after other coverage binaries; final reports use counters from
passing isolated reruns. On Linux/GCC 13 (this run's environment) no such
flakes were observed.
