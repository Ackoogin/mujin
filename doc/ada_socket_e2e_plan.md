# Plan: Extend E2E Test with Ada Language Client and Socket Transport

## Overview

Extend the existing tactical objects E2E test (`Test_TacticalObjects_E2E.cpp`) to
exercise a **true cross-process, cross-language** scenario:

- **Server process** (C++): `TacticalObjectsComponent` running in a PCL executor
  with `pcl_transport_socket` in server mode.
- **Client process** (Ada): An Ada executable using `pcl_bindings.ads` and the
  socket transport wire protocol to subscribe to an interest, receive entity
  update frames over TCP, and decode them.

This validates the full stack: Ada FFI bindings → TCP socket transport →
binary streaming codec → interest-filtered entity publication.

### What already exists

| Piece | Location | Status |
|-------|----------|--------|
| Ada FFI bindings | `examples/ada/pcl_bindings.ads` | Working (lifecycle, subscribe, post_incoming) |
| Ada OO wrapper | `examples/ada/pcl_component.ads/.adb` | Working (tagged types, RAII) |
| Ada sensor demo | `examples/ada/pcl_sensor_demo.adb` | Working (intra-process pub/sub) |
| TCP socket transport | `src/pcl/pcl_transport_socket.c` | Working (server + client, threaded recv) |
| Socket transport header | `include/pcl/pcl_transport_socket.h` | Stable C API |
| Binary streaming codec | `pyramid/tactical_objects/src/StreamingCodec.cpp` | Working |
| In-process E2E test | `tests/tactical_objects/Test_TacticalObjects_E2E.cpp` | Passing |

### What needs to be built

1. Ada bindings for the socket transport API (`pcl_transport_socket`)
2. Ada bindings for `pcl_executor_invoke_service` (missing from current bindings)
3. Ada binary streaming codec (decode-only, matching `StreamingCodec` wire format)
4. Ada tactical objects client executable (connects, subscribes, receives, decodes)
5. C++ socket transport E2E test (server-side harness)
6. Cross-process test driver script

---

## Step 1 — Extend Ada FFI Bindings for Socket Transport

**Goal:** Add Ada import declarations for the socket transport C API and the
missing `pcl_executor_invoke_service` function.

### Tasks

- [ ] Add to `examples/ada/pcl_bindings.ads`:
  - `Pcl_Socket_Transport` opaque type + access type
  - `Create_Socket_Server` → `pcl_socket_transport_create_server`
  - `Create_Socket_Client` → `pcl_socket_transport_create_client`
  - `Get_Transport` → `pcl_socket_transport_get_transport`
  - `Invoke_Remote` → `pcl_socket_transport_invoke_remote`
  - `Destroy_Socket_Transport` → `pcl_socket_transport_destroy`
  - `Pcl_Transport` record matching `pcl_transport_t` (5 function pointers + `adapter_ctx`)
  - `Set_Transport` → `pcl_executor_set_transport`
  - `Invoke_Service` → `pcl_executor_invoke_service`

- [ ] Add `Pcl_Service_Handler_Access` callback type for completeness (allows
  Ada-side service registration in future)

### Files touched

- `examples/ada/pcl_bindings.ads` (extend)

### Notes

- All types use `pragma Convention(C, ...)` for ABI compatibility.
- `pcl_transport_t` contains C function pointers — Ada must declare matching
  `access function` types with `Convention => C`.  However, the Ada client
  only calls `Set_Transport` / `Invoke_Remote` and never fills in the vtable
  directly, so the record can be treated as an opaque blob
  (`array (1..48) of Interfaces.C.unsigned_char` or similar) if function
  pointer mapping proves fragile.

---

## Step 2 — Ada Binary Streaming Codec (Decode-Only)

**Goal:** Implement a minimal Ada package that can decode the batch frame wire
format produced by `StreamingCodec::encodeBatchFrame`.  Encode is not needed
since the Ada client only receives entity updates.

### Wire format recap

```
Batch header: type(1)=0x03 + entity_count(4 LE) + tick_timestamp(8 LE)
Per entity:   size(4 LE) + inner_frame
Inner frame:  msg_type(1) + uuid(16) + version(8 LE) + timestamp(8 LE)
              + field_mask(2 LE) + field payloads (variable)
```

### Tasks

- [ ] Create `examples/ada/streaming_codec.ads` (spec):
  - `type Entity_Update_Frame` record with optional-style fields
  - `function Decode_Batch(Data : System.Address; Len : Interfaces.C.unsigned)
     return Entity_Frame_Array`
  - Little-endian read helpers (`Read_U16_LE`, `Read_U32_LE`, `Read_U64_LE`,
    `Read_F64_LE`)

- [ ] Create `examples/ada/streaming_codec.adb` (body):
  - Parse batch header (validate `0x03` type byte, read count and timestamp)
  - Loop over per-entity frames: read size prefix, then inner frame
  - For each set bit in `field_mask`, read the corresponding payload
  - Skip fields that are not needed by the client (or decode all for validation)
  - Return array of decoded frames

- [ ] Unit validation:
  - Hard-code a known-good encoded batch (captured from C++ test) and verify
    Ada decode matches expected field values
  - Verify truncated input returns empty array (no crash)

### Design decisions

- Ada `Unchecked_Conversion` or overlay (`for X'Address use ...`) for
  little-endian reads on little-endian hosts.  For portability, use
  shift-and-mask like the C codec.
- Use `Ada.Streams.Stream_Element_Array` or raw `System.Address` + offset
  for buffer traversal (same pattern as the C `readU8` / `readU16LE` helpers).

---

## Step 3 — Ada Tactical Objects Client Executable

**Goal:** A standalone Ada executable that:
1. Connects to a `TacticalObjectsComponent` server via TCP socket transport.
2. Invokes `subscribe_interest` (remote service call over socket).
3. Receives `entity_updates` (PUBLISH messages dispatched to a subscriber).
4. Decodes batch frames using the Ada streaming codec.
5. Prints results to stderr and exits with status 0 on success.

### Tasks

- [ ] Create `examples/ada/ada_tobj_client.adb`:
  - Parse command-line args: `--host <host> --port <port>` (default: 127.0.0.1:0)
  - Create executor and socket client transport
  - Set transport on executor
  - Create a PCL container "ada_tobj_client" with:
    - `on_configure`: subscribe to `entity_updates` topic
    - `on_message`: decode binary batch frame, count received entities
  - Configure + activate container, add to executor
  - Build JSON request for `subscribe_interest`:
    ```json
    {"object_type":"Platform","affiliation":"Hostile",
     "area":{"min_lat":50,"max_lat":52,"min_lon":-1,"max_lon":1},
     "expires_at":9999}
    ```
  - Call `Invoke_Remote("subscribe_interest", ...)` — blocks for response
  - Parse response JSON to extract `interest_id`
  - Spin executor in a loop (up to N iterations or timeout)
  - Assert at least one entity update frame was received
  - Print summary and exit

- [ ] Create `examples/ada/ada_tobj_client.gpr`:
  - Languages: Ada, C
  - Source dirs: `.`, `../../src/pcl` (for `pcl_container.c`, `pcl_executor.c`,
    `pcl_transport_socket.c`, etc.)
  - C compiler switches: `-std=c17 -I../../include -I../../src/pcl`
  - Linker switches: `-lpthread` (POSIX)

- [ ] Minimal JSON builder for the subscribe request:
  - Option A: String concatenation (sufficient for a fixed request shape)
  - Option B: Tiny Ada JSON package (future extensibility)
  - Recommend Option A for initial implementation

### Flow diagram

```
Ada Client Process                         C++ Server Process
──────────────────                         ──────────────────
executor + socket client transport    ←TCP→ executor + socket server transport
ada_tobj_client container                   TacticalObjectsComponent
  ├─ subscribe to "entity_updates"          ├─ services: subscribe_interest,
  ├─ invoke_remote("subscribe_interest")    │   create_object, etc.
  │   → SERVICE_REQ over TCP ──────────→    │   → gateway dispatches to handler
  │   ← SERVICE_RESP over TCP ←────────    │   ← response with interest_id
  │                                         │
  │                                         │ on_tick():
  │                                         │   assembleStreamFrame
  │   ← PUBLISH "entity_updates" ←────    │   pcl_port_publish → socket_publish
  └─ on_message: decode batch frame         └─
```

---

## Step 4 — C++ Server-Side E2E Test Harness

**Goal:** A C++ test executable that starts a `TacticalObjectsComponent` with
socket transport in server mode, creates test entities, and waits for the
Ada client to connect and exercise the flow.

### Tasks

- [ ] Create `tests/tactical_objects/Test_TacticalObjects_SocketE2E.cpp`:
  - Use ephemeral port (port=0) for the server to avoid conflicts
  - Write the assigned port to a temp file so the Ada client can read it
  - Sequence:
    1. Create executor
    2. Create socket server transport (port=0, blocks on accept)
       → Run in a background thread or use a connection-ready callback
    3. Set transport on executor
    4. Create and configure `TacticalObjectsComponent`
    5. Add gateway container + tobj container to executor
    6. Create a test entity (Platform, Hostile, position in zone)
    7. Spin executor until shutdown requested or timeout
    8. Destroy transport and executor

- [ ] Alternative: integrate into existing `Test_TacticalObjects_E2E.cpp` as
  a second test case (`SocketTransport_AdaClient`) gated behind an
  environment variable or CMake option (since it requires GNAT toolchain)

- [ ] Add CMake target `test_tobj_socket_e2e`:
  - Link against `tactical_objects`, `tactical_objects_component`, `pcl_core`,
    `pcl_transport_socket`
  - Platform link: `pthread` (POSIX) or `ws2_32` (Windows)

### Server threading consideration

The `pcl_socket_transport_create_server` call blocks on `accept()`.  The test
must either:
- (A) Spawn a thread that calls `accept` then runs the executor spin loop, or
- (B) Fork the Ada client first (it will connect), then call `create_server`
  (accept returns immediately since client is waiting)

**Recommended: Option A** — spawn a `std::thread` that creates the server
transport (which blocks on accept), then spins the executor.  The main thread
launches the Ada client subprocess after a brief delay, pointing it at the
known port.

Port discovery: use port=0 (ephemeral), but since `create_server` blocks on
accept, the port is only known after `create_server` returns.  So the server
thread must signal the port back to the main thread.  Use a simple
`std::promise<uint16_t>` or write it to a pipe.

### Alternative: avoid threading

A simpler approach:
1. Pre-select a port (e.g., 0 with `getsockname` after bind — but this requires
   splitting `create_server` or using raw sockets).
2. Or use a fixed port from a range unlikely to conflict (e.g., 19000 + pid%1000).

**Simplest viable approach:** Use `pcl_socket_transport_create_server` with a
fixed test port, but run the Ada client in a background process that retries
connection.  Accept that CI may need port isolation.

---

## Step 5 — Cross-Process Test Driver

**Goal:** A script (or CTest custom command) that orchestrates the server and
client processes and reports pass/fail.

### Tasks

- [ ] Create `scripts/test_ada_socket_e2e.sh` (POSIX) /
  `scripts/test_ada_socket_e2e.bat` (Windows):
  1. Build Ada client: `gprbuild -P examples/ada/ada_tobj_client.gpr`
  2. Start C++ server in background: `build/tests/test_tobj_socket_e2e &`
  3. Wait for port file to appear (poll with timeout)
  4. Read port from file
  5. Start Ada client: `examples/ada/bin/ada_tobj_client --port $PORT`
  6. Wait for Ada client to exit
  7. Signal server to stop (send SIGTERM or wait for timeout exit)
  8. Report pass/fail based on exit codes

- [ ] Add CTest integration (optional, gated on GNAT availability):
  ```cmake
  find_program(GPRBUILD gprbuild)
  if(GPRBUILD)
    add_test(NAME tobj_ada_socket_e2e
             COMMAND ${CMAKE_SOURCE_DIR}/scripts/test_ada_socket_e2e.sh)
    set_tests_properties(tobj_ada_socket_e2e PROPERTIES
      LABELS "cross_language;integration"
      TIMEOUT 30)
  endif()
  ```

---

## Step 6 — C++ Only Socket Transport Test (No Ada Dependency)

**Goal:** Even without GNAT, validate the socket transport path using a C++
client that mimics what the Ada client would do.  This test is always buildable
and runnable.

### Tasks

- [ ] Add test case to `Test_TacticalObjects_E2E.cpp`:
  `SocketTransport_CppClientReceivesEntityUpdates`
  - Spawn server thread (creates socket server transport, configures tobj,
    creates entity, spins)
  - Main thread creates socket client transport, creates subscriber container,
    invokes remote `subscribe_interest`, spins, and asserts receipt of
    entity update frames
  - Validates the full socket transport round-trip without requiring Ada

- [ ] This serves as both a standalone test and a reference implementation for
  the Ada client

### Files touched

- `tests/tactical_objects/Test_TacticalObjects_E2E.cpp` (extend)
- `tests/CMakeLists.txt` (add `pcl_transport_socket` to link)

---

## File Summary

### New files

| File | Language | Purpose |
|------|----------|---------|
| `examples/ada/streaming_codec.ads` | Ada | Binary batch frame decoder (spec) |
| `examples/ada/streaming_codec.adb` | Ada | Binary batch frame decoder (body) |
| `examples/ada/ada_tobj_client.adb` | Ada | Client executable |
| `examples/ada/ada_tobj_client.gpr` | Ada | GNAT project file |
| `tests/tactical_objects/Test_TacticalObjects_SocketE2E.cpp` | C++ | Server harness |
| `scripts/test_ada_socket_e2e.sh` | Shell | Cross-process test driver |

### Modified files

| File | Change |
|------|--------|
| `examples/ada/pcl_bindings.ads` | Add socket transport + invoke_service bindings |
| `tests/tactical_objects/Test_TacticalObjects_E2E.cpp` | Add C++ socket transport test case |
| `tests/CMakeLists.txt` | Add socket E2E test target, optional Ada test |

---

## Dependency Graph

```
Step 1 (Ada socket bindings)
  │
  ├──> Step 2 (Ada streaming codec)
  │      │
  │      └──> Step 3 (Ada client executable)
  │             │
  │             └──> Step 5 (Cross-process driver script)
  │
  └──> Step 4 (C++ server harness)
         │
         ├──> Step 5 (Cross-process driver script)
         │
         └──> Step 6 (C++ only socket test — no Ada dependency)
```

Steps 1+4 can proceed in parallel.  Step 6 is independent of Ada and can
start immediately.

## Implementation Order (Recommended)

1. **Step 6** — C++ socket transport test (validates transport layer, no Ada needed)
2. **Step 1** — Ada FFI binding extensions
3. **Step 2** — Ada streaming codec
4. **Step 4** — C++ server harness
5. **Step 3** — Ada client executable
6. **Step 5** — Cross-process test driver

## Risks and Mitigations

| Risk | Mitigation |
|------|------------|
| GNAT not available in CI | Step 6 provides C++-only validation; Ada steps gated behind `find_program(gprbuild)` |
| Port conflicts in CI | Use ephemeral ports (port=0) with port-file discovery |
| `accept()` blocks indefinitely | Add timeout via `select()`/`poll()` with 10s deadline |
| Ada task model vs PCL threading | Ada client runs single-threaded; socket recv thread is spawned by C code (transparent to Ada) |
| Wire format drift | Add a version byte to batch header in future; current format is Step 2 stable |
| Windows GNAT uses MinGW (incompatible with MSVC libs) | Ada test is POSIX-first; Windows support requires separate GNAT build of PCL (already done in existing `.gpr` files) |
