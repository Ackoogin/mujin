# PYRAMID — Tracked Follow-ups

## Transport capability model (heterogeneous middleware)

Status: proposed (planning). Raised 2026-06-26.

Middleware expose different primitives — gRPC is RPC-only, UDP/MQTT are
pub/sub-only, ROS2/socket/shm are mixed. We need a capability model so each
transport plugin **declares** what it supports (`PUBSUB` / `RPC_UNARY` /
`RPC_STREAM` / `RPC_ACTION` + QoS), the framework **validates** at compose time
that every contract endpoint's required primitive is carried by its routed
transport (fail closed, precise diagnostic), and deployments can **route different
endpoints over different transports**. Full design, per-middleware matrix, and
staged plan:
[`doc/plans/PYRAMID/transport_capability_model_plan.md`](../../plans/PYRAMID/transport_capability_model_plan.md).
This subsumes the per-plugin capability gaps noted below (e.g. ROS2 consumed
`invoke_async`, gRPC pub/sub absence).

## ROS2 coupled plugin (production completeness)

Status: open. Raised 2026-06-26 after reviewing the in-tree
`pyramid_ros2_coupled_plugin` work.

### Current state

The ROS2 coupled plugin is more complete than the gRPC coupled plugin:

- `pyramid_ros2_coupled_plugin` constructs a real rclcpp node, a
  `RclcppRuntimeAdapter`, and a background spin thread.
- Its transport vtable implements `publish`, `subscribe`, and `shutdown`, and
  plugin-private symbols expose topic binding, unary-service advertising,
  stream-service advertising, and destroy.
- Its `application/ros2` codec is a pass-through envelope codec for already
  encoded payload bytes.
- `test_rclcpp_runtime_adapter` proves live ROS2 topic ingress, unary service
  ingress, stream-frame publication, and outbound topic publication through the
  adapter library.
- `test_ros2_coupled_plugin_load` proves that the `.so` loads as both a
  transport and codec plugin and that the pass-through codec copies payload
  bytes correctly.

### Gaps found in review

1. **Fresh-tree ament build is not reproducible from the current source paths.**
   `subprojects/PYRAMID/ros2/CMakeLists.txt` expects generated ROS2 support
   sources under
   `subprojects/PYRAMID/bindings/cpp/generated/ros2/cpp`, but that directory is
   absent in this checkout. A colcon build with ROS2 Humble reached CMake
   generation and failed on missing
   `pyramid_ros2_transport_support.cpp`.
2. **The plugin-loaded path is not live-traffic tested.**
   `test_ros2_coupled_plugin_load` loads the plugin, resolves symbols, and
   checks codec encode/decode, but it does not call the plugin bind/advertise
   symbols with valid arguments or assert that traffic crosses ROS2 through the
   plugin-loaded transport instance.
3. **Client-side RPC is not implemented in the transport vtable.**
   The plugin leaves `invoke_async`, `invoke_stream`, `respond`, stream send/end,
   and cancel unset. That is acceptable for server-side ingress only, but it
   means generated consumed-service calls routed through
   `pcl_executor_invoke_async` / `pcl_executor_invoke_stream` will not work over
   `application/ros2` via the generic PCL transport path.
4. **rclcpp ownership is process-global but tracked per plugin instance.**
   The first plugin instance that calls `rclcpp::init` owns shutdown. If multiple
   plugin instances are active, destroying the first owner can call
   `rclcpp::shutdown()` while later instances still depend on rclcpp.
5. **Destroy is a private symbol, not enforced by the generic loader.**
   A caller that loads the transport and then only calls `pcl_plugin_unload`
   leaks the context and leaves the spin thread lifecycle undefined. Current
   tests call `pcl_ros2_transport_plugin_destroy` explicitly, but the generic
   loader API cannot enforce that pattern.
6. **Deployment staging copies only the plugin `.so`.**
   The staged deployment needs a documented/runtime-checked assumption about
   ROS2 installation and generated message/type-support shared libraries being
   discoverable at run time; otherwise a dlopen-only client may still fail
   outside a sourced ROS2/colcon environment.

### What needs to be done

1. Make the ROS2 ament package consume generated support files from a
   reproducible location: either commit the generated `ros2/cpp` support tree,
   generate it as part of `build_ros2_transport`, or pass the top-level build's
   `${PYRAMID_CPP_BINDINGS_DIR}` into the ament package.
2. Add a plugin-loaded live E2E test:
   load `libpyramid_ros2_coupled_plugin.so`, call
   `pcl_ros2_transport_plugin_bind_topic` / `advertise_unary` /
   `advertise_stream` with valid names, publish/call from a separate rclcpp
   client node, and assert the PCL handler runs on the executor thread and the
   response/frame payloads are correct.
3. Decide the intended scope for ROS2 consumed/client endpoints. If ROS2 should
   support generated consumed-service calls, implement `invoke_async` and
   `invoke_stream` in the plugin transport vtable; otherwise document
   `application/ros2` as server-ingress/pub-sub only for now and add tests that
   fail closed with `PCL_ERR_NOT_FOUND`.
4. Replace per-instance `owns_rclcpp` shutdown with a process-safe lifecycle
   policy: shared reference counting, an externally owned context mode, or a
   documented rule that the plugin never shuts down rclcpp if it did not create
   the entire ROS2 process context.
5. Add a generic plugin lifecycle helper or wrapper so applications cannot unload
   `pyramid_ros2_coupled_plugin` without first destroying the transport context.
6. Extend staging docs/scripts to include the required ROS2 runtime environment:
   sourced setup files, type-support library paths, and any package install
   directories needed for `dlopen` and message/service creation.

## gRPC coupled plugin (runtime plugin functionality)

Status: largely closed. Raised 2026-06-26; server-ingress runtime + live E2E
landed 2026-06-26. Remaining: client-side `invoke_async` and a Linux Ada gRPC
interop harness (see "Remaining" below).

### Current state

`pyramid_grpc_coupled_plugin` is now a real, server-ingress gRPC runtime, not a
structural stub:

- On `pcl_transport_plugin_entry` it reads a config contract
  (`executor`, `component`, `role`, `address`) and starts a real gRPC server for
  the configured component via a new **proto-driven aggregator**
  (`pyramid_grpc_plugin_aggregator.{hpp,cpp}`, generated by
  `grpc_backend.py`). The aggregator exposes a stable C ABI
  (`pyramid_grpc_plugin_server_start/stop`) and registers every configured
  component's generated `*ServiceImpl` onto one `grpc::Server`, so the set of
  servable `(component, role)` pairs is driven by the `.proto` data model with
  no hand-maintained dispatch.
- Inbound RPCs are dispatched to the PCL executor (handlers run on the executor
  thread), exactly like the static `buildGrpcServer` path.
- The `application/grpc` codec vtable is a **pass-through envelope codec** (copy
  pre-serialized protobuf bytes), mirroring the ROS2 coupled plugin. Generated
  gRPC transport owns protobuf serialization; generic codec-registry users must
  not expect `application/grpc` to behave like the JSON/FlatBuffers schema codecs.
- Lifecycle: `shutdown()` (vtable) stops the server; a private
  `pcl_grpc_transport_plugin_destroy` symbol stops the server and frees the
  context. With an empty/executor-less config the entry point fails closed
  (loader returns `PCL_ERR_STATE`).

`test_grpc_coupled_plugin_e2e` loads `libpyramid_grpc_coupled_plugin.so` through
`pcl_plugin_load_transport`, drives a real unary RPC from a separate gRPC client
stub, and asserts the PCL handler runs on the executor thread and the response
decodes — proving the plugin-loaded path gives a working gRPC runtime, usable by
any gRPC client (C++ proven here; Ada is a gRPC client like any other).
`test_grpc_coupled_plugin_load` now asserts the structural ABI plus fail-closed
behaviour. `test_grpc_transport_smoke` and `BindingPerformanceTest.Grpc_Tcp`
continue to cover the static transport path. All pass under `build-grpc`.

### Done

1. Config contract defined (`executor`/`component`/`role`/`address`) with a
   start-on-entry / shutdown+destroy lifecycle.
2. Real `pcl_transport_t` server startup for provided/consumed endpoints via the
   generated aggregator, plus shutdown/free hooks.
3. Codec vtable decided and documented as pass-through (above).
4. gRPC-specific lifecycle symbol (`pcl_grpc_transport_plugin_destroy`) added;
   the E2E test resolves and calls it via `pcl_plugin_symbol`.
5. Plugin-loaded live runtime test added (`test_grpc_coupled_plugin_e2e`).

6. **Client (consumed) side implemented** — the plugin completes the transport
   contract both ways. In `mode:"client"` it dials a remote endpoint and
   implements `invoke_async` (unary) and `invoke_stream` (server-streaming) by
   routing a component's consumed-service calls through the generated typed gRPC
   stubs (a `service_name -> stub` dispatch in the aggregator).
   `test_grpc_coupled_plugin_e2e.ConsumedUnaryThroughLoadedPlugin` drives a
   client-mode plugin against a server-mode plugin (PCL ↔ plugin ↔ plugin ↔ PCL).

### Remaining

- **Coupled codec should own protobuf marshalling → retire the Ada gRPC shim.**
  Decision (2026-06-26): retire the shim fully. The `application/grpc` codec is
  currently pass-through (expects already-serialized protobuf bytes), which suits
  C++ (direct protobuf codec) but means Ada still consumes gRPC via the bespoke
  JSON shim (`pyramid_grpc_c_shim`). C-ABI marshalling alone is not sufficient —
  it yields the frozen `pyramid_<T>_c` struct, not protobuf wire bytes. Make the
  registry own struct↔protobuf so Ada (and C++) consume gRPC identically to
  socket/shm with no shim and no JSON detour. Staged plan:

  1. **Protobuf codec backend** (`pim/cpp_codegen.py` `_write_codec_plugins` /
     `_write_codec_plugin_impl`): emit a per-component `application/protobuf`
     codec plugin (`pyramid_codec_protobuf_<component>`) mirroring the
     json/flatbuffers ones. Messages → `data_model::<mod>::protobuf_codec`
     `toBinary`/`fromBinary<Short>`. **Aliases** (e.g. `Identifier`, a collapsed
     `std::string`) have no `toBinary`; the plugin must build the `.pb` wrapper
     message and set its single field, so the generator needs the alias field
     name from the index. **Arrays** (streaming responses) have no protobuf array
     type; frame as concatenated varint-length-prefixed messages (matching the
     gRPC stream wire in `*_grpc_transport.cpp`).
  2. **CMake**: the protobuf codec-plugin targets must link `pyramid_protobuf_support`
     and include the protobuf gen dir (unlike json/flatbuffers).
  3. **Ada facade**: drop the `Grpc_Transport`/shim special-branch in
     `ada_codegen.py` consumed/provided invoke; route consumed gRPC through the
     standard PCL transport-plugin path (load the gRPC coupled plugin in client
     mode + the protobuf codec), exactly like socket/shm.
  4. **Remove** the `pyramid_grpc_c_shim` build + generated `*_grpc_c_api*` and
     Ada `*-grpc_transport.*` sources once nothing references them.
  5. **Linux Ada gRPC e2e**: Ada client consumes via plugin (no shim) against a
     plugin-hosted server; `.sh` + ctest, un-gated from WIN32.

  See [`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md).
- **Linux Ada gRPC interop harness**: `ada_grpc_cpp_interop_e2e` is currently
  Windows-only (`.bat`). The Linux blockers are now fixed at the generator level
  (the `To_Json` codec-source-free break and the Win32-only dynamic loader — now
  the portable `pcl_plugin_open`); a Linux `.sh` + ctest pointing an Ada client at
  the plugin-hosted server still needs to be wired.

## Protobuf codec plugin (generic transport coverage)

Status: open. Raised 2026-06-26 from the binding-performance regression check
([`doc/reports/PYRAMID/binding_performance_report_2026-04-28.md`](../../reports/PYRAMID/binding_performance_report_2026-04-28.md),
Linux Refresh section).

### Problem

The plugin-era generated C++ facade resolves codecs **only** through the codec
registry and **fails closed** with no static fallback. The registry is populated
by loading codec **plugins** (`.so`). Today only **json** and **flatbuffers**
codec plugins exist (`pyramid_codec_json_*`, `pyramid_codec_flatbuffers_*`).

There is **no `application/protobuf` codec plugin**. Consequences:

- Protobuf over the **generic transports** (local / shmem / socket), which go
  through the facade + registry, has no codec to resolve → encode/decode produce
  nothing.
- Protobuf is still fully functional via the **direct** codec
  (`*_protobuf_codec.hpp`, used by the codec microbenchmark) and via the
  **direct generated gRPC transport** (`pyramid_grpc_transport`), which
  serialises protobuf directly rather than through the generic registry.

This is a **coverage gap**, not a performance regression — the C-ABI marshalling
boundary added no measurable overhead (see the report).

### Done so far

- `test_binding_performance` now **skips** the protobuf transport rows honestly
  (`GTEST_SKIP` + a `(skipped)` summary entry) when no `application/protobuf`
  codec is registered, instead of masking a hard `0/N` `EXPECT_EQ` failure.
  See `skipIfNoProtobufCodec` in
  [`subprojects/PYRAMID/tests/test_binding_performance.cpp`](../../../subprojects/PYRAMID/tests/test_binding_performance.cpp).

### Options to close

1. **Add an `application/protobuf` codec plugin** per component (mirror the json /
   flatbuffers plugin generation: `pyramid_codec_protobuf_<component>`), so the
   generic transports can carry protobuf via the registry like the other codecs.
   This is the real fix and restores the Windows-baseline protobuf transport rows.
2. If protobuf is intended to be **gRPC-coupled only** (not a standalone wire
   codec for socket/shmem), document that decision and keep the benchmark skip as
   the permanent, intended behaviour.

## Ada facade strictly codec-source-free (committed bindings = dist)

Status: done. Raised 2026-06-26. Closed 2026-06-26.

The committed `bindings/ada/generated` tree was trimmed to Ada `.ads/.adb` only
(removed stale C++ `.cpp/.hpp/.fbs` cruft). To make the committed tree match a
**plugin-only Ada dist**, the in-tree codec sources (json `*-types_codec.adb`,
`flatbuffers/ada/*`) should also drop out — but the Ada facade still compiles and
references them: post-W4 it fails closed via `Require_Codec`, yet `ada_codegen.py`
still emits `with …Types_Codec` clauses and the `From_Json` /
`Flatbuffers_Codec.From_Binary_*` fallback branches (dead when a codec plugin is
registered). C++ is already registry-only (no compiled-in codec fallback).

Closed by making `ada_codegen.py` emit registry-only service facades, routing
array schemas through the C-ABI codec registry, and removing the committed
json/flatbuffers Ada codec sources from the dist tree.

### Related direction

Fits the broader goal of making proto→binding→**plugin** a separate,
CI/CD-controllable generation step and dropping committed C++ bindings in favour
of build-time generation (see
[`doc/plans/PYRAMID/plugin_binding_v1_plan.md`](../../plans/PYRAMID/plugin_binding_v1_plan.md)).
