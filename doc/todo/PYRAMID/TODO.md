# PYRAMID — Tracked Follow-ups

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

Status: open. Raised 2026-06-26 after reviewing the gRPC plugin target.

### Current state

There are two different gRPC-related artifacts, and they should not be treated
as equivalent:

- `pyramid_grpc_transport` is a generated/static C++ transport library. It is
  functional on Linux: `test_grpc_transport_smoke` performs a real unary gRPC
  round trip, and `BindingPerformanceTest.Grpc_Tcp` exercises the same direct
  transport path.
- `pyramid_grpc_coupled_plugin` is a loadable `.so` that exposes both
  `pcl_transport_plugin_entry` and `pcl_codec_plugin_entry` under
  `application/grpc`. Today it is only a structural ABI proof: its codec and
  transport operations return `PCL_ERR_NOT_FOUND` instead of encoding,
  decoding, starting a server, publishing, or invoking an RPC.

`test_grpc_coupled_plugin_load` therefore proves only that the plugin can be
loaded and registered. It does **not** prove that loading the plugin through
`pcl_plugin_load_transport` / `pcl_plugin_load_codec` gives an application a
working gRPC runtime.

### What needs to be done

1. Define the plugin config contract for gRPC, analogous to socket/shm
   `config_json`. At minimum this needs role, address/host/port, and the
   executor pointer; it also needs a lifecycle model for server startup and
   shutdown.
2. Implement a real `pcl_transport_t` in `pyramid_grpc_coupled_plugin` by
   adapting the generated `pyramid_grpc_transport` server/client helpers:
   `serve`/server startup for provided endpoints, `invoke_async` for consumed
   endpoints, and proper shutdown/free hooks.
3. Decide whether the plugin's `application/grpc` codec vtable should be a
   thin pass-through for already-serialized protobuf messages or should expose
   real schema-based encode/decode. If it is pass-through only, document that
   generated gRPC transport owns protobuf serialization and that generic codec
   registry users should not expect `application/grpc` to behave like JSON or
   FlatBuffers.
4. Update app/client plugin loading to support gRPC-specific lifecycle symbols
   instead of assuming socket gateway/destroy symbols.
5. Add runtime tests that fail against the current stub:
   load `libpyramid_grpc_coupled_plugin.so` through the PCL plugin loader, set
   it on an executor, issue a unary request through the generated typed facade,
   and assert that the handler is called and the response decodes correctly.

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
