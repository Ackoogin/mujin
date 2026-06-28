# PYRAMID Transport & Codec Plugins — Capability & Remaining Work

This is the single living doc for the PYRAMID transport/codec plugin work: what
the system can do today, and what is left. **How the plugin system works and how
to use it** lives in the architecture reference —
[`subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md)
(plugin model, capability model + matrix, config/deploy/author) and
[`ros2_transport_semantics.md`](../../../subprojects/PYRAMID/doc/architecture/ros2_transport_semantics.md)
(ROS2 mapping). Last updated 2026-06-27.

---

## Current capability

**Plugin binding.** A component/client links only `pcl_core` + its generated
contract (typed facade + native↔C-ABI-struct marshalling). Codec and transport
are runtime-loaded `.so` plugins selected by an opaque `config_json`. With no
plugin registered, encode/decode/transport **fail closed** — in both C++ and Ada,
including Ada scalar aliases. One cross-language codec `.so` serves both languages
over the frozen `pyramid_<T>_c` boundary.

**Codecs (runtime plugins).** `application/json`, `application/flatbuffers`, and
`application/protobuf` (per-component registry codec plugins, e.g.
`pyramid_codec_protobuf_tactical_objects`).

**Transports.**
- Generic decoupled: **socket**, **shared memory**, **udp**.
- **gRPC coupled** plugin — both directions (server ingress via the proto-driven
  aggregator; client `invoke_async` unary + `invoke_stream` server-streaming).
- **ROS2 coupled** plugin — both directions (pub/sub, consumed unary + streaming),
  process-safe rclcpp lifecycle, reproducible ament build, teardown-then-unload.

**Capability model (heterogeneous middleware).** Each plugin declares its
primitives (`PUBSUB`/`RPC_UNARY`/`RPC_STREAM`, + offered QoS); the framework
validates each endpoint↔transport route at compose time and **fails closed** with
a precise diagnostic. A line-based manifest routes endpoints across multiple
transport plugins (mixed middleware), with caps + QoS-floor validation and
unregister-before-unload lifecycle.

**Config convention.** Directional selector unified on `role: provided|consumed`
(`mode`/`server`/`client` kept as back-compat aliases).

**Native ROS2 IDL + marshalling.** `pim/ros2_idl_codegen.py` generates real ROS2
`.msg`/`.srv` from the `.proto` contracts (the `pyramid_msgs` ament package builds
them through rosidl); `pim/ros2_marshal_codegen.py` generates `pyramid_ros2_codec.hpp`
— `domain_model` ↔ `pyramid_msgs` converters + an `rclcpp` wire codec — all
auto-generated from the shared `ros2_ir` domain field model and **round-trip
verified** for every type (`test_ros2_codec_roundtrip`, all green on Humble). The
live ROS2 transport still carries the pass-through `PclEnvelope`; switching it to
the typed codec is the headline remaining item below.

---

## Remaining work

### 1. Put the typed ROS2 codec on the live wire (headline)

The native IDL types and the `domain_model`↔`pyramid_msgs` wire codec exist and
round-trip; what's left is making the live transport use them instead of the
pass-through envelope:

1. Register `application/ros2` as a real registry codec backed by `ros2_codec`
   (a `cpp_codegen._write_codec_plugin_impl` ros2 branch). The codec plugin must
   build **inside the ament package** (rclcpp only resolves there), unlike the
   json/fb/protobuf plugins built by the top-level CMake.
2. A typed `RclcppRuntimeAdapter` that publishes/serves the `pyramid_msgs`
   messages (e.g. `rclcpp::GenericPublisher`/`GenericSubscription` keyed by the
   `pyramid_msgs/...` type) so `ros2 topic echo` shows typed fields, replacing
   `PclEnvelope`/`PclService`/`PclOpenStream`.
3. Interop proof: a plain `rclcpp` node (no PYRAMID linkage) round-tripping the
   generated IDL against a PYRAMID component.

Keep the envelope path as the decoupled codec-over-ROS2 fallback; native typed is
the default.

### 2. ROS2 actions (`RPC_ACTION`)

First-class by decision (the reserved `ACTION` pattern, not composed from
`RPC_STREAM`). Work: an `action` contract construct → required cap `RPC_ACTION`,
generated `.action` IDL (depends on §1), `RclcppRuntimeAdapter` action
server/client, caps declaration. Mapped when a production user needs it.

### 3. Smaller / deferred

| Item | Notes |
|------|-------|
| Opt-in capability adapters (`PUBSUB over RPC_STREAM`, `RPC_UNARY over PUBSUB`) | Deferred — stay strictly fail-closed until a concrete need; each would advertise its derived capability behind explicit config. |
| Ada ROS2 runtime | Ada has generated ROS2 constants/specs only; no rclcpp-equivalent runtime. |
| Top-level (non-ament) ROS2 plugin target | The coupled ROS2 plugin builds via colcon inside the ament package (rclcpp discoverability), not the ament-free `pyramid_plugins` aggregate. |
| Stream cancel — direct coverage | Runtime cancel works; add a dedicated test with the first production ROS2 user. |
| AME contract canonicalization | AME doesn't yet expose its interface as canonical PYRAMID `.proto`, so it can't consume the generated ROS2 bindings directly. |
