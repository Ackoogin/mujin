# PYRAMID Transport & Codec Plugins — Capability & Remaining Work

This doc records the transport/codec plugin capability and the detail of its
remaining work; execution order and status across all PYRAMID work is tracked
in [`doc/todo/PYRAMID/TODO.md`](../../todo/PYRAMID/TODO.md). **How the plugin system works and how
to use it** lives in the architecture reference —
[`subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md)
(plugin model, capability model + matrix, config/deploy/author) and
[`ros2_transport_semantics.md`](../../../subprojects/PYRAMID/doc/architecture/ros2_transport_semantics.md)
(ROS2 mapping). Last updated 2026-07-06.

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
  Typed `pyramid_msgs` topic wire is the default; the opaque envelope wire
  stays selectable (`RclcppRuntimeAdapter::Options::use_envelope_wire`) and
  still carries array topics and unary/stream service framing.

**Capability model (heterogeneous middleware).** Each plugin declares its
primitives (`PUBSUB`/`RPC_UNARY`/`RPC_STREAM`, + offered QoS); the framework
validates each endpoint↔transport route at compose time and **fails closed** with
a precise diagnostic. A line-based manifest routes endpoints across multiple
transport plugins (mixed middleware), with caps + QoS-floor validation and
unregister-before-unload lifecycle.

**Config convention.** Directional selector unified on `role: provided|consumed`
(`mode`/`server`/`client` kept as back-compat aliases).

**Native ROS2 IDL + typed wire (default).** `pim/ros2_idl_codegen.py` generates
real ROS2 `.msg`/`.srv` from the `.proto` contracts (the `pyramid_msgs` ament
package builds them through rosidl); `pim/ros2_marshal_codegen.py` generates
`pyramid_ros2_codec.hpp` — `domain_model` ↔ `pyramid_msgs` converters + an
`rclcpp` wire codec — all auto-generated from the shared `ros2_ir` domain field
model and **round-trip verified** for every type (`test_ros2_codec_roundtrip`,
all green on Humble). Since 2026-07-04 (TODO items A1–A4) the typed codec is on
the **live wire**: `application/ros2` is a real registry codec (generated
`*_ros2_codec_plugin.cpp` via `pim/cpp/codec_plugin_gen.py`, built inside the
ament package), `RclcppRuntimeAdapter` publishes/subscribes typed `pyramid_msgs`
topic messages by default (envelope wire selectable via
`Options::use_envelope_wire`; array topics and unary/stream service framing
still use the envelope), and a plain-`rclcpp` interop test proves the topic
wire is standard typed ROS2 with no PYRAMID framing. See
`doc/todo/PYRAMID/TODO.md` WS-A for the execution record and
`ros2_transport_semantics.md` for the delivered semantics.

---

## Remaining work

### 1. ~~Put the typed ROS2 codec on the live wire~~ — DONE 2026-07-04

Delivered as TODO items A1–A4 (package-neutral marshal codegen, typed
`application/ros2` registry codec plugins in the ament build, typed
`RclcppRuntimeAdapter` default wire, plain-rclcpp interop proof; verified live
on Humble). The envelope path remains the decoupled codec-over-ROS2 fallback.
Detail now lives in `doc/todo/PYRAMID/TODO.md` WS-A; kept here only so old
references to "§1" resolve.

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
