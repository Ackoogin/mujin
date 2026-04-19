# PYRAMID

`subprojects/PYRAMID` contains PYRAMID core, tactical-objects libraries, and PYRAMID-owned tests.

It depends on the `PCL` subproject and is laid out so it can later become its own repository with minimal path churn.

## Documentation

| Document | Purpose |
|----------|---------|
| [docs/generated_bindings.md](docs/generated_bindings.md) | Canonical v1 guide for proto schemas, generated bindings, codecs, transports, and how components should use them |
| [docs/generated_bindings_status.md](docs/generated_bindings_status.md) | Current Tactical Objects binding/conformance status, proof matrix, and remaining gaps |
| [docs/ros2_transport_semantics.md](docs/ros2_transport_semantics.md) | ROS2 topic/service/stream mapping rules |
| [tactical_objects/README.md](tactical_objects/README.md) | Tactical Objects runtime, app, and test usage |

Older binding plan documents are retained as redirects for existing links.
