# PYRAMID

`subprojects/PYRAMID` contains PYRAMID core, tactical-objects libraries, and PYRAMID-owned tests.

It depends on the `PCL` subproject and is laid out so it can later become its own repository with minimal path churn.

## What It Provides

- `pyramid_core`: shared runtime services such as UUIDs, logging, event support, and job helpers
- Generated C++/Ada service bindings from `.proto` contracts
- JSON, FlatBuffers, Protobuf, gRPC, and ROS2 transport/codegen support
- Tactical Objects component and standalone app
- Ada and C++ examples for generated service clients/providers

## Build And Test

From the workspace root:

```bat
cmake --preset default
cmake --build build --config Release --target pyramid_core tactical_objects tactical_objects_app
ctest --test-dir build --output-on-failure -C Release -R "Tactical|ProtoBindings|CodecDispatch|tobj_|Ros2TransportSemantics"
```

During CMake configure, PYRAMID generates a build-local C++ binding tree under
`${binaryDir}/generated/pyramid_cpp_bindings` by default. CMake globs that tree
for generated service facades, codecs, schemas, and transport projections, then
build targets refresh it through `pyramid_cpp_bindings_codegen` when proto or
generator inputs change.

For component repositories that receive the contract separately, set
`PYRAMID_GENERATE_CPP_BINDINGS=OFF` and point `PYRAMID_CPP_BINDINGS_DIR` at the
delivered generated C++ binding tree.

Regenerate bindings after changing proto contracts:

```bat
subprojects\PYRAMID\scripts\generate_bindings.bat
```

## Directory Map

| Path | Contents |
|------|----------|
| `core/` | PYRAMID shared runtime layer |
| `proto/` | Data-model and component service contracts |
| `pim/` | Binding/code generation tooling |
| `bindings/` | Checked-in generated C++/Ada bindings plus Protobuf, gRPC, and ROS2 projections |
| `examples/` | Hand-written Ada and C++ example apps/support code |
| `tactical_objects/` | Tactical Objects runtime, component, app, and local README |
| `tests/` | Generated binding, transport, and Tactical Objects tests |
| `ros2/` | ROS2 transport adapter package |
| `scripts/` | Binding generation, coverage, and interop test helpers |
| `doc/architecture/` | Generated-binding, ROS2 transport, and responsibility docs |
| `doc/requirements/` | PYRAMID-owned requirements |

## Documentation

| Document | Purpose |
|----------|---------|
| [doc/architecture/pcl_pyramid_binding_generation_overview.md](doc/architecture/pcl_pyramid_binding_generation_overview.md) | Broad engineer-facing overview of how PYRAMID generated bindings plug into the PCL runtime |
| [doc/architecture/generated_bindings.md](doc/architecture/generated_bindings.md) | Canonical v1 guide for proto schemas, generated bindings, codecs, transports, and how components should use them |
| [generated bindings status](../../doc/reports/PYRAMID/generated_bindings_status.md) | Current Tactical Objects binding/conformance status, proof matrix, and remaining gaps |
| [doc/architecture/ros2_transport_semantics.md](doc/architecture/ros2_transport_semantics.md) | ROS2 topic/service/stream mapping rules |
| [doc/architecture/PYRAMID_COMPONENT_RESPONSIBILITIES.md](doc/architecture/PYRAMID_COMPONENT_RESPONSIBILITIES.md) | Component responsibility map and standard responsibility IDs |
| [standard alignment plan](../../doc/plans/PYRAMID/standard_alignment_plan.md) | Tactical Objects alignment with the generated PYRAMID proto interface |
| [tactical_objects/README.md](tactical_objects/README.md) | Tactical Objects runtime, app, and test usage |
