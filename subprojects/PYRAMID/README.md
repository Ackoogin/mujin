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
| `pim/` | Binding/code generation tooling (plus the upstream SysML/MBSE import tools) |
| `plugins/` | Coupled gRPC/ROS2 transport plugin sources |
| `examples/` | Hand-written Ada and C++ example apps/support code |
| `tactical_objects/` | Tactical Objects runtime, component, app, and local README |
| `tests/` | Generated binding, transport, and Tactical Objects tests |
| `ros2/` | ROS2 transport adapter (ament package) |
| `ros2_msgs/` | Generated `pyramid_msgs` interface package build |
| `pyramid_bridge/` | Ada/C++ bridge applications |
| `sdk_template/` | Offline SDK project template (see `doc/architecture/sdk_packaging.md`) |
| `cmake/` | Manifest-driven binding-source CMake helpers and their tests |
| `scripts/` | Binding generation, plugin build, SDK packaging, coverage, and interop test helpers |
| `doc/architecture/` | Generated-binding, plugin, ROS2 transport, and responsibility docs |
| `doc/guides/` | Developer-facing guides (user guide entry point; pub/sub & the interaction facade) |
| `doc/requirements/` | PYRAMID-owned requirements |

## Documentation

| Document | Purpose |
|----------|---------|
| [doc/guides/pyramid_user_guide.md](doc/guides/pyramid_user_guide.md) | **The single user-guide entry point**: design intent, high-level architecture with diagrams, usage, and the full document map |
| [doc/guides/pubsub_interaction_guide.md](doc/guides/pubsub_interaction_guide.md) | Developer sub-guide to contract-driven pub/sub and the interaction facade: submit/transitions/publish APIs, RPC ↔ pub/sub realization choice per leg, routing manifests, worked examples |
| [doc/architecture/pcl_pyramid_binding_generation_overview.md](doc/architecture/pcl_pyramid_binding_generation_overview.md) | Broad engineer-facing overview of how PYRAMID generated bindings plug into the PCL runtime |
| [doc/architecture/generated_bindings.md](doc/architecture/generated_bindings.md) | Canonical v1 guide for proto schemas, generated bindings, codecs, transports, and how components should use them |
| [doc/architecture/build_artefacts.md](doc/architecture/build_artefacts.md) | **High-level map of every build artefact** (core static libs, marshal libs, plugin dynamic libs, executables) and every deployment configuration file, with diagrams |
| [doc/architecture/transport_codec_plugin_system.md](doc/architecture/transport_codec_plugin_system.md) | Runtime transport/codec **plugin** system: ABI, loader, config pass-through, build targets, and per-component deployment staging (with diagrams) |
| [doc/slides/pcl_pyramid_cal_overview.md](doc/slides/pcl_pyramid_cal_overview.md) | **Slide deck**: PCL/PYRAMID and the PYRAMID Critical Abstraction Layer (CAL) for peer engineers |
| [doc/architecture/ros2_transport_semantics.md](doc/architecture/ros2_transport_semantics.md) | ROS2 topic/service/stream mapping rules |
| [doc/architecture/PYRAMID_COMPONENT_RESPONSIBILITIES.md](doc/architecture/PYRAMID_COMPONENT_RESPONSIBILITIES.md) | Component responsibility map and standard responsibility IDs |
| [doc/architecture/tactical_objects/standard_alignment.md](doc/architecture/tactical_objects/standard_alignment.md) | Tactical Objects alignment with the generated PYRAMID proto interface |
| [split & PIM migration plan](../../doc/plans/PYRAMID/pyramid_split_and_tobj_pim_migration_plan.md) | Proposed capability/consumers subproject split, and the Tactical Objects gap analysis for moving onto the `pim/test` PIM Osprey port-grammar contract |
| [outstanding work](../../doc/todo/PYRAMID/TODO.md) | The single tracker for all remaining PCL/PYRAMID work |
| [tactical_objects/README.md](tactical_objects/README.md) | Tactical Objects runtime, app, and test usage |
