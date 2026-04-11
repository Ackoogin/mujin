# Service Binding Code Generation

## Overview

PYRAMID service binding generation is proto-native.

The source-of-truth layering is:

1. MBSE / SysML model
2. generated `.proto`
3. generated language bindings, codec backends, and transport backends

For all tooling below the MBSE extraction step, generated `.proto` is the
only canonical contract artifact.

## Current Generator Entry Point

The unified entry point is:

- [generate_bindings.py](/D:/Dev/repo/mujin/subprojects/PYRAMID/pim/generate_bindings.py)

It orchestrates:

- proto-native Ada bindings
- proto-native C++ bindings
- codec backend generation
- transport backend generation

The backend registry lives under:

- [codec_backends.py](/D:/Dev/repo/mujin/subprojects/PYRAMID/pim/codec_backends.py)
- [backends/](/D:/Dev/repo/mujin/subprojects/PYRAMID/pim/backends)

## Standard Terms

This document uses the same terms as the roadmap:

- `codec backend`: payload encoding choice
- `transport backend`: delivery mechanism
- `proto-native typed surface`: the public generated API
- `Ada shim`: an acceptable internal implementation detail for non-JSON
  backends

## Current Pipeline

```text
MBSE model
  -> proto_generator.py
  -> generated .proto
  -> generate_bindings.py
       -> ada_codegen.py
       -> cpp_codegen.py
       -> codec/transport backends
```

## What Is Generated Today

From the proto contract, the active generation flow produces:

- Ada data-model types
- Ada proto-native JSON codecs for generated types
- Ada service bindings
- Ada FlatBuffers codec packages
- Ada gRPC transport import specs over canonical C ABI entry points
- C++ data-model types
- C++ proto-native JSON codecs for generated types
- C++ service bindings
- C++ FlatBuffers codecs and `.fbs` schemas
- C++ gRPC transport projections
- canonical C ABI gRPC shim libraries for Ada interop

Tactical Objects is still the main proving-ground path, but current backend and
transport status now lives in:

- [service_schema_tactical_objects.md](/D:/Dev/repo/mujin/subprojects/PYRAMID/docs/service_schema_tactical_objects.md)

This page stays focused on generator architecture and emitted artifacts so we do
not maintain a second drifting status table here. That status page now also
tracks the live Ada/C++ master conformance matrix for the currently implemented
transport and codec combinations.

## JSON Model

JSON support now means:

- serialize proto-native generated types
- deserialize proto-native generated types
- keep the same logical payload shape as the other codecs

There is no longer a Tactical Objects service-local `Json_Codec` or
`Wire_Types` layer in the active path.

In practice, JSON currently uses the proto-derived type codec packages:

- Ada:
  - [pyramid-data_model-common-types_codec.ads](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/ada/generated/pyramid-data_model-common-types_codec.ads)
  - [pyramid-data_model-tactical-types_codec.ads](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/ada/generated/pyramid-data_model-tactical-types_codec.ads)
- C++:
  - [pyramid_data_model_common_codec.hpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/cpp/generated/pyramid_data_model_common_codec.hpp)
  - [pyramid_data_model_tactical_codec.hpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/cpp/generated/pyramid_data_model_tactical_codec.hpp)

The design direction is to keep JSON as a first-class backend within the same
backend model as FlatBuffers and Protobuf, while preserving this proto-native
typed surface.

## FlatBuffers Model

FlatBuffers support is also generated from proto.

Current Tactical Objects status:

- C++ uses generated `.fbs` plus `flatc`
- the generated C++ API is typed and proto-native
- Ada exposes typed and proto-native APIs
- the current Ada implementation may use a generated shim internally
- an internal JSON bridge in that Ada shim is currently acceptable

The main backend implementation lives in:

- [flatbuffers_backend.py](/D:/Dev/repo/mujin/subprojects/PYRAMID/pim/backends/flatbuffers_backend.py)

## Protobuf Model

Protobuf is generated through the same registry/backend model as the other
codecs. For current Tactical Objects proof status, use the Tactical Objects
status page rather than this generator reference.

## Transport Model

The generated service handler contract is intended to be transport-agnostic.

The active transport baseline is:

- `pcl`

The planned transport projections are:

- `grpc`
- `shared_memory`
- later `ros2`

Transport code owns:

- endpoint binding
- routing
- framing
- delivery semantics
- thread handoff from external I/O threads to the PCL executor-owned
  business-logic context

Transport code does not own:

- the payload model
- handler signatures
- backend-specific type systems

Transport projections must preserve the PCL threading rule: external transport
threads do wire work only, and business logic is invoked only on the executor
thread selected by the binding. For the current Tactical Objects gRPC path,
ingress hands off through `pcl_executor_post_service_request(...)`; the
transport does not own a parallel business-logic thread.

## Runtime Selection Rules

### Codec selection

- selected by `content_type`
- the generated dispatch layer decodes bytes to proto-native types
- handlers operate only on typed values
- responses are encoded using the selected codec backend

### Transport selection

- selected by generated transport binding
- handlers must not branch on transport choice

## Ada Policy

The current Ada policy is:

- the public API must always be typed and proto-native
- JSON is expected to remain native at the Ada layer
- FlatBuffers and Protobuf may use generated C/C++ shims internally
- gRPC Ada support currently uses generated canonical C ABI shims under
  `examples/grpc/cpp`
- shared-memory Ada support may also use generated shims initially

This policy is a short-term implementation choice, not a change to the public
contract.

## Tactical Objects Output Reference

The most important active Tactical Objects generated outputs include:

- [pyramid-services-tactical_objects-provided.ads](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/ada/generated/pyramid-services-tactical_objects-provided.ads)
- [pyramid-services-tactical_objects-consumed.ads](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/ada/generated/pyramid-services-tactical_objects-consumed.ads)
- [pyramid-services-tactical_objects-flatbuffers_codec.ads](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/ada/generated/flatbuffers/ada/pyramid-services-tactical_objects-flatbuffers_codec.ads)
- [pyramid-components-tactical_objects-services-provided-grpc_transport.ads](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/ada/generated/grpc/ada/pyramid-components-tactical_objects-services-provided-grpc_transport.ads)
- [pyramid-data_model-tactical-protobuf_codec.ads](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/ada/generated/protobuf/ada/pyramid-data_model-tactical-protobuf_codec.ads)
- [pyramid_services_tactical_objects_provided.hpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/cpp/generated/pyramid_services_tactical_objects_provided.hpp)
- [pyramid_services_tactical_objects_consumed.hpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/cpp/generated/pyramid_services_tactical_objects_consumed.hpp)
- [pyramid_services_tactical_objects_flatbuffers_codec.hpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/cpp/generated/flatbuffers/cpp/pyramid_services_tactical_objects_flatbuffers_codec.hpp)
- [pyramid_services_tactical_objects_protobuf_codec.hpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/protobuf/cpp/pyramid_services_tactical_objects_protobuf_codec.hpp)
- [pyramid_components_tactical_objects_services_provided_grpc_transport.hpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/grpc/cpp/pyramid_components_tactical_objects_services_provided_grpc_transport.hpp)
- [pyramid_components_tactical_objects_services_provided_grpc_c_api.hpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/grpc/cpp/pyramid_components_tactical_objects_services_provided_grpc_c_api.hpp)
- [pyramid_components_tactical_objects_services_consumed_grpc_c_api.hpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/grpc/cpp/pyramid_components_tactical_objects_services_consumed_grpc_c_api.hpp)
- [pyramid_services_tactical_objects_grpc_c_api_support.hpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/grpc/cpp/pyramid_services_tactical_objects_grpc_c_api_support.hpp)

## Near-Term Standardization Work

The next generator changes should focus on:

1. making backend metadata explicit in the registry
2. treating JSON as a fully standardized backend within that registry model
3. reducing remaining Ada filename/package compatibility glue in generated
   non-JSON backends
4. adding shared-memory transport without changing the proto-native typed
   surface
