# Tactical Objects Backend And Transport Status

## Purpose

This is the single current-status document for Tactical Objects as the
proving-ground component for PYRAMID's proto-native binding, codec, and
transport model.

If a roadmap or generator note disagrees with this page, this page wins for
current implementation status.

## Canonical Scope

This page answers:

- what Tactical Objects currently proves
- which backend and transport paths are implemented
- which proofs are covered by standalone tests today
- what remains partial or still planned

It does not redefine the contract. The contract remains:

1. MBSE / SysML semantics
2. generated `.proto`
3. generated bindings, codecs, and transport projections

No Tactical Objects-specific schema should be introduced below the generated
`.proto` layer.

## Related Docs

The doc split is now:

- this page: current Tactical Objects status and proving-ground roadmap
- `service_binding_codegen.md`: generator architecture and implementation
  reference
- `ros2_transport_semantics.md`: canonical ROS2 topic/service/stream mapping
  rules
- `standard_alignment_plan.md`: legacy standard-bridge alignment context only

The older overlapping plan docs have been reduced to redirect notes so there is
one current status page rather than several drifting ones.

## Contract And Model Rules

Tactical Objects continues to prove the following rules:

- handlers operate on one proto-native typed surface
- codec choice must not change handler signatures
- transport choice must not change handler signatures
- generated transports may project the same handler contract in different ways
- bridge-local payload models are not part of the core generation story

Externally, the canonical standard-topic payloads remain:

- `standard.entity_matches` -> `ObjectMatch[]`
- `standard.object_evidence` -> `ObjectDetail`
- `standard.evidence_requirements` -> `ObjectEvidenceRequirement[]`

## Current Status

### Service bindings

Current state:

- Ada and C++ Tactical Objects service bindings are proto-native
- generated service dispatch is the live runtime seam for backend selection
- the old Tactical Objects-local service `Json_Codec` / `Wire_Types` path is no
  longer the active model

### Codec backends

| Codec backend | Status | Notes |
|---------------|--------|-------|
| `json` | active | baseline typed path on PCL |
| `flatbuffers` | active | C++ native, Ada typed API with generated shim allowance |
| `protobuf` | active on PCL | runtime dispatch supports `application/protobuf` alongside JSON and FlatBuffers |

### Transport backends

| Transport backend | Status | Notes |
|-------------------|--------|-------|
| `pcl` | active baseline | current production proving path |
| `grpc` | active projection, optional build | generated C++ transport reuses Tactical Objects `dispatch(...)` over protobuf |
| `shared_memory` | PCL foundation active | inter-process central named-bus transport exists at the PCL layer with standalone coverage; Tactical Objects-specific projection is still next |
| `ros2` | runtime binding active | generated Tactical Objects ROS2 transport projection, generic ROS2 envelope package, and standalone `rclcpp` runtime proof |

### Ada status

Current Ada policy is:

- public Ada APIs remain typed and proto-native
- JSON is a straightforward Ada path
- FlatBuffers, Protobuf, and gRPC may use generated C/C++ shims internally
- this is an implementation detail, not a contract exception

## Implemented Proofs

### PCL + Protobuf

Implemented and passing:

- generated protobuf codec support for Tactical Objects payloads
- PCL runtime dispatch using `application/protobuf`
- standalone protobuf dispatch and round-trip tests for generated bindings

Relevant tests:

- `test_pcl_proto_bindings`
- `test_codec_dispatch_e2e`

### gRPC + C++

Implemented and passing:

- generated provided/consumed Tactical Objects gRPC transport projection
- optional build through local `gRPC` / `grpc_cpp_plugin`
- standalone C++ unary round-trip proof through generated gRPC transport
- gRPC worker threads no longer call Tactical Objects business logic directly;
  generated transport ingress posts requests onto the real PCL executor via
  `pcl_executor_post_service_request(...)`, so handler execution stays on the
  executor thread rather than on transport-owned threads
- the standalone smoke test now asserts executor-thread execution for the
  service handler path

Relevant test:

- `test_grpc_transport_smoke`

### gRPC + Ada to C++

Implemented and passing on Windows:

- generated Ada protobuf and gRPC specs build successfully
- Ada can drive a real standalone C++ Tactical Objects gRPC server process
  through the canonical generated/canonical gRPC C ABI symbol surface
- the gRPC call path uses the shared `pyramid_grpc_c_shim` library under
  `examples/grpc/cpp`, not a test-local shim
- the current gRPC interop test now sends a mid-complexity
  `ObjectInterestRequirement` request over protobuf rather than a policy-only
  probe

Relevant tests:

- `ada_generated_bindings_roundtrip`
- `tobj_ada_grpc_cpp_interop_e2e`

### Master Cross-Language Conformance

Implemented and passing on Windows:

- one master conformance driver now exercises all currently available
  Tactical Objects Ada/C++ transport and codec combinations
- the matrix currently covered is:
  - socket + JSON
  - socket + FlatBuffers
  - gRPC + Protobuf
- shared-memory is intentionally not in the master matrix yet because the
  Tactical Objects-specific projection is not implemented yet at that layer
- socket + Protobuf is not in the master matrix because there is not yet a
  standalone Tactical Objects socket/protobuf bridge path

Relevant test:

- `tobj_master_conformance_e2e`

### PCL + Shared Memory Bus

Implemented and passing:

- pluggable shared-memory-style PCL transport with a central named bus
- bus fan-out pub/sub between multiple participants
- async remote service routing with deferred response support
- peer-filter enforcement on remote subscribers

Relevant test:

- `test_pcl_shared_memory_transport`

### ROS2 Runtime Binding

Implemented and passing:

- generated Tactical Objects ROS2 transport projection for provided and
  consumed service packages
- generic ROS2 runtime package `pyramid_ros2_transport` for envelope/service
  bindings
- canonical ROS2 mapping for:
  - topics: `/pyramid/topic/...`
  - unary services: `/pyramid/service/...`
  - streaming services: `/pyramid/stream/.../{open,frames,cancel}`
- standalone fake-adapter proof for ROS2 topic ingress, unary service ingress,
  and streaming service ingress
- standalone `rclcpp` runtime proof for ROS2 topic ingress, unary service
  ingress, streaming service ingress, and outbound PCL publish to ROS2 topic
- ROS2 ingress threads do not call Tactical Objects business logic directly;
  the shared support layer hands off onto the PCL executor before subscriber or
  service handler execution

Relevant test:

- `test_ros2_transport_semantics`
- `test_rclcpp_runtime_adapter`

## Important Boundary

The new Protobuf and gRPC proving path does not depend on the standard bridge.

That means:

- Protobuf and gRPC coverage is now validated through standalone generated
  binding and transport tests
- the standard bridge remains separate legacy context
- new codec and transport work should not be forced through bridge-specific
  flows unless that is the explicit target under test

## Current Gaps

The following areas are still incomplete:

- Tactical Objects shared-memory transport projection
- socket/protobuf cross-process Tactical Objects projection
- Ada ROS2 runtime beyond generated endpoint constants/specs
- ROS2 action mapping for long-running / feedback-style workflows
- fully Ada-native gRPC runtime without generated C/C++ shim support
- generalization of the same backend/transport proof level to other PYRAMID
  components

## Milestone Order

The current Tactical Objects proving order is:

1. keep JSON stable as the baseline typed path
2. keep FlatBuffers stable and complete
3. keep Protobuf stable on PCL
4. keep gRPC projection buildable and exercised by standalone tests
5. keep ROS2 runtime binding stable and extend it beyond the current envelope
   service/topic model where needed
6. layer Tactical Objects shared-memory projection onto the new PCL bus
   transport
7. generalize the same machinery beyond Tactical Objects

## Review Checklist

Use this table in reviews:

| Capability | Status |
|------------|--------|
| Proto-native Ada bindings | yes |
| Proto-native C++ bindings | yes |
| JSON runtime selection on PCL | yes |
| FlatBuffers runtime selection on PCL | yes |
| Protobuf runtime selection on PCL | yes |
| Standalone protobuf binding tests | yes |
| C++ gRPC transport smoke test | yes |
| Ada generated protobuf/grpc spec build proof | yes |
| Ada to C++ gRPC interop proof | yes, Windows |
| Master Ada/C++ transport/codec conformance matrix | yes, Windows |
| PCL shared-memory bus transport | yes |
| ROS2 semantic transport projection | yes |
| ROS2 `rclcpp` runtime binding | yes, Windows |
| shared-memory Tactical path | foundation only |

## What To Avoid

Do not regress Tactical Objects into:

- a Tactical Objects-only schema below `.proto`
- transport-specific handler interfaces
- codec-specific handler interfaces
- bridge-local payload types becoming the contract source of truth
- reintroducing the standard bridge as a requirement for protobuf/grpc proving
- transport-owned threads invoking business logic directly instead of handing
  work off to the PCL executor thread
