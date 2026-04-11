# Proto-Native Backend And Transport Standardization Plan

## Purpose

This is the authoritative roadmap for PYRAMID binding generation below the
MBSE extraction step.

The source-of-truth layering is:

1. MBSE / SysML is the ultimate semantic source.
2. Generated `.proto` is the only downstream contract artifact.
3. All generated codecs and transports project that same contract.

No backend or bridge layer may redefine the payload model once `.proto` has
been produced.

## Core Terminology

- `codec backend`: payload encoding choice
- `transport backend`: delivery mechanism
- `proto-native typed surface`: the public generated API exposed to component
  code and client code
- `Ada shim`: an acceptable internal implementation detail for non-JSON
  backends where Ada delegates to generated C/C++ glue

## Standard Backend Contract

Every backend must comply with the same contract.

### Codec backends

Codec backends are currently:

- `json`
- `flatbuffers`
- `protobuf`

They must:

- generate C++ codec artifacts for all reachable proto payloads in scope
- generate Ada typed codec artifacts for the same payloads
- expose encode/decode by proto-native type, not by backend-local wire model
- plug into the same runtime dispatch model using `content_type`
- support service request/response payloads and canonical standard-topic
  payloads
- provide backend conformance tests and at least one cross-language proof

### Transport backends

Transport backends are currently planned as:

- `pcl`
- `grpc`
- `shared_memory`
- later `ros2`

They must:

- project the same proto-native handler contract
- never redefine payload types
- keep component logic transport-agnostic
- allow transport selection without changing handler signatures
- reuse codec backends unless the transport has a transport-mandated wire
  format

## Ada Policy

The short-term Ada policy is explicit:

- public Ada APIs are always typed and proto-native
- for non-JSON codecs and transports, Ada may be implemented by generated
  C/C++ shims
- an internal JSON bridge inside that shim is acceptable for now, provided the
  public Ada API remains typed and the selected on-wire codec remains
  proto-native

This policy is acceptable for:

- FlatBuffers
- Protobuf
- shared-memory transport if it reuses one of those codecs
- gRPC transport if Ada delegates to generated C++ helpers

## Target Architecture

```text
MBSE / SysML
  -> proto_generator.py
  -> generated .proto
  -> generate_bindings.py
       -> language bindings
       -> codec backends
       -> transport backends
```

The generated service handler contract is the stable middle layer:

```text
component logic
  -> proto-native typed surface
  -> codec backend selection
  -> transport backend selection
```

Handlers never branch on codec or transport.

## Backend Matrix

| Backend | Kind | Runtime selector | C++ strategy | Ada strategy | Status |
|--------|------|------------------|--------------|--------------|--------|
| `json` | codec | `content_type` | native | native | active baseline |
| `flatbuffers` | codec | `content_type` | native `flatc`-based | typed API + generated shim allowed | active, still tightening |
| `protobuf` | codec | `content_type` | native `protoc`-based | typed API + generated shim allowed | partial / not standard yet |
| `pcl` | transport | transport choice | native | native / generated binding | active baseline |
| `grpc` | transport | transport choice | native `protoc` + `grpc_cpp_plugin` | typed API + generated shim allowed | generated stubs only |
| `shared_memory` | transport | transport choice | native | typed API + generated shim allowed | planned |
| `ros2` | transport | transport choice | generated projection | optional later | planned |

## Shared Runtime Rules

### Codec selection

- codec selection is by `content_type`
- the generated dispatch layer decodes bytes into proto-native types before
  invoking handlers
- the same handler output is re-encoded using the selected codec backend

### Transport selection

- transport selection is by generated transport binding
- transport-specific code owns framing, routing, and delivery
- transport code never changes the logical payload model

### Failure behavior

- unsupported codec and transport combinations fail explicitly
- unsupported `content_type` values fail explicitly
- handlers should not be aware of the failure mode details beyond transport
  status and codec decode/encode errors

## Shared-Memory Transport Policy

`shared_memory` is a first-class transport backend, not a codec backend.

Its contract is:

- it carries the same proto-native request/response/topic payloads as other
  transports
- payload encoding is still selected by codec backend and `content_type`
- preferred codecs are:
  - `application/flatbuffers`
  - `application/protobuf`
- `application/json` is allowed for debug and compatibility, not for the
  performance-critical default path

The generated shared-memory frame/header format should include:

- endpoint or service name
- operation or channel identifier
- `content_type`
- payload size
- correlation or request id for RPC

The transport must support:

- request/reply RPC
- pub/sub topics

## Implementation Phases

### Phase 0: Terminology and generator contract

- make the backend registry aware of codec vs transport backends
- record backend metadata explicitly:
  - backend kind
  - implementation mode
  - required tools
  - runtime selector
- ensure docs use the same terms everywhere

### Phase 1: JSON as a standard backend

- make `json_backend.py` the canonical JSON backend surface
- keep JSON as the semantic reference backend
- eliminate any remaining “JSON is special because it lives outside the
  backend model” assumptions

Definition of done:

- JSON participates in the same registry/runtime selection model as the other
  codec backends
- Ada and C++ JSON APIs match the same typed surface shape as the other codecs

### Phase 2: FlatBuffers to standard

- keep C++ as the authoritative native FlatBuffers implementation using
  generated `.fbs` and `flatc`
- complete full service/topic coverage on the proto-native model
- keep Ada typed and proto-native at the public API
- allow the Ada implementation to continue using a generated shim internally

Definition of done:

- full Tactical Objects RPC/topic coverage over `application/flatbuffers`
- Ada and C++ interoperate over the same proto-native contract

### Phase 3: Protobuf to standard

- generate C++ protobuf wrappers around `protoc` output for all reachable
  payloads
- add runtime dispatch support for `application/protobuf`
- generate Ada typed protobuf APIs matching the JSON and FlatBuffers surface
- implement Ada protobuf through a generated shim into the C++ protobuf path

Definition of done:

- Tactical Objects full RPC/topic coverage over `application/protobuf`
- Ada and C++ interoperate over protobuf with the same generated typed APIs

### Phase 4: gRPC transport projection

- generate gRPC transport bindings from the same service closure used by PCL
- use native C++ `protoc` + `grpc_cpp_plugin`
- generate Ada typed gRPC-facing APIs backed by generated C++ helpers
- ensure one handler implementation can be projected to both PCL and gRPC

Definition of done:

- same service implementation can be exposed over PCL and gRPC without
  handwritten payload conversion
- at least one end-to-end demo proves the same contract over both transports

### Phase 5: shared-memory transport projection

- define a generated shared-memory transport projection with the same handler
  contract as PCL and gRPC
- start with C++ native implementation
- generate Ada typed APIs that call a generated C/C++ shared-memory shim
- reuse codec backends for payload encoding and decoding

Definition of done:

- the same Tactical Objects service contract works over shared memory with
  FlatBuffers and Protobuf
- the same handler implementation can be exercised over PCL and shared memory
- a cross-language Ada/C++ demo exists for at least one RPC and one topic flow

### Phase 6: ROS2 projection

- generate ROS2 message/service/topic projections from proto
- keep all ROS2-specific mapping in generation rather than in component code
- reuse the same canonical handler contract where feasible

Definition of done:

- ROS2 projection does not redefine the payload model
- component code remains transport-agnostic

## Test Plan

### Documentation acceptance

The documentation set is complete when:

- one doc is clearly the authoritative roadmap
- one doc describes current generator/runtime behavior
- one doc explains protocol-agnostic handler and transport rules
- the Tactical Objects docs reflect the actual proving-ground state

### Codec conformance

Every standardized codec backend must pass:

- scalar alias roundtrips, especially `Identifier`
- nested message roundtrips
- enum roundtrips
- repeated field roundtrips
- default and empty payload behavior
- request/response roundtrips
- canonical topic payload roundtrips

### Runtime dispatch

For PCL:

- `application/json`
- `application/flatbuffers`
- `application/protobuf`
- mixed codecs on different endpoints
- explicit failure on unsupported content types

### Cross-language

For standardized backends:

- C++ server / Ada client
- Ada server / C++ client
- JSON and FlatBuffers first
- Protobuf once standardized

### Transport

For shared memory:

- request/reply RPC over FlatBuffers
- request/reply RPC over Protobuf
- pub/sub topic path over at least one non-JSON codec
- same handler implementation exercised through both PCL and shared memory

For gRPC:

- same service contract exercised through both PCL and gRPC
- no handwritten payload conversion in component code

## Tactical Objects As Proving Ground

Tactical Objects remains the proving-ground component for this work.

The proving-ground sequence is:

1. standardize JSON
2. standardize FlatBuffers
3. standardize Protobuf
4. prove transport reuse across PCL, gRPC, and shared memory
5. then generalize the machinery to other PYRAMID components

