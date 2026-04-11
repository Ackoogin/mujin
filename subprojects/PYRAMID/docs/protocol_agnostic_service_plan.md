# Protocol-Agnostic Service Plan

## Intent

PYRAMID components should define one canonical interface and then project it
into multiple codecs and transports without component authors maintaining
protocol-specific glue.

The canonical interface below MBSE extraction is the generated `.proto`
contract.

## Invariants

The following rules are mandatory:

1. component logic sees one proto-native typed surface
2. codec choice must not change handler signatures
3. transport choice must not change handler signatures
4. no backend may redefine payload types after `.proto` generation
5. adapters may translate delivery behavior, but not contract meaning

## Standard Terms

- `proto-native typed surface`: the generated typed API used by handlers and
  clients
- `codec backend`: payload encoding choice
- `transport backend`: delivery mechanism
- `Ada shim`: generated internal glue used when Ada delegates to C/C++

## Handler Contract

The desired shape is:

```text
component logic
  -> generated typed handler interface
  -> generated codec dispatch
  -> generated transport projection
```

The typed handler layer is the stable boundary. Backends vary around it, not
through it.

### Handler responsibilities

Handlers own:

- domain logic
- validation at the business-logic level
- interpretation of request intent
- construction of typed responses

Handlers do not own:

- raw payload decoding
- `content_type` dispatch
- transport framing
- protocol-specific channel or endpoint naming

## Runtime Selection Model

### Codec selection

Codec selection is by `content_type`.

That means:

- `application/json`
- `application/flatbuffers`
- `application/protobuf`

The dispatch layer:

1. decodes incoming bytes according to `content_type`
2. invokes the typed handler
3. encodes the response with the same selected codec unless the transport
   explicitly defines a different rule

### Transport selection

Transport selection is by generated transport binding.

That means:

- `pcl`
- `grpc`
- `shared_memory`
- later `ros2`

The transport layer owns framing and delivery, but it must reuse the same
typed handler contract.

## Transport Matrix

| Transport backend | Payload model | Preferred codecs | Ada strategy | Notes |
|------------------|---------------|------------------|--------------|-------|
| `pcl` | proto-native typed surface | JSON, FlatBuffers, Protobuf | native binding or generated shim | baseline transport |
| `grpc` | proto-native typed surface | Protobuf first, JSON only if needed for debug tooling | generated shim acceptable | transport projection, not a second contract |
| `shared_memory` | proto-native typed surface | FlatBuffers, Protobuf; JSON for debug only | generated shim acceptable | transport projection with explicit frame/header |
| `ros2` | proto-native typed surface | transport-defined ROS2 projection | optional later | later projection, not a competing IDL |

## Shared-Memory Transport Rules

Shared memory is a transport backend, not a codec backend.

It must:

- carry the same proto-native request/response/topic payloads
- reuse codec backends for payload encoding and decoding
- support request/reply RPC and pub/sub
- include explicit framing metadata

The standard shared-memory frame/header should carry:

- endpoint or service name
- operation or channel identifier
- `content_type`
- payload size
- correlation or request id for RPC

Preferred codec pairings for shared memory are:

- `application/flatbuffers`
- `application/protobuf`

`application/json` is allowed for debug and compatibility, not as the default
production choice.

## Ada Policy

Ada must always expose typed proto-native APIs.

For non-JSON backends and transports:

- a generated C/C++ shim is acceptable
- an internal JSON bridge inside that shim is acceptable short-term
- the public Ada surface must remain typed
- the selected on-wire codec must remain the real backend codec

This policy applies to:

- FlatBuffers
- Protobuf
- gRPC projection
- shared-memory projection

## Endpoint And Naming Rules

Transport backends may differ in how they address endpoints, but the naming
source must still be the proto service contract.

That means:

- service names derive from generated service groups
- topic names derive from canonical standard-topic mappings
- request/response operation identity derives from the proto service and RPC
  definition

No transport backend should invent a second naming taxonomy for the logical
contract.

## Acceptance Criteria

The protocol-agnostic service model is in place when:

- the same handler implementation can be projected to at least two transports
- the same payload contract can be encoded by at least two codecs
- component code contains no transport-specific payload translation
- component code contains no codec-specific branching
- backend-specific logic is confined to generated layers

## Tactical Objects Proving Sequence

Tactical Objects is the proving-ground component for this model.

The proving order should be:

1. PCL + JSON
2. PCL + FlatBuffers
3. PCL + Protobuf
4. shared_memory + FlatBuffers
5. shared_memory + Protobuf
6. gRPC + Protobuf

The same generated typed surface should remain stable throughout that sequence.

