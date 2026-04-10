# Proto-Native Service Binding Plan

## Intent

PYRAMID component interfaces should be defined once and then projected into
multiple languages, codecs, and transports without component authors needing
to hand-maintain protocol glue.

The source-of-truth layering is:

1. MBSE / SysML model is the ultimate semantic source
2. generated `.proto` is the canonical downstream contract artefact
3. bindings, codecs, and transport projections are generated from that proto

## What this means

The generated contract should drive:

- Ada bindings
- C++ bindings
- JSON codecs
- FlatBuffers codecs
- Protobuf codecs
- protocol projections such as PCL, ROS2, and gRPC

The same logical payload type should exist regardless of codec.

## Current state

Implemented now:

- [generate_bindings.py](/D:/Dev/repo/mujin/subprojects/PYRAMID/pim/generate_bindings.py) is the unified entrypoint
- Tactical Objects Ada and C++ service bindings are proto-native
- JSON uses the proto-derived data-model codec packages
- FlatBuffers generation is proto-native
- runtime codec selection works for `application/json` and `application/flatbuffers`
- the old Tactical Objects service-local `Json_Codec` and `Wire_Types` artefacts have been removed

Still incomplete:

- Ada FlatBuffers still uses generated shim glue internally rather than a fully independent native implementation
- additional protocol backends such as ROS2 and gRPC still need to be demonstrated end-to-end on the same canonical contract
- some docs and tests still reflect transitional assumptions and should continue to be tightened

## Rules

The downstream generation stack should follow these rules:

1. No generator below MBSE extraction may redefine the service payload model.
2. No transport-specific schema should become the core contract.
3. Component logic should see one handler interface per service, not one per transport.
4. Runtime codec dispatch should choose encoding, not data model.

## Target architecture

```text
MBSE / SysML
  -> proto_generator.py
  -> generated .proto
  -> generate_bindings.py
       -> language bindings
       -> codec backends
       -> transport/protocol projections
```

For a new component, the expected workflow is:

1. define or update the interface in MBSE
2. regenerate `.proto`
3. regenerate language bindings and codecs
4. implement the component handler logic once
5. select the transport / codec combination at integration time

## Tactical Objects standard topics

For the Tactical Objects bridge path, the canonical PYRAMID topic payloads are:

- `standard.entity_matches` -> `ObjectMatch[]`
- `standard.object_evidence` -> `ObjectDetail`
- `standard.evidence_requirements` -> `ObjectEvidenceRequirement`

The bridge is an adapter between canonical PYRAMID types and Tactical Objects
internals. It is not a second service-contract source.

## Next steps

1. Continue tightening docs and examples around the proto-native model.
2. Extend the same contract flow to additional protocol backends.
3. Reduce remaining internal shims where they are only transitional.
4. Keep the generated surface symmetric across Ada and C++.
