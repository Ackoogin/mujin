# Tactical Objects Service Schema Status

## Purpose

This document records Tactical Objects as the proving-ground component for the
proto-native backend and transport model.

The goal is not to define a second contract. The contract is already the
generated `.proto` model. This document exists to show:

- what Tactical Objects currently proves
- what is still partial
- what backend and transport milestones remain

## Source-Of-Truth Clarification

The Tactical Objects source-of-truth layering is:

1. MBSE / SysML model is the ultimate semantic source
2. generated `.proto` is the canonical downstream contract artifact
3. generated bindings, codec backends, and transport backends derive from that
   `.proto`

No Tactical Objects-specific bridge schema is part of the core generation path.

## Canonical External Model

Externally, Tactical Objects should present canonical PYRAMID payloads.

For the standard topics used today:

- `standard.entity_matches` -> `ObjectMatch[]`
- `standard.object_evidence` -> `ObjectDetail`
- `standard.evidence_requirements` -> `ObjectEvidenceRequirement`

The bridge role is:

- Tactical Objects internals
- to canonical PYRAMID proto-derived external types

It is not:

- a second service schema
- a second payload model
- a competing source of truth

## Current Tactical Objects Status

### Service bindings

Current state:

- Tactical Objects Ada and C++ service bindings are proto-native
- runtime codec selection works for:
  - `application/json`
  - `application/flatbuffers`
- the old Tactical Objects service-local `Json_Codec` and `Wire_Types`
  artifacts are gone

### Codec backend status

| Codec backend | Tactical Objects status | Notes |
|--------------|-------------------------|-------|
| `json` | active and usable | proto-native type codecs are the live JSON path |
| `flatbuffers` | active and usable | C++ is native; Ada is typed and may use a generated shim internally |
| `protobuf` | not yet standardized | generated artifacts exist, but full runtime integration is still pending |

### Transport backend status

| Transport backend | Tactical Objects status | Notes |
|------------------|-------------------------|-------|
| `pcl` | active baseline | current proving path |
| `grpc` | partial / generated only | not yet an end-to-end tactical runtime path |
| `shared_memory` | planned | intended next transport proving target after Protobuf/PCL tightening |
| `ros2` | planned | later projection, not part of the current proving path |

## Ada Status

Ada policy for Tactical Objects is currently:

- public Ada APIs are typed and proto-native
- JSON remains a straightforward native Ada path
- FlatBuffers may use a generated C/C++ shim internally
- an internal JSON bridge inside that shim is acceptable for now
- the same policy is expected to apply to Protobuf and shared-memory transport
  support when those are added

This is an implementation choice, not a contract-layer exception.

## Backend And Transport Targets

### JSON

Tactical Objects must continue to prove that JSON is just one backend over the
proto-native typed surface.

Definition of done:

- JSON participates in the same backend-selection story as the other codecs
- no JSON-specific service payload model exists

### FlatBuffers

Tactical Objects must continue proving:

- full proto-native RPC/topic coverage over `application/flatbuffers`
- C++ native implementation via generated `.fbs` and `flatc`
- Ada typed APIs over the same contract

### Protobuf

Tactical Objects is the first target for bringing Protobuf to standard.

Definition of done:

- full PCL runtime support for `application/protobuf`
- typed Ada and C++ APIs matching the JSON/FlatBuffers surface
- cross-language Ada/C++ proof over the same contract

### gRPC

Tactical Objects should eventually prove that the same handler implementation
can be projected to both:

- `pcl`
- `grpc`

without handwritten payload translation.

### Shared memory

Tactical Objects should also prove:

- request/reply RPC over shared memory
- pub/sub over shared memory
- FlatBuffers and Protobuf as preferred shared-memory codecs
- same handler implementation reusable across PCL and shared memory

## Tactical Objects Milestone Order

The intended proving order is:

1. keep JSON stable as the semantic baseline
2. keep FlatBuffers stable and complete full coverage
3. bring Protobuf to the same standard on PCL
4. add shared-memory transport using FlatBuffers and Protobuf
5. add gRPC transport projection
6. generalize the same machinery to other PYRAMID components

## Status Table For Reviews

Use this quick table in future reviews:

| Capability | Status |
|-----------|--------|
| Proto-native Ada bindings | yes |
| Proto-native C++ bindings | yes |
| JSON runtime selection on PCL | yes |
| FlatBuffers runtime selection on PCL | yes |
| Protobuf runtime selection on PCL | not yet |
| C++ native FlatBuffers | yes |
| Ada typed FlatBuffers API | yes |
| Ada fully native FlatBuffers internals | not required yet |
| gRPC end-to-end Tactical path | not yet |
| shared-memory Tactical path | not yet |

## What To Avoid

The Tactical Objects work should not regress into any of the following:

- a Tactical Objects-only service schema
- a bridge-local wire type system
- transport-specific handler interfaces
- codec-specific handler interfaces
- backend hardcoding of Tactical Objects payload semantics that should be
  derived from proto

