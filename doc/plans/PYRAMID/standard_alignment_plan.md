# Tactical Objects Standard Alignment

## Purpose

This document describes the current Tactical Objects alignment with the
PYRAMID proto interface. It is the stable design reference for the shipped
component. It is not an implementation backlog; completed migration work has
been folded into the current-state sections below.

## Current State (2026-04-21)

Tactical Objects exposes the PYRAMID tactical-objects proto contract through
generated service bindings. The shipped executable path is:

```text
remote client
  -> PCL socket transport
  -> tactical_objects_app
  -> StandardBridge
  -> TacticalObjectsRuntime
```

`StandardBridge` is the production adapter between the generated PYRAMID
service interface and the internal Tactical Objects business model. It is not a
hand-written wire protocol. It registers the generated provided service names,
uses generated dispatch/codecs for request and response payloads, and maps
typed standard data-model objects into the runtime's domain model.

The internal runtime remains the owner of business logic:

- object storage and component tables
- correlation and identity/classification state
- spatial matching and query filtering
- interest lifecycle and derived evidence requirements
- high-rate stream subscriber state

Generated PYRAMID types remain at the external interface boundary and in the
adapter. The domain runtime continues to use tactical-object domain types where
they are a better fit for correlation, spatial indexing, history, and MIL-STD
metadata.

## Provided Interface

The shipped app exposes these generated provided services:

| Generated service | Request | Response | Runtime mapping |
|-------------------|---------|----------|-----------------|
| `object_of_interest.create_requirement` | `ObjectInterestRequirement` | `Identifier` | register interest, register stream subscriber, optionally derive evidence requirements |
| `object_of_interest.read_requirement` | `Query` | `ObjectInterestRequirement[]` | read active interest records |
| `object_of_interest.update_requirement` | `ObjectInterestRequirement` | `Ack` | update an existing interest and refresh derived requirements |
| `object_of_interest.delete_requirement` | `Identifier` | `Ack` | cancel interest and remove stream subscriber |
| `matching_objects.read_match` | `Query` | `ObjectMatch[]` | query runtime objects and project matches |
| `specific_object_detail.read_detail` | `Query` | `ObjectDetail[]` | read full object detail projection from store components |

The shipped app publishes these generated topics:

| Topic | Payload | Source |
|-------|---------|--------|
| `standard.entity_matches` | `ObjectMatch[]` | runtime stream frames projected to standard matches |
| `standard.evidence_requirements` | `ObjectEvidenceRequirement` | active-find derived evidence requirements |

The shipped app consumes:

| Topic | Payload | Runtime mapping |
|-------|---------|-----------------|
| `standard.object_evidence` | `ObjectDetail` | converted to `ObservationBatch` and processed by the runtime |

## Codec Support

The standard interface can be hosted with a single selected frontend content
type per app instance:

| Content type | Status | Notes |
|--------------|--------|-------|
| `application/json` | supported | baseline inspectable wire format |
| `application/flatbuffers` | supported | wire-efficient binary path for high-rate object data |
| `application/protobuf` | supported | generated protobuf codec path for C++ PCL clients |

The C++ generated bindings support all three content types on the real app
path. The current Ada ActiveFind socket client exercises JSON and FlatBuffers.
Ada protobuf/gRPC coverage is handled by the generated binding and gRPC
interop tests described in
[generated_bindings_status.md](../../reports/PYRAMID/generated_bindings_status.md).

## Standard To Internal Mapping

The current business/runtime model is intentionally compatible with the PYRAMID
proto surface, but it is not a wholesale replacement of the domain model with
generated structs. The adapter owns schema-boundary translations.

### Identity And Dimension

Internal `Affiliation` and `BattleDimension` ordinals are aligned with the
standard enum values where both models overlap. Unsupported or extension-only
values are mapped to a safe standard fallback at the boundary.

| Boundary item | Current rule |
|---------------|--------------|
| `StandardIdentity` -> `Affiliation` | ordinal cast for aligned values |
| `Affiliation` -> `StandardIdentity` | ordinal cast for aligned values |
| `BattleDimension` standard/internal | ordinal cast for aligned values; unsupported values map to `Unknown` |
| Internal `Space` / `SOF` | retained as domain extensions; not represented in the standard enum |

### Position And Area

Internal `Position.lat/lon` and interest `BoundingBox` values are radians, the
same unit used by the PYRAMID `GeodeticPosition` contract. No degree/radian
conversion is performed at the standard boundary.

| Standard input | Internal mapping |
|----------------|------------------|
| `Point` | zero-area `BoundingBox` |
| `CircleArea` | circumscribed `BoundingBox` using center +/- radius |
| `PolyArea` | axis-aligned `BoundingBox` over all polygon vertices |

The internal matcher currently uses axis-aligned bounding boxes. Polygon and
circle geometry are accepted through the standard interface but reduced to the
runtime's current spatial-query shape.

### Interest And Evidence

`ObjectInterestRequirement.policy` maps to runtime query mode:

| Standard policy | Runtime query mode |
|-----------------|--------------------|
| `Query` | `ReadCurrent` |
| `Obtain` | `ActiveFind` |

Active-find interests trigger runtime solution determination. Derived evidence
requirements are projected as standard `ObjectEvidenceRequirement` payloads and
published on `standard.evidence_requirements`. They are also offered to the
local consumed service when the app is configured to expose the consumed
interface.

### Object Detail

`specific_object_detail.read_detail` projects runtime component state into the
standard `ObjectDetail` model:

| Standard field | Runtime source |
|----------------|----------------|
| `id` | entity UUID |
| `update_time` | lifecycle last update timestamp |
| `entity_source` | first correlated/identity source reference |
| `source[]` | source-system projection, defaulting to local when absent |
| `position` | kinematics position |
| `creation_time` | earliest source first-seen timestamp or update-time fallback |
| `quality` | quality confidence |
| `course` | derived from velocity east/north |
| `speed` | velocity magnitude |
| `identity` | MIL classification affiliation |
| `dimension` | MIL classification battle dimension |

Fields without a runtime source remain unset. Domain extensions such as full
MIL classification profile, behavior, lifecycle status, relationships, zones,
and history stay in the internal model unless a standard contract is added for
them.

### Object Match Stream

`standard.entity_matches` is the standard lightweight high-rate projection of
runtime stream frames. It publishes `ObjectMatch[]` rather than full
`ObjectDetail[]` to avoid sending large detail payloads on every stream tick.
Consumers that need full detail should call `specific_object_detail.read_detail`
for matched identifiers.

## Deployment Forms

| Form | Path | Purpose |
|------|------|---------|
| Real app | `tactical_objects_app` | Production-style executable with local runtime and remote generated provided interface |
| Standalone bridge harness | `tests/tactical_objects/standalone_bridge.cpp` | Compatibility/interoperability harness for split-process tests |

The real app is the authoritative production path for Tactical Objects
standard-interface validation. The standalone harness remains useful for
transport experiments and compatibility split-process tests, but it is not a
substitute for real-app coverage.

## Verification

Current focused real-app tests:

| Test | Coverage |
|------|----------|
| `tobj_cpp_app_client_e2e` | C++ generated client against `tactical_objects_app`, JSON |
| `tobj_cpp_app_client_flatbuffers_e2e` | C++ generated client against `tactical_objects_app`, FlatBuffers |
| `tobj_cpp_app_client_protobuf_e2e` | C++ generated client against `tactical_objects_app`, Protobuf |
| `tobj_ada_active_find_app_e2e` | Ada ActiveFind/evidence-provider flow against `tactical_objects_app`, JSON |
| `tobj_ada_active_find_app_flatbuffers_e2e` | Ada ActiveFind/evidence-provider flow against `tactical_objects_app`, FlatBuffers |

The focused real-app verification command is:

```bat
ctest --test-dir build -C Release -R "tobj_cpp_app_client_(e2e|flatbuffers_e2e|protobuf_e2e)|tobj_ada_active_find_app(_flatbuffers)?_e2e" --output-on-failure
```

Broader generated-binding, gRPC, ROS2, and transport status is tracked in
[generated_bindings_status.md](../../reports/PYRAMID/generated_bindings_status.md).

## Remaining Design Point

The current standard high-rate stream is `standard.entity_matches`. That keeps
the mass-send path compact and wire efficient. If consumers need full detail in
bulk without repeated `read_detail` calls, the next design decision is whether
to add a standard batch-detail or bulk-detail path rather than overloading the
match stream with large `ObjectDetail` payloads.
