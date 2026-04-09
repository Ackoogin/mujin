# Proto-Native Service Binding Plan

## Status

This document records the intended architecture for PYRAMID service binding
generation going forward.

It exists because the current implementation drifted toward a
`json_schema.py`-driven tactical wire model, while the original requirement was
clear:

- `.proto` is the single source of truth.
- Ada and C++ bindings are generated from `.proto`.
- JSON, FlatBuffers, and Protobuf codecs are generated from the same proto
  contract.
- Any bridge-specific reshaping is an adapter, not the core service model.

This document is the plan to realign the codebase to that architecture.

---

## Problem Summary

The current tactical generation path mixes two concepts:

1. The canonical service contract from the proto files under
   [subprojects/PYRAMID/proto](/D:/Dev/repo/mujin/subprojects/PYRAMID/proto)
2. A tactical bridge-specific wire schema in
   [json_schema.py](/D:/Dev/repo/mujin/subprojects/PYRAMID/pim/json_schema.py)

That bridge schema was useful for shaping specific JSON payloads such as
`CreateRequirementRequest`, `EntityMatch`, and `ObjectEvidence`, but once it
became the basis for runtime dispatch and non-JSON codecs it introduced the
wrong dependency direction:

- service bindings started depending on tactical-specific wire types
- FlatBuffers support started needing tactical-specific mapping logic
- extending the full RPC surface pushed the backend toward hard-coded message
  knowledge

That is the opposite of the intended architecture.

If the goal is true automatic generation, the generator must derive service
payload types from the proto RPC signatures and the proto message graph, not
from a hand-maintained tactical bridge schema.

---

## Architectural Intent

### Core rule

`.proto` defines the service contract.

Everything else is generated from it.

### Consequences

- Service request and response bindings in Ada and C++ use proto-native types.
- JSON codec generation uses proto-native types.
- FlatBuffers codec generation uses proto-native types.
- Protobuf codec generation uses proto-native types.
- Runtime codec dispatch chooses a codec for the same logical payload type,
  rather than choosing between unrelated data models.

### What does not belong in the core binding model

The following are not canonical service types:

- tactical bridge helper structs such as `CreateRequirementRequest`
- bridge-specific field renaming
- bridge-specific flattening or enrichment
- topic-specific JSON reshaping for external consumers

Those concerns belong in an adapter layer above or beside the generated core
bindings.

---

## Target Model

## 1. Canonical source

The proto files under
[subprojects/PYRAMID/proto](/D:/Dev/repo/mujin/subprojects/PYRAMID/proto)
define:

- RPC names
- request types
- response types
- streaming shapes
- enums
- nested message graphs

These proto definitions are the only canonical model for service generation.

## 2. Generated type layers

From proto, the generator should emit:

- Ada data model types
- C++ data model types
- service binding code for Ada and C++
- codec backends for JSON, FlatBuffers, and Protobuf over the same proto-native
  message types

No tactical-specific service-local `Wire_Types` package should be required for
the core binding path.

## 3. Runtime dispatch model

Runtime dispatch should be:

- logical type = proto-defined request/response type
- encoding = selected by `content_type`
- serialization backend = JSON / FlatBuffers / Protobuf

The same handler signature should work regardless of codec:

- deserialize bytes into the proto-native generated type
- invoke handler with that type
- serialize response of the proto-native generated type using the selected
  backend

## 4. Adapter model

If the standard bridge requires reshaped JSON for compatibility with existing
consumers, that should be implemented separately as one of:

- a generated adapter layer
- a bridge-specific translation module
- proto annotations that drive an optional transform

But the adapter must not become the source for service bindings.

---

## Design Principles

## Single source of truth

Every core service artifact must be derivable from the proto graph.

## Backend symmetry

If JSON and FlatBuffers are both supported for a service, they must operate on
the same logical payload types.

## No tactical special cases in core backends

A codec backend may know about proto constructs, scalar mappings, oneofs,
repeated fields, enums, and streaming responses.

It should not know about specific RPC names like `CreateRequirement` or
specific tactical shapes like `EntityMatch`.

## Separation of contract from presentation

Proto contract and bridge presentation are different concerns.

The contract belongs in core generation.
The presentation belongs in an adapter.

---

## Migration Plan

## Phase 1. Identify and isolate `json_schema.py` ownership

Current issue:

- `json_schema.py` currently owns service-local wire types and service JSON
  codecs for tactical bindings.

Actions:

- audit all generator paths that depend on `json_schema.py`
- classify each dependency as either:
  - core service binding concern
  - bridge adapter concern
- keep only adapter concerns attached to `json_schema.py`

Expected result:

- the repo clearly distinguishes proto-native generation from bridge reshaping

## Phase 2. Make proto-native service generation the default

Actions:

- refactor [cpp_codegen.py](/D:/Dev/repo/mujin/subprojects/PYRAMID/pim/cpp_codegen.py)
  so generated service bindings use proto request and response types directly
- refactor [ada_codegen.py](/D:/Dev/repo/mujin/subprojects/PYRAMID/pim/ada_codegen.py)
  to do the same
- remove the dependency on generated service-local `Wire_Types` for the core
  path
- make invoke, publish, subscribe, and dispatch operate on proto-native types

Expected result:

- handler signatures match proto contract types
- service binding code no longer depends on tactical bridge-specific structs

## Phase 3. Make backend codecs proto-native

Actions:

- generate JSON codec functions for proto-native service payload types
- generate FlatBuffers codec functions for proto-native service payload types
- keep Protobuf aligned with the same type surface
- ensure streaming responses are supported as repeated/array holders generated
  from proto response types

Expected result:

- backend selection is just an encoding choice, not a model choice

## Phase 4. Move bridge reshaping into adapters

Actions:

- either retain `json_schema.py` as a bridge adapter specification
- or replace it with a more explicit adapter/annotation mechanism
- keep bridge-facing JSON conversions outside the core service generator

Expected result:

- the bridge can preserve legacy JSON contracts without polluting the core
  service binding model

## Phase 5. Update demos and tests

Actions:

- update C++ and Ada demos to use proto-native generated types for RPC calls
- keep any bridge-specific conversion at the edge only
- expand tests to prove that:
  - JSON and FlatBuffers both work over the same proto-native handler surface
  - Ada and C++ interoperate under both codecs
  - no JSON payload wrapping is used in FlatBuffers mode

Expected result:

- both languages exercise the same contract through multiple codecs

---

## Deliverables

The refactor is complete when all of the following are true:

- core service bindings are derived from proto RPC signatures
- core runtime dispatch does not depend on `json_schema.py`
- FlatBuffers backend does not hard-code tactical RPC/message knowledge
- Ada and C++ both use proto-native generated types in the binding surface
- JSON and FlatBuffers support the full tactical RPC surface
- any bridge-specific JSON reshaping lives in a separate adapter layer
- documentation describes proto as the canonical source and bridge reshaping as
  optional adaptation

---

## Acceptance Criteria

The new architecture will be considered correct when:

1. A new service added only in proto can generate Ada and C++ bindings without
   hand-editing tactical schema files.
2. Adding FlatBuffers support for that service does not require naming the RPCs
   or messages in backend code.
3. The generated runtime dispatch path can serve JSON and FlatBuffers for the
   same proto-native handler signature.
4. Existing bridge compatibility can be preserved without changing the core
   service model.

---

## Open Questions

## How should bridge reshaping be expressed?

Options:

- keep `json_schema.py` as an adapter spec
- move to proto annotations
- move to a dedicated adapter DSL

This does not block the proto-native core refactor. It only affects the shape
of the bridge compatibility layer.

## Should service-local helper types still exist at all?

Only if they are explicitly adapter-layer types.

They should not be part of the core binding/runtime contract.

## How should Ada FlatBuffers be implemented?

Preferred direction:

- generate the FlatBuffers schema from proto
- use `flatc` for C++
- expose stable codec entry points that Ada can consume cleanly
- keep the public Ada binding surface proto-native

Whether Ada uses a native implementation or C interop behind the scenes is a
secondary implementation detail. The important point is that the logical type
surface remains proto-native.

---

## Immediate Next Step

The next implementation step should be:

- remove `json_schema.py` from the core service binding generation path
- rework the generated service bindings so proto request/response types are the
  direct typed API for invoke and dispatch
- then regenerate JSON and FlatBuffers codec surfaces against those proto-native
  types

That is the path consistent with the original requirement and with long-term
automatic generation.
