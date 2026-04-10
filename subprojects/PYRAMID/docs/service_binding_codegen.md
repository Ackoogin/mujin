# Service Binding Code Generation

## Overview

PYRAMID service binding generation is now proto-native.

The source-of-truth layering is:

1. MBSE / SysML model
2. generated `.proto`
3. generated language bindings, codecs, and protocol projections

For the generator pipeline below the MBSE extraction step, generated `.proto`
is the only canonical contract artefact.

There is no longer a separate service-local bridge schema for Tactical Objects.

## Current pipeline

```text
MBSE model
  -> proto_generator.py
  -> generated .proto
  -> generate_bindings.py
       -> ada_codegen.py
       -> cpp_codegen.py
       -> backends/json_backend.py
       -> backends/flatbuffers_backend.py
       -> other backend generators
```

The unified entrypoint is:

- [generate_bindings.py](/D:/Dev/repo/mujin/subprojects/PYRAMID/pim/generate_bindings.py)

## What gets generated

From the proto contract, the generator emits:

- Ada data-model types
- Ada JSON codecs for proto-native types
- Ada service bindings
- Ada FlatBuffers codec packages
- C++ data-model types
- C++ JSON codecs for proto-native types
- C++ service bindings
- C++ FlatBuffers codecs and `.fbs` schemas

For Tactical Objects, the active generated outputs include:

- [pyramid-data_model-common-types_codec.ads](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/ada/generated/pyramid-data_model-common-types_codec.ads)
- [pyramid-data_model-tactical-types_codec.ads](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/ada/generated/pyramid-data_model-tactical-types_codec.ads)
- [pyramid-services-tactical_objects-provided.ads](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/ada/generated/pyramid-services-tactical_objects-provided.ads)
- [pyramid-services-tactical_objects-consumed.ads](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/ada/generated/pyramid-services-tactical_objects-consumed.ads)
- [pyramid_data_model_common_codec.hpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/cpp/generated/pyramid_data_model_common_codec.hpp)
- [pyramid_data_model_tactical_codec.hpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/cpp/generated/pyramid_data_model_tactical_codec.hpp)
- [pyramid_services_tactical_objects_provided.hpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/cpp/generated/pyramid_services_tactical_objects_provided.hpp)
- [pyramid_services_tactical_objects_consumed.hpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/cpp/generated/pyramid_services_tactical_objects_consumed.hpp)

## JSON model

JSON support now means:

- serialize proto-native generated types
- deserialize proto-native generated types
- keep the same logical payload shape as the other codecs

There is no longer a separate Tactical Objects service `Json_Codec` package or
`wire_types` layer.

In practice, JSON uses:

- Ada:
  - [pyramid-data_model-common-types_codec.ads](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/ada/generated/pyramid-data_model-common-types_codec.ads)
  - [pyramid-data_model-tactical-types_codec.ads](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/ada/generated/pyramid-data_model-tactical-types_codec.ads)
- C++:
  - [pyramid_data_model_common_codec.hpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/cpp/generated/pyramid_data_model_common_codec.hpp)
  - [pyramid_data_model_tactical_codec.hpp](/D:/Dev/repo/mujin/subprojects/PYRAMID/examples/cpp/generated/pyramid_data_model_tactical_codec.hpp)

## Runtime codec selection

Generated bindings dispatch by `content_type`.

For Tactical Objects, the active runtime selections are:

- `application/json`
- `application/flatbuffers`

The same handler signatures are used regardless of codec. The binding layer:

1. decodes bytes into the proto-native generated type
2. invokes the handler with that type
3. encodes the response using the selected backend

## StandardBridge

`StandardBridge` is now an adapter between:

- the canonical PYRAMID proto-derived external model
- the Tactical Objects internal runtime model

It does not define an independent service contract.

For Tactical Objects standard topics, the canonical payload types are:

- `standard.entity_matches` -> `ObjectMatch[]`
- `standard.object_evidence` -> `ObjectDetail`
- `standard.evidence_requirements` -> `ObjectEvidenceRequirement`

## FlatBuffers

FlatBuffers support is generated from proto as well.

Current Tactical Objects status:

- C++ uses generated `.fbs` plus `flatc`
- Ada uses generated FlatBuffers codec packages
- runtime dispatch works for both JSON and FlatBuffers

The main backend implementation lives in:

- [flatbuffers_backend.py](/D:/Dev/repo/mujin/subprojects/PYRAMID/pim/backends/flatbuffers_backend.py)

## Removed architecture

The repository no longer uses:

- a Tactical Objects service-local `Json_Codec`
- a Tactical Objects service-local `Wire_Types`
- a separate Tactical Objects bridge schema file as an input to core binding generation

That older split was replaced with a single proto-native contract flow.
