# PYRAMID PIM And Binding Tools

This directory contains the PYRAMID platform-independent model tools: proto parsing, generated C++/Ada bindings, JSON/FlatBuffers/Protobuf/gRPC/ROS2 backends, and a SysML XMI parser for importing logical data models.

Two independent toolchains live here:

- **The binding generator** (active pipeline): `generate_bindings.py`,
  `proto_parser.py`, `binding_contract.py`, `cpp/`, `ada/`,
  `cabi_codegen.py`, `ada_cabi_codegen.py`, `ros2_idl_codegen.py`,
  `ros2_marshal_codegen.py`, `standard_topics.py`, `codec_backends.py`, and
  `backends/`. Input is `.proto`; output is generated bindings.
- **The upstream MBSE/SysML import tools** (standalone, run manually) under
  `mbse/`: `sysml_parser.py` (XMI → JSON), `proto_generator.py`
  (JSON → `.proto`), `ada_type_generator.py`, and `contract_resolver.py`.
  `mbse/test.json` is a sample parsed-model output, and
  `mbse/pyramid-middleware.ads`/`.adb` is a legacy Ada middleware binding
  template from this chain. None of these are imported by the binding
  generator.

`test/` holds the duplicate-heavy PIM proto fixture tree and `test_harness/`
its viability/comms test scripts (see `test_harness/FINDINGS.md`).

The primary entry point for the current generated-binding pipeline is:

```bash
python generate_bindings.py
```

From the workspace root, prefer the wrapper scripts:

```bat
subprojects\PYRAMID\scripts\generate_bindings.bat
```

```bash
subprojects/PYRAMID/scripts/generate_bindings.sh
```

For generated-binding architecture and usage, see [`../doc/architecture/generated_bindings.md`](../doc/architecture/generated_bindings.md).

For a shorter engineer-facing architecture overview of how generated PYRAMID
bindings plug into PCL, see
[`../doc/architecture/pcl_pyramid_binding_generation_overview.md`](../doc/architecture/pcl_pyramid_binding_generation_overview.md).

## Contract Layouts (PYRAMID and generic proto)

The generator supports two contract layouts via `--contract-layout`:

- `pyramid` (default) — PYRAMID conventions; output is unchanged.
- `generic` — bindings for **arbitrary `.proto` contracts** that do not use
  PYRAMID package roots. Names derive from the proto package/service/message
  identity; classification is by parsed content, not package strings.

```bash
# Arbitrary proto contract -> C++ (all backends) and Ada
python generate_bindings.py my_proto/ out/ --languages cpp --backends json,flatbuffers,protobuf,grpc,ros2 --contract-layout generic
python generate_bindings.py my_proto/ out/ --languages ada  --backends json --contract-layout generic
```

Each run also writes `binding_manifest.json` (generated artifacts by role), which
CMake can consume via `PYRAMID_BINDING_SOURCE_MODE=manifest` instead of
`pyramid_*` filename globs. Standard-topic data is now loaded from
`topic_metadata/tactical_objects_topics.json` (not hardcoded). Full design:
[`../doc/architecture/generic_contract_layout.md`](../doc/architecture/generic_contract_layout.md).

## Build-Local Generation

The CMake build invokes `generate_bindings.py` during configure when
`PYRAMID_GENERATE_CPP_BINDINGS=ON`. The default output is
`${binaryDir}/generated/pyramid_cpp_bindings`, controlled by
`PYRAMID_CPP_BINDINGS_DIR`. Build targets then depend on
`pyramid_cpp_bindings_codegen`, which reruns the generator through a stamp file
when proto contracts or generator Python files change.

For externally delivered contracts, configure with
`PYRAMID_GENERATE_CPP_BINDINGS=OFF` and set `PYRAMID_CPP_BINDINGS_DIR` to the
delivered generated C++ binding tree. CMake will glob that tree instead of
running the generator.

## SysML XMI Parser

`mbse/sysml_parser.py` extracts logical data models from Cameo Systems Modeler XMI exports into structured JSON format.

## Usage

```bash
python mbse/sysml_parser.py <input.xmi> [output.json]
```

## What it extracts

- **DataTypes** with:
  - Properties (name, type, multiplicity, visibility)
  - Inheritance relationships (generalizations)
  - Abstract flag
  - Documentation/comments
  
- **Enumerations** with:
  - Enumeration literals
  
- **Associations** (basic support)

## Output Format

```json
{
  "dataTypes": [
    {
      "id": "...",
      "name": "Entity",
      "isAbstract": true,
      "properties": [
        {
          "name": "id",
          "type": "...",
          "typeName": "Identifier",
          "visibility": "public",
          "multiplicity": {
            "lower": "0",
            "upper": "1"
          }
        }
      ],
      "generalizes": ["BaseType"],
      "documentation": "..."
    }
  ],
  "enumerations": [
    {
      "name": "Status",
      "literals": [
        {"name": "Active"},
        {"name": "Inactive"}
      ]
    }
  ]
}
```

## Compatibility

- Tested with Cameo Systems Modeler 2022x
- Supports XMI 2.1 with UML 2.5 (2013 spec)
- Handles nested package structures

## Downstream Uses

This JSON output can be transformed into:
- Protobuf `.proto` files
- JSON Schema
- OpenAPI specifications
- Code generation templates

The current PYRAMID generated-binding system is built from `.proto` contracts.
Current
Tactical Objects binding/conformance status is tracked in
[`generated_bindings_status.md`](../../../doc/reports/PYRAMID/generated_bindings_status.md).

## Notes

- Type references are resolved to human-readable names where possible
- Multiplicities are captured as lower/upper bounds (supports 0..1, 1..1, 1..*, etc.)
- Properties without explicit types will have `typeName: null`
