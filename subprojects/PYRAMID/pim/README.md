# PYRAMID PIM And Binding Tools

This directory contains the PYRAMID platform-independent model tools: proto parsing, generated C++/Ada bindings, JSON/FlatBuffers/Protobuf/gRPC/ROS2 backends, and a SysML XMI parser for importing logical data models.

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

## SysML XMI Parser

`sysml_parser.py` extracts logical data models from Cameo Systems Modeler XMI exports into structured JSON format.

## Usage

```bash
python sysml_parser.py <input.xmi> [output.json]
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
