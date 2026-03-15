# SysML XMI Parser

Extracts logical data models from Cameo Systems Modeler XMI exports into structured JSON format.

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

## Next Steps

This JSON output can be transformed into:
- Protobuf `.proto` files
- JSON Schema
- OpenAPI specifications
- Code generation templates

## Notes

- Type references are resolved to human-readable names where possible
- Multiplicities are captured as lower/upper bounds (supports 0..1, 1..1, 1..*, etc.)
- Properties without explicit types will have `typeName: null`
