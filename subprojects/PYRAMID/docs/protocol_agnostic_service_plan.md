# Protocol-Agnostic Service Plan

## Status

This document is now a short redirect rather than a second status page.

For the current Tactical Objects backend and transport state, use:

- [service_schema_tactical_objects.md](/D:/Dev/repo/mujin/subprojects/PYRAMID/docs/service_schema_tactical_objects.md)

For generator implementation details, use:

- [service_binding_codegen.md](/D:/Dev/repo/mujin/subprojects/PYRAMID/docs/service_binding_codegen.md)

## Preserved Invariants

The protocol-agnostic rules remain unchanged:

1. component logic sees one proto-native typed surface
2. codec choice must not change handler signatures
3. transport choice must not change handler signatures
4. no backend may redefine payload types after `.proto` generation
5. generated adapters may translate delivery behavior, but not contract meaning
