# Protocol-Agnostic Service Plan

## Status

This document is now a short redirect rather than a second status page.

For the current generated-binding architecture and usage guide, use:

- [generated_bindings.md](generated_bindings.md)

For the current Tactical Objects backend and transport state, use:

- [generated_bindings_status.md](generated_bindings_status.md)

## Preserved Invariants

The protocol-agnostic rules remain unchanged:

1. component logic sees one proto-native typed surface
2. codec choice must not change handler signatures
3. transport choice must not change handler signatures
4. no backend may redefine payload types after `.proto` generation
5. generated adapters may translate delivery behavior, but not contract meaning
