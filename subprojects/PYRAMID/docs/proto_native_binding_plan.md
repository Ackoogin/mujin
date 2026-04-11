# Proto-Native Binding Plan

## Status

This document is retained only as a redirect for older references.

The planning and status material that used to live here has converged into:

- [service_schema_tactical_objects.md](/D:/Dev/repo/mujin/subprojects/PYRAMID/docs/service_schema_tactical_objects.md)
  for current Tactical Objects proving-ground status
- [service_binding_codegen.md](/D:/Dev/repo/mujin/subprojects/PYRAMID/docs/service_binding_codegen.md)
  for generator architecture and emitted artifacts

## Preserved Decisions

The decisions from the older plan still stand:

- generated `.proto` is the canonical downstream contract artifact
- bindings, codec backends, and transport backends must project the same typed
  contract
- Ada may use generated C/C++ helpers internally for non-JSON backends
- Tactical Objects remains the first proving-ground component for this work
