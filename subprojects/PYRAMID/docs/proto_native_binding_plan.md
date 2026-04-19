# Proto-Native Binding Plan

## Status

This document is retained only as a redirect for older references.

The planning and status material that used to live here has converged into:

- [generated_bindings.md](generated_bindings.md)
  for the current generated-binding architecture and usage guide
- [generated_bindings_status.md](generated_bindings_status.md)
  for current Tactical Objects proving-ground status

## Preserved Decisions

The decisions from the older plan still stand:

- generated `.proto` is the canonical downstream contract artifact
- bindings, codec backends, and transport backends must project the same typed
  contract
- Ada may use generated C/C++ helpers internally for non-JSON backends
- Tactical Objects remains the first proving-ground component for this work
