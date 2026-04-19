# Service Binding Code Generation

## Status

This document is retained as a redirect for older references.

The current generated binding architecture, usage rules, generation commands,
and C++/Ada facade guidance now live in:

- [generated_bindings.md](generated_bindings.md)

The current Tactical Objects proof matrix and implementation status now live in:

- [generated_bindings_status.md](generated_bindings_status.md)

## Preserved Summary

The generator remains proto-native:

1. MBSE / SysML model
2. generated `.proto`
3. generated language bindings, codec backends, and transport projections

For all tooling below MBSE extraction, `.proto` is the canonical downstream
contract artifact. Component code should use the generated service binding
facade rather than codec-specific branches.
