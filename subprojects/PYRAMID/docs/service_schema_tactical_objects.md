# Tactical Objects Backend And Transport Status

## Status

This document is retained as a redirect for older references.

The current Tactical Objects generated-binding status, proof matrix, review
checklist, and remaining gaps now live in:

- [generated_bindings_status.md](generated_bindings_status.md)

For the generated binding architecture and usage guide, use:

- [generated_bindings.md](generated_bindings.md)

## Preserved Boundary

Tactical Objects remains the proving-ground component for PYRAMID generated
bindings, codecs, and transport projections.

No Tactical Objects-specific schema should be introduced below the generated
`.proto` layer. The current standard topics remain:

- `standard.entity_matches` -> `ObjectMatch[]`
- `standard.object_evidence` -> `ObjectDetail`
- `standard.evidence_requirements` -> `ObjectEvidenceRequirement`
