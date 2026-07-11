# xsd2proto output trees

`pim/xsd2proto.py` writes one directory per schema drop here
(`<drop>/pyramid/data_model/<package>.proto` + `wire_names.json` +
`closure_report.json`). Trees are regenerated deterministically from the
sha256-pinned drops in `pim/schemas/schema_manifest.json`;
`xsd2proto.py <profile> --check` fails on any drift, and
`test_xsd2proto.py` re-derives the checked-in tree from the pinned drop
and byte-compares (SKIP-safe when the drop is absent).

## Check-in status

Per the [UCI MMS conversion plan](../../../../doc/plans/PYRAMID/uci_mms_conversion_plan.md)
D2, generated trees are checked in for reviewability:

- **`uci_2_5_0/` (profile P1) — checked in** (2026-07-11): 515 messages
  (6 synthesized) + 188 enums from 6 roots, converted strict-clean.
- **`agra_5_0a/` (profile P2) — not yet**: converts today (strict-clean,
  1,163 messages + 297 enums from 18 roots, `test_xsd2proto.py` pins the
  counts) but check-in is a Phase-4 exit-gate item, after deliberate
  closure pruning (top fan-in hubs: `ID_Type` 74, `SystemID_Type` 49,
  `ForeignKeyType` 40) and the derived-artefact posture note in
  `pim/schemas/README.md`.

## P1 reconciliation against `pim/uci_seam_example` (plan D6)

The hand-authored seam contract and the converted tree describe the same
two wire messages; every divergence resolves toward the XSD, and all are
wire-compatible at the OMS JSON level:

| Aspect | Hand contract (`uci_seam_example`) | Converted tree (XSD-faithful) | Wire impact |
|--------|-------------------------------------|-------------------------------|-------------|
| Root naming | message `ActionCommand` | element `ActionCommand` → message `ActionCommandMT` (element→type indirection recorded in `wire_names.json` `roots`) | none — the root wrapper key comes from the *element* name |
| Envelope | `SecurityInformation`/`MessageHeader` fields flattened into the root message | `MessageType base = 1` composition (extension per D5) | none — XSD extension is invisible in instance XML/JSON, so codecs must **flatten `base` fields into the parent object** (the hand codec already does this implicitly; a Phase-2 codec-gen rule) |
| `MessageData.Command` | `Command { Capability capability = 1; }` — capability arm only | `ActionCommandType` with a real `oneof choice` (capability \| activity interaction) | none for capability-arm traffic; the converted tree additionally expresses the activity arm the hand contract omitted |
| ID types | `SystemId { Uuid uuid = 1; }` + codec special-case collapsing `Uuid` to a JSON string | `SystemID_Type { ID_Type base = 1 }`, `ID_Type { string uuid = 1; ... }` — the scalar collapse falls out of the simpleType rule, no special case | none — both render `{"UUID": "..."}` given base-flattening |
| Header field order | approximated (`MissionID` first) | XSD declaration order (`SystemID`, `Timestamp`, `SchemaVersion`, `Mode`, ...) | none — OMS JSON is name-keyed; field numbers are contract-internal |
| Timestamp type | `string` | `string` (with the XSD `DateTimeType` pattern `.+Z` retained as a comment) | none — validates the converter's temporal-types-as-string deviation |

Conclusion recorded for the Phase-1 exit: the conversion **supersedes** the
hand contract in fidelity (real choice arms, real ID composition, real
facets); `uci_seam_example` remains the frozen golden fixture per plan D6,
and the one substantive rule the codec generator must add for the
generated tree is **`base`-field flattening on the wire** (Phase 2).
