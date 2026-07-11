# xsd2proto output trees

`pim/xsd2proto.py` writes one directory per schema drop here
(`<drop>/pyramid/data_model/<package>.proto` + `wire_names.json` +
`closure_report.json`). Trees are regenerated deterministically from the
sha256-pinned drops in `pim/schemas/schema_manifest.json`;
`xsd2proto.py <profile> --check` fails on any drift.

## Check-in status

Per the [UCI MMS conversion plan](../../../../doc/plans/PYRAMID/uci_mms_conversion_plan.md)
D2, generated trees are checked in for reviewability — but **not yet**:

- `uci_2_5_0/` (profile P1) awaits the UCI 2.5 XSD itself (no public URL
  known; sourced from `UCI_XSD_PATH` or the `external/ams-gra/` checkout —
  see `pim/schemas/README.md`). Check-in happens with the Phase-1 exit
  (reconciliation against `pim/uci_seam_example` documented).
- `agra_5_0a/` (profile P2) converts today — strict-clean, measured
  2026-07-11 at **1,163 messages (30 synthesized) + 297 enums from 18
  roots** in <1 s, output parses with `proto_parser.py`
  (`test_xsd2proto.py` pins those numbers) — but check-in is a Phase-4
  exit-gate item, after deliberate closure pruning (top fan-in hubs:
  `ID_Type` 74, `SystemID_Type` 49, `ForeignKeyType` 40) and the
  derived-artefact posture note in `pim/schemas/README.md`.

Until then, whatever is generated locally under `<drop>/` is untracked
scratch; regenerate at will.
