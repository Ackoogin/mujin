# Conversion profile manifests

Input manifests for `pim/xsd2proto.py` — one per profile of the
[UCI MMS conversion plan](../../../../doc/plans/PYRAMID/uci_mms_conversion_plan.md)
§5 ladder. A manifest names the schema drop (pinned in
`pim/schemas/schema_manifest.json`), the target proto package, and the
top-level **root message names** to convert; the converter resolves the
transitive type closure from the roots and reports its size, so profiles
are pruned deliberately rather than growing by accident.

Format deviation from the plan text: manifests are **JSON, not YAML** — the
pim toolchain is Python-stdlib-only and stays that way (PyYAML exists in
some dev environments but is not a dependency anywhere else in `pim/`).
Commentary lives in `_comment` arrays.

| Profile | Drop | Ladder rung | Validation tier | Converted (2026-07-11) |
|---------|------|-------------|-----------------|------------------------|
| `p1_kitty_hawk.json` | `uci_2_5_0` | P1 — working set, provable live vs Sleet | (a)+(b)+(c) | strict-clean: 515 messages + 188 enums from 6 roots; tree checked in |
| `p2_agra_planning_core.json` | `agra_5_0a` | P2 — A-GRA `MA_*` planning core | (a) offline only | strict-clean: 1,163 messages + 297 enums from 18 roots; check-in gated on Phase-4 pruning |

Rules of the road (plan D1/D2):

- A profile targets exactly one drop; generated types are never shared
  across drops.
- Generated output is checked in and must be byte-stable under converter
  re-runs (`xsd2proto.py --check` is the CI guard).
- Growing a profile means reading the closure report the converter prints
  and pruning hub types deliberately.
