# Schema drops for the UCI MMS conversion

Phase 0 of
[`doc/plans/PYRAMID/uci_mms_conversion_plan.md`](../../../../doc/plans/PYRAMID/uci_mms_conversion_plan.md):
pin the exact XSD artefacts `pim/xsd2proto.py` converts, per drop, by hash.

## Layout

- `schema_manifest.json` — the pin manifest: per-drop schema version,
  file list, sha256 pins, and ordered acquisition sources.
- `fetch_schemas.py` — materialises the files under `dl/<drop>/` from an
  env override, the `external/` Kitty Hawk checkout, or a pinned URL, and
  verifies them against the manifest.
- `dl/` — the downloaded XSDs (git-ignored; see redistribution posture
  below).

## Redistribution posture (Phase 0 item 1) — status

**Recorded 2026-07-11, unresolved → fetch-not-vendor.** Both upstream sets
are public releases (the ASK 5.0a package including the Start Here Guide,
21 APR 2026; the OMS v2.5 document set at `github.com/open-arsenal/oms`),
but the standing residual from
[`a_gra_standard_review.md`](../../../../doc/research/AME/a_gra_standard_review.md)
§7 Phase 0 — confirm CUI status of derived artefacts before publishing
converted material — has not been discharged in writing. Until it is:

- the XSDs themselves are **fetched, never committed** (`dl/` is
  git-ignored);
- the sha256 pins in `schema_manifest.json` are committed, so what we
  convert is nailed down even though the bytes are not vendored;
- generated proto trees (which *are* checked in, per plan D2) carry the
  drop name, schema version, and source hash in their headers, so their
  provenance is auditable against this manifest.

If the posture check concludes the XSDs are freely redistributable, move
them from `dl/` into this directory, commit, and simplify `fetch_schemas.py`
to a verify-only role. Record that conclusion here either way.

Network note (measured 2026-07-11): the development environment's egress
proxy blocks the GitHub API and codeload, but **allows
raw.githubusercontent.com and git-over-HTTPS** (github.com and gitlab.com
both fetch). Both drops were bootstrapped and sha256-pinned from this
environment: A-GRA 5.0a from the public a-gra repo (commit
`953c6a11b83dc96d2a5133a49d4691e71150750c` at bootstrap time), and UCI 2.5
via the `git` source — the public USAF UCI standard repo at the exact
revision Sleet's own `make fetch-spec` pins. The upstream UCI drop carries
a "Government Owned" license marker (`LICENSES/uci-schema` in the sleet
repo records the same posture question we track above).

## Bootstrap (trust-on-first-use, once per drop)

```sh
python3 fetch_schemas.py uci_2_5_0      # or agra_5_0a, or no args for all
```

While a file's `sha256` is `null` in the manifest, the script obtains the
file, reports it **UNVERIFIED**, and prints the computed hash with
instructions to commit it into `schema_manifest.json` (exit code 3). Once
pinned, every subsequent fetch hard-fails on mismatch and the file is
removed. Never re-bootstrap a pin to make a mismatch go away — a changed
hash means a changed upstream drop, which per plan D1/D2 is a **new** drop
entry and a new generated tree, reviewed as a diff.

## Consumers

Everything that needs an XSD (xsd2proto runs against real drops, the
SKIP-gated parts of `pim/test_xsd2proto.py`) locates it via
`dl/<drop>/<name>` or the same env overrides the manifest lists
(`UCI_XSD_PATH`, `AGRA_XSD_DIR`), and **SKIPs with a printed reason** when
absent — the established Sleet/GNAT-absence pattern. Nothing in the default
build or CTest run requires network or the XSDs.
