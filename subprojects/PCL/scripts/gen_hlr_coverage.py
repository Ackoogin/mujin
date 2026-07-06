#!/usr/bin/env python3
"""Regenerate the PCL HLR/LLR/test traceability matrix.

Parses:
  - subprojects/PCL/doc/requirements/HLR.md   (### PCL.NNN - Title)
  - subprojects/PCL/doc/requirements/LLR.md   (### REQ_PCL_NNN - Title,
                                               **Traces**: PCL.xxx,
                                               **Verification**: ...)
  - requirement tags (REQ_PCL_NNN) in the PCL, PYRAMID proto-binding, and
    AME integration test sources.

Writes doc/reports/PCL/HLR_COVERAGE.md (test -> LLR -> HLR matrix plus gap
lists). This file is generated evidence: do not hand-edit the output; fix the
requirements documents or test tags and regenerate.

Usage:
  python3 subprojects/PCL/scripts/gen_hlr_coverage.py [--check]

  --check   exit 1 if any trace gap exists (for CI use); the matrix is
            written either way.
"""

import argparse
import datetime
import re
import sys
from collections import OrderedDict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
HLR_PATH = REPO_ROOT / "subprojects/PCL/doc/requirements/HLR.md"
LLR_PATH = REPO_ROOT / "subprojects/PCL/doc/requirements/LLR.md"
OUT_PATH = REPO_ROOT / "doc/reports/PCL/HLR_COVERAGE.md"

# Test source trees scanned for ///< REQ_PCL_NNN tags.
TEST_SOURCES = [
    REPO_ROOT / "subprojects/PCL/tests",
    REPO_ROOT / "subprojects/PYRAMID/tests/test_pcl_proto_bindings.cpp",
    REPO_ROOT / "subprojects/AME/tests/test_pcl_integration.cpp",
]

HLR_HEADING = re.compile(r"^### (PCL\.\d+[a-z]?) - (.+)$")
LLR_HEADING = re.compile(r"^### (REQ_PCL_\d+) - (.+)$")
SECTION_HEADING = re.compile(r"^## (.+)$")
TRACE_IDS = re.compile(r"PCL\.\d+[a-z]?")
REQ_TAG = re.compile(r"REQ_PCL_(\d+)")
TESTFILE_REF = re.compile(r"([A-Za-z0-9_]+\.(?:cpp|hpp|c))")


def parse_hlr(path):
    """Return OrderedDict hlr_id -> {'title', 'section'}."""
    hlrs = OrderedDict()
    section = ""
    for line in path.read_text(encoding="utf-8").splitlines():
        m = SECTION_HEADING.match(line)
        if m:
            section = m.group(1).strip()
            continue
        m = HLR_HEADING.match(line)
        if m:
            hlrs[m.group(1)] = {"title": m.group(2).strip(), "section": section}
    return hlrs


def parse_llr(path):
    """Return OrderedDict llr_id -> {'title', 'traces': [hlr ids],
    'verification_files': [file names]}."""
    llrs = OrderedDict()
    current = None
    for line in path.read_text(encoding="utf-8").splitlines():
        m = LLR_HEADING.match(line)
        if m:
            current = m.group(1)
            llrs[current] = {
                "title": m.group(2).strip(),
                "traces": [],
                "verification_files": [],
            }
            continue
        if current is None:
            continue
        if line.startswith("**Traces**"):
            llrs[current]["traces"] = TRACE_IDS.findall(line)
        elif line.startswith("**Verification**"):
            llrs[current]["verification_files"] = sorted(
                set(TESTFILE_REF.findall(line))
            )
    return llrs


def scan_test_tags(paths):
    """Return dict llr_id -> sorted list of test file names containing the tag."""
    tags = {}
    files = []
    for p in paths:
        if p.is_dir():
            files.extend(sorted(p.glob("*.c")))
            files.extend(sorted(p.glob("*.cpp")))
            files.extend(sorted(p.glob("*.hpp")))
        elif p.is_file():
            files.append(p)
    for f in files:
        text = f.read_text(encoding="utf-8", errors="replace")
        for m in REQ_TAG.finditer(text):
            llr_id = "REQ_PCL_%03d" % int(m.group(1))
            tags.setdefault(llr_id, set()).add(f.name)
    return {k: sorted(v) for k, v in tags.items()}


def llr_sort_key(llr_id):
    return int(llr_id.rsplit("_", 1)[1])


def hlr_sort_key(hlr_id):
    m = re.match(r"PCL\.(\d+)([a-z]?)", hlr_id)
    return (int(m.group(1)), m.group(2))


def short_llr(llr_id):
    return llr_id.replace("REQ_PCL_", "")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="exit 1 if any trace gap exists")
    args = ap.parse_args()

    hlrs = parse_hlr(HLR_PATH)
    llrs = parse_llr(LLR_PATH)
    tags = scan_test_tags(TEST_SOURCES)

    # HLR -> [LLR ids]
    hlr_to_llrs = {h: [] for h in hlrs}
    unknown_hlr_refs = {}
    for llr_id, info in llrs.items():
        for hlr_id in info["traces"]:
            if hlr_id in hlr_to_llrs:
                hlr_to_llrs[hlr_id].append(llr_id)
            else:
                unknown_hlr_refs.setdefault(hlr_id, []).append(llr_id)

    # LLR -> test files (union of source tags and Verification references)
    llr_to_tests = {}
    for llr_id, info in llrs.items():
        files = set(tags.get(llr_id, []))
        files.update(f for f in info["verification_files"]
                     if f.startswith("test_") or f == "pcl_transport_conformance.hpp")
        llr_to_tests[llr_id] = sorted(files)

    # Gap lists
    hlrs_without_llr = [h for h, ls in hlr_to_llrs.items() if not ls]
    llrs_without_test = [l for l, fs in llr_to_tests.items() if not fs]
    tags_without_llr = sorted(
        (t for t in tags if t not in llrs), key=llr_sort_key)
    llrs_without_hlr = [l for l, info in llrs.items() if not info["traces"]]

    today = datetime.date.today().isoformat()
    lines = []
    w = lines.append
    w("# HLR Requirements Coverage")
    w("")
    w("> **Generated file.** Regenerate with "
      "`python3 subprojects/PCL/scripts/gen_hlr_coverage.py`.")
    w("> Do not hand-edit; fix `HLR.md`/`LLR.md` or the test requirement tags "
      "and regenerate.")
    w("")
    w(f"Generated: {today}")
    w("")
    w("End-to-end traceability: **Test -> LLR -> HLR** for all PCL "
      "requirements.")
    w("")
    w("Each test carries `///< REQ_PCL_NNN` tags; each LLR's **Traces** field "
      "names its parent HLR(s). The matrix below is derived from those two "
      "sources plus the **Verification** pointers in `LLR.md`.")
    w("")

    section = None
    for hlr_id in sorted(hlrs, key=hlr_sort_key):
        info = hlrs[hlr_id]
        if info["section"] != section:
            section = info["section"]
            w(f"## {section}")
            w("")
            w("| HLR | Description | LLR(s) | Test File(s) |")
            w("|-----|-------------|--------|--------------|")
        llr_list = sorted(hlr_to_llrs[hlr_id], key=llr_sort_key)
        test_files = sorted({f for l in llr_list for f in llr_to_tests[l]})
        llr_cell = ", ".join(short_llr(l) for l in llr_list) or "**NONE**"
        test_cell = ", ".join(f.replace(".cpp", "").replace(".hpp", "")
                              for f in test_files) or "**NONE**"
        w(f"| {hlr_id} | {info['title']} | {llr_cell} | {test_cell} |")
        if hlr_id == sorted(hlrs, key=hlr_sort_key)[-1]:
            w("")
    w("")

    w("## Summary")
    w("")
    w("| Metric | Count |")
    w("|--------|-------|")
    w(f"| HLRs | {len(hlrs)} |")
    w(f"| LLRs | {len(llrs)} |")
    w(f"| LLRs with at least one verifying test file | "
      f"{len(llrs) - len(llrs_without_test)} |")
    w(f"| Distinct test files carrying REQ_PCL tags | "
      f"{len(set(f for fs in tags.values() for f in fs))} |")
    w(f"| HLRs with no tracing LLR | {len(hlrs_without_llr)} |")
    w(f"| LLRs with no test evidence | {len(llrs_without_test)} |")
    w(f"| LLRs with no parent HLR | {len(llrs_without_hlr)} |")
    w(f"| Test tags naming unknown LLRs | {len(tags_without_llr)} |")
    w("")

    def gap_section(title, items, fmt=lambda x: x):
        w(f"### {title}")
        w("")
        if items:
            for it in items:
                w(f"- {fmt(it)}")
        else:
            w("None.")
        w("")

    w("## Trace Gaps")
    w("")
    gap_section("HLRs with no tracing LLR",
                sorted(hlrs_without_llr, key=hlr_sort_key))
    gap_section("LLRs with no test evidence",
                sorted(llrs_without_test, key=llr_sort_key))
    gap_section("LLRs with no parent HLR trace",
                sorted(llrs_without_hlr, key=llr_sort_key))
    gap_section(
        "LLR references to unknown HLRs",
        sorted(unknown_hlr_refs.items()),
        fmt=lambda kv: f"{kv[0]} referenced by {', '.join(kv[1])}")
    gap_section("Test tags naming unknown LLRs", tags_without_llr)

    w("## Trace Tag Format")
    w("")
    w("Tests use LLR requirement tags in comments:")
    w("")
    w("```")
    w("///< REQ_PCL_NNN: Brief description. PCL.0XX.")
    w("```")
    w("")
    w("Each LLR in `LLR.md` has a **Traces** field listing the parent HLR(s), "
      "completing the chain:")
    w("")
    w("```")
    w("Test (code tag) -> LLR (LLR.md) -> HLR (HLR.md)")
    w("```")
    w("")

    OUT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")

    gap_count = (len(hlrs_without_llr) + len(llrs_without_test)
                 + len(llrs_without_hlr) + len(tags_without_llr)
                 + len(unknown_hlr_refs))
    print(f"Wrote {OUT_PATH}")
    print(f"HLRs: {len(hlrs)}  LLRs: {len(llrs)}  gaps: {gap_count}")
    if args.check and gap_count:
        print("FAIL: trace gaps present (see Trace Gaps section)")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
