#!/usr/bin/env python3
"""Regenerate the PCL HLR/LLR/test traceability matrix.

Parses:
  - subprojects/PCL/doc/requirements/HLR.md   (### PCL.NNN - Title)
  - subprojects/PCL/doc/requirements/LLR.md   (### REQ_PCL_NNN - Title,
                                               **Traces**: PCL.xxx,
                                               **Verification**: ...)
  - requirement tags (REQ_PCL_NNN) in PCL test sources.

Writes doc/reports/PCL/HLR_COVERAGE.md (test -> LLR -> HLR matrix plus gap
lists). This file is generated evidence: do not hand-edit the output; fix the
requirements documents or test tags and regenerate.

In addition to trace gaps, `--check` also runs the requirement quality gate
described in `subprojects/PCL/doc/standards/requirements_standard.md`: it
rejects duplicate identifiers, letter-suffixed identifiers, HLR/LLR normative
paragraphs that do not contain exactly one "shall", HLR text that names PCL
implementation identifiers, and HLR text that reaches into concepts owned by
a layer above PCL (PYRAMID protocol wire details, ROS2/DDS specifics, AME
application concepts, and similar).

Usage:
  python3 subprojects/PCL/scripts/gen_hlr_coverage.py [--check]

  --check   exit 1 if any trace gap or requirement quality issue exists (for
            CI use); the matrix is written either way.
"""

import argparse
import datetime
import re
import sys
from collections import Counter, OrderedDict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
HLR_PATH = REPO_ROOT / "subprojects/PCL/doc/requirements/HLR.md"
LLR_PATH = REPO_ROOT / "subprojects/PCL/doc/requirements/LLR.md"
OUT_PATH = REPO_ROOT / "doc/reports/PCL/HLR_COVERAGE.md"

# Test source trees scanned for ///< REQ_PCL_NNN tags.
TEST_SOURCES = [
    REPO_ROOT / "subprojects/PCL/tests",
]

HLR_HEADING = re.compile(r"^### (PCL\.\d+) - (.+)$")
LLR_HEADING = re.compile(r"^### (REQ_PCL_\d+) - (.+)$")
SECTION_HEADING = re.compile(r"^## (.+)$")
TRACE_IDS = re.compile(r"PCL\.\d+")
REQ_TAG = re.compile(r"REQ_PCL_(\d+)")
TESTFILE_REF = re.compile(r"([A-Za-z0-9_]+\.(?:cpp|hpp|c))")

# -- Requirement quality gate -------------------------------------------
#
# These patterns implement the checkable rules from
# subprojects/PCL/doc/standards/requirements_standard.md sections 1.1-1.3
# and 2.1-2.3: no suffix identifiers, no duplicate identifiers, exactly one
# "shall" per normative paragraph, no implementation identifiers or
# higher-layer concepts inside HLR text.

SUFFIX_HLR_ID = re.compile(r"PCL\.\d+[a-zA-Z]")
SUFFIX_LLR_ID = re.compile(r"REQ_PCL_\d+[a-zA-Z]")
SHALL_WORD = re.compile(r"\bshall\b")

# Implementation identifiers that do not belong in HLR normative text: PCL
# C function/macro names, C++ wrapper type names, status-code-style
# ALL_CAPS constants, and source file names.
HLR_IMPL_PATTERNS = [
    ("C function or macro name", re.compile(r"\bpcl_[a-z0-9_]*[a-z0-9](?:\(\))?\b")),
    ("C++ wrapper type name", re.compile(r"\bPcl[A-Z][A-Za-z0-9_]*\b")),
    ("status code or ALL_CAPS constant", re.compile(r"\bPCL_[A-Z][A-Z0-9_]*\b")),
    ("source file name", re.compile(r"\b[A-Za-z0-9_]+\.(?:c|h)\b")),
]

# Concepts owned by a layer above PCL (see doc/todo/PCL/TODO.md and the
# commit that deleted the generated-binding/application-domain HLRs for
# examples of what this is meant to catch). Acronyms are matched
# case-sensitively with word boundaries so they do not fire on ordinary
# English substrings (e.g. "AME" inside "name", "DDS" inside a hyphenated
# token); phrases are matched case-insensitively.
HLR_HIGHER_LAYER_TERMS = [
    (re.compile(r"\bPYRAMID protocol\b", re.IGNORECASE), "PYRAMID protocol"),
    (re.compile(r"\bPYRAMID component\b", re.IGNORECASE), "PYRAMID component"),
    (re.compile(r"\bwire[- ]name\b", re.IGNORECASE), "wire name"),
    (re.compile(r"\bproto(?:col|buf)? definition\b", re.IGNORECASE), "proto definition"),
    (re.compile(r"\bprotobuf\b", re.IGNORECASE), "protobuf"),
    (re.compile(r"\bgenerated (?:service )?binding", re.IGNORECASE), "generated binding"),
    (re.compile(r"\bROS2\b"), "ROS2"),
    (re.compile(r"\bDDS\b"), "DDS"),
    (re.compile(r"\bAME\b"), "AME"),
    (re.compile(r"\bmission planning\b", re.IGNORECASE), "mission planning"),
    (re.compile(r"\bautonomy mission\b", re.IGNORECASE), "autonomy mission"),
    (re.compile(r"\bbehaviou?r tree\b", re.IGNORECASE), "behaviour tree"),
]


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


def parse_hlr_bodies(path):
    """Return OrderedDict hlr_id -> normative paragraph text (the lines
    between the heading and the **Rationale** marker, joined with spaces).

    Duplicate headings overwrite earlier entries here, same as parse_hlr;
    duplicate detection itself is done separately in find_duplicate_ids so
    both the first and later occurrences are reported.
    """
    bodies = OrderedDict()
    current = None
    buf = []

    def flush():
        if current is not None:
            bodies[current] = " ".join(x for x in buf if x.strip())

    for line in path.read_text(encoding="utf-8").splitlines():
        m = HLR_HEADING.match(line)
        if m:
            flush()
            current = m.group(1)
            buf = []
            continue
        if current is None:
            continue
        if line.startswith("**Rationale**") or SECTION_HEADING.match(line):
            flush()
            current = None
            buf = []
            continue
        buf.append(line)
    flush()
    return bodies


def parse_llr_bodies(path):
    """Return OrderedDict llr_id -> normative paragraph text (the lines
    between the heading and the **Traces** marker, joined with spaces)."""
    bodies = OrderedDict()
    current = None
    buf = []

    def flush():
        if current is not None:
            bodies[current] = " ".join(x for x in buf if x.strip())

    for line in path.read_text(encoding="utf-8").splitlines():
        m = LLR_HEADING.match(line)
        if m:
            flush()
            current = m.group(1)
            buf = []
            continue
        if current is None:
            continue
        if line.startswith("**Traces**"):
            flush()
            current = None
            buf = []
            continue
        buf.append(line)
    flush()
    return bodies


def find_duplicate_ids(path, heading_re):
    """Return sorted list of identifiers whose heading appears more than
    once in the file at `path`."""
    ids = [m.group(1) for line in path.read_text(encoding="utf-8").splitlines()
           for m in [heading_re.match(line)] if m]
    counts = Counter(ids)
    return sorted(i for i, n in counts.items() if n > 1)


def find_suffix_ids(path, suffix_re):
    """Return sorted list of distinct letter-suffixed identifiers appearing
    anywhere in `path` (headings or trace references)."""
    return sorted(set(suffix_re.findall(path.read_text(encoding="utf-8"))))


def check_hlr_quality(hlr_path):
    """Run the HLR-specific quality rules from requirements_standard.md
    section 1 and return a list of human-readable issue strings."""
    issues = []

    dup = find_duplicate_ids(hlr_path, HLR_HEADING)
    for hid in dup:
        issues.append(f"duplicate HLR identifier: {hid}")

    suffix = find_suffix_ids(hlr_path, SUFFIX_HLR_ID)
    for sid in suffix:
        issues.append(f"suffix HLR identifier is not permitted: {sid}")

    bodies = parse_hlr_bodies(hlr_path)
    for hid, text in bodies.items():
        count = len(SHALL_WORD.findall(text))
        if count != 1:
            issues.append(
                f"{hid}: normative paragraph contains {count} \"shall\" "
                f"statements (must be exactly 1)")
        for label, pattern in HLR_IMPL_PATTERNS:
            hit = pattern.search(text)
            if hit:
                issues.append(
                    f"{hid}: HLR text names an implementation identifier "
                    f"({label} \"{hit.group(0)}\"); move it to an LLR")
        for pattern, label in HLR_HIGHER_LAYER_TERMS:
            if pattern.search(text):
                issues.append(
                    f"{hid}: HLR text references a higher-layer concept "
                    f"(\"{label}\"); keep higher-layer concepts out of PCL "
                    f"requirements")
    return issues


def check_llr_quality(llr_path):
    """Run the LLR-specific quality rules from requirements_standard.md
    section 2 and return a list of human-readable issue strings."""
    issues = []

    dup = find_duplicate_ids(llr_path, LLR_HEADING)
    for lid in dup:
        issues.append(f"duplicate LLR identifier: {lid}")

    suffix = find_suffix_ids(llr_path, SUFFIX_LLR_ID)
    for sid in suffix:
        issues.append(f"suffix LLR identifier is not permitted: {sid}")

    bodies = parse_llr_bodies(llr_path)
    for lid, text in bodies.items():
        count = len(SHALL_WORD.findall(text))
        if count != 1:
            issues.append(
                f"{lid}: normative paragraph contains {count} \"shall\" "
                f"statements (must be exactly 1)")
    return issues


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
    return int(hlr_id.split(".", 1)[1])


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

    quality_issues = check_hlr_quality(HLR_PATH) + check_llr_quality(LLR_PATH)
    w("## Quality Checks")
    w("")
    w("Automated requirement quality gate from "
      "`subprojects/PCL/doc/standards/requirements_standard.md` sections 1 "
      "and 2: duplicate identifiers, suffix identifiers, one-\"shall\"-per-"
      "requirement, HLR implementation identifiers, and HLR references to "
      "higher-layer concepts.")
    w("")
    if quality_issues:
        for issue in quality_issues:
            w(f"- {issue}")
    else:
        w("None.")
    w("")

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
    if quality_issues:
        print(f"Quality issues: {len(quality_issues)}")
        for issue in quality_issues:
            print(f"  - {issue}")
    if args.check and (gap_count or quality_issues):
        if gap_count:
            print("FAIL: trace gaps present (see Trace Gaps section)")
        if quality_issues:
            print("FAIL: requirement quality issues present "
                  "(see Quality Checks section)")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
