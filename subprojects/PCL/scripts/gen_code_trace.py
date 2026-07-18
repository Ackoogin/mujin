#!/usr/bin/env python3
"""Regenerate the PCL code-to-LLR traceability matrix.

Parses:
  - subprojects/PCL/doc/requirements/LLR.md      (### REQ_PCL_NNN - Title,
                                                  optional **Implementation**
                                                  marker)
  - PCL production sources: subprojects/PCL/src/*.c, src/pcl_internal.h,
    the host APOS stub subprojects/PCL/bindings/apos/apos.c (it implements
    the "Apos Stub" requirements), and the C++ wrapper headers in
    subprojects/PCL/include/pcl/*.hpp.

Production functions carry a trace comment immediately before their
definition, in one of two forms (see the C coding standard, rule 5.6):

  /* Implements: REQ_PCL_012, REQ_PCL_094. */
  /* No LLR: <justification naming the requirements that cover it>. */

Writes doc/reports/PCL/CODE_TO_LLR.md (function -> LLR and LLR -> function
matrices plus gap lists). This file is generated evidence: do not hand-edit
the output; fix the source annotations or LLR.md and regenerate.

Checks performed (all are failures under --check):
  - an Implements tag names an LLR that does not exist in LLR.md;
  - a non-static function in subprojects/PCL/src/*.c has neither an
    Implements tag nor a "No LLR:" justification comment;
  - an LLR has no implementation tag anywhere and is not marked
    "**Implementation**: test-only" or "**Implementation**:
    documentation-only" in LLR.md.

Static helper functions may be tagged when they carry requirement behaviour
of their own; untagged static functions inherit their callers' trace and are
not reported.

Usage:
  python3 subprojects/PCL/scripts/gen_code_trace.py [--check]

  --check   exit 1 if any violation exists and do not rewrite the report
            (for CI use); without it the matrix is written and the same
            checks are reported.
"""

import argparse
import datetime
import re
import sys
from collections import OrderedDict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
LLR_PATH = REPO_ROOT / "subprojects/PCL/doc/requirements/LLR.md"
OUT_PATH = REPO_ROOT / "doc/reports/PCL/CODE_TO_LLR.md"

SRC_DIR = REPO_ROOT / "subprojects/PCL/src"
WRAPPER_DIR = REPO_ROOT / "subprojects/PCL/include/pcl"

LLR_HEADING = re.compile(r"^### (REQ_PCL_\d+) - (.+)$")
IMPL_MARKER = re.compile(r"^\*\*Implementation\*\*:\s*(test-only|documentation-only)\b")
REQ_ID = re.compile(r"REQ_PCL_(\d+)")

# A C function definition in PCL style starts at column 0 with the return
# type and name on one line (optionally preceded by `static`), e.g.:
#   pcl_status_t pcl_executor_add(pcl_executor_t* e, ...) {
#   static void udp_shutdown(void* adapter_ctx) {
C_FUNC_DEF = re.compile(
    r"^(?P<static>static\s+)?"
    r"(?!typedef\b|struct\b|enum\b|union\b|else\b|return\b|goto\b)"
    r"[A-Za-z_][A-Za-z0-9_]*[^=;(){}]*?"
    r"\b(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*\(")

C_CONTROL_KEYWORDS = frozenset(
    ["if", "for", "while", "switch", "sizeof", "return", "defined"])

# A C++ wrapper method or free function: used only to attribute a tag
# comment in a .hpp file to the declaration that follows it.
HPP_NAME = re.compile(r"\b(~?[A-Za-z_][A-Za-z0-9_]*)\s*\(")


def parse_llrs(path):
    """Return OrderedDict llr_id -> {'title', 'exempt': None|str}.

    'exempt' records the **Implementation** marker value (test-only or
    documentation-only) when present.
    """
    llrs = OrderedDict()
    current = None
    for line in path.read_text(encoding="utf-8").splitlines():
        m = LLR_HEADING.match(line)
        if m:
            current = m.group(1)
            llrs[current] = {"title": m.group(2).strip(), "exempt": None}
            continue
        if current is None:
            continue
        m = IMPL_MARKER.match(line)
        if m:
            llrs[current]["exempt"] = m.group(1)
    return llrs


def strip_line_noise(line):
    """Remove trailing CR so CRLF sources parse like LF sources."""
    return line.rstrip("\r\n")


def preceding_comment(lines, def_index):
    """Return the comment block text immediately above lines[def_index],
    or "" if there is none.

    Blank lines and preprocessor lines between the comment and the
    definition are skipped so a single comment can govern both branches of
    an #ifdef'd definition pair.
    """
    i = def_index - 1
    while i >= 0:
        line = strip_line_noise(lines[i]).strip()
        if line == "" or line.startswith("#"):
            i -= 1
            continue
        if line.endswith("*/"):
            block = []
            while i >= 0:
                text = strip_line_noise(lines[i]).strip()
                block.append(text)
                if text.startswith("/*"):
                    return " ".join(reversed(block))
                i -= 1
            return ""
        if line.startswith("//") or line.startswith("///"):
            block = []
            while i >= 0 and strip_line_noise(lines[i]).strip().startswith("//"):
                block.append(strip_line_noise(lines[i]).strip())
                i -= 1
            return " ".join(reversed(block))
        return ""
    return ""


def is_definition(lines, start_index):
    """Return True when the parenthesised parameter list starting on
    lines[start_index] is followed by '{' (a definition) rather than ';'
    (a declaration)."""
    depth = 0
    seen_open = False
    for i in range(start_index, min(start_index + 40, len(lines))):
        for ch in strip_line_noise(lines[i]):
            if ch == "(":
                depth += 1
                seen_open = True
            elif ch == ")":
                depth -= 1
            elif seen_open and depth == 0:
                if ch == "{":
                    return True
                if ch == ";":
                    return False
    return False


def scan_c_file(path):
    """Return list of dicts describing functions defined in a C file:
    {'name', 'line', 'static', 'tags': [llr ids], 'no_llr': bool,
     'comment': str}."""
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    funcs = OrderedDict()
    in_block_comment = False
    for idx, raw in enumerate(lines):
        line = strip_line_noise(raw)
        if in_block_comment:
            if "*/" in line:
                in_block_comment = False
            continue
        if line.lstrip().startswith("/*") and "*/" not in line:
            in_block_comment = True
            continue
        if not line or line[0] in " \t#/}{":
            continue
        m = C_FUNC_DEF.match(line)
        if not m:
            continue
        name = m.group("name")
        if name in C_CONTROL_KEYWORDS:
            continue
        if not is_definition(lines, idx):
            continue
        comment = preceding_comment(lines, idx)
        tags = ["REQ_PCL_%03d" % int(n) for n in REQ_ID.findall(comment)] \
            if "Implements:" in comment else []
        no_llr = comment.startswith("/* No LLR") or "No LLR:" in comment
        prev = funcs.get(name)
        if prev is None:
            funcs[name] = {
                "name": name,
                "line": idx + 1,
                "static": bool(m.group("static")),
                "tags": tags,
                "no_llr": no_llr,
            }
        else:
            # Second definition of the same name (an #ifdef platform pair):
            # merge, so a tag on either branch counts for both.
            prev["tags"] = prev["tags"] or tags
            prev["no_llr"] = prev["no_llr"] or no_llr
    return list(funcs.values())


def scan_hpp_file(path):
    """Return list of {'name', 'line', 'tags'} for tag comments in a C++
    wrapper header. Only tagged declarations are collected; the C++ layer
    is outside the strict function-inventory check (see the report notes).
    """
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    entries = []
    for idx, raw in enumerate(lines):
        line = strip_line_noise(raw)
        if "Implements:" not in line:
            continue
        comment = line.strip()
        j = idx
        while not comment.rstrip().endswith("*/") and j + 1 < len(lines):
            j += 1
            comment += " " + strip_line_noise(lines[j]).strip()
        tags = ["REQ_PCL_%03d" % int(n) for n in REQ_ID.findall(comment)]
        name = "(file-level)"
        k = j + 1
        while k < len(lines):
            candidate = strip_line_noise(lines[k]).strip()
            if candidate and not candidate.startswith(("/", "*", "#")):
                m = HPP_NAME.search(candidate)
                if m:
                    name = m.group(1)
                break
            k += 1
        entries.append({"name": name, "line": idx + 1, "tags": tags})
    return entries


def llr_sort_key(llr_id):
    return int(llr_id.rsplit("_", 1)[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="exit 1 on any violation; do not rewrite the report")
    args = ap.parse_args()

    llrs = parse_llrs(LLR_PATH)

    c_files = (sorted(SRC_DIR.glob("*.c"))
               + [SRC_DIR / "pcl_internal.h",
                  REPO_ROOT / "subprojects/PCL/bindings/apos/apos.c"])
    hpp_files = sorted(WRAPPER_DIR.glob("*.hpp"))

    per_file = OrderedDict()
    for f in c_files:
        if f.exists():
            per_file[f.name] = scan_c_file(f)
    hpp_per_file = OrderedDict()
    for f in hpp_files:
        entries = scan_hpp_file(f)
        if entries:
            hpp_per_file[f.name] = entries

    # LLR -> [(file, function)] reverse index.
    llr_to_funcs = {}
    for fname, funcs in per_file.items():
        for fn in funcs:
            for t in fn["tags"]:
                llr_to_funcs.setdefault(t, []).append((fname, fn["name"]))
    for fname, entries in hpp_per_file.items():
        for e in entries:
            for t in e["tags"]:
                llr_to_funcs.setdefault(t, []).append((fname, e["name"]))

    # Violations.
    unknown_tags = []
    for fname, funcs in per_file.items():
        for fn in funcs:
            for t in fn["tags"]:
                if t not in llrs:
                    unknown_tags.append((fname, fn["name"], t))
    for fname, entries in hpp_per_file.items():
        for e in entries:
            for t in e["tags"]:
                if t not in llrs:
                    unknown_tags.append((fname, e["name"], t))

    untagged = []
    justified = []
    for fname, funcs in per_file.items():
        for fn in funcs:
            if fn["static"]:
                continue
            if fn["tags"]:
                continue
            if fn["no_llr"]:
                justified.append((fname, fn["name"]))
            else:
                untagged.append((fname, fn["name"]))

    unimplemented = [
        lid for lid in llrs
        if lid not in llr_to_funcs and llrs[lid]["exempt"] is None
    ]
    exempt = [(lid, llrs[lid]["exempt"]) for lid in llrs
              if llrs[lid]["exempt"] is not None]

    violations = len(unknown_tags) + len(untagged) + len(unimplemented)

    # Report.
    today = datetime.date.today().isoformat()
    lines = []
    w = lines.append
    w("# PCL Code-to-LLR Traceability Matrix")
    w("")
    w("> **Generated file.** Regenerate with "
      "`python3 subprojects/PCL/scripts/gen_code_trace.py`.")
    w("> Do not hand-edit; fix the source annotations or `LLR.md` and "
      "regenerate.")
    w("")
    w(f"Generated: {today}")
    w("")
    w("Direct trace from production source code to low-level requirements "
      "(GAP-C-08). Every non-static function in `subprojects/PCL/src/*.c` "
      "carries either an `Implements: REQ_PCL_NNN` comment or a reviewed "
      "`No LLR:` justification naming the requirements that cover it "
      "indirectly. Static helpers are tagged when they carry requirement "
      "behaviour of their own; otherwise they inherit their callers' trace. "
      "Tags in the C++ wrapper headers (`include/pcl/*.hpp`) are collected "
      "for the reverse index, but the strict function-inventory check "
      "applies to the C production sources only, which form the certified "
      "boundary (see `DO178C_GAP_ANALYSIS.md`, section 4).")
    w("")

    total_funcs = sum(len(v) for v in per_file.values())
    tagged_funcs = sum(1 for v in per_file.values() for fn in v if fn["tags"])
    w("## Summary")
    w("")
    w("| Metric | Count |")
    w("|--------|-------|")
    w(f"| C production functions (incl. static) | {total_funcs} |")
    w(f"| Functions with Implements tags | {tagged_funcs} |")
    w(f"| Non-static functions with a reviewed No-LLR justification | "
      f"{len(justified)} |")
    w(f"| Non-static functions with neither tag nor justification | "
      f"{len(untagged)} |")
    w(f"| LLRs | {len(llrs)} |")
    w(f"| LLRs implemented by at least one tagged function | "
      f"{len([l for l in llrs if l in llr_to_funcs])} |")
    w(f"| LLRs exempt (test-only / documentation-only) | {len(exempt)} |")
    w(f"| LLRs with no implementation tag and no exemption | "
      f"{len(unimplemented)} |")
    w(f"| Tags naming unknown LLRs | {len(unknown_tags)} |")
    w("")

    w("## Function to LLR (by source file)")
    w("")
    for fname, funcs in per_file.items():
        rows = [fn for fn in funcs if fn["tags"] or not fn["static"]]
        if not rows:
            continue
        w(f"### `{fname}`")
        w("")
        w("| Function | Linkage | LLR(s) |")
        w("|----------|---------|--------|")
        for fn in rows:
            linkage = "static" if fn["static"] else "external"
            if fn["tags"]:
                cell = ", ".join(t.replace("REQ_PCL_", "")
                                 for t in sorted(fn["tags"], key=llr_sort_key))
            elif fn["no_llr"]:
                cell = "*No LLR (justified in source)*"
            else:
                cell = "**MISSING**"
            w(f"| `{fn['name']}` | {linkage} | {cell} |")
        w("")
    if hpp_per_file:
        w("### C++ wrapper headers (`include/pcl/*.hpp`)")
        w("")
        w("| File | Function | LLR(s) |")
        w("|------|----------|--------|")
        for fname, entries in hpp_per_file.items():
            for e in entries:
                cell = ", ".join(t.replace("REQ_PCL_", "")
                                 for t in sorted(e["tags"], key=llr_sort_key))
                w(f"| `{fname}` | `{e['name']}` | {cell} |")
        w("")

    w("## LLR to Function (reverse index)")
    w("")
    w("| LLR | Implemented by |")
    w("|-----|----------------|")
    for lid in sorted(llrs, key=llr_sort_key):
        if lid in llr_to_funcs:
            cell = "; ".join(
                f"`{fn}` ({fname})" for fname, fn in llr_to_funcs[lid])
        elif llrs[lid]["exempt"]:
            cell = f"*{llrs[lid]['exempt']}*"
        else:
            cell = "**NONE**"
        w(f"| {lid.replace('REQ_PCL_', '')} | {cell} |")
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

    w("## Gaps")
    w("")
    gap_section("Tags naming unknown LLRs", unknown_tags,
                fmt=lambda t: f"{t[0]}:`{t[1]}` names {t[2]}")
    gap_section("Non-static functions with neither tag nor justification",
                untagged, fmt=lambda t: f"{t[0]}:`{t[1]}`")
    gap_section("LLRs with no implementation tag and no exemption",
                sorted(unimplemented, key=llr_sort_key))
    gap_section(
        "Exempt LLRs (marked in LLR.md)",
        sorted(exempt, key=lambda kv: llr_sort_key(kv[0])),
        fmt=lambda kv: f"{kv[0]} ({kv[1]})")

    if not args.check:
        OUT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(f"Wrote {OUT_PATH}")
    print(f"C functions: {total_funcs}  tagged: {tagged_funcs}  "
          f"LLRs: {len(llrs)}  violations: {violations}")
    for fname, fn, t in unknown_tags:
        print(f"  - {fname}:{fn} names unknown LLR {t}")
    for fname, fn in untagged:
        print(f"  - {fname}:{fn} has no Implements tag and no justification")
    for lid in sorted(unimplemented, key=llr_sort_key):
        print(f"  - {lid} has no implementation tag and no exemption")
    if args.check and violations:
        print("FAIL: code-to-LLR trace violations present")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
