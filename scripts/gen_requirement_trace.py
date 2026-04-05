#!/usr/bin/env python3
"""
Generate HLR → LLR → test traceability report for tactical_objects.

Reads HLR_COVERAGE.md, LLR.md, and scans test files for ///< REQ_/TOBJ/PYR-RESP tags.
Produces a completeness report showing requirement-to-test mapping.
"""

import re
import sys
from pathlib import Path


def parse_hlr_coverage(content: str) -> dict:
  """Extract TOBJ|Description|Tests from HLR_COVERAGE.md tables."""
  result = {}
  in_table = False
  for line in content.splitlines():
    if "| TOBJ | Description |" in line or "| TOBJ | Description | Unit" in line:
      in_table = True
      continue
    if in_table and line.startswith("|") and "---" not in line:
      parts = [p.strip() for p in line.split("|") if p.strip()]
      if len(parts) >= 2:
        tobj = parts[0]
        desc = parts[1] if len(parts) > 1 else ""
        tests = parts[2] if len(parts) > 2 else ""
        if tobj.startswith("TOBJ.") or tobj.startswith("RESP.") or tobj.startswith("PYR-RESP-"):
          result[tobj] = {"description": desc, "tests": tests}
    elif in_table and not line.strip():
      in_table = False
  return result


def parse_llr(content: str) -> dict:
  """Extract REQ_TACTICAL_OBJECTS_NNN with Traces and Verification from LLR.md."""
  result = {}
  current_req = None
  current_traces = []
  current_verification = ""

  for line in content.splitlines():
    m = re.match(r"### (REQ_TACTICAL_OBJECTS_\d+) - (.+)", line)
    if m:
      if current_req:
        result[current_req] = {
          "title": current_title,
          "traces": current_traces,
          "verification": current_verification.strip(),
        }
      current_req = m.group(1)
      current_title = m.group(2)
      current_traces = []
      current_verification = ""
      continue

    if current_req:
      if line.strip().startswith("**Traces**:"):
        trace_str = line.split(":", 1)[1].strip()
        current_traces = [t.strip() for t in trace_str.split(",")]
      elif line.strip().startswith("**Verification**:"):
        current_verification = line.split(":", 1)[1].strip()

  if current_req:
    result[current_req] = {
      "title": current_title,
      "traces": current_traces,
      "verification": current_verification.strip(),
    }
  return result


def scan_test_tags(test_dir: Path) -> dict:
  """Scan test files for ///< REQ_ TOBJ. PYR-RESP/RESP tags."""
  req_to_tests = {}
  tobj_to_tests = {}
  resp_to_tests = {}

  for f in test_dir.glob("Test_*.cpp"):
    content = f.read_text(encoding="utf-8", errors="replace")
    test_name = f.stem
    for line in content.splitlines():
      m = re.search(r"///<\s*(REQ_TACTICAL_OBJECTS_\d+|TOBJ\.\d+|PYR-RESP-\d+|RESP\.\d+):", line)
      if m:
        tag = m.group(1)
        if tag.startswith("REQ_"):
          req_to_tests.setdefault(tag, []).append(test_name)
        elif tag.startswith("TOBJ."):
          tobj_to_tests.setdefault(tag, []).append(test_name)
        elif tag.startswith("RESP.") or tag.startswith("PYR-RESP-"):
          resp_to_tests.setdefault(tag, []).append(test_name)

  return {"req": req_to_tests, "tobj": tobj_to_tests, "resp": resp_to_tests}


def main():
  root = Path(__file__).resolve().parent.parent
  tobj_dir = root / "pyramid" / "tactical_objects"
  test_dir = root / "tests" / "tactical_objects"

  if not tobj_dir.exists():
    print("tactical_objects dir not found", file=sys.stderr)
    sys.exit(1)

  hlr_coverage = (tobj_dir / "HLR_COVERAGE.md").read_text(encoding="utf-8", errors="replace")
  llr_content = (tobj_dir / "LLR.md").read_text(encoding="utf-8", errors="replace")

  hlr = parse_hlr_coverage(hlr_coverage)
  llr = parse_llr(llr_content)
  tags = scan_test_tags(test_dir) if test_dir.exists() else {"req": {}, "tobj": {}, "resp": {}}

  lines = []
  lines.append("# HLR → LLR → Test Traceability Report")
  lines.append("")
  lines.append("Generated from HLR_COVERAGE.md, LLR.md, and test file tags.")
  lines.append("")
  lines.append("## Summary")
  lines.append("")
  hlr_count = len(hlr)
  llr_count = len(llr)
  llr_with_tests = sum(1 for r, d in llr.items() if d["verification"])
  llr_tagged = sum(1 for r in llr if r in tags["req"])
  lines.append(f"| Metric | Count |")
  lines.append(f"|--------|-------|")
  lines.append(f"| HLR (TOBJ/PYR-RESP) in HLR_COVERAGE | {hlr_count} |")
  lines.append(f"| LLR (REQ_TACTICAL_OBJECTS_*) | {llr_count} |")
  lines.append(f"| LLR with verification test specified | {llr_with_tests} |")
  lines.append(f"| LLR with ///< tag in tests | {llr_tagged} |")
  lines.append("")
  lines.append("## HLR → Tests (from HLR_COVERAGE.md)")
  lines.append("")
  lines.append("| HLR | Description | Test(s) |")
  lines.append("|-----|-------------|---------|")
  for tobj in sorted(hlr.keys()):
    d = hlr[tobj]
    desc = d["description"][:60] + "…" if len(d["description"]) > 60 else d["description"]
    lines.append(f"| {tobj} | {desc} | {d['tests']} |")
  lines.append("")
  lines.append("## LLR → HLR → Test (from LLR.md)")
  lines.append("")
  lines.append("| LLR | Traces (HLR) | Verification | Tagged |")
  lines.append("|-----|--------------|--------------|--------|")

  for req in sorted(llr.keys(), key=lambda x: (int(re.search(r"\d+", x).group()) if re.search(r"\d+", x) else 0)):
    d = llr[req]
    traces = ", ".join(d["traces"]) if d["traces"] else "—"
    verif = d["verification"][:40] + "..." if len(d["verification"]) > 40 else d["verification"]
    tagged = "✓" if req in tags["req"] else ""
    lines.append(f"| {req} | {traces} | {verif} | {tagged} |")

  lines.append("")
  lines.append("## Test Files → Requirements (from ///< tags)")
  lines.append("")
  for test_file in sorted(set(
    t for tests in tags["req"].values() for t in tests
  )):
    reqs = [r for r, tests in tags["req"].items() if test_file in tests]
    tobjs = [t for t, tests in tags["tobj"].items() if test_file in tests]
    resps = [r for r, tests in tags["resp"].items() if test_file in tests]
    all_tags = reqs + tobjs + resps
    if all_tags:
      lines.append(f"- **{test_file}**: {', '.join(sorted(all_tags))}")

  out = root / "coverage_tactical_objects" / "requirement_traceability.md"
  out.parent.mkdir(parents=True, exist_ok=True)
  out.write_text("\n".join(lines), encoding="utf-8")
  print(f"Wrote {out}")


if __name__ == "__main__":
  main()
