# PCL Requirements and Coverage Completion

Status: **complete** (2026-07-18). This file previously recorded the state
left by the interrupted checkpoint commit
`782f0cbf96646d9078e4806220827b7a6c4b89d8`; all four work packages it
described have been finished on branch
`claude/pcl-requirements-coverage-i88ksr`. This section is the final
handoff; the detailed evidence lives in the referenced reports.

## Final Handoff

**Requirements trace coverage.** 97 HLRs, 444 LLRs, zero trace gaps and
zero quality issues (`gen_hlr_coverage.py --check`). Every HLR and LLR
contains exactly one "shall"; the 78 multi-"shall" LLRs were rewritten (38)
or split into new identifiers (40, using previously deleted numbers). The
generator now enforces the quality rules mechanically: duplicate or
letter-suffixed identifiers, wrong "shall" counts, implementation
identifiers in HLR text, and higher-layer concepts in HLR text all fail
`--check`. The stale hand-maintained summary in `LLR.md` was replaced by a
pointer to the generated `doc/reports/PCL/HLR_COVERAGE.md`. The mis-tagged
route/stream validation test in `test_pcl_robustness.cpp` (REQ_PCL_179) was
retagged to the requirements it actually verifies. No letter-suffixed
identifiers remain anywhere in the repository.

**Statement and branch coverage.** 100% statement coverage, 5,072/5,072
lines, over all 18 measured production sources, from one clean full-suite
run (counters deleted first, every binary run exactly once, no test-order
dependence). Branch coverage 76% (2,930/3,846) recorded as the DAL B
planning input. Commands, tool versions (gcovr 8.6, GCC 13.3.0), and
per-file tables are in `doc/reports/PCL/COVERAGE_REPORT.md`. The coverage
harness now calls `enable_testing()`, so `ctest --test-dir
build-coverage-pcl` runs the suite.

**Test results.** 499 GoogleTest cases across 22 PCL test executables, all
passing on the evidence run.

**Accepted coverage exclusions.** 104 exclusion sites remain, each stating
its invariant inline, catalogued in six categories in
`COVERAGE_REPORT.md` (OS resource faults, non-injectable heap exhaustion,
defensive guards against corrupted shared state, teardown reclaim,
caller-proven preconditions, and non-deterministic congestion retries).
One former exclusion in `pcl_executor.c` failed review — its line is
reachable through the public API — and was replaced by the test
`LateDefaultTransportSkipsPeerSelectedSubscriber`. The dispositions are
engineering analysis; the formal independent review cycle is tracked as
GAP-C-07.

**GAP-C-08 status.** Closed. Every non-static production function carries
a function-level `Implements: REQ_PCL_NNN` tag or a reviewed `No LLR:`
justification (C coding standard rule 5.6), machine-checked in both
directions by `subprojects/PCL/scripts/gen_code_trace.py --check` (zero
violations) and reported in `doc/reports/PCL/CODE_TO_LLR.md`. Four
cross-cutting LLRs carry documented `**Implementation**: test-only`
exemptions. Three LLRs whose wording had drifted from verified behaviour
(REQ_PCL_189, REQ_PCL_305, REQ_PCL_308) were corrected during the mapping
review. See `doc/reports/PCL/DO178C_GAP_ANALYSIS.md`, section 1b, for the
disposition and for the refreshed GAP-C-13 and GAP-B-01 entries.

## Remaining Follow-On Work (outside this task)

- Wire `gen_hlr_coverage.py --check`, `gen_code_trace.py --check`, the
  build, the test suite, and the coverage run into CI (SCMP item;
  GAP-C-16).
- Run the formal review cycle with named reviewers over the requirements,
  the code-to-LLR mapping, and the coverage-exclusion dispositions
  (GAP-C-07).
- Target-environment testing (GAP-C-11) and the remaining open items in
  `DO178C_GAP_ANALYSIS.md`.
