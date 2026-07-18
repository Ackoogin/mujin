# PCL Software Verification Plan (SVP)

| Field | Value |
|-------|-------|
| Document | PCL-SVP |
| Status | **Draft — initial DAL C pass, 2026-07-06** |
| Applies to | `subprojects/PCL` |

Defines the verification processes for PCL against the DO-178C Table A-3
through A-7 objectives at DAL C, and identifies the DAL B delta where it
changes the method.

## 1. Verification Methods Overview

| Objective area | Method | Evidence location |
|----------------|--------|-------------------|
| HLR / LLR / design / code reviews | Checklisted reviews | `doc/reviews/PCL/` (repo root) |
| Requirements-based testing | GoogleTest suites, requirement-tagged | `tests/`, plus PYRAMID proto-binding and AME integration tests |
| Trace closure | Generated matrix, machine-checked | `doc/reports/PCL/HLR_COVERAGE.md` via `scripts/gen_hlr_coverage.py --check` |
| Structural coverage | gcovr statement coverage, gap disposition | `doc/reports/PCL/COVERAGE_REPORT.md` |
| Concurrency / coupling analysis | Mutex audit + data/control coupling analysis | `doc/reports/PCL/MUTEX_AUDIT.md` (coupling analysis: open item) |

## 2. Reviews (Tables A-3, A-4, A-5)

Reviews use the checklists in `doc/reviews/PCL/checklists.md` and record
results on the template in `doc/reviews/PCL/review_record_template.md`. A
review record names the item and its git revision, the checklist used, the
reviewer(s), findings, and disposition.

DAL C requires review evidence for: HLR accuracy/consistency/verifiability
and standard conformance; LLR same plus HLR compliance and trace; architecture
compatibility with HLRs; code compliance with LLRs, architecture, and the
coding standard; and trace accuracy at every link.

**Independence (DAL B delta)**: at DAL B most of these reviews must be
performed by someone other than the developer of the item. Review records
carry an "independent: yes/no" field from the start so the DAL B position is
visible without re-review.

Current status: no completed review records exist yet. The first review cycle
over the existing baseline is the largest open Phase 1 work item.

## 3. Requirements-Based Testing (Table A-6)

- Every LLR names its verifying test(s) in its **Verification** field; every
  test carries `///< REQ_PCL_NNN` tags. Normal-range and robustness cases are
  both required; robustness cases include NULL/invalid arguments, capacity
  overflow, allocation-failure injection (`test_pcl_oom`,
  `test_pcl_socket_faults`), malformed wire frames, and cross-thread stress.
- Conformance suites: every transport must pass the delivery conformance
  suite (PCL.072) and the threading-model conformance suite (PCL.076).
- **Test-case specification shortfall**: test cases exist as reviewed code
  with requirement tags, not as separate input/expected-result specification
  documents. For DAL C credit PCL treats the tagged test source as the test
  case and procedure description; this position must be recorded and accepted
  in the (future) certification liaison, or the specifications must be
  extracted. Retained per-baseline test *results* start with the
  `COVERAGE_REPORT.md` run records; a per-baseline results log is required
  from the next baseline onward (see SCMP).
- **Target testing**: all evidence is host-based today. The credit-seeking
  campaign must be re-run on the target or a justified representative
  environment once the target is defined.

## 4. Structural Coverage (Table A-7)

- **DAL C objective: statement coverage.** Measured with gcovr/GCC using the
  harness in `cmake/pcl_coverage/`. Current baseline: 100% statement coverage
  (4300/4300 lines, 2026-07-06, all 21 test binaries passing). Any future
  uncovered line must be dispositioned as (a) additional test, (b) deactivated
  code, or (c) documented analysis, in `COVERAGE_REPORT.md`.
- **DAL B delta: decision coverage.** Branch coverage is recorded
  informationally (73% on the current baseline) to size the work; credit
  requires a decision-coverage-capable qualified tool.
- **Data/control coupling coverage** (A-7 objective 8, required at DAL C):
  a systematic analysis of the executor ↔ container ↔ transport ↔ plugin
  coupling exercised by the test suite. Open item; `MUTEX_AUDIT.md` is the
  seed input.

## 5. Trace Verification

`scripts/gen_hlr_coverage.py --check` fails on any of: HLR without LLR, LLR
without parent HLR, LLR without test evidence, test tag naming an unknown
LLR. It runs on demand today; wiring it into CI is an SCMP item.

`scripts/gen_code_trace.py --check` verifies the code-to-LLR direction: it
fails on a source tag naming an unknown LLR, a non-static production
function with neither an `Implements:` tag nor a reviewed `No LLR:`
justification, or an LLR with no implementing function and no
`**Implementation**:` exemption marker in `LLR.md`. Run without `--check`
it regenerates the reviewable matrix `doc/reports/PCL/CODE_TO_LLR.md`. The
same CI wiring item applies.

## 6. Analyses

Required analyses not yet performed (all open items):

| Analysis | Scope | Priority |
|----------|-------|----------|
| Stack usage | Worst-case depth of executor tick path and transport worker threads | Phase 1 |
| Timing | Tick-loop budget vs. PCL.015 rate; jitter under ingress load | Phase 1 |
| Memory bounds | Heap growth bound for queue nodes and deep-copied ingress under stated arrival-rate assumptions | Phase 1 |
| Data/control coupling | See section 4 | Phase 1 |
| Worst-case arithmetic | Overflow analysis of size arithmetic (seeded by `pcl_alloc` overflow rejection) | Phase 1 |

## 7. Verification Environment and Tool Qualification

| Tool | Use | Credit taken | Qualification position |
|------|-----|--------------|------------------------|
| gcovr 8.6 / GCC gcov | Structural coverage measurement | Yes (A-7) | Needs TQL-5 qualification or documented independent output checking — open |
| GoogleTest 1.14 / CTest | Test execution and pass/fail | Yes (A-6) | Same — open |
| `gen_hlr_coverage.py` | Trace matrix generation | Yes (A-6 trace objectives) | Small in-repo tool; qualify by review + output spot-check against the source documents — open |
| CMake, compilers | Build | No direct credit | Compiler output is verified by the testing it enables (standard DO-178C position) |

## 8. Pass/Fail and Regression Policy

A verification baseline passes when: all test binaries exit 0 with zero
failed tests on every supported host toolchain; statement coverage is 100%
or every gap is dispositioned; and the trace check reports zero gaps. Any
source change re-runs the full suite (no test selection), because the
deterministic-executor contract makes cross-component regressions cheap to
catch and the suite is fast (< 2 minutes).
