# PCL Requirements and Coverage Completion

This file records the state left after resuming interrupted commit
`782f0cbf96646d9078e4806220827b7a6c4b89d8` on 2026-07-18. The intended
outcome is complete PCL requirements traceability, 100 percent statement
coverage, an updated DO-178C gap analysis, and a concrete resolution of
GAP-C-08.

The work in the checkpoint commit is intentionally incomplete. Do not treat
the generated trace matrix or the coverage exclusions as certification
approval.

## Completed in This Checkpoint

- Restricted the HLR coverage generator to PCL tests. PYRAMID protocol-binding
  tests and AME integration tests are no longer accepted as PCL evidence.
- Deleted PCL requirements that described generated protocol bindings and
  application-domain service names. The deleted HLR and LLR identifiers are
  available for reuse.
- Removed letter-suffixed HLR identifiers and reassigned their content to
  ordinary numeric identifiers:

  | Old ID | New ID |
  |--------|--------|
  | PCL.011a | PCL.085 |
  | PCL.011b | PCL.086 |
  | PCL.011c | PCL.087 |
  | PCL.030a | PCL.088 |
  | PCL.030b | PCL.089 |
  | PCL.030c | PCL.090 |
  | PCL.030d | PCL.091 |
  | PCL.036a | PCL.092 |
  | PCL.036b | PCL.093 |
  | PCL.036c | PCL.094 |
  | PCL.036d | PCL.095 |
  | PCL.036e | PCL.096 |
  | PCL.036f | PCL.097 |
  | PCL.036g | PCL.098 |
  | PCL.078a | PCL.099 |
  | PCL.078b | PCL.100 |
  | PCL.082a | PCL.101 |
  | PCL.084a | PCL.102 |

- Split shared-memory atomic fan-out and topic backpressure into PCL.098 and
  PCL.103.
- Rewrote every HLR normative paragraph so that it contains exactly one
  "shall" and does not name PCL functions, types, status codes, or source
  identifiers.
- Updated the requirements standard to prohibit suffix IDs, allow reuse of
  deleted IDs, require one testable HLR statement, and keep higher-layer
  concepts outside PCL requirements.
- Added tests for invalid route and stream definitions, executor route
  validation, plugin-loader arguments, routing-manifest edge cases, malformed
  UDP datagrams, remote stream dispatch, and truncated socket payloads.
- Added narrowly scoped statement-coverage exclusions for paths that appear
  unreachable through bounded internal callers. These exclusions still need
  independent review before they can be accepted as verification evidence.

## Required Next Work

### 1. Finish Requirement Quality Review

- Regenerate `doc/reports/PCL/HLR_COVERAGE.md` after the PCL.103 split and
  confirm that every HLR has at least one LLR and every LLR has PCL-local test
  evidence.
- Add an automated HLR quality check to
  `subprojects/PCL/scripts/gen_hlr_coverage.py`. It should reject duplicate
  identifiers, suffix identifiers, normative paragraphs with other than one
  "shall", implementation identifiers in HLR text, and references to concepts
  owned by higher layers.
- Review the LLR corpus for the same single-behaviour rule. At this checkpoint,
  78 of 403 LLRs contain more than one "shall"; split or rewrite them without
  weakening their observable acceptance criteria.
- Remove the stale hand-maintained traceability summary inside `LLR.md` or
  replace it with a link to the generated report.
- Review test trace comments for accuracy. In particular,
  `test_pcl_robustness.cpp` currently tags a route/stream input-validation test
  with REQ_PCL_179, which is not the best matching requirement.
- Search the complete repository, not only PCL, for old suffixed IDs:

  ```text
  rg -n "PCL\.[0-9]+[a-z]" .
  ```

### 2. Complete and Reproduce 100 Percent Statement Coverage

The last clean run before the final requirement edits reported 5,067 of 5,073
statements, or 99 percent. All 22 PCL test executables passed in that run, for
498 GoogleTest cases. The current worktree has not been rebuilt after the last
test and exclusion edits.

The remaining reported locations were:

- `pcl_container.c:505`;
- `pcl_executor.c:243`;
- `pcl_transport_routing.c:280`;
- `pcl_transport_socket.c:874-875`;
- `pcl_transport_socket.c:888`.

The first three now have either added test input or a proposed exclusion and
must be rechecked. The socket reconnect lines are timing-dependent. The
existing focused tests
`PclSocketTransport.AutoReconnectAfterServerRestart` and
`PclSocketFaults.AutoReconnectBackoffRunsWhileServerStaysDown` exercised them
when run separately. Make the clean full-suite coverage result reproducible;
do not rely on a stale counter or on test ordering.

Use the existing WSL coverage build:

```text
wsl.exe bash -lc "cd /mnt/d/Dev/repo/mujin && cmake --build build-coverage-pcl-wsl-20260717 -j\$(nproc)"
```

Clear old `.gcda` files before the evidence run, run every PCL test executable,
and then run gcovr with the source filter documented in
`doc/reports/PCL/COVERAGE_REPORT.md`. The coverage-only CMake harness does not
currently call `enable_testing()`, so `ctest` reports no tests; either fix the
harness or continue to invoke all 22 executables explicitly.

Review every `GCOVR_EXCL_START` block against its stated invariant. If a path
can be reached through a public API, cover it with a test instead of excluding
it. Record both statement and branch results and update
`doc/reports/PCL/COVERAGE_REPORT.md` with the exact commands, tool versions,
source-file count, test count, and final totals.

### 3. Address GAP-C-08 With Direct Code-to-LLR Traceability

GAP-C-08 is still open. Test-to-LLR-to-HLR traceability is not a substitute
for code-to-LLR traceability.

Implement a mechanically checked trace from each production implementation
unit to its governing `REQ_PCL_NNN` requirements. Prefer function-level source
annotations because that is the convention already described in the PCL
coding standard. Add a generator or checker that:

- extracts requirement tags from PCL production source;
- rejects tags naming absent LLRs;
- reports production functions with no LLR tag;
- reports LLRs with no implementation tag unless they are explicitly
  test-only or documentation-only requirements; and
- writes a reviewable code-to-LLR matrix under `doc/reports/PCL/`.

Do not infer implementation trace solely from test coverage. Review the
generated mapping against the LLR wording, then update the requirements
standard, coding standard, and lifecycle plans so the check is repeatable.

### 4. Update the DO-178C Gap Analysis

Update `doc/reports/PCL/DO178C_GAP_ANALYSIS.md` only after the evidence above
is regenerated.

- Replace obsolete HLR/LLR/test and coverage counts.
- Record the removal of higher-layer protocol requirements and suffix IDs.
- Update the statement-coverage status with reproducible evidence.
- Mark GAP-C-08 closed only if the direct code-to-LLR checker passes with no
  unexplained gaps. Otherwise describe the implemented partial control and
  leave the residual gap open.
- Review the status of GAP-C-13 and every other entry affected by the new
  requirements and coverage evidence.

## Checkpoint Validation

Run these checks before the completion commit:

```text
python subprojects/PCL/scripts/gen_hlr_coverage.py --check
rg -n "PCL\.[0-9]+[a-z]" .
git diff --check
```

Then run the clean coverage build and all PCL tests. The final handoff should
state separately:

- requirements trace coverage;
- statement and branch coverage;
- test results;
- accepted coverage exclusions; and
- GAP-C-08 status.
