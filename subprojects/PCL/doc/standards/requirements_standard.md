# PCL Requirements Standard

| Field | Value |
|-------|-------|
| Document | PCL-RS |
| Status | **Draft — initial DAL C pass, 2026-07-06** |
| Applies to | `subprojects/PCL/doc/requirements/HLR.md`, `LLR.md`, `derived_requirements.md` |

Rules for writing and maintaining PCL requirements, per DO-178C section 11.6.
The existing HLR/LLR corpus largely follows these rules already; the standard
makes them checkable at review.

## 1. High-Level Requirements (HLR)

1.1. **Identifier**: `PCL.NNN`. Letter suffixes are not permitted. The active
     baseline shall not contain duplicate identifiers. An identifier removed
     from the PCL baseline may be assigned to a future PCL requirement.
1.2. **Form**: a heading `### PCL.NNN - Title`, one testable normative
     statement containing exactly one "shall", and a **Rationale** paragraph.
     Split independent behaviours into separate requirements.
1.3. **Scope**: an HLR states externally observable behaviour or a binding
     design constraint, not implementation detail. Function names, type names,
     status-code names, source identifiers, and details owned by a layer above
     PCL belong in LLRs or in the requirements of the owning layer.
1.4. **Design-decision trace**: each HLR appears in the design-decision
     traceability table at the bottom of `HLR.md`, mapping to D1–D9.
1.5. **Verifiability**: every HLR must be decomposable into testable LLRs.
     A "support" or "provide" statement must identify an observable effect
     without prescribing its implementation.

## 2. Low-Level Requirements (LLR)

2.1. **Identifier**: `REQ_PCL_NNN` (zero-padded to three digits in tags).
     The active baseline shall not contain duplicates. An identifier removed
     from the PCL baseline may be assigned to a future PCL requirement.
2.2. **Form**: a heading `### REQ_PCL_NNN - Title`, a "shall" statement
     naming the specific C function(s) and observable behaviour, then:
     - `**Traces**:` comma-separated parent HLR IDs (at least one, unless the
       LLR is registered as derived);
     - `**Verification**:` the verifying test(s), file-qualified
       (`test_file.cpp::TestCase`). Prose-only verification pointers are
       non-conforming (the trace generator cannot extract them).
2.3. **Granularity**: one testable behaviour per LLR, and exactly one
     "shall" per normative paragraph, the same rule that applies to HLRs.
     Error/robustness behaviour gets its own LLR rather than a clause
     inside the normal-case LLR where practical. When a review finds more
     than one "shall" in an LLR, either rewrite it as a single compound
     "shall" statement if the clauses describe one observable behaviour
     (for example, a bounded value and the rejection that follows from it),
     or split it into a second LLR with a fresh identifier if the clauses
     describe independently testable behaviours (for example, a function's
     success path and its null-argument path). A split must keep the
     original identifier on the first behaviour, give the new behaviour an
     unused `REQ_PCL_NNN` identifier, and preserve the parent HLR trace on
     both.
2.4. **Test tags**: each verifying test carries
     `///< REQ_PCL_NNN: description. PCL.0XX.` adjacent to the test case.

## 3. Derived Requirements

3.1. A requirement with no parent above HLR level (or an LLR whose substance
     is not demanded by any HLR) is **derived** and must be registered in
     `derived_requirements.md` with: the reason it exists, and its potential
     safety impact for feedback to the system safety process.
3.2. Derived status does not exempt a requirement from verification or
     trace rules.

## 4. Change Rules

4.1. A requirement change, addition, or withdrawal lands in the same change
     set as the code/test change it motivates.
4.2. Removing a requirement also removes its test, code, and parent traces.
     The trace checks shall confirm that no active reference to the removed
     identifier remains.
4.3. After any requirements change, `scripts/gen_hlr_coverage.py --check`
     must pass.

## 5. Review Criteria

At review (checklist in `doc/reviews/PCL/checklists.md`), each requirement is
checked for: unambiguity, verifiability, consistency with neighbours,
conformance to this standard, correct trace links, and (HLR) rationale
quality / (LLR) accuracy against the named functions.

## 6. Automated Quality Gate

`subprojects/PCL/scripts/gen_hlr_coverage.py --check` is the mechanically
checked gate for this standard and must pass before any requirements change
is accepted. In addition to the trace-gap checks (every HLR has at least
one LLR, every LLR has PCL-local test evidence and a parent HLR trace), the
same `--check` run enforces:

- **No duplicate identifiers** (rule 1.1 / 2.1): a `PCL.NNN` or
  `REQ_PCL_NNN` heading may not appear twice in the active baseline.
- **No suffix identifiers** (rule 1.1 / 2.1): a numeric identifier followed
  directly by a letter, with no separator, is rejected wherever it appears
  in `HLR.md` or `LLR.md`, not only in headings.
- **Exactly one "shall"** per HLR normative paragraph (rule 1.2) and per LLR
  normative paragraph (rule 2.3). Zero or more than one "shall" fails the
  check.
- **No implementation identifiers in HLR text** (rule 1.3): the checker
  rejects PCL C function/macro names (`pcl_*`), C++ wrapper type names
  (`Pcl*`), status-code or other ALL_CAPS constants (`PCL_*`), and source
  file names (`*.c`, `*.h`) appearing in an HLR's normative paragraph. LLRs
  are expected to name these identifiers and are not checked for this rule.
- **No higher-layer concepts in HLR text** (rule 1.3): the checker rejects
  HLR normative paragraphs that mention PYRAMID wire-protocol details,
  generated service bindings, ROS2/DDS specifics, or AME application
  concepts. These belong in the requirements of the layer that owns them,
  not in PCL's HLRs.

The checker's HLR and LLR rules only examine the normative paragraph (the
text between the heading and `**Rationale**`/`**Traces**`); rationale text
and the Design Decisions section (`D1`-`D9`) may reference implementation
identifiers or other systems for context without failing the gate.

Quality failures are listed under "Quality Checks" in the generated
`doc/reports/PCL/HLR_COVERAGE.md` and printed to the console; `--check`
exits non-zero when any are present, exactly as it does for trace gaps.
