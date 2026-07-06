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

1.1. **Identifier**: `PCL.NNN` with optional single-letter suffix for
     requirements inserted next to an existing one (`PCL.011a`). Identifiers
     are never reused or renumbered.
1.2. **Form**: a heading `### PCL.NNN - Title`, a normative statement using
     "shall", and a **Rationale** paragraph. Multi-clause requirements use
     lettered sub-clauses ((a), (b), …) so verification can reference each
     clause.
1.3. **Scope**: an HLR states externally observable behaviour or a binding
     design constraint — not implementation detail. Implementation detail
     belongs in LLRs.
1.4. **Design-decision trace**: each HLR appears in the design-decision
     traceability table at the bottom of `HLR.md`, mapping to D1–D9.
1.5. **Verifiability**: every HLR must be decomposable into testable LLRs;
     "support"/"provide" statements must name the concrete API or observable
     effect.

## 2. Low-Level Requirements (LLR)

2.1. **Identifier**: `REQ_PCL_NNN` (zero-padded to three digits in tags).
     Never reused or renumbered.
2.2. **Form**: a heading `### REQ_PCL_NNN - Title`, a "shall" statement
     naming the specific C function(s) and observable behaviour, then:
     - `**Traces**:` comma-separated parent HLR IDs (at least one, unless the
       LLR is registered as derived);
     - `**Verification**:` the verifying test(s), file-qualified
       (`test_file.cpp::TestCase`). Prose-only verification pointers are
       non-conforming (the trace generator cannot extract them).
2.3. **Granularity**: one testable behaviour per LLR. Error/robustness
     behaviour gets its own LLR rather than a clause inside the normal-case
     LLR where practical.
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
4.2. Withdrawn requirements keep their identifier with the statement replaced
     by `Withdrawn (date): reason` — the ID is never recycled.
4.3. After any requirements change, `scripts/gen_hlr_coverage.py --check`
     must pass.

## 5. Review Criteria

At review (checklist in `doc/reviews/PCL/checklists.md`), each requirement is
checked for: unambiguity, verifiability, consistency with neighbours,
conformance to this standard, correct trace links, and (HLR) rationale
quality / (LLR) accuracy against the named functions.
