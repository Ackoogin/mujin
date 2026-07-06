# PCL Software Quality Assurance Plan (SQAP)

| Field | Value |
|-------|-------|
| Document | PCL-SQAP |
| Status | **Draft — initial DAL C pass, 2026-07-06** |
| Applies to | `subprojects/PCL` |

Defines quality assurance for PCL per DO-178C section 11.5 and Table A-9.

## 1. Scope and Honesty Statement

PCL is currently developed by a very small team without a separate QA
organisation. This plan therefore defines QA activities that are performable
now (process conformity checks, records audits) and flags where DAL C/B
expectations require organisational independence that does not yet exist.
Records of QA activities go in `doc/reviews/PCL/` alongside review records,
using the same record template with type = "QA audit".

## 2. QA Activities

| Activity | When | What is checked |
|----------|------|-----------------|
| Process conformity audit | Per baseline | Changes followed the SDP flow (requirement → design → code → test in one change set); reviews recorded; trace check green; coverage dispositioned |
| Life-cycle data audit | Per baseline | Documents in `doc/plans/`, `doc/standards/`, `doc/requirements/` are internally consistent, dated, and match actual practice |
| Standards compliance sampling | Per baseline | Sampled source files against the C coding standard; sampled LLRs against the requirements standard |
| Conformity review | Before any release/certification submission | DO-178C section 8.3 conformity review of the release baseline |

## 3. Independence

DO-178C Table A-9 requires QA independence at all levels including DAL C.
Currently all artifacts are effectively single-author, so genuine QA
independence **does not exist yet**. Interim position: QA audits are
performed by a person (or at minimum a session/review pass) distinct from the
author of the audited change, and every record states the auditor identity
and whether independence held. This is an explicitly declared shortfall, not
a claim of compliance — closing it is an organisational decision tracked as
GAP-C-17/GAP-B-02 in the gap analysis.

## 4. Deviations

Deviations from the plans or standards are recorded in the review/QA record
that found them, with disposition (fix, or documented-and-accepted deviation
with rationale). Accepted deviations against the C coding standard live in
the standard's own deviation register (its section 6), mirroring MISRA
practice.

## 5. Supplier Control

Not applicable — no software suppliers. Verification-environment tools are
controlled through the SECI (see SCMP section 2).
