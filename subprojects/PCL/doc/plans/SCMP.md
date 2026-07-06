# PCL Software Configuration Management Plan (SCMP)

| Field | Value |
|-------|-------|
| Document | PCL-SCMP |
| Status | **Draft — initial DAL C pass, 2026-07-06** |
| Applies to | `subprojects/PCL` |

Defines configuration management for PCL life-cycle data per DO-178C
section 11.4 and Table A-8.

## 1. Configuration Identification

Every configuration item is a path in the git repository; the configuration
identifier is the git commit hash (and tag, for baselines).

| Item class | Paths |
|------------|-------|
| Requirements | `subprojects/PCL/doc/requirements/HLR.md`, `LLR.md`, `derived_requirements.md` |
| Plans and standards | `subprojects/PCL/doc/plans/`, `subprojects/PCL/doc/standards/` |
| Design data | `subprojects/PCL/doc/architecture/`, `subprojects/PCL/include/pcl/` |
| Source code | `subprojects/PCL/src/`, `subprojects/PCL/bindings/` |
| Verification cases and procedures | `subprojects/PCL/tests/`, `cmake/pcl_coverage/`, `subprojects/PCL/scripts/` |
| Verification results and reports | `doc/reports/PCL/` |
| Review records | `doc/reviews/PCL/` |

## 2. Baselines

A PCL baseline is an annotated git tag `pcl-baseline-YYYYMMDD[-suffix]` on a
commit where: all tests pass on the supported host toolchains, statement
coverage is 100% or dispositioned, and the trace check is gap-free. Each
baseline tag message names the coverage/trace evidence commits. No baseline
tags exist yet; the first should be cut when the Phase 1 review cycle
completes.

**Software Configuration Index (SCI)**: for each baseline, the SCI is the
tagged tree itself plus a generated file listing (path + blob hash) of the
certified-part list (PSAC section 3.2). A generator script is an open item.

**Software life cycle Environment Configuration Index (SECI)**: toolchain
versions are recorded today in `COVERAGE_REPORT.md` (gcovr, GCC) and
`CMakePresets.json` (build options). A consolidated SECI per baseline is an
open item; the target-computer toolchain will be added when defined.

## 3. Change Control

- All changes land through git commits on branches, merged via GitHub pull
  requests (repo `Ackoogin/mujin`); the default branch is protected by
  review-before-merge convention.
- **Problem reports**: GitHub issues on the repository serve as the
  problem-report system. A PCL problem report must name the affected
  requirement(s) when known, and its fix commit must reference the issue.
  Problem-report discipline is currently informal — formal categorisation
  (safety-related vs. not) and a per-baseline open-problem summary are
  required before first certification liaison.
- CC1/CC2: all items in section 1 are treated as CC1 (full change control).
  Build outputs and coverage artifacts are CC2 (regenerable from a CC1
  baseline).

## 4. Build and Environment Reproducibility

Builds are reproducible from a clean checkout via CMake presets; third-party
verification-environment dependencies (GoogleTest) are pinned by tag in the
fetch declarations. **There is currently no CI pipeline in the repository** —
build/test reproducibility is demonstrated manually. Standing up CI that runs
the PCL suite, coverage, and `gen_hlr_coverage.py --check` on every change is
the highest-value open SCM item.

## 5. Archive, Retrieval, Release

The GitHub-hosted git repository (with its fork/clone redundancy) is the
archive. Retrieval of any baseline is `git checkout <tag>`. Release packaging
(the SDK packaging flow under `subprojects/PYRAMID/doc/architecture/sdk_packaging.md`)
is outside the airborne boundary and carries no certification credit.

## 6. Open Items

1. CI pipeline (build + test + coverage + trace check per change).
2. First baseline tag with SCI/SECI generation.
3. Formal problem-report categorisation and per-baseline summaries.
4. Certified-part-list generator (ties to PSAC 3.2 deactivated-code claims).
