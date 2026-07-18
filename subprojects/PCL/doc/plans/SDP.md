# PCL Software Development Plan (SDP)

| Field | Value |
|-------|-------|
| Document | PCL-SDP |
| Status | **Draft — initial DAL C pass, 2026-07-06** |
| Applies to | `subprojects/PCL` |

Describes the development processes actually used for PCL, per DO-178C
section 11.2. Where current practice falls short of the DAL C objective, the
shortfall is stated rather than papered over.

## 1. Standards

| Activity | Standard |
|----------|----------|
| High-level requirements | `../standards/requirements_standard.md` |
| Low-level requirements | `../standards/requirements_standard.md` |
| Design | `../standards/design_standard.md` |
| C source code | `../standards/c_coding_standard.md` |

## 2. Software Development Life Cycle

PCL uses an incremental life cycle with the following per-change flow:

1. **Requirements.** New behaviour is captured first as an HLR in
   `../requirements/HLR.md` (with rationale and design-decision trace) and
   decomposed into LLRs in `../requirements/LLR.md` (with HLR trace and a
   verification pointer). Derived requirements are flagged in
   `../requirements/derived_requirements.md` and fed back to the (future)
   system safety process.
2. **Design.** Architecture-level design lives in `../architecture/`
   (`component_container_design.md`, `pcl-component-system.md`) and the
   public headers (`include/pcl/*.h`), which are the authoritative interface
   specification. Design changes update these before or with the code.
3. **Coding.** Strict C17, zero external dependencies, no STL, per the
   C coding standard. The C ABI is the compatibility boundary (D4).
4. **Integration.** Static libraries (`pcl_core` + per-transport libraries)
   are integrated via CMake; the executable environment is supplied by the
   embedding programme.

Each change lands as a git commit (or pull-request series) containing the
requirement, design, code, and test updates together, and must keep the full
test suite green and the traceability matrix gap-free
(`scripts/gen_hlr_coverage.py --check`).

## 3. Requirements Process

- **HLR identification**: `PCL.NNN` (optional letter suffix for post-hoc
  insertions). Each HLR has a title, a "shall" statement, and a rationale.
- **LLR identification**: `REQ_PCL_NNN`. Each LLR has a "shall" statement at
  the level of specific C functions and observable behaviour, a **Traces**
  field naming parent HLR(s), and a **Verification** field naming the
  test file and case(s).
- **Trace closure** is machine-checked: `scripts/gen_hlr_coverage.py`
  regenerates `doc/reports/PCL/HLR_COVERAGE.md` and fails `--check` when any
  HLR lacks an LLR, any LLR lacks a parent HLR or test evidence, or any test
  tag names a nonexistent LLR.
- **Derived requirements** (no parent above HLR level) are recorded in the
  derived-requirements register with the reason they exist and their safety
  relevance.

## 4. Design Process

The design description comprises:

- Design decisions D1–D9 in `HLR.md` (the architectural commitments).
- `../architecture/component_container_design.md` — container/executor
  internals, data structures, state machine.
- `../architecture/pcl-component-system.md` — component-system overview.
- Public headers — interface contracts, threading rules, ownership rules
  (documented per function).

Known DAL C shortfall: resource budgets (stack, heap bound, tick timing) are
asserted, not analysed. The analyses are planned in SVP section 6.

## 5. Coding Process

- Compiler discipline: builds warning-clean under MSVC and GCC; C17 with
  extensions off (`C_EXTENSIONS OFF`).
- Defensive-programming rules, banned constructs, and the allocator rule
  (all variable-length ABI-crossing buffers through `pcl_alloc`/`pcl_free`)
  are in the C coding standard.
- Requirement tags: test code carries `///< REQ_PCL_NNN` tags. Production
  source code carries function-level `Implements: REQ_PCL_NNN` trace
  comments (C coding standard, rule 5.6). The code-to-LLR trace is
  machine-checked in both directions by
  `scripts/gen_code_trace.py --check`, which also generates the matrix
  `doc/reports/PCL/CODE_TO_LLR.md` (closes the mechanism for gap analysis
  GAP-C-08).

## 6. Development Environment

| Item | Tool |
|------|------|
| Host toolchains | MSVC (VS 2022 x64), GCC 13 (Linux), GNAT/MinGW (Ada consumers) |
| Build system | CMake ≥ 3.21, presets at repo root |
| Test framework | GoogleTest v1.14.0 (verification environment only) |
| Version control | git (GitHub remote) |

The target computer environment (expected: ASAAC/APOS RTOS) is not yet
defined; compilation options for the certified target will be recorded in the
SECI when it is (see SCMP).

## 7. Traceability Summary

```
HLR (PCL.NNN)  <—Traces—  LLR (REQ_PCL_NNN)  <—tags/Verification—  test case
      |                        |
      +—— Design decisions ————+—— source file (file-level today;
          (D1–D9)                  function-level tagging is an open item)
```
