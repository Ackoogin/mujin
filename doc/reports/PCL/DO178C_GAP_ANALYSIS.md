# PCL DO-178C Gap Analysis (DAL C and DAL B)

Generated: 2026-07-04

Scope: `subprojects/PCL` (the PYRAMID Composition Library runtime: `pcl_core`,
reference transports, plugin loader, routing, C++ wrappers, Ada binding) assessed
against the DO-178C objectives for Design Assurance Level C and Level B.

This is an engineering gap analysis, not a certification statement. Objective
counts reference DO-178C Annex A tables (A-1 through A-10). DAL C requires 62
objectives (5 with independence); DAL B requires 69 objectives (18 with
independence), adding decision coverage and independence on many verification
objectives.

---

## 1. Evidence Inventory (What Exists Today)

| DO-178C life-cycle data | PCL artifact | Status |
|---|---|---|
| High-level requirements | `subprojects/PCL/doc/requirements/HLR.md` (PCL.001-PCL.074, with rationale and design-decision trace) | Present, current |
| Low-level requirements | `subprojects/PCL/doc/requirements/LLR.md` (362 LLRs, REQ_PCL_001-447, each with HLR trace + verification pointer) | Present, current |
| Design description | `doc/architecture/component_container_design.md`, `08-pcl-component-system.md` | Present (informal) |
| Source code | 15 C17 source files (~9.2k LOC), zero external deps, no `assert`, no setjmp | Present |
| Requirements-based tests | 29 test files, ~293 tests: normal, robustness, OOM injection, socket fault injection, threading, conformance suite | Present, strong |
| Test-to-requirement trace | `///< REQ_PCL_NNN` tags in tests; `doc/reports/PCL/HLR_COVERAGE.md` matrix | Present but **stale** (matrix stops at PCL.056; HLR now reaches PCL.074) |
| Structural coverage | `doc/reports/PCL/COVERAGE_REPORT.md`: 92% line coverage via gcovr/GCC, with uncovered-line analysis | Present but **stale** (predates codec/plugin/routing/capabilities/alloc sources) and **statement-only** |
| Concurrency analysis | `doc/reports/PCL/MUTEX_AUDIT.md` (lock hierarchy, deadlock analysis, unprotected-state contract) | Present — useful input to data/control coupling analysis |
| Coding standard | `doc/CODING_STYLE.md` | Present but **C++-oriented; does not govern the C17 core** |
| Plans (PSAC/SDP/SVP/SCMP/SQAP) | — | **Absent** |
| Requirements/design/code standards | — | **Absent** |
| Review records (HLR/LLR/design/code) | — (`doc/reviews/` contains AME and PYRAMID only) | **Absent** |
| SCM records (baselines, problem reports, SCI/SECI) | git history only; no CI in repo (`.github/` absent) | **Absent** |
| SQA records | — | **Absent** |
| Tool qualification data | — (gcovr, GoogleTest, CMake/CTest used for verification credit) | **Absent** |
| System-level requirements allocated to software | — (HLR is the top of the stack) | **Absent** |

The engineering culture is already certification-friendly: strict lifecycle state
machine, bounded port/parameter capacities, fail-closed compose-time validation
(D9), deterministic single-threaded execution (D2), a portable allocator with
overflow rejection, and a requirements-tagged test suite. The gaps are mostly
process/evidence gaps plus a handful of technical items listed in section 5.

---

## 2. Gaps Against DAL C (Annex A objectives)

### A-1 Planning (all 7 objectives open)

- **GAP-C-01 — No PSAC.** No Plan for Software Aspects of Certification defining
  software level, life cycle, and the certification liaison approach.
- **GAP-C-02 — No SDP, SVP, SCMP, SQAP.** Development, verification, CM, and QA
  plans do not exist for PCL.
- **GAP-C-03 — No development standards.** Requirements standard, design
  standard, and a *C* coding standard are missing. `doc/CODING_STYLE.md` targets
  C++ (STL, exceptions, smart pointers) and cannot govern `pcl_core` (strict
  C17, no STL). A MISRA C:2012 (or equivalent) subset with documented deviations
  is the customary route.

### A-2 Development

- **GAP-C-04 — No system requirements / allocation.** DO-178C assumes system
  requirements allocated to software (with safety-related requirements
  identified). PCL's HLR is currently the top; a parent
  system/subsystem-requirements layer (or an explicit statement that PCL HLRs
  *are* the allocated requirements for a given system context) is needed.
- **GAP-C-05 — Derived requirements not identified.** Several HLRs are clearly
  derived (e.g., wire-protocol details PCL.033, allocator behaviour PCL.074) but
  nothing flags derived requirements or records their feedback to the system
  safety assessment process.
- **GAP-C-06 — Design data is informal.** The architecture documents describe
  the design well but are not structured as controlled design description data
  (interfaces, data/control flow, resource budgets, partitioning claims).

### A-3 / A-4 / A-5 Verification of requirements, design, and code (reviews)

- **GAP-C-07 — No review records.** No evidence that HLRs, LLRs, architecture,
  or source code were reviewed for accuracy, consistency, verifiability,
  target-computer compatibility, conformance to standards, and traceability.
  Checklisted review records (with author/reviewer identity and disposition) are
  required at DAL C for most A-3..A-5 objectives.
- **GAP-C-08 — Code-to-LLR traceability missing.** The chain test → LLR → HLR is
  in place, but there is no trace from *source code* (functions/files) to LLRs.
  DO-178C requires bidirectional trace HLR ↔ LLR ↔ source code as well as
  requirements ↔ test cases.
- **GAP-C-09 — No accuracy/consistency analyses.** No stack-usage analysis, no
  worst-case execution time / tick-budget analysis, no arithmetic
  overflow/underflow analysis, no memory-usage analysis. The mutex audit is a
  good start on the concurrency portion but is not framed as verification
  evidence with review sign-off.

### A-6 Testing

- **GAP-C-10 — Tests are not controlled verification cases/procedures.** Test
  *code* exists and is tagged, but there are no test-case specifications
  (inputs, expected results, pass/fail criteria per requirement) or test
  procedure documents, and no retained test *results* records for a defined
  baseline.
- **GAP-C-11 — No target-environment testing.** All evidence is host-based
  (Linux/GCC, Windows/MSVC-GNAT). DO-178C credit requires testing on the target
  computer (or a justified target-representative environment). The APOS/ASAAC
  binding suggests the real target differs materially from the host (threading,
  clock, allocator).
- **GAP-C-12 — Requirements-based test coverage analysis is stale.**
  `HLR_COVERAGE.md` claims completeness for PCL.001-PCL.056, but HLR.md now
  extends to PCL.074 (codecs, capabilities/QoS, plugin ABI/loader, manifest
  routing, transport template, APOS, allocator). LLRs and tagged tests exist for
  the new areas (REQ_PCL up to 447), so this is a regeneration/verification
  task, not new test development — but as evidence it is currently incomplete.

### A-7 Verification of verification

- **GAP-C-13 — Structural coverage is stale and incomplete.** The last
  measurement (2026-05-21, 92% statement) excludes `pcl_codec_registry.c`,
  `pcl_plugin_loader.c`, `pcl_transport_routing.c`, `pcl_capabilities.c`, and
  `pcl_alloc.c`. DAL C requires statement coverage with every gap resolved as
  (a) additional test, (b) deactivated code, or (c) documented analysis. The
  existing uncovered-line analysis is close to the right shape but needs to be
  completed, refreshed, and tied to dispositions.
- **GAP-C-14 — No data-coupling and control-coupling coverage analysis**
  (Table A-7 objective 8, required at DAL C). The mutex audit and the D2/D5
  threading contract are strong inputs, but a systematic analysis of coupling
  between components (executor ↔ container ↔ transport ↔ plugin) exercised by
  the test suite does not exist.
- **GAP-C-15 — Verification tools not qualified.** Coverage credit is taken
  from gcovr; test pass/fail credit from GoogleTest/CTest. Tools whose output is
  used for certification credit without independent verification of that output
  need TQL-5 qualification (criteria 3) or a documented rationale for why their
  output is independently checked.

### A-8 / A-9 / A-10 CM, QA, certification liaison

- **GAP-C-16 — No SCM process evidence.** Git provides version control, but
  there are no identified baselines, no configuration index (SCI) or software
  life cycle environment configuration index (SECI), no problem-report system
  tied to change control, no CC1/CC2 data categorization, and no
  archive/retrieval procedures. There is also no CI pipeline in the repo, so
  build/test reproducibility is undemonstrated.
- **GAP-C-17 — No SQA records.** No evidence of QA audits of processes and
  life-cycle data, and no conformity review before release.
- **GAP-C-18 — No certification liaison data.** No PSAC submission, no Software
  Accomplishment Summary (SAS), no problem-report summary at release.

---

## 3. Additional Gaps for DAL B (delta over DAL C)

- **GAP-B-01 — Decision coverage.** DAL B requires decision coverage in addition
  to statement coverage. Only line coverage is measured today. gcovr/gcc can
  report branch coverage as an approximation, but a decision-coverage-capable
  tool (and its TQL-5 qualification) is the standard route. Expect real test
  additions: the defensive error paths currently analysed away at statement
  level (socket/pthread failure branches, SHM malformed-frame parsing, template
  transport OOM paths) become decision obligations — the coverage report already
  names the needed harnesses (bytestream fuzz for SHM, template-transport OOM
  harness, APOS stub error injection, syscall wrap/shim for socket faults).
- **GAP-B-02 — Independence.** DAL B raises independence requirements to 18
  objectives: reviews of HLR/LLR accuracy and compliance, code compliance with
  LLRs and standards, test-procedure correctness, requirements-based coverage
  analysis, and structural coverage analysis must be performed by someone other
  than the developer of the item. This is an organizational/process gap —
  today all artifacts appear single-author with no reviewer records.
- **GAP-B-03 — Robustness depth.** DAL B scrutiny of abnormal-input testing is
  higher. PCL's robustness suite is genuinely good (OOM injection, fault
  injection, malformed frames on socket), but SHM/UDP/APOS/template transports
  have documented untested defensive paths; at B these need closing rather than
  waiving.
- **GAP-B-04 — Stronger accuracy/consistency evidence.** Stack usage, WCET of
  the tick path, memory-exhaustion behaviour, and scheduling analysis (executor
  tick jitter vs. PCL.015) need quantitative, reviewed analyses rather than
  design assertions.

(Source-to-object-code traceability and MC/DC are DAL A only — not required for
this target.)

---

## 4. Technical / Code-Level Findings Affecting Certifiability

These are design characteristics that will attract certification scrutiny at
either level and are cheaper to address early:

1. **Runtime dynamic memory.** Message ingress deep-copies (`pcl_executor_post_incoming`),
   queue nodes, and port payloads allocate from the heap during ACTIVE
   operation. DO-178C does not prohibit this, but it requires analysis
   (bounded-memory argument, fragmentation, allocation-failure behaviour — the
   OOM test suite helps). A static-pool or bounded-arena allocation mode behind
   `pcl_alloc` would materially simplify the argument, and bounded capacities
   (PCL.008, PCL.013) already point that way.
2. **Runtime plugin loading (`dlopen`) — PCL.064-070.** Field-loadable /
   dynamically-linked code is a major certification complication. For the
   certified configuration, provide a **static-registration build option** (all
   transports/codecs linked and registered at build time, loader compiled out)
   so the plugin path can be classified as deactivated code in the airborne
   configuration. Keep the manifest-driven composition for non-certified
   deployments.
3. **UDP transport synchronization uses `volatile int` flags,** not C11 atomics.
   "Safe by inspection" (per the mutex audit) is not a portable data-race-freedom
   argument; convert to `stdatomic.h` or protect with the existing mutex shim.
4. **OS abstraction.** pthreads/Win32 threading, sockets, and clocks are used
   directly. Certification is against a specific target (likely ASAAC/APOS per
   PCL.073); the OS-facing surface should be isolated behind one documented
   porting layer so target testing and the RTOS's own certification evidence
   line up.
5. **Deactivated/dead code.** The template transport, conformance suite, and
   test-only plugins live beside production code; the certified-part list
   (which files constitute the airborne software) must be pinned in the SCI, and
   defensive branches classified deliberately.
6. **C++ wrappers and Ada binding.** If the certified image is C-only, scope the
   certification boundary explicitly to `pcl_core` (+ selected transports) and
   declare the C++/Ada layers as separate items with their own (or their
   consumers') assurance arguments.

---

## 5. Recommended Roadmap

### Phase 1 — Reach DAL C readiness

1. Write the five plans (PSAC, SDP, SVP, SCMP, SQAP) and three standards
   (requirements, design, C coding standard — MISRA C:2012 subset with
   deviations). Small, honest documents that describe the process actually used.
2. Stand up SCM mechanics: CI pipeline building `all-off`-style minimal config,
   running the PCL test suite and coverage on every change; problem-report
   workflow; tagged baselines; SCI/SECI generation.
3. Regenerate and complete traceability: extend `HLR_COVERAGE.md` to PCL.074,
   add code-to-LLR trace (file/function ↔ REQ_PCL tags — the tag convention
   extends naturally to source), and automate matrix generation so it cannot go
   stale.
4. Re-run structural coverage over **all** current sources; disposition every
   uncovered line (test / deactivated / analysis).
5. Perform and record reviews (HLR, LLR, design, code) against the new
   standards, with checklists and named reviewers.
6. Produce the missing analyses: stack usage, tick-loop timing, memory bounds,
   data/control coupling (seed from `MUTEX_AUDIT.md`).
7. Qualify (TQL-5) or independently check gcovr and the test framework outputs.
8. Define the target environment and repeat the test campaign on
   target-representative hardware.

### Phase 2 — Delta to DAL B

9. Switch coverage measurement to a decision-coverage-capable qualified tool;
   close the documented defensive-path gaps (SHM frame fuzzing, template OOM
   harness, APOS error injection, socket syscall shims).
10. Introduce verification independence: independent reviewers for
    requirements/code reviews and independent performance of coverage analyses;
    record independence in every review/analysis record.
11. Harden the technical items in section 4 (static-registration certified
    build, atomics in UDP transport, bounded-allocation mode) so the DAL B
    scrutiny of the design holds up.

### Effort profile

The unusual position of PCL is that the *hardest* DAL C material — layered
requirements, requirement-tagged robustness testing, near-total statement
coverage with gap analysis — largely exists, while the *routine* material
(plans, standards, review records, CM/QA evidence) is entirely absent. Most of
the DAL C effort is therefore process formalization and evidence capture, not
engineering rework. The genuinely new engineering work is: target testing,
data/control coupling analysis, decision coverage (for B), the independence
organization (for B), and the certified-configuration decisions around dynamic
memory and plugin loading.
