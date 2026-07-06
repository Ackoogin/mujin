# PCL C Coding Standard

| Field | Value |
|-------|-------|
| Document | PCL-CS-C |
| Status | **Draft — initial DAL C pass, 2026-07-06** |
| Applies to | All C sources and headers in `subprojects/PCL/src/`, `include/pcl/*.h`, `bindings/apos/` |

The workspace-level `doc/CODING_STYLE.md` targets C++ (STL, exceptions,
smart pointers) and does **not** govern the PCL C core. This document does.
It codifies the rules the existing code already follows; where a rule is
aspirational rather than current practice it is marked *(to adopt)*.

Orientation: the ruleset is aligned with the intent of MISRA C:2012.
A rule-by-rule MISRA compliance matrix with a formal deviation record is a
Phase 1 open item; until then section 6 is the deviation register.

## 1. Language and Environment

1.1. Strict ISO C with `C_EXTENSIONS OFF`; code must compile warning-clean
     under MSVC and GCC. (Note: `HLR.md` D1 states C17 while the build pins
     `C_STANDARD 11` — reconcile at the first standards review.)
1.2. No external dependencies: only the C standard library and the
     platform layers explicitly listed in 1.3. No C++ constructs, no STL.
1.3. Platform-specific code (Win32 vs. POSIX threads, sockets, shared
     memory, dynamic loading) is confined behind `#if defined(_WIN32)`
     blocks or dedicated abstraction files; business logic never branches
     on platform.
1.4. Banned constructs: `assert()` in production code, `setjmp`/`longjmp`,
     `abort()`/`exit()` from library code, variable-length arrays, `alloca`,
     recursion *(to adopt: recursion audit — no known instances, not yet
     mechanically checked)*, function pointers cast between incompatible
     signatures.

## 2. Memory and Resources

2.1. All variable-length data crossing the C ABI is allocated and freed
     through `pcl_alloc`/`pcl_calloc`/`pcl_realloc`/`pcl_free` (PCL.074),
     never bare `malloc`/`free`, so buffers survive crossing CRT boundaries.
2.2. Size arithmetic that can overflow must be checked before the
     allocation (`total / nmemb != size` pattern in `pcl_alloc.c`).
2.3. Every allocation failure path returns `PCL_ERR_NOMEM` (or NULL for
     constructors) and releases everything partially allocated (PCL.046);
     the OOM injection suite (`test_pcl_oom`) is the enforcement mechanism.
2.4. Fixed capacities (`PCL_MAX_PORTS`, `PCL_MAX_PARAMS`, peer-list bounds)
     are compile-time constants; capacity overflow is a checked error, not
     undefined behaviour.

## 3. Error Handling

3.1. Every public function validates its arguments; NULL handles yield
     `PCL_ERR_INVALID` (or NULL/safe default), never a crash (PCL.045).
3.2. Return values are `pcl_status_t` from the fixed set (PCL.047); no
     errno-style side channels, no silent failure. Fail closed: anything
     unprovable at compose time is rejected with a diagnostic (D9).
3.3. Errors are reported to the caller; `pcl_log()` is diagnostic
     supplement, never the only signal.

## 4. Concurrency

4.1. Component callbacks execute only on the executor thread (D2). Transport
     worker threads interact with the executor exclusively through the
     deep-copy ingress queues (`pcl_executor_post_*`) per PCL.075; no
     callback is ever invoked from a transport thread.
4.2. Every mutex acquisition follows the documented lock hierarchy in
     `doc/reports/PCL/MUTEX_AUDIT.md`; new locks require updating that
     audit.
4.3. Cross-thread flags must use the mutex shim or C11 `stdatomic.h` —
     plain or `volatile` int flags are non-conforming *(deviation DEV-01,
     section 6)*.
4.4. Thread teardown: wake, then join, then free — no freeing of state a
     worker may still touch (PCL.075(d)).

## 5. Style and Structure

5.1. Naming: `pcl_` prefix for public symbols; `snake_case` functions and
     variables; `PCL_` prefix for macros/constants; `_t`-suffixed typedefs.
5.2. Public headers document each function's contract: arguments, ownership,
     threading rules, and status codes.
5.3. Include order: own header first, then platform, then C standard
     library.
5.4. Two-space indentation, braces on the statement line, pointer `*` bound
     to the type in declarations (`void* ptr`).
5.5. Comments state contracts and invariants, not restatements of the code.
5.6. Test requirement tags: `///< REQ_PCL_NNN: description. PCL.0XX.` in
     tests; *(to adopt)* function-level LLR tags in production sources for
     code-to-LLR trace (GAP-C-08).

## 6. Deviation Register

| ID | Rule | Deviation | Rationale / plan |
|----|------|-----------|------------------|
| DEV-01 | 4.3 | `pcl_transport_udp.c` uses `volatile int` flags for thread signalling | Safe-by-inspection per mutex audit, but not a portable data-race-freedom argument. Plan: convert to `stdatomic.h` (gap analysis section 4.3). Accepted until then. |
| DEV-02 | 1.4 | `pcl_plugin_loader.c` uses platform dynamic loading (`dlopen`/`LoadLibrary`) | Required by PCL.064–070. In the certified configuration it is either deactivated (PSAC 3.3.1 Option A) or airborne software under the bounded-composition conditions (Option B: enumerated CC1 plugin set, controlled manifest, init-time-only load, integrity check). |

## 7. Enforcement

- Compiler warnings clean on MSVC + GCC (current practice).
- Code reviews check this standard via the code checklist in
  `doc/reviews/PCL/checklists.md`.
- *(to adopt)* Static analysis (MISRA checker or clang-tidy profile) in CI.
