# PCL Design Standard

| Field | Value |
|-------|-------|
| Document | PCL-DS |
| Status | **Draft — initial DAL C pass, 2026-07-06** |
| Applies to | `subprojects/PCL/doc/architecture/`, `include/pcl/*.h` interface contracts |

Rules for PCL design data, per DO-178C section 11.7. PCL's design description
is deliberately lightweight: the architecture documents carry structure and
rationale; the public headers are the authoritative interface specification.

## 1. Design Data Set

The controlled design description consists of:

| Artifact | Role |
|----------|------|
| Design decisions D1–D9 (`../requirements/HLR.md`) | Binding architectural commitments |
| `../architecture/component_container_design.md` | Container/executor internals: state machine, data structures, queues |
| `../architecture/08-pcl-component-system.md` | Component-system overview and composition model |
| `include/pcl/*.h` header contracts | Interface specification: per-function arguments, ownership, threading, status codes |
| `doc/reports/PCL/MUTEX_AUDIT.md` | Concurrency design record: lock hierarchy, unprotected-state contracts |

## 2. Architectural Rules

2.1. **Layering**: `pcl_core` depends on nothing; transports depend on
     `pcl_core` only; plugins depend on their transport only. No upward or
     lateral includes. Component business logic never includes transport
     headers.
2.2. **Threading**: any design element that creates a thread must state the
     thread's lifetime, what wakes it, how teardown joins it, and which
     queues it touches — and must satisfy the PCL.075 contract.
2.3. **Interfaces**: every public function documents ownership transfer
     (who frees what), permitted calling thread, permitted lifecycle states,
     and the complete set of returnable status codes.
2.4. **Resource bounds**: new state must have a compile-time capacity or a
     documented growth bound tied to input rates; "unbounded by design"
     requires an explicit accepted deviation.
2.5. **Fail-closed**: any compose-time configuration input (routes,
     manifests, plugins, QoS) that cannot be validated is rejected before
     traffic flows, with a diagnostic naming the deficiency (D9); partial
     installation must be rolled back (PCL.070).
2.6. **Partitioning claims**: PCL claims no memory or time partitioning;
     anything requiring partitioning must state that it relies on the
     platform/RTOS.

## 3. Documentation Rules

3.1. A design change that alters structure, threading, locking, or resource
     bounds updates the corresponding design artifact in the same change set.
3.2. New locks or lock-order changes update `MUTEX_AUDIT.md`.
3.3. Diagrams are welcome but the text is normative.

## 4. Review Criteria

At review (checklist in `doc/reviews/PCL/checklists.md`), design data is
checked for: compatibility with the HLRs it implements, consistency with
D1–D9 and the rules above, verifiability of stated properties,
target-computer compatibility assumptions made explicit, and absence of
undocumented interfaces.

## 5. Known Shortfalls (Phase 1 open items)

- Resource budgets (stack, heap bound, tick timing) are asserted, not
  analysed — analyses planned in `../plans/SVP.md` section 6.
- Data/control-coupling documentation exists only as the mutex audit; the
  systematic coupling analysis is GAP-C-14.
