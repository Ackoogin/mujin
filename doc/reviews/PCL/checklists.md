# PCL Review Checklists

Checklists for the reviews required at DAL C (DO-178C Tables A-3..A-5), used
with `review_record_template.md`. Answer each item yes / no / n-a; every "no"
becomes a finding with a disposition.

## HLR Review (Table A-3)

1. Statement uses "shall" and is unambiguous (one interpretation).
2. Verifiable: decomposable into testable LLRs; no vague "support/handle".
3. Consistent with neighbouring HLRs and design decisions D1–D9.
4. Rationale present and gives the *why*, not a restatement.
5. Identifier follows the requirements standard; never reused.
6. Appears in the design-decision traceability table.
7. No implementation detail that belongs in LLRs (or it is justified).
8. Target-computer assumptions (threading, clocks, sockets) explicit.
9. If derived: registered in `derived_requirements.md` with safety feedback.
10. Conforms to `subprojects/PCL/doc/standards/requirements_standard.md`.

## LLR Review (Table A-4)

1. Statement names the concrete C function(s) and observable behaviour.
2. Complies with (does not contradict or exceed) its parent HLR(s).
3. **Traces** field present, parents correct and sufficient.
4. **Verification** field file-qualified (`test_file.cpp::TestCase`) and the
   named tests actually verify the statement.
5. One testable behaviour per LLR; robustness split out where practical.
6. Accurate against the current code (no drift).
7. If derived: registered with safety feedback.
8. Conforms to the requirements standard.
9. `gen_hlr_coverage.py --check` passes after the change.

## Design Review (Table A-4 architecture objectives)

1. Compatible with the HLRs it implements.
2. Consistent with D1–D9 and the design standard's architectural rules
   (layering, threading statement, ownership, fail-closed, bounds).
3. New threads state lifetime/wake/join/queues and meet PCL.075.
4. New locks recorded in `MUTEX_AUDIT.md` with hierarchy position.
5. Resource bounds stated (capacity or growth bound tied to input rate).
6. Interfaces fully specified (ownership, thread, states, status codes).
7. No undocumented interface or hidden coupling introduced.

## Code Review (Table A-5)

1. Complies with the LLRs it implements (each named LLR's behaviour is
   actually implemented; nothing implemented lacks an LLR).
2. Conforms to `subprojects/PCL/doc/standards/c_coding_standard.md`
   (or the deviation is registered).
3. Argument validation and NULL safety per PCL.045.
4. Allocation failures return `PCL_ERR_NOMEM` and roll back partial state.
5. ABI-crossing buffers use `pcl_alloc`/`pcl_free`.
6. Threading: no callback from transport threads; queue ingress only;
   teardown wakes-joins-frees in order.
7. Warning-clean on MSVC and GCC.
8. Tests added/updated with `///< REQ_PCL_NNN` tags; suite green; statement
   coverage held at 100% or the gap dispositioned.
9. Accurate and consistent: no dead stores, unreachable code, or
   unintentional dead code (intentional deactivated code identified).

## Trace Review (all tables)

1. `gen_hlr_coverage.py --check` reports zero gaps.
2. Spot-check: sampled matrix rows against the actual documents/tests
   (guards against generator defects; part of its TQL-5 position).
