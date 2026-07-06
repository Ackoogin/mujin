# PCL Derived Requirements Register

| Field | Value |
|-------|-------|
| Document | PCL-DR |
| Status | **Draft — initial classification pass, 2026-07-06; pending review** |
| Governed by | `../standards/requirements_standard.md` section 3 |

DO-178C requires derived requirements — those not traceable to parent
(system) requirements — to be identified and fed back to the system safety
assessment process. PCL currently has **no parent system-requirements
layer**: the HLRs are the top of the stack (gap analysis GAP-C-04). Strictly,
that makes every HLR "derived" until an embedding programme allocates system
requirements onto PCL.

This register therefore records the *engineering* classification: which HLRs
exist because of PCL-internal design/implementation choices (and would remain
derived even after system allocation), and what a system safety process
should know about each. When a programme adopts PCL, this register is the
feed-back input required by DO-178C section 5.1.2.b.

## Register

| ID | HLR(s) | Why it is derived | Safety-relevant feedback |
|----|--------|-------------------|--------------------------|
| DR-01 | PCL.033 | TCP wire-protocol framing (`[len][type][payload]`) is an implementation choice, not a system need | Framing errors are detected by length/type validation; malformed frames are dropped, not propagated. Fault-injection tested. |
| DR-02 | PCL.074 | Portable allocator exists because buffers cross CRT/DLL boundaries between MSVC and GNAT/MinGW artifacts | Heap corruption from mismatched allocators is a silent-failure hazard; the single-heap rule removes it. Overflow-checked size arithmetic. |
| DR-03 | PCL.008, PCL.013 | Specific capacity numbers (64 ports, 128 params) are budget choices for embedded targets | Capacity exhaustion surfaces as a checked error at configure time, not a runtime allocation failure in flight. |
| DR-04 | PCL.036e | Connect retry/backoff/keepalive semantics respond to deployment realities (peer start order, restarts), not a system requirement | Silent peer death is detected within ~8 s (keepalive tuning); reconnection changes message-loss windows — embedding programmes must account for this in their failure analysis. |
| DR-05 | PCL.036g | Atomic fan-out and per-topic backpressure on the SHM bus derive from the bus implementation | Prevents partial fan-out (some peers seeing a frame, others not) — a state-divergence hazard for redundant consumers. Topics opted into blocking backpressure trade latency for delivery; deadline budgets must be reviewed per deployment. |
| DR-06 | PCL.047 | The specific status-code set is an API design choice | Uniform fail-closed error discrimination; no silent failure modes. |
| DR-07 | PCL.060–PCL.063 | Capability masks and QoS floor algebra derive from the pluggable-transport design (D8), not from a system need | Misrouted safety-relevant traffic is rejected at compose time (fail closed); reliability must be *declared* by the transport, never assumed. |
| DR-08 | PCL.064–PCL.070 | Plugin ABI, loader, and manifest routing exist to support non-certified composable deployments | Dynamically loaded code is excluded from the certified configuration (PSAC 3.2/3.3); loaders fail closed. Programmes must not enable the loader in an airborne build. |
| DR-09 | PCL.071, PCL.075, PCL.076 | The transport threading template and threading-model contract codify the implementation's concurrency architecture | These are the load-bearing guarantees for deterministic execution (D2/D5); regressions are caught by the threading conformance suite. |
| DR-10 | PCL.028–PCL.030, PCL.030a | Transport vtable shape is an internal abstraction choice | The vtable boundary is where third-party adapters attach; the conformance suites are the acceptance gate for them. |

## Rules

- New HLRs/LLRs judged derived at review are added here in the same change
  set, with the reviewer named in the review record.
- When a system-requirements layer is introduced, each non-derived HLR gains
  a system trace and is removed from the "everything is technically derived"
  caveat above; this register then shrinks to the true derived set.
