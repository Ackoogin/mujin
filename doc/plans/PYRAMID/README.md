# PYRAMID Plans — Index & Design-Intent Summary

**Consolidated 2026-07-10.** The PYRAMID documentation set was reviewed and
pruned: executed plans, completed reviews, and point-in-time status reports
were removed. What remains is:

| What you want | Where it lives |
|---------------|----------------|
| **Outstanding work** (the single tracker) | [`doc/todo/PYRAMID/TODO.md`](../../todo/PYRAMID/TODO.md) |
| **High-level design & usage** (single user-guide entry, with diagrams) | [`subprojects/PYRAMID/doc/guides/pyramid_user_guide.md`](../../../subprojects/PYRAMID/doc/guides/pyramid_user_guide.md) |
| **Live plan** (proposed, not yet scheduled) | [`pyramid_split_and_tobj_pim_migration_plan.md`](pyramid_split_and_tobj_pim_migration_plan.md) |
| **Live plan** (in progress; Phases 0–4 and Phase 5 step 1 verified against real Sleet; Phase 6 superseded) | [`la_cal_integration_plan.md`](la_cal_integration_plan.md) — LA-CAL (`owp`) transport plugin + OMS JSON codec, rung 1 of the OMS CAL join |
| **Live plan** (done 2026-07-12; supersedes the AME-facing scope of Phase 6 above) | [`kitty_hawk_pcl_consumer_plan.md`](kitty_hawk_pcl_consumer_plan.md) — PCL-only consumer proof against the full live Kitty Hawk stack |
| **Live plan** (in progress; Phase 3 complete and live-verified 2026-07-12 — generated P1 tree drives both the command seam and the Kitty Hawk consumer harness over real Sleet; Phase 4 (A-GRA/P2) unscheduled) | [`uci_mms_conversion_plan.md`](uci_mms_conversion_plan.md) — `xsd2proto` + profile ladder: from the hand-authored UCI seam test cases to a GRA-scale minimum message set converted from the XSD (rung 3) |
| **Architecture reference** | `subprojects/PYRAMID/doc/architecture/` — see especially [`oms_agra_compatibility.md`](../../../subprojects/PYRAMID/doc/architecture/oms_agra_compatibility.md) for current OMS/AMS-GRA/A-GRA compatibility status |
| Full text of any retired document | git history (`git log --diff-filter=D --oneline -- doc/plans/PYRAMID doc/reports/PYRAMID doc/reviews/PYRAMID`) |

## Design intents of the retired documents

Each retired plan/report/review is summarised below: the intent it
established (still binding) and where the delivered design is now
documented. Code and requirement comments that cite the old filenames refer
to these entries; the full text is in git history.

### Plans (`doc/plans/PYRAMID/`)

- **`transport_plugins.md`** — *Intent:* components and clients link only
  `pcl_core` plus their generated contract; codecs and transports are
  runtime-loaded `.so` plugins selected by opaque `config_json`, **failing
  closed** when absent; heterogeneous middleware is handled by a declared
  capability model (`PUBSUB`/`RPC_UNARY`/`RPC_STREAM` + QoS) validated at
  compose time. *Now:*
  [`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md),
  [`ros2_transport_semantics.md`](../../../subprojects/PYRAMID/doc/architecture/ros2_transport_semantics.md);
  deferred items are TODO WS-D rows.

- **`architecture_cleanup_plan.md`** and **`generator_refactor_plan.md`** —
  *Intent:* one proto parser; the generator organised as per-artifact emitter
  packages (`pim/cpp/`, `pim/ada/`) with no module globals and deterministic
  emission; manifest-driven CMake source selection; domain literals confined
  to `PyramidCompatNamingPolicy`; and the standing regression bar that the
  default `pyramid` layout output stays **byte-for-byte identical** across
  generator changes. *Now:*
  [`generic_contract_layout.md`](../../../subprojects/PYRAMID/doc/architecture/generic_contract_layout.md)
  and `subprojects/PYRAMID/pim/README.md`; the regression bar is restated in
  TODO.md.

- **`pubsub_contract_generation_plan.md`** — *Intent:* the `.proto` contract
  is the single source of truth for interaction pattern, topic names, and
  QoS (`pyramid.options.pyramid_op`, MBSE-stamped). Adopted the
  A-GRA-informed **correlated-pair pattern**: request/requirement as two
  topics on a flat namespace, correlation by payload id (not per-instance
  topics), acceptance as status transitions rather than synchronous acks.
  QoS in the contract is *intent* (a floor); a transport plugin declares
  *capability* (a ceiling); compose-time validation reconciles the two and
  fails closed. *Now:*
  [`pyramid_interaction_semantics.md`](../../../subprojects/PYRAMID/doc/architecture/pyramid_interaction_semantics.md)
  and the
  [pub/sub & interaction facade guide](../../../subprojects/PYRAMID/doc/guides/pubsub_interaction_guide.md).

- **`agra_pubsub_shm_udp_proving_plan.md`** — *Intent:* prove the
  contract-derived pub/sub data plane over **real decoupled transports**
  (shared memory and UDP), including the RELIABLE-over-BEST_EFFORT
  fail-closed negative and the explicit deploy-time downgrade path (writing
  `best_effort` as a route's floor is an operator decision, never a silent
  default). *Now:* the `agra_*` harnesses under
  `subprojects/PYRAMID/pim/test_harness/` (see `FINDINGS.md` there),
  `subprojects/PYRAMID/pim/agra_example/README.md`.

- **`rpc_pubsub_interchangeability_plan.md`** — *Intent:* RPC and pub/sub
  are **interchangeable realizations of one contract interaction**.
  Component code targets the interaction facade
  (`submit()`/`transitions()`/`publish()`/`subscribe()`); the realization is
  chosen per leg, per deployment, at compose time; routing both realizations
  of one leg is a compose-time error (`exclusive` groups); and the facade
  promises only the weaker realization's guarantee — no synthesised acks.
  Design points D1–D7 and the evidence ledger are in the retired plan (git
  history). *Now:*
  [pub/sub & interaction facade guide](../../../subprojects/PYRAMID/doc/guides/pubsub_interaction_guide.md),
  [`cpp_component_authoring.md`](../../../subprojects/PYRAMID/doc/architecture/cpp_component_authoring.md).

- **`standard_alignment_plan.md`** — not retired; it is the stable design
  reference for the shipped Tactical Objects component and **moved** to
  [`subprojects/PYRAMID/doc/architecture/tactical_objects/standard_alignment.md`](../../../subprojects/PYRAMID/doc/architecture/tactical_objects/standard_alignment.md).

### Reviews (`doc/reviews/PYRAMID/`)

- **`review_pyramid_bindings_pluggability.md`** — *Intent:* swapping codecs
  and transports is configuration/startup wiring only; no per-codec or
  per-transport branching in business logic. All findings closed. *Now:*
  [`generated_bindings.md`](../../../subprojects/PYRAMID/doc/architecture/generated_bindings.md).

### Reports (`doc/reports/PYRAMID/`)

- **`binding_hard_coupling_removal_report.md`**,
  **`arbitrary_proto_binding_gap_analysis.md`**,
  **`generic_binding_implementation_progress.md`** — *Intent:* zero hard
  coupling in reusable binding infrastructure — any valid `.proto` tree
  generates bindings; PYRAMID package conventions survive only as an
  explicit compatibility policy (`--contract-layout pyramid`). Delivered by
  the generic-layout workstream. *Now:*
  [`generic_contract_layout.md`](../../../subprojects/PYRAMID/doc/architecture/generic_contract_layout.md).

- **`codec_plugin_abi_findings.md`** — *Intent:* only C-ABI-safe data
  crosses a plugin boundary (the frozen `pyramid_<T>_c` struct layer, one
  cross-language codec `.so` for C++ and Ada); entry points are
  ABI-versioned and fail closed on mismatch; buffer ownership crosses via
  the `free_msg` hook. *Now:*
  [`transport_codec_plugin_system.md`](../../../subprojects/PYRAMID/doc/architecture/transport_codec_plugin_system.md)
  ("Code mechanism — ABI contracts").

- **`pcl_plugin_threading_model_report.md`** — *Intent:* PCL business logic
  runs **only on the executor thread**; transport egress entry points never
  block (validate, copy, enqueue, return); ingress from foreign threads
  deep-copies and posts to executor queues; callbacks are never fired inline;
  plugin unload joins all plugin-owned threads before `dlclose`. *Now:* the
  contract is documented in `subprojects/PCL/include/pcl/pcl_transport.h` /
  `pcl_executor.h` header docs and enforced by the `PclTransportThreading`
  conformance suite. The outstanding plugin-level harness
  (`PyramidPluginThreading.Ros2*`/`Grpc*`) is a TODO WS-D row.

- **`in_process_service_pubsub_todo.md`** — *Intent:* same-process
  component-to-component service/pub/sub is a first-class facade
  configuration (explicit local routing, facade-owned pub/sub), not raw PCL
  calls in component code. E1–E4/E6 delivered; the one open item (E5,
  `StandardBridge` raw-PCL classification) is tracked in TODO WS-E.

- **`binding_performance_report_2026-04-28.md`** — point-in-time
  performance baselines (JSON/FlatBuffers/protobuf codec costs, transport
  latencies, the shared-memory benchmark fix). The live benchmark is
  `subprojects/PYRAMID/tests/test_binding_performance.cpp`; historical
  numbers are in git history.

- **`generated_bindings_status.md`** — point-in-time proof-matrix/status
  snapshot for the Tactical Objects binding paths. Live status is the CTest
  suite itself plus TODO.md; architecture/usage is
  [`generated_bindings.md`](../../../subprojects/PYRAMID/doc/architecture/generated_bindings.md).

`doc/reports/PYRAMID/tactical_objects/HLR_COVERAGE.md` is **kept**: it is
the maintained HLR→test traceability input consumed by
`subprojects/PYRAMID/scripts/gen_requirement_trace.py`.
