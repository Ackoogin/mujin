# Generic Proto Binding — Implementation Record

Status: **delivered 2026-07-01**. The generator now supports both
**PYRAMID-style** and **arbitrary/generic** proto contracts via
`--contract-layout {pyramid,generic}`. The design and usage reference is
[`generic_contract_layout.md`](../../../subprojects/PYRAMID/doc/architecture/generic_contract_layout.md);
the motivating analyses were
[`arbitrary_proto_binding_gap_analysis.md`](arbitrary_proto_binding_gap_analysis.md) and
[`binding_hard_coupling_removal_report.md`](binding_hard_coupling_removal_report.md)
(both historical).

## What Was Delivered

| # | Increment | Outcome |
|---|-----------|---------|
| 1 | Neutral `BindingContract` + `NamingPolicy` (pyramid_compat / generic); `--contract-layout` flag | `pim/binding_contract.py`; content-based classification, policy-injected C++ generators |
| 2 | Generic C++ JSON binding path | Package-derived namespaces/headers, role-neutral service facades |
| 3 | Generator artifact manifest | `binding_manifest.json` written for both layouts, artifacts recorded by role |
| 4 | Generic protobuf / FlatBuffers / gRPC / ROS2 C++ backends | Import-root-relative includes, policy-derived names, service detection by parsed `service` decls |
| 5 | Manifest-driven CMake + data-driven topic metadata | `PYRAMID_BINDING_SOURCE_MODE=manifest` (opt-in, real-build verified); `standard_topics.py` loads `pim/topic_metadata/tactical_objects_topics.json`, no domain branching |
| 6 | Ada parity for generic layout | Types + codecs + role-neutral service facade (`AdaGenericServiceGenerator`), GNAT-compiled |

Backward compatibility was held throughout: default `pyramid` layout output is
byte-for-byte identical to the pre-refactor generator (regression-locked by
`diff -qr` proofs in the Python test suite; only `binding_manifest.json` is
added). Test coverage lives in `subprojects/PYRAMID/tests/test_generic_*.py`,
`test_binding_manifest.py`, `test_topic_metadata.py`,
`test_manifest_cmake_helper.py`, and `cmake/tests/`.

## Remaining Open Items

Carried forward from the hard-coupling sweep; these are the still-open
dispositions. Actionable follow-up plans for them live in
[`doc/plans/PYRAMID/architecture_cleanup_plan.md`](../../plans/PYRAMID/architecture_cleanup_plan.md).

| Item | Location | Problem | Disposition |
|------|----------|---------|-------------|
| Hardcoded base-type short-name map | `pim/cpp_codegen.py` `BASE_TYPE_MAP` | Maps specific `pyramid.data_model.base.*` / `common.*` FQNs to short names. | Domain-specific; confine to `pyramid_compat`. |
| ROS2 typed marshal codec | `pim/ros2_marshal_codegen.py`, `pim/ros2_ir.py` | Typed ROS2 marshal codec is tied to `pyramid::domain_model`, `pyramid_msgs`, `pyramid_ros2_codec.hpp`. | Emitted only for `pyramid_compat`; make package-neutral before enabling for generic. |
| CMake `pyramid_*` globs (partial) | `subprojects/PYRAMID/CMakeLists.txt` | Manifest mode covers FlatBuffers schemas + JSON codec sources; the per-module C-ABI marshal loop, codec-plugin targets, and protoc/gRPC proto input paths still use globs/fixed paths. | Extend manifest with module identity, then flip default from `glob`. |
| Duplicate `ProtoFile`/`parse_proto` | `pim/cpp_codegen.py` vs `pim/proto_parser.py` | `cpp_codegen.py` keeps its own lightweight proto parser parallel to `proto_parser.py` — redundancy/skew risk. | Consolidate onto the `proto_parser` model. |
| Checked-in Tactical Objects protobuf support | `src/protobuf_support/*` | Domain-specific protobuf codec support sits checked-in on the active path. | Move to generated/manifest-selected; keep tactical shim as compat only. |
| Residual `pyramid_*` literal sweep | generators/backends | A full repo-wide sweep for residual `pyramid_*` literals (Ada service generator + some backend compat shims) is outstanding. | Audit; confine to `PyramidCompatNamingPolicy`. |
