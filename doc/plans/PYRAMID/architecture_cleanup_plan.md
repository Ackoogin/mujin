# PCL/PYRAMID Architecture & Organisation Cleanup Plan

Source: the 2026-07-01 PCL/PYRAMID documentation and architecture review.
Scope: structural/code-organisation follow-ups in `subprojects/PCL` and
`subprojects/PYRAMID`. Functional transport/codec work (typed ROS2 wire, ROS2
actions, manifest-routed remote ingress, capability direction/mode) is **not**
duplicated here — that is tracked in
[`transport_plugins.md`](transport_plugins.md).

> **Tracking:** outstanding-work status for all PYRAMID docs is consolidated
> in [`doc/todo/PYRAMID/TODO.md`](../../todo/PYRAMID/TODO.md). Items 1 and 2
> below were **executed 2026-07-03** via
> [`generator_refactor_plan.md`](generator_refactor_plan.md)
> (commits `3fb876f..14d3b23`); this doc remains the design detail for the
> rest.

## Completed directly (with the review)

These were low-risk moves/deletes and one packaging defect fix, applied
directly rather than planned:

| ID | Action | Notes |
|----|--------|-------|
| D1 | Moved the standalone MBSE/SysML import tools (`sysml_parser.py`, `proto_generator.py`, `ada_type_generator.py`, `contract_resolver.py`, sample `test.json`, legacy `pyramid-middleware.ads/.adb`) from `pim/` into `pim/mbse/` | They share no imports with the binding generator; `pim/README.md` documents the split. Side benefit: `package_sdk` copies `pim/*.py` flat, so the SDK no longer ships the unrelated MBSE tools. |
| D2 | Deleted the empty legacy `subprojects/PYRAMID/bindings/` directory (tombstone README) and its dead `.gitignore` entries | All bindings are generated build-local; remaining doc links updated to the real locations. |
| D3 | Fixed SDK packaging to ship `pim/topic_metadata/*.json` | `standard_topics.py` loads `topic_metadata/tactical_objects_topics.json` at import; a packaged SDK without it crashed on `import generate_bindings` (`FileNotFoundError`). Verified: import succeeds with the file, fails without. `package_sdk.sh` and `.bat` both patched. |

## Work items

Ordered so each item reduces risk for the next. Every item carries the same
regression bar: **default `pyramid` layout output stays byte-for-byte
identical** (`diff -qr` against a pre-change generated baseline, as the
existing `test_generic_*.py` suites do) and the full Python suite
(`python3 -m pytest subprojects/PYRAMID/tests -q`) stays green.

### 1. Consolidate the duplicate proto parser in `cpp_codegen.py`

- **Problem**: `pim/cpp_codegen.py` defines its own lightweight
  `ProtoFile`/`parse_proto` alongside the richer
  `proto_parser.ProtoFile`/`parse_proto_tree`. Two parsers of the same IDL is
  a skew risk — the PIM-proto viability work hit exactly this class of bug
  (short-name resolution picking the wrong declaring package).
- **Approach**: inventory what the lightweight parser extracts that
  `proto_parser` does not (if anything); extend `proto_parser` to cover it;
  switch `cpp_codegen.py` call sites to the `proto_parser` model; delete the
  local parser.
- **Acceptance**: no `ProtoFile`/`parse_proto` definition remains in
  `cpp_codegen.py`; byte-identical pyramid output; suite green.

### 2. Split the generator monoliths into per-artifact emitter modules

- **Problem**: `cpp_codegen.py` (~4.1k lines) and `ada_codegen.py` (~3.7k
  lines) each contain many independent emitters (types, JSON codecs, service
  facades, component facades, C-ABI call sites, topic helpers). They are where
  most generator defects have surfaced, and every contract-shape change lands
  in the same two files.
- **Approach**: mechanical extraction, no behaviour change. Create
  `pim/cpp/` and `pim/ada/` packages (mirroring the existing `pim/backends/`
  precedent) and move one emitter class per module, e.g. `cpp/types.py`,
  `cpp/json_codec.py`, `cpp/service_facade.py`, `cpp/component_facade.py`;
  `ada/types.py`, `ada/codec.py`, `ada/service_facade.py`,
  `ada/generic_service_facade.py`. Keep `cpp_codegen.py`/`ada_codegen.py` as
  thin re-export shims so `generate_bindings.py`, `package_sdk` copies, and
  tests keep working, then migrate imports and retire the shims.
- **Prerequisite**: item 1 (do not move the duplicate parser, delete it
  first). Update `package_sdk.sh`/`.bat` to copy the new subpackages
  (currently `pim/*.py` + `pim/backends/*.py` only).
- **Acceptance**: no generator source file over ~1.5k lines; byte-identical
  pyramid output; SDK import smoke check
  (`python3 -c "import generate_bindings"` from a packaged copy) passes.

### 3. Retire checked-in `src/protobuf_support/` from the active path

- **Problem**: `pyramid_services_tactical_objects_protobuf_codec.{hpp,cpp}`
  is hand-maintained, domain-specific codec support checked in on an
  otherwise fully generated path — the last non-generated binding artifact.
- **Approach**: teach the protobuf backend to emit the service-codec support
  it covers (the generic protobuf backend already emits data-model codec
  stubs); record it in `binding_manifest.json` under a role; select it via
  `pyramid_binding_sources`. Keep the checked-in copy until the generated
  output is diff-verified against it, then delete.
- **Acceptance**: `src/protobuf_support/` deleted; protobuf E2E tests
  (`tobj_cpp_app_client_protobuf_e2e` etc.) green from the generated source.

### 4. Complete manifest-driven CMake and flip the default

- **Problem**: `PYRAMID_BINDING_SOURCE_MODE=manifest` covers FlatBuffers
  schemas + JSON codec sources; the per-data-model-module C-ABI marshal loop
  still parses module identity from `pyramid_data_model_*_cabi_marshal.cpp`
  filenames, and codec-plugin targets and protoc/gRPC proto inputs remain on
  globs/fixed paths.
- **Approach**: add module identity (and plugin/proto-input roles) to
  `binding_manifest.json`; extend `pyramid_manifest.cmake` accessors with
  matching `cmake -P` tests; convert the remaining consumers; after one
  full-matrix verification (`default`, `all-on`, `all-off`,
  `flatbuffers-only` presets), flip the default from `glob` to `manifest` and
  keep `glob` one release as the fallback.
- **Acceptance**: manifest mode selects every generated source; glob and
  manifest modes produce identical target source lists on the pyramid tree
  and the `pim/test` tree; default flipped.

### 5. Confine residual domain knowledge to the compat policy

- **Problem**: two known leftovers from the generic-layout work —
  `BASE_TYPE_MAP` in `cpp_codegen.py` (hardcoded
  `pyramid.data_model.base.*`/`common.*` short-name mapping), and residual
  `pyramid_*` literals in the Ada service generator and some backend compat
  shims (never fully swept).
- **Approach**: move `BASE_TYPE_MAP` behind `PyramidCompatNamingPolicy`; run
  a repo-wide sweep of `pim/` for `pyramid`-literal tokens outside
  `binding_contract.py`'s compat policy, `topic_metadata/`, and explicitly
  compat-marked branches; add a source-guard test (the pattern
  `test_topic_metadata.py` already uses) asserting the generic code path
  contains no domain literals.
- **Acceptance**: source-guard test in place and green; byte-identical
  pyramid output.

### 6. Make the typed ROS2 marshal codec package-neutral

- **Problem**: `pim/ros2_marshal_codegen.py`/`pim/ros2_ir.py` are tied to
  `pyramid::domain_model`, `pyramid_msgs`, and `pyramid_ros2_codec.hpp`, so
  generic layouts cannot get a typed ROS2 codec.
- **Approach**: route the marshal codegen through the naming policy (package
  name, msg-package name, codec header name), keeping pyramid output
  byte-identical. Sequence this with (or after) the "typed ROS2 codec on the
  live wire" work in [`transport_plugins.md`](transport_plugins.md) §1 so the
  wire switch does not land on a domain-coupled generator.
- **Acceptance**: generic-layout fixture generates a compiling typed ROS2
  codec with zero `pyramid` tokens; pyramid `pyramid_msgs` output unchanged.

### 7. Fix the FlatBuffers `.cpp` JSON-bridge for nested packages

- **Problem** (carried from `pim/test_harness/FINDINGS.md`): the FlatBuffers
  backend's `.cpp` JSON-bridge include/call derivation assumes
  single-segment data-model packages; nested packages (e.g.
  `common_pim_components.authorisation` in the PIM fixture tree) need the
  full-namespace treatment already applied to the header path.
- **Approach**: reuse `cpp_codegen._native_namespace_for_type` (or its
  post-item-2 home) for the bridge include/call derivation; add a
  nested-package fixture to `test_generic_flatbuffers_protobuf.py`.
- **Acceptance**: FlatBuffers codecs for the `pim/test` tree compile;
  pyramid output unchanged.

### 8. PCL source-size watch (deferred, trigger-based)

`pcl_transport_shared_memory.c` (~2.2k lines) is the one PCL source that is
becoming hard to navigate. No action now — C transports are naturally
self-contained — but if the peer-identity threading work in
`transport_plugins.md` grows it further, split it along its existing internal
seams (ring/mailbox management, frame codec, gateway dispatch, plugin entry)
into `pcl_transport_shared_memory_*.c` files within the same target.

## Out of scope (tracked elsewhere)

- Typed ROS2 codec on the live wire, ROS2 actions, manifest-routed remote
  ingress / peer identity, capability direction/mode encoding →
  [`transport_plugins.md`](transport_plugins.md).
- Interaction-pattern contract options and `standard_topics.py` retirement
  into the `.proto` contract →
  [`pyramid_interaction_semantics.md`](../../../subprojects/PYRAMID/doc/architecture/pyramid_interaction_semantics.md)
  (design proposal; migration steps listed there).
