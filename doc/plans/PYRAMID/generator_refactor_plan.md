# Generator Monolith Refactor Plan — `cpp_codegen.py` / `ada_codegen.py`

Executes items **1** and **2** of
[`architecture_cleanup_plan.md`](architecture_cleanup_plan.md) (duplicate
parser consolidation, then the monolith split), updated for the state of the
tree after the pub/sub contract work (`7e97368`, `2aa1d63`).

**Date:** 2026-07-03

## Current state (post-pubsub baseline)

| File | Lines | Top-level structure |
|------|-------|---------------------|
| `pim/cpp_codegen.py` | 4197 | naming/rpc-symbol helpers; local `ProtoRpc/ProtoService/ProtoFile` + `parse_proto` (now a thin wrapper over `proto_parser`); namespace/topic derivation; `CppServiceGenerator` (~2 700 lines); `CppTypesGenerator`; `CppDataModelCodecGenerator`; `main()` |
| `pim/ada_codegen.py` | 3722 | same shape: `AdaGenericServiceGenerator`, `AdaServiceGenerator` (~1 650 lines), `AdaTypesGenerator`, `AdaDataModelCodecGenerator`, `main()` |

The pub/sub work already did **half of cleanup item 1**: both files'
`parse_proto` now delegate to `proto_parser` instead of running their own
regexes. What remains of the duplication is the local wrapper model classes
(`ProtoFile` carrying `full_proto`, plus local `ProtoRpc`/`ProtoService`)
and the cross-file helper imports.

Constraints discovered in the inventory:

- **Importers of `cpp_codegen`**: `generate_bindings.py`, `cabi_codegen.py`,
  `ros2_ir.py`, `backends/flatbuffers_backend.py`, `ada_cabi_codegen.py`,
  and `ada_codegen.py` itself (`_field_with_type`, `_proto_type_fqn`,
  `_resolve_message`, `_package_for_proto_type` — proto-model *resolution*
  helpers that have nothing C++-specific about them).
- **Importers of `ada_codegen`**: `generate_bindings.py`,
  `ada_cabi_codegen.py`.
- **Flat import model**: `pim/` modules import each other by bare name with
  `pim/` on `sys.path`; `backends/` is the one existing subpackage
  precedent.
- **SDK packaging** copies `pim/*.py` + `pim/backends/*.py` +
  `pim/topic_metadata/*.json` (`package_sdk.sh:131-135`, `.bat` mirror).
  New subpackages must be added there or a packaged SDK import-crashes
  (this exact failure mode already happened once — cleanup plan D3).
- **New shared mutable state from the pub/sub work**: both codegens hold a
  module-global `_CONTRACT_TOPIC_SPECS` dict, populated as a side effect of
  `_topics_for_proto()` and read through a shadowing `topic_spec()` that
  falls back to the legacy side-table. The split must not silently change
  when that cache is populated relative to when it is read.
- Both codegens duplicate `_is_pyramid_compat_service_package`,
  `_topics_for_proto`, `_strip_comments`, and two snake-casing helpers that
  also exist in `proto_parser` / `standard_topics` (with **potentially
  different acronym behaviour** — do not merge them without a differential
  test).

## Regression bar (every phase)

1. **Byte-identical output, both trees.** Before starting, generate
   baselines: legacy `proto/` tree with `--contract-layout pyramid`, and the
   `pim/test/` tree with the generic layout, all backends on. After each
   phase: `diff -qr` against the baseline must show no difference (the
   pattern `tests/test_generic_*.py` already implements).
2. **Suite green**: `python3 -m pytest subprojects/PYRAMID/tests -q` and
   `python3 -m unittest subprojects/PYRAMID/pim/test_proto_parser.py`.
3. **End of plan only** (slow): `viability_check.sh`, `build_comms_test.sh`,
   `build_plugin_load_test.sh`, `build_contract_routing_test.sh`, and SDK
   smoke: package with `package_sdk.sh`, then
   `python3 -c "import generate_bindings"` from the packaged `generator/`.

Each phase is a separate commit, independently revertable.

## Phase 0 — baseline + guard rails (small)

- Script the two baseline generations into a scratch dir and a `diff -qr`
  helper (may live in `tests/` as a fixture helper if not already covered).
- Add a trivial test asserting `cpp_codegen`/`ada_codegen` expose the names
  their importers use (grep-derived list), so shim regressions fail fast in
  Python rather than at generation time.

## Phase 1 — finish parser consolidation (cleanup item 1)

- Delete the local `ProtoRpc`/`ProtoService`/`ProtoFile` classes and
  `parse_proto` wrappers in both codegens; consume
  `proto_parser.ProtoFile` directly (it already carries
  `pattern`/`topic`/`qos`/`port_kind`, so the `full_proto` indirection and
  the `getattr(parsed, 'full_proto', None)` dance in `_topics_for_proto`
  disappear).
- Move the resolution helpers `ada_codegen` imports from `cpp_codegen`
  (`_resolve_message`, `_proto_type_fqn`, `_package_for_proto_type`,
  `_field_with_type`) into a shared `pim/proto_resolve.py` (or into
  `proto_parser` beside `ProtoTypeIndex`). `ada_codegen` must no longer
  import from `cpp_codegen`.
- **Accept:** no proto model/parser definitions remain in either codegen;
  byte-identical baselines; suite green.

## Phase 2 — explicit topic-spec resolution (kills the module globals)

- Introduce a small `TopicSpecResolver` (home: `binding_contract.py`)
  owning the contract-topic map and the legacy side-table fallback,
  constructed per generation run and passed into the generator classes.
  Delete both `_CONTRACT_TOPIC_SPECS` globals and the shadowing
  `topic_spec()` functions.
- Replace the `hasattr(spec, 'reliability_floor')` duck-typing (QoS-constant
  emission, ROS2 bind emission) with an explicit `spec.has_qos`-style flag
  on both `BindingTopic` and legacy `TopicSpec`.
- Dedupe the copied `_is_pyramid_compat_service_package` /
  `_topics_for_proto` pair into the same module.
- **Do not** merge the snake-casing helpers yet; first add a differential
  test running all payload/type names from both trees through each variant.
  Merge only the ones proven identical; keep divergent ones side by side
  with a comment stating the divergence.
- **Accept:** no module-level mutable dicts in either codegen;
  byte-identical baselines; suite green.

## Phase 3 — split `cpp_codegen.py` into `pim/cpp/`

Mechanical moves only — no behaviour change, no renames beyond module
paths. Mirror the `backends/` subpackage precedent.

| New module | Contents |
|------------|----------|
| `cpp/naming.py` | namespace derivation, rpc-symbol derivation, `BASE_TYPE_MAP` (+ its policy hook when cleanup item 5 lands), misc casing helpers |
| `cpp/types_gen.py` | `CppTypesGenerator` |
| `cpp/json_codec_gen.py` | `CppDataModelCodecGenerator` |
| `cpp/service_gen.py` | `CppServiceGenerator` |

`CppServiceGenerator` is ~2 700 lines on its own, so `service_gen.py` still
breaches the ~1.5 k target. Second step, still mechanical: carve its
per-artifact emitter method groups into sibling modules as free functions or
mixins along the existing section comments — service constants/topics,
dispatch/handler surface, client invoke surface, ROS2 bind hooks, component
facade. Only groups that touch disjoint `self` state move; anything
entangled stays in `service_gen.py` and is recorded as follow-up rather than
forced.

- Keep `cpp_codegen.py` as a pure re-export shim covering the grep-derived
  import surface (Phase 0 test enforces it).
- Update `package_sdk.sh`/`.bat` to copy `pim/cpp/` (and later `pim/ada/`).
- **Accept:** byte-identical baselines; suite green; no module over ~1.5 k
  lines except a documented `service_gen.py` remainder if entanglement
  forces it.

## Phase 4 — split `ada_codegen.py` into `pim/ada/`

Same recipe: `ada/naming.py`, `ada/types_gen.py`, `ada/codec_gen.py`
(`AdaDataModelCodecGenerator`), `ada/service_gen.py` (`AdaServiceGenerator`),
`ada/generic_service_gen.py` (`AdaGenericServiceGenerator`);
`ada_codegen.py` becomes the shim. Ada has been the historic pain point:
after the split, additionally object-compile the generated Ada for both
trees (`gnatgcc -c -gnat2020`, the FINDINGS.md bar) — line-based moves can
pass `diff -qr` yet mask an accidental emission change only if the diff is
run against stale baselines, and the Ada compile is cheap insurance.

## Phase 5 — migrate importers and retire the shims

- Point `generate_bindings.py`, `cabi_codegen.py`, `ada_cabi_codegen.py`,
  `ros2_ir.py`, `backends/flatbuffers_backend.py`, and tests at the new
  module paths.
- Shrink the shims to the externally consumed surface only; keep them one
  release (the SDK ships the generator to third parties), then delete.
- Run the full end-of-plan gate from the regression bar, including the
  packaged-SDK import smoke and the four harness scripts, plus the Windows
  `.bat` packaging parity check when a Windows environment is available.

## Risks

- **`CppServiceGenerator` entanglement** — mitigated by the "move whole
  class first, carve only disjoint groups second" rule; never force a split
  that requires threading new state.
- **Snake-casing helper skew** (`proto_parser.camel_to_lower_snake` vs
  `standard_topics._camel_to_snake` vs codegen locals) — differential test
  before any merge (Phase 2).
- **SDK packaging drift** — the flat `pim/*.py` copy silently drops new
  subpackages; both scripts change in the same commit as each new package,
  and the end-of-plan smoke check is the backstop.
- **Shim surface gaps** — Phase 0's export-surface test turns a missed
  re-export into an immediate unit failure.
- **Stale baselines** — regenerate baselines from `main` at Phase 0 and
  never regenerate mid-plan; the Ada object-compile in Phase 4 is the
  independent check.
