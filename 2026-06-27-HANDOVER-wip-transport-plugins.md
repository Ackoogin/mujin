# 2026-06-27 Handover: WIP Transport Plugins

## Current Status

Completed the generated-bindings migration and Ada gRPC shim retirement in the
working tree.

This covers:

- `doc/plans/PYRAMID/transport_plugins_wip.md` Section 2.A.3-A.5: retire the
  Ada gRPC shim.
- `doc/plans/PYRAMID/transport_plugins_wip.md` Section 2.G.2: keep generated
  bindings out of git and use build-local generated trees.

## Completed Changes

- C++ bindings now default to
  `${binaryDir}/generated/pyramid_cpp_bindings`.
- Ada bindings now default to
  `${binaryDir}/generated/pyramid_ada_bindings`.
- `.gitignore` excludes the old source-tree generated binding locations:
  `subprojects/PYRAMID/bindings/cpp/generated/`,
  `subprojects/PYRAMID/bindings/ada/generated/`, and
  `subprojects/PYRAMID/bindings/protobuf/cpp/`.
- The tracked generated C++ and Ada binding trees are removed from git.
- Tactical Objects protobuf codec support moved to
  `subprojects/PYRAMID/src/protobuf_support/`.
- The stale protobuf JSON C shim pair was removed.
- The `pyramid_grpc_c_shim` target was removed.
- Ada gRPC interop now loads:
  - `pyramid_grpc_coupled_plugin` as the PCL transport plugin.
  - `pyramid_codec_protobuf_tactical_objects` as the
    `application/protobuf` codec plugin.
- Added Linux Ada gRPC interop driver:
  `subprojects/PYRAMID/scripts/test_ada_grpc_cpp_interop_e2e.sh`.
- Updated active architecture and binding docs to describe build-local generated
  outputs and the plugin-based Ada gRPC path.

## Verification Run

- `python3 subprojects/PYRAMID/tests/test_binding_generation_dependencies.py --generator subprojects/PYRAMID/pim/generate_bindings.py --proto-dir subprojects/PYRAMID/proto`
- `cmake --preset all-off`
- `cmake --build --preset all-off-release --parallel $(nproc)`
- `ctest --preset all-off-release`
  - Result: 605/605 passed, with the existing
    `CodecPluginSwap.SwapJsonThenFlatbuffersNoRebuild` skip.
- `cmake --preset all-on`
- `cmake --build build-all-enabled --target tobj_grpc_server pyramid_grpc_coupled_plugin pyramid_codec_protobuf_tactical_objects pyramid_ada_all --parallel $(nproc)`
- `ctest --test-dir build-all-enabled -R 'tobj_ada_grpc_cpp_interop_e2e|pyramid_binding_generation_dependencies|ada_cpp_codec_roundtrip' --output-on-failure`
  - Result: 4/4 passed.
- `cmake --preset flatbuffers-only`
- `cmake --build --preset flatbuffers-only-release --parallel $(nproc)`
- `ctest --test-dir build-flatbuffers-only -R 'pyramid_ada_build_artifacts|ada_plugin_loader_abi|ada_generated_bindings_roundtrip|ada_cpp_codec_roundtrip|tobj_ada_|pyramid_bridge_e2e' --output-on-failure`
  - Result: 12/12 passed.
- Final hygiene:
  - `git diff --check`
  - conflict-marker scan excluding build and ignored generated trees
  - `git diff --name-only --diff-filter=U`
  - `git ls-files -u`

## Remaining Follow-Up

Run the default preset full suite if an extra confirmation pass is wanted after
this lands. The high-risk generated-binding migration itself has been exercised
across all-off, flatbuffers-only, and all-on/gRPC paths.
