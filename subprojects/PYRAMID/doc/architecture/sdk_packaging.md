# Offline PCL/PYRAMID SDK Packaging

## Purpose

This page describes the maintainer/user split behind `package_sdk.bat`/`.sh`:
producing a self-contained deploy directory that lets a downstream developer
build **their own** `.proto` contracts into runtime plugins, offline, without
this monorepo. It exists because this repo is heavy (LAPKT, BT.CPP,
googletest, FlatBuffers are all fetched from git during a normal configure)
and because real deployment targets are firewalled -- no network access at
all. See [pcl_pyramid_binding_generation_overview.md](pcl_pyramid_binding_generation_overview.md)
and [transport_codec_plugin_system.md](transport_codec_plugin_system.md) for
the runtime concepts this builds on.

## The split

| Role | Runs | Produces |
|------|------|----------|
| Maintainer | `package_sdk.bat`/`.sh`, inside this repo, against an already-built `build-flatbuffers-only`-style build dir | A deploy directory (default `dist/pcl_pyramid_sdk`) |
| Downstream user | `scripts/generate_bindings` -> `scripts/build_plugins` (optionally `--smoke-tests`, then `scripts/build_ada`), inside the deploy directory only | Codec plugin `.dll`/`.so`, smoke-test executables, and optionally an Ada codec archive built from their own `proto/` tree |

Everything the maintainer step needs (source, FetchContent-resolved
dependencies) stays behind; everything the user step needs travels in the
deploy directory. The user never touches this monorepo and never needs
network access.

## What's prebuilt vs. what the user builds

PCL (`subprojects/PCL/src`) is pure C with zero external dependencies (only
`Threads`/`ws2_32`), so it packages cleanly as a binary: `lib/msvc/*.lib` for
the C++ plugin build, `lib/gnat/*.a` for Ada clients (GNAT/MinGW and MSVC
produce mutually incompatible object formats, which is why both are shipped
separately -- see `subprojects/PCL/scripts/build_gnat_pcl_static_libs.*`).

Transport plugins (`pcl_transport_socket_plugin`, `pcl_transport_shared_memory_plugin`)
are proto-agnostic -- they never change when a user edits `.proto` -- so they
ship prebuilt in `plugins/`, the same way `stage_plugin_deploy.bat` already
stages them for known, already-built components.

Codec plugins (`pyramid_codec_json_<component>`, `pyramid_codec_flatbuffers_<component>`)
are proto-dependent: one per component in the user's own `proto/` tree. These
cannot be prebuilt by the maintainer, so the deploy directory instead ships
everything needed to build them offline:

- `generator/` -- a verbatim copy of `pim/*.py`, the proto -> bindings
  generator. It is pure Python stdlib (no pip dependencies), so it just needs
  a Python 3 interpreter.
- `tools/flatc` -- a prebuilt FlatBuffers schema compiler (needed at generate
  time to turn a `.fbs` schema into a C++ header; the FlatBuffers C++ runtime
  itself is header-only, vendored under `include/external/flatbuffers/`).
- `include/external/{nlohmann,tl}/` -- the JSON codec's only dependencies,
  both vendored single headers already in this repo
  (`subprojects/PYRAMID/core/external/`), so JSON needs no external fetch at
  all.
- `sdk_project/CMakeLists.txt` -- a standalone CMake project (no
  `FetchContent`, no parent monorepo) that glob-discovers the generated
  marshal/codec/plugin sources and builds the codec plugin `.dll`s against
  the imported `pcl_core` static lib. It mirrors the shape of
  `subprojects/PYRAMID/CMakeLists.txt`'s codec-plugin section at a much
  smaller scale. It also defines no-framework smoke-test executables and a
  `pyramid_sdk_smoke_tests` target so the packaged SDK can prove it can load
  the generated plugins without depending on GoogleTest or the monorepo.
- `gnat/pyramid_sdk_ada.gpr` -- a source-contributor GNAT project (no
  `Main`) that a downstream Ada client project `with`s, exposing PCL's Ada
  bindings and the generated Ada sources, and linking the GNAT-compatible
  C-ABI marshal archive.

## Ada depends on PCL + the C-ABI layer only

Ada consumers link **only** PCL and the C-ABI marshal layer
(`to_c`/`from_c`/`_c_free`, generated as `*_cabi_marshal.cpp`) -- never
codec-specific (wire codec) content. Ada has its own independent, pure-Ada
JSON codec and always talks to the wire through a loaded codec plugin
`.dll`/`.so`; it never calls `toJson`/`fromJson`/`toBinary`/`fromBinary`
directly. This is why the GNAT-compatible archive built by
`build_gnat_pyramid_cabi_marshal_libs.*` (both in this repo and in the SDK's
`gnat/` template) is named `pyramid_generated_cabi_marshal`, not
"...codec" or "...flatbuffers" -- those names described what the archive used
to also contain (the wire codec, before it was trimmed down to marshal-only)
and were actively misleading about what Ada actually depends on. The single
`PYRAMID_CABI_LIB_DIR`/`PYRAMID_CABI_LIB_NAME` external variables (consumed by
every Ada `.gpr` project in this repo) replace what used to be a redundant
`PYRAMID_GEN_LIB_*`/`PYRAMID_CODECS_LIB_*` pair pointing at the same archive
under two names.

## Deploy directory layout

```
pcl_pyramid_sdk/
  README.md                      quick start for the downstream user
  include/pcl, include/external  PCL headers + vendored nlohmann/tl/flatbuffers
  lib/msvc, lib/gnat             prebuilt PCL static libs (both toolchains)
  plugins/                       prebuilt transport plugins
  tools/flatc(.exe)              prebuilt FlatBuffers schema compiler
  generator/                     pim/*.py generator (pure Python)
  proto/pyramid/                 starter .proto contracts to edit/extend
  sdk_project/                   standalone CMake project (C++ codec plugins + smoke tests)
  gnat/                          standalone GNAT project (Ada C-ABI archive + bindings)
  scripts/                       generate_bindings, build_plugins, build_ada
  MANIFEST.txt                   full file listing
```

## Maintainer packaging flow

1. Configure and build a FlatBuffers-capable repo build, normally via
   `cmake --preset flatbuffers-only` and
   `cmake --build --preset flatbuffers-only-release`.
2. Build the plugin artifacts package_sdk needs:
   `subprojects/PYRAMID/scripts/build_plugins.bat --build-dir build-flatbuffers-only`
   or the `.sh` equivalent. This produces PCL, `flatc`, transport plugins, and
   generated-code dependent codec plugin artifacts in the expected build tree.
3. For Ada-capable SDKs, build GNAT-compatible PCL archives explicitly with
   `subprojects/PCL/scripts/build_gnat_pcl_static_libs.* build-flatbuffers-only/ada_gnat_pcl`.
   `package_sdk` can attempt this automatically, but the explicit step makes a
   missing GNAT/GCC toolchain visible before packaging.
4. Run `subprojects/PYRAMID/scripts/package_sdk.* --build-dir build-flatbuffers-only --clean`
   with `--out <dir>` when the default `dist/pcl_pyramid_sdk` is not desired.
5. Check `<out>/MANIFEST.txt` and run the downstream verification flow from the
   packaged directory before release.

## Downstream verification flow

Inside the packaged SDK, run `scripts/generate_bindings.*` followed by
`scripts/build_plugins.* --smoke-tests`. The `--smoke-tests` flag builds the
normal codec plugins, builds `pyramid_sdk_smoke_tests`, and runs CTest tests
whose names start with `sdk_`.

The generic smoke app loads every generated codec plugin and verifies the codec
registry receives JSON and, when FlatBuffers plugins were built, FlatBuffers
codecs. With the bundled starter contracts, a second smoke app also compiles
the tactical_objects consumed facade and verifies it fails closed when no codec
is registered. That second app is guarded by CMake so custom downstream proto
sets that omit tactical_objects can still use the generic plugin-load smoke.

## Key files

| Area | Files |
|------|-------|
| Maintainer packaging script | `subprojects/PYRAMID/scripts/package_sdk.bat`, `.sh` |
| SDK project template (copied verbatim into the deploy dir) | `subprojects/PYRAMID/sdk_template/` |
| SDK smoke-test sources | `subprojects/PYRAMID/sdk_template/sdk_project/smoke_tests/` |
| C-ABI marshal archive build (both monorepo and SDK copies) | `subprojects/PYRAMID/scripts/build_gnat_pyramid_cabi_marshal_libs.bat`, `.sh` |
| PCL GNAT static libs | `subprojects/PCL/scripts/build_gnat_pcl_static_libs.bat`, `.sh` |
| Existing per-component plugin staging (different problem, see above) | `subprojects/PYRAMID/scripts/stage_plugin_deploy.bat`, `.sh` |
