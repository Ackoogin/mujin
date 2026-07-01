# PCL/PYRAMID Offline SDK

This directory is a self-contained toolkit for building PCL/PYRAMID runtime
plugins from your **own** `.proto` contracts, without the parent monorepo and
without network access. Everything under here was produced once by a
maintainer running `package_sdk` inside the full repo; from this point on you
never need that repo again.

## What's in here

```
include/        PCL public headers + vendored header-only deps
                (nlohmann/json.hpp, tl/optional.hpp, flatbuffers/*.h)
lib/msvc/       prebuilt pcl_core.lib (Debug + Release)
lib/gnat/       prebuilt libpcl_core.a + libpcl_transport_*.a (GNAT/mingw)
plugins/        prebuilt, proto-agnostic transport plugins
                (pcl_transport_socket_plugin.dll, ..._shared_memory_plugin.dll)
tools/          flatc.exe (FlatBuffers schema compiler)
generator/      the proto -> bindings generator (pure Python, stdlib only)
proto/          starter .proto contracts -- edit or extend these
sdk_project/    standalone CMake project: builds C++ codec plugin DLLs
gnat/           standalone GNAT project: builds the Ada codec archive
scripts/        the three commands below
```

Only the **codec** plugins (JSON/FlatBuffers, one per component/service) are
proto-dependent and built by you. The **transport** plugins never change when
you edit `.proto` -- they ship prebuilt in `plugins/`.

## Prerequisites

- Python 3 (stdlib only -- nothing to `pip install`)
- CMake >= 3.21 + a C++17 compiler (MSVC, from a VS developer command prompt,
  is what `lib/msvc/*.lib` was built with)
- Optional, for Ada consumers: a GNAT toolchain (`gnat`/`gprbuild`/`g++`) on `PATH`

No network access is required at any step.

## Quick start

1. Edit/extend the contracts under `proto/` (or leave them as-is to try the
   starter set).
2. Generate bindings:
   ```bat
   scripts\generate_bindings.bat
   ```
3. Build the C++ codec plugin DLLs (from a VS developer command prompt):
   ```bat
   scripts\build_plugins.bat
   ```
   Produces `pyramid_codec_json_<component>.dll` and
   `pyramid_codec_flatbuffers_<component>.dll` under `build\`.
4. Optional -- build the C-ABI marshal archive, then build your own Ada
   client against `gnat\pyramid_sdk_ada.gpr`:
   ```bat
   scripts\build_ada.bat
   gprbuild -P your_client.gpr -XPYRAMID_SDK_ROOT=<sdk_root> ^
     -XGNATCOLL_OS=windows -XOS=Windows_NT
   ```
   `GNATCOLL_OS`/`OS` must be passed on the command line every time (Linux/macOS:
   `-XGNATCOLL_OS=unix|osx -XOS=linux|darwin`) -- see the header comment in
   `gnat\pyramid_sdk_ada.gpr` for why. Ada links only PCL + the C-ABI marshal
   layer, never codec-specific content -- see
   `../doc/architecture/sdk_packaging.md` upstream for the full rationale.

On Linux/macOS, use the `.sh` counterparts.

## Running against the plugins

Load `plugins\pcl_transport_socket_plugin.dll` (or `_shared_memory_plugin`)
plus the codec plugin(s) your component needs via `--codec-plugin` /
`PCL_CODEC_PLUGIN_PATH` (or a codec manifest), exactly as in the main
PYRAMID transport/codec plugin system. A client links only `pcl_core` +
the generated marshalling/facade code it compiles from `sdk_project\generated`
-- no wire codec, no transport is linked directly; both are composed at
run time from `plugins\` and your freshly built codec `.dll`.
