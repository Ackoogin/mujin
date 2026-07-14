# PCL/PYRAMID Offline SDK

This directory is a self-contained toolkit for building PCL/PYRAMID runtime
plugins from your **own** `.proto` contracts, without the parent monorepo and
without network access. Everything under here was produced once by a
maintainer running `package_sdk` inside the full repo. From this point on, a
downstream user never needs that repo again.

## What's in here

```
include/        PCL public headers + vendored header-only deps
                (nlohmann/json.hpp, tl/optional.hpp, flatbuffers/*.h)
lib/msvc/       prebuilt pcl_core.lib (Debug + Release)
lib/gnat/       prebuilt libpcl_core.a + libpcl_transport_*.a (GNAT/mingw)
plugins/        prebuilt transport plugins; a --gra package also contains the
                LA-CAL WebSocket transport and tested UCI OMS JSON codec
tools/          flatc.exe (FlatBuffers schema compiler)
generator/      the proto -> bindings generator (pure Python, stdlib only)
proto/          starter .proto contracts -- edit or extend these
sdk_project/    standalone CMake project: builds C++ codec plugin DLLs
                and small smoke-test executables
gnat/           standalone GNAT project: builds the Ada codec archive
scripts/        generation, plugin/Ada build, and end-to-end commands
MANIFEST.txt    full file listing written by the packaging step
```

The generated **codec** plugins (JSON/FlatBuffers, one per component/service)
are proto-dependent and built by you. The **transport** plugins do not change
when you edit `.proto` and ship prebuilt in `plugins/`. A distribution made
with `--gra` also carries the prebuilt, tested UCI 2.5 OMS JSON codec; it is not
a codec for arbitrary replacement contracts.

## Prerequisites

- Python 3 (stdlib only -- nothing to `pip install`)
- CMake >= 3.21 + a C++17 compiler (MSVC, from a VS developer command prompt,
  is what `lib/msvc/*.lib` was built with)
- Optional, for Ada consumers: a GNAT toolchain (`gnat`/`gprbuild`/`g++`) on `PATH`

No network access is required at any step.

## Maintainer: create a distribution

**One command** (inside the full monorepo, from a VS developer command prompt):

```bat
subprojects\PYRAMID\scripts\create_sdk.bat --verify
```
```bash
subprojects/PYRAMID/scripts/create_sdk.sh --verify
```

That runs the whole flow below end-to-end — configure, build, plugins, optional
Ada libs, package into `dist\pcl_pyramid_sdk`, and (`--verify`) build + smoke-test
the packaged SDK. Run `create_sdk.bat --help` for options (`--out`, `--config`,
`--no-ada`, `--skip-build`, `--build-dir`, `--proto-dir`, `--gra`, `--jobs`). `--proto-dir` selects
the contract tree copied into the package's `proto/` directory; it does not need
to be the repository's legacy `subprojects/PYRAMID/proto` tree. The numbered
steps below are the same flow spelled out for when you need to run or debug a
single stage.

For example, package the GRA runtime modules and verify the non-wire
A-GRA-vocabulary fixture against the same offline workflow a downstream user
receives:

```bat
subprojects\PYRAMID\scripts\create_sdk.bat --verify --gra ^
  --proto-dir subprojects\PYRAMID\pim\agra_example ^
  --out dist\pcl_pyramid_sdk_agra
```

Use `--skip-build` to reuse an existing `build-flatbuffers-only` tree. Pass
`--jobs 1` if the target's MSBuild/filesystem combination reports a transient
file-sharing failure during parallel custom commands.

This example tree is an A-GRA-vocabulary port-grammar fixture, not an OMS wire
contract. The run verifies its generated JSON/FlatBuffers plugins and
separately verifies that the packaged OMS JSON codec and LA-CAL WebSocket
transport load with the expected PCL ABI.

Run these steps inside the full monorepo before handing the SDK directory to a
downstream user.

1. Configure and build the repo with FlatBuffers and generated C++ bindings:
   ```bat
   cmake --preset flatbuffers-only
   cmake --build --preset flatbuffers-only-release --parallel %NUMBER_OF_PROCESSORS%
   ```
   ```bash
   cmake --preset flatbuffers-only
   cmake --build --preset flatbuffers-only-release --parallel "$(nproc)"
   ```
2. Build the plugin-focused artifacts used by the package step:
   ```bat
   subprojects\PYRAMID\scripts\build_plugins.bat --build-dir build-flatbuffers-only
   ```
   ```bash
   subprojects/PYRAMID/scripts/build_plugins.sh --build-dir build-flatbuffers-only
   ```
3. Optional, but required for Ada consumers: build GNAT-compatible PCL archives:
   ```bat
   subprojects\PCL\scripts\build_gnat_pcl_static_libs.bat build-flatbuffers-only\ada_gnat_pcl
   ```
   ```bash
   subprojects/PCL/scripts/build_gnat_pcl_static_libs.sh build-flatbuffers-only/ada_gnat_pcl
   ```
   `package_sdk` will attempt this automatically when the archive is missing
   and a suitable GCC/GNAT toolchain is on `PATH`, but running it explicitly
   makes packaging failures easier to diagnose.
4. Package the deploy directory:
   ```bat
   subprojects\PYRAMID\scripts\package_sdk.bat --build-dir build-flatbuffers-only --clean
   ```
   ```bash
   subprojects/PYRAMID/scripts/package_sdk.sh --build-dir build-flatbuffers-only --clean
   ```
   The default output is `dist\pcl_pyramid_sdk` on Windows and
   `dist/pcl_pyramid_sdk` on Linux/macOS. Use `--out <dir>` to select another
   destination. Add `--proto-dir <contract-tree>` to choose the contracts
   shipped under the SDK's `proto/`; the default is
   `subprojects/PYRAMID/proto`.
   For a GRA runtime package, add `--gra` to steps 2 and 4 and pass the chosen
   contract tree with `--proto-dir` in step 4.
5. Inspect the package:
   ```bat
   type dist\pcl_pyramid_sdk\MANIFEST.txt
   ```
   ```bash
   sed -n '1,120p' dist/pcl_pyramid_sdk/MANIFEST.txt
   ```
   Confirm that it includes `include/`, `lib/`, `plugins/`, `tools/flatc`,
   `generator/`, `proto/`, `sdk_project/`, `gnat/`, `scripts/`, and this
   `README.md`.

Copy the resulting `pcl_pyramid_sdk` directory to the target machine. The
directory is the distribution; it is not an installer and does not require the
monorepo.

## Downstream: verify the distribution

**One command** (from a VS developer command prompt, in the SDK root):

```bat
scripts\build_sdk.bat
```
```bash
scripts/build_sdk.sh
```

That generates bindings from `proto\`, builds the C++ codec plugins, and runs the
smoke tests — the whole verify flow end-to-end. Add `--ada` to also build the Ada
archive, `--no-smoke` for the plain "build my contracts" path, or `--help` for all
options. The numbered steps below are the same flow spelled out stage by stage.

Before replacing the starter contracts, run the bundled contracts through the
same offline workflow the user will use later:

1. Generate C++ and Ada bindings from `proto/`:
   ```bat
   scripts\generate_bindings.bat
   ```
   ```bash
   scripts/generate_bindings.sh
   ```
2. Build the generated C++ codec plugins and run the smoke-test executables:
   ```bat
   scripts\build_plugins.bat --smoke-tests
   ```
   ```bash
   scripts/build_plugins.sh --smoke-tests
   ```
   This builds:
   - `sdk_codec_plugin_load_smoke`: loads every generated codec plugin and
     confirms JSON and, when present, FlatBuffers codecs registered.
   - `sdk_gra_plugin_load_smoke` (only in a `--gra` package): loads the UCI OMS
     JSON codec and checks the LA-CAL WebSocket transport ABI and PUBSUB
     capability without needing a live CAL server.
   - `sdk_tactical_objects_fail_closed_smoke`: starter-contract check that the
     generated tactical_objects facade rejects encode/decode calls when no
     matching codec is registered.
3. If a smoke test fails, keep the `build/` directory and rerun CTest directly:
   ```bat
   ctest --test-dir build --output-on-failure -C Release -R "^sdk_"
   ```
   ```bash
   ctest --test-dir build --output-on-failure -C Release -R '^sdk_'
   ```

The tactical_objects smoke app is only added when the selected contract tree's
tactical_objects generated facade exists. If you later replace `proto/` with a
contract set that does not define tactical_objects, the generic codec plugin
load smoke test still runs.

## Downstream: build your contracts

**One command** after editing `proto/`: `scripts\build_sdk.bat --no-smoke`
(add `--ada` for the Ada archive). The numbered steps below break out each stage.

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
   `pyramid_codec_flatbuffers_<component>.dll` under `build\` on Windows, or
   `libpyramid_codec_json_<component>.so` and
   `libpyramid_codec_flatbuffers_<component>.so` under `build/` on Linux/macOS.
   Add `--smoke-tests` when you want to compile and run the SDK validation
   executables as part of the build.
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
plus the codec plugin(s) your component needs via `--codec-plugin`, a codec
manifest (`PCL_CODEC_MANIFEST` / `--codec-manifest`), or — for Ada clients —
the path-separated `PYRAMID_CODEC_PLUGINS` environment variable, exactly as
in the main PYRAMID transport/codec plugin system. A client links only `pcl_core` +
the generated marshalling/facade code it compiles from `sdk_project\generated`
-- no wire codec, no transport is linked directly; both are composed at
run time from `plugins\` and your freshly built codec `.dll`.
