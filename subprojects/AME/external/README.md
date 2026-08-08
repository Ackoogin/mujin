# AME's third-party sources

AME is deployed into air-gapped environments, where nothing can be downloaded
while the build is running. Every library AME needs is therefore checked into
this directory, the way [PCL](../../PCL) and [PYRAMID](../../PYRAMID) are
already part of the repository. A build that reaches this directory needs no
network access at any point.

Only the files AME compiles or includes are here. Upstream test suites,
documentation, demo programs and ports for platforms AME does not target are
left out, which is the difference between about 36 MB and about 190 MB.

## What is here

| Directory | Version | Size | Files | What AME uses it for |
|-----------|---------|------|-------|----------------------|
| `sdl2` | release-2.30.10 | 18.6 MB | 1236 | Window, input and OpenGL context for the authoring tool |
| `asio` | asio-1-28-0 | 5.5 MB | 675 | Standalone Asio, which websocketpp needs |
| `imgui` | v1.91.6-docking | 3.7 MB | 17 | Immediate-mode user interface for the authoring tool |
| `behaviortree_cpp` | 4.6.2 | 3.0 MB | 272 | Behaviour tree engine that runs compiled plans |
| `lapkt` | Devel2.0 | 1.5 MB | 254 | Classical planner behind `ame::Planner` |
| `googletest` | v1.14.0 | 1.0 MB | 45 | Test framework for every AME test binary |
| `websocketpp` | 0.8.2 | 0.8 MB | 95 | Foxglove bridge WebSocket server, only when `AME_FOXGLOVE` is on |
| `nlohmann_json` | v3.11.3 | 0.8 MB | 1 | The authoring project format, run records and reports |
| `imgui_node_editor` | develop | 0.3 MB | 14 | Canvas behind the domain, plan and behaviour-tree views |
| `tinyfiledialogs` | master | 0.3 MB | 3 | Native open and save dialogs |
| `jetbrains_mono` | v2.304 | 0.3 MB | 2 | The authoring tool's interface font |
| `stb` | 5736b15…6a3aae9 | 0.1 MB | 2 | Writing the self-test screenshot as a PNG |

`manifest.json` records the same list in machine-readable form, with the
upstream repository each copy came from and a digest over its contents.

## How the build finds them

[`../cmake/AmeDependencies.cmake`](../cmake/AmeDependencies.cmake) takes each
dependency from this directory when it is present, and downloads it when it is
not. The fallback exists so that a developer who has deleted this directory, or
who is adding a dependency before vendoring it, still has a working build.

Configure with `AME_REQUIRE_VENDORED_DEPENDENCIES=ON` to turn a missing copy
into an error instead. An air-gapped build should always set it, so that it
fails immediately and says what is missing rather than hanging on a network
call. AME's own `default` and `no-authoring` presets set it.

## Updating or adding one

Do not edit these files by hand. They are written by
[`../scripts/vendor_dependencies.py`](../scripts/vendor_dependencies.py), which
holds the version of each dependency and the list of files kept from it.

To change a version, edit the `tag` in that script's `DEPENDENCIES` table and
re-run it on a machine with network access:

```bash
python3 subprojects/AME/scripts/vendor_dependencies.py --fetch
```

To add a new dependency, add an entry to the same table saying what is kept and
why, add it to the module above so the build can find it, and add it to the
`required` list in
[`../tests/test_vendored_dependencies.cpp`](../tests/test_vendored_dependencies.cpp)
so that a future change cannot quietly leave it out.

To check the checked-in copies against the manifest without changing anything:

```bash
python3 subprojects/AME/scripts/vendor_dependencies.py --verify
```

## Licences

Each copy keeps its upstream licence file. AME does not modify any of these
sources; where a build needs different behaviour it is expressed as a CMake
option rather than as a patch, so that the copies stay comparable with upstream.
