# Ada PCL Example

This directory shows how Ada can call the PCL C ABI directly.

Files:

- `pcl_bindings.ads`: thin Ada import layer for the C API
- `pcl_component.ads` / `.adb`: OO wrapper with tagged `Component` and `Executor` types
- `pcl_sensor_demo.adb`: demo built on top of `Pcl_Component`
- `pcl_component_tests.adb`: Ada test runner for the OO wrapper

The example mirrors the C `external_io_bridge_example.c` flow:

1. an external thread/task receives data,
2. it calls `pcl_executor_post_incoming()`,
3. the executor drains that queue on its single logic thread,
4. the subscriber callback runs on the executor thread.

The Ada examples link against a prebuilt static `pcl_core` library instead of compiling `src/pcl` sources directly.

The Ada projects that compile C units use `-std=c11` for broader toolchain compatibility.

This matches the core CMake PCL targets (`pcl_core`, `pcl_transport_socket`), which are also configured with `C_STANDARD 11`.

By default the `.gpr` files expect:

- `MUJIN_ROOT=../..`
- `PCL_INCLUDE_DIR=<MUJIN_ROOT>/include`
- `PCL_LIB_DIR=<MUJIN_ROOT>/build/install/lib`
- `PCL_LIB_NAME=pcl_core`
- `PCL_SOCKET_LIB_NAME=pcl_transport_socket` (for generated-service examples)

If your static libraries live elsewhere (or have different basenames), override these with `-X` switches.

On Windows, GNAT (MinGW/GCC) requires GCC-compatible static libraries (`.a`). MSVC-produced `.lib` files from the default CMake/VS build are not link-compatible with GNAT.

Build:

```sh
gprbuild -P pcl_sensor_demo.gpr
```

Example with explicit overrides:

```sh
gprbuild -P pcl_sensor_demo.gpr \
  -XMUJIN_ROOT=../.. \
  -XPCL_INCLUDE_DIR=../../include \
  -XPCL_LIB_DIR=../../build/install/lib \
  -XPCL_LIB_NAME=pcl_core \
  -XPCL_SOCKET_LIB_NAME=pcl_transport_socket
```

Build OO tests:

```sh
gprbuild -P pcl_component_tests.gpr
```

Clean:

```sh
gprclean -P pcl_sensor_demo.gpr
```

Run:

```sh
.\bin\pcl_sensor_demo.exe
```

Run OO tests:

```sh
.\bin\pcl_component_tests.exe
```

Notes:

- `gprbuild` places generated objects and ALI files under `obj/` and the executable under `bin/`.
- If you previously built the demo with `gnatmake`, you may have stale `.ali`, `.o`, or `b~*` files beside the sources. Those are legacy by-products from that earlier build mode, not from the current `gprbuild` project.
