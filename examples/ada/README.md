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

On Windows, the GNAT Community toolchain is MinGW/GCC-based, while the main repo build uses MSVC. Because those static libraries are not link-compatible, the Ada demo uses `gprbuild` to compile the small PCL C sources directly with the same GNU toolchain as the Ada code.

Build:

```sh
gprbuild -P pcl_sensor_demo.gpr
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
