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

## Generated Service Server Example

For generated PYRAMID service bindings, topic subscriptions still use
`Subscribe_*`, but service implementations can now be registered in one step
with the generated `Register_Services`.

The generated `Provided` package gives you:

- typed callback access types collected into `Service_Handlers`
- `Svc_*` wire-name constants for each service
- `Register_Services` to register all service ports for a container
- `Dispatch` to decode the request, call the typed callback, and encode the response

```ada
with System;
with Pyramid.Services.Tactical_Objects.Provided;

procedure Example_Server_Wiring is
   package Prov renames Pyramid.Services.Tactical_Objects.Provided;

   procedure Create_Requirement
     (Request  : in  Object_Interest_Requirement;
      Response : out Identifier);

   function Read_Requirement
     (Request : Query) return Prov.Object_Interest_Requirement_Array;

   procedure Create_Requirement
     (Request  : in  Object_Interest_Requirement;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := To_Unbounded_String ("new-interest-id");
   end Create_Requirement;

   function Read_Requirement
     (Request : Query) return Prov.Object_Interest_Requirement_Array
   is
      pragma Unreferenced (Request);
      Empty : Prov.Object_Interest_Requirement_Array (1 .. 0);
   begin
      return Empty;
   end Read_Requirement;

   Handlers : aliased constant Prov.Service_Handlers :=
     (On_Read_Match          => null,
      On_Create_Requirement  => Create_Requirement'Access,
      On_Read_Requirement    => Read_Requirement'Access,
      On_Update_Requirement  => null,
      On_Delete_Requirement  => null,
      On_Read_Detail         => null);
begin
   --  Inside your container On_Configure:
   Prov.Register_Services
     (Container => My_Container_Handle,
      Handlers  => Handlers'Access);
end Example_Server_Wiring;
```

In practice:

1. Write plain Ada business-logic subprograms with the generated typed signatures.
2. Fill a `Service_Handlers` record with the callbacks your container actually provides.
3. Call `Register_Services` once during `on_configure`.

This is similar to the generated C++ flow, but not identical. C++ generates a
`ServiceHandler` base class with virtual `handle*` methods; Ada now uses a
callback record instead of requiring edits to generated package bodies.
