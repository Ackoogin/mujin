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
`Subscribe_*`, but service implementations are registered with
`pcl_container_add_service()` via `Pcl_Bindings.Add_Service`.

The generated `Provided` package gives you:

- `Handle_*` stubs for business logic
- `Svc_*` wire-name constants for each service
- `Dispatch` to decode the request, call the typed handler, and encode the response

The missing step is a small C-convention trampoline that PCL can call:

```ada
with Interfaces.C;
with Interfaces.C.Strings;
with System;
with Pcl_Bindings;
with Pyramid.Services.Tactical_Objects.Provided;

procedure Example_Server_Wiring is
   package Prov renames Pyramid.Services.Tactical_Objects.Provided;

   function On_Configure
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, On_Configure);

   function Create_Requirement_Service
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Create_Requirement_Service);

   function On_Configure
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (User_Data);
      Service_Name : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Prov.Svc_Create_Requirement);
      Type_Name : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("application/json");
      Port : Pcl_Bindings.Pcl_Port_Access;
   begin
      Port := Pcl_Bindings.Add_Service
        (Container    => Self,
         Service_Name => Service_Name,
         Type_Name    => Type_Name,
         Handler      => Create_Requirement_Service'Access,
         User_Data    => System.Null_Address);
      Interfaces.C.Strings.Free (Service_Name);
      Interfaces.C.Strings.Free (Type_Name);

      if Port = null then
         return Pcl_Bindings.PCL_ERR_STATE;
      end if;

      return Pcl_Bindings.PCL_OK;
   end On_Configure;

   function Create_Requirement_Service
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx, User_Data);
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Prov.Dispatch
        (Channel       => Prov.Ch_Create_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);

      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String ("application/json");
      return Pcl_Bindings.PCL_OK;
   end Create_Requirement_Service;
begin
   null;
end Example_Server_Wiring;
```

In practice:

1. Implement the generated `Handle_*` routines in the generated package body.
2. Register one `Add_Service` callback per `Svc_*` name during `on_configure`.
3. In each callback, call `Dispatch` with the matching `Ch_*` enum.

For deferred replies, return `PCL_PENDING`, hold on to the supplied
`Pcl_Svc_Context_Access`, and later complete the response with
`Pcl_Bindings.Service_Respond`.
