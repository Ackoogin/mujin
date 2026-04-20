# Ada PCL Example

This directory shows how Ada can call the PCL C ABI directly.

Files:

- `pcl_bindings.ads`: thin Ada import layer for the C API
- `pcl_component.ads` / `.adb`: OO wrapper with tagged `Component` and `Executor` types
- `pcl_sensor_demo.adb`: demo built on top of `Pcl_Component`

Ada test runners live under `subprojects/PYRAMID/tests/ada`. Generated Ada
bindings live under `subprojects/PYRAMID/bindings/ada/generated`.

The example mirrors the C `external_io_bridge_example.c` flow:

1. an external thread/task receives data,
2. it calls `pcl_executor_post_incoming()`,
3. the executor drains that queue on its single logic thread,
4. the subscriber callback runs on the executor thread.

The Ada examples link against a prebuilt static `pcl_core` library instead of compiling `src/pcl` sources directly.

The Ada projects that compile C units use `-std=c11` for broader toolchain compatibility.

This matches the core CMake PCL targets (`pcl_core`, `pcl_transport_socket`), which are also configured with `C_STANDARD 11`.

By default the `.gpr` files use historical relative defaults. In this workspace,
the most reliable approach is to pass explicit paths from `subprojects/PYRAMID/examples/ada`.

Recommended values:

- `MUJIN_ROOT=../../../..`
- `PCL_INCLUDE_DIR=../../../../subprojects/PCL/include`
- `PCL_LIB_DIR=../../../../subprojects/PCL/build/ada_gnat_pcl` or another directory containing GCC-compatible PCL static libraries
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
  -XMUJIN_ROOT=../../../.. \
  -XPCL_INCLUDE_DIR=../../../../subprojects/PCL/include \
  -XPCL_LIB_DIR=../../../../subprojects/PCL/build/ada_gnat_pcl \
  -XPCL_LIB_NAME=pcl_core \
  -XPCL_SOCKET_LIB_NAME=pcl_transport_socket
```

Build those GCC-compatible PCL libraries from the workspace root with:

```sh
subprojects/PCL/scripts/build_gnat_pcl_static_libs.sh
```

or on Windows:

```bat
subprojects\PCL\scripts\build_gnat_pcl_static_libs.bat
```

Build the OO wrapper test from the test tree:

```sh
cd ../../tests/ada
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

Run the OO wrapper test:

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
with Interfaces.C;
with Interfaces.C.Strings;
with Pcl_Bindings;
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

   --  Optional: route a generated consumed endpoint to a specific peer.
   declare
      Peer      : aliased Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("bridge-client");
      Peer_List : aliased constant System.Address := Peer'Address;
      Route     : aliased Pcl_Bindings.Pcl_Endpoint_Route :=
        (Endpoint_Name => Interfaces.C.Strings.New_String
           (Prov.Svc_Create_Requirement),
         Endpoint_Kind => Pcl_Bindings.PCL_ENDPOINT_CONSUMED,
         Route_Mode    => Pcl_Bindings.PCL_ROUTE_REMOTE,
         Peer_Ids      => Peer_List,
         Peer_Count    => 1);
   begin
      Pcl_Bindings.Set_Endpoint_Route
        (Exec  => My_Executor_Handle,
         Route => Route'Access);
      Interfaces.C.Strings.Free (Route.Endpoint_Name);
      Interfaces.C.Strings.Free (Peer);
   end;
end Example_Server_Wiring;
```

In practice:

1. Write plain Ada business-logic subprograms with the generated typed signatures.
2. Fill a `Service_Handlers` record with the callbacks your container actually provides.
3. Call `Register_Services` once during `on_configure`.
4. Apply `Pcl_Bindings.Port_Set_Route` / `Set_Endpoint_Route` when a generated
   `provided` or `consumed` endpoint should be local, remote, or both.

This is similar to the generated C++ flow, but not identical. C++ generates a
`ServiceHandler` base class with virtual `handle*` methods; Ada now uses a
callback record instead of requiring edits to generated package bodies.

### Triggering a Generated Service in a Local Test

Application examples should call the generated service binding instead of
constructing `Pcl_Msg` payloads or choosing JSON/FlatBuffers paths directly.
The binding owns the stable service name, content type, request encoding, and
response decoding.

```ada
with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid.Services.Tactical_Objects.Provided;

procedure Example_Invoke_Create_Requirement is
   package Prov renames Pyramid.Services.Tactical_Objects.Provided;

   procedure Response_Cb
     (Resp      : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address)
   is
      pragma Unreferenced (User_Data);
      Interest_Id : constant Identifier :=
        Prov.Decode_Create_Requirement_Response (Resp);
   begin
      pragma Assert (Interest_Id = To_Unbounded_String ("created-by-test"));
   end Response_Cb;

   Request_Value : Object_Interest_Requirement;
begin
   Request_Value.Source := Source_Local;
   Request_Value.Policy := Policy_Obtain;
   Request_Value.Dimension :=
     new Dimension_Array'(1 => Dimension_Sea_Surface);
   Request_Value.Has_Val_Point := True;
   Request_Value.Val_Point.Position.Latitude := 0.1;
   Request_Value.Val_Point.Position.Longitude := 0.3;

   Prov.Invoke_Create_Requirement
     (Executor     => Exec,
      Request      => Request_Value,
      Callback     => Response_Cb'Access,
      Content_Type => Prov.Json_Content_Type);

   Pcl_Bindings.Spin_Once (Exec, 0);
end Example_Invoke_Create_Requirement;
```

Use lower-level `Pcl_Bindings.Invoke_Service` or
`Pcl_Bindings.Post_Service_Request` only in binding tests that explicitly need
to prove the generated trampoline, content-type negotiation, or executor queue
semantics.  In those tests, keep the raw message construction in the test body
and do not copy it into reusable component examples.

`Post_Incoming` does not work for service calls.  It only dispatches to
subscriber (`PCL_PORT_SUBSCRIBER`) ports.  Service ports
(`PCL_PORT_SERVICE`) are reached through service invocation APIs.

## Routing and Peer Configuration

For Ada, the routing rules are the same as the C API:

- concrete ports use `Pcl_Bindings.Port_Set_Route`
- consumed/client endpoints use `Pcl_Bindings.Set_Endpoint_Route`
- remote peers are registered with `Pcl_Bindings.Register_Transport`
- `provided` and `consumed` do not imply locality on their own

The route modes are:

- `Pcl_Bindings.PCL_ROUTE_LOCAL`
- `Pcl_Bindings.PCL_ROUTE_REMOTE`
- `Pcl_Bindings.PCL_ROUTE_LOCAL + Pcl_Bindings.PCL_ROUTE_REMOTE`

### Registering Peer Transports

If one executor needs to talk to multiple peers, create one transport per peer
and register each under a stable logical peer ID.

```ada
declare
   Exec        : constant Pcl_Bindings.Pcl_Executor_Access :=
     Pcl_Bindings.Create_Executor;
   Planner_Id  : Interfaces.C.Strings.chars_ptr :=
     Interfaces.C.Strings.New_String ("planner");
   Bridge_Id   : Interfaces.C.Strings.chars_ptr :=
     Interfaces.C.Strings.New_String ("bridge_b");
   Planner_Host : Interfaces.C.Strings.chars_ptr :=
     Interfaces.C.Strings.New_String ("127.0.0.1");
   Bridge_Host  : Interfaces.C.Strings.chars_ptr :=
     Interfaces.C.Strings.New_String ("127.0.0.1");
   Planner_Tx  : constant Pcl_Bindings.Pcl_Socket_Transport_Access :=
     Pcl_Bindings.Create_Socket_Client (Planner_Host, 7001, Exec);
   Bridge_Tx   : constant Pcl_Bindings.Pcl_Socket_Transport_Access :=
     Pcl_Bindings.Create_Socket_Client (Bridge_Host, 7002, Exec);
begin
   Pcl_Bindings.Set_Socket_Peer_Id (Planner_Tx, Planner_Id);
   Pcl_Bindings.Set_Socket_Peer_Id (Bridge_Tx, Bridge_Id);

   Pcl_Bindings.Register_Transport
     (Exec,
      Planner_Id,
      Pcl_Bindings.Get_Socket_Transport (Planner_Tx));

   Pcl_Bindings.Register_Transport
     (Exec,
      Bridge_Id,
      Pcl_Bindings.Get_Socket_Transport (Bridge_Tx));

   Interfaces.C.Strings.Free (Planner_Id);
   Interfaces.C.Strings.Free (Bridge_Id);
   Interfaces.C.Strings.Free (Planner_Host);
   Interfaces.C.Strings.Free (Bridge_Host);
end;
```

Use peer IDs that describe the remote executor role, not the raw host/port.

### Local `provided`, Remote `consumed`

This is the most common mixed deployment:

- the container implements a local service
- the same executor calls some other service on a named remote peer

```ada
declare
   package Prov renames Pyramid.Services.Tactical_Objects.Provided;

   Peer      : Interfaces.C.Strings.chars_ptr :=
     Interfaces.C.Strings.New_String ("planner");
   Peer_List : aliased constant System.Address := Peer'Address;
   Route     : aliased Pcl_Bindings.Pcl_Endpoint_Route :=
     (Endpoint_Name => Interfaces.C.Strings.New_String
        (Prov.Svc_Read_Detail),
      Endpoint_Kind => Pcl_Bindings.PCL_ENDPOINT_CONSUMED,
      Route_Mode    => Pcl_Bindings.PCL_ROUTE_REMOTE,
      Peer_Ids      => Peer_List,
      Peer_Count    => 1);
begin
   --  Local service registration on the container:
   Prov.Register_Services
     (Container => My_Container_Handle,
      Handlers  => Handlers'Access);

   --  Remote consumed route on the executor:
   Pcl_Bindings.Set_Endpoint_Route
     (Exec  => My_Executor_Handle,
      Route => Route'Access);

   Interfaces.C.Strings.Free (Route.Endpoint_Name);
   Interfaces.C.Strings.Free (Peer);
end;
```

`Register_Services` creates the service ports. Without extra route changes, they
behave as local endpoints.

### Remote `provided`

If a generated `provided` service should be exposed remotely, route the concrete
service port after registration.

Today that means:

1. register the service with `Register_Services`
2. retain the returned `Pcl_Port_Access` if you are wiring services manually,
   or extend your generated helper usage with a post-registration route call

For a hand-written port example:

```ada
declare
   Content_Type : constant String := Prov.Json_Content_Type;
   Peer      : Interfaces.C.Strings.chars_ptr :=
     Interfaces.C.Strings.New_String ("bridge_a");
   Peer_List : aliased constant System.Address := Peer'Address;
   Service   : constant Pcl_Bindings.Pcl_Port_Access :=
     Pcl_Bindings.Add_Service
       (Container    => My_Container_Handle,
        Service_Name => Interfaces.C.Strings.New_String ("track.update"),
         Type_Name    => Interfaces.C.Strings.New_String (Content_Type),
        Handler      => My_Service_Trampoline'Access,
        User_Data    => System.Null_Address);
begin
   Pcl_Bindings.Port_Set_Route
     (Port       => Service,
      Route_Mode => Pcl_Bindings.PCL_ROUTE_REMOTE,
      Peer_Ids   => Peer_List,
      Peer_Count => 1);

   Interfaces.C.Strings.Free (Peer);
end;
```

If you want a local-plus-remote `provided` endpoint, use:

```ada
Pcl_Bindings.PCL_ROUTE_LOCAL + Pcl_Bindings.PCL_ROUTE_REMOTE
```

with the same peer list.

### Remote Subscriber Allow-List

A subscriber may be configured to accept traffic only from specific peers.

```ada
declare
   Content_Type : constant String := Prov.Json_Content_Type;
   Peer      : Interfaces.C.Strings.chars_ptr :=
     Interfaces.C.Strings.New_String ("bridge_b");
   Peer_List : aliased constant System.Address := Peer'Address;
   Port      : constant Pcl_Bindings.Pcl_Port_Access :=
     Pcl_Bindings.Add_Subscriber
       (Container => My_Container_Handle,
        Topic     => Interfaces.C.Strings.New_String ("intel/topic"),
         Type_Name => Interfaces.C.Strings.New_String (Content_Type),
        Callback  => My_Subscriber'Access,
        User_Data => System.Null_Address);
begin
   Pcl_Bindings.Port_Set_Route
     (Port       => Port,
      Route_Mode => Pcl_Bindings.PCL_ROUTE_REMOTE,
      Peer_Ids   => Peer_List,
      Peer_Count => 1);

   Interfaces.C.Strings.Free (Peer);
end;
```

Only remote ingress tagged with `"bridge_b"` will reach this subscriber.

### Bridge-Style Executor

For a chained topology:

- executor A <-> bridge executor B <-> executor C

the bridge behavior stays explicit in Ada too. A bridge container in executor B:

- subscribes or serves on one routed endpoint
- republishes or invokes on another routed endpoint

Typical layout:

- peer `"left"` registered on executor B
- peer `"right"` registered on executor B
- incoming subscriber or provided service routed remote from `"left"`
- outgoing publisher or consumed service routed remote to `"right"`

That means the forwarding policy lives in normal Ada business logic, not hidden
inside the transport layer.

### Practical Rules

- use `Port_Set_Route` for publishers, subscribers, and provided services
- use `Set_Endpoint_Route` for consumed unary services
- use exactly one peer for remote consumed services in v1
- use stable peer IDs and reuse them consistently between socket transport setup
  and route tables
- if no route is configured, concrete generated services are local by default
