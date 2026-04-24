# PCL User Guide

This guide is the quickest way to get productive with the PYRAMID Container
Library (PCL) from C, C++, or Ada.

It complements the lower-level routing guide in
[`peer_transport_configuration.md`](./peer_transport_configuration.md) and
focuses on day-to-day usage:

- creating components and executors
- publishing, subscribing, and serving requests
- selecting transports
- choosing wire formats
- using the C++ and Ada wrapper layers

## 1. Choose Your Layer

PCL exposes three supported entry points:

| Layer | Where | Best for |
|------|------|------|
| C ABI | `subprojects/PCL/include/pcl/*.h` | Runtime internals, transport implementations, bindings generation |
| C++ wrapper | `subprojects/PCL/include/pcl/component.hpp`, `executor.hpp` | Native C++ applications that want RAII and virtual lifecycle hooks |
| Ada wrapper | `subprojects/PCL/bindings/ada/` | Canonical Ada binding for PCL and generated PYRAMID Ada service bindings |

The Ada binding now lives inside `subprojects/PCL` so it can evolve alongside
the public C ABI and the C++ wrapper rather than drifting in downstream
examples.

## 2. Core Model

Every PCL application uses the same vocabulary:

- `Container` / `Component`
  user logic with lifecycle callbacks
- `Executor`
  single-threaded runtime that drives ticks, subscriber callbacks, and service handlers
- `Port`
  a concrete publisher, subscriber, service, or stream-service endpoint created during configure
- `Transport`
  the adapter that carries messages across process or network boundaries
- `Route`
  the local/remote exposure policy for a concrete port or consumed endpoint

The usual startup flow is:

1. create an executor
2. create a transport and attach or register it
3. create/configure/activate components
4. add components to the executor
5. spin

## 3. Wire Formats

PCL itself treats the payload as bytes plus a `type_name` / content-type string.
That keeps the runtime agnostic about whether the bytes are:

- JSON
- FlatBuffers
- Protobuf
- gRPC framing
- ROS 2 transport payloads

Common content-type names are exposed in Ada via
`Pcl_Content_Types`:

- `Json_Content_Type`
- `Flatbuffers_Content_Type`
- `Protobuf_Content_Type`
- `Grpc_Content_Type`
- `Ros2_Content_Type`

Generated PYRAMID bindings still accept strings, which means the simplest rule
is:

- use `Pcl_Content_Types` when choosing a format in Ada
- pass the same string into generated `Invoke_*`, `Subscribe_*`, or `Register_Services`

## 4. C++ Wrapper

The C++ wrapper gives you:

- RAII ownership for `pcl_container_t` and `pcl_executor_t`
- virtual lifecycle methods
- a lightweight `pcl::Port` helper for routing and publishing
- executor helpers for peer registration, endpoint routing, and remote ingress

### 4.1 Minimal Component

```cpp
#include <pcl/component.hpp>
#include <pcl/executor.hpp>

class TelemetryComponent : public pcl::Component {
public:
  TelemetryComponent() : pcl::Component("telemetry") {}

protected:
  pcl_status_t on_configure() override {
    pub_ = addPublisher("telemetry/state", "application/json");
    return pub_ ? PCL_OK : PCL_ERR_CALLBACK;
  }

  pcl_status_t on_tick(double) override {
    const std::string payload = R"({"ready":true})";
    return pub_.publish(payload);
  }

private:
  pcl::Port pub_;
};

int main() {
  pcl::Executor exec;
  TelemetryComponent component;

  component.configure();
  component.activate();
  exec.add(component);
  exec.spin();
}
```

### 4.2 Port Helpers

`pcl::Port` mirrors the concrete-port operations from the C ABI:

- `routeLocal()`
- `routeRemote("peer_a")`
- `routeLocalAndRemote("peer_a")`
- `setRoute(...)`
- `publish(msg)`
- `publish(payload, type_name)`

That makes common startup wiring much shorter:

```cpp
pub_ = addPublisher("intel/out", "application/flatbuffers");
pub_.routeLocalAndRemote("planner");
```

### 4.3 Executor Helpers

`pcl::Executor` now wraps the most common transport/routing operations too:

- `remove(component)`
- `registerTransport("planner", transport)`
- `postRemoteIncoming("sensor_head", "telemetry/raw", &msg)`
- `publish("topic", &msg)`
- `routeLocal(...)`
- `routeRemote(...)`
- `routeLocalAndRemote(...)`
- `setEndpointRoute(...)`

For consumed/client endpoints, the typical pattern is:

```cpp
exec.registerTransport("planner", planner_transport);
exec.routeRemote("planner.solve", "planner");
```

## 5. Ada Wrapper

The canonical Ada packages are:

- `Pcl_Bindings`
  thin binding to the public C ABI
- `Pcl_Component`
  tagged OO wrapper for components, executors, ports, params, and routing
- `Pcl_Transports`
  socket, UDP, and shared-memory transport helpers
- `Pcl_Content_Types`
  named content-type constants and enum helpers
- `Pcl_Typed_Ports`
  generic helper for typed encode/decode pairs

### 5.1 Add The Canonical Ada Source Directory

Use:

- `subprojects/PCL/bindings/ada`

The older copy in `subprojects/PYRAMID/examples/ada` has been removed on
purpose so there is only one authoritative `Pcl_Bindings` / `Pcl_Component`
package set in the workspace.

### 5.2 Minimal Ada Component

```ada
with Interfaces.C;
with Pcl_Component;

procedure Example is
   type Telemetry_Component is new Pcl_Component.Component with record
      Publisher : Pcl_Component.Port;
   end record;

   overriding procedure On_Configure (This : in out Telemetry_Component);
   overriding procedure On_Tick
     (This       : in out Telemetry_Component;
      Dt_Seconds : Interfaces.C.double);

   procedure On_Configure (This : in out Telemetry_Component) is
   begin
      This.Publisher :=
        Pcl_Component.Add_Publisher
          (This      => This,
           Topic     => "telemetry/state",
           Type_Name => "application/json");
      This.Publisher.Route_Local;
   end On_Configure;

   procedure On_Tick
     (This       : in out Telemetry_Component;
      Dt_Seconds : Interfaces.C.double) is
      pragma Unreferenced (Dt_Seconds);
   begin
      This.Publisher.Publish ("{""ready"":true}");
   end On_Tick;

   Exec : Pcl_Component.Executor;
   Comp : Telemetry_Component;
begin
   Pcl_Component.Create (Exec);
   Pcl_Component.Create (Comp, "telemetry");
   Pcl_Component.Configure (Comp);
   Pcl_Component.Activate (Comp);
   Pcl_Component.Add (Exec, Comp);
   Pcl_Component.Spin (Exec);
end Example;
```

### 5.3 Transport Helpers

The Ada transport wrapper is intentionally opinionated:

- `Create_*` creates the transport against an executor
- `Use_As_Default` attaches it as the executor default transport
- `Register` registers it under a peer ID
- `Start_Gateway` configures, activates, and adds the transport gateway container when needed

Socket client:

```ada
declare
   Exec      : Pcl_Component.Executor;
   Planner_Tx : Pcl_Transports.Socket_Transport;
begin
   Pcl_Component.Create (Exec);
   Pcl_Transports.Create_Client
     (This => Planner_Tx,
      Exec => Exec,
      Host => "127.0.0.1",
      Port => 7001);
   Pcl_Transports.Register (Planner_Tx, Exec, "planner");
end;
```

Shared memory:

```ada
declare
   Exec : Pcl_Component.Executor;
   Bus  : Pcl_Transports.Shared_Memory_Transport;
begin
   Pcl_Component.Create (Exec);
   Pcl_Transports.Create
     (This           => Bus,
      Exec           => Exec,
      Bus_Name       => "mission_bus",
      Participant_Id => "bridge");
   Pcl_Transports.Use_As_Default (Bus, Exec);
   Pcl_Transports.Start_Gateway (Bus, Exec);
end;
```

UDP:

```ada
declare
   Exec : Pcl_Component.Executor;
   Tx   : Pcl_Transports.Udp_Transport;
begin
   Pcl_Component.Create (Exec);
   Pcl_Transports.Create
     (This        => Tx,
      Exec        => Exec,
      Local_Port  => 9200,
      Remote_Host => "127.0.0.1",
      Remote_Port => 9100);
   Pcl_Transports.Register (Tx, Exec, "sensor_head");
end;
```

### 5.4 Typed Ports With Ada Generics

`Pcl_Typed_Ports` is for cases where you already have a codec pair and want to
stop repeating:

- content-type selection
- message construction
- decode boilerplate

```ada
with Pcl_Typed_Ports;

package Json_State_Port is new Pcl_Typed_Ports
  (Payload_Type   => State_Update,
   Encode         => Encode_State_Update,
   Decode_Payload => Decode_State_Update);
```

Then publish and decode through the strongly-typed helper:

```ada
Json_State_Port.Publish (Port => State_Pub, Payload => Update_Value);
Value := Json_State_Port.Decode (Message);
```

## 6. Routing Patterns

The wrapper layers do not change the routing model. They just make it easier to
express.

Concrete ports:

- publisher
- subscriber
- provided service
- stream service

Consumed/client endpoints:

- executor-level route, because there is no local concrete server port

Use:

- `Port.routeLocal()` / `Port.Route_Local`
- `Port.routeRemote(...)` / `Port.Route_Remote(...)`
- `Executor.routeRemote(...)` / `Executor.Route_Remote(...)`

For the full routing rules and validation constraints, see
[`peer_transport_configuration.md`](./peer_transport_configuration.md).

## 7. Generated PYRAMID Bindings

Generated `Provided` / `Consumed` packages layer on top of PCL rather than
replace it.

The practical split is:

- let generated code own service names, topic names, encoding, and decoding
- let PCL own lifecycle, executor, transport, and routing

That means a generated Ada service call typically looks like:

```ada
Provided_Tobj.Invoke_Create_Requirement
  (Executor     => Pcl_Component.Handle (Exec),
   Request      => Req,
   Callback     => On_Response'Access,
   Content_Type => Pcl_Content_Types.Json_Content_Type);
```

## 8. Examples To Copy From

Good workspace references:

- C++ wrapper tests:
  `subprojects/PCL/tests/test_pcl_cpp_wrappers.cpp`
- Ada wrapper demo:
  `subprojects/PYRAMID/examples/ada/pcl_sensor_demo.adb`
- Ada bridge using the canonical wrapper:
  `subprojects/PYRAMID/pyramid_bridge/ada/pyramid_bridge_main.adb`

## 9. Building Ada Consumers

GNAT needs GCC-compatible static archives, not MSVC `.lib` files.

Build them from the workspace root with:

```bat
subprojects\PCL\scripts\build_gnat_pcl_static_libs.bat
```

That now produces:

- `libpcl_core.a`
- `libpcl_transport_socket.a`
- `libpcl_transport_udp.a`
- `libpcl_transport_shared_memory.a`

If your Ada project uses `Pcl_Transports`, link the transport libraries that
match the transports you intend to instantiate.

## 10. Rules Of Thumb

- keep lifecycle logic in the component, not in transport setup code
- use generated service bindings for payload encode/decode
- use PCL wrappers for executor, port, and transport wiring
- pick a stable peer ID once and reuse it everywhere
- treat wire format as configuration, not as a hard-coded transport detail
