--  Object-oriented Ada wrapper for PCL containers, executors, and ports.
--
--  ``Pcl_Component`` is the Ada equivalent of the C++ wrapper in
--  ``include/pcl/component.hpp`` and ``executor.hpp``.  It hides the manual
--  callback wiring and resource ownership of the C ABI behind:
--
--   * ``Component`` - tagged base type with overridable ``On_*`` hooks
--   * ``Executor``  - RAII wrapper around ``pcl_executor_t``
--   * ``Port``      - tagged value carrying a publish/subscribe/service handle
--   * ``Message_View`` - safe accessor for an inbound ``Pcl_Msg``
--
--  Most application code only touches this package; reach into
--  ``Pcl_Bindings`` only for low-level interop (transport adapters,
--  generated bindings, etc.).

with Ada.Containers.Indefinite_Vectors;
with Ada.Finalization;
with Ada.Strings.Unbounded;
with Interfaces.C;
with Interfaces.C.Strings;
with Pcl_Bindings;
with Pcl_Content_Types;
with System;

package Pcl_Component is
  --  Raised by wrapper subprograms for any underlying C-level failure that
  --  cannot be recovered from (allocation failure, invalid handle, etc.).
  Pcl_Error : exception;

  --  Convenience container for a logical peer-ID list (e.g. routing
  --  allow-lists).
  package Peer_Id_Vectors is new Ada.Containers.Indefinite_Vectors
    (Index_Type   => Positive,
     Element_Type => String);

  --  Read-only view of an inbound message handed to ``On_Message`` or
  --  ``Pcl_Typed_Ports.Decode``.  The underlying buffer lives only for the
  --  duration of the dispatching callback; copy the data out if it is
  --  needed later.
  type Message_View is private;

  function Data_Address(Message : Message_View) return System.Address;
  function Size_Bytes(Message : Message_View) return Interfaces.C.unsigned;
  function Type_Name(Message : Message_View) return String;
  function To_Raw_Message(Message : Message_View) return Pcl_Bindings.Pcl_Msg;

  --  Concrete publisher/subscriber/service port.  Construct via
  --  ``Component.Add_Publisher`` etc.; copy semantics intentionally allow
  --  components to keep a ``Port`` value as a record field.
  type Port is tagged private;

  --  True once the wrapped C handle has been set (i.e. ``Add_*`` succeeded).
  function Is_Valid(This : Port) return Boolean;

  --  Content-type string the port was created with; useful for codecs that
  --  pick a wire format from the port itself.
  function Default_Type_Name(This : Port) return String;

  --  Routing helpers that translate to ``pcl_port_set_route``.  See
  --  ``doc/guides/peer_transport_configuration.md`` for the full route rules.
  procedure Route_Local(This : in out Port);
  procedure Route_Remote(This : in out Port; Peer_Id : String);
  procedure Route_Remote(This : in out Port; Peer_Ids : Peer_Id_Vectors.Vector);
  procedure Route_Local_And_Remote(This : in out Port; Peer_Id : String);
  procedure Route_Local_And_Remote
    (This     : in out Port;
     Peer_Ids : Peer_Id_Vectors.Vector);

  --  Publish overloads.  All forms eventually call ``pcl_port_publish``;
  --  they only differ in how the payload and content-type are supplied.
  --  Raises ``Pcl_Error`` on a non-OK status from the C ABI.
  procedure Publish(This : Port; Msg : Pcl_Bindings.Pcl_Msg);
  procedure Publish
    (This      : Port;
     Data      : System.Address;
     Size      : Interfaces.C.unsigned;
     Type_Name : String := "");
  procedure Publish
    (This      : Port;
     Payload   : String;
     Type_Name : String := "");
  procedure Publish
    (This    : Port;
     Payload : String;
     Format  : Pcl_Content_Types.Content_Type_Kind);

  --  Base type for application components.  Derive from this, override the
  --  ``On_*`` hooks below, and pass instances to ``Executor.Add``.  The
  --  underlying ``pcl_container_t`` is owned and freed automatically via
  --  ``Limited_Controlled``.
  type Component is abstract new Ada.Finalization.Limited_Controlled with private;

  --  Allocate the underlying ``pcl_container_t`` and bind callback shims.
  --  Must be called once before any other ``Component`` operation.
  procedure Create(This : in out Component'Class; Name : String);

  --  Raw C handle for interop with ``Pcl_Bindings``.
  function Handle(This : Component'Class) return Pcl_Bindings.Pcl_Container_Access;

  function Name(This : Component'Class) return String;
  function State(This : Component'Class) return Pcl_Bindings.Pcl_State;

  --  Lifecycle drivers.  Each translates to the matching C function and
  --  raises ``Pcl_Error`` on failure.
  procedure Configure(This : in out Component'Class);
  procedure Activate(This : in out Component'Class);
  procedure Deactivate(This : in out Component'Class);
  procedure Cleanup(This : in out Component'Class);
  procedure Shutdown(This : in out Component'Class);

  --  Configure how often ``On_Tick`` is invoked.  ``Hz <= 0`` disables
  --  ticking entirely.
  procedure Set_Tick_Rate_Hz
    (This : in out Component'Class;
     Hz   : Interfaces.C.double);
  function Tick_Rate_Hz(This : Component'Class) return Interfaces.C.double;

  --  Strongly-typed parameter setters.  Parameters are stored on the
  --  underlying container and survive across lifecycle transitions.
  procedure Set_Param(This : in out Component'Class; Key : String; Value : String);
  procedure Set_Param
    (This  : in out Component'Class;
     Key   : String;
     Value : Interfaces.C.double);
  procedure Set_Param
    (This  : in out Component'Class;
     Key   : String;
     Value : Interfaces.C.long_long);
  procedure Set_Param
    (This  : in out Component'Class;
     Key   : String;
     Value : Boolean);

  --  Parameter readers; ``Default_Val`` is returned when the key is unset.
  function Param_Str
    (This        : Component'Class;
     Key         : String;
     Default_Val : String := "") return String;
  function Param_F64
    (This        : Component'Class;
     Key         : String;
     Default_Val : Interfaces.C.double := 0.0) return Interfaces.C.double;
  function Param_I64
    (This        : Component'Class;
     Key         : String;
     Default_Val : Interfaces.C.long_long := 0) return Interfaces.C.long_long;
  function Param_Bool
    (This        : Component'Class;
     Key         : String;
     Default_Val : Boolean := False) return Boolean;

  --  Port factories.  Call from ``On_Configure``; the returned ``Port`` is
  --  invalid (``Is_Valid = False``) on allocation failure.
  function Add_Publisher
    (This      : in out Component'Class;
     Topic     : String;
     Type_Name : String) return Port;
  function Add_Subscriber
    (This      : in out Component'Class;
     Topic     : String;
     Type_Name : String;
     Callback  : Pcl_Bindings.Pcl_Sub_Callback_Access;
     User_Data : System.Address := System.Null_Address) return Port;
  function Add_Service
    (This         : in out Component'Class;
     Service_Name : String;
     Type_Name    : String;
     Handler      : Pcl_Bindings.Pcl_Service_Handler_Access;
     User_Data    : System.Address := System.Null_Address) return Port;
  function Add_Stream_Service
    (This         : in out Component'Class;
     Service_Name : String;
     Type_Name    : String;
     Handler      : Pcl_Bindings.Pcl_Stream_Handler_Access;
     User_Data    : System.Address := System.Null_Address) return Port;

  --  Convenience shim that registers an internal subscriber routing inbound
  --  messages on ``Topic`` to the component's ``On_Message`` hook.  Use this
  --  when the component prefers the OO hook over a free-standing callback.
  procedure Subscribe
    (This      : in out Component'Class;
     Topic     : String;
     Type_Name : String);

  --  Lifecycle hooks.  Override the ones the component cares about; default
  --  implementations are no-ops.  All hooks run on the executor thread.

  procedure On_Configure(This : in out Component) is null;
  procedure On_Activate(This : in out Component) is null;
  procedure On_Deactivate(This : in out Component) is null;
  procedure On_Cleanup(This : in out Component) is null;
  procedure On_Shutdown(This : in out Component) is null;

  --  Periodic tick.  ``Dt_Seconds`` is the wall-clock interval since the
  --  previous tick; rate is controlled by ``Set_Tick_Rate_Hz``.
  procedure On_Tick
    (This       : in out Component;
     Dt_Seconds : Interfaces.C.double) is null;

  --  Generic message hook used by the ``Subscribe`` shim above.
  procedure On_Message
    (This    : in out Component;
     Topic   : String;
     Message : Message_View) is null;

  --  RAII wrapper around ``pcl_executor_t``.  An executor must outlive any
  --  components and transports added to it; ``Limited_Controlled`` enforces
  --  the destruction order.
  type Executor is new Ada.Finalization.Limited_Controlled with private;

  --  Allocate the underlying ``pcl_executor_t``.  Required before any other
  --  ``Executor`` operation.
  procedure Create(This : in out Executor);
  function Handle(This : Executor) return Pcl_Bindings.Pcl_Executor_Access;

  --  Add a component (or raw container, e.g. a transport gateway) to the
  --  executor.  The component must already be ``Configure``-d (and usually
  --  ``Activate``-d).
  procedure Add(This : in out Executor; Item : in out Component'Class);
  procedure Add(This : in out Executor; Container : Pcl_Bindings.Pcl_Container_Access);

  procedure Remove(This : in out Executor; Item : in out Component'Class);
  procedure Remove(This : in out Executor; Container : Pcl_Bindings.Pcl_Container_Access);

  --  Run the executor loop until shutdown.  ``Spin_Status`` returns the
  --  terminating status instead of raising on errors.
  procedure Spin(This : in out Executor);
  function Spin_Status(This : in out Executor) return Pcl_Bindings.Pcl_Status;

  --  Single-step the executor; useful when the application owns the loop.
  procedure Spin_Once
    (This       : in out Executor;
     Timeout_Ms : Interfaces.C.unsigned := 0);
  function Spin_Once_Status
    (This       : in out Executor;
     Timeout_Ms : Interfaces.C.unsigned := 0) return Pcl_Bindings.Pcl_Status;

  --  Drain pending work for up to ``Timeout_Ms`` and then stop.
  procedure Shutdown_Graceful
    (This       : in out Executor;
     Timeout_Ms : Interfaces.C.unsigned := 5_000);

  --  Ask ``Spin`` to exit at the next safe point.
  procedure Request_Shutdown(This : in out Executor);

  --  Install ``Transport`` as the default remote path.  Prefer
  --  ``Register_Transport`` for multi-peer setups; ``Set_Transport`` is the
  --  fallback used when an endpoint route names no peer.
  procedure Set_Transport
    (This      : in out Executor;
     Transport : Pcl_Bindings.Pcl_Transport_Const_Access);

  --  Bind ``Transport`` under the logical ``Peer_Id``.  Routes naming this
  --  peer dispatch through this transport; ingress from this peer is
  --  filtered against per-port allow-lists.
  procedure Register_Transport
    (This      : in out Executor;
     Peer_Id   : String;
     Transport : Pcl_Bindings.Pcl_Transport_Const_Access);

  --  Endpoint-level route helpers, primarily for ``consumed`` (client)
  --  endpoints that have no concrete local port object to attach a route to.
  --  ``Kind`` defaults to ``CONSUMED`` because that is the typical case.
  --  Consumed unary endpoints may not use ``Route_Local_And_Remote``.
  procedure Route_Local
    (This          : in out Executor;
     Endpoint_Name : String;
     Kind          : Pcl_Bindings.Pcl_Endpoint_Kind :=
       Pcl_Bindings.PCL_ENDPOINT_CONSUMED);
  procedure Route_Remote
    (This          : in out Executor;
     Endpoint_Name : String;
     Peer_Id       : String;
     Kind          : Pcl_Bindings.Pcl_Endpoint_Kind :=
       Pcl_Bindings.PCL_ENDPOINT_CONSUMED);
  procedure Route_Remote
    (This          : in out Executor;
     Endpoint_Name : String;
     Peer_Ids      : Peer_Id_Vectors.Vector;
     Kind          : Pcl_Bindings.Pcl_Endpoint_Kind :=
       Pcl_Bindings.PCL_ENDPOINT_CONSUMED);
  procedure Route_Local_And_Remote
    (This          : in out Executor;
     Endpoint_Name : String;
     Peer_Id       : String;
     Kind          : Pcl_Bindings.Pcl_Endpoint_Kind);
  procedure Route_Local_And_Remote
    (This          : in out Executor;
     Endpoint_Name : String;
     Peer_Ids      : Peer_Id_Vectors.Vector;
     Kind          : Pcl_Bindings.Pcl_Endpoint_Kind);

  --  Inject a topic message as if it had arrived locally.  Mostly used by
  --  Ada-implemented transports and by tests; production code should publish
  --  through a real ``Port``.
  procedure Post_Incoming
    (This      : in out Executor;
     Topic     : String;
     Type_Name : String;
     Data      : System.Address;
     Size      : Interfaces.C.unsigned);
  procedure Post_Incoming
    (This    : in out Executor;
     Topic   : String;
     Payload : String;
     Format  : Pcl_Content_Types.Content_Type_Kind);

  --  Inject a topic message attributed to ``Peer_Id``.  Subscribers with a
  --  remote allow-list including ``Peer_Id`` will see the message.
  procedure Post_Remote_Incoming
    (This      : in out Executor;
     Peer_Id   : String;
     Topic     : String;
     Type_Name : String;
     Data      : System.Address;
     Size      : Interfaces.C.unsigned);
  procedure Post_Remote_Incoming
    (This    : in out Executor;
     Peer_Id : String;
     Topic   : String;
     Payload : String;
     Format  : Pcl_Content_Types.Content_Type_Kind);

  --  Publish on the executor's default transport (or local fan-out only,
  --  depending on routing).  Component-owned ``Port`` objects are usually a
  --  better choice; these forms exist for ad-hoc one-shot publishes.
  procedure Publish
    (This      : in out Executor;
     Topic     : String;
     Type_Name : String;
     Data      : System.Address;
     Size      : Interfaces.C.unsigned);
  procedure Publish
    (This    : in out Executor;
     Topic   : String;
     Payload : String;
     Format  : Pcl_Content_Types.Content_Type_Kind);

private
  Max_Subscriptions : constant Positive := 16;

  type Subscription_Context is record
    In_Use        : Boolean := False;
    Owner_Address : System.Address := System.Null_Address;
    Topic         : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
    Type_Name     : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
  end record;

  type Subscription_Array is
    array (Positive range 1 .. Max_Subscriptions) of aliased Subscription_Context;

  type Message_View is record
    Data          : System.Address := System.Null_Address;
    Size          : Interfaces.C.unsigned := 0;
    Raw_Type_Name : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
  end record;

  type Port is tagged record
    Handle               : Pcl_Bindings.Pcl_Port_Access := null;
    Default_Port_Type_Name : Ada.Strings.Unbounded.Unbounded_String :=
      Ada.Strings.Unbounded.Null_Unbounded_String;
  end record;

  type Component is abstract new Ada.Finalization.Limited_Controlled with record
    Handle        : Pcl_Bindings.Pcl_Container_Access := null;
    Raw_Name      : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
    Callbacks     : aliased Pcl_Bindings.Pcl_Callbacks :=
      (On_Configure  => null,
       On_Activate   => null,
       On_Deactivate => null,
       On_Cleanup    => null,
       On_Shutdown   => null,
       On_Tick       => null);
    Subscriptions : Subscription_Array;
  end record;

  overriding procedure Finalize(This : in out Component);

  type Executor is new Ada.Finalization.Limited_Controlled with record
    Handle : Pcl_Bindings.Pcl_Executor_Access := null;
  end record;

  overriding procedure Finalize(This : in out Executor);
end Pcl_Component;
