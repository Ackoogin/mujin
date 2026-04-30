--  Thin Ada binding to the public PCL C ABI declared in
--  ``subprojects/PCL/include/pcl/*.h``.
--
--  Each declaration here mirrors a single C entity with no policy applied.
--  Higher-level Ada wrappers live in:
--
--   * ``Pcl_Component``   - OO components, executors, ports, params, routing
--   * ``Pcl_Transports``  - socket/UDP/shared-memory transport helpers
--   * ``Pcl_Typed_Ports`` - typed encode/decode generic
--
--  Use this package directly only when the higher-level wrappers do not yet
--  cover a feature, or when generated PYRAMID bindings need raw access to the
--  C handles and callback typedefs.

with Interfaces.C;
with Interfaces.C.Extensions;
with Interfaces.C.Strings;
with System;

package Pcl_Bindings is
  --  C ``bool`` exposed at the ABI boundary for routing callbacks etc.
  subtype C_Bool is Interfaces.C.Extensions.bool;

  --  Numeric status code matching ``pcl_status_t`` in the C ABI.
  --
  --  Non-negative codes are normal outcomes (``OK``, ``PENDING``,
  --  ``STREAMING``); negative codes are failures.  All PCL entry points use
  --  this single status type for uniform error handling.
  type Pcl_Status is new Interfaces.C.int;

  PCL_OK              : constant Pcl_Status := 0;
  --  Service handler will respond later via ``Service_Respond``;
  --  callers must keep the request payload alive until that point.
  PCL_PENDING         : constant Pcl_Status := 1;
  --  Streaming service handler accepted the request and will emit zero or
  --  more responses through the stream context.
  PCL_STREAMING       : constant Pcl_Status := 2;
  PCL_ERR_INVALID     : constant Pcl_Status := -1;
  PCL_ERR_STATE       : constant Pcl_Status := -2;
  PCL_ERR_TIMEOUT     : constant Pcl_Status := -3;
  PCL_ERR_CALLBACK    : constant Pcl_Status := -4;
  PCL_ERR_NOMEM       : constant Pcl_Status := -5;
  PCL_ERR_NOT_FOUND   : constant Pcl_Status := -6;
  PCL_ERR_PORT_CLOSED : constant Pcl_Status := -7;
  PCL_ERR_CANCELLED   : constant Pcl_Status := -8;

  --  Container lifecycle phases.  Transitions are linear:
  --  ``Unconfigured`` -> ``Configured`` -> ``Active`` -> ... -> ``Finalized``.
  type Pcl_State is
    (PCL_STATE_UNCONFIGURED,
     PCL_STATE_CONFIGURED,
     PCL_STATE_ACTIVE,
     PCL_STATE_FINALIZED);
  pragma Convention(C, Pcl_State);

  --  Discriminator for the kinds of port a container can register.
  type Pcl_Port_Type is
    (PCL_PORT_PUBLISHER,
     PCL_PORT_SUBSCRIBER,
     PCL_PORT_SERVICE,
     PCL_PORT_CLIENT,
     PCL_PORT_STREAM_SERVICE);
  pragma Convention(C, Pcl_Port_Type);

  --  Routing bit-mask values applied per concrete port or per consumed
  --  endpoint.  A port may combine ``LOCAL`` and ``REMOTE`` for fan-out;
  --  consumed unary endpoints must pick exactly one.  See
  --  ``doc/guides/peer_transport_configuration.md`` for the full rule set.
  PCL_ROUTE_NONE   : constant Interfaces.C.unsigned := 0;
  PCL_ROUTE_LOCAL  : constant Interfaces.C.unsigned := 1;
  PCL_ROUTE_REMOTE : constant Interfaces.C.unsigned := 2;

  --  Hard upper bound on the peer allow-list length per route.
  PCL_MAX_ENDPOINT_PEERS : constant Interfaces.C.unsigned := 8;

  --  Endpoint role used in ``Pcl_Endpoint_Route``.  ``Provided`` /
  --  ``Consumed`` describe interface direction (server vs. client) and are
  --  independent of locality.
  type Pcl_Endpoint_Kind is
    (PCL_ENDPOINT_PUBLISHER,
     PCL_ENDPOINT_SUBSCRIBER,
     PCL_ENDPOINT_PROVIDED,
     PCL_ENDPOINT_CONSUMED,
     PCL_ENDPOINT_STREAM_PROVIDED);
  pragma Convention(C, Pcl_Endpoint_Kind);

  type Pcl_Log_Level is
    (PCL_LOG_DEBUG,
     PCL_LOG_INFO,
     PCL_LOG_WARN,
     PCL_LOG_ERROR,
     PCL_LOG_FATAL);
  pragma Convention(C, Pcl_Log_Level);

  --  Connection state of a TCP socket transport instance.
  type Pcl_Socket_State is
    (PCL_SOCKET_STATE_CONNECTING,
     PCL_SOCKET_STATE_CONNECTED,
     PCL_SOCKET_STATE_DISCONNECTED);
  pragma Convention(C, Pcl_Socket_State);

  --  Opaque handle to ``pcl_executor_t``.  The executor is the single-thread
  --  runtime that drives ticks, subscriber callbacks, and service handlers.
  type Pcl_Executor is limited private;
  type Pcl_Executor_Access is access all Pcl_Executor;
  pragma Convention(C, Pcl_Executor_Access);

  --  Opaque handle to ``pcl_container_t``: a lifecycle-managed component
  --  with its own port set and parameter store.
  type Pcl_Container is limited private;
  type Pcl_Container_Access is access all Pcl_Container;
  pragma Convention(C, Pcl_Container_Access);

  --  Opaque handle to a concrete publisher/subscriber/service port created
  --  inside a container.
  type Pcl_Port is limited private;
  type Pcl_Port_Access is access all Pcl_Port;
  pragma Convention(C, Pcl_Port_Access);

  --  Service-call response ticket given to a service handler that returns
  --  ``PCL_PENDING``.  The handler must invoke ``Service_Respond`` (or free
  --  the context) before the ticket goes out of scope.
  type Pcl_Svc_Context is limited private;
  type Pcl_Svc_Context_Access is access all Pcl_Svc_Context;
  pragma Convention(C, Pcl_Svc_Context_Access);

  --  Per-stream context handed to streaming service handlers; carries the
  --  state needed by ``Stream_Send`` / ``Stream_End`` / ``Stream_Cancel``.
  type Pcl_Stream_Context is limited private;
  type Pcl_Stream_Context_Access is access all Pcl_Stream_Context;
  pragma Convention(C, Pcl_Stream_Context_Access);

  --  Generic transport vtable + adapter context expected by the executor.
  --  Transport adapters fill in the ``Pcl_Transport_Record`` fields they
  --  support; unset slots cause the executor to return ``ERR_NOT_FOUND``.
  type Pcl_Transport is limited private;
  type Pcl_Transport_Access is access all Pcl_Transport;
  pragma Convention(C, Pcl_Transport_Access);

  type Pcl_Transport_Const_Access is access constant Pcl_Transport;
  pragma Convention(C, Pcl_Transport_Const_Access);

  --  Opaque handle to a TCP socket transport instance.
  type Pcl_Socket_Transport is limited private;
  type Pcl_Socket_Transport_Access is access all Pcl_Socket_Transport;
  pragma Convention(C, Pcl_Socket_Transport_Access);

  --  Opaque handle to a UDP datagram transport instance.
  type Pcl_Udp_Transport is limited private;
  type Pcl_Udp_Transport_Access is access all Pcl_Udp_Transport;
  pragma Convention(C, Pcl_Udp_Transport_Access);

  --  Opaque handle to a shared-memory bus participant.  Each instance owns
  --  one mailbox slot inside an OS-named shared region (see
  --  ``Create_Shared_Memory_Transport``).
  type Pcl_Shared_Memory_Transport is limited private;
  type Pcl_Shared_Memory_Transport_Access is access all Pcl_Shared_Memory_Transport;
  pragma Convention(C, Pcl_Shared_Memory_Transport_Access);

  --  Plain byte-buffer message exchanged over the C ABI.  PCL never
  --  inspects ``Data``; ``Type_Name`` is a free-form content-type string
  --  (see ``Pcl_Content_Types``).
  type Pcl_Msg is record
    Data      : System.Address;
    Size      : Interfaces.C.unsigned;
    Type_Name : Interfaces.C.Strings.chars_ptr;
  end record;
  pragma Convention(C, Pcl_Msg);

  --  Route descriptor passed to ``Set_Endpoint_Route``.  ``Peer_Ids`` is a
  --  C array of ``char *`` of length ``Peer_Count`` and is only consulted
  --  when ``Route_Mode`` includes ``PCL_ROUTE_REMOTE``.
  type Pcl_Endpoint_Route is record
    Endpoint_Name : Interfaces.C.Strings.chars_ptr;
    Endpoint_Kind : Pcl_Endpoint_Kind;
    Route_Mode    : Interfaces.C.unsigned;
    Peer_Ids      : System.Address;
    Peer_Count    : Interfaces.C.unsigned;
  end record;
  pragma Convention(C, Pcl_Endpoint_Route);

  --  Lifecycle callback typedefs.  All run on the executor thread; returning
  --  a non-OK status aborts the lifecycle transition and leaves the
  --  container in its previous state.
  type On_Configure_Access is access function
    (Self      : Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Status;
  pragma Convention(C, On_Configure_Access);

  type On_Activate_Access is access function
    (Self      : Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Status;
  pragma Convention(C, On_Activate_Access);

  type On_Deactivate_Access is access function
    (Self      : Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Status;
  pragma Convention(C, On_Deactivate_Access);

  type On_Cleanup_Access is access function
    (Self      : Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Status;
  pragma Convention(C, On_Cleanup_Access);

  type On_Shutdown_Access is access function
    (Self      : Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Status;
  pragma Convention(C, On_Shutdown_Access);

  --  Periodic tick callback.  ``Dt_Seconds`` is the wall-clock interval since
  --  the previous tick (subject to ``Set_Tick_Rate_Hz``).
  type On_Tick_Access is access function
    (Self       : Pcl_Container_Access;
     Dt_Seconds : Interfaces.C.double;
     User_Data  : System.Address) return Pcl_Status;
  pragma Convention(C, On_Tick_Access);

  --  Bundle of lifecycle callbacks supplied to ``Create_Container``.  Any
  --  field may be left ``null`` if the component does not need that hook.
  type Pcl_Callbacks is record
    On_Configure  : On_Configure_Access := null;
    On_Activate   : On_Activate_Access := null;
    On_Deactivate : On_Deactivate_Access := null;
    On_Cleanup    : On_Cleanup_Access := null;
    On_Shutdown   : On_Shutdown_Access := null;
    On_Tick       : On_Tick_Access := null;
  end record;
  pragma Convention(C, Pcl_Callbacks);

  --  Subscriber callback fired on the executor thread when a matching message
  --  is delivered (locally or via a transport).
  type Pcl_Sub_Callback_Access is access procedure
    (Self      : Pcl_Container_Access;
     Msg       : access constant Pcl_Msg;
     User_Data : System.Address);
  pragma Convention(C, Pcl_Sub_Callback_Access);

  --  Unary service handler.  Return ``PCL_OK`` after filling ``Response``
  --  for synchronous replies, or ``PCL_PENDING`` to defer; in the latter
  --  case the handler must ultimately call ``Service_Respond`` against the
  --  ``Ctx`` ticket.
  type Pcl_Service_Handler_Access is access function
    (Self      : Pcl_Container_Access;
     Request   : access constant Pcl_Msg;
     Response  : access Pcl_Msg;
     Ctx       : Pcl_Svc_Context_Access;
     User_Data : System.Address) return Pcl_Status;
  pragma Convention(C, Pcl_Service_Handler_Access);

  --  Streaming service handler.  Returns ``PCL_STREAMING`` and emits replies
  --  through ``Stream_Send`` / ``Stream_End`` against ``Stream_Ctx``.
  type Pcl_Stream_Handler_Access is access function
    (Self       : Pcl_Container_Access;
     Request    : access constant Pcl_Msg;
     Stream_Ctx : Pcl_Stream_Context_Access;
     User_Data  : System.Address) return Pcl_Status;
  pragma Convention(C, Pcl_Stream_Handler_Access);

  --  Response callback fired by the executor when an async unary call
  --  completes.  ``Resp`` is non-null on success and may be a zero-byte
  --  message on transport-level errors (see remote-not-found semantics).
  type Pcl_Resp_Cb_Access is access procedure
    (Resp      : access constant Pcl_Msg;
     User_Data : System.Address);
  pragma Convention(C, Pcl_Resp_Cb_Access);

  --  Streaming response callback.  ``End_Of_Stream`` is true on the final
  --  invocation; ``Status`` carries the terminal status code in that case.
  type Pcl_Stream_Msg_Cb_Access is access procedure
    (Msg           : access constant Pcl_Msg;
     End_Of_Stream : C_Bool;
     Status        : Pcl_Status;
     User_Data     : System.Address);
  pragma Convention(C, Pcl_Stream_Msg_Cb_Access);

  --  Socket-transport state-change callback.  May fire from arbitrary
  --  threads (initial connect on the creating thread, reconnects on the
  --  receive thread); implementations must be thread-safe.
  type Pcl_Socket_State_Cb_Access is access procedure
    (State     : Pcl_Socket_State;
     User_Data : System.Address);
  pragma Convention(C, Pcl_Socket_State_Cb_Access);

  --  Vtable entries implemented by transport adapters.  ``Adapter_Ctx`` is
  --  the opaque pointer registered in ``Pcl_Transport_Record``; the executor
  --  invokes only the slots a given transport provides.

  --  Send a published message on ``Topic``.  Implementations are expected to
  --  copy ``Msg`` if they need to outlive the call.
  type Transport_Publish_Access is access function
    (Adapter_Ctx : System.Address;
     Topic       : Interfaces.C.Strings.chars_ptr;
     Msg         : access constant Pcl_Msg) return Pcl_Status;
  pragma Convention(C, Transport_Publish_Access);

  type Transport_Serve_Access is access function
    (Adapter_Ctx  : System.Address;
     Service_Name : Interfaces.C.Strings.chars_ptr;
     Request      : access constant Pcl_Msg;
     Response     : access Pcl_Msg) return Pcl_Status;
  pragma Convention(C, Transport_Serve_Access);

  type Transport_Subscribe_Access is access function
    (Adapter_Ctx : System.Address;
     Topic       : Interfaces.C.Strings.chars_ptr;
     Type_Name   : Interfaces.C.Strings.chars_ptr) return Pcl_Status;
  pragma Convention(C, Transport_Subscribe_Access);

  type Transport_Invoke_Async_Access is access function
    (Adapter_Ctx  : System.Address;
     Service_Name : Interfaces.C.Strings.chars_ptr;
     Request      : access constant Pcl_Msg;
     Callback     : Pcl_Resp_Cb_Access;
     User_Data    : System.Address) return Pcl_Status;
  pragma Convention(C, Transport_Invoke_Async_Access);

  type Transport_Respond_Access is access function
    (Adapter_Ctx : System.Address;
     Svc_Ctx     : Pcl_Svc_Context_Access;
     Response    : access constant Pcl_Msg) return Pcl_Status;
  pragma Convention(C, Transport_Respond_Access);

  type Transport_Shutdown_Access is access procedure
    (Adapter_Ctx : System.Address);
  pragma Convention(C, Transport_Shutdown_Access);

  type Transport_Invoke_Stream_Access is access function
    (Adapter_Ctx  : System.Address;
     Service_Name : Interfaces.C.Strings.chars_ptr;
     Request      : access constant Pcl_Msg;
     Callback     : Pcl_Stream_Msg_Cb_Access;
     User_Data    : System.Address;
     Stream_Handle : access System.Address) return Pcl_Status;
  pragma Convention(C, Transport_Invoke_Stream_Access);

  type Transport_Stream_Send_Access is access function
    (Adapter_Ctx   : System.Address;
     Stream_Handle : System.Address;
     Msg           : access constant Pcl_Msg) return Pcl_Status;
  pragma Convention(C, Transport_Stream_Send_Access);

  type Transport_Stream_End_Access is access function
    (Adapter_Ctx   : System.Address;
     Stream_Handle : System.Address;
     Status        : Pcl_Status) return Pcl_Status;
  pragma Convention(C, Transport_Stream_End_Access);

  type Transport_Stream_Cancel_Access is access function
    (Adapter_Ctx   : System.Address;
     Stream_Handle : System.Address) return Pcl_Status;
  pragma Convention(C, Transport_Stream_Cancel_Access);

  --  Vtable presented by a transport adapter.  Slots left ``null`` cause the
  --  executor to return ``ERR_NOT_FOUND`` when an operation needs them
  --  (e.g. UDP leaves ``Invoke_Async`` / ``Respond`` null on purpose so it
  --  cannot accidentally carry RPC traffic).
  type Pcl_Transport_Record is record
    Publish       : Transport_Publish_Access := null;
    Serve         : Transport_Serve_Access := null;
    Subscribe     : Transport_Subscribe_Access := null;
    Invoke_Async  : Transport_Invoke_Async_Access := null;
    Respond       : Transport_Respond_Access := null;
    Shutdown      : Transport_Shutdown_Access := null;
    Invoke_Stream : Transport_Invoke_Stream_Access := null;
    Stream_Send   : Transport_Stream_Send_Access := null;
    Stream_End    : Transport_Stream_End_Access := null;
    Stream_Cancel : Transport_Stream_Cancel_Access := null;
    Adapter_Ctx   : System.Address := System.Null_Address;
  end record;
  pragma Convention(C, Pcl_Transport_Record);

  --  Optional knobs for ``Create_Socket_Client_Ex``.  All-zero gives
  --  legacy single-shot connect behaviour; populate the fields to enable
  --  bounded retry, auto-reconnect, and state-change callbacks.  See
  --  ``doc/guides/peer_transport_configuration.md`` section 8.1.
  type Pcl_Socket_Client_Opts is record
    Connect_Timeout_Ms : Interfaces.C.unsigned := 0;
    Max_Retries        : Interfaces.C.unsigned := 0;
    Auto_Reconnect     : Interfaces.C.int := 0;
    State_Cb           : Pcl_Socket_State_Cb_Access := null;
    State_Cb_Data      : System.Address := System.Null_Address;
  end record;
  pragma Convention(C, Pcl_Socket_Client_Opts);

  ---------------------------------------------------------------------------
  --  Executor lifecycle
  ---------------------------------------------------------------------------

  --  Allocate a fresh executor.  Returns ``null`` on allocation failure.
  function Create_Executor return Pcl_Executor_Access;
  pragma Import(C, Create_Executor, "pcl_executor_create");

  --  Tear down an executor and any registered transports/containers it owns.
  procedure Destroy_Executor(Exec : Pcl_Executor_Access);
  pragma Import(C, Destroy_Executor, "pcl_executor_destroy");

  function Add_Container
    (Exec      : Pcl_Executor_Access;
     Container : Pcl_Container_Access) return Pcl_Status;
  pragma Import(C, Add_Container, "pcl_executor_add");

  function Remove_Container
    (Exec      : Pcl_Executor_Access;
     Container : Pcl_Container_Access) return Pcl_Status;
  pragma Import(C, Remove_Container, "pcl_executor_remove");

  --  Run the executor loop until ``Request_Shutdown`` is called or all
  --  containers report shutdown.
  function Spin(Exec : Pcl_Executor_Access) return Pcl_Status;
  pragma Import(C, Spin, "pcl_executor_spin");

  --  Process queued work for at most ``Timeout_Ms`` milliseconds.  Use this
  --  instead of ``Spin`` when the application owns the outer loop.
  function Spin_Once
    (Exec       : Pcl_Executor_Access;
     Timeout_Ms : Interfaces.C.unsigned) return Pcl_Status;
  pragma Import(C, Spin_Once, "pcl_executor_spin_once");

  --  Ask the executor to break out of ``Spin`` at the next safe point.
  procedure Request_Shutdown(Exec : Pcl_Executor_Access);
  pragma Import(C, Request_Shutdown, "pcl_executor_request_shutdown");

  --  Drain pending work for up to ``Timeout_Ms``, then stop.
  function Shutdown_Graceful
    (Exec       : Pcl_Executor_Access;
     Timeout_Ms : Interfaces.C.unsigned) return Pcl_Status;
  pragma Import(C, Shutdown_Graceful, "pcl_executor_shutdown_graceful");

  ---------------------------------------------------------------------------
  --  External ingress
  --
  --  Transports call these to inject inbound traffic.  Application code
  --  rarely uses them directly; they exist on the binding for transports
  --  written in Ada and for tests that simulate remote traffic.
  ---------------------------------------------------------------------------

  --  Inject a locally-sourced topic message as if it were published inside
  --  the executor.
  function Post_Incoming
    (Exec  : Pcl_Executor_Access;
     Topic : Interfaces.C.Strings.chars_ptr;
     Msg   : access constant Pcl_Msg) return Pcl_Status;
  pragma Import(C, Post_Incoming, "pcl_executor_post_incoming");

  --  Inject a topic message attributed to ``Peer_Id``.  Used by transports
  --  to feed remote subscribers; per-port allow-lists then decide whether
  --  the message is dispatched.
  function Post_Remote_Incoming
    (Exec    : Pcl_Executor_Access;
     Peer_Id : Interfaces.C.Strings.chars_ptr;
     Topic   : Interfaces.C.Strings.chars_ptr;
     Msg     : access constant Pcl_Msg) return Pcl_Status;
  pragma Import(C, Post_Remote_Incoming, "pcl_executor_post_remote_incoming");

  function Post_Service_Request
    (Exec         : Pcl_Executor_Access;
     Service_Name : Interfaces.C.Strings.chars_ptr;
     Request      : access constant Pcl_Msg;
     Callback     : Pcl_Resp_Cb_Access;
     User_Data    : System.Address) return Pcl_Status;
  pragma Import(C, Post_Service_Request, "pcl_executor_post_service_request");

  function Post_Response_Cb
    (Exec      : Pcl_Executor_Access;
     Callback  : Pcl_Resp_Cb_Access;
     User_Data : System.Address;
     Data      : System.Address;
     Size      : Interfaces.C.unsigned) return Pcl_Status;
  pragma Import(C, Post_Response_Cb, "pcl_executor_post_response_cb");

  function Post_Response_Msg
    (Exec      : Pcl_Executor_Access;
     Callback  : Pcl_Resp_Cb_Access;
     User_Data : System.Address;
     Msg       : access constant Pcl_Msg) return Pcl_Status;
  pragma Import(C, Post_Response_Msg, "pcl_executor_post_response_msg");

  --  Synchronous local service invocation; resolves the service via the
  --  executor's port table.
  function Invoke_Service
    (Exec         : Pcl_Executor_Access;
     Service_Name : Interfaces.C.Strings.chars_ptr;
     Request      : access constant Pcl_Msg;
     Response     : access Pcl_Msg) return Pcl_Status;
  pragma Import(C, Invoke_Service, "pcl_executor_invoke_service");

  --  Synchronous remote service invocation through the transport registered
  --  for ``Peer_Id``.
  function Invoke_Service_Remote
    (Exec         : Pcl_Executor_Access;
     Peer_Id      : Interfaces.C.Strings.chars_ptr;
     Service_Name : Interfaces.C.Strings.chars_ptr;
     Request      : access constant Pcl_Msg;
     Response     : access Pcl_Msg) return Pcl_Status;
  pragma Import(C, Invoke_Service_Remote, "pcl_executor_invoke_service_remote");

  --  Publish on ``Topic`` using the executor's default transport (or local
  --  fan-out only, depending on the active route).
  function Publish
    (Exec  : Pcl_Executor_Access;
     Topic : Interfaces.C.Strings.chars_ptr;
     Msg   : access constant Pcl_Msg) return Pcl_Status;
  pragma Import(C, Publish, "pcl_executor_publish");

  ---------------------------------------------------------------------------
  --  Container lifecycle and ports
  ---------------------------------------------------------------------------

  --  Allocate a container with the given lifecycle callbacks.  ``Name`` is
  --  copied; ``Callbacks`` and ``User_Data`` are retained until destruction.
  function Create_Container
    (Name      : Interfaces.C.Strings.chars_ptr;
     Callbacks : access constant Pcl_Callbacks;
     User_Data : System.Address) return Pcl_Container_Access;
  pragma Import(C, Create_Container, "pcl_container_create");

  procedure Destroy_Container(Container : Pcl_Container_Access);
  pragma Import(C, Destroy_Container, "pcl_container_destroy");

  --  Drive container lifecycle transitions.  Each call invokes the matching
  --  ``On_*`` callback and updates ``Container_State``.
  function Configure(Container : Pcl_Container_Access) return Pcl_Status;
  pragma Import(C, Configure, "pcl_container_configure");

  function Activate(Container : Pcl_Container_Access) return Pcl_Status;
  pragma Import(C, Activate, "pcl_container_activate");

  function Deactivate(Container : Pcl_Container_Access) return Pcl_Status;
  pragma Import(C, Deactivate, "pcl_container_deactivate");

  function Cleanup(Container : Pcl_Container_Access) return Pcl_Status;
  pragma Import(C, Cleanup, "pcl_container_cleanup");

  function Shutdown(Container : Pcl_Container_Access) return Pcl_Status;
  pragma Import(C, Shutdown, "pcl_container_shutdown");

  function Container_State(Container : Pcl_Container_Access) return Pcl_State;
  pragma Import(C, Container_State, "pcl_container_state");

  function Container_Name
    (Container : Pcl_Container_Access) return Interfaces.C.Strings.chars_ptr;
  pragma Import(C, Container_Name, "pcl_container_name");

  function Set_Tick_Rate_Hz
    (Container : Pcl_Container_Access;
     Hz        : Interfaces.C.double) return Pcl_Status;
  pragma Import(C, Set_Tick_Rate_Hz, "pcl_container_set_tick_rate_hz");

  function Get_Tick_Rate_Hz
    (Container : Pcl_Container_Access) return Interfaces.C.double;
  pragma Import(C, Get_Tick_Rate_Hz, "pcl_container_get_tick_rate_hz");

  function Set_Param_Str
    (Container : Pcl_Container_Access;
     Key       : Interfaces.C.Strings.chars_ptr;
     Value     : Interfaces.C.Strings.chars_ptr) return Pcl_Status;
  pragma Import(C, Set_Param_Str, "pcl_container_set_param_str");

  function Set_Param_F64
    (Container : Pcl_Container_Access;
     Key       : Interfaces.C.Strings.chars_ptr;
     Value     : Interfaces.C.double) return Pcl_Status;
  pragma Import(C, Set_Param_F64, "pcl_container_set_param_f64");

  function Set_Param_Bool
    (Container : Pcl_Container_Access;
     Key       : Interfaces.C.Strings.chars_ptr;
     Value     : C_Bool) return Pcl_Status;
  pragma Import(C, Set_Param_Bool, "pcl_container_set_param_bool");

  function Get_Param_Str
    (Container   : Pcl_Container_Access;
     Key         : Interfaces.C.Strings.chars_ptr;
     Default_Val : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr;
  pragma Import(C, Get_Param_Str, "pcl_container_get_param_str");

  function Get_Param_F64
    (Container   : Pcl_Container_Access;
     Key         : Interfaces.C.Strings.chars_ptr;
     Default_Val : Interfaces.C.double) return Interfaces.C.double;
  pragma Import(C, Get_Param_F64, "pcl_container_get_param_f64");

  function Get_Param_Bool
    (Container   : Pcl_Container_Access;
     Key         : Interfaces.C.Strings.chars_ptr;
     Default_Val : C_Bool) return C_Bool;
  pragma Import(C, Get_Param_Bool, "pcl_container_get_param_bool");

  --  Concrete-port creators.  Call these from inside ``On_Configure``; the
  --  returned handle is owned by the container and freed during cleanup.
  function Add_Publisher
    (Container : Pcl_Container_Access;
     Topic     : Interfaces.C.Strings.chars_ptr;
     Type_Name : Interfaces.C.Strings.chars_ptr) return Pcl_Port_Access;
  pragma Import(C, Add_Publisher, "pcl_container_add_publisher");

  function Add_Subscriber
    (Container : Pcl_Container_Access;
     Topic     : Interfaces.C.Strings.chars_ptr;
     Type_Name : Interfaces.C.Strings.chars_ptr;
     Callback  : Pcl_Sub_Callback_Access;
     User_Data : System.Address) return Pcl_Port_Access;
  pragma Import(C, Add_Subscriber, "pcl_container_add_subscriber");

  function Add_Service
    (Container    : Pcl_Container_Access;
     Service_Name : Interfaces.C.Strings.chars_ptr;
     Type_Name    : Interfaces.C.Strings.chars_ptr;
     Handler      : Pcl_Service_Handler_Access;
     User_Data    : System.Address) return Pcl_Port_Access;
  pragma Import(C, Add_Service, "pcl_container_add_service");

  function Add_Stream_Service
    (Container    : Pcl_Container_Access;
     Service_Name : Interfaces.C.Strings.chars_ptr;
     Type_Name    : Interfaces.C.Strings.chars_ptr;
     Handler      : Pcl_Stream_Handler_Access;
     User_Data    : System.Address) return Pcl_Port_Access;
  pragma Import(C, Add_Stream_Service, "pcl_container_add_stream_service");

  --  Apply a route to a concrete port.  ``Peer_Ids`` is required for remote
  --  routes and must be ``System.Null_Address`` with ``Peer_Count = 0`` for
  --  local-only routes.
  function Port_Set_Route
    (Port       : Pcl_Port_Access;
     Route_Mode : Interfaces.C.unsigned;
     Peer_Ids   : System.Address;
     Peer_Count : Interfaces.C.unsigned) return Pcl_Status;
  pragma Import(C, Port_Set_Route, "pcl_port_set_route");

  --  Publish through a publisher port.  Honours the route configured by
  --  ``Port_Set_Route`` (local fan-out, remote fan-out, or both).
  function Port_Publish
    (Port : Pcl_Port_Access;
     Msg  : access constant Pcl_Msg) return Pcl_Status;
  pragma Import(C, Port_Publish, "pcl_port_publish");

  function Container_Invoke_Async
    (Container    : Pcl_Container_Access;
     Service_Name : Interfaces.C.Strings.chars_ptr;
     Request      : access constant Pcl_Msg;
     Callback     : Pcl_Resp_Cb_Access;
     User_Data    : System.Address) return Pcl_Status;
  pragma Import(C, Container_Invoke_Async, "pcl_container_invoke_async");

  ---------------------------------------------------------------------------
  --  Deferred service responses and streaming
  ---------------------------------------------------------------------------

  --  Send the deferred reply for a handler that returned ``PCL_PENDING``.
  function Service_Respond
    (Ctx      : Pcl_Svc_Context_Access;
     Response : access constant Pcl_Msg) return Pcl_Status;
  pragma Import(C, Service_Respond, "pcl_service_respond");

  --  Discard a deferred response ticket without replying.  Use only when
  --  the request is being abandoned without a reply.
  procedure Service_Context_Free(Ctx : Pcl_Svc_Context_Access);
  pragma Import(C, Service_Context_Free, "pcl_service_context_free");

  function Stream_Send
    (Ctx : Pcl_Stream_Context_Access;
     Msg : access constant Pcl_Msg) return Pcl_Status;
  pragma Import(C, Stream_Send, "pcl_stream_send");

  function Stream_End(Ctx : Pcl_Stream_Context_Access) return Pcl_Status;
  pragma Import(C, Stream_End, "pcl_stream_end");

  function Stream_Abort
    (Ctx        : Pcl_Stream_Context_Access;
     Error_Code : Pcl_Status) return Pcl_Status;
  pragma Import(C, Stream_Abort, "pcl_stream_abort");

  function Stream_Is_Cancelled
    (Ctx : Pcl_Stream_Context_Access) return C_Bool;
  pragma Import(C, Stream_Is_Cancelled, "pcl_stream_is_cancelled");

  function Stream_Cancel
    (Ctx : Pcl_Stream_Context_Access) return Pcl_Status;
  pragma Import(C, Stream_Cancel, "pcl_stream_cancel");

  ---------------------------------------------------------------------------
  --  Transport binding
  ---------------------------------------------------------------------------

  --  Install a default transport used for any remote endpoint that does not
  --  name a specific peer.  Most deployments prefer ``Register_Transport``
  --  per-peer; ``Set_Transport`` exists for legacy single-peer setups.
  function Set_Transport
    (Exec      : Pcl_Executor_Access;
     Transport : Pcl_Transport_Const_Access) return Pcl_Status;
  pragma Import(C, Set_Transport, "pcl_executor_set_transport");

  --  Bind ``Transport`` under the logical ``Peer_Id``.  Routes naming this
  --  peer dispatch through ``Transport``; ingress from the same peer is
  --  filtered against per-port allow-lists.
  function Register_Transport
    (Exec      : Pcl_Executor_Access;
     Peer_Id   : Interfaces.C.Strings.chars_ptr;
     Transport : Pcl_Transport_Const_Access) return Pcl_Status;
  pragma Import(C, Register_Transport, "pcl_executor_register_transport");

  function Dispatch_Incoming
    (Exec  : Pcl_Executor_Access;
     Topic : Interfaces.C.Strings.chars_ptr;
     Msg   : access constant Pcl_Msg) return Pcl_Status;
  pragma Import(C, Dispatch_Incoming, "pcl_executor_dispatch_incoming");

  function Invoke_Async
    (Exec         : Pcl_Executor_Access;
     Service_Name : Interfaces.C.Strings.chars_ptr;
     Request      : access constant Pcl_Msg;
     Callback     : Pcl_Resp_Cb_Access;
     User_Data    : System.Address) return Pcl_Status;
  pragma Import(C, Invoke_Async, "pcl_executor_invoke_async");

  --  Apply an endpoint-level route, primarily for consumed (client) services
  --  that have no concrete local port object to attach a route to.
  function Set_Endpoint_Route
    (Exec  : Pcl_Executor_Access;
     Route : access constant Pcl_Endpoint_Route) return Pcl_Status;
  pragma Import(C, Set_Endpoint_Route, "pcl_executor_set_endpoint_route");

  function Invoke_Stream
    (Exec         : Pcl_Executor_Access;
     Service_Name : Interfaces.C.Strings.chars_ptr;
     Request      : access constant Pcl_Msg;
     Callback     : Pcl_Stream_Msg_Cb_Access;
     User_Data    : System.Address;
     Out_Ctx      : access Pcl_Stream_Context_Access) return Pcl_Status;
  pragma Import(C, Invoke_Stream, "pcl_executor_invoke_stream");

  ---------------------------------------------------------------------------
  --  TCP socket transport
  --
  --  See ``include/pcl/pcl_transport_socket.h`` for the wire protocol and
  --  the gateway-container conventions used by the server side.
  ---------------------------------------------------------------------------

  --  Create a server-mode socket transport that binds and listens on
  --  ``Port`` (0 = OS-chosen ephemeral) and blocks until one client
  --  connects.  The caller must afterwards configure, activate, and add
  --  the result of ``Socket_Gateway_Container`` to ``Executor`` so that
  --  inbound service requests can be dispatched.
  function Create_Socket_Server
    (Port     : Interfaces.C.unsigned_short;
     Executor : Pcl_Executor_Access) return Pcl_Socket_Transport_Access;
  pragma Import(C, Create_Socket_Server, "pcl_socket_transport_create_server");

  --  Server variant that publishes the bound port via ``Port_Ready`` before
  --  blocking on accept.  Lets a sibling thread spin up a client without
  --  hard-coding an ephemeral port.
  function Create_Socket_Server_Ex
    (Port       : Interfaces.C.unsigned_short;
     Executor   : Pcl_Executor_Access;
     Port_Ready : access Interfaces.C.unsigned_short)
      return Pcl_Socket_Transport_Access;
  pragma Import(C, Create_Socket_Server_Ex, "pcl_socket_transport_create_server_ex");

  --  Return the TCP port currently bound to this transport (the assigned
  --  ephemeral port for ``Port = 0``, otherwise the supplied value).
  function Get_Socket_Port
    (Ctx : Pcl_Socket_Transport_Access) return Interfaces.C.unsigned_short;
  pragma Import(C, Get_Socket_Port, "pcl_socket_transport_get_port");

  --  Create a single-shot client transport.  Returns ``null`` if the server
  --  is not currently listening; use ``Create_Socket_Client_Ex`` for
  --  retry / auto-reconnect.
  function Create_Socket_Client
    (Host     : Interfaces.C.Strings.chars_ptr;
     Port     : Interfaces.C.unsigned_short;
     Executor : Pcl_Executor_Access) return Pcl_Socket_Transport_Access;
  pragma Import(C, Create_Socket_Client, "pcl_socket_transport_create_client");

  --  Create a client transport with bounded retry, optional auto-reconnect,
  --  and state callbacks (see ``Pcl_Socket_Client_Opts``).
  function Create_Socket_Client_Ex
    (Host     : Interfaces.C.Strings.chars_ptr;
     Port     : Interfaces.C.unsigned_short;
     Executor : Pcl_Executor_Access;
     Opts     : access constant Pcl_Socket_Client_Opts)
      return Pcl_Socket_Transport_Access;
  pragma Import(C, Create_Socket_Client_Ex, "pcl_socket_transport_create_client_ex");

  --  Thread-safe query of the current connection state.
  function Get_Socket_State
    (Ctx : Pcl_Socket_Transport_Access) return Pcl_Socket_State;
  pragma Import(C, Get_Socket_State, "pcl_socket_transport_get_state");

  --  Transport vtable handle for ``Set_Transport`` / ``Register_Transport``.
  function Get_Socket_Transport
    (Ctx : Pcl_Socket_Transport_Access) return Pcl_Transport_Const_Access;
  pragma Import(C, Get_Socket_Transport, "pcl_socket_transport_get_transport");

  --  Bind a logical peer ID for ingress filtering and route matching.  This
  --  is the same name used in ``Register_Transport`` and route peer lists.
  function Set_Socket_Peer_Id
    (Ctx     : Pcl_Socket_Transport_Access;
     Peer_Id : Interfaces.C.Strings.chars_ptr) return Pcl_Status;
  pragma Import(C, Set_Socket_Peer_Id, "pcl_socket_transport_set_peer_id");

  --  Server-mode gateway container.
  --
  --  The "gateway" is a synthetic container that the transport adds to the
  --  executor so that *inbound* service requests can be dispatched on the
  --  executor thread instead of on the transport's receive thread.
  --
  --  The gateway's only job is:
  --
  --   * subscribe to an internal service-request channel populated by the
  --     transport's receive thread;
  --   * look up the matching ``provided`` service handler in the executor
  --     and invoke it (honouring per-peer routing and allow-lists);
  --   * hand the response back to the transport so it can be framed and
  --     sent on the wire.
  --
  --  The caller is responsible for transitioning the gateway through
  --  ``Configure`` -> ``Activate`` and adding it to the executor.  Returns
  --  ``null`` for client-mode transports (which never accept inbound
  --  requests).
  function Socket_Gateway_Container
    (Ctx : Pcl_Socket_Transport_Access) return Pcl_Container_Access;
  pragma Import(C, Socket_Gateway_Container,
                "pcl_socket_transport_gateway_container");

  --  Client-side async service invocation.  Enqueues the request to the
  --  transport's send thread and returns immediately; ``Callback`` fires on
  --  the executor thread when the response arrives.
  function Invoke_Remote_Async
    (Ctx          : Pcl_Socket_Transport_Access;
     Service_Name : Interfaces.C.Strings.chars_ptr;
     Request      : access constant Pcl_Msg;
     Callback     : Pcl_Resp_Cb_Access;
     User_Data    : System.Address) return Pcl_Status;
  pragma Import(C, Invoke_Remote_Async,
                "pcl_socket_transport_invoke_remote_async");

  --  Tear down the socket, drain the send queue, and join transport threads.
  procedure Destroy_Socket_Transport(Ctx : Pcl_Socket_Transport_Access);
  pragma Import(C, Destroy_Socket_Transport, "pcl_socket_transport_destroy");

  ---------------------------------------------------------------------------
  --  UDP datagram transport (publish/subscribe only)
  --
  --  The UDP adapter intentionally leaves ``invoke_async`` / ``respond`` /
  --  ``invoke_stream`` slots null in its vtable so that the executor refuses
  --  to route service RPC over UDP.  Use the socket transport for RPC.
  ---------------------------------------------------------------------------

  --  Bind ``Local_Port`` and target ``Remote_Host``:``Remote_Port`` for
  --  outbound publishes.  One transport instance addresses one peer; create
  --  one per peer for fan-out.
  function Create_Udp_Transport
    (Local_Port  : Interfaces.C.unsigned_short;
     Remote_Host : Interfaces.C.Strings.chars_ptr;
     Remote_Port : Interfaces.C.unsigned_short;
     Executor    : Pcl_Executor_Access) return Pcl_Udp_Transport_Access;
  pragma Import(C, Create_Udp_Transport, "pcl_udp_transport_create");

  function Get_Udp_Local_Port
    (Ctx : Pcl_Udp_Transport_Access) return Interfaces.C.unsigned_short;
  pragma Import(C, Get_Udp_Local_Port, "pcl_udp_transport_get_local_port");

  function Set_Udp_Peer_Id
    (Ctx     : Pcl_Udp_Transport_Access;
     Peer_Id : Interfaces.C.Strings.chars_ptr) return Pcl_Status;
  pragma Import(C, Set_Udp_Peer_Id, "pcl_udp_transport_set_peer_id");

  function Get_Udp_Transport
    (Ctx : Pcl_Udp_Transport_Access) return Pcl_Transport_Const_Access;
  pragma Import(C, Get_Udp_Transport, "pcl_udp_transport_get_transport");

  procedure Destroy_Udp_Transport(Ctx : Pcl_Udp_Transport_Access);
  pragma Import(C, Destroy_Udp_Transport, "pcl_udp_transport_destroy");

  ---------------------------------------------------------------------------
  --  Shared-memory bus transport
  --
  --  Multiple processes on the same host join a logical bus identified by
  --  ``Bus_Name``.  Each process holds one mailbox slot keyed by
  --  ``Participant_Id``; published messages fan out to every other
  --  participant on the bus, and async service requests are delivered to
  --  the matching provider through an internal "service-request" channel.
  ---------------------------------------------------------------------------

  --  Join (or create) the shared-memory bus named ``Bus_Name`` with the
  --  given ``Participant_Id`` and bind it to ``Executor``.  The first
  --  participant on a bus initialises the OS shared region; later
  --  participants reuse it.  Participant IDs must be unique within a bus;
  --  collisions cause this call to return ``null``.
  function Create_Shared_Memory_Transport
    (Bus_Name       : Interfaces.C.Strings.chars_ptr;
     Participant_Id : Interfaces.C.Strings.chars_ptr;
     Executor       : Pcl_Executor_Access)
      return Pcl_Shared_Memory_Transport_Access;
  pragma Import(C, Create_Shared_Memory_Transport,
                "pcl_shared_memory_transport_create");

  --  Transport vtable handle for ``Set_Transport`` / ``Register_Transport``.
  function Get_Shared_Memory_Transport
    (Ctx : Pcl_Shared_Memory_Transport_Access) return Pcl_Transport_Const_Access;
  pragma Import(C, Get_Shared_Memory_Transport,
                "pcl_shared_memory_transport_get_transport");

  --  Shared-memory gateway container.
  --
  --  The gateway plays the same role as in the socket transport: it lets
  --  *inbound* service requests be dispatched on the executor thread.  The
  --  shared-memory implementation does this by:
  --
  --   * receiving service-request frames from the bus on the transport's
  --     internal poll thread;
  --   * republishing them on a private internal topic (``__pcl_shm_svc_req``)
  --     to which the gateway subscribes with ``PCL_ROUTE_REMOTE``;
  --   * having the gateway's subscriber callback invoke the matching
  --     provided service in the executor and write the response back to the
  --     originating participant's mailbox.
  --
  --  Shared-memory transports are symmetric, so unlike the socket transport
  --  every participant that wants to expose remote-callable services must
  --  configure, activate, and add this gateway to its executor.  Pure
  --  publishers and pure clients can omit it.  Returns ``null`` if ``Ctx``
  --  is null.
  function Shared_Memory_Gateway_Container
    (Ctx : Pcl_Shared_Memory_Transport_Access) return Pcl_Container_Access;
  pragma Import(C, Shared_Memory_Gateway_Container,
                "pcl_shared_memory_transport_gateway_container");

  --  Detach from the named bus.  When the last participant leaves, the
  --  underlying OS shared-memory object is unlinked.
  procedure Destroy_Shared_Memory_Transport
    (Ctx : Pcl_Shared_Memory_Transport_Access);
  pragma Import(C, Destroy_Shared_Memory_Transport,
                "pcl_shared_memory_transport_destroy");

private
  type Pcl_Executor is null record;
  pragma Convention(C, Pcl_Executor);

  type Pcl_Container is null record;
  pragma Convention(C, Pcl_Container);

  type Pcl_Port is null record;
  pragma Convention(C, Pcl_Port);

  type Pcl_Svc_Context is null record;
  pragma Convention(C, Pcl_Svc_Context);

  type Pcl_Stream_Context is null record;
  pragma Convention(C, Pcl_Stream_Context);

  type Pcl_Transport is null record;
  pragma Convention(C, Pcl_Transport);

  type Pcl_Socket_Transport is null record;
  pragma Convention(C, Pcl_Socket_Transport);

  type Pcl_Udp_Transport is null record;
  pragma Convention(C, Pcl_Udp_Transport);

  type Pcl_Shared_Memory_Transport is null record;
  pragma Convention(C, Pcl_Shared_Memory_Transport);
end Pcl_Bindings;
