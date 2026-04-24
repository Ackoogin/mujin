with Interfaces.C;
with Interfaces.C.Extensions;
with Interfaces.C.Strings;
with System;

package Pcl_Bindings is
  subtype C_Bool is Interfaces.C.Extensions.bool;

  type Pcl_Status is new Interfaces.C.int;

  PCL_OK              : constant Pcl_Status := 0;
  PCL_PENDING         : constant Pcl_Status := 1;
  PCL_STREAMING       : constant Pcl_Status := 2;
  PCL_ERR_INVALID     : constant Pcl_Status := -1;
  PCL_ERR_STATE       : constant Pcl_Status := -2;
  PCL_ERR_TIMEOUT     : constant Pcl_Status := -3;
  PCL_ERR_CALLBACK    : constant Pcl_Status := -4;
  PCL_ERR_NOMEM       : constant Pcl_Status := -5;
  PCL_ERR_NOT_FOUND   : constant Pcl_Status := -6;
  PCL_ERR_PORT_CLOSED : constant Pcl_Status := -7;
  PCL_ERR_CANCELLED   : constant Pcl_Status := -8;

  type Pcl_State is
    (PCL_STATE_UNCONFIGURED,
     PCL_STATE_CONFIGURED,
     PCL_STATE_ACTIVE,
     PCL_STATE_FINALIZED);
  pragma Convention(C, Pcl_State);

  type Pcl_Port_Type is
    (PCL_PORT_PUBLISHER,
     PCL_PORT_SUBSCRIBER,
     PCL_PORT_SERVICE,
     PCL_PORT_CLIENT,
     PCL_PORT_STREAM_SERVICE);
  pragma Convention(C, Pcl_Port_Type);

  PCL_ROUTE_NONE   : constant Interfaces.C.unsigned := 0;
  PCL_ROUTE_LOCAL  : constant Interfaces.C.unsigned := 1;
  PCL_ROUTE_REMOTE : constant Interfaces.C.unsigned := 2;

  PCL_MAX_ENDPOINT_PEERS : constant Interfaces.C.unsigned := 8;

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

  type Pcl_Socket_State is
    (PCL_SOCKET_STATE_CONNECTING,
     PCL_SOCKET_STATE_CONNECTED,
     PCL_SOCKET_STATE_DISCONNECTED);
  pragma Convention(C, Pcl_Socket_State);

  type Pcl_Executor is limited private;
  type Pcl_Executor_Access is access all Pcl_Executor;
  pragma Convention(C, Pcl_Executor_Access);

  type Pcl_Container is limited private;
  type Pcl_Container_Access is access all Pcl_Container;
  pragma Convention(C, Pcl_Container_Access);

  type Pcl_Port is limited private;
  type Pcl_Port_Access is access all Pcl_Port;
  pragma Convention(C, Pcl_Port_Access);

  type Pcl_Svc_Context is limited private;
  type Pcl_Svc_Context_Access is access all Pcl_Svc_Context;
  pragma Convention(C, Pcl_Svc_Context_Access);

  type Pcl_Stream_Context is limited private;
  type Pcl_Stream_Context_Access is access all Pcl_Stream_Context;
  pragma Convention(C, Pcl_Stream_Context_Access);

  type Pcl_Transport is limited private;
  type Pcl_Transport_Access is access all Pcl_Transport;
  pragma Convention(C, Pcl_Transport_Access);

  type Pcl_Transport_Const_Access is access constant Pcl_Transport;
  pragma Convention(C, Pcl_Transport_Const_Access);

  type Pcl_Socket_Transport is limited private;
  type Pcl_Socket_Transport_Access is access all Pcl_Socket_Transport;
  pragma Convention(C, Pcl_Socket_Transport_Access);

  type Pcl_Udp_Transport is limited private;
  type Pcl_Udp_Transport_Access is access all Pcl_Udp_Transport;
  pragma Convention(C, Pcl_Udp_Transport_Access);

  type Pcl_Shared_Memory_Transport is limited private;
  type Pcl_Shared_Memory_Transport_Access is access all Pcl_Shared_Memory_Transport;
  pragma Convention(C, Pcl_Shared_Memory_Transport_Access);

  type Pcl_Msg is record
    Data      : System.Address;
    Size      : Interfaces.C.unsigned;
    Type_Name : Interfaces.C.Strings.chars_ptr;
  end record;
  pragma Convention(C, Pcl_Msg);

  type Pcl_Endpoint_Route is record
    Endpoint_Name : Interfaces.C.Strings.chars_ptr;
    Endpoint_Kind : Pcl_Endpoint_Kind;
    Route_Mode    : Interfaces.C.unsigned;
    Peer_Ids      : System.Address;
    Peer_Count    : Interfaces.C.unsigned;
  end record;
  pragma Convention(C, Pcl_Endpoint_Route);

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

  type On_Tick_Access is access function
    (Self       : Pcl_Container_Access;
     Dt_Seconds : Interfaces.C.double;
     User_Data  : System.Address) return Pcl_Status;
  pragma Convention(C, On_Tick_Access);

  type Pcl_Callbacks is record
    On_Configure  : On_Configure_Access := null;
    On_Activate   : On_Activate_Access := null;
    On_Deactivate : On_Deactivate_Access := null;
    On_Cleanup    : On_Cleanup_Access := null;
    On_Shutdown   : On_Shutdown_Access := null;
    On_Tick       : On_Tick_Access := null;
  end record;
  pragma Convention(C, Pcl_Callbacks);

  type Pcl_Sub_Callback_Access is access procedure
    (Self      : Pcl_Container_Access;
     Msg       : access constant Pcl_Msg;
     User_Data : System.Address);
  pragma Convention(C, Pcl_Sub_Callback_Access);

  type Pcl_Service_Handler_Access is access function
    (Self      : Pcl_Container_Access;
     Request   : access constant Pcl_Msg;
     Response  : access Pcl_Msg;
     Ctx       : Pcl_Svc_Context_Access;
     User_Data : System.Address) return Pcl_Status;
  pragma Convention(C, Pcl_Service_Handler_Access);

  type Pcl_Stream_Handler_Access is access function
    (Self       : Pcl_Container_Access;
     Request    : access constant Pcl_Msg;
     Stream_Ctx : Pcl_Stream_Context_Access;
     User_Data  : System.Address) return Pcl_Status;
  pragma Convention(C, Pcl_Stream_Handler_Access);

  type Pcl_Resp_Cb_Access is access procedure
    (Resp      : access constant Pcl_Msg;
     User_Data : System.Address);
  pragma Convention(C, Pcl_Resp_Cb_Access);

  type Pcl_Stream_Msg_Cb_Access is access procedure
    (Msg           : access constant Pcl_Msg;
     End_Of_Stream : C_Bool;
     Status        : Pcl_Status;
     User_Data     : System.Address);
  pragma Convention(C, Pcl_Stream_Msg_Cb_Access);

  type Pcl_Socket_State_Cb_Access is access procedure
    (State     : Pcl_Socket_State;
     User_Data : System.Address);
  pragma Convention(C, Pcl_Socket_State_Cb_Access);

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

  type Pcl_Socket_Client_Opts is record
    Connect_Timeout_Ms : Interfaces.C.unsigned := 0;
    Max_Retries        : Interfaces.C.unsigned := 0;
    Auto_Reconnect     : Interfaces.C.int := 0;
    State_Cb           : Pcl_Socket_State_Cb_Access := null;
    State_Cb_Data      : System.Address := System.Null_Address;
  end record;
  pragma Convention(C, Pcl_Socket_Client_Opts);

  function Create_Executor return Pcl_Executor_Access;
  pragma Import(C, Create_Executor, "pcl_executor_create");

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

  function Spin(Exec : Pcl_Executor_Access) return Pcl_Status;
  pragma Import(C, Spin, "pcl_executor_spin");

  function Spin_Once
    (Exec       : Pcl_Executor_Access;
     Timeout_Ms : Interfaces.C.unsigned) return Pcl_Status;
  pragma Import(C, Spin_Once, "pcl_executor_spin_once");

  procedure Request_Shutdown(Exec : Pcl_Executor_Access);
  pragma Import(C, Request_Shutdown, "pcl_executor_request_shutdown");

  function Shutdown_Graceful
    (Exec       : Pcl_Executor_Access;
     Timeout_Ms : Interfaces.C.unsigned) return Pcl_Status;
  pragma Import(C, Shutdown_Graceful, "pcl_executor_shutdown_graceful");

  function Post_Incoming
    (Exec  : Pcl_Executor_Access;
     Topic : Interfaces.C.Strings.chars_ptr;
     Msg   : access constant Pcl_Msg) return Pcl_Status;
  pragma Import(C, Post_Incoming, "pcl_executor_post_incoming");

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

  function Invoke_Service
    (Exec         : Pcl_Executor_Access;
     Service_Name : Interfaces.C.Strings.chars_ptr;
     Request      : access constant Pcl_Msg;
     Response     : access Pcl_Msg) return Pcl_Status;
  pragma Import(C, Invoke_Service, "pcl_executor_invoke_service");

  function Invoke_Service_Remote
    (Exec         : Pcl_Executor_Access;
     Peer_Id      : Interfaces.C.Strings.chars_ptr;
     Service_Name : Interfaces.C.Strings.chars_ptr;
     Request      : access constant Pcl_Msg;
     Response     : access Pcl_Msg) return Pcl_Status;
  pragma Import(C, Invoke_Service_Remote, "pcl_executor_invoke_service_remote");

  function Publish
    (Exec  : Pcl_Executor_Access;
     Topic : Interfaces.C.Strings.chars_ptr;
     Msg   : access constant Pcl_Msg) return Pcl_Status;
  pragma Import(C, Publish, "pcl_executor_publish");

  function Create_Container
    (Name      : Interfaces.C.Strings.chars_ptr;
     Callbacks : access constant Pcl_Callbacks;
     User_Data : System.Address) return Pcl_Container_Access;
  pragma Import(C, Create_Container, "pcl_container_create");

  procedure Destroy_Container(Container : Pcl_Container_Access);
  pragma Import(C, Destroy_Container, "pcl_container_destroy");

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

  function Set_Param_I64
    (Container : Pcl_Container_Access;
     Key       : Interfaces.C.Strings.chars_ptr;
     Value     : Interfaces.C.long_long) return Pcl_Status;
  pragma Import(C, Set_Param_I64, "pcl_container_set_param_i64");

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

  function Get_Param_I64
    (Container   : Pcl_Container_Access;
     Key         : Interfaces.C.Strings.chars_ptr;
     Default_Val : Interfaces.C.long_long) return Interfaces.C.long_long;
  pragma Import(C, Get_Param_I64, "pcl_container_get_param_i64");

  function Get_Param_Bool
    (Container   : Pcl_Container_Access;
     Key         : Interfaces.C.Strings.chars_ptr;
     Default_Val : C_Bool) return C_Bool;
  pragma Import(C, Get_Param_Bool, "pcl_container_get_param_bool");

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

  function Port_Set_Route
    (Port       : Pcl_Port_Access;
     Route_Mode : Interfaces.C.unsigned;
     Peer_Ids   : System.Address;
     Peer_Count : Interfaces.C.unsigned) return Pcl_Status;
  pragma Import(C, Port_Set_Route, "pcl_port_set_route");

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

  function Service_Respond
    (Ctx      : Pcl_Svc_Context_Access;
     Response : access constant Pcl_Msg) return Pcl_Status;
  pragma Import(C, Service_Respond, "pcl_service_respond");

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

  function Set_Transport
    (Exec      : Pcl_Executor_Access;
     Transport : Pcl_Transport_Const_Access) return Pcl_Status;
  pragma Import(C, Set_Transport, "pcl_executor_set_transport");

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

  function Create_Socket_Server
    (Port     : Interfaces.C.unsigned_short;
     Executor : Pcl_Executor_Access) return Pcl_Socket_Transport_Access;
  pragma Import(C, Create_Socket_Server, "pcl_socket_transport_create_server");

  function Create_Socket_Server_Ex
    (Port       : Interfaces.C.unsigned_short;
     Executor   : Pcl_Executor_Access;
     Port_Ready : access Interfaces.C.unsigned_short)
      return Pcl_Socket_Transport_Access;
  pragma Import(C, Create_Socket_Server_Ex, "pcl_socket_transport_create_server_ex");

  function Get_Socket_Port
    (Ctx : Pcl_Socket_Transport_Access) return Interfaces.C.unsigned_short;
  pragma Import(C, Get_Socket_Port, "pcl_socket_transport_get_port");

  function Create_Socket_Client
    (Host     : Interfaces.C.Strings.chars_ptr;
     Port     : Interfaces.C.unsigned_short;
     Executor : Pcl_Executor_Access) return Pcl_Socket_Transport_Access;
  pragma Import(C, Create_Socket_Client, "pcl_socket_transport_create_client");

  function Create_Socket_Client_Ex
    (Host     : Interfaces.C.Strings.chars_ptr;
     Port     : Interfaces.C.unsigned_short;
     Executor : Pcl_Executor_Access;
     Opts     : access constant Pcl_Socket_Client_Opts)
      return Pcl_Socket_Transport_Access;
  pragma Import(C, Create_Socket_Client_Ex, "pcl_socket_transport_create_client_ex");

  function Get_Socket_State
    (Ctx : Pcl_Socket_Transport_Access) return Pcl_Socket_State;
  pragma Import(C, Get_Socket_State, "pcl_socket_transport_get_state");

  function Get_Socket_Transport
    (Ctx : Pcl_Socket_Transport_Access) return Pcl_Transport_Const_Access;
  pragma Import(C, Get_Socket_Transport, "pcl_socket_transport_get_transport");

  function Set_Socket_Peer_Id
    (Ctx     : Pcl_Socket_Transport_Access;
     Peer_Id : Interfaces.C.Strings.chars_ptr) return Pcl_Status;
  pragma Import(C, Set_Socket_Peer_Id, "pcl_socket_transport_set_peer_id");

  function Socket_Gateway_Container
    (Ctx : Pcl_Socket_Transport_Access) return Pcl_Container_Access;
  pragma Import(C, Socket_Gateway_Container,
                "pcl_socket_transport_gateway_container");

  function Invoke_Remote_Async
    (Ctx          : Pcl_Socket_Transport_Access;
     Service_Name : Interfaces.C.Strings.chars_ptr;
     Request      : access constant Pcl_Msg;
     Callback     : Pcl_Resp_Cb_Access;
     User_Data    : System.Address) return Pcl_Status;
  pragma Import(C, Invoke_Remote_Async,
                "pcl_socket_transport_invoke_remote_async");

  procedure Destroy_Socket_Transport(Ctx : Pcl_Socket_Transport_Access);
  pragma Import(C, Destroy_Socket_Transport, "pcl_socket_transport_destroy");

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

  function Create_Shared_Memory_Transport
    (Bus_Name       : Interfaces.C.Strings.chars_ptr;
     Participant_Id : Interfaces.C.Strings.chars_ptr;
     Executor       : Pcl_Executor_Access)
      return Pcl_Shared_Memory_Transport_Access;
  pragma Import(C, Create_Shared_Memory_Transport,
                "pcl_shared_memory_transport_create");

  function Get_Shared_Memory_Transport
    (Ctx : Pcl_Shared_Memory_Transport_Access) return Pcl_Transport_Const_Access;
  pragma Import(C, Get_Shared_Memory_Transport,
                "pcl_shared_memory_transport_get_transport");

  function Shared_Memory_Gateway_Container
    (Ctx : Pcl_Shared_Memory_Transport_Access) return Pcl_Container_Access;
  pragma Import(C, Shared_Memory_Gateway_Container,
                "pcl_shared_memory_transport_gateway_container");

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
