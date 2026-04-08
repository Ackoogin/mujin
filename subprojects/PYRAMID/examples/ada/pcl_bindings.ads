with Interfaces.C;
with Interfaces.C.Strings;
with System;

package Pcl_Bindings is
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

  type Pcl_Msg is record
    Data      : System.Address;
    Size      : Interfaces.C.unsigned;
    Type_Name : Interfaces.C.Strings.chars_ptr;
  end record;
  pragma Convention(C, Pcl_Msg);

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

  function Create_Executor return Pcl_Executor_Access;
  pragma Import(C, Create_Executor, "pcl_executor_create");

  procedure Destroy_Executor(Exec : Pcl_Executor_Access);
  pragma Import(C, Destroy_Executor, "pcl_executor_destroy");

  function Add_Container(Exec      : Pcl_Executor_Access;
                         Container : Pcl_Container_Access) return Pcl_Status;
  pragma Import(C, Add_Container, "pcl_executor_add");

  function Spin(Exec : Pcl_Executor_Access) return Pcl_Status;
  pragma Import(C, Spin, "pcl_executor_spin");

  function Spin_Once(Exec       : Pcl_Executor_Access;
                     Timeout_Ms : Interfaces.C.unsigned) return Pcl_Status;
  pragma Import(C, Spin_Once, "pcl_executor_spin_once");

  function Shutdown_Graceful(Exec       : Pcl_Executor_Access;
                             Timeout_Ms : Interfaces.C.unsigned) return Pcl_Status;
  pragma Import(C, Shutdown_Graceful, "pcl_executor_shutdown_graceful");

  procedure Request_Shutdown(Exec : Pcl_Executor_Access);
  pragma Import(C, Request_Shutdown, "pcl_executor_request_shutdown");

  function Post_Incoming(Exec  : Pcl_Executor_Access;
                         Topic : Interfaces.C.Strings.chars_ptr;
                         Msg   : access constant Pcl_Msg) return Pcl_Status;
  pragma Import(C, Post_Incoming, "pcl_executor_post_incoming");

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

  function Set_Tick_Rate_Hz(Container : Pcl_Container_Access;
                            Hz        : Interfaces.C.double) return Pcl_Status;
  pragma Import(C, Set_Tick_Rate_Hz, "pcl_container_set_tick_rate_hz");

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

  -- -- Service invocation -------------------------------------------------

  function Invoke_Service
    (Exec         : Pcl_Executor_Access;
     Service_Name : Interfaces.C.Strings.chars_ptr;
     Request      : access constant Pcl_Msg;
     Response     : access Pcl_Msg) return Pcl_Status;
  pragma Import(C, Invoke_Service, "pcl_executor_invoke_service");

  function Service_Respond
    (Ctx      : Pcl_Svc_Context_Access;
     Response : access constant Pcl_Msg) return Pcl_Status;
  pragma Import(C, Service_Respond, "pcl_service_respond");

  procedure Service_Context_Free(Ctx : Pcl_Svc_Context_Access);
  pragma Import(C, Service_Context_Free, "pcl_service_context_free");

  -- -- Async service response callback ------------------------------------

  type Pcl_Resp_Cb_Access is access procedure
    (Resp      : access constant Pcl_Msg;
     User_Data : System.Address);
  pragma Convention(C, Pcl_Resp_Cb_Access);

  -- -- Transport adapter --------------------------------------------------

  type Pcl_Transport is limited private;
  type Pcl_Transport_Access is access all Pcl_Transport;
  pragma Convention(C, Pcl_Transport_Access);

  type Pcl_Transport_Const_Access is access constant Pcl_Transport;
  pragma Convention(C, Pcl_Transport_Const_Access);

  function Set_Transport
    (Exec      : Pcl_Executor_Access;
     Transport : Pcl_Transport_Const_Access) return Pcl_Status;
  pragma Import(C, Set_Transport, "pcl_executor_set_transport");

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

  function Publish
    (Exec  : Pcl_Executor_Access;
     Topic : Interfaces.C.Strings.chars_ptr;
     Msg   : access constant Pcl_Msg) return Pcl_Status;
  pragma Import(C, Publish, "pcl_executor_publish");

  -- -- Socket transport ---------------------------------------------------

  type Pcl_Socket_Transport is limited private;
  type Pcl_Socket_Transport_Access is access all Pcl_Socket_Transport;
  pragma Convention(C, Pcl_Socket_Transport_Access);

  function Create_Socket_Server
    (Port     : Interfaces.C.unsigned_short;
     Executor : Pcl_Executor_Access) return Pcl_Socket_Transport_Access;
  pragma Import(C, Create_Socket_Server, "pcl_socket_transport_create_server");

  function Create_Socket_Client
    (Host     : Interfaces.C.Strings.chars_ptr;
     Port     : Interfaces.C.unsigned_short;
     Executor : Pcl_Executor_Access) return Pcl_Socket_Transport_Access;
  pragma Import(C, Create_Socket_Client, "pcl_socket_transport_create_client");

  function Get_Socket_Transport
    (Ctx : Pcl_Socket_Transport_Access) return Pcl_Transport_Const_Access;
  pragma Import(C, Get_Socket_Transport, "pcl_socket_transport_get_transport");

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

private
  type Pcl_Executor is null record;
  pragma Convention(C, Pcl_Executor);

  type Pcl_Container is null record;
  pragma Convention(C, Pcl_Container);

  type Pcl_Port is null record;
  pragma Convention(C, Pcl_Port);

  type Pcl_Svc_Context is null record;
  pragma Convention(C, Pcl_Svc_Context);

  type Pcl_Transport is null record;
  pragma Convention(C, Pcl_Transport);

  type Pcl_Socket_Transport is null record;
  pragma Convention(C, Pcl_Socket_Transport);
end Pcl_Bindings;
