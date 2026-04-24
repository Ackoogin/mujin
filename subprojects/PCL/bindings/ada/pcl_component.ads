with Ada.Containers.Indefinite_Vectors;
with Ada.Finalization;
with Ada.Strings.Unbounded;
with Interfaces.C;
with Interfaces.C.Strings;
with Pcl_Bindings;
with Pcl_Content_Types;
with System;

package Pcl_Component is
  Pcl_Error : exception;

  package Peer_Id_Vectors is new Ada.Containers.Indefinite_Vectors
    (Index_Type   => Positive,
     Element_Type => String);

  type Message_View is private;

  function Data_Address(Message : Message_View) return System.Address;
  function Size_Bytes(Message : Message_View) return Interfaces.C.unsigned;
  function Type_Name(Message : Message_View) return String;
  function To_Raw_Message(Message : Message_View) return Pcl_Bindings.Pcl_Msg;

  type Port is tagged private;

  function Is_Valid(This : Port) return Boolean;
  function Default_Type_Name(This : Port) return String;

  procedure Route_Local(This : in out Port);
  procedure Route_Remote(This : in out Port; Peer_Id : String);
  procedure Route_Remote(This : in out Port; Peer_Ids : Peer_Id_Vectors.Vector);
  procedure Route_Local_And_Remote(This : in out Port; Peer_Id : String);
  procedure Route_Local_And_Remote
    (This     : in out Port;
     Peer_Ids : Peer_Id_Vectors.Vector);

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

  type Component is abstract new Ada.Finalization.Limited_Controlled with private;

  procedure Create(This : in out Component'Class; Name : String);
  function Handle(This : Component'Class) return Pcl_Bindings.Pcl_Container_Access;
  function Name(This : Component'Class) return String;
  function State(This : Component'Class) return Pcl_Bindings.Pcl_State;

  procedure Configure(This : in out Component'Class);
  procedure Activate(This : in out Component'Class);
  procedure Deactivate(This : in out Component'Class);
  procedure Cleanup(This : in out Component'Class);
  procedure Shutdown(This : in out Component'Class);

  procedure Set_Tick_Rate_Hz
    (This : in out Component'Class;
     Hz   : Interfaces.C.double);
  function Tick_Rate_Hz(This : Component'Class) return Interfaces.C.double;

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

  procedure Subscribe
    (This      : in out Component'Class;
     Topic     : String;
     Type_Name : String);

  procedure On_Configure(This : in out Component) is null;
  procedure On_Activate(This : in out Component) is null;
  procedure On_Deactivate(This : in out Component) is null;
  procedure On_Cleanup(This : in out Component) is null;
  procedure On_Shutdown(This : in out Component) is null;
  procedure On_Tick
    (This       : in out Component;
     Dt_Seconds : Interfaces.C.double) is null;
  procedure On_Message
    (This    : in out Component;
     Topic   : String;
     Message : Message_View) is null;

  type Executor is new Ada.Finalization.Limited_Controlled with private;

  procedure Create(This : in out Executor);
  function Handle(This : Executor) return Pcl_Bindings.Pcl_Executor_Access;

  procedure Add(This : in out Executor; Item : in out Component'Class);
  procedure Add(This : in out Executor; Container : Pcl_Bindings.Pcl_Container_Access);
  procedure Remove(This : in out Executor; Item : in out Component'Class);
  procedure Remove(This : in out Executor; Container : Pcl_Bindings.Pcl_Container_Access);

  procedure Spin(This : in out Executor);
  function Spin_Status(This : in out Executor) return Pcl_Bindings.Pcl_Status;
  procedure Spin_Once
    (This       : in out Executor;
     Timeout_Ms : Interfaces.C.unsigned := 0);
  function Spin_Once_Status
    (This       : in out Executor;
     Timeout_Ms : Interfaces.C.unsigned := 0) return Pcl_Bindings.Pcl_Status;
  procedure Shutdown_Graceful
    (This       : in out Executor;
     Timeout_Ms : Interfaces.C.unsigned := 5_000);
  procedure Request_Shutdown(This : in out Executor);

  procedure Set_Transport
    (This      : in out Executor;
     Transport : Pcl_Bindings.Pcl_Transport_Const_Access);
  procedure Register_Transport
    (This      : in out Executor;
     Peer_Id   : String;
     Transport : Pcl_Bindings.Pcl_Transport_Const_Access);

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
