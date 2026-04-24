with Ada.Finalization;
with Ada.Strings.Unbounded;
with Interfaces.C;
with Pcl_Bindings;
with Pcl_Component;
with System;

package Pcl_Transports is
  type Socket_Client_Options is record
    Connect_Timeout_Ms : Interfaces.C.unsigned := 0;
    Max_Retries        : Interfaces.C.unsigned := 0;
    Auto_Reconnect     : Boolean := False;
    State_Callback     : Pcl_Bindings.Pcl_Socket_State_Cb_Access := null;
    State_Callback_Data : System.Address := System.Null_Address;
  end record;

  type Socket_Transport is new Ada.Finalization.Limited_Controlled with private;

  procedure Create_Server
    (This : in out Socket_Transport;
     Exec : in out Pcl_Component.Executor;
     Port : Interfaces.C.unsigned_short := 0);
  procedure Create_Server
    (This       : in out Socket_Transport;
     Exec       : in out Pcl_Component.Executor;
     Port       : Interfaces.C.unsigned_short;
     Bound_Port : out Interfaces.C.unsigned_short);
  procedure Create_Client
    (This    : in out Socket_Transport;
     Exec    : in out Pcl_Component.Executor;
     Host    : String;
     Port    : Interfaces.C.unsigned_short;
     Options : Socket_Client_Options := (others => <>));

  function Handle(This : Socket_Transport) return Pcl_Bindings.Pcl_Socket_Transport_Access;
  function Transport(This : Socket_Transport) return Pcl_Bindings.Pcl_Transport_Const_Access;
  function Gateway_Container(This : Socket_Transport) return Pcl_Bindings.Pcl_Container_Access;
  function Port(This : Socket_Transport) return Interfaces.C.unsigned_short;
  function State(This : Socket_Transport) return Pcl_Bindings.Pcl_Socket_State;

  procedure Set_Peer_Id(This : in out Socket_Transport; Peer_Id : String);
  procedure Use_As_Default
    (This : in out Socket_Transport;
     Exec : in out Pcl_Component.Executor);
  procedure Register
    (This : in out Socket_Transport;
     Exec : in out Pcl_Component.Executor);
  procedure Register
    (This    : in out Socket_Transport;
     Exec    : in out Pcl_Component.Executor;
     Peer_Id : String);
  procedure Start_Gateway
    (This : in out Socket_Transport;
     Exec : in out Pcl_Component.Executor);

  type Udp_Transport is new Ada.Finalization.Limited_Controlled with private;

  procedure Create
    (This        : in out Udp_Transport;
     Exec        : in out Pcl_Component.Executor;
     Local_Port  : Interfaces.C.unsigned_short;
     Remote_Host : String;
     Remote_Port : Interfaces.C.unsigned_short);

  function Handle(This : Udp_Transport) return Pcl_Bindings.Pcl_Udp_Transport_Access;
  function Transport(This : Udp_Transport) return Pcl_Bindings.Pcl_Transport_Const_Access;
  function Local_Port(This : Udp_Transport) return Interfaces.C.unsigned_short;

  procedure Set_Peer_Id(This : in out Udp_Transport; Peer_Id : String);
  procedure Use_As_Default
    (This : in out Udp_Transport;
     Exec : in out Pcl_Component.Executor);
  procedure Register
    (This : in out Udp_Transport;
     Exec : in out Pcl_Component.Executor);
  procedure Register
    (This    : in out Udp_Transport;
     Exec    : in out Pcl_Component.Executor;
     Peer_Id : String);

  type Shared_Memory_Transport is new Ada.Finalization.Limited_Controlled with private;

  procedure Create
    (This           : in out Shared_Memory_Transport;
     Exec           : in out Pcl_Component.Executor;
     Bus_Name       : String;
     Participant_Id : String);

  function Handle
    (This : Shared_Memory_Transport)
      return Pcl_Bindings.Pcl_Shared_Memory_Transport_Access;
  function Transport
    (This : Shared_Memory_Transport)
      return Pcl_Bindings.Pcl_Transport_Const_Access;
  function Gateway_Container
    (This : Shared_Memory_Transport) return Pcl_Bindings.Pcl_Container_Access;

  procedure Use_As_Default
    (This : in out Shared_Memory_Transport;
     Exec : in out Pcl_Component.Executor);
  procedure Register
    (This : in out Shared_Memory_Transport;
     Exec : in out Pcl_Component.Executor);
  procedure Register
    (This    : in out Shared_Memory_Transport;
     Exec    : in out Pcl_Component.Executor;
     Peer_Id : String);
  procedure Start_Gateway
    (This : in out Shared_Memory_Transport;
     Exec : in out Pcl_Component.Executor);

private
  type Socket_Transport is new Ada.Finalization.Limited_Controlled with record
    Handle  : Pcl_Bindings.Pcl_Socket_Transport_Access := null;
    Peer_Id : Ada.Strings.Unbounded.Unbounded_String :=
      Ada.Strings.Unbounded.Null_Unbounded_String;
  end record;

  overriding procedure Finalize(This : in out Socket_Transport);

  type Udp_Transport is new Ada.Finalization.Limited_Controlled with record
    Handle  : Pcl_Bindings.Pcl_Udp_Transport_Access := null;
    Peer_Id : Ada.Strings.Unbounded.Unbounded_String :=
      Ada.Strings.Unbounded.Null_Unbounded_String;
  end record;

  overriding procedure Finalize(This : in out Udp_Transport);

  type Shared_Memory_Transport is new Ada.Finalization.Limited_Controlled with record
    Handle         : Pcl_Bindings.Pcl_Shared_Memory_Transport_Access := null;
    Participant_Id : Ada.Strings.Unbounded.Unbounded_String :=
      Ada.Strings.Unbounded.Null_Unbounded_String;
  end record;

  overriding procedure Finalize(This : in out Shared_Memory_Transport);
end Pcl_Transports;
