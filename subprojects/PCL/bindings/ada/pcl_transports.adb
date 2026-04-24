with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Interfaces.C.Strings;
with System;

package body Pcl_Transports is
  use type Pcl_Bindings.Pcl_Socket_Transport_Access;
  use type Pcl_Bindings.Pcl_Udp_Transport_Access;
  use type Pcl_Bindings.Pcl_Shared_Memory_Transport_Access;
  use type Pcl_Bindings.Pcl_Container_Access;
  use type Pcl_Bindings.Pcl_Status;
  use type Interfaces.C.unsigned_short;

  procedure Raise_Error(Message : String) is
  begin
    raise Pcl_Component.Pcl_Error with Message;
  end Raise_Error;

  procedure Check(Status : Pcl_Bindings.Pcl_Status;
                  Context : String) is
  begin
    if Status /= Pcl_Bindings.PCL_OK then
      Raise_Error(Context & " failed with status" &
                    Interfaces.C.int'Image(Interfaces.C.int(Status)));
    end if;
  end Check;

  procedure Start_Gateway_Container
    (Exec      : in out Pcl_Component.Executor;
     Container : Pcl_Bindings.Pcl_Container_Access;
     Context   : String) is
  begin
    if Container = null then
      Raise_Error(Context & " has no gateway container");
    end if;

    Check(Pcl_Bindings.Configure(Container), Context & " configure");
    Check(Pcl_Bindings.Activate(Container), Context & " activate");
    Check(Pcl_Bindings.Add_Container(Pcl_Component.Handle(Exec), Container),
          Context & " add gateway");
  end Start_Gateway_Container;

  procedure Register_Peer
    (Exec      : in out Pcl_Component.Executor;
     Peer_Id   : String;
     Transport : Pcl_Bindings.Pcl_Transport_Const_Access) is
    Peer_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Peer_Id);
  begin
    Check(Pcl_Bindings.Register_Transport(Pcl_Component.Handle(Exec), Peer_C, Transport),
          "register transport");
    Interfaces.C.Strings.Free(Peer_C);
  end Register_Peer;

  procedure Create_Server
    (This : in out Socket_Transport;
     Exec : in out Pcl_Component.Executor;
     Port : Interfaces.C.unsigned_short := 0) is
    Bound_Port : Interfaces.C.unsigned_short;
    pragma Unreferenced(Bound_Port);
  begin
    Create_Server(This, Exec, Port, Bound_Port);
  end Create_Server;

  procedure Create_Server
    (This       : in out Socket_Transport;
     Exec       : in out Pcl_Component.Executor;
     Port       : Interfaces.C.unsigned_short;
     Bound_Port : out Interfaces.C.unsigned_short) is
    Ready : aliased Interfaces.C.unsigned_short := 0;
  begin
    if This.Handle /= null then
      Raise_Error("socket transport already created");
    end if;

    This.Handle := Pcl_Bindings.Create_Socket_Server_Ex
      (Port       => Port,
       Executor   => Pcl_Component.Handle(Exec),
       Port_Ready => Ready'Access);
    if This.Handle = null then
      Raise_Error("failed to create socket server transport");
    end if;

    Bound_Port := (if Ready /= 0 then Ready else Pcl_Bindings.Get_Socket_Port(This.Handle));
  end Create_Server;

  procedure Create_Client
    (This    : in out Socket_Transport;
     Exec    : in out Pcl_Component.Executor;
     Host    : String;
     Port    : Interfaces.C.unsigned_short;
     Options : Socket_Client_Options := (others => <>)) is
    Host_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Host);
    Opts_C  : aliased Pcl_Bindings.Pcl_Socket_Client_Opts :=
      (Connect_Timeout_Ms => Options.Connect_Timeout_Ms,
       Max_Retries        => Options.Max_Retries,
       Auto_Reconnect     => (if Options.Auto_Reconnect then 1 else 0),
       State_Cb           => Options.State_Callback,
       State_Cb_Data      => Options.State_Callback_Data);
  begin
    if This.Handle /= null then
      Raise_Error("socket transport already created");
    end if;

    This.Handle := Pcl_Bindings.Create_Socket_Client_Ex
      (Host     => Host_C,
       Port     => Port,
       Executor => Pcl_Component.Handle(Exec),
       Opts     => Opts_C'Access);
    Interfaces.C.Strings.Free(Host_C);

    if This.Handle = null then
      Raise_Error("failed to create socket client transport");
    end if;
  end Create_Client;

  function Handle(This : Socket_Transport) return Pcl_Bindings.Pcl_Socket_Transport_Access is
  begin
    return This.Handle;
  end Handle;

  function Transport(This : Socket_Transport) return Pcl_Bindings.Pcl_Transport_Const_Access is
  begin
    return Pcl_Bindings.Get_Socket_Transport(This.Handle);
  end Transport;

  function Gateway_Container(This : Socket_Transport) return Pcl_Bindings.Pcl_Container_Access is
  begin
    return Pcl_Bindings.Socket_Gateway_Container(This.Handle);
  end Gateway_Container;

  function Port(This : Socket_Transport) return Interfaces.C.unsigned_short is
  begin
    return Pcl_Bindings.Get_Socket_Port(This.Handle);
  end Port;

  function State(This : Socket_Transport) return Pcl_Bindings.Pcl_Socket_State is
  begin
    return Pcl_Bindings.Get_Socket_State(This.Handle);
  end State;

  procedure Set_Peer_Id(This : in out Socket_Transport; Peer_Id : String) is
    Peer_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Peer_Id);
  begin
    if This.Handle = null then
      Raise_Error("socket transport has not been created");
    end if;

    Check(Pcl_Bindings.Set_Socket_Peer_Id(This.Handle, Peer_C),
          "set socket peer id");
    Interfaces.C.Strings.Free(Peer_C);
    This.Peer_Id := To_Unbounded_String(Peer_Id);
  end Set_Peer_Id;

  procedure Use_As_Default
    (This : in out Socket_Transport;
     Exec : in out Pcl_Component.Executor) is
  begin
    if This.Handle = null then
      Raise_Error("socket transport has not been created");
    end if;

    Pcl_Component.Set_Transport(Exec, Transport(This));
  end Use_As_Default;

  procedure Register
    (This : in out Socket_Transport;
     Exec : in out Pcl_Component.Executor) is
  begin
    if Length(This.Peer_Id) = 0 then
      Raise_Error("socket transport peer id has not been set");
    end if;

    Register_Peer(Exec, To_String(This.Peer_Id), Transport(This));
  end Register;

  procedure Register
    (This    : in out Socket_Transport;
     Exec    : in out Pcl_Component.Executor;
     Peer_Id : String) is
  begin
    Set_Peer_Id(This, Peer_Id);
    Register(This, Exec);
  end Register;

  procedure Start_Gateway
    (This : in out Socket_Transport;
     Exec : in out Pcl_Component.Executor) is
  begin
    Start_Gateway_Container(Exec, Gateway_Container(This), "socket transport");
  end Start_Gateway;

  overriding procedure Finalize(This : in out Socket_Transport) is
  begin
    if This.Handle /= null then
      Pcl_Bindings.Destroy_Socket_Transport(This.Handle);
      This.Handle := null;
    end if;
  end Finalize;

  procedure Create
    (This        : in out Udp_Transport;
     Exec        : in out Pcl_Component.Executor;
     Local_Port  : Interfaces.C.unsigned_short;
     Remote_Host : String;
     Remote_Port : Interfaces.C.unsigned_short) is
    Host_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Remote_Host);
  begin
    if This.Handle /= null then
      Raise_Error("udp transport already created");
    end if;

    This.Handle := Pcl_Bindings.Create_Udp_Transport
      (Local_Port  => Local_Port,
       Remote_Host => Host_C,
       Remote_Port => Remote_Port,
       Executor    => Pcl_Component.Handle(Exec));
    Interfaces.C.Strings.Free(Host_C);

    if This.Handle = null then
      Raise_Error("failed to create udp transport");
    end if;
  end Create;

  function Handle(This : Udp_Transport) return Pcl_Bindings.Pcl_Udp_Transport_Access is
  begin
    return This.Handle;
  end Handle;

  function Transport(This : Udp_Transport) return Pcl_Bindings.Pcl_Transport_Const_Access is
  begin
    return Pcl_Bindings.Get_Udp_Transport(This.Handle);
  end Transport;

  function Local_Port(This : Udp_Transport) return Interfaces.C.unsigned_short is
  begin
    return Pcl_Bindings.Get_Udp_Local_Port(This.Handle);
  end Local_Port;

  procedure Set_Peer_Id(This : in out Udp_Transport; Peer_Id : String) is
    Peer_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Peer_Id);
  begin
    if This.Handle = null then
      Raise_Error("udp transport has not been created");
    end if;

    Check(Pcl_Bindings.Set_Udp_Peer_Id(This.Handle, Peer_C), "set udp peer id");
    Interfaces.C.Strings.Free(Peer_C);
    This.Peer_Id := To_Unbounded_String(Peer_Id);
  end Set_Peer_Id;

  procedure Use_As_Default
    (This : in out Udp_Transport;
     Exec : in out Pcl_Component.Executor) is
  begin
    if This.Handle = null then
      Raise_Error("udp transport has not been created");
    end if;

    Pcl_Component.Set_Transport(Exec, Transport(This));
  end Use_As_Default;

  procedure Register
    (This : in out Udp_Transport;
     Exec : in out Pcl_Component.Executor) is
  begin
    if Length(This.Peer_Id) = 0 then
      Raise_Error("udp transport peer id has not been set");
    end if;

    Register_Peer(Exec, To_String(This.Peer_Id), Transport(This));
  end Register;

  procedure Register
    (This    : in out Udp_Transport;
     Exec    : in out Pcl_Component.Executor;
     Peer_Id : String) is
  begin
    Set_Peer_Id(This, Peer_Id);
    Register(This, Exec);
  end Register;

  overriding procedure Finalize(This : in out Udp_Transport) is
  begin
    if This.Handle /= null then
      Pcl_Bindings.Destroy_Udp_Transport(This.Handle);
      This.Handle := null;
    end if;
  end Finalize;

  procedure Create
    (This           : in out Shared_Memory_Transport;
     Exec           : in out Pcl_Component.Executor;
     Bus_Name       : String;
     Participant_Id : String) is
    Bus_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Bus_Name);
    Part_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Participant_Id);
  begin
    if This.Handle /= null then
      Raise_Error("shared-memory transport already created");
    end if;

    This.Handle := Pcl_Bindings.Create_Shared_Memory_Transport
      (Bus_Name       => Bus_C,
       Participant_Id => Part_C,
       Executor       => Pcl_Component.Handle(Exec));
    Interfaces.C.Strings.Free(Bus_C);
    Interfaces.C.Strings.Free(Part_C);

    if This.Handle = null then
      Raise_Error("failed to create shared-memory transport");
    end if;

    This.Participant_Id := To_Unbounded_String(Participant_Id);
  end Create;

  function Handle
    (This : Shared_Memory_Transport)
      return Pcl_Bindings.Pcl_Shared_Memory_Transport_Access is
  begin
    return This.Handle;
  end Handle;

  function Transport
    (This : Shared_Memory_Transport)
      return Pcl_Bindings.Pcl_Transport_Const_Access is
  begin
    return Pcl_Bindings.Get_Shared_Memory_Transport(This.Handle);
  end Transport;

  function Gateway_Container
    (This : Shared_Memory_Transport) return Pcl_Bindings.Pcl_Container_Access is
  begin
    return Pcl_Bindings.Shared_Memory_Gateway_Container(This.Handle);
  end Gateway_Container;

  procedure Use_As_Default
    (This : in out Shared_Memory_Transport;
     Exec : in out Pcl_Component.Executor) is
  begin
    if This.Handle = null then
      Raise_Error("shared-memory transport has not been created");
    end if;

    Pcl_Component.Set_Transport(Exec, Transport(This));
  end Use_As_Default;

  procedure Register
    (This : in out Shared_Memory_Transport;
     Exec : in out Pcl_Component.Executor) is
  begin
    if Length(This.Participant_Id) = 0 then
      Raise_Error("shared-memory participant id has not been set");
    end if;

    Register_Peer(Exec, To_String(This.Participant_Id), Transport(This));
  end Register;

  procedure Register
    (This    : in out Shared_Memory_Transport;
     Exec    : in out Pcl_Component.Executor;
     Peer_Id : String) is
  begin
    Register_Peer(Exec, Peer_Id, Transport(This));
  end Register;

  procedure Start_Gateway
    (This : in out Shared_Memory_Transport;
     Exec : in out Pcl_Component.Executor) is
  begin
    Start_Gateway_Container(Exec, Gateway_Container(This), "shared-memory transport");
  end Start_Gateway;

  overriding procedure Finalize(This : in out Shared_Memory_Transport) is
  begin
    if This.Handle /= null then
      Pcl_Bindings.Destroy_Shared_Memory_Transport(This.Handle);
      This.Handle := null;
    end if;
  end Finalize;
end Pcl_Transports;
