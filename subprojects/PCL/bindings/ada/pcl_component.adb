with Ada.Containers;
with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Unchecked_Conversion;
with Interfaces.C.Strings;
with System;

package body Pcl_Component is
  use type Ada.Containers.Count_Type;
  use type Pcl_Bindings.C_Bool;
  use type Interfaces.C.unsigned;
  use type Interfaces.C.Strings.chars_ptr;
  use type Pcl_Bindings.Pcl_Container_Access;
  use type Pcl_Bindings.Pcl_Endpoint_Kind;
  use type Pcl_Bindings.Pcl_Executor_Access;
  use type Pcl_Bindings.Pcl_Port_Access;
  use type Pcl_Bindings.Pcl_Status;

  type Component_Class_Access is access all Component'Class;
  type Subscription_Context_Access is access all Subscription_Context;

  function To_Component is new Ada.Unchecked_Conversion
    (Source => System.Address,
     Target => Component_Class_Access);

  function To_Subscription is new Ada.Unchecked_Conversion
    (Source => System.Address,
     Target => Subscription_Context_Access);

  function Configure_Trampoline
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Configure_Trampoline);

  function Activate_Trampoline
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Activate_Trampoline);

  function Deactivate_Trampoline
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Deactivate_Trampoline);

  function Cleanup_Trampoline
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Cleanup_Trampoline);

  function Shutdown_Trampoline
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Shutdown_Trampoline);

  function Tick_Trampoline
    (Container  : Pcl_Bindings.Pcl_Container_Access;
     Dt_Seconds : Interfaces.C.double;
     User_Data  : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Tick_Trampoline);

  procedure Message_Trampoline
    (Container : Pcl_Bindings.Pcl_Container_Access;
     Msg       : access constant Pcl_Bindings.Pcl_Msg;
     User_Data : System.Address);
  pragma Convention(C, Message_Trampoline);

  type C_String_Array is
    array (Positive range <>) of aliased Interfaces.C.Strings.chars_ptr;

  procedure Raise_Error(Message : String) is
  begin
    raise Pcl_Error with Message;
  end Raise_Error;

  function To_C_Bool(Value : Boolean) return Pcl_Bindings.C_Bool is
  begin
    return Pcl_Bindings.C_Bool(Value);
  end To_C_Bool;

  function To_Boolean(Value : Pcl_Bindings.C_Bool) return Boolean is
  begin
    return Boolean(Value);
  end To_Boolean;

  procedure Check(Status : Pcl_Bindings.Pcl_Status;
                  Context : String) is
  begin
    if Status /= Pcl_Bindings.PCL_OK then
      Raise_Error(Context & " failed with status" &
                    Interfaces.C.int'Image(Interfaces.C.int(Status)));
    end if;
  end Check;

  procedure Ensure_Created(This : Component'Class) is
  begin
    if This.Handle = null then
      Raise_Error("component has not been created");
    end if;
  end Ensure_Created;

  procedure Ensure_Created(This : Executor) is
  begin
    if This.Handle = null then
      Raise_Error("executor has not been created");
    end if;
  end Ensure_Created;

  procedure Ensure_Valid(This : Port) is
  begin
    if This.Handle = null then
      Raise_Error("port has not been created");
    end if;
  end Ensure_Valid;

  function Single_Peer(Peer_Id : String) return Peer_Id_Vectors.Vector is
    Result : Peer_Id_Vectors.Vector;
  begin
    Result.Append(Peer_Id);
    return Result;
  end Single_Peer;

  procedure Free_Strings(Values : in out C_String_Array) is
  begin
    for Index in Values'Range loop
      if Values(Index) /= Interfaces.C.Strings.Null_Ptr then
        Interfaces.C.Strings.Free(Values(Index));
        Values(Index) := Interfaces.C.Strings.Null_Ptr;
      end if;
    end loop;
  end Free_Strings;

  procedure Apply_Port_Route
    (Handle     : Pcl_Bindings.Pcl_Port_Access;
     Route_Mode : Interfaces.C.unsigned;
     Peer_Ids   : Peer_Id_Vectors.Vector) is
  begin
    if Handle = null then
      Raise_Error("port has not been created");
    end if;

    if Peer_Ids.Length = 0 then
      Check(Pcl_Bindings.Port_Set_Route
              (Port       => Handle,
               Route_Mode => Route_Mode,
               Peer_Ids   => System.Null_Address,
               Peer_Count => 0),
            "set port route");
      return;
    end if;

    declare
      Values : C_String_Array(1 .. Natural(Peer_Ids.Length));
    begin
      for Index in Values'Range loop
        Values(Index) := Interfaces.C.Strings.New_String
          (Peer_Ids.Element(Positive(Index)));
      end loop;

      Check(Pcl_Bindings.Port_Set_Route
              (Port       => Handle,
               Route_Mode => Route_Mode,
               Peer_Ids   => Values(Values'First)'Address,
               Peer_Count => Interfaces.C.unsigned(Values'Length)),
            "set port route");

      Free_Strings(Values);
    exception
      when others =>
        Free_Strings(Values);
        raise;
    end;
  end Apply_Port_Route;

  procedure Apply_Endpoint_Route
    (Exec          : Pcl_Bindings.Pcl_Executor_Access;
     Endpoint_Name : String;
     Kind          : Pcl_Bindings.Pcl_Endpoint_Kind;
     Route_Mode    : Interfaces.C.unsigned;
     Peer_Ids      : Peer_Id_Vectors.Vector) is
    Name_C : Interfaces.C.Strings.chars_ptr :=
      Interfaces.C.Strings.New_String(Endpoint_Name);
  begin
    if Exec = null then
      Interfaces.C.Strings.Free(Name_C);
      Raise_Error("executor has not been created");
    end if;

    if Kind = Pcl_Bindings.PCL_ENDPOINT_CONSUMED
      and then Route_Mode = Pcl_Bindings.PCL_ROUTE_LOCAL + Pcl_Bindings.PCL_ROUTE_REMOTE
    then
      Interfaces.C.Strings.Free(Name_C);
      Raise_Error("consumed endpoints must choose local or remote, not both");
    end if;

    if Peer_Ids.Length = 0 then
      declare
        Route : aliased Pcl_Bindings.Pcl_Endpoint_Route :=
          (Endpoint_Name => Name_C,
           Endpoint_Kind => Kind,
           Route_Mode    => Route_Mode,
           Peer_Ids      => System.Null_Address,
           Peer_Count    => 0);
      begin
        Check(Pcl_Bindings.Set_Endpoint_Route(Exec, Route'Access),
              "set endpoint route");
      end;
      Interfaces.C.Strings.Free(Name_C);
      return;
    end if;

    declare
      Values : C_String_Array(1 .. Natural(Peer_Ids.Length));
      Route  : aliased Pcl_Bindings.Pcl_Endpoint_Route;
    begin
      for Index in Values'Range loop
        Values(Index) := Interfaces.C.Strings.New_String
          (Peer_Ids.Element(Positive(Index)));
      end loop;

      Route :=
        (Endpoint_Name => Name_C,
         Endpoint_Kind => Kind,
         Route_Mode    => Route_Mode,
         Peer_Ids      => Values(Values'First)'Address,
         Peer_Count    => Interfaces.C.unsigned(Values'Length));

      Check(Pcl_Bindings.Set_Endpoint_Route(Exec, Route'Access),
            "set endpoint route");

      Free_Strings(Values);
    exception
      when others =>
        Free_Strings(Values);
        Interfaces.C.Strings.Free(Name_C);
        raise;
    end;

    Interfaces.C.Strings.Free(Name_C);
  end Apply_Endpoint_Route;

  function Make_Port
    (Handle     : Pcl_Bindings.Pcl_Port_Access;
     Type_Name  : String) return Port is
  begin
    return (Handle                => Handle,
            Default_Port_Type_Name => To_Unbounded_String(Type_Name));
  end Make_Port;

  function Data_Address(Message : Message_View) return System.Address is
  begin
    return Message.Data;
  end Data_Address;

  function Size_Bytes(Message : Message_View) return Interfaces.C.unsigned is
  begin
    return Message.Size;
  end Size_Bytes;

  function Type_Name(Message : Message_View) return String is
  begin
    if Message.Raw_Type_Name = Interfaces.C.Strings.Null_Ptr then
      return "";
    end if;

    return Interfaces.C.Strings.Value(Message.Raw_Type_Name);
  end Type_Name;

  function To_Raw_Message(Message : Message_View) return Pcl_Bindings.Pcl_Msg is
  begin
    return (Data      => Message.Data,
            Size      => Message.Size,
            Type_Name => Message.Raw_Type_Name);
  end To_Raw_Message;

  function Is_Valid(This : Port) return Boolean is
  begin
    return This.Handle /= null;
  end Is_Valid;

  function Default_Type_Name(This : Port) return String is
  begin
    return To_String(This.Default_Port_Type_Name);
  end Default_Type_Name;

  procedure Route_Local(This : in out Port) is
    Empty : Peer_Id_Vectors.Vector;
  begin
    Apply_Port_Route(This.Handle, Pcl_Bindings.PCL_ROUTE_LOCAL, Empty);
  end Route_Local;

  procedure Route_Remote(This : in out Port; Peer_Id : String) is
  begin
    Apply_Port_Route(This.Handle, Pcl_Bindings.PCL_ROUTE_REMOTE,
                     Single_Peer(Peer_Id));
  end Route_Remote;

  procedure Route_Remote(This : in out Port; Peer_Ids : Peer_Id_Vectors.Vector) is
  begin
    if Peer_Ids.Length = 0 then
      Raise_Error("remote route requires at least one peer id");
    end if;

    Apply_Port_Route(This.Handle, Pcl_Bindings.PCL_ROUTE_REMOTE, Peer_Ids);
  end Route_Remote;

  procedure Route_Local_And_Remote(This : in out Port; Peer_Id : String) is
  begin
    Apply_Port_Route(This.Handle,
                     Pcl_Bindings.PCL_ROUTE_LOCAL + Pcl_Bindings.PCL_ROUTE_REMOTE,
                     Single_Peer(Peer_Id));
  end Route_Local_And_Remote;

  procedure Route_Local_And_Remote
    (This     : in out Port;
     Peer_Ids : Peer_Id_Vectors.Vector) is
  begin
    if Peer_Ids.Length = 0 then
      Raise_Error("local+remote route requires at least one peer id");
    end if;

    Apply_Port_Route(This.Handle,
                     Pcl_Bindings.PCL_ROUTE_LOCAL + Pcl_Bindings.PCL_ROUTE_REMOTE,
                     Peer_Ids);
  end Route_Local_And_Remote;

  procedure Publish(This : Port; Msg : Pcl_Bindings.Pcl_Msg) is
    Aliased_Msg : aliased constant Pcl_Bindings.Pcl_Msg := Msg;
  begin
    Ensure_Valid(This);
    Check(Pcl_Bindings.Port_Publish(This.Handle, Aliased_Msg'Access),
          "publish");
  end Publish;

  procedure Publish
    (This      : Port;
     Data      : System.Address;
     Size      : Interfaces.C.unsigned;
     Type_Name : String := "") is
    Effective_Type : constant String :=
      (if Type_Name'Length = 0 then Default_Type_Name(This) else Type_Name);
    Type_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
    Msg    : aliased Pcl_Bindings.Pcl_Msg;
  begin
    Ensure_Valid(This);

    if Effective_Type'Length > 0 then
      Type_C := Interfaces.C.Strings.New_String(Effective_Type);
    end if;

    Msg := (Data      => Data,
            Size      => Size,
            Type_Name => Type_C);

    Check(Pcl_Bindings.Port_Publish(This.Handle, Msg'Access), "publish");

    if Type_C /= Interfaces.C.Strings.Null_Ptr then
      Interfaces.C.Strings.Free(Type_C);
    end if;
  end Publish;

  procedure Publish
    (This      : Port;
     Payload   : String;
     Type_Name : String := "") is
    Bytes : aliased constant String := Payload;
  begin
    Publish
      (This      => This,
       Data      => (if Bytes'Length = 0
                     then System.Null_Address
                     else Bytes(Bytes'First)'Address),
       Size      => Interfaces.C.unsigned(Bytes'Length),
       Type_Name => Type_Name);
  end Publish;

  procedure Publish
    (This    : Port;
     Payload : String;
     Format  : Pcl_Content_Types.Content_Type_Kind) is
  begin
    Publish(This, Payload, Pcl_Content_Types.Image(Format));
  end Publish;

  procedure Create(This : in out Component'Class; Name : String) is
  begin
    if This.Handle /= null then
      Raise_Error("component already created");
    end if;

    This.Raw_Name := Interfaces.C.Strings.New_String(Name);
    This.Callbacks :=
      (On_Configure  => Configure_Trampoline'Access,
       On_Activate   => Activate_Trampoline'Access,
       On_Deactivate => Deactivate_Trampoline'Access,
       On_Cleanup    => Cleanup_Trampoline'Access,
       On_Shutdown   => Shutdown_Trampoline'Access,
       On_Tick       => Tick_Trampoline'Access);

    This.Handle := Pcl_Bindings.Create_Container
      (Name      => This.Raw_Name,
       Callbacks => This.Callbacks'Access,
       User_Data => This'Address);

    if This.Handle = null then
      Interfaces.C.Strings.Free(This.Raw_Name);
      This.Raw_Name := Interfaces.C.Strings.Null_Ptr;
      Raise_Error("failed to create component");
    end if;
  end Create;

  function Handle(This : Component'Class) return Pcl_Bindings.Pcl_Container_Access is
  begin
    return This.Handle;
  end Handle;

  function Name(This : Component'Class) return String is
    Name_C : Interfaces.C.Strings.chars_ptr;
  begin
    Ensure_Created(This);
    Name_C := Pcl_Bindings.Container_Name(This.Handle);
    if Name_C = Interfaces.C.Strings.Null_Ptr then
      return "";
    end if;
    return Interfaces.C.Strings.Value(Name_C);
  end Name;

  function State(This : Component'Class) return Pcl_Bindings.Pcl_State is
  begin
    Ensure_Created(This);
    return Pcl_Bindings.Container_State(This.Handle);
  end State;

  procedure Configure(This : in out Component'Class) is
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Configure(This.Handle), "configure");
  end Configure;

  procedure Activate(This : in out Component'Class) is
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Activate(This.Handle), "activate");
  end Activate;

  procedure Deactivate(This : in out Component'Class) is
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Deactivate(This.Handle), "deactivate");
  end Deactivate;

  procedure Cleanup(This : in out Component'Class) is
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Cleanup(This.Handle), "cleanup");
  end Cleanup;

  procedure Shutdown(This : in out Component'Class) is
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Shutdown(This.Handle), "shutdown");
  end Shutdown;

  procedure Set_Tick_Rate_Hz
    (This : in out Component'Class;
     Hz   : Interfaces.C.double) is
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Set_Tick_Rate_Hz(This.Handle, Hz), "set tick rate");
  end Set_Tick_Rate_Hz;

  function Tick_Rate_Hz(This : Component'Class) return Interfaces.C.double is
  begin
    Ensure_Created(This);
    return Pcl_Bindings.Get_Tick_Rate_Hz(This.Handle);
  end Tick_Rate_Hz;

  procedure Set_Param(This : in out Component'Class; Key : String; Value : String) is
    Key_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Key);
    Value_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Value);
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Set_Param_Str(This.Handle, Key_C, Value_C), "set string param");
    Interfaces.C.Strings.Free(Key_C);
    Interfaces.C.Strings.Free(Value_C);
  end Set_Param;

  procedure Set_Param
    (This  : in out Component'Class;
     Key   : String;
     Value : Interfaces.C.double) is
    Key_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Key);
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Set_Param_F64(This.Handle, Key_C, Value), "set float param");
    Interfaces.C.Strings.Free(Key_C);
  end Set_Param;

  procedure Set_Param
    (This  : in out Component'Class;
     Key   : String;
     Value : Boolean) is
    Key_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Key);
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Set_Param_Bool(This.Handle, Key_C, To_C_Bool(Value)),
          "set bool param");
    Interfaces.C.Strings.Free(Key_C);
  end Set_Param;

  function Param_Str
    (This        : Component'Class;
     Key         : String;
     Default_Val : String := "") return String is
    Key_C     : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Key);
    Default_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
    Result_C  : Interfaces.C.Strings.chars_ptr;
  begin
    Ensure_Created(This);

    if Default_Val'Length > 0 then
      Default_C := Interfaces.C.Strings.New_String(Default_Val);
    end if;

    Result_C := Pcl_Bindings.Get_Param_Str(This.Handle, Key_C, Default_C);

    Interfaces.C.Strings.Free(Key_C);
    if Default_C /= Interfaces.C.Strings.Null_Ptr then
      Interfaces.C.Strings.Free(Default_C);
    end if;

    if Result_C = Interfaces.C.Strings.Null_Ptr then
      return "";
    end if;

    return Interfaces.C.Strings.Value(Result_C);
  end Param_Str;

  function Param_F64
    (This        : Component'Class;
     Key         : String;
     Default_Val : Interfaces.C.double := 0.0) return Interfaces.C.double is
    Key_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Key);
    Value : Interfaces.C.double;
  begin
    Ensure_Created(This);
    Value := Pcl_Bindings.Get_Param_F64(This.Handle, Key_C, Default_Val);
    Interfaces.C.Strings.Free(Key_C);
    return Value;
  end Param_F64;

  function Param_Bool
    (This        : Component'Class;
     Key         : String;
     Default_Val : Boolean := False) return Boolean is
    Key_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Key);
    Value : Boolean;
  begin
    Ensure_Created(This);
    Value := To_Boolean
      (Pcl_Bindings.Get_Param_Bool(This.Handle, Key_C, To_C_Bool(Default_Val)));
    Interfaces.C.Strings.Free(Key_C);
    return Value;
  end Param_Bool;

  function Add_Publisher
    (This      : in out Component'Class;
     Topic     : String;
     Type_Name : String) return Port is
    Topic_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Topic);
    Type_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Type_Name);
    Handle  : Pcl_Bindings.Pcl_Port_Access;
  begin
    Ensure_Created(This);
    Handle := Pcl_Bindings.Add_Publisher(This.Handle, Topic_C, Type_C);
    Interfaces.C.Strings.Free(Topic_C);
    Interfaces.C.Strings.Free(Type_C);

    if Handle = null then
      Raise_Error("publisher must be created during on_configure");
    end if;

    return Make_Port(Handle, Type_Name);
  end Add_Publisher;

  function Add_Subscriber
    (This      : in out Component'Class;
     Topic     : String;
     Type_Name : String;
     Callback  : Pcl_Bindings.Pcl_Sub_Callback_Access;
     User_Data : System.Address := System.Null_Address) return Port is
    Topic_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Topic);
    Type_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Type_Name);
    Handle  : Pcl_Bindings.Pcl_Port_Access;
  begin
    Ensure_Created(This);
    Handle := Pcl_Bindings.Add_Subscriber
      (Container => This.Handle,
       Topic     => Topic_C,
       Type_Name => Type_C,
       Callback  => Callback,
       User_Data => User_Data);
    Interfaces.C.Strings.Free(Topic_C);
    Interfaces.C.Strings.Free(Type_C);

    if Handle = null then
      Raise_Error("subscriber must be created during on_configure");
    end if;

    return Make_Port(Handle, Type_Name);
  end Add_Subscriber;

  function Add_Service
    (This         : in out Component'Class;
     Service_Name : String;
     Type_Name    : String;
     Handler      : Pcl_Bindings.Pcl_Service_Handler_Access;
     User_Data    : System.Address := System.Null_Address) return Port is
    Service_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Service_Name);
    Type_C    : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Type_Name);
    Handle    : Pcl_Bindings.Pcl_Port_Access;
  begin
    Ensure_Created(This);
    Handle := Pcl_Bindings.Add_Service
      (Container    => This.Handle,
       Service_Name => Service_C,
       Type_Name    => Type_C,
       Handler      => Handler,
       User_Data    => User_Data);
    Interfaces.C.Strings.Free(Service_C);
    Interfaces.C.Strings.Free(Type_C);

    if Handle = null then
      Raise_Error("service must be created during on_configure");
    end if;

    return Make_Port(Handle, Type_Name);
  end Add_Service;

  function Add_Stream_Service
    (This         : in out Component'Class;
     Service_Name : String;
     Type_Name    : String;
     Handler      : Pcl_Bindings.Pcl_Stream_Handler_Access;
     User_Data    : System.Address := System.Null_Address) return Port is
    Service_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Service_Name);
    Type_C    : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Type_Name);
    Handle    : Pcl_Bindings.Pcl_Port_Access;
  begin
    Ensure_Created(This);
    Handle := Pcl_Bindings.Add_Stream_Service
      (Container    => This.Handle,
       Service_Name => Service_C,
       Type_Name    => Type_C,
       Handler      => Handler,
       User_Data    => User_Data);
    Interfaces.C.Strings.Free(Service_C);
    Interfaces.C.Strings.Free(Type_C);

    if Handle = null then
      Raise_Error("stream service must be created during on_configure");
    end if;

    return Make_Port(Handle, Type_Name);
  end Add_Stream_Service;

  procedure Subscribe
    (This      : in out Component'Class;
     Topic     : String;
     Type_Name : String) is
    Slot : Subscription_Context_Access := null;
    Port_Handle : Pcl_Bindings.Pcl_Port_Access;
  begin
    Ensure_Created(This);

    for Index in This.Subscriptions'Range loop
      if not This.Subscriptions(Index).In_Use then
        Slot := This.Subscriptions(Index)'Unchecked_Access;
        exit;
      end if;
    end loop;

    if Slot = null then
      Raise_Error("subscription capacity exceeded");
    end if;

    Slot.In_Use := True;
    Slot.Owner_Address := This'Address;
    Slot.Topic := Interfaces.C.Strings.New_String(Topic);
    Slot.Type_Name := Interfaces.C.Strings.New_String(Type_Name);

    Port_Handle := Pcl_Bindings.Add_Subscriber
      (Container => This.Handle,
       Topic     => Slot.Topic,
       Type_Name => Slot.Type_Name,
       Callback  => Message_Trampoline'Access,
       User_Data => Slot.all'Address);

    if Port_Handle = null then
      Interfaces.C.Strings.Free(Slot.Topic);
      Interfaces.C.Strings.Free(Slot.Type_Name);
      Slot.In_Use := False;
      Slot.Owner_Address := System.Null_Address;
      Slot.Topic := Interfaces.C.Strings.Null_Ptr;
      Slot.Type_Name := Interfaces.C.Strings.Null_Ptr;
      Raise_Error("subscriber must be created during on_configure");
    end if;
  end Subscribe;

  overriding procedure Finalize(This : in out Component) is
  begin
    if This.Handle /= null then
      Pcl_Bindings.Destroy_Container(This.Handle);
      This.Handle := null;
    end if;

    if This.Raw_Name /= Interfaces.C.Strings.Null_Ptr then
      Interfaces.C.Strings.Free(This.Raw_Name);
      This.Raw_Name := Interfaces.C.Strings.Null_Ptr;
    end if;

    for Index in This.Subscriptions'Range loop
      if This.Subscriptions(Index).Topic /= Interfaces.C.Strings.Null_Ptr then
        Interfaces.C.Strings.Free(This.Subscriptions(Index).Topic);
        This.Subscriptions(Index).Topic := Interfaces.C.Strings.Null_Ptr;
      end if;

      if This.Subscriptions(Index).Type_Name /= Interfaces.C.Strings.Null_Ptr then
        Interfaces.C.Strings.Free(This.Subscriptions(Index).Type_Name);
        This.Subscriptions(Index).Type_Name := Interfaces.C.Strings.Null_Ptr;
      end if;

      This.Subscriptions(Index).In_Use := False;
      This.Subscriptions(Index).Owner_Address := System.Null_Address;
    end loop;
  end Finalize;

  procedure Create(This : in out Executor) is
  begin
    if This.Handle /= null then
      Raise_Error("executor already created");
    end if;

    This.Handle := Pcl_Bindings.Create_Executor;
    if This.Handle = null then
      Raise_Error("failed to create executor");
    end if;
  end Create;

  function Handle(This : Executor) return Pcl_Bindings.Pcl_Executor_Access is
  begin
    return This.Handle;
  end Handle;

  procedure Add(This : in out Executor; Item : in out Component'Class) is
  begin
    Ensure_Created(This);
    Ensure_Created(Item);
    Check(Pcl_Bindings.Add_Container(This.Handle, Item.Handle), "add container");
  end Add;

  procedure Add(This : in out Executor; Container : Pcl_Bindings.Pcl_Container_Access) is
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Add_Container(This.Handle, Container), "add container");
  end Add;

  procedure Remove(This : in out Executor; Item : in out Component'Class) is
  begin
    Ensure_Created(This);
    Ensure_Created(Item);
    Check(Pcl_Bindings.Remove_Container(This.Handle, Item.Handle), "remove container");
  end Remove;

  procedure Remove(This : in out Executor; Container : Pcl_Bindings.Pcl_Container_Access) is
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Remove_Container(This.Handle, Container), "remove container");
  end Remove;

  procedure Spin(This : in out Executor) is
  begin
    Check(Spin_Status(This), "spin");
  end Spin;

  function Spin_Status(This : in out Executor) return Pcl_Bindings.Pcl_Status is
  begin
    Ensure_Created(This);
    return Pcl_Bindings.Spin(This.Handle);
  end Spin_Status;

  procedure Spin_Once
    (This       : in out Executor;
     Timeout_Ms : Interfaces.C.unsigned := 0) is
  begin
    Check(Spin_Once_Status(This, Timeout_Ms), "spin once");
  end Spin_Once;

  function Spin_Once_Status
    (This       : in out Executor;
     Timeout_Ms : Interfaces.C.unsigned := 0) return Pcl_Bindings.Pcl_Status is
  begin
    Ensure_Created(This);
    return Pcl_Bindings.Spin_Once(This.Handle, Timeout_Ms);
  end Spin_Once_Status;

  procedure Shutdown_Graceful
    (This       : in out Executor;
     Timeout_Ms : Interfaces.C.unsigned := 5_000) is
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Shutdown_Graceful(This.Handle, Timeout_Ms),
          "graceful shutdown");
  end Shutdown_Graceful;

  procedure Request_Shutdown(This : in out Executor) is
  begin
    Ensure_Created(This);
    Pcl_Bindings.Request_Shutdown(This.Handle);
  end Request_Shutdown;

  procedure Set_Transport
    (This      : in out Executor;
     Transport : Pcl_Bindings.Pcl_Transport_Const_Access) is
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Set_Transport(This.Handle, Transport), "set transport");
  end Set_Transport;

  procedure Register_Transport
    (This      : in out Executor;
     Peer_Id   : String;
     Transport : Pcl_Bindings.Pcl_Transport_Const_Access) is
    Peer_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Peer_Id);
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Register_Transport(This.Handle, Peer_C, Transport),
          "register transport");
    Interfaces.C.Strings.Free(Peer_C);
  end Register_Transport;

  procedure Route_Local
    (This          : in out Executor;
     Endpoint_Name : String;
     Kind          : Pcl_Bindings.Pcl_Endpoint_Kind :=
       Pcl_Bindings.PCL_ENDPOINT_CONSUMED) is
    Empty : Peer_Id_Vectors.Vector;
  begin
    Ensure_Created(This);
    Apply_Endpoint_Route(This.Handle, Endpoint_Name, Kind,
                         Pcl_Bindings.PCL_ROUTE_LOCAL, Empty);
  end Route_Local;

  procedure Route_Remote
    (This          : in out Executor;
     Endpoint_Name : String;
     Peer_Id       : String;
     Kind          : Pcl_Bindings.Pcl_Endpoint_Kind :=
       Pcl_Bindings.PCL_ENDPOINT_CONSUMED) is
  begin
    Ensure_Created(This);
    Apply_Endpoint_Route(This.Handle, Endpoint_Name, Kind,
                         Pcl_Bindings.PCL_ROUTE_REMOTE,
                         Single_Peer(Peer_Id));
  end Route_Remote;

  procedure Route_Remote
    (This          : in out Executor;
     Endpoint_Name : String;
     Peer_Ids      : Peer_Id_Vectors.Vector;
     Kind          : Pcl_Bindings.Pcl_Endpoint_Kind :=
       Pcl_Bindings.PCL_ENDPOINT_CONSUMED) is
  begin
    Ensure_Created(This);
    if Peer_Ids.Length = 0 then
      Raise_Error("remote route requires at least one peer id");
    end if;

    Apply_Endpoint_Route(This.Handle, Endpoint_Name, Kind,
                         Pcl_Bindings.PCL_ROUTE_REMOTE, Peer_Ids);
  end Route_Remote;

  procedure Route_Local_And_Remote
    (This          : in out Executor;
     Endpoint_Name : String;
     Peer_Id       : String;
     Kind          : Pcl_Bindings.Pcl_Endpoint_Kind) is
  begin
    Ensure_Created(This);
    Apply_Endpoint_Route(This.Handle, Endpoint_Name, Kind,
                         Pcl_Bindings.PCL_ROUTE_LOCAL + Pcl_Bindings.PCL_ROUTE_REMOTE,
                         Single_Peer(Peer_Id));
  end Route_Local_And_Remote;

  procedure Route_Local_And_Remote
    (This          : in out Executor;
     Endpoint_Name : String;
     Peer_Ids      : Peer_Id_Vectors.Vector;
     Kind          : Pcl_Bindings.Pcl_Endpoint_Kind) is
  begin
    Ensure_Created(This);
    if Peer_Ids.Length = 0 then
      Raise_Error("local+remote route requires at least one peer id");
    end if;

    Apply_Endpoint_Route(This.Handle, Endpoint_Name, Kind,
                         Pcl_Bindings.PCL_ROUTE_LOCAL + Pcl_Bindings.PCL_ROUTE_REMOTE,
                         Peer_Ids);
  end Route_Local_And_Remote;

  procedure Post_Incoming
    (This      : in out Executor;
     Topic     : String;
     Type_Name : String;
     Data      : System.Address;
     Size      : Interfaces.C.unsigned) is
    Topic_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Topic);
    Type_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Type_Name);
    Msg     : aliased Pcl_Bindings.Pcl_Msg :=
      (Data      => Data,
       Size      => Size,
       Type_Name => Type_C);
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Post_Incoming(This.Handle, Topic_C, Msg'Access),
          "post incoming");
    Interfaces.C.Strings.Free(Topic_C);
    Interfaces.C.Strings.Free(Type_C);
  end Post_Incoming;

  procedure Post_Incoming
    (This    : in out Executor;
     Topic   : String;
     Payload : String;
     Format  : Pcl_Content_Types.Content_Type_Kind) is
    Bytes : aliased constant String := Payload;
  begin
    Post_Incoming
      (This      => This,
       Topic     => Topic,
       Type_Name => Pcl_Content_Types.Image(Format),
       Data      => (if Bytes'Length = 0
                     then System.Null_Address
                     else Bytes(Bytes'First)'Address),
       Size      => Interfaces.C.unsigned(Bytes'Length));
  end Post_Incoming;

  procedure Post_Remote_Incoming
    (This      : in out Executor;
     Peer_Id   : String;
     Topic     : String;
     Type_Name : String;
     Data      : System.Address;
     Size      : Interfaces.C.unsigned) is
    Peer_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Peer_Id);
    Topic_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Topic);
    Type_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Type_Name);
    Msg     : aliased Pcl_Bindings.Pcl_Msg :=
      (Data      => Data,
       Size      => Size,
       Type_Name => Type_C);
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Post_Remote_Incoming(This.Handle, Peer_C, Topic_C, Msg'Access),
          "post remote incoming");
    Interfaces.C.Strings.Free(Peer_C);
    Interfaces.C.Strings.Free(Topic_C);
    Interfaces.C.Strings.Free(Type_C);
  end Post_Remote_Incoming;

  procedure Post_Remote_Incoming
    (This    : in out Executor;
     Peer_Id : String;
     Topic   : String;
     Payload : String;
     Format  : Pcl_Content_Types.Content_Type_Kind) is
    Bytes : aliased constant String := Payload;
  begin
    Post_Remote_Incoming
      (This      => This,
       Peer_Id   => Peer_Id,
       Topic     => Topic,
       Type_Name => Pcl_Content_Types.Image(Format),
       Data      => (if Bytes'Length = 0
                     then System.Null_Address
                     else Bytes(Bytes'First)'Address),
       Size      => Interfaces.C.unsigned(Bytes'Length));
  end Post_Remote_Incoming;

  procedure Publish
    (This      : in out Executor;
     Topic     : String;
     Type_Name : String;
     Data      : System.Address;
     Size      : Interfaces.C.unsigned) is
    Topic_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Topic);
    Type_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Type_Name);
    Msg     : aliased Pcl_Bindings.Pcl_Msg :=
      (Data      => Data,
       Size      => Size,
       Type_Name => Type_C);
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Publish(This.Handle, Topic_C, Msg'Access),
          "publish");
    Interfaces.C.Strings.Free(Topic_C);
    Interfaces.C.Strings.Free(Type_C);
  end Publish;

  procedure Publish
    (This    : in out Executor;
     Topic   : String;
     Payload : String;
     Format  : Pcl_Content_Types.Content_Type_Kind) is
    Bytes : aliased constant String := Payload;
  begin
    Publish
      (This      => This,
       Topic     => Topic,
       Type_Name => Pcl_Content_Types.Image(Format),
       Data      => (if Bytes'Length = 0
                     then System.Null_Address
                     else Bytes(Bytes'First)'Address),
       Size      => Interfaces.C.unsigned(Bytes'Length));
  end Publish;

  overriding procedure Finalize(This : in out Executor) is
  begin
    if This.Handle /= null then
      Pcl_Bindings.Destroy_Executor(This.Handle);
      This.Handle := null;
    end if;
  end Finalize;

  function Configure_Trampoline
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status is
    pragma Unreferenced(Container);
    This : constant Component_Class_Access := To_Component(User_Data);
  begin
    This.On_Configure;
    return Pcl_Bindings.PCL_OK;
  exception
    when others =>
      return Pcl_Bindings.PCL_ERR_CALLBACK;
  end Configure_Trampoline;

  function Activate_Trampoline
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status is
    pragma Unreferenced(Container);
    This : constant Component_Class_Access := To_Component(User_Data);
  begin
    This.On_Activate;
    return Pcl_Bindings.PCL_OK;
  exception
    when others =>
      return Pcl_Bindings.PCL_ERR_CALLBACK;
  end Activate_Trampoline;

  function Deactivate_Trampoline
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status is
    pragma Unreferenced(Container);
    This : constant Component_Class_Access := To_Component(User_Data);
  begin
    This.On_Deactivate;
    return Pcl_Bindings.PCL_OK;
  exception
    when others =>
      return Pcl_Bindings.PCL_ERR_CALLBACK;
  end Deactivate_Trampoline;

  function Cleanup_Trampoline
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status is
    pragma Unreferenced(Container);
    This : constant Component_Class_Access := To_Component(User_Data);
  begin
    This.On_Cleanup;
    return Pcl_Bindings.PCL_OK;
  exception
    when others =>
      return Pcl_Bindings.PCL_ERR_CALLBACK;
  end Cleanup_Trampoline;

  function Shutdown_Trampoline
    (Container : Pcl_Bindings.Pcl_Container_Access;
     User_Data : System.Address) return Pcl_Bindings.Pcl_Status is
    pragma Unreferenced(Container);
    This : constant Component_Class_Access := To_Component(User_Data);
  begin
    This.On_Shutdown;
    return Pcl_Bindings.PCL_OK;
  exception
    when others =>
      return Pcl_Bindings.PCL_ERR_CALLBACK;
  end Shutdown_Trampoline;

  function Tick_Trampoline
    (Container  : Pcl_Bindings.Pcl_Container_Access;
     Dt_Seconds : Interfaces.C.double;
     User_Data  : System.Address) return Pcl_Bindings.Pcl_Status is
    pragma Unreferenced(Container);
    This : constant Component_Class_Access := To_Component(User_Data);
  begin
    This.On_Tick(Dt_Seconds);
    return Pcl_Bindings.PCL_OK;
  exception
    when others =>
      return Pcl_Bindings.PCL_ERR_CALLBACK;
  end Tick_Trampoline;

  procedure Message_Trampoline
    (Container : Pcl_Bindings.Pcl_Container_Access;
     Msg       : access constant Pcl_Bindings.Pcl_Msg;
     User_Data : System.Address) is
    pragma Unreferenced(Container);
    Slot    : constant Subscription_Context_Access := To_Subscription(User_Data);
    This    : constant Component_Class_Access := To_Component(Slot.Owner_Address);
    Message : constant Message_View :=
      (Data          => Msg.Data,
       Size          => Msg.Size,
       Raw_Type_Name => Msg.Type_Name);
  begin
    This.On_Message
      (Topic   => Interfaces.C.Strings.Value(Slot.Topic),
       Message => Message);
  exception
    when others =>
      null;
  end Message_Trampoline;
end Pcl_Component;
