with Ada.Unchecked_Conversion;
with Interfaces.C.Strings;
with Pcl_Bindings;
with System;

package body Pcl_Component is
  use type Interfaces.C.Strings.chars_ptr;
  use type Pcl_Bindings.Pcl_Container_Access;
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
    (Container  : Pcl_Bindings.Pcl_Container_Access;
     User_Data  : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Configure_Trampoline);

  function Activate_Trampoline
    (Container  : Pcl_Bindings.Pcl_Container_Access;
     User_Data  : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Activate_Trampoline);

  function Deactivate_Trampoline
    (Container  : Pcl_Bindings.Pcl_Container_Access;
     User_Data  : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Deactivate_Trampoline);

  function Cleanup_Trampoline
    (Container  : Pcl_Bindings.Pcl_Container_Access;
     User_Data  : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Cleanup_Trampoline);

  function Shutdown_Trampoline
    (Container  : Pcl_Bindings.Pcl_Container_Access;
     User_Data  : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Shutdown_Trampoline);

  function Tick_Trampoline
    (Container  : Pcl_Bindings.Pcl_Container_Access;
     Dt_Seconds : Interfaces.C.double;
     User_Data  : System.Address) return Pcl_Bindings.Pcl_Status;
  pragma Convention(C, Tick_Trampoline);

  procedure Message_Trampoline
    (Container  : Pcl_Bindings.Pcl_Container_Access;
     Msg        : access constant Pcl_Bindings.Pcl_Msg;
     User_Data  : System.Address);
  pragma Convention(C, Message_Trampoline);

  procedure Raise_Error(Message : String) is
  begin
    raise Pcl_Error with Message;
  end Raise_Error;

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

  procedure Create(This : in out Component'Class; Name : String) is
  begin
    if This.Handle /= null then
      Raise_Error("component already created");
    end if;

    This.Name := Interfaces.C.Strings.New_String(Name);
    This.Callbacks :=
      (On_Configure  => Configure_Trampoline'Access,
       On_Activate   => Activate_Trampoline'Access,
       On_Deactivate => Deactivate_Trampoline'Access,
       On_Cleanup    => Cleanup_Trampoline'Access,
       On_Shutdown   => Shutdown_Trampoline'Access,
       On_Tick       => Tick_Trampoline'Access);

    This.Handle := Pcl_Bindings.Create_Container(
      Name      => This.Name,
      Callbacks => This.Callbacks'Access,
      User_Data => This'Address);

    if This.Handle = null then
      Interfaces.C.Strings.Free(This.Name);
      This.Name := Interfaces.C.Strings.Null_Ptr;
      Raise_Error("failed to create component");
    end if;
  end Create;

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

  procedure Set_Tick_Rate_Hz(This : in out Component'Class;
                             Hz   : Interfaces.C.double) is
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Set_Tick_Rate_Hz(This.Handle, Hz), "set tick rate");
  end Set_Tick_Rate_Hz;

  procedure Subscribe(This      : in out Component'Class;
                      Topic     : String;
                      Type_Name : String) is
    Slot : Subscription_Context_Access := null;
    Port : Pcl_Bindings.Pcl_Port_Access;
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

    Port := Pcl_Bindings.Add_Subscriber(
      Container => This.Handle,
      Topic     => Slot.Topic,
      Type_Name => Slot.Type_Name,
      Callback  => Message_Trampoline'Access,
      User_Data => Slot.all'Address);

    if Port = null then
      Interfaces.C.Strings.Free(Slot.Topic);
      Interfaces.C.Strings.Free(Slot.Type_Name);
      Slot.In_Use := False;
      Slot.Owner_Address := System.Null_Address;
      Slot.Topic := Interfaces.C.Strings.Null_Ptr;
      Slot.Type_Name := Interfaces.C.Strings.Null_Ptr;
      Raise_Error("subscribe must be called during on_configure");
    end if;
  end Subscribe;

  overriding procedure Finalize(This : in out Component) is
  begin
    if This.Handle /= null then
      Pcl_Bindings.Destroy_Container(This.Handle);
      This.Handle := null;
    end if;

    if This.Name /= Interfaces.C.Strings.Null_Ptr then
      Interfaces.C.Strings.Free(This.Name);
      This.Name := Interfaces.C.Strings.Null_Ptr;
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

  procedure Add(This : in out Executor;
                Item : in out Component'Class) is
  begin
    Ensure_Created(This);
    Ensure_Created(Item);
    Check(Pcl_Bindings.Add_Container(This.Handle, Item.Handle), "add container");
  end Add;

  procedure Spin(This : in out Executor) is
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Spin(This.Handle), "spin");
  end Spin;

  procedure Spin_Once(This : in out Executor;
                      Timeout_Ms : Interfaces.C.unsigned := 0) is
  begin
    Ensure_Created(This);
    Check(Pcl_Bindings.Spin_Once(This.Handle, Timeout_Ms), "spin once");
  end Spin_Once;

  procedure Shutdown_Graceful(This : in out Executor;
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

  procedure Post_Incoming(This : in out Executor;
                          Topic : String;
                          Type_Name : String;
                          Data : System.Address;
                          Size : Interfaces.C.unsigned) is
    Topic_C : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Topic);
    Type_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String(Type_Name);
    Msg     : aliased Pcl_Bindings.Pcl_Msg :=
      (Data      => Data,
       Size      => Size,
       Type_Name => Type_C);
    Status  : Pcl_Bindings.Pcl_Status;
  begin
    Ensure_Created(This);

    Status := Pcl_Bindings.Post_Incoming(
      Exec  => This.Handle,
      Topic => Topic_C,
      Msg   => Msg'Access);

    Interfaces.C.Strings.Free(Topic_C);
    Interfaces.C.Strings.Free(Type_C);

    Check(Status, "post incoming");
  end Post_Incoming;

  overriding procedure Finalize(This : in out Executor) is
  begin
    if This.Handle /= null then
      Pcl_Bindings.Destroy_Executor(This.Handle);
      This.Handle := null;
    end if;
  end Finalize;

  function Configure_Trampoline
    (Container  : Pcl_Bindings.Pcl_Container_Access;
     User_Data  : System.Address) return Pcl_Bindings.Pcl_Status is
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
    (Container  : Pcl_Bindings.Pcl_Container_Access;
     User_Data  : System.Address) return Pcl_Bindings.Pcl_Status is
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
    (Container  : Pcl_Bindings.Pcl_Container_Access;
     User_Data  : System.Address) return Pcl_Bindings.Pcl_Status is
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
    (Container  : Pcl_Bindings.Pcl_Container_Access;
     User_Data  : System.Address) return Pcl_Bindings.Pcl_Status is
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
    (Container  : Pcl_Bindings.Pcl_Container_Access;
     User_Data  : System.Address) return Pcl_Bindings.Pcl_Status is
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
    (Container  : Pcl_Bindings.Pcl_Container_Access;
     Msg        : access constant Pcl_Bindings.Pcl_Msg;
     User_Data  : System.Address) is
    pragma Unreferenced(Container);
    Slot    : constant Subscription_Context_Access := To_Subscription(User_Data);
    This    : constant Component_Class_Access := To_Component(Slot.Owner_Address);
    Message : constant Message_View :=
      (Data          => Msg.Data,
       Size          => Msg.Size,
       Raw_Type_Name => Msg.Type_Name);
  begin
    This.On_Message(
      Topic   => Interfaces.C.Strings.Value(Slot.Topic),
      Message => Message);
  exception
    when others =>
      null;
  end Message_Trampoline;
end Pcl_Component;
