with Ada.Finalization;
with Interfaces.C;
with Interfaces.C.Strings;
with Pcl_Bindings;
with System;

package Pcl_Component is
  Pcl_Error : exception;

  type Message_View is private;

  function Data_Address(Message : Message_View) return System.Address;
  function Size_Bytes(Message : Message_View) return Interfaces.C.unsigned;
  function Type_Name(Message : Message_View) return String;

  type Component is abstract new Ada.Finalization.Limited_Controlled with private;

  procedure Create(This : in out Component'Class; Name : String);
  procedure Configure(This : in out Component'Class);
  procedure Activate(This : in out Component'Class);
  procedure Set_Tick_Rate_Hz(This : in out Component'Class;
                             Hz   : Interfaces.C.double);
  procedure Subscribe(This      : in out Component'Class;
                      Topic     : String;
                      Type_Name : String);

  procedure On_Configure(This : in out Component) is null;
  procedure On_Activate(This : in out Component) is null;
  procedure On_Deactivate(This : in out Component) is null;
  procedure On_Cleanup(This : in out Component) is null;
  procedure On_Shutdown(This : in out Component) is null;
  procedure On_Tick(This : in out Component;
                    Dt_Seconds : Interfaces.C.double) is null;
  procedure On_Message(This : in out Component;
                       Topic : String;
                       Message : Message_View) is null;

  type Executor is new Ada.Finalization.Limited_Controlled with private;

  procedure Create(This : in out Executor);
  procedure Add(This : in out Executor;
                Item : in out Component'Class);
  procedure Spin(This : in out Executor);
  procedure Spin_Once(This : in out Executor;
                      Timeout_Ms : Interfaces.C.unsigned := 0);
  procedure Shutdown_Graceful(This : in out Executor;
                              Timeout_Ms : Interfaces.C.unsigned := 5_000);
  procedure Request_Shutdown(This : in out Executor);
  procedure Post_Incoming(This : in out Executor;
                          Topic : String;
                          Type_Name : String;
                          Data : System.Address;
                          Size : Interfaces.C.unsigned);

private
  Max_Subscriptions : constant Positive := 16;

  type Subscription_Context is record
    In_Use : Boolean := False;
    Owner_Address : System.Address := System.Null_Address;
    Topic : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
    Type_Name : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
  end record;

  type Subscription_Array is
    array (Positive range 1 .. Max_Subscriptions) of aliased Subscription_Context;

  type Message_View is record
    Data : System.Address := System.Null_Address;
    Size : Interfaces.C.unsigned := 0;
    Raw_Type_Name : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
  end record;

  type Component is abstract new Ada.Finalization.Limited_Controlled with record
    Handle : Pcl_Bindings.Pcl_Container_Access := null;
    Name : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
    Callbacks : aliased Pcl_Bindings.Pcl_Callbacks :=
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
