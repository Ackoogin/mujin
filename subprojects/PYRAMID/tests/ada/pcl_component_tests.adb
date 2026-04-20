with Ada.Text_IO;
with Interfaces.C;
with Pcl_Component;
with System;
with System.Address_To_Access_Conversions;

procedure Pcl_Component_Tests is
  use type Interfaces.C.int;

  procedure Log_Line(Message : String) is
  begin
    Ada.Text_IO.Put_Line(Ada.Text_IO.Standard_Error, Message);
    Ada.Text_IO.Flush(Ada.Text_IO.Standard_Error);
  end Log_Line;

  procedure Assert(Condition : Boolean; Message : String) is
  begin
    if not Condition then
      raise Program_Error with Message;
    end if;
  end Assert;

  package Int_Pointers is new System.Address_To_Access_Conversions(Interfaces.C.int);

  type Counting_Component is new Pcl_Component.Component with record
    Tick_Count : Natural := 0;
  end record;

  overriding procedure On_Tick(This : in out Counting_Component;
                               Dt_Seconds : Interfaces.C.double);

  procedure On_Tick(This : in out Counting_Component;
                    Dt_Seconds : Interfaces.C.double) is
    pragma Unreferenced(Dt_Seconds);
  begin
    This.Tick_Count := This.Tick_Count + 1;
  end On_Tick;

  type Subscriber_Component is new Pcl_Component.Component with record
    Updates_Received : Natural := 0;
    Last_Value : Interfaces.C.int := 0;
  end record;

  overriding procedure On_Configure(This : in out Subscriber_Component);
  overriding procedure On_Message(This : in out Subscriber_Component;
                                  Topic : String;
                                  Message : Pcl_Component.Message_View);

  procedure On_Configure(This : in out Subscriber_Component) is
  begin
    Pcl_Component.Subscribe(This, "sensor_updates", "SensorMsg");
  end On_Configure;

  procedure On_Message(This : in out Subscriber_Component;
                       Topic : String;
                       Message : Pcl_Component.Message_View) is
    Value : constant Int_Pointers.Object_Pointer :=
      Int_Pointers.To_Pointer(Pcl_Component.Data_Address(Message));
  begin
    Assert(Topic = "sensor_updates", "unexpected topic in message callback");
    Assert(Pcl_Component.Type_Name(Message) = "SensorMsg",
           "unexpected type name in message callback");

    This.Updates_Received := This.Updates_Received + 1;
    This.Last_Value := Value.all;
  end On_Message;

  procedure Test_Ticking is
    Exec : Pcl_Component.Executor;
    Comp : Counting_Component;
  begin
    Log_Line("test: ticking");

    Pcl_Component.Create(Exec);
    Pcl_Component.Create(Comp, "counting_component");
    Pcl_Component.Set_Tick_Rate_Hz(Comp, 100.0);
    Pcl_Component.Configure(Comp);
    Pcl_Component.Activate(Comp);
    Pcl_Component.Add(Exec, Comp);

    for Iteration in 1 .. 10 loop
      pragma Unreferenced(Iteration);
      delay 0.02;
      Pcl_Component.Spin_Once(Exec);
    end loop;

    Assert(Comp.Tick_Count > 0, "component did not tick");
    Pcl_Component.Shutdown_Graceful(Exec);
  end Test_Ticking;

  procedure Test_Post_Incoming is
    Exec : aliased Pcl_Component.Executor;
    Comp : Subscriber_Component;

    task type Producer_Task is
      entry Start;
    end Producer_Task;

    task body Producer_Task is
      Payload : aliased Interfaces.C.int := 0;
    begin
      accept Start;

      for I in 1 .. 3 loop
        Payload := Interfaces.C.int(I * 10);
        Pcl_Component.Post_Incoming(
          This => Exec,
          Topic => "sensor_updates",
          Type_Name => "SensorMsg",
          Data => Payload'Address,
          Size => Interfaces.C.unsigned(Interfaces.C.int'Size / System.Storage_Unit));

        Payload := -1;
        delay 0.02;
      end loop;
    end Producer_Task;

    Producer : Producer_Task;
  begin
    Log_Line("test: post incoming");

    Pcl_Component.Create(Exec);
    Pcl_Component.Create(Comp, "subscriber_component");
    Pcl_Component.Set_Tick_Rate_Hz(Comp, 50.0);
    Pcl_Component.Configure(Comp);
    Pcl_Component.Activate(Comp);
    Pcl_Component.Add(Exec, Comp);

    Producer.Start;

    for Iteration in 1 .. 20 loop
      pragma Unreferenced(Iteration);
      delay 0.02;
      Pcl_Component.Spin_Once(Exec);
      exit when Comp.Updates_Received = 3;
    end loop;

    Assert(Comp.Updates_Received = 3, "expected 3 posted updates");
    Assert(Comp.Last_Value = 30, "expected last payload to be 30");
    Pcl_Component.Shutdown_Graceful(Exec);
  end Test_Post_Incoming;
begin
  Test_Ticking;
  Test_Post_Incoming;
  Log_Line("all Ada OO wrapper tests passed");
end Pcl_Component_Tests;
