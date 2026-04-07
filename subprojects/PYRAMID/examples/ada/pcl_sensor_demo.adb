with Ada.Text_IO;
with Interfaces.C;
with Pcl_Component;
with System;
with System.Address_To_Access_Conversions;

procedure Pcl_Sensor_Demo is
  use type Interfaces.C.int;

  package Int_Pointers is new System.Address_To_Access_Conversions(Interfaces.C.int);

  type Sensor_Component is new Pcl_Component.Component with record
    Updates_Received : Natural := 0;
    Last_Value       : Interfaces.C.int := 0;
  end record;

  overriding procedure On_Configure(This : in out Sensor_Component);
  overriding procedure On_Tick(This : in out Sensor_Component;
                               Dt_Seconds : Interfaces.C.double);
  overriding procedure On_Message(This : in out Sensor_Component;
                                  Topic : String;
                                  Message : Pcl_Component.Message_View);

  task type Producer_Task is
    entry Start;
  end Producer_Task;

  procedure Log_Line(Message : String) is
  begin
    Ada.Text_IO.Put_Line(Ada.Text_IO.Standard_Error, Message);
    Ada.Text_IO.Flush(Ada.Text_IO.Standard_Error);
  end Log_Line;

  procedure On_Configure(This : in out Sensor_Component) is
  begin
    Pcl_Component.Subscribe(This, "sensor_updates", "SensorMsg");
  end On_Configure;

  procedure On_Tick(This : in out Sensor_Component;
                    Dt_Seconds : Interfaces.C.double) is
    pragma Unreferenced(Dt_Seconds);
  begin
    if This.Updates_Received >= 3 then
      Log_Line("received all sensor updates");
    end if;
  end On_Tick;

  procedure On_Message(This : in out Sensor_Component;
                       Topic : String;
                       Message : Pcl_Component.Message_View) is
    pragma Unreferenced(Topic);
    Value : constant Int_Pointers.Object_Pointer :=
      Int_Pointers.To_Pointer(Pcl_Component.Data_Address(Message));
  begin
    This.Updates_Received := This.Updates_Received + 1;
    This.Last_Value := Value.all;
  end On_Message;

  Exec : aliased Pcl_Component.Executor;
  Comp : Sensor_Component;

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

      -- The queue owns its own copy after post_incoming returns.
      Payload := -1;
      delay 0.02;
    end loop;
  end Producer_Task;

  Producer : Producer_Task;
begin
  Pcl_Component.Create(Exec);
  Pcl_Component.Create(Comp, "ada_sensor_consumer");
  Pcl_Component.Set_Tick_Rate_Hz(Comp, 50.0);
  Pcl_Component.Configure(Comp);
  Pcl_Component.Activate(Comp);
  Pcl_Component.Add(Exec, Comp);

  Log_Line("Starting Ada PCL sensor demo...");
  Log_Line("A producer task injects updates into the executor queue.");
  Producer.Start;

  for Iteration in 1 .. 20 loop
    pragma Unreferenced(Iteration);
    delay 0.02;
    Pcl_Component.Spin_Once(Exec);
    exit when Comp.Updates_Received = 3;
  end loop;

  Pcl_Component.Shutdown_Graceful(Exec);
  Log_Line(
    "Done. Updates=" & Interfaces.C.int'Image(Interfaces.C.int(Comp.Updates_Received)) &
    " Last=" & Interfaces.C.int'Image(Comp.Last_Value));
end Pcl_Sensor_Demo;
