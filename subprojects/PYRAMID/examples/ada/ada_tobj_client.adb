--  ada_tobj_client.adb
--
--  Standalone Ada executable that connects to a TacticalObjectsComponent
--  server via TCP socket transport.
--
--  Uses the interest client component (standard bridge interface):
--    - Calls  "object_of_interest.create_requirement"  (standard JSON)
--    - Subscribes to "standard.entity_matches"          (standard JSON array)
--
--  Architecture: main (this) > component logic > service binding > PCL
--
--  Usage: ada_tobj_client --host 127.0.0.1 --port 19123

with Ada.Command_Line;
with Ada.Strings.Unbounded;      use Ada.Strings.Unbounded;
with Ada.Text_IO;
with Interfaces.C;
with Interfaces.C.Strings;
with Pcl_Bindings;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Tobj_Interest_Client;
with System;

procedure Ada_Tobj_Client is
   use type Interfaces.C.unsigned_short;
   use type Pcl_Bindings.Pcl_Status;
   use type Pcl_Bindings.Pcl_Executor_Access;
   use type Pcl_Bindings.Pcl_Container_Access;
   use type Pcl_Bindings.Pcl_Socket_Transport_Access;

   Pi         : constant Long_Float := 3.14159265358979323846;
   Deg_To_Rad : constant Long_Float := Pi / 180.0;

   -- -- Configuration ----------------------------------------------------------

   Host_Str : Interfaces.C.Strings.chars_ptr :=
     Interfaces.C.Strings.New_String ("127.0.0.1");
   Port_Val : Interfaces.C.unsigned_short := 19000;

   procedure Parse_Args is
      use Ada.Command_Line;
      I : Natural := 1;
   begin
      while I <= Argument_Count loop
         if Argument (I) = "--host" and then I + 1 <= Argument_Count then
            Interfaces.C.Strings.Free (Host_Str);
            Host_Str := Interfaces.C.Strings.New_String (Argument (I + 1));
            I := I + 2;
         elsif Argument (I) = "--port" and then I + 1 <= Argument_Count then
            Port_Val := Interfaces.C.unsigned_short'Value (Argument (I + 1));
            I := I + 2;
         elsif Argument (I) = "--content-type" and then I + 1 <= Argument_Count then
            Tobj_Interest_Client.Content_Type :=
              To_Unbounded_String (Argument (I + 1));
            I := I + 2;
         else
            I := I + 1;
         end if;
      end loop;
   end Parse_Args;

   procedure Log (Msg : String) is
   begin
      Ada.Text_IO.Put_Line (Ada.Text_IO.Standard_Error,
                            "[ada_tobj_client] " & Msg);
      Ada.Text_IO.Flush (Ada.Text_IO.Standard_Error);
   end Log;

   -- -- Main variables ---------------------------------------------------------

   Exec      : Pcl_Bindings.Pcl_Executor_Access;
   Transport : Pcl_Bindings.Pcl_Socket_Transport_Access;
   Container : Pcl_Bindings.Pcl_Container_Access;
   Name_C    : Interfaces.C.Strings.chars_ptr;
   Cbs       : aliased Pcl_Bindings.Pcl_Callbacks;
   Status    : Pcl_Bindings.Pcl_Status;

begin
   Parse_Args;
   Log ("Connecting to " & Interfaces.C.Strings.Value (Host_Str) &
        ":" & Interfaces.C.unsigned_short'Image (Port_Val) &
        " content-type=" & To_String (Tobj_Interest_Client.Content_Type));

   -- -- Create executor --------------------------------------------------------

   Exec := Pcl_Bindings.Create_Executor;
   if Exec = null then
      Log ("FAIL: could not create executor");
      Ada.Command_Line.Set_Exit_Status (1);
      return;
   end if;

   -- -- Connect via socket client transport ------------------------------------

   Transport := Pcl_Bindings.Create_Socket_Client (Host_Str, Port_Val, Exec);
   Interfaces.C.Strings.Free (Host_Str);

   if Transport = null then
      Log ("FAIL: could not connect to server");
      Pcl_Bindings.Destroy_Executor (Exec);
      Ada.Command_Line.Set_Exit_Status (1);
      return;
   end if;

   Status := Pcl_Bindings.Set_Transport
     (Exec, Pcl_Bindings.Get_Socket_Transport (Transport));
   if Status /= Pcl_Bindings.PCL_OK then
      Log ("FAIL: could not set transport");
      Pcl_Bindings.Destroy_Socket_Transport (Transport);
      Pcl_Bindings.Destroy_Executor (Exec);
      Ada.Command_Line.Set_Exit_Status (1);
      return;
   end if;

   -- -- Create subscriber container --------------------------------------------

   Cbs := (On_Configure  => Tobj_Interest_Client.On_Configure'Access,
           On_Activate   => null,
           On_Deactivate => null,
           On_Cleanup    => null,
           On_Shutdown   => null,
           On_Tick       => null);
   Name_C := Interfaces.C.Strings.New_String ("ada_tobj_client");
   Container := Pcl_Bindings.Create_Container
     (Name_C, Cbs'Access, System.Null_Address);
   Interfaces.C.Strings.Free (Name_C);

   if Container = null then
      Log ("FAIL: could not create container");
      Pcl_Bindings.Destroy_Socket_Transport (Transport);
      Pcl_Bindings.Destroy_Executor (Exec);
      Ada.Command_Line.Set_Exit_Status (1);
      return;
   end if;

   Status := Pcl_Bindings.Configure (Container);
   Status := Pcl_Bindings.Activate (Container);
   Status := Pcl_Bindings.Add_Container (Exec, Container);

   -- -- Send create_requirement (passive query via bridge) ---------------------

   Tobj_Interest_Client.Send_Create_Requirement
     (Exec        => Exec,
      Policy      => Policy_Query,
      Identity    => Identity_Hostile,
      Dimension   => Dimension_Unspecified,
      Min_Lat_Rad => 50.0 * Deg_To_Rad,
      Max_Lat_Rad => 52.0 * Deg_To_Rad,
      Min_Lon_Rad => (-1.0) * Deg_To_Rad,
      Max_Lon_Rad => 1.0 * Deg_To_Rad);

   -- -- Spin until service response and entity matches arrive ------------------

   Log ("Spinning to receive service response and entity matches...");
   for Iteration in 1 .. 200 loop
      Status := Pcl_Bindings.Spin_Once (Exec, 0);
      exit when Tobj_Interest_Client.Svc_Response_Ready
        and then Tobj_Interest_Client.Matches_Received > 0;
      delay 0.01;
   end loop;

   if Tobj_Interest_Client.Svc_Response_Ready then
      Log ("create_requirement OK");
      if Tobj_Interest_Client.Interest_Id_Received then
         Log ("  interest_id present");
      else
         Log ("  WARNING: interest_id NOT found in response");
      end if;
   else
      Log ("create_requirement TIMEOUT: no response");
   end if;

   -- -- Report pass/fail -------------------------------------------------------

   if Tobj_Interest_Client.Matches_Received > 0 then
      Log ("PASS: received" &
           Natural'Image (Tobj_Interest_Client.Matches_Received) &
           " standard entity match(es) via bridge");
      Ada.Command_Line.Set_Exit_Status (0);
   else
      Log ("FAIL: no standard entity matches received");
      Ada.Command_Line.Set_Exit_Status (1);
   end if;

   -- -- Cleanup ----------------------------------------------------------------

   Pcl_Bindings.Destroy_Socket_Transport (Transport);
   Pcl_Bindings.Destroy_Container (Container);
   Pcl_Bindings.Destroy_Executor (Exec);
end Ada_Tobj_Client;
