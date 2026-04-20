--  ada_active_find_e2e.adb
--
--  E2E test for the standard ActiveFind flow via StandardBridge.
--
--  Two reusable components connect to the server over TCP socket transport:
--    1. Interest Client  — calls create_requirement, receives entity_matches
--    2. Evidence Provider — receives evidence_requirements, publishes evidence
--
--  Architecture: main (this) > component logic > service binding > PCL
--
--  Business/test logic (what to request, what to assert) is authored here.
--  Serialisation is handled by the generated proto-native codecs.
--
--  Usage: ada_active_find_e2e --host 127.0.0.1 --port 19123

with Ada.Command_Line;
with Ada.Strings.Unbounded;      use Ada.Strings.Unbounded;
with Ada.Text_IO;
with Interfaces.C;
with Interfaces.C.Strings;
with Pcl_Bindings;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Tobj_Interest_Client;
with Tobj_Evidence_Provider;
with System;

procedure Ada_Active_Find_E2E is
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
            Tobj_Evidence_Provider.Content_Type :=
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
                            "[ada_active_find] " & Msg);
      Ada.Text_IO.Flush (Ada.Text_IO.Standard_Error);
   end Log;

   -- -- Main variables ---------------------------------------------------------

   Exec       : Pcl_Bindings.Pcl_Executor_Access;
   Transport  : Pcl_Bindings.Pcl_Socket_Transport_Access;
   Client_C   : Pcl_Bindings.Pcl_Container_Access;
   Provider_C : Pcl_Bindings.Pcl_Container_Access;
   Name_C     : Interfaces.C.Strings.chars_ptr;
   Client_Cbs : aliased Pcl_Bindings.Pcl_Callbacks;
   Prov_Cbs   : aliased Pcl_Bindings.Pcl_Callbacks;
   Status     : Pcl_Bindings.Pcl_Status;

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
   Tobj_Evidence_Provider.Exec_Handle := Exec;

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

   -- -- Create interest client container ---------------------------------------

   Client_Cbs := (On_Configure  => Tobj_Interest_Client.On_Configure'Access,
                  On_Activate   => null,
                  On_Deactivate => null,
                  On_Cleanup    => null,
                  On_Shutdown   => null,
                  On_Tick       => null);
   Name_C := Interfaces.C.Strings.New_String ("ada_client");
   Client_C := Pcl_Bindings.Create_Container
     (Name_C, Client_Cbs'Access, System.Null_Address);
   Interfaces.C.Strings.Free (Name_C);

   if Client_C = null then
      Log ("FAIL: could not create client container");
      Pcl_Bindings.Destroy_Socket_Transport (Transport);
      Pcl_Bindings.Destroy_Executor (Exec);
      Ada.Command_Line.Set_Exit_Status (1);
      return;
   end if;

   Status := Pcl_Bindings.Configure (Client_C);
   Status := Pcl_Bindings.Activate (Client_C);
   Status := Pcl_Bindings.Add_Container (Exec, Client_C);

   -- -- Create evidence provider container -------------------------------------

   Prov_Cbs := (On_Configure  => Tobj_Evidence_Provider.On_Configure'Access,
                On_Activate   => null,
                On_Deactivate => null,
                On_Cleanup    => null,
                On_Shutdown   => null,
                On_Tick       => null);
   Name_C := Interfaces.C.Strings.New_String ("ada_evidence_provider");
   Provider_C := Pcl_Bindings.Create_Container
     (Name_C, Prov_Cbs'Access, System.Null_Address);
   Interfaces.C.Strings.Free (Name_C);

   if Provider_C = null then
      Log ("FAIL: could not create provider container");
      Pcl_Bindings.Destroy_Container (Client_C);
      Pcl_Bindings.Destroy_Socket_Transport (Transport);
      Pcl_Bindings.Destroy_Executor (Exec);
      Ada.Command_Line.Set_Exit_Status (1);
      return;
   end if;

   Status := Pcl_Bindings.Configure (Provider_C);
   Status := Pcl_Bindings.Activate (Provider_C);
   Status := Pcl_Bindings.Add_Container (Exec, Provider_C);

   -- -- Send create_requirement (ActiveFind via bridge) ------------------------
   --
   --  Business logic: request SEA_SURFACE entities within
   --  a bounding box covering the English Channel (50–52°N, 1°W–1°E).
   --  Typed enum values avoid stringly-typed JSON construction here.

   Tobj_Interest_Client.Send_Create_Requirement
     (Exec        => Exec,
      Policy      => Policy_Obtain,
      Identity    => Identity_Hostile,
      Dimension   => Dimension_SeaSurface,
      Min_Lat_Rad => 50.0 * Deg_To_Rad,
      Max_Lat_Rad => 52.0 * Deg_To_Rad,
      Min_Lon_Rad => (-1.0) * Deg_To_Rad,
      Max_Lon_Rad => 1.0 * Deg_To_Rad);

   -- -- Spin until the full standard flow completes ----------------------------

   Log ("Spinning to drive standard ActiveFind flow...");
   for Iteration in 1 .. 400 loop
      Status := Pcl_Bindings.Spin_Once (Exec, 0);
      exit when Tobj_Interest_Client.Svc_Response_Ready
        and then Tobj_Interest_Client.Matches_Received > 0;
      delay 0.01;
   end loop;

   if Tobj_Interest_Client.Svc_Response_Ready then
      Log ("create_requirement OK");
      if Tobj_Interest_Client.Interest_Id_Received then
         Log ("  interest_id present in response");
      else
         Log ("  WARNING: interest_id NOT found in response");
      end if;
   else
      Log ("create_requirement TIMEOUT: no response");
   end if;

   -- -- Report pass/fail -------------------------------------------------------

   Log ("--- Results ---");
   Log ("  Evidence requirement received: " &
        Boolean'Image (Tobj_Evidence_Provider.Evidence_Req_Received));
   Log ("  Standard observation sent:     " &
        Boolean'Image (Tobj_Evidence_Provider.Observation_Sent));
   Log ("  Standard entity matches:       " &
        Natural'Image (Tobj_Interest_Client.Matches_Received));
   Log ("  Found matching entity:         " &
        Boolean'Image (Tobj_Interest_Client.Found_Hostile_Entity));

   if Tobj_Evidence_Provider.Evidence_Req_Received
     and then Tobj_Evidence_Provider.Observation_Sent
     and then Tobj_Interest_Client.Matches_Received > 0
     and then Tobj_Interest_Client.Found_Hostile_Entity
   then
      Log ("PASS: Standard ActiveFind flow — evidence provider drove entity " &
           "creation via bridge, client received canonical standard.entity_matches");
      Ada.Command_Line.Set_Exit_Status (0);
   else
      Log ("FAIL: Standard ActiveFind flow incomplete");
      Ada.Command_Line.Set_Exit_Status (1);
   end if;

   -- -- Cleanup ----------------------------------------------------------------

   Pcl_Bindings.Destroy_Socket_Transport (Transport);
   Pcl_Bindings.Destroy_Container (Provider_C);
   Pcl_Bindings.Destroy_Container (Client_C);
   Pcl_Bindings.Destroy_Executor (Exec);
end Ada_Active_Find_E2E;
