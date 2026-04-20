--  pyramid_bridge_main.adb
--
--  Pyramid Bridge — standalone Ada process.
--
--  Bridges the AME Autonomy Backend to the Tactical Objects component using
--  generated proto-native JSON bindings and PCL transports.
--
--  Transport topology
--  ------------------
--  [tactical_objects_app]  <-- TCP socket (client) -- [this process]
--                                                             |
--                                                  shared-memory bus
--                                                             |
--                                                  [ame_backend_stub]
--
--  Behaviour
--  ---------
--  1. Connect to tactical_objects_app as a TCP socket client.
--  2. Join the AME shared-memory bus as participant "bridge".
--  3. Place a global (all-entities) ObjectInterestRequirement with
--     Policy_Obtain, no area or dimension filter.
--  4. On each standard.entity_matches message from Tactical Objects, build
--     a World_Fact_Update for every entity:
--       key   = "entity_<id>_exists"
--       value = true
--       authority = Level_Confirmed
--  5. Invoke state.update_state on the AME backend via shared-memory.
--  6. Spin until timeout or SIGINT/SIGTERM.
--
--  Usage
--  -----
--    pyramid_bridge_main
--      [--tobj-host HOST]   (default 127.0.0.1)
--      [--tobj-port PORT]   (default 19123)
--      [--ame-bus  NAME]    (default "pyramid_bridge")
--      [--timeout  SECS]    (default 0 = run until signalled)

with Ada.Command_Line;
with Ada.Strings.Unbounded;      use Ada.Strings.Unbounded;
with Ada.Text_IO;
with Ada.Unchecked_Deallocation;
with Interfaces.C;
with Interfaces.C.Strings;
with Pcl_Bindings;
with Pcl_Shmem_Bindings;
with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Autonomy.Types;  use Pyramid.Data_Model.Autonomy.Types;
with Pyramid.Data_Model.Tactical.Types;  use Pyramid.Data_Model.Tactical.Types;
with Pyramid.Services.Tactical_Objects.Provided;
with Pyramid.Services.Autonomy_Backend.Provided;
with Bridge_Entity_Tracker;
with System;

procedure Pyramid_Bridge_Main is

   use type Interfaces.C.unsigned_short;
   use type Interfaces.C.unsigned;
   use type Pcl_Bindings.Pcl_Status;
   use type Pcl_Bindings.Pcl_Executor_Access;
   use type Pcl_Bindings.Pcl_Container_Access;
   use type Pcl_Bindings.Pcl_Socket_Transport_Access;
   use type Pcl_Shmem_Bindings.Pcl_Shared_Memory_Transport_Access;
   use type System.Address;

   package Provided_Tobj renames Pyramid.Services.Tactical_Objects.Provided;
   package Provided_Ame  renames Pyramid.Services.Autonomy_Backend.Provided;

   -- -- Configuration variables --------------------------------------------------

   Tobj_Host_Str : Interfaces.C.Strings.chars_ptr :=
     Interfaces.C.Strings.New_String ("127.0.0.1");
   Tobj_Port     : Interfaces.C.unsigned_short := 19123;
   Ame_Bus_Name  : Unbounded_String :=
     To_Unbounded_String ("pyramid_bridge");
   Timeout_Secs  : Natural := 0;

   procedure Parse_Args is
      use Ada.Command_Line;
      I : Natural := 1;
   begin
      while I <= Argument_Count loop
         if Argument (I) = "--tobj-host" and then I + 1 <= Argument_Count then
            Interfaces.C.Strings.Free (Tobj_Host_Str);
            Tobj_Host_Str :=
              Interfaces.C.Strings.New_String (Argument (I + 1));
            I := I + 2;
         elsif Argument (I) = "--tobj-port" and then I + 1 <= Argument_Count then
            Tobj_Port :=
              Interfaces.C.unsigned_short'Value (Argument (I + 1));
            I := I + 2;
         elsif Argument (I) = "--ame-bus" and then I + 1 <= Argument_Count then
            Ame_Bus_Name := To_Unbounded_String (Argument (I + 1));
            I := I + 2;
         elsif Argument (I) = "--timeout" and then I + 1 <= Argument_Count then
            Timeout_Secs := Natural'Value (Argument (I + 1));
            I := I + 2;
         else
            I := I + 1;
         end if;
      end loop;
   end Parse_Args;

   procedure Log (Msg : String) is
   begin
      Ada.Text_IO.Put_Line (Ada.Text_IO.Standard_Error,
                            "[pyramid_bridge] " & Msg);
      Ada.Text_IO.Flush (Ada.Text_IO.Standard_Error);
   end Log;

   -- -- Runtime variables -------------------------------------------------------

   --  Tactical Objects side (TCP socket client)
   Tobj_Exec      : Pcl_Bindings.Pcl_Executor_Access;
   Tobj_Transport : Pcl_Bindings.Pcl_Socket_Transport_Access;

   --  AME side (shared-memory bus participant)
   Ame_Exec      : Pcl_Bindings.Pcl_Executor_Access;
   Ame_Transport : Pcl_Shmem_Bindings.Pcl_Shared_Memory_Transport_Access;

   -- -- Place a global interest requirement ------------------------------------
   --  Policy_Obtain, no dimension/area filter => all entities, continuously.

   procedure Place_Global_Interest is
      Req : Object_Interest_Requirement;
   begin
      Req.Source := Source_Local;
      Req.Policy := Policy_Obtain;
      --  Leave Dimension null and area unset for a global (all-entity) query.

      Log ("placing global ObjectInterestRequirement (Policy_Obtain, all entities)");
      Provided_Tobj.Invoke_Create_Requirement
        (Executor     => Tobj_Exec,
         Request      => Req,
         Callback     => Bridge_Entity_Tracker.On_Create_Req_Response'Access,
         Content_Type => Provided_Tobj.Json_Content_Type);
   end Place_Global_Interest;

   -- -- Build and push world facts to AME for a batch of entity IDs -----------

   procedure Free_Fact_Array is new Ada.Unchecked_Deallocation
     (Fact_Update_Array, Fact_Update_Array_Acc);

   procedure Push_World_Facts
     (Ids   : Bridge_Entity_Tracker.Id_Array;
      Count : Bridge_Entity_Tracker.Pending_Index)
   is
      Facts   : Fact_Update_Array (1 .. Count);
      Update  : State_Update;
   begin
      for I in 1 .. Count loop
         Facts (I).Key    := To_Unbounded_String
           ("entity_" & To_String (Ids (I)) & "_exists");
         Facts (I).Value     := True;
         Facts (I).Authority := Level_Confirmed;
         Facts (I).Source    := To_Unbounded_String ("pyramid_bridge");
         Log ("world fact: " & To_String (Facts (I).Key));
      end loop;

      Update.Source      := To_Unbounded_String ("pyramid_bridge");
      Update.Fact_Update := new Fact_Update_Array'(Facts);

      Provided_Ame.Invoke_Update_State
        (Executor     => Ame_Exec,
         Request      => Update,
         Callback     =>
           Bridge_Entity_Tracker.On_Update_State_Response'Unrestricted_Access,
         Content_Type => Provided_Ame.Json_Content_Type);

      Free_Fact_Array (Update.Fact_Update);

      Bridge_Entity_Tracker.Facts_Sent :=
        Bridge_Entity_Tracker.Facts_Sent + Count;
   end Push_World_Facts;

   -- -- Main ------------------------------------------------------------------

   Name_C   : Interfaces.C.Strings.chars_ptr;
   Tobj_Cbs : aliased Pcl_Bindings.Pcl_Callbacks;
   Tobj_C   : Pcl_Bindings.Pcl_Container_Access;
   Gw_C     : Pcl_Bindings.Pcl_Container_Access;
   Status   : Pcl_Bindings.Pcl_Status;
   pragma Unreferenced (Status);

   Iteration       : Natural := 0;
   Max_Iterations  : Natural;

begin
   Parse_Args;

   Log ("Tactical Objects: " &
        Interfaces.C.Strings.Value (Tobj_Host_Str) & ":" &
        Interfaces.C.unsigned_short'Image (Tobj_Port));
   Log ("AME shared-memory bus: " & To_String (Ame_Bus_Name));

   -- -------------------------------------------------------------------------
   --  1. Create the AME executor and join the shared-memory bus first so the
   --     stub can discover the bridge participant on the bus.
   -- -------------------------------------------------------------------------

   Ame_Exec := Pcl_Bindings.Create_Executor;
   if Ame_Exec = null then
      Log ("FAIL: could not create AME executor");
      Ada.Command_Line.Set_Exit_Status (1);
      return;
   end if;

   declare
      Bus_Cstr  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (To_String (Ame_Bus_Name));
      Part_Cstr : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("bridge");
   begin
      Ame_Transport :=
        Pcl_Shmem_Bindings.Create (Bus_Cstr, Part_Cstr, Ame_Exec);
      Interfaces.C.Strings.Free (Bus_Cstr);
      Interfaces.C.Strings.Free (Part_Cstr);
   end;

   if Ame_Transport = null then
      Log ("FAIL: could not create shared-memory transport (bus=" &
           To_String (Ame_Bus_Name) & ")");
      Pcl_Bindings.Destroy_Executor (Ame_Exec);
      Ada.Command_Line.Set_Exit_Status (1);
      return;
   end if;

   Status := Pcl_Bindings.Set_Transport
     (Ame_Exec, Pcl_Shmem_Bindings.Get_Transport (Ame_Transport));

   Gw_C := Pcl_Shmem_Bindings.Gateway_Container (Ame_Transport);
   Status := Pcl_Bindings.Configure (Gw_C);
   Status := Pcl_Bindings.Activate (Gw_C);
   Status := Pcl_Bindings.Add_Container (Ame_Exec, Gw_C);

   Log ("joined AME shared-memory bus as participant 'bridge'");

   -- -------------------------------------------------------------------------
   --  2. Connect to the deployed Tactical Objects app via TCP socket.
   -- -------------------------------------------------------------------------

   Tobj_Exec := Pcl_Bindings.Create_Executor;
   if Tobj_Exec = null then
      Log ("FAIL: could not create Tobj executor");
      Pcl_Shmem_Bindings.Destroy (Ame_Transport);
      Pcl_Bindings.Destroy_Executor (Ame_Exec);
      Ada.Command_Line.Set_Exit_Status (1);
      return;
   end if;

   Tobj_Transport :=
     Pcl_Bindings.Create_Socket_Client (Tobj_Host_Str, Tobj_Port, Tobj_Exec);
   Interfaces.C.Strings.Free (Tobj_Host_Str);

   if Tobj_Transport = null then
      Log ("FAIL: could not connect to tactical_objects_app on port" &
           Interfaces.C.unsigned_short'Image (Tobj_Port));
      Pcl_Bindings.Destroy_Executor (Tobj_Exec);
      Pcl_Shmem_Bindings.Destroy (Ame_Transport);
      Pcl_Bindings.Destroy_Executor (Ame_Exec);
      Ada.Command_Line.Set_Exit_Status (1);
      return;
   end if;

   Status := Pcl_Bindings.Set_Transport
     (Tobj_Exec, Pcl_Bindings.Get_Socket_Transport (Tobj_Transport));

   Log ("connected to tactical_objects_app");

   -- -------------------------------------------------------------------------
   --  3. Create the entity-tracker container on the Tobj executor.
   --     It subscribes to standard.entity_matches.
   -- -------------------------------------------------------------------------

   Bridge_Entity_Tracker.Ame_Exec := Ame_Exec;

   Tobj_Cbs :=
     (On_Configure  => Bridge_Entity_Tracker.On_Configure'Access,
      On_Activate   => null,
      On_Deactivate => null,
      On_Cleanup    => null,
      On_Shutdown   => null,
      On_Tick       => null);

   Name_C := Interfaces.C.Strings.New_String ("bridge_tobj_consumer");
   Tobj_C := Pcl_Bindings.Create_Container
     (Name_C, Tobj_Cbs'Access, System.Null_Address);
   Interfaces.C.Strings.Free (Name_C);

   if Tobj_C = null then
      Log ("FAIL: could not create bridge_tobj_consumer container");
      Pcl_Bindings.Destroy_Socket_Transport (Tobj_Transport);
      Pcl_Bindings.Destroy_Executor (Tobj_Exec);
      Pcl_Shmem_Bindings.Destroy (Ame_Transport);
      Pcl_Bindings.Destroy_Executor (Ame_Exec);
      Ada.Command_Line.Set_Exit_Status (1);
      return;
   end if;

   Status := Pcl_Bindings.Configure (Tobj_C);
   Status := Pcl_Bindings.Activate (Tobj_C);
   Status := Pcl_Bindings.Add_Container (Tobj_Exec, Tobj_C);

   -- -------------------------------------------------------------------------
   --  4. Place global interest requirement on Tactical Objects.
   -- -------------------------------------------------------------------------

   Place_Global_Interest;

   -- -------------------------------------------------------------------------
   --  5. Spin loop — forward entity matches to AME as world facts.
   -- -------------------------------------------------------------------------

   Max_Iterations :=
     (if Timeout_Secs > 0 then Timeout_Secs * 200 else Natural'Last);

   Log ("spinning (max_iterations=" & Natural'Image (Max_Iterations) & ")");

   while Iteration < Max_Iterations loop
      --  Drive Tactical Objects executor (receives entity_matches).
      Status := Pcl_Bindings.Spin_Once (Tobj_Exec, 0);

      --  Drain pending entity IDs and push world facts to AME.
      if Bridge_Entity_Tracker.Has_Pending then
         declare
            Ids   : Bridge_Entity_Tracker.Id_Array;
            Count : Bridge_Entity_Tracker.Pending_Index;
         begin
            Bridge_Entity_Tracker.Drain_Pending (Ids, Count);
            Push_World_Facts (Ids, Count);
         end;
      end if;

      --  Drive AME executor (delivers state.update_state and receives ACK).
      Status := Pcl_Bindings.Spin_Once (Ame_Exec, 0);

      Iteration := Iteration + 1;
      delay 0.005;
   end loop;

   Log ("spin complete — facts_sent=" &
        Natural'Image (Bridge_Entity_Tracker.Facts_Sent));

   -- -------------------------------------------------------------------------
   --  6. Cleanup.
   -- -------------------------------------------------------------------------

   Pcl_Bindings.Destroy_Socket_Transport (Tobj_Transport);
   Pcl_Bindings.Destroy_Container (Tobj_C);
   Pcl_Bindings.Destroy_Executor (Tobj_Exec);

   Pcl_Shmem_Bindings.Destroy (Ame_Transport);
   Pcl_Bindings.Destroy_Executor (Ame_Exec);

   if Bridge_Entity_Tracker.Facts_Sent > 0 then
      Log ("PASS: " & Natural'Image (Bridge_Entity_Tracker.Facts_Sent) &
           " world fact(s) delivered to AME backend");
      Ada.Command_Line.Set_Exit_Status (0);
   else
      Log ("WARN: no world facts delivered (no entities seen from Tactical Objects)");
      Ada.Command_Line.Set_Exit_Status (0);
   end if;

end Pyramid_Bridge_Main;
