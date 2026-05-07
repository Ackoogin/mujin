--  pyramid_bridge_main.adb
--
--  Pyramid Bridge — standalone Ada process.
--
--  Bridges the AME Autonomy Backend to the Tactical Objects component using
--  generated PYRAMID Ada bindings together with the canonical PCL Ada wrapper.
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
--      [--tobj-bus  NAME]   (use shared-memory bus instead of TCP socket)
--      [--ame-bus  NAME]    (default "pyramid_bridge")
--      [--content-type FMT] (json | flatbuffers | grpc, default json)
--      [--timeout  SECS]    (default 0 = run until signalled)

with Ada.Command_Line;
with Ada.Characters.Handling;
with Ada.Exceptions;
with Ada.Strings.Unbounded;      use Ada.Strings.Unbounded;
with Ada.Text_IO;
with Ada.Unchecked_Deallocation;
with Interfaces.C;
with Pcl_Bindings;
with Pcl_Component;
with Pcl_Content_Types;
with Pcl_Transports;
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
   use type Pcl_Bindings.Pcl_Executor_Access;
   use type System.Address;

   package Provided_Tobj renames Pyramid.Services.Tactical_Objects.Provided;
   package Provided_Ame  renames Pyramid.Services.Autonomy_Backend.Provided;

   -- -- Configuration variables --------------------------------------------------

   Tobj_Host_Str : Unbounded_String := To_Unbounded_String ("127.0.0.1");
   Tobj_Port     : Interfaces.C.unsigned_short := 19123;
   Tobj_Bus_Name : Unbounded_String := Null_Unbounded_String;
   Ame_Bus_Name  : Unbounded_String :=
     To_Unbounded_String ("pyramid_bridge");
   Bridge_Content_Type : Unbounded_String :=
     To_Unbounded_String (Pcl_Content_Types.Json_Content_Type);
   Timeout_Secs  : Natural := 0;

   function Resolve_Content_Type (Value : String) return String is
      Lowered : constant String := Ada.Characters.Handling.To_Lower (Value);
   begin
      if Lowered = "json"
        or else Lowered = Pcl_Content_Types.Json_Content_Type
      then
         return Pcl_Content_Types.Json_Content_Type;
      elsif Lowered = "flatbuffers"
        or else Lowered = Pcl_Content_Types.Flatbuffers_Content_Type
      then
         return Pcl_Content_Types.Flatbuffers_Content_Type;
      elsif Lowered = "grpc"
        or else Lowered = Pcl_Content_Types.Grpc_Content_Type
      then
         return Pcl_Content_Types.Grpc_Content_Type;
      else
         raise Constraint_Error with
           "unsupported content type '" & Value &
           "' (expected json, flatbuffers, or grpc)";
      end if;
   end Resolve_Content_Type;

   procedure Parse_Args is
      use Ada.Command_Line;
      I : Natural := 1;
   begin
      while I <= Argument_Count loop
         if Argument (I) = "--tobj-host" and then I + 1 <= Argument_Count then
            Tobj_Host_Str := To_Unbounded_String (Argument (I + 1));
            I := I + 2;
         elsif Argument (I) = "--tobj-port" and then I + 1 <= Argument_Count then
            Tobj_Port :=
              Interfaces.C.unsigned_short'Value (Argument (I + 1));
            I := I + 2;
         elsif Argument (I) = "--tobj-bus" and then I + 1 <= Argument_Count then
            Tobj_Bus_Name := To_Unbounded_String (Argument (I + 1));
            I := I + 2;
         elsif Argument (I) = "--ame-bus" and then I + 1 <= Argument_Count then
            Ame_Bus_Name := To_Unbounded_String (Argument (I + 1));
            I := I + 2;
         elsif Argument (I) = "--timeout" and then I + 1 <= Argument_Count then
            Timeout_Secs := Natural'Value (Argument (I + 1));
            I := I + 2;
         elsif Argument (I) = "--content-type" and then I + 1 <= Argument_Count then
            Bridge_Content_Type :=
              To_Unbounded_String (Resolve_Content_Type (Argument (I + 1)));
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

   procedure Check_Spin_Result
     (Label  : String;
      Status : Pcl_Bindings.Pcl_Status) is
   begin
      case Status is
         when Pcl_Bindings.PCL_OK | Pcl_Bindings.PCL_ERR_NOT_FOUND =>
            null;
         when others =>
            raise Pcl_Component.Pcl_Error with
              Label & " failed with status" &
              Interfaces.C.int'Image (Interfaces.C.int (Status));
      end case;
   end Check_Spin_Result;

   -- -- Runtime variables -------------------------------------------------------

   Tobj_Exec        : Pcl_Component.Executor;
   Ame_Exec         : Pcl_Component.Executor;
   Tobj_Transport   : Pcl_Transports.Socket_Transport;
   Tobj_Shmem_Bus   : Pcl_Transports.Shared_Memory_Transport;
   Ame_Transport    : Pcl_Transports.Shared_Memory_Transport;
   Tobj_Tracker     : Bridge_Entity_Tracker.Tracker_Component;

   -- -- Place a global interest requirement ------------------------------------
   --  Policy_Obtain, no dimension/area filter => all entities, continuously.

   procedure Place_Global_Interest is
      Req : Object_Interest_Requirement;
   begin
      Req.Source := Source_Local;
      Req.Policy := Policy_Obtain;
      --  Leave Dimension null and area unset for a global (all-entity) query.

      Log ("placing global ObjectInterestRequirement (Policy_Obtain, all entities)");
      Provided_Tobj.Invoke_Object_Of_Interest_Create_Requirement
        (Executor     => Pcl_Component.Handle (Tobj_Exec),
         Request      => Req,
         Callback     => Bridge_Entity_Tracker.On_Create_Req_Response'Access,
         User_Data    => Tobj_Tracker'Address,
         Content_Type => To_String (Bridge_Content_Type));
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

      Provided_Ame.Invoke_State_Update_State
        (Executor     => Pcl_Component.Handle (Ame_Exec),
         Request      => Update,
         Callback     =>
           Bridge_Entity_Tracker.On_Update_State_Response'Unrestricted_Access,
         Content_Type => To_String (Bridge_Content_Type));

      Free_Fact_Array (Update.Fact_Update);

      Bridge_Entity_Tracker.Note_Facts_Sent (Tobj_Tracker, Count);
   end Push_World_Facts;

   Iteration       : Natural := 0;
   Max_Iterations  : Natural;

begin
   Parse_Args;

   if Ada.Strings.Unbounded.Length (Tobj_Bus_Name) > 0 then
      Log ("Tactical Objects shared-memory bus: " &
           To_String (Tobj_Bus_Name));
   else
      Log ("Tactical Objects: " &
           To_String (Tobj_Host_Str) & ":" &
           Interfaces.C.unsigned_short'Image (Tobj_Port));
   end if;
   Log ("AME shared-memory bus: " & To_String (Ame_Bus_Name));
   Log ("content type: " & To_String (Bridge_Content_Type));

   -- -------------------------------------------------------------------------
   --  1. Create the AME executor and join the shared-memory bus first so the
   --     stub can discover the bridge participant on the bus.
   -- -------------------------------------------------------------------------

   Pcl_Component.Create (Ame_Exec);
   Pcl_Transports.Create
     (This           => Ame_Transport,
      Exec           => Ame_Exec,
      Bus_Name       => To_String (Ame_Bus_Name),
      Participant_Id => "bridge");
   Pcl_Transports.Use_As_Default (Ame_Transport, Ame_Exec);
   Pcl_Transports.Start_Gateway (Ame_Transport, Ame_Exec);

   Log ("joined AME shared-memory bus as participant 'bridge'");

   -- -------------------------------------------------------------------------
   --  2. Connect to the deployed Tactical Objects app via TCP socket.
   -- -------------------------------------------------------------------------

   Pcl_Component.Create (Tobj_Exec);

   if Ada.Strings.Unbounded.Length (Tobj_Bus_Name) > 0 then
      Pcl_Transports.Create
        (This           => Tobj_Shmem_Bus,
         Exec           => Tobj_Exec,
         Bus_Name       => To_String (Tobj_Bus_Name),
         Participant_Id => "bridge_tobj");
      Pcl_Transports.Use_As_Default (Tobj_Shmem_Bus, Tobj_Exec);
      Pcl_Transports.Start_Gateway (Tobj_Shmem_Bus, Tobj_Exec);

      Log ("joined tactical_objects_app shared-memory bus");
   else
      Pcl_Transports.Create_Client
        (This => Tobj_Transport,
         Exec => Tobj_Exec,
         Host => To_String (Tobj_Host_Str),
         Port => Tobj_Port);
      Pcl_Transports.Use_As_Default (Tobj_Transport, Tobj_Exec);

      Log ("connected to tactical_objects_app");
   end if;

   -- -------------------------------------------------------------------------
   --  3. Create the entity-tracker container on the Tobj executor.
   --     It subscribes to standard.entity_matches.
   -- -------------------------------------------------------------------------

   Bridge_Entity_Tracker.Set_Content_Type
     (Tobj_Tracker, To_String (Bridge_Content_Type));
   Pcl_Component.Create (Tobj_Tracker, "bridge_tobj_consumer");
   Pcl_Component.Configure (Tobj_Tracker);
   Pcl_Component.Activate (Tobj_Tracker);
   Pcl_Component.Add (Tobj_Exec, Tobj_Tracker);

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
      Check_Spin_Result
        ("tactical objects spin once",
         Pcl_Component.Spin_Once_Status (Tobj_Exec, 0));

      --  Drain pending entity IDs and push world facts to AME.
      if Bridge_Entity_Tracker.Has_Pending (Tobj_Tracker) then
         declare
            Ids   : Bridge_Entity_Tracker.Id_Array;
            Count : Bridge_Entity_Tracker.Pending_Index;
         begin
            Bridge_Entity_Tracker.Drain_Pending (Tobj_Tracker, Ids, Count);
            Push_World_Facts (Ids, Count);
         end;
      end if;

      --  Drive AME executor (delivers state.update_state and receives ACK).
      Check_Spin_Result
        ("ame spin once",
         Pcl_Component.Spin_Once_Status (Ame_Exec, 0));

      Iteration := Iteration + 1;
      delay 0.005;
   end loop;

   Log ("spin complete — facts_sent=" &
        Natural'Image (Bridge_Entity_Tracker.Facts_Sent (Tobj_Tracker)));

   -- -------------------------------------------------------------------------
   --  6. Cleanup.
   -- -------------------------------------------------------------------------

   if Pcl_Component.Handle (Tobj_Exec) /= null then
      Pcl_Component.Shutdown_Graceful (Tobj_Exec);
   end if;
   if Pcl_Component.Handle (Ame_Exec) /= null then
      Pcl_Component.Shutdown_Graceful (Ame_Exec);
   end if;

   if Bridge_Entity_Tracker.Facts_Sent (Tobj_Tracker) > 0 then
      Log ("PASS: " &
           Natural'Image (Bridge_Entity_Tracker.Facts_Sent (Tobj_Tracker)) &
           " world fact(s) delivered to AME backend");
      Ada.Command_Line.Set_Exit_Status (0);
   else
      Log ("WARN: no world facts delivered (no entities seen from Tactical Objects)");
      Ada.Command_Line.Set_Exit_Status (0);
   end if;

exception
   when E : others =>
      Log ("FAIL: " & Ada.Exceptions.Exception_Message (E));
      Ada.Command_Line.Set_Exit_Status (1);
end Pyramid_Bridge_Main;
