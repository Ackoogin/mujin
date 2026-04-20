--  bridge_entity_tracker.ads
--
--  PCL container component for the Pyramid Bridge.
--
--  Responsibilities:
--    1. Subscribe to standard.entity_matches on the Tactical Objects executor.
--    2. For each matched entity build a World_Fact_Update with
--       key "entity_<id>_exists" and value True.
--    3. Batch pending fact updates so the main loop can drain them and
--       invoke state.update_state on the AME backend executor.
--
--  Architecture:
--    pyramid_bridge_main (single-threaded spin loop)
--      -> Bridge_Entity_Tracker.On_Configure  (tobj executor, configure phase)
--      -> Bridge_Entity_Tracker.On_Entity_Matches  (tobj executor, callback)
--      -> Bridge_Entity_Tracker.Has_Pending / Drain_Pending  (main loop drain)
--      -> Pyramid.Services.Autonomy_Backend.Provided.Invoke_Update_State
--           (ame executor)

with Ada.Strings.Unbounded;    use Ada.Strings.Unbounded;
with Pcl_Bindings;
with Pyramid.Data_Model.Autonomy.Types;  use Pyramid.Data_Model.Autonomy.Types;
with System;

package Bridge_Entity_Tracker is

   --  AME executor: set by pyramid_bridge_main before On_Configure is called.
   Ame_Exec : Pcl_Bindings.Pcl_Executor_Access := null;

   --  Running count of World_Fact_Update messages delivered to AME.
   Facts_Sent : Natural := 0;

   --  Interest-requirement ID returned by create_requirement response.
   Interest_Id       : Unbounded_String := Null_Unbounded_String;
   Interest_Id_Ready : Boolean := False;

   --  Maximum pending entity IDs buffered between spin cycles.
   Max_Pending : constant := 256;

   subtype Pending_Index is Natural range 0 .. Max_Pending;

   type Id_Array is array (1 .. Max_Pending) of Unbounded_String;

   Pending_Ids   : Id_Array;
   Pending_Count : Pending_Index := 0;

   --  True when at least one entity ID is waiting to be forwarded to AME.
   function Has_Pending return Boolean;

   --  Transfer all pending IDs into Ids(1..Count) and clear the buffer.
   procedure Drain_Pending
     (Ids   : out Id_Array;
      Count : out Pending_Index);

   --  PCL lifecycle callback: registers standard.entity_matches subscriber.
   function On_Configure
     (Container : Pcl_Bindings.Pcl_Container_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, On_Configure);

   --  Subscriber callback: decodes ObjectMatch array and buffers entity IDs.
   procedure On_Entity_Matches
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Msg       : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address);
   pragma Convention (C, On_Entity_Matches);

   --  Async response callback for state.update_state (fire-and-forget).
   procedure On_Update_State_Response
     (Resp      : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address);
   pragma Convention (C, On_Update_State_Response);

   --  Response callback for create_requirement (captures interest ID).
   procedure On_Create_Req_Response
     (Resp      : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address);
   pragma Convention (C, On_Create_Req_Response);

end Bridge_Entity_Tracker;
