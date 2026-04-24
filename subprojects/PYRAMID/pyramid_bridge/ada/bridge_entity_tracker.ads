--  bridge_entity_tracker.ads
--
--  OO PCL component used by the Pyramid Bridge.
--
--  Responsibilities:
--    1. Subscribe to standard.entity_matches on the Tactical Objects executor.
--    2. Buffer entity IDs so the main bridge loop can batch updates.
--    3. Track create_requirement responses and delivery counters.

with Ada.Strings.Unbounded;    use Ada.Strings.Unbounded;
with Pcl_Bindings;
with Pcl_Component;
with System;

package Bridge_Entity_Tracker is

   --  Maximum pending entity IDs buffered between spin cycles.
   Max_Pending : constant := 256;

   subtype Pending_Index is Natural range 0 .. Max_Pending;

   type Id_Array is array (1 .. Max_Pending) of Unbounded_String;

   type Tracker_Component is new Pcl_Component.Component with private;

   procedure Set_Content_Type
     (This         : in out Tracker_Component;
      Content_Type : String);
   function Content_Type(This : Tracker_Component) return String;

   --  True when at least one entity ID is waiting to be forwarded to AME.
   function Has_Pending(This : Tracker_Component) return Boolean;

   --  Transfer all pending IDs into Ids(1..Count) and clear the buffer.
   procedure Drain_Pending
     (This  : in out Tracker_Component;
      Ids   : out Id_Array;
      Count : out Pending_Index);

   procedure Note_Facts_Sent
     (This  : in out Tracker_Component;
      Count : Pending_Index);
   function Facts_Sent(This : Tracker_Component) return Natural;
   function Interest_Id(This : Tracker_Component) return String;
   function Interest_Id_Ready(This : Tracker_Component) return Boolean;

   overriding procedure On_Configure(This : in out Tracker_Component);

   --  Async response callback for state.update_state (fire-and-forget).
   procedure On_Update_State_Response
     (Resp      : access constant Pcl_Bindings.Pcl_Msg;
     User_Data : System.Address);
   pragma Convention (C, On_Update_State_Response);

   --  Response callback for create_requirement (captures interest ID).
   --  User_Data is expected to be Tracker_Component'Address.
   procedure On_Create_Req_Response
     (Resp      : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address);
   pragma Convention (C, On_Create_Req_Response);

private

   type Tracker_Component is new Pcl_Component.Component with record
      Pending_Ids            : Id_Array;
      Pending_Count          : Pending_Index := 0;
      Facts_Sent_Count       : Natural := 0;
      Interest_Id_Value      : Unbounded_String := Null_Unbounded_String;
      Interest_Id_Is_Ready   : Boolean := False;
      Selected_Content_Type  : Unbounded_String := Null_Unbounded_String;
   end record;

end Bridge_Entity_Tracker;
