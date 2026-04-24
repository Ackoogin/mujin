--  bridge_entity_tracker.adb

with Ada.Text_IO;
with Ada.Strings.Unbounded;    use Ada.Strings.Unbounded;
with Ada.Unchecked_Conversion;
with Interfaces.C;
with System;
with Pyramid.Services.Tactical_Objects.Provided;
with Pcl_Component;

package body Bridge_Entity_Tracker is

   package Provided_Tobj renames Pyramid.Services.Tactical_Objects.Provided;

   use type Interfaces.C.unsigned;
   use type System.Address;

   procedure Log (Msg : String) is
   begin
      Ada.Text_IO.Put_Line (Ada.Text_IO.Standard_Error,
                            "[pyramid_bridge] " & Msg);
      Ada.Text_IO.Flush (Ada.Text_IO.Standard_Error);
   end Log;

   type Tracker_Component_Access is access all Tracker_Component'Class;

   function To_Tracker is new Ada.Unchecked_Conversion
     (Source => System.Address,
      Target => Tracker_Component_Access);

   function Default_Content_Type(This : Tracker_Component) return String is
      pragma Unreferenced (This);
   begin
      return Provided_Tobj.Json_Content_Type;
   end Default_Content_Type;

   procedure Set_Content_Type
     (This         : in out Tracker_Component;
      Content_Type : String) is
   begin
      This.Selected_Content_Type := To_Unbounded_String (Content_Type);
   end Set_Content_Type;

   function Content_Type(This : Tracker_Component) return String is
   begin
      if Length (This.Selected_Content_Type) = 0 then
         return Default_Content_Type (This);
      end if;

      return To_String (This.Selected_Content_Type);
   end Content_Type;

   function Has_Pending(This : Tracker_Component) return Boolean is
   begin
      return This.Pending_Count > 0;
   end Has_Pending;

   procedure Drain_Pending
     (This  : in out Tracker_Component;
      Ids   : out Id_Array;
      Count : out Pending_Index)
   is
   begin
      Count := This.Pending_Count;
      for I in 1 .. This.Pending_Count loop
         Ids (I) := This.Pending_Ids (I);
      end loop;
      This.Pending_Count := 0;
   end Drain_Pending;

   procedure Note_Facts_Sent
     (This  : in out Tracker_Component;
      Count : Pending_Index) is
   begin
      This.Facts_Sent_Count := This.Facts_Sent_Count + Count;
   end Note_Facts_Sent;

   function Facts_Sent(This : Tracker_Component) return Natural is
   begin
      return This.Facts_Sent_Count;
   end Facts_Sent;

   function Interest_Id(This : Tracker_Component) return String is
   begin
      return To_String (This.Interest_Id_Value);
   end Interest_Id;

   function Interest_Id_Ready(This : Tracker_Component) return Boolean is
   begin
      return This.Interest_Id_Is_Ready;
   end Interest_Id_Ready;

   procedure On_Entity_Matches
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Msg       : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address);
   pragma Convention (C, On_Entity_Matches);

   overriding procedure On_Configure(This : in out Tracker_Component) is
   begin
      Provided_Tobj.Subscribe_Entity_Matches
        (Container    => Pcl_Component.Handle (This),
         Callback     => On_Entity_Matches'Unrestricted_Access,
         User_Data    => This'Address,
         Content_Type => Content_Type (This));
   end On_Configure;

   procedure On_Entity_Matches
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Msg       : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address)
   is
      pragma Unreferenced (Container);
      This : constant Tracker_Component_Access := To_Tracker (User_Data);
   begin
      if This = null
        or else Msg = null
        or else Msg.Data = System.Null_Address
        or else Msg.Size = 0
      then
         return;
      end if;

      declare
         Matches : constant Provided_Tobj.Object_Match_Array :=
           Provided_Tobj.Decode_Entity_Matches (Msg);
      begin
         for I in Matches'Range loop
            declare
               Id : constant String := To_String (Matches (I).Id);
            begin
               if Id /= "" and then This.Pending_Count < Max_Pending then
                  This.Pending_Count := This.Pending_Count + 1;
                  This.Pending_Ids (This.Pending_Count) :=
                    To_Unbounded_String (Id);
                  Log ("entity queued for world-fact: id=" & Id);
               end if;
            end;
         end loop;
      end;
   end On_Entity_Matches;

   procedure On_Update_State_Response
     (Resp      : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address)
   is
      pragma Unreferenced (Resp, User_Data);
   begin
      Log ("state.update_state response received");
   end On_Update_State_Response;

   procedure On_Create_Req_Response
     (Resp      : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address)
   is
      This : constant Tracker_Component_Access := To_Tracker (User_Data);
   begin
      if This = null then
         return;
      end if;

      if Resp /= null and then Resp.Data /= System.Null_Address
        and then Resp.Size > 0
      then
         This.Interest_Id_Value :=
           Provided_Tobj.Decode_Create_Requirement_Response (Resp);
         Log ("create_requirement response: interest_id=" &
              To_String (This.Interest_Id_Value));
      end if;
      This.Interest_Id_Is_Ready := True;
   end On_Create_Req_Response;

end Bridge_Entity_Tracker;
