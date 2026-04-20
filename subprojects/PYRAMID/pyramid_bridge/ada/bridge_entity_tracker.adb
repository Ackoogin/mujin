--  bridge_entity_tracker.adb

with Ada.Text_IO;
with Ada.Strings.Unbounded;    use Ada.Strings.Unbounded;
with Interfaces.C;
with Interfaces.C.Strings;
with System;
with System.Storage_Elements;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Autonomy.Types;  use Pyramid.Data_Model.Autonomy.Types;
with Pyramid.Services.Tactical_Objects.Provided;
with Pyramid.Services.Autonomy_Backend.Provided;

package body Bridge_Entity_Tracker is

   package Provided_Tobj renames Pyramid.Services.Tactical_Objects.Provided;
   package Provided_Ame  renames Pyramid.Services.Autonomy_Backend.Provided;

   use type Interfaces.C.unsigned;
   use type System.Address;

   procedure Log (Msg : String) is
   begin
      Ada.Text_IO.Put_Line (Ada.Text_IO.Standard_Error,
                            "[pyramid_bridge] " & Msg);
      Ada.Text_IO.Flush (Ada.Text_IO.Standard_Error);
   end Log;

   -- -- Has_Pending -----------------------------------------------------------

   function Has_Pending return Boolean is
   begin
      return Pending_Count > 0;
   end Has_Pending;

   -- -- Drain_Pending ---------------------------------------------------------

   procedure Drain_Pending
     (Ids   : out Id_Array;
      Count : out Pending_Index)
   is
   begin
      Count := Pending_Count;
      for I in 1 .. Pending_Count loop
         Ids (I) := Pending_Ids (I);
      end loop;
      Pending_Count := 0;
   end Drain_Pending;

   -- -- On_Configure ----------------------------------------------------------

   function On_Configure
     (Container : Pcl_Bindings.Pcl_Container_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (User_Data);
   begin
      Provided_Tobj.Subscribe_Entity_Matches
        (Container    => Container,
         Callback     => On_Entity_Matches'Unrestricted_Access,
         Content_Type => Provided_Tobj.Json_Content_Type);
      return Pcl_Bindings.PCL_OK;
   end On_Configure;

   -- -- On_Entity_Matches -----------------------------------------------------

   procedure On_Entity_Matches
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Msg       : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address)
   is
      pragma Unreferenced (Container, User_Data);
   begin
      if Msg = null or else Msg.Data = System.Null_Address
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
               if Id /= "" and then Pending_Count < Max_Pending then
                  Pending_Count := Pending_Count + 1;
                  Pending_Ids (Pending_Count) := To_Unbounded_String (Id);
                  Log ("entity queued for world-fact: id=" & Id);
               end if;
            end;
         end loop;
      end;
   end On_Entity_Matches;

   -- -- On_Update_State_Response ----------------------------------------------

   procedure On_Update_State_Response
     (Resp      : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address)
   is
      pragma Unreferenced (Resp, User_Data);
   begin
      Log ("state.update_state response received");
   end On_Update_State_Response;

end Bridge_Entity_Tracker;
