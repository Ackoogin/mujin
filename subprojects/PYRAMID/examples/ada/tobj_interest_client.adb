--  tobj_interest_client.adb
--
--  Component body: subscribes to entity_matches, invokes create_requirement.
--  Uses proto-native tactical codecs for both RPC and standard topic payloads.

with Ada.Text_IO;
with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Interfaces.C;
with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Tactical.Types;  use Pyramid.Data_Model.Tactical.Types;
with Pyramid.Services.Tactical_Objects.Provided;
with System;

package body Tobj_Interest_Client is

   package Provided renames Pyramid.Services.Tactical_Objects.Provided;

   use type Interfaces.C.unsigned;
   use type System.Address;

   procedure Log (Msg : String) is
   begin
      Ada.Text_IO.Put_Line (Ada.Text_IO.Standard_Error,
                            "[interest_client] " & Msg);
      Ada.Text_IO.Flush (Ada.Text_IO.Standard_Error);
   end Log;

   -- -- On_Configure ----------------------------------------------------------

   function On_Configure
     (Container : Pcl_Bindings.Pcl_Container_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (User_Data);
   begin
      Provided.Subscribe_Entity_Matches
        (Container => Container,
         Callback  => On_Entity_Matches'Unrestricted_Access,
         Content_Type => To_String (Content_Type));
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
      if Msg.Data = System.Null_Address or else Msg.Size = 0 then
         return;
      end if;

      declare
         Matches  : constant Provided.Object_Match_Array :=
           Provided.Decode_Entity_Matches (Msg);
      begin
         Log ("standard.entity_matches received");
         for I in Matches'Range loop
            Log ("  entity[" & Natural'Image (I) & "]"
                 & " id=" & To_String (Matches (I).Id)
                 & " matching_object_id=" &
                   To_String (Matches (I).Matching_Object_Id));
            Matches_Received := Matches_Received + 1;
            Found_Hostile_Entity := True;
         end loop;
      end;
   end On_Entity_Matches;

   -- -- On_Create_Requirement_Response ----------------------------------------

   procedure On_Create_Requirement_Response
     (Resp      : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address)
   is
      pragma Unreferenced (User_Data);
   begin
      if Resp /= null and then
         Resp.Data /= System.Null_Address and then
         Resp.Size > 0
      then
         declare
            Interest_Id : constant Identifier :=
              Provided.Decode_Object_Of_Interest_Create_Requirement_Response
                (Resp);
         begin
            Log ("create_requirement response id: " & To_String (Interest_Id));
            if Interest_Id /= Null_Unbounded_String then
               Interest_Id_Received := True;
            end if;
         end;
      end if;
      Svc_Response_Ready := True;
   end On_Create_Requirement_Response;

   -- -- Send_Create_Requirement -----------------------------------------------

   procedure Send_Create_Requirement
     (Exec        : Pcl_Bindings.Pcl_Executor_Access;
      Policy      : Pyramid.Data_Model.Common.Types.Data_Policy;
      Identity    : Pyramid.Data_Model.Common.Types.Standard_Identity;
      Dimension   : Pyramid.Data_Model.Common.Types.Battle_Dimension :=
                     Pyramid.Data_Model.Common.Types.Dimension_Unspecified;
      Min_Lat_Rad : Long_Float := 0.0;
      Max_Lat_Rad : Long_Float := 0.0;
      Min_Lon_Rad : Long_Float := 0.0;
      Max_Lon_Rad : Long_Float := 0.0)
   is
      pragma Unreferenced (Identity);
      Req : Object_Interest_Requirement;
      use type Pcl_Bindings.Pcl_Status;
   begin
      Req.Source      := Source_Local;
      Req.Policy      := Policy;
      Req.Dimension   := new Dimension_Array'(1 => Dimension);
      if Min_Lat_Rad = Max_Lat_Rad and then Min_Lon_Rad = Max_Lon_Rad then
         Req.Has_Val_Point := True;
         Req.Val_Point.Position.Latitude := Min_Lat_Rad;
         Req.Val_Point.Position.Longitude := Min_Lon_Rad;
      else
         Req.Has_Val_Poly_Area := True;
         Req.Val_Poly_Area.Points := new Points_Array'
           (1 => (Latitude => Min_Lat_Rad, Longitude => Min_Lon_Rad),
            2 => (Latitude => Min_Lat_Rad, Longitude => Max_Lon_Rad),
            3 => (Latitude => Max_Lat_Rad, Longitude => Max_Lon_Rad),
            4 => (Latitude => Max_Lat_Rad, Longitude => Min_Lon_Rad));
      end if;

      Log ("create_requirement request (proto-native typed)");
      declare
         Status : Pcl_Bindings.Pcl_Status;
      begin
         Provided.Invoke_Object_Of_Interest_Create_Requirement
           (Executor     => Exec,
            Request      => Req,
            Callback     => On_Create_Requirement_Response'Unrestricted_Access,
            Content_Type => To_String (Content_Type));
         Status := Pcl_Bindings.PCL_OK;
         if Status /= Pcl_Bindings.PCL_OK then
            Log ("create_requirement invoke failed");
            Svc_Response_Ready := True;
         end if;
      end;
   end Send_Create_Requirement;

   -- -- Reset -----------------------------------------------------------------

   procedure Reset is
   begin
      Matches_Received     := 0;
      Found_Hostile_Entity := False;
      Svc_Response_Ready   := False;
      Interest_Id_Received := False;
   end Reset;

end Tobj_Interest_Client;
