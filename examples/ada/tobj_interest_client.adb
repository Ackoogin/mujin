--  tobj_interest_client.adb
--
--  Component body: subscribes to entity_matches, invokes create_requirement.
--  Uses generated service bindings and Json_Codec for all serialisation.

with Ada.Text_IO;
with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Interfaces.C;
with Pyramid.Services.Tactical_Objects.Provided;
with Pyramid.Services.Tactical_Objects.Json_Codec;
with System;

package body Tobj_Interest_Client is

   package Provided renames Pyramid.Services.Tactical_Objects.Provided;
   package Codec    renames Pyramid.Services.Tactical_Objects.Json_Codec;

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
         Callback  => On_Entity_Matches'Unrestricted_Access);
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
         Body_Str : constant String :=
           Provided.Msg_To_String (Msg.Data, Msg.Size);
         Matches  : constant Codec.Entity_Match_Array :=
           Codec.Entity_Matches_From_Json (Body_Str);
      begin
         Log ("standard.entity_matches: " & Body_Str);
         for I in Matches'Range loop
            Log ("  entity[" & Natural'Image (I) & "]"
                 & " id="       & To_String (Matches (I).Object_Id)
                 & " identity=" & Codec.Standard_Identity_To_String
                                    (Matches (I).Identity));
            Matches_Received := Matches_Received + 1;
            if Matches (I).Identity =
                 Tactical_Objects_Types.Identity_Hostile
            then
               Found_Hostile_Entity := True;
            end if;
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
            Body_Str : constant String :=
              Provided.Msg_To_String (Resp.Data, Resp.Size);
            R        : constant Codec.Create_Requirement_Response :=
              Codec.From_Json (Body_Str);
         begin
            Log ("create_requirement response: " & Body_Str);
            if R.Interest_Id /= Null_Unbounded_String then
               Interest_Id_Received := True;
            end if;
         end;
      end if;
      Svc_Response_Ready := True;
   end On_Create_Requirement_Response;

   -- -- Send_Create_Requirement -----------------------------------------------

   procedure Send_Create_Requirement
     (Transport   : Pcl_Bindings.Pcl_Socket_Transport_Access;
      Policy      : Tactical_Objects_Types.Data_Policy;
      Identity    : Tactical_Objects_Types.Standard_Identity;
      Dimension   : Tactical_Objects_Types.Battle_Dimension :=
                      Tactical_Objects_Types.Dimension_Unspecified;
      Min_Lat_Rad : Long_Float := 0.0;
      Max_Lat_Rad : Long_Float := 0.0;
      Min_Lon_Rad : Long_Float := 0.0;
      Max_Lon_Rad : Long_Float := 0.0)
   is
      Req : Codec.Create_Requirement_Request;
   begin
      Req.Policy      := Policy;
      Req.Identity    := Identity;
      Req.Dimension   := Dimension;
      Req.Min_Lat_Rad := Min_Lat_Rad;
      Req.Max_Lat_Rad := Max_Lat_Rad;
      Req.Min_Lon_Rad := Min_Lon_Rad;
      Req.Max_Lon_Rad := Max_Lon_Rad;

      declare
         Req_Str : constant String := Codec.To_Json (Req);
      begin
         Log ("create_requirement request: " & Req_Str);
         Provided.Invoke_Create_Requirement
           (Transport => Transport,
            Request   => Req_Str,
            Callback  => On_Create_Requirement_Response'Unrestricted_Access);
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
