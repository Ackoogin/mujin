--  tobj_interest_client.adb
--
--  Component body: subscribes to entity_matches, invokes create_requirement.
--  Uses generated service bindings and Json_Codec for all serialisation.

with Ada.Text_IO;
with Ada.Unchecked_Conversion;
with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Interfaces.C;
with Interfaces.C.Strings;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Services.Tactical_Objects.Provided;
with Pyramid.Services.Tactical_Objects.Flatbuffers_Codec;
with Pyramid.Services.Tactical_Objects.Json_Codec;
with Pyramid.Services.Tactical_Objects.Wire_Types;
with System;

package body Tobj_Interest_Client is

   package Provided renames Pyramid.Services.Tactical_Objects.Provided;
   package Flat     renames Pyramid.Services.Tactical_Objects.Flatbuffers_Codec;
   package Codec    renames Pyramid.Services.Tactical_Objects.Json_Codec;
   package Wire     renames Pyramid.Services.Tactical_Objects.Wire_Types;

   use type Interfaces.C.unsigned;
   use type Interfaces.C.Strings.chars_ptr;
   use type System.Address;

   procedure Log (Msg : String) is
   begin
      Ada.Text_IO.Put_Line (Ada.Text_IO.Standard_Error,
                            "[interest_client] " & Msg);
      Ada.Text_IO.Flush (Ada.Text_IO.Standard_Error);
   end Log;

   function To_Address is new
     Ada.Unchecked_Conversion (Interfaces.C.Strings.chars_ptr, System.Address);

   function Decode_Wire_Payload
     (Msg : access constant Pcl_Bindings.Pcl_Msg) return String
   is
      Payload : constant String := Provided.Msg_To_String (Msg.Data, Msg.Size);
   begin
      if Payload'Length >= 4
        and then Payload (Payload'First .. Payload'First + 3) = "PWFB"
      then
         return Flat.Decode_Payload (Payload);
      end if;
      return Payload;
   end Decode_Wire_Payload;

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
         Body_Str : constant String := Decode_Wire_Payload (Msg);
         Matches  : constant Wire.Entity_Match_Array :=
           Codec.Entity_Matches_From_Json (Body_Str);
      begin
         Log ("standard.entity_matches: " & Body_Str);
         for I in Matches'Range loop
            Log ("  entity[" & Natural'Image (I) & "]"
                 & " id="       & To_String (Matches (I).Object_Id)
                 & " identity=" & Codec.Standard_Identity_To_String
                                    (Matches (I).Identity));
            Matches_Received := Matches_Received + 1;
            if Matches (I).Identity = Identity_Hostile then
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
            Body_Str : constant String := Decode_Wire_Payload (Resp);
            R        : constant Wire.Create_Requirement_Response :=
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
      Policy      : Pyramid.Data_Model.Common.Types.Data_Policy;
      Identity    : Pyramid.Data_Model.Common.Types.Standard_Identity;
      Dimension   : Pyramid.Data_Model.Common.Types.Battle_Dimension :=
                     Pyramid.Data_Model.Common.Types.Dimension_Unspecified;
      Min_Lat_Rad : Long_Float := 0.0;
      Max_Lat_Rad : Long_Float := 0.0;
      Min_Lon_Rad : Long_Float := 0.0;
      Max_Lon_Rad : Long_Float := 0.0)
   is
      Req : Wire.Create_Requirement_Request;
      use type Pcl_Bindings.Pcl_Status;
   begin
      Req.Policy      := Policy;
      Req.Identity    := Identity;
      Req.Dimension   := Dimension;
      Req.Min_Lat_Rad := Min_Lat_Rad;
      Req.Max_Lat_Rad := Max_Lat_Rad;
      Req.Min_Lon_Rad := Min_Lon_Rad;
      Req.Max_Lon_Rad := Max_Lon_Rad;

      Log ("create_requirement request (typed)");
      declare
         Json_Payload : constant String := Codec.To_Json (Req);
         Payload      : constant String :=
           (if To_String (Content_Type) = Flat.Content_Type
            then Flat.Encode_Payload (Json_Payload)
            else Json_Payload);
         Req_C  : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Payload);
         Svc_C  : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Provided.Svc_Create_Requirement);
         Msg    : aliased Pcl_Bindings.Pcl_Msg;
         Status : Pcl_Bindings.Pcl_Status;
      begin
         Msg.Data      := To_Address (Req_C);
         Msg.Size      := Interfaces.C.unsigned (Payload'Length);
         Msg.Type_Name := Interfaces.C.Strings.New_String
           (To_String (Content_Type));
         Status := Pcl_Bindings.Invoke_Remote_Async
           (Transport, Svc_C, Msg'Access,
            On_Create_Requirement_Response'Unrestricted_Access,
            System.Null_Address);
         if Status /= Pcl_Bindings.PCL_OK then
            Log ("create_requirement invoke failed");
            Svc_Response_Ready := True;
         end if;
         Interfaces.C.Strings.Free (Req_C);
         Interfaces.C.Strings.Free (Svc_C);
         Interfaces.C.Strings.Free (Msg.Type_Name);
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
