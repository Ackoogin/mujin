--  tobj_interest_client.adb
--
--  Component body: subscribes to entity_matches, invokes create_requirement.
--  Uses proto-native tactical codecs for RPC payloads and bridge Json_Codec
--  only for standard topic adapters.

with Ada.Text_IO;
with Ada.Unchecked_Conversion;
with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with GNATCOLL.JSON;  use GNATCOLL.JSON;
with Interfaces.C;
with Interfaces.C.Strings;
with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Tactical.Types;  use Pyramid.Data_Model.Tactical.Types;
with Pyramid.Data_Model.Tactical.Types_Codec;
with Pyramid.Services.Tactical_Objects.Provided;
with Pyramid.Services.Tactical_Objects.Flatbuffers_Codec;
with Pyramid.Services.Tactical_Objects.Json_Codec;
with Pyramid.Services.Tactical_Objects.Wire_Types;
with System;

package body Tobj_Interest_Client is

   package Provided renames Pyramid.Services.Tactical_Objects.Provided;
   package Flat     renames Pyramid.Services.Tactical_Objects.Flatbuffers_Codec;
   package Codec    renames Pyramid.Services.Tactical_Objects.Json_Codec;
   package Tactical renames Pyramid.Data_Model.Tactical.Types_Codec;
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

   function Decode_Entity_Matches_Payload
     (Msg : access constant Pcl_Bindings.Pcl_Msg) return String
   is
      Payload : constant String := Provided.Msg_To_String (Msg.Data, Msg.Size);
   begin
      if Msg.Type_Name /= Interfaces.C.Strings.Null_Ptr
        and then Interfaces.C.Strings.Value (Msg.Type_Name) = Flat.Content_Type
      then
         return Flat.From_Binary_Entity_Match_Array (Payload);
      end if;
      return Payload;
   end Decode_Entity_Matches_Payload;

   function Decode_Identifier_Payload (Payload : String) return Identifier is
      J : constant JSON_Value := Read (Payload);
   begin
      return To_Unbounded_String (String'(UTF8_String'(Get (J))));
   exception
      when others =>
         begin
            if Has_Field (J, "uuid") then
               return To_Unbounded_String
                 (String'(UTF8_String'(Get (J, "uuid"))));
            end if;
         exception
            when others =>
               null;
         end;
         return Null_Unbounded_String;
   end Decode_Identifier_Payload;

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
         Body_Str : constant String := Decode_Entity_Matches_Payload (Msg);
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
            Raw_Payload : constant String := Provided.Msg_To_String (Resp.Data, Resp.Size);
            Interest_Id : constant Identifier :=
              (if Resp.Type_Name /= Interfaces.C.Strings.Null_Ptr
                  and then Interfaces.C.Strings.Value (Resp.Type_Name) = Flat.Content_Type
               then Flat.From_Binary_Identifier (Raw_Payload, null)
               else Decode_Identifier_Payload (Raw_Payload));
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
      pragma Unreferenced (Identity, Max_Lat_Rad, Max_Lon_Rad);
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
         Payload      : constant String :=
           (if To_String (Content_Type) = Flat.Content_Type
            then Flat.To_Binary_Object_Interest_Requirement (Req)
            else Tactical.To_Json (Req));
         Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
         Payload_Bytes : aliased constant String := Payload;
         Svc_C  : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Provided.Svc_Create_Requirement);
         Msg    : aliased Pcl_Bindings.Pcl_Msg;
         Status : Pcl_Bindings.Pcl_Status;
      begin
         if To_String (Content_Type) = Flat.Content_Type then
            Log ("create_requirement payload: [flatbuffers " &
                 Integer'Image (Payload_Bytes'Length) & " bytes]");
         else
            Log ("create_requirement payload: " & Payload);
         end if;
         if To_String (Content_Type) = Flat.Content_Type then
            Msg.Data :=
              (if Payload_Bytes'Length = 0
               then System.Null_Address
               else Payload_Bytes (Payload_Bytes'First)'Address);
         else
            Req_C := Interfaces.C.Strings.New_String (Payload);
            Msg.Data := To_Address (Req_C);
        end if;
        Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
        Msg.Type_Name := Interfaces.C.Strings.New_String
          (To_String (Content_Type));
         Status := Pcl_Bindings.Invoke_Async
           (Exec, Svc_C, Msg'Access,
            On_Create_Requirement_Response'Unrestricted_Access,
            System.Null_Address);
         if Status /= Pcl_Bindings.PCL_OK then
            Log ("create_requirement invoke failed");
            Svc_Response_Ready := True;
         end if;
         if Req_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Req_C);
         end if;
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
