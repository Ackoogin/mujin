--  Auto-generated service binding body
--  Package body: Pyramid.Services.Tactical_Objects.Provided

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Ada.Unchecked_Conversion;
with GNATCOLL.JSON;  use GNATCOLL.JSON;
with Interfaces.C.Strings;
with Pcl_Bindings;
with Pcl_Plugins;
with System;
with System.Storage_Elements;
with Pyramid.Data_Model.Common.Types_Codec;  use Pyramid.Data_Model.Common.Types_Codec;
with Pyramid.Data_Model.Tactical.Types_Codec;  use Pyramid.Data_Model.Tactical.Types_Codec;
with Pyramid.Services.Tactical_Objects.Flatbuffers_Codec;

package body Pyramid.Services.Tactical_Objects.Provided is
   use type System.Address;
   use type Interfaces.C.unsigned;
   use type Interfaces.C.Strings.chars_ptr;
   use type Pcl_Bindings.Pcl_Resp_Cb_Access;
   use type Pcl_Bindings.Pcl_Status;
   use type Pcl_Plugins.Pcl_Codec_Const_Access;
   use type Pcl_Plugins.Pcl_Codec_Decode_Access;
   use type Pcl_Plugins.Pcl_Codec_Encode_Access;
   use type Pcl_Plugins.Pcl_Codec_Free_Msg_Access;

   function To_Address is new
     Ada.Unchecked_Conversion (Interfaces.C.Strings.chars_ptr, System.Address);

   type Service_Handlers_Access is access constant Service_Handlers;

   function To_Handlers is new
     Ada.Unchecked_Conversion (System.Address, Service_Handlers_Access);

   function Handler_Address
     (Handlers : access constant Service_Handlers) return System.Address is
   begin
      if Handlers = null then
         return System.Null_Address;
      end if;

      return Handlers.all'Address;
   end Handler_Address;

   function Msg_To_String
     (Data : System.Address;
      Size : Interfaces.C.unsigned) return String
   is
      use System.Storage_Elements;
      type Char_Array is array (1 .. Natural (Size)) of Character;
      pragma Pack (Char_Array);
      Chars : Char_Array;
      for Chars'Address use Data;
      pragma Import (Ada, Chars);
   begin
      return String (Chars);
   end Msg_To_String;

   function Registry_Has_Codec (Content_Type : String) return Boolean is
      Content_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Content_Type);
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
   begin
      if Content_Type = "" then
         Interfaces.C.Strings.Free (Content_C);
         return False;
      end if;
      Codec := Pcl_Plugins.Pcl_Codec_Registry_Get
        (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C);
      Interfaces.C.Strings.Free (Content_C);
      return Codec /= null;
   exception
      when others =>
         if Content_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Content_C);
         end if;
         return False;
   end Registry_Has_Codec;

   function Try_Registry_Encode
     (Content_Type : String;
      Schema_Id    : String;
      Value        : System.Address;
      Wire         : out Unbounded_String) return Boolean
   is
      Content_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Content_Type);
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Schema_Id);
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Msg   : aliased Pcl_Bindings.Pcl_Msg :=
        (Data      => System.Null_Address,
         Size      => 0,
         Type_Name => Interfaces.C.Strings.Null_Ptr);
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
   begin
      Wire := Null_Unbounded_String;
      if Content_Type = "" then
         Interfaces.C.Strings.Free (Content_C);
         Interfaces.C.Strings.Free (Schema_C);
         return False;
      end if;
      Codec := Pcl_Plugins.Pcl_Codec_Registry_Get
        (Pcl_Plugins.Pcl_Codec_Registry_Default, Content_C);
      if Codec = null or else Codec.all.Encode = null then
         Interfaces.C.Strings.Free (Content_C);
         Interfaces.C.Strings.Free (Schema_C);
         return False;
      end if;
      Status := Codec.all.Encode.all
        (Codec.all.Codec_Ctx, Schema_C, Value, Msg'Access);
      if Status = Pcl_Bindings.PCL_OK then
         if Msg.Data /= System.Null_Address and then Msg.Size > 0 then
            Wire := To_Unbounded_String (Msg_To_String (Msg.Data, Msg.Size));
         end if;
         if Codec.all.Free_Msg /= null then
            Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
         end if;
         Interfaces.C.Strings.Free (Content_C);
         Interfaces.C.Strings.Free (Schema_C);
         return True;
      end if;
      if Msg.Data /= System.Null_Address and then Codec.all.Free_Msg /= null then
         Codec.all.Free_Msg.all (Codec.all.Codec_Ctx, Msg'Access);
      end if;
      Interfaces.C.Strings.Free (Content_C);
      Interfaces.C.Strings.Free (Schema_C);
      return False;
   exception
      when others =>
         if Content_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Content_C);
         end if;
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         return False;
   end Try_Registry_Encode;

   function Try_Registry_Decode
     (Msg       : access constant Pcl_Bindings.Pcl_Msg;
      Schema_Id : String;
      Value     : System.Address) return Boolean
   is
      Schema_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Schema_Id);
      Codec : Pcl_Plugins.Pcl_Codec_Const_Access := null;
      Status : Pcl_Bindings.Pcl_Status := Pcl_Bindings.PCL_ERR_NOT_FOUND;
   begin
      if Msg = null or else Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Schema_C);
         return False;
      end if;
      Codec := Pcl_Plugins.Pcl_Codec_Registry_Get
        (Pcl_Plugins.Pcl_Codec_Registry_Default, Msg.Type_Name);
      if Codec = null or else Codec.all.Decode = null then
         Interfaces.C.Strings.Free (Schema_C);
         return False;
      end if;
      Status := Codec.all.Decode.all
        (Codec.all.Codec_Ctx, Schema_C, Msg, Value);
      Interfaces.C.Strings.Free (Schema_C);
      return Status = Pcl_Bindings.PCL_OK;
   exception
      when others =>
         if Schema_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Schema_C);
         end if;
         return False;
   end Try_Registry_Decode;

   function Try_Registry_Decode_Raw
     (Content_Type : String;
      Data         : System.Address;
      Size         : Natural;
      Schema_Id    : String;
      Value        : System.Address) return Boolean
   is
      Type_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Content_Type);
      Msg : aliased Pcl_Bindings.Pcl_Msg :=
        (Data      => Data,
         Size      => Interfaces.C.unsigned (Size),
         Type_Name => Type_C);
      Ok : Boolean := False;
   begin
      Ok := Try_Registry_Decode (Msg'Access, Schema_Id, Value);
      Interfaces.C.Strings.Free (Type_C);
      return Ok;
   exception
      when others =>
         if Type_C /= Interfaces.C.Strings.Null_Ptr then
            Interfaces.C.Strings.Free (Type_C);
         end if;
         return False;
   end Try_Registry_Decode_Raw;

   package Flatbuffers_Codec renames Pyramid.Services.Tactical_Objects.Flatbuffers_Codec;

   function Supports_Content_Type (Content_Type : String) return Boolean is
   begin
      return Content_Type = ""
        or else Content_Type = Json_Content_Type
        or else Content_Type = Flatbuffers_Content_Type
        or else Registry_Has_Codec (Content_Type);
   end Supports_Content_Type;

   function Message_Content_Type
     (Msg : access constant Pcl_Bindings.Pcl_Msg) return String is
   begin
      if Msg = null or else Msg.Type_Name = Interfaces.C.Strings.Null_Ptr then
         return Json_Content_Type;
      end if;
      return Interfaces.C.Strings.Value (Msg.Type_Name);
   end Message_Content_Type;

   function Decode_Identifier_Payload (Payload : String) return Identifier is
   begin
      declare
         J : constant JSON_Value := Read (Payload);
      begin
         if J.Kind = JSON_String_Type then
            return To_Unbounded_String (String'(UTF8_String'(Get (J))));
         elsif J.Kind = JSON_Object_Type and then Has_Field (J, "uuid") then
            return To_Unbounded_String (String'(UTF8_String'(Get (J, "uuid"))));
         end if;
      exception
         when others =>
            null;
      end;
      return To_Unbounded_String (Payload);
   end Decode_Identifier_Payload;

   function Decode_Entity_Matches
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Object_Match_Array
   is
      Empty : Object_Match_Array (1 .. 0);
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return Empty;
      end if;

      declare
         Json_Payload : constant String :=
           (if Content_Type = "" or else Content_Type = Json_Content_Type
            then Payload
            elsif Content_Type = Flatbuffers_Content_Type
            then Flatbuffers_Codec.From_Binary_object_match_array (Payload)
            else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         R : constant Read_Result := Read (Json_Payload);
      begin
         if not R.Success or else R.Value.Kind /= JSON_Array_Type then
            return Empty;
         end if;
         declare
            Arr    : constant JSON_Array := Get (R.Value);
            Result : Object_Match_Array (1 .. GNATCOLL.JSON.Length (Arr));
         begin
            for I in 1 .. GNATCOLL.JSON.Length (Arr) loop
               Result (I) := From_Json (Write (Get (Arr, I)), null);
            end loop;
            return Result;
         end;
      end;
   end Decode_Entity_Matches;

   function Decode_Evidence_Requirements
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Object_Evidence_Requirement
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return From_Json ("{}", null);
      end if;

      declare
         Result : Object_Evidence_Requirement;
      begin
         if Try_Registry_Decode (Msg, "ObjectEvidenceRequirement", Result'Address) then
            return Result;
         end if;
      end;

      if Content_Type = "" or else Content_Type = Json_Content_Type then
         return From_Json (Payload, null);
      elsif Content_Type = Flatbuffers_Content_Type then
         return Flatbuffers_Codec.From_Binary_object_evidence_requirement (Payload, null);
      end if;
      raise Constraint_Error with "Unsupported content type: " & Content_Type;
   end Decode_Evidence_Requirements;

   function Decode_Matching_Objects_Read_Match_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Object_Match_Array
   is
      Empty : Object_Match_Array (1 .. 0);
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return Empty;
      end if;

      declare
         Json_Payload : constant String :=
           (if Content_Type = "" or else Content_Type = Json_Content_Type
            then Payload
            elsif Content_Type = Flatbuffers_Content_Type
            then Flatbuffers_Codec.From_Binary_Object_Match_Array (Payload)
            else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         R : constant Read_Result := Read (Json_Payload);
      begin
         if not R.Success or else R.Value.Kind /= JSON_Array_Type then
            return Empty;
         end if;
         declare
            Arr    : constant JSON_Array := Get (R.Value);
            Result : Object_Match_Array (1 .. GNATCOLL.JSON.Length (Arr));
         begin
            for I in 1 .. GNATCOLL.JSON.Length (Arr) loop
               Result (I) := From_Json (Write (Get (Arr, I)), null);
            end loop;
            return Result;
         end;
      end;
   end Decode_Matching_Objects_Read_Match_Response;

   function Decode_Object_Of_Interest_Create_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Identifier
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return Null_Unbounded_String;
      end if;

      declare
         Result : Identifier;
      begin
         if Try_Registry_Decode (Msg, "Identifier", Result'Address) then
            return Result;
         end if;
      end;

      if Content_Type = "" or else Content_Type = Json_Content_Type then
         return Decode_Identifier_Payload (Payload);
      elsif Content_Type = Flatbuffers_Content_Type then
         return Flatbuffers_Codec.From_Binary_Identifier (Payload, null);
      end if;
      raise Constraint_Error with "Unsupported content type: " & Content_Type;
   end Decode_Object_Of_Interest_Create_Requirement_Response;

   function Decode_Object_Of_Interest_Read_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Object_Interest_Requirement_Array
   is
      Empty : Object_Interest_Requirement_Array (1 .. 0);
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return Empty;
      end if;

      declare
         Json_Payload : constant String :=
           (if Content_Type = "" or else Content_Type = Json_Content_Type
            then Payload
            elsif Content_Type = Flatbuffers_Content_Type
            then Flatbuffers_Codec.From_Binary_Object_Interest_Requirement_Array (Payload)
            else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         R : constant Read_Result := Read (Json_Payload);
      begin
         if not R.Success or else R.Value.Kind /= JSON_Array_Type then
            return Empty;
         end if;
         declare
            Arr    : constant JSON_Array := Get (R.Value);
            Result : Object_Interest_Requirement_Array (1 .. GNATCOLL.JSON.Length (Arr));
         begin
            for I in 1 .. GNATCOLL.JSON.Length (Arr) loop
               Result (I) := From_Json (Write (Get (Arr, I)), null);
            end loop;
            return Result;
         end;
      end;
   end Decode_Object_Of_Interest_Read_Requirement_Response;

   function Decode_Object_Of_Interest_Update_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return From_Json ("{}", null);
      end if;

      declare
         Result : Ack;
      begin
         if Try_Registry_Decode (Msg, "Ack", Result'Address) then
            return Result;
         end if;
      end;

      if Content_Type = "" or else Content_Type = Json_Content_Type then
         return From_Json (Payload, null);
      elsif Content_Type = Flatbuffers_Content_Type then
         return Flatbuffers_Codec.From_Binary_Ack (Payload, null);
      end if;
      raise Constraint_Error with "Unsupported content type: " & Content_Type;
   end Decode_Object_Of_Interest_Update_Requirement_Response;

   function Decode_Object_Of_Interest_Delete_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Ack
   is
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return From_Json ("{}", null);
      end if;

      declare
         Result : Ack;
      begin
         if Try_Registry_Decode (Msg, "Ack", Result'Address) then
            return Result;
         end if;
      end;

      if Content_Type = "" or else Content_Type = Json_Content_Type then
         return From_Json (Payload, null);
      elsif Content_Type = Flatbuffers_Content_Type then
         return Flatbuffers_Codec.From_Binary_Ack (Payload, null);
      end if;
      raise Constraint_Error with "Unsupported content type: " & Content_Type;
   end Decode_Object_Of_Interest_Delete_Requirement_Response;

   function Decode_Specific_Object_Detail_Read_Detail_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Object_Detail_Array
   is
      Empty : Object_Detail_Array (1 .. 0);
      Payload : constant String :=
        (if Msg = null or else Msg.Data = System.Null_Address
         then ""
         else Msg_To_String (Msg.Data, Msg.Size));
      Content_Type : constant String := Message_Content_Type (Msg);
   begin
      if Payload = "" then
         return Empty;
      end if;

      declare
         Json_Payload : constant String :=
           (if Content_Type = "" or else Content_Type = Json_Content_Type
            then Payload
            elsif Content_Type = Flatbuffers_Content_Type
            then Flatbuffers_Codec.From_Binary_Object_Detail_Array (Payload)
            else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         R : constant Read_Result := Read (Json_Payload);
      begin
         if not R.Success or else R.Value.Kind /= JSON_Array_Type then
            return Empty;
         end if;
         declare
            Arr    : constant JSON_Array := Get (R.Value);
            Result : Object_Detail_Array (1 .. GNATCOLL.JSON.Length (Arr));
         begin
            for I in 1 .. GNATCOLL.JSON.Length (Arr) loop
               Result (I) := From_Json (Write (Get (Arr, I)), null);
            end loop;
            return Result;
         end;
      end;
   end Decode_Specific_Object_Detail_Read_Detail_Response;

   --  -- Matching_Objects_Service ------------------------------------
   function Default_Handle_Matching_Objects_Read_Match
     (Request : Query) return Object_Match_Array
   is
      pragma Unreferenced (Request);
      Empty : Object_Match_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Matching_Objects_Read_Match;

   --  -- Object_Of_Interest_Service ------------------------------------
   procedure Default_Handle_Object_Of_Interest_Create_Requirement
     (Request  : in  Object_Interest_Requirement;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Unbounded_String;
   end Default_Handle_Object_Of_Interest_Create_Requirement;

   function Default_Handle_Object_Of_Interest_Read_Requirement
     (Request : Query) return Object_Interest_Requirement_Array
   is
      pragma Unreferenced (Request);
      Empty : Object_Interest_Requirement_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Object_Of_Interest_Read_Requirement;

   procedure Default_Handle_Object_Of_Interest_Update_Requirement
     (Request  : in  Object_Interest_Requirement;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Object_Of_Interest_Update_Requirement;

   procedure Default_Handle_Object_Of_Interest_Delete_Requirement
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Object_Of_Interest_Delete_Requirement;

   --  -- Specific_Object_Detail_Service ------------------------------------
   function Default_Handle_Specific_Object_Detail_Read_Detail
     (Request : Query) return Object_Detail_Array
   is
      pragma Unreferenced (Request);
      Empty : Object_Detail_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Specific_Object_Detail_Read_Detail;

   function Service_Matching_Objects_Read_Match
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Matching_Objects_Read_Match);

   function Service_Object_Of_Interest_Create_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Object_Of_Interest_Create_Requirement);

   function Service_Object_Of_Interest_Read_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Object_Of_Interest_Read_Requirement);

   function Service_Object_Of_Interest_Update_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Object_Of_Interest_Update_Requirement);

   function Service_Object_Of_Interest_Delete_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Object_Of_Interest_Delete_Requirement);

   function Service_Specific_Object_Detail_Read_Detail
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Specific_Object_Detail_Read_Detail);

   procedure Register_Services
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Handlers  : access constant Service_Handlers := null;
      Content_Type : String := "application/json")
   is
      Handler_Ptr : constant System.Address := Handler_Address (Handlers);
   begin
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Matching_Objects_Read_Match);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Matching_Objects_Read_Match'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Object_Of_Interest_Create_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Object_Of_Interest_Create_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Object_Of_Interest_Read_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Object_Of_Interest_Read_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Object_Of_Interest_Update_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Object_Of_Interest_Update_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Object_Of_Interest_Delete_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Object_Of_Interest_Delete_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Specific_Object_Detail_Read_Detail);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Specific_Object_Detail_Read_Detail'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
   end Register_Services;

   function Service_Matching_Objects_Read_Match
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Matching_Objects_Read_Match,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Matching_Objects_Read_Match;

   function Service_Object_Of_Interest_Create_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Object_Of_Interest_Create_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Object_Of_Interest_Create_Requirement;

   function Service_Object_Of_Interest_Read_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Object_Of_Interest_Read_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Object_Of_Interest_Read_Requirement;

   function Service_Object_Of_Interest_Update_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Object_Of_Interest_Update_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Object_Of_Interest_Update_Requirement;

   function Service_Object_Of_Interest_Delete_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Object_Of_Interest_Delete_Requirement,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Object_Of_Interest_Delete_Requirement;

   function Service_Specific_Object_Detail_Read_Detail
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (Self, Ctx);
      Handlers_Ptr : constant Service_Handlers_Access :=
        (if User_Data = System.Null_Address then null else To_Handlers (User_Data));
      Req_Type  : constant String :=
        (if Request.Type_Name = Interfaces.C.Strings.Null_Ptr
         then "application/json"
         else Interfaces.C.Strings.Value (Request.Type_Name));
      Resp_Buf  : System.Address := System.Null_Address;
      Resp_Size : Natural := 0;
   begin
      Dispatch
        (Handlers      => Handlers_Ptr,
         Channel       => Ch_Specific_Object_Detail_Read_Detail,
         Request_Buf   => Request.Data,
         Request_Size  => Natural (Request.Size),
         Content_Type  => Req_Type,
         Response_Buf  => Resp_Buf,
         Response_Size => Resp_Size);
      Response.Data := Resp_Buf;
      Response.Size := Interfaces.C.unsigned (Resp_Size);
      Response.Type_Name :=
        Interfaces.C.Strings.New_String (Req_Type);
      return Pcl_Bindings.PCL_OK;
   exception
      when others =>
         Response.Data := System.Null_Address;
         Response.Size := 0;
         Response.Type_Name := Interfaces.C.Strings.Null_Ptr;
         return Pcl_Bindings.PCL_ERR_INVALID;
   end Service_Specific_Object_Detail_Read_Detail;

   --  -- PCL binding implementations -------------------------------

   procedure Subscribe_Entity_Matches
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Callback  : Pcl_Bindings.Pcl_Sub_Callback_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      Topic  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Topic_Entity_Matches);
      Type_N : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Content_Type);
      Port   : Pcl_Bindings.Pcl_Port_Access;
      pragma Unreferenced (Port);
   begin
      Port := Pcl_Bindings.Add_Subscriber
        (Container => Container,
         Topic     => Topic,
         Type_Name => Type_N,
         Callback  => Callback,
         User_Data => User_Data);
      Interfaces.C.Strings.Free (Topic);
      Interfaces.C.Strings.Free (Type_N);
   end Subscribe_Entity_Matches;

   procedure Subscribe_Evidence_Requirements
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Callback  : Pcl_Bindings.Pcl_Sub_Callback_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      Topic  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Topic_Evidence_Requirements);
      Type_N : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Content_Type);
      Port   : Pcl_Bindings.Pcl_Port_Access;
      pragma Unreferenced (Port);
   begin
      Port := Pcl_Bindings.Add_Subscriber
        (Container => Container,
         Topic     => Topic,
         Type_Name => Type_N,
         Callback  => Callback,
         User_Data => User_Data);
      Interfaces.C.Strings.Free (Topic);
      Interfaces.C.Strings.Free (Type_N);
   end Subscribe_Evidence_Requirements;

   procedure Invoke_Matching_Objects_Read_Match
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         if Try_Registry_Encode
           (Content_Type, "Query", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;

         declare
            Json_Payload : constant String := To_Json (Request);
         begin
            return
              (if Content_Type = "" or else Content_Type = "application/json"
               then Json_Payload
               elsif Content_Type = "application/flatbuffers"
               then Flatbuffers_Codec.To_Binary_Query (Request)
               else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         end;
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Matching_Objects_Read_Match);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Matching_Objects_Read_Match;

   procedure Invoke_Object_Of_Interest_Create_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Object_Interest_Requirement;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         if Try_Registry_Encode
           (Content_Type, "ObjectInterestRequirement", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;

         declare
            Json_Payload : constant String := To_Json (Request);
         begin
            return
              (if Content_Type = "" or else Content_Type = "application/json"
               then Json_Payload
               elsif Content_Type = "application/flatbuffers"
               then Flatbuffers_Codec.To_Binary_Object_Interest_Requirement (Request)
               else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         end;
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Object_Of_Interest_Create_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Object_Of_Interest_Create_Requirement;

   procedure Invoke_Object_Of_Interest_Read_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         if Try_Registry_Encode
           (Content_Type, "Query", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;

         declare
            Json_Payload : constant String := To_Json (Request);
         begin
            return
              (if Content_Type = "" or else Content_Type = "application/json"
               then Json_Payload
               elsif Content_Type = "application/flatbuffers"
               then Flatbuffers_Codec.To_Binary_Query (Request)
               else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         end;
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Object_Of_Interest_Read_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Object_Of_Interest_Read_Requirement;

   procedure Invoke_Object_Of_Interest_Update_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Object_Interest_Requirement;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         if Try_Registry_Encode
           (Content_Type, "ObjectInterestRequirement", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;

         declare
            Json_Payload : constant String := To_Json (Request);
         begin
            return
              (if Content_Type = "" or else Content_Type = "application/json"
               then Json_Payload
               elsif Content_Type = "application/flatbuffers"
               then Flatbuffers_Codec.To_Binary_Object_Interest_Requirement (Request)
               else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         end;
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Object_Of_Interest_Update_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Object_Of_Interest_Update_Requirement;

   procedure Invoke_Object_Of_Interest_Delete_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         if Try_Registry_Encode
           (Content_Type, "Identifier", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;

         declare
            Json_Payload : constant String := To_String (Request);
         begin
            return
              (if Content_Type = "" or else Content_Type = "application/json"
               then Json_Payload
               elsif Content_Type = "application/flatbuffers"
               then Flatbuffers_Codec.To_Binary_Identifier (Request)
               else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         end;
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Object_Of_Interest_Delete_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Object_Of_Interest_Delete_Requirement;

   procedure Invoke_Specific_Object_Detail_Read_Detail
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      function Build_Payload return String is
         Registry_Payload : Unbounded_String := Null_Unbounded_String;
      begin
         if Try_Registry_Encode
           (Content_Type, "Query", Request'Address,
            Registry_Payload)
         then
            return To_String (Registry_Payload);
         end if;

         declare
            Json_Payload : constant String := To_Json (Request);
         begin
            return
              (if Content_Type = "" or else Content_Type = "application/json"
               then Json_Payload
               elsif Content_Type = "application/flatbuffers"
               then Flatbuffers_Codec.To_Binary_Query (Request)
               else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         end;
      end Build_Payload;
      Payload : constant String := Build_Payload;
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Specific_Object_Detail_Read_Detail);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = "" or else Content_Type = "application/json" then
         Req_C := Interfaces.C.Strings.New_String (Payload);
         Msg.Data := To_Address (Req_C);
      else
         Msg.Data :=
           (if Payload_Bytes'Length = 0
            then System.Null_Address
            else Payload_Bytes (Payload_Bytes'First)'Address);
      end if;
      Msg.Size      := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Interfaces.C.Strings.New_String (Content_Type);
      Status := Pcl_Bindings.Invoke_Async
        (Executor, Svc_C, Msg'Access, Callback, User_Data);
      if Req_C /= Interfaces.C.Strings.Null_Ptr then
         Interfaces.C.Strings.Free (Req_C);
      end if;
      Interfaces.C.Strings.Free (Svc_C);
      Interfaces.C.Strings.Free (Msg.Type_Name);
   end Invoke_Specific_Object_Detail_Read_Detail;

   procedure Copy_To_Buf
     (S    : in  String;
      Buf  : out System.Address;
      Size : out Natural)
   is
      C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (S);
   begin
      Buf  := To_Address (C);
      Size := S'Length;
   end Copy_To_Buf;

   procedure Dispatch
     (Handlers      : access constant Service_Handlers := null;
      Channel       : in  Service_Channel;
      Request_Buf   : in  System.Address;
      Request_Size  : in  Natural;
      Content_Type  : in  String := "application/json";
      Response_Buf  : out System.Address;
      Response_Size : out Natural)
   is
      Request_Payload : constant String :=
        Msg_To_String (Request_Buf, Interfaces.C.unsigned (Request_Size));
   begin
      Response_Buf  := System.Null_Address;
      Response_Size := 0;
      case Channel is
         when Ch_Matching_Objects_Read_Match =>
            declare
               function Decode_Request return Query is
                  Result : Query;
               begin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Query", Result'Address)
                  then
                     return Result;
                  end if;

                  return
                    (if Content_Type = "" or else Content_Type = "application/json"
                     then From_Json (Request_Payload, null)
                     elsif Content_Type = "application/flatbuffers"
                     then Flatbuffers_Codec.From_Binary_Query (Request_Payload, null)
                     else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               end Decode_Request;
               Req : constant Query := Decode_Request;
               Rsp : constant Object_Match_Array :=
                 (if Handlers /= null and then Handlers.On_Matching_Objects_Read_Match /= null
                  then Handlers.On_Matching_Objects_Read_Match.all (Req)
                  else Default_Handle_Matching_Objects_Read_Match (Req));
            begin
               declare
                  use Ada.Strings.Unbounded;
                  Acc : Unbounded_String :=
                    To_Unbounded_String ("[");
               begin
                  for I in Rsp'Range loop
                     if I > Rsp'First then
                        Append (Acc, ",");
                     end if;
                     Append (Acc, To_Json (Rsp (I)));
                  end loop;
                  Append (Acc, "]");
                  Copy_To_Buf
                    ((if Content_Type = "" or else Content_Type = "application/json"
                      then To_String (Acc)
                      elsif Content_Type = "application/flatbuffers"
                      then Flatbuffers_Codec.To_Binary_Object_Match_Array (To_String (Acc))
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Object_Of_Interest_Create_Requirement =>
            declare
               function Decode_Request return Object_Interest_Requirement is
                  Result : Object_Interest_Requirement;
               begin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "ObjectInterestRequirement", Result'Address)
                  then
                     return Result;
                  end if;

                  return
                    (if Content_Type = "" or else Content_Type = "application/json"
                     then From_Json (Request_Payload, null)
                     elsif Content_Type = "application/flatbuffers"
                     then Flatbuffers_Codec.From_Binary_Object_Interest_Requirement (Request_Payload, null)
                     else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               end Decode_Request;
               Req : constant Object_Interest_Requirement := Decode_Request;
               Rsp : Identifier;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Object_Of_Interest_Create_Requirement /= null then
                  Handlers.On_Object_Of_Interest_Create_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Object_Of_Interest_Create_Requirement (Req, Rsp);
               end if;
               if not Try_Registry_Encode
                 (Content_Type, "Identifier", Rsp'Address,
                  Wire_Response)
               then
                  Wire_Response := To_Unbounded_String
                    ((if Content_Type = "" or else Content_Type = "application/json"
                      then To_String (Rsp)
                      elsif Content_Type = "application/flatbuffers"
                      then Flatbuffers_Codec.To_Binary_Identifier (Rsp)
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type));
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_Object_Of_Interest_Read_Requirement =>
            declare
               function Decode_Request return Query is
                  Result : Query;
               begin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Query", Result'Address)
                  then
                     return Result;
                  end if;

                  return
                    (if Content_Type = "" or else Content_Type = "application/json"
                     then From_Json (Request_Payload, null)
                     elsif Content_Type = "application/flatbuffers"
                     then Flatbuffers_Codec.From_Binary_Query (Request_Payload, null)
                     else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               end Decode_Request;
               Req : constant Query := Decode_Request;
               Rsp : constant Object_Interest_Requirement_Array :=
                 (if Handlers /= null and then Handlers.On_Object_Of_Interest_Read_Requirement /= null
                  then Handlers.On_Object_Of_Interest_Read_Requirement.all (Req)
                  else Default_Handle_Object_Of_Interest_Read_Requirement (Req));
            begin
               declare
                  use Ada.Strings.Unbounded;
                  Acc : Unbounded_String :=
                    To_Unbounded_String ("[");
               begin
                  for I in Rsp'Range loop
                     if I > Rsp'First then
                        Append (Acc, ",");
                     end if;
                     Append (Acc, To_Json (Rsp (I)));
                  end loop;
                  Append (Acc, "]");
                  Copy_To_Buf
                    ((if Content_Type = "" or else Content_Type = "application/json"
                      then To_String (Acc)
                      elsif Content_Type = "application/flatbuffers"
                      then Flatbuffers_Codec.To_Binary_Object_Interest_Requirement_Array (To_String (Acc))
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Object_Of_Interest_Update_Requirement =>
            declare
               function Decode_Request return Object_Interest_Requirement is
                  Result : Object_Interest_Requirement;
               begin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "ObjectInterestRequirement", Result'Address)
                  then
                     return Result;
                  end if;

                  return
                    (if Content_Type = "" or else Content_Type = "application/json"
                     then From_Json (Request_Payload, null)
                     elsif Content_Type = "application/flatbuffers"
                     then Flatbuffers_Codec.From_Binary_Object_Interest_Requirement (Request_Payload, null)
                     else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               end Decode_Request;
               Req : constant Object_Interest_Requirement := Decode_Request;
               Rsp : Ack;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Object_Of_Interest_Update_Requirement /= null then
                  Handlers.On_Object_Of_Interest_Update_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Object_Of_Interest_Update_Requirement (Req, Rsp);
               end if;
               if not Try_Registry_Encode
                 (Content_Type, "Ack", Rsp'Address,
                  Wire_Response)
               then
                  Wire_Response := To_Unbounded_String
                    ((if Content_Type = "" or else Content_Type = "application/json"
                      then To_Json (Rsp)
                      elsif Content_Type = "application/flatbuffers"
                      then Flatbuffers_Codec.To_Binary_Ack (Rsp)
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type));
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_Object_Of_Interest_Delete_Requirement =>
            declare
               function Decode_Request return Identifier is
                  Result : Identifier;
               begin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Identifier", Result'Address)
                  then
                     return Result;
                  end if;

                  return
                    (if Content_Type = "" or else Content_Type = "application/json"
                     then To_Unbounded_String (Request_Payload)
                     elsif Content_Type = "application/flatbuffers"
                     then Flatbuffers_Codec.From_Binary_Identifier (Request_Payload, null)
                     else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               end Decode_Request;
               Req : constant Identifier := Decode_Request;
               Rsp : Ack;
               Wire_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Object_Of_Interest_Delete_Requirement /= null then
                  Handlers.On_Object_Of_Interest_Delete_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Object_Of_Interest_Delete_Requirement (Req, Rsp);
               end if;
               if not Try_Registry_Encode
                 (Content_Type, "Ack", Rsp'Address,
                  Wire_Response)
               then
                  Wire_Response := To_Unbounded_String
                    ((if Content_Type = "" or else Content_Type = "application/json"
                      then To_Json (Rsp)
                      elsif Content_Type = "application/flatbuffers"
                      then Flatbuffers_Codec.To_Binary_Ack (Rsp)
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type));
               end if;
               Copy_To_Buf
                 (To_String (Wire_Response),
                  Response_Buf, Response_Size);
            end;
         when Ch_Specific_Object_Detail_Read_Detail =>
            declare
               function Decode_Request return Query is
                  Result : Query;
               begin
                  if Try_Registry_Decode_Raw
                    (Content_Type, Request_Buf, Request_Size,
                     "Query", Result'Address)
                  then
                     return Result;
                  end if;

                  return
                    (if Content_Type = "" or else Content_Type = "application/json"
                     then From_Json (Request_Payload, null)
                     elsif Content_Type = "application/flatbuffers"
                     then Flatbuffers_Codec.From_Binary_Query (Request_Payload, null)
                     else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               end Decode_Request;
               Req : constant Query := Decode_Request;
               Rsp : constant Object_Detail_Array :=
                 (if Handlers /= null and then Handlers.On_Specific_Object_Detail_Read_Detail /= null
                  then Handlers.On_Specific_Object_Detail_Read_Detail.all (Req)
                  else Default_Handle_Specific_Object_Detail_Read_Detail (Req));
            begin
               declare
                  use Ada.Strings.Unbounded;
                  Acc : Unbounded_String :=
                    To_Unbounded_String ("[");
               begin
                  for I in Rsp'Range loop
                     if I > Rsp'First then
                        Append (Acc, ",");
                     end if;
                     Append (Acc, To_Json (Rsp (I)));
                  end loop;
                  Append (Acc, "]");
                  Copy_To_Buf
                    ((if Content_Type = "" or else Content_Type = "application/json"
                      then To_String (Acc)
                      elsif Content_Type = "application/flatbuffers"
                      then Flatbuffers_Codec.To_Binary_Object_Detail_Array (To_String (Acc))
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                    Response_Buf, Response_Size);
               end;
            end;
      end case;
   end Dispatch;

end Pyramid.Services.Tactical_Objects.Provided;
