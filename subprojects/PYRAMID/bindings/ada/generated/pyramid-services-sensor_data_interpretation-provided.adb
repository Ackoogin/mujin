--  Auto-generated service binding body
--  Package body: Pyramid.Services.Sensor_Data_Interpretation.Provided

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Ada.Unchecked_Conversion;
with GNATCOLL.JSON;  use GNATCOLL.JSON;
with Interfaces.C.Strings;
with System;
with System.Storage_Elements;
with Pyramid.Data_Model.Common.Types_Codec;  use Pyramid.Data_Model.Common.Types_Codec;
with Pyramid.Data_Model.Sensors.Types_Codec;  use Pyramid.Data_Model.Sensors.Types_Codec;
with Pyramid.Services.Sensor_Data_Interpretation.Flatbuffers_Codec;
with Pyramid.Components.Sensor_data_interpretation.Services.Provided.GRPC_Transport;

package body Pyramid.Services.Sensor_Data_Interpretation.Provided is
   use type System.Address;
   use type Interfaces.C.Strings.chars_ptr;
   use type Pcl_Bindings.Pcl_Resp_Cb_Access;

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

   package Flatbuffers_Codec renames Pyramid.Services.Sensor_Data_Interpretation.Flatbuffers_Codec;
   package Grpc_Transport renames Pyramid.Components.Sensor_data_interpretation.Services.Provided.GRPC_Transport;
   Grpc_Channel : Unbounded_String := Null_Unbounded_String;

   function Supports_Content_Type (Content_Type : String) return Boolean is
   begin
      return Content_Type = ""
        or else Content_Type = Json_Content_Type
        or else Content_Type = Flatbuffers_Content_Type
        or else Content_Type = Grpc_Content_Type;
   end Supports_Content_Type;

   procedure Configure_Grpc_Library (Path : String) is
   begin
      Grpc_Transport.Configure_Library (Path);
   end Configure_Grpc_Library;

   procedure Configure_Grpc_Channel (Channel : String) is
   begin
      Grpc_Channel := To_Unbounded_String (Channel);
   end Configure_Grpc_Channel;

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

   procedure Emit_Invoke_Response
     (Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address;
      Payload   : String) is
      Payload_Bytes : aliased constant String := Payload;
      Type_Name : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Json_Content_Type);
      Msg : aliased Pcl_Bindings.Pcl_Msg;
   begin
      Msg.Data :=
        (if Payload_Bytes'Length = 0
         then System.Null_Address
         else Payload_Bytes (Payload_Bytes'First)'Address);
      Msg.Size := Interfaces.C.unsigned (Payload_Bytes'Length);
      Msg.Type_Name := Type_Name;
      if Callback /= null then
         Callback (Msg'Access, User_Data);
      end if;
      Interfaces.C.Strings.Free (Type_Name);
   end Emit_Invoke_Response;

   function Decode_Interpretation_Requirement_Read_Capability_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Capability_Array
   is
      Empty : Capability_Array (1 .. 0);
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
            then Flatbuffers_Codec.From_Binary_Capability_Array (Payload)
            else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         R : constant Read_Result := Read (Json_Payload);
      begin
         if not R.Success or else R.Value.Kind /= JSON_Array_Type then
            return Empty;
         end if;
         declare
            Arr    : constant JSON_Array := Get (R.Value);
            Result : Capability_Array (1 .. GNATCOLL.JSON.Length (Arr));
         begin
            for I in 1 .. GNATCOLL.JSON.Length (Arr) loop
               Result (I) := From_Json (Write (Get (Arr, I)), null);
            end loop;
            return Result;
         end;
      end;
   end Decode_Interpretation_Requirement_Read_Capability_Response;

   function Decode_Interpretation_Requirement_Create_Requirement_Response
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

      if Content_Type = "" or else Content_Type = Json_Content_Type then
         return Decode_Identifier_Payload (Payload);
      elsif Content_Type = Flatbuffers_Content_Type then
         return Flatbuffers_Codec.From_Binary_Identifier (Payload, null);
      end if;
      raise Constraint_Error with "Unsupported content type: " & Content_Type;
   end Decode_Interpretation_Requirement_Create_Requirement_Response;

   function Decode_Interpretation_Requirement_Read_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Interpretation_Requirement_Array
   is
      Empty : Interpretation_Requirement_Array (1 .. 0);
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
            then Flatbuffers_Codec.From_Binary_Interpretation_Requirement_Array (Payload)
            else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         R : constant Read_Result := Read (Json_Payload);
      begin
         if not R.Success or else R.Value.Kind /= JSON_Array_Type then
            return Empty;
         end if;
         declare
            Arr    : constant JSON_Array := Get (R.Value);
            Result : Interpretation_Requirement_Array (1 .. GNATCOLL.JSON.Length (Arr));
         begin
            for I in 1 .. GNATCOLL.JSON.Length (Arr) loop
               Result (I) := From_Json (Write (Get (Arr, I)), null);
            end loop;
            return Result;
         end;
      end;
   end Decode_Interpretation_Requirement_Read_Requirement_Response;

   function Decode_Interpretation_Requirement_Update_Requirement_Response
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

      if Content_Type = "" or else Content_Type = Json_Content_Type then
         return From_Json (Payload, null);
      elsif Content_Type = Flatbuffers_Content_Type then
         return Flatbuffers_Codec.From_Binary_Ack (Payload, null);
      end if;
      raise Constraint_Error with "Unsupported content type: " & Content_Type;
   end Decode_Interpretation_Requirement_Update_Requirement_Response;

   function Decode_Interpretation_Requirement_Delete_Requirement_Response
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

      if Content_Type = "" or else Content_Type = Json_Content_Type then
         return From_Json (Payload, null);
      elsif Content_Type = Flatbuffers_Content_Type then
         return Flatbuffers_Codec.From_Binary_Ack (Payload, null);
      end if;
      raise Constraint_Error with "Unsupported content type: " & Content_Type;
   end Decode_Interpretation_Requirement_Delete_Requirement_Response;

   --  -- Interpretation_Requirement_Service ------------------------------------
   function Default_Handle_Interpretation_Requirement_Read_Capability
     (Request : Query) return Capability_Array
   is
      pragma Unreferenced (Request);
      Empty : Capability_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Interpretation_Requirement_Read_Capability;

   procedure Default_Handle_Interpretation_Requirement_Create_Requirement
     (Request  : in  Interpretation_Requirement;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Unbounded_String;
   end Default_Handle_Interpretation_Requirement_Create_Requirement;

   function Default_Handle_Interpretation_Requirement_Read_Requirement
     (Request : Query) return Interpretation_Requirement_Array
   is
      pragma Unreferenced (Request);
      Empty : Interpretation_Requirement_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Interpretation_Requirement_Read_Requirement;

   procedure Default_Handle_Interpretation_Requirement_Update_Requirement
     (Request  : in  Interpretation_Requirement;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Interpretation_Requirement_Update_Requirement;

   procedure Default_Handle_Interpretation_Requirement_Delete_Requirement
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Interpretation_Requirement_Delete_Requirement;

   function Service_Interpretation_Requirement_Read_Capability
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Interpretation_Requirement_Read_Capability);

   function Service_Interpretation_Requirement_Create_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Interpretation_Requirement_Create_Requirement);

   function Service_Interpretation_Requirement_Read_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Interpretation_Requirement_Read_Requirement);

   function Service_Interpretation_Requirement_Update_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Interpretation_Requirement_Update_Requirement);

   function Service_Interpretation_Requirement_Delete_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Interpretation_Requirement_Delete_Requirement);

   procedure Register_Services
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Handlers  : access constant Service_Handlers := null;
      Content_Type : String := "application/json")
   is
      Handler_Ptr : constant System.Address := Handler_Address (Handlers);
   begin
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Interpretation_Requirement_Read_Capability);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Interpretation_Requirement_Read_Capability'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Interpretation_Requirement_Create_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Interpretation_Requirement_Create_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Interpretation_Requirement_Read_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Interpretation_Requirement_Read_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Interpretation_Requirement_Update_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Interpretation_Requirement_Update_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Interpretation_Requirement_Delete_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Interpretation_Requirement_Delete_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
   end Register_Services;

   function Service_Interpretation_Requirement_Read_Capability
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
         Channel       => Ch_Interpretation_Requirement_Read_Capability,
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
   end Service_Interpretation_Requirement_Read_Capability;

   function Service_Interpretation_Requirement_Create_Requirement
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
         Channel       => Ch_Interpretation_Requirement_Create_Requirement,
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
   end Service_Interpretation_Requirement_Create_Requirement;

   function Service_Interpretation_Requirement_Read_Requirement
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
         Channel       => Ch_Interpretation_Requirement_Read_Requirement,
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
   end Service_Interpretation_Requirement_Read_Requirement;

   function Service_Interpretation_Requirement_Update_Requirement
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
         Channel       => Ch_Interpretation_Requirement_Update_Requirement,
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
   end Service_Interpretation_Requirement_Update_Requirement;

   function Service_Interpretation_Requirement_Delete_Requirement
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
         Channel       => Ch_Interpretation_Requirement_Delete_Requirement,
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
   end Service_Interpretation_Requirement_Delete_Requirement;

   --  -- PCL binding implementations -------------------------------

   procedure Invoke_Interpretation_Requirement_Read_Capability
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      Json_Payload : constant String := To_Json (Request);
      Payload : constant String :=
        (if Content_Type = "" or else Content_Type = "application/json"
         then Json_Payload
         elsif Content_Type = "application/flatbuffers"
         then Flatbuffers_Codec.To_Binary_Query (Request)
         elsif Content_Type = Grpc_Content_Type
         then ""
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Interpretation_Requirement_Read_Capability);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = Grpc_Content_Type then
         if Ada.Strings.Unbounded.Length (Grpc_Channel) = 0 then
            raise Program_Error with "gRPC channel not configured";
         end if;
         declare
            Rsp : constant Grpc_Transport.Capability_Array :=
              Grpc_Transport.Invoke_Interpretation_Requirement_Read_Capability
                (To_String (Grpc_Channel), Request);
            Acc : Unbounded_String := To_Unbounded_String ("[");
         begin
            for I in Rsp'Range loop
               if I > Rsp'First then
                  Append (Acc, ",");
               end if;
               Append (Acc, To_Json (Rsp (I)));
            end loop;
            Append (Acc, "]");
            Emit_Invoke_Response
              (Callback, User_Data, To_String (Acc));
         end;
         return;
      end if;

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
   end Invoke_Interpretation_Requirement_Read_Capability;

   procedure Invoke_Interpretation_Requirement_Create_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Interpretation_Requirement;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      Json_Payload : constant String := To_Json (Request);
      Payload : constant String :=
        (if Content_Type = "" or else Content_Type = "application/json"
         then Json_Payload
         elsif Content_Type = "application/flatbuffers"
         then Flatbuffers_Codec.To_Binary_Interpretation_Requirement (Request)
         elsif Content_Type = Grpc_Content_Type
         then ""
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Interpretation_Requirement_Create_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = Grpc_Content_Type then
         if Ada.Strings.Unbounded.Length (Grpc_Channel) = 0 then
            raise Program_Error with "gRPC channel not configured";
         end if;
         declare
            Rsp : constant Identifier :=
              Grpc_Transport.Invoke_Interpretation_Requirement_Create_Requirement
                (To_String (Grpc_Channel), Request);
            Response_Payload : constant String :=
              """" & To_String (Rsp) & """";
         begin
            Emit_Invoke_Response
              (Callback, User_Data, Response_Payload);
         end;
         return;
      end if;

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
   end Invoke_Interpretation_Requirement_Create_Requirement;

   procedure Invoke_Interpretation_Requirement_Read_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Query;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      Json_Payload : constant String := To_Json (Request);
      Payload : constant String :=
        (if Content_Type = "" or else Content_Type = "application/json"
         then Json_Payload
         elsif Content_Type = "application/flatbuffers"
         then Flatbuffers_Codec.To_Binary_Query (Request)
         elsif Content_Type = Grpc_Content_Type
         then ""
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Interpretation_Requirement_Read_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = Grpc_Content_Type then
         if Ada.Strings.Unbounded.Length (Grpc_Channel) = 0 then
            raise Program_Error with "gRPC channel not configured";
         end if;
         declare
            Rsp : constant Grpc_Transport.Interpretation_Requirement_Array :=
              Grpc_Transport.Invoke_Interpretation_Requirement_Read_Requirement
                (To_String (Grpc_Channel), Request);
            Acc : Unbounded_String := To_Unbounded_String ("[");
         begin
            for I in Rsp'Range loop
               if I > Rsp'First then
                  Append (Acc, ",");
               end if;
               Append (Acc, To_Json (Rsp (I)));
            end loop;
            Append (Acc, "]");
            Emit_Invoke_Response
              (Callback, User_Data, To_String (Acc));
         end;
         return;
      end if;

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
   end Invoke_Interpretation_Requirement_Read_Requirement;

   procedure Invoke_Interpretation_Requirement_Update_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Interpretation_Requirement;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      Json_Payload : constant String := To_Json (Request);
      Payload : constant String :=
        (if Content_Type = "" or else Content_Type = "application/json"
         then Json_Payload
         elsif Content_Type = "application/flatbuffers"
         then Flatbuffers_Codec.To_Binary_Interpretation_Requirement (Request)
         elsif Content_Type = Grpc_Content_Type
         then ""
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Interpretation_Requirement_Update_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = Grpc_Content_Type then
         if Ada.Strings.Unbounded.Length (Grpc_Channel) = 0 then
            raise Program_Error with "gRPC channel not configured";
         end if;
         declare
            Rsp : constant Ack :=
              Grpc_Transport.Invoke_Interpretation_Requirement_Update_Requirement
                (To_String (Grpc_Channel), Request);
            Response_Payload : constant String := To_Json (Rsp);
         begin
            Emit_Invoke_Response
              (Callback, User_Data, Response_Payload);
         end;
         return;
      end if;

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
   end Invoke_Interpretation_Requirement_Update_Requirement;

   procedure Invoke_Interpretation_Requirement_Delete_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Identifier;
      Callback  : Pcl_Bindings.Pcl_Resp_Cb_Access;
      User_Data : System.Address := System.Null_Address;
      Content_Type : String := "application/json")
   is
      use type Pcl_Bindings.Pcl_Status;
      Json_Payload : constant String := To_String (Request);
      Payload : constant String :=
        (if Content_Type = "" or else Content_Type = "application/json"
         then Json_Payload
         elsif Content_Type = "application/flatbuffers"
         then Flatbuffers_Codec.To_Binary_Identifier (Request)
         elsif Content_Type = Grpc_Content_Type
         then ""
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Interpretation_Requirement_Delete_Requirement);
      Msg    : aliased Pcl_Bindings.Pcl_Msg;
      Status : Pcl_Bindings.Pcl_Status;
      pragma Unreferenced (Status);
   begin
      if Content_Type = Grpc_Content_Type then
         if Ada.Strings.Unbounded.Length (Grpc_Channel) = 0 then
            raise Program_Error with "gRPC channel not configured";
         end if;
         declare
            Rsp : constant Ack :=
              Grpc_Transport.Invoke_Interpretation_Requirement_Delete_Requirement
                (To_String (Grpc_Channel), Request);
            Response_Payload : constant String := To_Json (Rsp);
         begin
            Emit_Invoke_Response
              (Callback, User_Data, Response_Payload);
         end;
         return;
      end if;

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
   end Invoke_Interpretation_Requirement_Delete_Requirement;

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
         when Ch_Interpretation_Requirement_Read_Capability =>
            declare
               Req : constant Query :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Query (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : constant Capability_Array :=
                 (if Handlers /= null and then Handlers.On_Interpretation_Requirement_Read_Capability /= null
                  then Handlers.On_Interpretation_Requirement_Read_Capability.all (Req)
                  else Default_Handle_Interpretation_Requirement_Read_Capability (Req));
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
                      then Flatbuffers_Codec.To_Binary_Capability_Array (To_String (Acc))
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Interpretation_Requirement_Create_Requirement =>
            declare
               Req : constant Interpretation_Requirement :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Interpretation_Requirement (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Identifier;
               Json_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Interpretation_Requirement_Create_Requirement /= null then
                  Handlers.On_Interpretation_Requirement_Create_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Interpretation_Requirement_Create_Requirement (Req, Rsp);
               end if;
               Json_Response := To_Unbounded_String (To_String (Rsp));
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then To_String (Json_Response)
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Identifier (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Interpretation_Requirement_Read_Requirement =>
            declare
               Req : constant Query :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Query (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : constant Interpretation_Requirement_Array :=
                 (if Handlers /= null and then Handlers.On_Interpretation_Requirement_Read_Requirement /= null
                  then Handlers.On_Interpretation_Requirement_Read_Requirement.all (Req)
                  else Default_Handle_Interpretation_Requirement_Read_Requirement (Req));
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
                      then Flatbuffers_Codec.To_Binary_Interpretation_Requirement_Array (To_String (Acc))
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Interpretation_Requirement_Update_Requirement =>
            declare
               Req : constant Interpretation_Requirement :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Interpretation_Requirement (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Ack;
               Json_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Interpretation_Requirement_Update_Requirement /= null then
                  Handlers.On_Interpretation_Requirement_Update_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Interpretation_Requirement_Update_Requirement (Req, Rsp);
               end if;
               Json_Response := To_Unbounded_String (To_Json (Rsp));
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then To_String (Json_Response)
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Ack (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Interpretation_Requirement_Delete_Requirement =>
            declare
               Req : constant Identifier :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then To_Unbounded_String (Request_Payload)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Identifier (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Ack;
               Json_Response : Unbounded_String := Null_Unbounded_String;
            begin
               if Handlers /= null and then Handlers.On_Interpretation_Requirement_Delete_Requirement /= null then
                  Handlers.On_Interpretation_Requirement_Delete_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Interpretation_Requirement_Delete_Requirement (Req, Rsp);
               end if;
               Json_Response := To_Unbounded_String (To_Json (Rsp));
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then To_String (Json_Response)
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Ack (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
      end case;
   end Dispatch;

end Pyramid.Services.Sensor_Data_Interpretation.Provided;
