--  Auto-generated service binding body
--  Package body: Pyramid.Services.Autonomy_Backend.Provided

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Ada.Unchecked_Conversion;
with GNATCOLL.JSON;  use GNATCOLL.JSON;
with Interfaces.C.Strings;
with System;
with System.Storage_Elements;
with Pyramid.Data_Model.Common.Types_Codec;  use Pyramid.Data_Model.Common.Types_Codec;
with Pyramid.Data_Model.Autonomy.Types_Codec;  use Pyramid.Data_Model.Autonomy.Types_Codec;
with Pyramid.Services.Autonomy_Backend.Flatbuffers_Codec;

package body Pyramid.Services.Autonomy_Backend.Provided is
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

   package Flatbuffers_Codec renames Pyramid.Services.Autonomy_Backend.Flatbuffers_Codec;

   function Supports_Content_Type (Content_Type : String) return Boolean is
   begin
      return Content_Type = ""
        or else Content_Type = Json_Content_Type
        or else Content_Type = Flatbuffers_Content_Type;
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

   function Decode_Read_Capabilities_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Capabilities_Array
   is
      Empty : Capabilities_Array (1 .. 0);
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
            then Flatbuffers_Codec.From_Binary_Capabilities_Array (Payload)
            else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         R : constant Read_Result := Read (Json_Payload);
      begin
         if not R.Success or else R.Value.Kind /= JSON_Array_Type then
            return Empty;
         end if;
         declare
            Arr    : constant JSON_Array := Get (R.Value);
            Result : Capabilities_Array (1 .. GNATCOLL.JSON.Length (Arr));
         begin
            for I in 1 .. GNATCOLL.JSON.Length (Arr) loop
               Result (I) := From_Json (Write (Get (Arr, I)), null);
            end loop;
            return Result;
         end;
      end;
   end Decode_Read_Capabilities_Response;

   function Decode_Create_Planning_Requirement_Response
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
   end Decode_Create_Planning_Requirement_Response;

   function Decode_Read_Planning_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Planning_Requirement_Array
   is
      Empty : Planning_Requirement_Array (1 .. 0);
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
            then Flatbuffers_Codec.From_Binary_Planning_Requirement_Array (Payload)
            else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         R : constant Read_Result := Read (Json_Payload);
      begin
         if not R.Success or else R.Value.Kind /= JSON_Array_Type then
            return Empty;
         end if;
         declare
            Arr    : constant JSON_Array := Get (R.Value);
            Result : Planning_Requirement_Array (1 .. GNATCOLL.JSON.Length (Arr));
         begin
            for I in 1 .. GNATCOLL.JSON.Length (Arr) loop
               Result (I) := From_Json (Write (Get (Arr, I)), null);
            end loop;
            return Result;
         end;
      end;
   end Decode_Read_Planning_Requirement_Response;

   function Decode_Update_Planning_Requirement_Response
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
   end Decode_Update_Planning_Requirement_Response;

   function Decode_Delete_Planning_Requirement_Response
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
   end Decode_Delete_Planning_Requirement_Response;

   function Decode_Create_Execution_Requirement_Response
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
   end Decode_Create_Execution_Requirement_Response;

   function Decode_Read_Execution_Requirement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Execution_Requirement_Array
   is
      Empty : Execution_Requirement_Array (1 .. 0);
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
            then Flatbuffers_Codec.From_Binary_Execution_Requirement_Array (Payload)
            else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         R : constant Read_Result := Read (Json_Payload);
      begin
         if not R.Success or else R.Value.Kind /= JSON_Array_Type then
            return Empty;
         end if;
         declare
            Arr    : constant JSON_Array := Get (R.Value);
            Result : Execution_Requirement_Array (1 .. GNATCOLL.JSON.Length (Arr));
         begin
            for I in 1 .. GNATCOLL.JSON.Length (Arr) loop
               Result (I) := From_Json (Write (Get (Arr, I)), null);
            end loop;
            return Result;
         end;
      end;
   end Decode_Read_Execution_Requirement_Response;

   function Decode_Update_Execution_Requirement_Response
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
   end Decode_Update_Execution_Requirement_Response;

   function Decode_Delete_Execution_Requirement_Response
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
   end Decode_Delete_Execution_Requirement_Response;

   function Decode_Create_State_Response
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
   end Decode_Create_State_Response;

   function Decode_Update_State_Response
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
   end Decode_Update_State_Response;

   function Decode_Delete_State_Response
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
   end Decode_Delete_State_Response;

   function Decode_Create_Plan_Response
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
   end Decode_Create_Plan_Response;

   function Decode_Read_Plan_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Plan_Array
   is
      Empty : Plan_Array (1 .. 0);
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
            then Flatbuffers_Codec.From_Binary_Plan_Array (Payload)
            else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         R : constant Read_Result := Read (Json_Payload);
      begin
         if not R.Success or else R.Value.Kind /= JSON_Array_Type then
            return Empty;
         end if;
         declare
            Arr    : constant JSON_Array := Get (R.Value);
            Result : Plan_Array (1 .. GNATCOLL.JSON.Length (Arr));
         begin
            for I in 1 .. GNATCOLL.JSON.Length (Arr) loop
               Result (I) := From_Json (Write (Get (Arr, I)), null);
            end loop;
            return Result;
         end;
      end;
   end Decode_Read_Plan_Response;

   function Decode_Update_Plan_Response
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
   end Decode_Update_Plan_Response;

   function Decode_Delete_Plan_Response
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
   end Decode_Delete_Plan_Response;

   function Decode_Read_Run_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Execution_Run_Array
   is
      Empty : Execution_Run_Array (1 .. 0);
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
            then Flatbuffers_Codec.From_Binary_Execution_Run_Array (Payload)
            else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         R : constant Read_Result := Read (Json_Payload);
      begin
         if not R.Success or else R.Value.Kind /= JSON_Array_Type then
            return Empty;
         end if;
         declare
            Arr    : constant JSON_Array := Get (R.Value);
            Result : Execution_Run_Array (1 .. GNATCOLL.JSON.Length (Arr));
         begin
            for I in 1 .. GNATCOLL.JSON.Length (Arr) loop
               Result (I) := From_Json (Write (Get (Arr, I)), null);
            end loop;
            return Result;
         end;
      end;
   end Decode_Read_Run_Response;

   function Decode_Read_Placement_Response
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Requirement_Placement_Array
   is
      Empty : Requirement_Placement_Array (1 .. 0);
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
            then Flatbuffers_Codec.From_Binary_Requirement_Placement_Array (Payload)
            else raise Constraint_Error with "Unsupported content type: " & Content_Type);
         R : constant Read_Result := Read (Json_Payload);
      begin
         if not R.Success or else R.Value.Kind /= JSON_Array_Type then
            return Empty;
         end if;
         declare
            Arr    : constant JSON_Array := Get (R.Value);
            Result : Requirement_Placement_Array (1 .. GNATCOLL.JSON.Length (Arr));
         begin
            for I in 1 .. GNATCOLL.JSON.Length (Arr) loop
               Result (I) := From_Json (Write (Get (Arr, I)), null);
            end loop;
            return Result;
         end;
      end;
   end Decode_Read_Placement_Response;

   --  -- Capabilities_Service ------------------------------------
   function Default_Handle_Read_Capabilities
     (Request : Query) return Capabilities_Array
   is
      pragma Unreferenced (Request);
      Empty : Capabilities_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Read_Capabilities;

   --  -- Planning_Requirement_Service ------------------------------------
   procedure Default_Handle_Create_Planning_Requirement
     (Request  : in  Planning_Requirement;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Unbounded_String;
   end Default_Handle_Create_Planning_Requirement;

   function Default_Handle_Read_Planning_Requirement
     (Request : Query) return Planning_Requirement_Array
   is
      pragma Unreferenced (Request);
      Empty : Planning_Requirement_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Read_Planning_Requirement;

   procedure Default_Handle_Update_Planning_Requirement
     (Request  : in  Planning_Requirement;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Update_Planning_Requirement;

   procedure Default_Handle_Delete_Planning_Requirement
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Delete_Planning_Requirement;

   --  -- Execution_Requirement_Service ------------------------------------
   procedure Default_Handle_Create_Execution_Requirement
     (Request  : in  Execution_Requirement;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Unbounded_String;
   end Default_Handle_Create_Execution_Requirement;

   function Default_Handle_Read_Execution_Requirement
     (Request : Query) return Execution_Requirement_Array
   is
      pragma Unreferenced (Request);
      Empty : Execution_Requirement_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Read_Execution_Requirement;

   procedure Default_Handle_Update_Execution_Requirement
     (Request  : in  Execution_Requirement;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Update_Execution_Requirement;

   procedure Default_Handle_Delete_Execution_Requirement
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Delete_Execution_Requirement;

   --  -- State_Service ------------------------------------
   procedure Default_Handle_Create_State
     (Request  : in  State_Update;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Unbounded_String;
   end Default_Handle_Create_State;

   procedure Default_Handle_Update_State
     (Request  : in  State_Update;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Update_State;

   procedure Default_Handle_Delete_State
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Delete_State;

   --  -- Plan_Service ------------------------------------
   procedure Default_Handle_Create_Plan
     (Request  : in  Plan;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Unbounded_String;
   end Default_Handle_Create_Plan;

   function Default_Handle_Read_Plan
     (Request : Query) return Plan_Array
   is
      pragma Unreferenced (Request);
      Empty : Plan_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Read_Plan;

   procedure Default_Handle_Update_Plan
     (Request  : in  Plan;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Update_Plan;

   procedure Default_Handle_Delete_Plan
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := (Success => True);
   end Default_Handle_Delete_Plan;

   --  -- Execution_Run_Service ------------------------------------
   function Default_Handle_Read_Run
     (Request : Query) return Execution_Run_Array
   is
      pragma Unreferenced (Request);
      Empty : Execution_Run_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Read_Run;

   --  -- Requirement_Placement_Service ------------------------------------
   function Default_Handle_Read_Placement
     (Request : Query) return Requirement_Placement_Array
   is
      pragma Unreferenced (Request);
      Empty : Requirement_Placement_Array (1 .. 0);
   begin
      return Empty;
   end Default_Handle_Read_Placement;

   function Service_Read_Capabilities
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Read_Capabilities);

   function Service_Create_Planning_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Create_Planning_Requirement);

   function Service_Read_Planning_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Read_Planning_Requirement);

   function Service_Update_Planning_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Update_Planning_Requirement);

   function Service_Delete_Planning_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Delete_Planning_Requirement);

   function Service_Create_Execution_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Create_Execution_Requirement);

   function Service_Read_Execution_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Read_Execution_Requirement);

   function Service_Update_Execution_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Update_Execution_Requirement);

   function Service_Delete_Execution_Requirement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Delete_Execution_Requirement);

   function Service_Create_State
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Create_State);

   function Service_Update_State
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Update_State);

   function Service_Delete_State
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Delete_State);

   function Service_Create_Plan
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Create_Plan);

   function Service_Read_Plan
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Read_Plan);

   function Service_Update_Plan
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Update_Plan);

   function Service_Delete_Plan
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Delete_Plan);

   function Service_Read_Run
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Read_Run);

   function Service_Read_Placement
     (Self      : Pcl_Bindings.Pcl_Container_Access;
      Request   : access constant Pcl_Bindings.Pcl_Msg;
      Response  : access Pcl_Bindings.Pcl_Msg;
      Ctx       : Pcl_Bindings.Pcl_Svc_Context_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status;
   pragma Convention (C, Service_Read_Placement);

   procedure Register_Services
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Handlers  : access constant Service_Handlers := null;
      Content_Type : String := "application/json")
   is
      Handler_Ptr : constant System.Address := Handler_Address (Handlers);
   begin
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Read_Capabilities);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Read_Capabilities'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Create_Planning_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Create_Planning_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Read_Planning_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Read_Planning_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Update_Planning_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Update_Planning_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Delete_Planning_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Delete_Planning_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Create_Execution_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Create_Execution_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Read_Execution_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Read_Execution_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Update_Execution_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Update_Execution_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Delete_Execution_Requirement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Delete_Execution_Requirement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Create_State);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Create_State'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Update_State);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Update_State'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Delete_State);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Delete_State'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Create_Plan);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Create_Plan'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Read_Plan);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Read_Plan'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Update_Plan);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Update_Plan'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Delete_Plan);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Delete_Plan'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Read_Run);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Read_Run'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
      declare
         Service_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Svc_Read_Placement);
         Type_Name : Interfaces.C.Strings.chars_ptr :=
           Interfaces.C.Strings.New_String (Content_Type);
         Port : Pcl_Bindings.Pcl_Port_Access;
         pragma Unreferenced (Port);
      begin
         Port := Pcl_Bindings.Add_Service
           (Container    => Container,
            Service_Name => Service_Name,
            Type_Name    => Type_Name,
            Handler      => Service_Read_Placement'Access,
            User_Data    => Handler_Ptr);
         Interfaces.C.Strings.Free (Service_Name);
         Interfaces.C.Strings.Free (Type_Name);
      end;
   end Register_Services;

   function Service_Read_Capabilities
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
         Channel       => Ch_Read_Capabilities,
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
   end Service_Read_Capabilities;

   function Service_Create_Planning_Requirement
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
         Channel       => Ch_Create_Planning_Requirement,
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
   end Service_Create_Planning_Requirement;

   function Service_Read_Planning_Requirement
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
         Channel       => Ch_Read_Planning_Requirement,
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
   end Service_Read_Planning_Requirement;

   function Service_Update_Planning_Requirement
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
         Channel       => Ch_Update_Planning_Requirement,
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
   end Service_Update_Planning_Requirement;

   function Service_Delete_Planning_Requirement
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
         Channel       => Ch_Delete_Planning_Requirement,
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
   end Service_Delete_Planning_Requirement;

   function Service_Create_Execution_Requirement
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
         Channel       => Ch_Create_Execution_Requirement,
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
   end Service_Create_Execution_Requirement;

   function Service_Read_Execution_Requirement
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
         Channel       => Ch_Read_Execution_Requirement,
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
   end Service_Read_Execution_Requirement;

   function Service_Update_Execution_Requirement
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
         Channel       => Ch_Update_Execution_Requirement,
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
   end Service_Update_Execution_Requirement;

   function Service_Delete_Execution_Requirement
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
         Channel       => Ch_Delete_Execution_Requirement,
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
   end Service_Delete_Execution_Requirement;

   function Service_Create_State
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
         Channel       => Ch_Create_State,
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
   end Service_Create_State;

   function Service_Update_State
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
         Channel       => Ch_Update_State,
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
   end Service_Update_State;

   function Service_Delete_State
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
         Channel       => Ch_Delete_State,
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
   end Service_Delete_State;

   function Service_Create_Plan
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
         Channel       => Ch_Create_Plan,
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
   end Service_Create_Plan;

   function Service_Read_Plan
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
         Channel       => Ch_Read_Plan,
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
   end Service_Read_Plan;

   function Service_Update_Plan
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
         Channel       => Ch_Update_Plan,
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
   end Service_Update_Plan;

   function Service_Delete_Plan
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
         Channel       => Ch_Delete_Plan,
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
   end Service_Delete_Plan;

   function Service_Read_Run
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
         Channel       => Ch_Read_Run,
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
   end Service_Read_Run;

   function Service_Read_Placement
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
         Channel       => Ch_Read_Placement,
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
   end Service_Read_Placement;

   --  -- PCL binding implementations -------------------------------

   procedure Invoke_Read_Capabilities
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
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Read_Capabilities);
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
   end Invoke_Read_Capabilities;

   procedure Invoke_Create_Planning_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Planning_Requirement;
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
         then Flatbuffers_Codec.To_Binary_Planning_Requirement (Request)
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Create_Planning_Requirement);
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
   end Invoke_Create_Planning_Requirement;

   procedure Invoke_Read_Planning_Requirement
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
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Read_Planning_Requirement);
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
   end Invoke_Read_Planning_Requirement;

   procedure Invoke_Update_Planning_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Planning_Requirement;
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
         then Flatbuffers_Codec.To_Binary_Planning_Requirement (Request)
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Update_Planning_Requirement);
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
   end Invoke_Update_Planning_Requirement;

   procedure Invoke_Delete_Planning_Requirement
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
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Delete_Planning_Requirement);
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
   end Invoke_Delete_Planning_Requirement;

   procedure Invoke_Create_Execution_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Execution_Requirement;
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
         then Flatbuffers_Codec.To_Binary_Execution_Requirement (Request)
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Create_Execution_Requirement);
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
   end Invoke_Create_Execution_Requirement;

   procedure Invoke_Read_Execution_Requirement
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
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Read_Execution_Requirement);
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
   end Invoke_Read_Execution_Requirement;

   procedure Invoke_Update_Execution_Requirement
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Execution_Requirement;
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
         then Flatbuffers_Codec.To_Binary_Execution_Requirement (Request)
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Update_Execution_Requirement);
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
   end Invoke_Update_Execution_Requirement;

   procedure Invoke_Delete_Execution_Requirement
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
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Delete_Execution_Requirement);
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
   end Invoke_Delete_Execution_Requirement;

   procedure Invoke_Create_State
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : State_Update;
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
         then Flatbuffers_Codec.To_Binary_State_Update (Request)
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Create_State);
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
   end Invoke_Create_State;

   procedure Invoke_Update_State
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : State_Update;
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
         then Flatbuffers_Codec.To_Binary_State_Update (Request)
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Update_State);
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
   end Invoke_Update_State;

   procedure Invoke_Delete_State
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
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Delete_State);
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
   end Invoke_Delete_State;

   procedure Invoke_Create_Plan
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Plan;
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
         then Flatbuffers_Codec.To_Binary_Plan (Request)
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Create_Plan);
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
   end Invoke_Create_Plan;

   procedure Invoke_Read_Plan
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
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Read_Plan);
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
   end Invoke_Read_Plan;

   procedure Invoke_Update_Plan
     (Executor  : Pcl_Bindings.Pcl_Executor_Access;
      Request   : Plan;
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
         then Flatbuffers_Codec.To_Binary_Plan (Request)
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Update_Plan);
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
   end Invoke_Update_Plan;

   procedure Invoke_Delete_Plan
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
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Delete_Plan);
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
   end Invoke_Delete_Plan;

   procedure Invoke_Read_Run
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
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Read_Run);
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
   end Invoke_Read_Run;

   procedure Invoke_Read_Placement
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
         else raise Constraint_Error with "Unsupported content type: " & Content_Type);
      Req_C  : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.Null_Ptr;
      Payload_Bytes : aliased constant String := Payload;
      Svc_C  : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Svc_Read_Placement);
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
   end Invoke_Read_Placement;

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
         when Ch_Read_Capabilities =>
            declare
               Req : constant Query :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Query (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : constant Capabilities_Array :=
                 (if Handlers /= null and then Handlers.On_Read_Capabilities /= null
                  then Handlers.On_Read_Capabilities.all (Req)
                  else Default_Handle_Read_Capabilities (Req));
            begin
               declare
                  use Ada.Strings.Unbounded;
                  Acc : Unbounded_String :=
                    To_Unbounded_String ("[");
                  Json_Response : String := "";
               begin
                  for I in Rsp'Range loop
                     if I > Rsp'First then
                        Append (Acc, ",");
                     end if;
                     Append (Acc, To_Json (Rsp (I)));
                  end loop;
                  Append (Acc, "]");
                  Json_Response := To_String (Acc);
                  Copy_To_Buf
                    ((if Content_Type = "" or else Content_Type = "application/json"
                      then Json_Response
                      elsif Content_Type = "application/flatbuffers"
                      then Flatbuffers_Codec.To_Binary_Capabilities_Array (Json_Response)
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Create_Planning_Requirement =>
            declare
               Req : constant Planning_Requirement :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Planning_Requirement (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Identifier;
               Json_Response : String := "";
            begin
               if Handlers /= null and then Handlers.On_Create_Planning_Requirement /= null then
                  Handlers.On_Create_Planning_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Create_Planning_Requirement (Req, Rsp);
               end if;
               Json_Response := To_String (Rsp);
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then Json_Response
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Identifier (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Read_Planning_Requirement =>
            declare
               Req : constant Query :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Query (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : constant Planning_Requirement_Array :=
                 (if Handlers /= null and then Handlers.On_Read_Planning_Requirement /= null
                  then Handlers.On_Read_Planning_Requirement.all (Req)
                  else Default_Handle_Read_Planning_Requirement (Req));
            begin
               declare
                  use Ada.Strings.Unbounded;
                  Acc : Unbounded_String :=
                    To_Unbounded_String ("[");
                  Json_Response : String := "";
               begin
                  for I in Rsp'Range loop
                     if I > Rsp'First then
                        Append (Acc, ",");
                     end if;
                     Append (Acc, To_Json (Rsp (I)));
                  end loop;
                  Append (Acc, "]");
                  Json_Response := To_String (Acc);
                  Copy_To_Buf
                    ((if Content_Type = "" or else Content_Type = "application/json"
                      then Json_Response
                      elsif Content_Type = "application/flatbuffers"
                      then Flatbuffers_Codec.To_Binary_Planning_Requirement_Array (Json_Response)
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Update_Planning_Requirement =>
            declare
               Req : constant Planning_Requirement :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Planning_Requirement (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Ack;
               Json_Response : String := "";
            begin
               if Handlers /= null and then Handlers.On_Update_Planning_Requirement /= null then
                  Handlers.On_Update_Planning_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Update_Planning_Requirement (Req, Rsp);
               end if;
               Json_Response := To_Json (Rsp);
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then Json_Response
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Ack (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Delete_Planning_Requirement =>
            declare
               Req : constant Identifier :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then To_Unbounded_String (Request_Payload)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Identifier (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Ack;
               Json_Response : String := "";
            begin
               if Handlers /= null and then Handlers.On_Delete_Planning_Requirement /= null then
                  Handlers.On_Delete_Planning_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Delete_Planning_Requirement (Req, Rsp);
               end if;
               Json_Response := To_Json (Rsp);
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then Json_Response
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Ack (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Create_Execution_Requirement =>
            declare
               Req : constant Execution_Requirement :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Execution_Requirement (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Identifier;
               Json_Response : String := "";
            begin
               if Handlers /= null and then Handlers.On_Create_Execution_Requirement /= null then
                  Handlers.On_Create_Execution_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Create_Execution_Requirement (Req, Rsp);
               end if;
               Json_Response := To_String (Rsp);
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then Json_Response
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Identifier (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Read_Execution_Requirement =>
            declare
               Req : constant Query :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Query (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : constant Execution_Requirement_Array :=
                 (if Handlers /= null and then Handlers.On_Read_Execution_Requirement /= null
                  then Handlers.On_Read_Execution_Requirement.all (Req)
                  else Default_Handle_Read_Execution_Requirement (Req));
            begin
               declare
                  use Ada.Strings.Unbounded;
                  Acc : Unbounded_String :=
                    To_Unbounded_String ("[");
                  Json_Response : String := "";
               begin
                  for I in Rsp'Range loop
                     if I > Rsp'First then
                        Append (Acc, ",");
                     end if;
                     Append (Acc, To_Json (Rsp (I)));
                  end loop;
                  Append (Acc, "]");
                  Json_Response := To_String (Acc);
                  Copy_To_Buf
                    ((if Content_Type = "" or else Content_Type = "application/json"
                      then Json_Response
                      elsif Content_Type = "application/flatbuffers"
                      then Flatbuffers_Codec.To_Binary_Execution_Requirement_Array (Json_Response)
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Update_Execution_Requirement =>
            declare
               Req : constant Execution_Requirement :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Execution_Requirement (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Ack;
               Json_Response : String := "";
            begin
               if Handlers /= null and then Handlers.On_Update_Execution_Requirement /= null then
                  Handlers.On_Update_Execution_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Update_Execution_Requirement (Req, Rsp);
               end if;
               Json_Response := To_Json (Rsp);
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then Json_Response
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Ack (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Delete_Execution_Requirement =>
            declare
               Req : constant Identifier :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then To_Unbounded_String (Request_Payload)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Identifier (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Ack;
               Json_Response : String := "";
            begin
               if Handlers /= null and then Handlers.On_Delete_Execution_Requirement /= null then
                  Handlers.On_Delete_Execution_Requirement.all (Req, Rsp);
               else
                  Default_Handle_Delete_Execution_Requirement (Req, Rsp);
               end if;
               Json_Response := To_Json (Rsp);
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then Json_Response
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Ack (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Create_State =>
            declare
               Req : constant State_Update :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_State_Update (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Identifier;
               Json_Response : String := "";
            begin
               if Handlers /= null and then Handlers.On_Create_State /= null then
                  Handlers.On_Create_State.all (Req, Rsp);
               else
                  Default_Handle_Create_State (Req, Rsp);
               end if;
               Json_Response := To_String (Rsp);
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then Json_Response
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Identifier (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Update_State =>
            declare
               Req : constant State_Update :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_State_Update (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Ack;
               Json_Response : String := "";
            begin
               if Handlers /= null and then Handlers.On_Update_State /= null then
                  Handlers.On_Update_State.all (Req, Rsp);
               else
                  Default_Handle_Update_State (Req, Rsp);
               end if;
               Json_Response := To_Json (Rsp);
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then Json_Response
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Ack (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Delete_State =>
            declare
               Req : constant Identifier :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then To_Unbounded_String (Request_Payload)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Identifier (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Ack;
               Json_Response : String := "";
            begin
               if Handlers /= null and then Handlers.On_Delete_State /= null then
                  Handlers.On_Delete_State.all (Req, Rsp);
               else
                  Default_Handle_Delete_State (Req, Rsp);
               end if;
               Json_Response := To_Json (Rsp);
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then Json_Response
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Ack (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Create_Plan =>
            declare
               Req : constant Plan :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Plan (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Identifier;
               Json_Response : String := "";
            begin
               if Handlers /= null and then Handlers.On_Create_Plan /= null then
                  Handlers.On_Create_Plan.all (Req, Rsp);
               else
                  Default_Handle_Create_Plan (Req, Rsp);
               end if;
               Json_Response := To_String (Rsp);
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then Json_Response
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Identifier (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Read_Plan =>
            declare
               Req : constant Query :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Query (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : constant Plan_Array :=
                 (if Handlers /= null and then Handlers.On_Read_Plan /= null
                  then Handlers.On_Read_Plan.all (Req)
                  else Default_Handle_Read_Plan (Req));
            begin
               declare
                  use Ada.Strings.Unbounded;
                  Acc : Unbounded_String :=
                    To_Unbounded_String ("[");
                  Json_Response : String := "";
               begin
                  for I in Rsp'Range loop
                     if I > Rsp'First then
                        Append (Acc, ",");
                     end if;
                     Append (Acc, To_Json (Rsp (I)));
                  end loop;
                  Append (Acc, "]");
                  Json_Response := To_String (Acc);
                  Copy_To_Buf
                    ((if Content_Type = "" or else Content_Type = "application/json"
                      then Json_Response
                      elsif Content_Type = "application/flatbuffers"
                      then Flatbuffers_Codec.To_Binary_Plan_Array (Json_Response)
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Update_Plan =>
            declare
               Req : constant Plan :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Plan (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Ack;
               Json_Response : String := "";
            begin
               if Handlers /= null and then Handlers.On_Update_Plan /= null then
                  Handlers.On_Update_Plan.all (Req, Rsp);
               else
                  Default_Handle_Update_Plan (Req, Rsp);
               end if;
               Json_Response := To_Json (Rsp);
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then Json_Response
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Ack (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Delete_Plan =>
            declare
               Req : constant Identifier :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then To_Unbounded_String (Request_Payload)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Identifier (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : Ack;
               Json_Response : String := "";
            begin
               if Handlers /= null and then Handlers.On_Delete_Plan /= null then
                  Handlers.On_Delete_Plan.all (Req, Rsp);
               else
                  Default_Handle_Delete_Plan (Req, Rsp);
               end if;
               Json_Response := To_Json (Rsp);
               Copy_To_Buf
                 ((if Content_Type = "" or else Content_Type = "application/json"
                   then Json_Response
                   elsif Content_Type = "application/flatbuffers"
                   then Flatbuffers_Codec.To_Binary_Ack (Rsp)
                   else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                  Response_Buf, Response_Size);
            end;
         when Ch_Read_Run =>
            declare
               Req : constant Query :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Query (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : constant Execution_Run_Array :=
                 (if Handlers /= null and then Handlers.On_Read_Run /= null
                  then Handlers.On_Read_Run.all (Req)
                  else Default_Handle_Read_Run (Req));
            begin
               declare
                  use Ada.Strings.Unbounded;
                  Acc : Unbounded_String :=
                    To_Unbounded_String ("[");
                  Json_Response : String := "";
               begin
                  for I in Rsp'Range loop
                     if I > Rsp'First then
                        Append (Acc, ",");
                     end if;
                     Append (Acc, To_Json (Rsp (I)));
                  end loop;
                  Append (Acc, "]");
                  Json_Response := To_String (Acc);
                  Copy_To_Buf
                    ((if Content_Type = "" or else Content_Type = "application/json"
                      then Json_Response
                      elsif Content_Type = "application/flatbuffers"
                      then Flatbuffers_Codec.To_Binary_Execution_Run_Array (Json_Response)
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                    Response_Buf, Response_Size);
               end;
            end;
         when Ch_Read_Placement =>
            declare
               Req : constant Query :=
                 (if Content_Type = "" or else Content_Type = "application/json"
                  then From_Json (Request_Payload, null)
                  elsif Content_Type = "application/flatbuffers"
                  then Flatbuffers_Codec.From_Binary_Query (Request_Payload, null)
                  else raise Constraint_Error with "Unsupported content type: " & Content_Type);
               Rsp : constant Requirement_Placement_Array :=
                 (if Handlers /= null and then Handlers.On_Read_Placement /= null
                  then Handlers.On_Read_Placement.all (Req)
                  else Default_Handle_Read_Placement (Req));
            begin
               declare
                  use Ada.Strings.Unbounded;
                  Acc : Unbounded_String :=
                    To_Unbounded_String ("[");
                  Json_Response : String := "";
               begin
                  for I in Rsp'Range loop
                     if I > Rsp'First then
                        Append (Acc, ",");
                     end if;
                     Append (Acc, To_Json (Rsp (I)));
                  end loop;
                  Append (Acc, "]");
                  Json_Response := To_String (Acc);
                  Copy_To_Buf
                    ((if Content_Type = "" or else Content_Type = "application/json"
                      then Json_Response
                      elsif Content_Type = "application/flatbuffers"
                      then Flatbuffers_Codec.To_Binary_Requirement_Placement_Array (Json_Response)
                      else raise Constraint_Error with "Unsupported content type: " & Content_Type),
                    Response_Buf, Response_Size);
               end;
            end;
      end case;
   end Dispatch;

end Pyramid.Services.Autonomy_Backend.Provided;
