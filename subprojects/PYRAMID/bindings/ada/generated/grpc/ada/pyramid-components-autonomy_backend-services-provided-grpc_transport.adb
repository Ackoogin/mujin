--  Auto-generated gRPC transport body -- do not edit

with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Unchecked_Conversion;
with Interfaces.C.Strings;
with GNATCOLL.JSON; use GNATCOLL.JSON;
with System;
with Pyramid.Data_Model.Autonomy.Types;
with Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Autonomy.Types_Codec;
with Pyramid.Data_Model.Common.Types_Codec;

package body Pyramid.Components.Autonomy_backend.Services.Provided.GRPC_Transport is

   use type Interfaces.C.Strings.chars_ptr;

   use type System.Address;

   type Invoke_Json_Access is access function
     (Channel : Interfaces.C.Strings.chars_ptr;
      Request : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr;
   pragma Convention (C, Invoke_Json_Access);

   type Free_String_Access is access procedure
     (Value : Interfaces.C.Strings.chars_ptr);
   pragma Convention (C, Free_String_Access);

   function To_Invoke_Json is new Ada.Unchecked_Conversion
     (System.Address, Invoke_Json_Access);
   function To_Free_String is new Ada.Unchecked_Conversion
     (System.Address, Free_String_Access);

   function Load_Library_A (Name : Interfaces.C.Strings.chars_ptr)
     return System.Address
     with Import, Convention => C, External_Name => "LoadLibraryA";

   function Get_Proc_Address
     (Module : System.Address; Name : Interfaces.C.Strings.chars_ptr)
      return System.Address
     with Import, Convention => C, External_Name => "GetProcAddress";

   Library_Path : Unbounded_String :=
     To_Unbounded_String ("pyramid_grpc_c_shim.dll");
   Library_Module : System.Address := System.Null_Address;

   procedure Configure_Library (Path : String) is
   begin
      Library_Path := To_Unbounded_String (Path);
      Library_Module := System.Null_Address;
   end Configure_Library;

   function Ensure_Library return System.Address is
      Path_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (To_String (Library_Path));
   begin
      if Library_Module = System.Null_Address then
         Library_Module := Load_Library_A (Path_C);
      end if;
      Interfaces.C.Strings.Free (Path_C);
      if Library_Module = System.Null_Address then
         raise Program_Error with "could not load gRPC C shim: " & To_String (Library_Path);
      end if;
      return Library_Module;
   end Ensure_Library;

   function Symbol_Address (Name : String) return System.Address is
      Name_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Name);
      Result : constant System.Address :=
        Get_Proc_Address (Ensure_Library, Name_C);
   begin
      Interfaces.C.Strings.Free (Name_C);
      if Result = System.Null_Address then
         raise Program_Error with "gRPC C shim symbol not found: " & Name;
      end if;
      return Result;
   end Symbol_Address;

   function Normalise_Json_String (Value : String) return String is
   begin
      if Value'Length >= 2
        and then Value (Value'First) = '"'
        and then Value (Value'Last) = '"'
      then
         return Value (Value'First + 1 .. Value'Last - 1);
      end if;
      return Value;
   end Normalise_Json_String;

   function Encode_Identifier
     (Value : Pyramid.Data_Model.Base.Types.Identifier) return String is
   begin
      return """" & Ada.Strings.Unbounded.To_String (Value) & """";
   end Encode_Identifier;

   function Call_Json
     (Channel : String;
      Request_Json : String;
      Invoke : Invoke_Json_Access)
      return String
   is
      Channel_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Channel);
      Request_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (Request_Json);
      Response_Ptr : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.Null_Ptr;
      Response_Text : Unbounded_String := Null_Unbounded_String;
   begin
      Response_Ptr := Invoke (Channel_C, Request_C);
      Interfaces.C.Strings.Free (Channel_C);
      Interfaces.C.Strings.Free (Request_C);
      if Response_Ptr /= Interfaces.C.Strings.Null_Ptr then
         Response_Text := To_Unbounded_String
           (Interfaces.C.Strings.Value (Response_Ptr));
         To_Free_String
           (Symbol_Address ("pyramid_services_tactical_objects_grpc_free_string"))
           (Response_Ptr);
      end if;
      return To_String (Response_Text);
   end Call_Json;

   function Decode_Capabilities_Array (Response_Json : String) return Capabilities_Array is
      J : constant JSON_Value := Read (Response_Json);
      Arr : constant JSON_Array := Get (J);
      Len : constant Natural := Length (Arr);
      Result : Capabilities_Array (1 .. Len);
   begin
      for I in Result'Range loop
         Result (I) := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Write (Get (Arr, I)), null);
      end loop;
      return Result;
   end Decode_Capabilities_Array;

   function Decode_Planning_Requirement_Array (Response_Json : String) return Planning_Requirement_Array is
      J : constant JSON_Value := Read (Response_Json);
      Arr : constant JSON_Array := Get (J);
      Len : constant Natural := Length (Arr);
      Result : Planning_Requirement_Array (1 .. Len);
   begin
      for I in Result'Range loop
         Result (I) := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Write (Get (Arr, I)), null);
      end loop;
      return Result;
   end Decode_Planning_Requirement_Array;

   function Decode_Execution_Requirement_Array (Response_Json : String) return Execution_Requirement_Array is
      J : constant JSON_Value := Read (Response_Json);
      Arr : constant JSON_Array := Get (J);
      Len : constant Natural := Length (Arr);
      Result : Execution_Requirement_Array (1 .. Len);
   begin
      for I in Result'Range loop
         Result (I) := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Write (Get (Arr, I)), null);
      end loop;
      return Result;
   end Decode_Execution_Requirement_Array;

   function Decode_Plan_Array (Response_Json : String) return Plan_Array is
      J : constant JSON_Value := Read (Response_Json);
      Arr : constant JSON_Array := Get (J);
      Len : constant Natural := Length (Arr);
      Result : Plan_Array (1 .. Len);
   begin
      for I in Result'Range loop
         Result (I) := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Write (Get (Arr, I)), null);
      end loop;
      return Result;
   end Decode_Plan_Array;

   function Decode_Execution_Run_Array (Response_Json : String) return Execution_Run_Array is
      J : constant JSON_Value := Read (Response_Json);
      Arr : constant JSON_Array := Get (J);
      Len : constant Natural := Length (Arr);
      Result : Execution_Run_Array (1 .. Len);
   begin
      for I in Result'Range loop
         Result (I) := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Write (Get (Arr, I)), null);
      end loop;
      return Result;
   end Decode_Execution_Run_Array;

   function Decode_Requirement_Placement_Array (Response_Json : String) return Requirement_Placement_Array is
      J : constant JSON_Value := Read (Response_Json);
      Arr : constant JSON_Array := Get (J);
      Len : constant Natural := Length (Arr);
      Result : Requirement_Placement_Array (1 .. Len);
   begin
      for I in Result'Range loop
         Result (I) := Pyramid.Data_Model.Autonomy.Types_Codec.From_Json (Write (Get (Arr, I)), null);
      end loop;
      return Result;
   end Decode_Requirement_Placement_Array;

   function Invoke_Read_Capabilities
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Capabilities_Array
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Common.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_capabilities_service_read_capabilities_json")));
   begin
      return Decode_Capabilities_Array (Response_Json);
   end Invoke_Read_Capabilities;

   function Invoke_Create_Planning_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Autonomy.Types.Planning_Requirement)
      return Pyramid.Data_Model.Base.Types.Identifier
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_planning_requirement_service_create_planning_requirement_json")));
   begin
      return Ada.Strings.Unbounded.To_Unbounded_String (Normalise_Json_String (Response_Json));
   end Invoke_Create_Planning_Requirement;

   function Invoke_Read_Planning_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Planning_Requirement_Array
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Common.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_planning_requirement_service_read_planning_requirement_json")));
   begin
      return Decode_Planning_Requirement_Array (Response_Json);
   end Invoke_Read_Planning_Requirement;

   function Invoke_Update_Planning_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Autonomy.Types.Planning_Requirement)
      return Pyramid.Data_Model.Common.Types.Ack
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_planning_requirement_service_update_planning_requirement_json")));
   begin
      return Pyramid.Data_Model.Common.Types_Codec.From_Json (Response_Json, null);
   end Invoke_Update_Planning_Requirement;

   function Invoke_Delete_Planning_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Base.Types.Identifier)
      return Pyramid.Data_Model.Common.Types.Ack
   is
      Response_Json : constant String :=
        Call_Json (Channel, Encode_Identifier (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_planning_requirement_service_delete_planning_requirement_json")));
   begin
      return Pyramid.Data_Model.Common.Types_Codec.From_Json (Response_Json, null);
   end Invoke_Delete_Planning_Requirement;

   function Invoke_Create_Execution_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Autonomy.Types.Execution_Requirement)
      return Pyramid.Data_Model.Base.Types.Identifier
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_execution_requirement_service_create_execution_requirement_json")));
   begin
      return Ada.Strings.Unbounded.To_Unbounded_String (Normalise_Json_String (Response_Json));
   end Invoke_Create_Execution_Requirement;

   function Invoke_Read_Execution_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Execution_Requirement_Array
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Common.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_execution_requirement_service_read_execution_requirement_json")));
   begin
      return Decode_Execution_Requirement_Array (Response_Json);
   end Invoke_Read_Execution_Requirement;

   function Invoke_Update_Execution_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Autonomy.Types.Execution_Requirement)
      return Pyramid.Data_Model.Common.Types.Ack
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_execution_requirement_service_update_execution_requirement_json")));
   begin
      return Pyramid.Data_Model.Common.Types_Codec.From_Json (Response_Json, null);
   end Invoke_Update_Execution_Requirement;

   function Invoke_Delete_Execution_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Base.Types.Identifier)
      return Pyramid.Data_Model.Common.Types.Ack
   is
      Response_Json : constant String :=
        Call_Json (Channel, Encode_Identifier (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_execution_requirement_service_delete_execution_requirement_json")));
   begin
      return Pyramid.Data_Model.Common.Types_Codec.From_Json (Response_Json, null);
   end Invoke_Delete_Execution_Requirement;

   function Invoke_Create_State
     (Channel : String;
      Request : Pyramid.Data_Model.Autonomy.Types.State_Update)
      return Pyramid.Data_Model.Base.Types.Identifier
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_state_service_create_state_json")));
   begin
      return Ada.Strings.Unbounded.To_Unbounded_String (Normalise_Json_String (Response_Json));
   end Invoke_Create_State;

   function Invoke_Update_State
     (Channel : String;
      Request : Pyramid.Data_Model.Autonomy.Types.State_Update)
      return Pyramid.Data_Model.Common.Types.Ack
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_state_service_update_state_json")));
   begin
      return Pyramid.Data_Model.Common.Types_Codec.From_Json (Response_Json, null);
   end Invoke_Update_State;

   function Invoke_Delete_State
     (Channel : String;
      Request : Pyramid.Data_Model.Base.Types.Identifier)
      return Pyramid.Data_Model.Common.Types.Ack
   is
      Response_Json : constant String :=
        Call_Json (Channel, Encode_Identifier (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_state_service_delete_state_json")));
   begin
      return Pyramid.Data_Model.Common.Types_Codec.From_Json (Response_Json, null);
   end Invoke_Delete_State;

   function Invoke_Create_Plan
     (Channel : String;
      Request : Pyramid.Data_Model.Autonomy.Types.Plan)
      return Pyramid.Data_Model.Base.Types.Identifier
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_plan_service_create_plan_json")));
   begin
      return Ada.Strings.Unbounded.To_Unbounded_String (Normalise_Json_String (Response_Json));
   end Invoke_Create_Plan;

   function Invoke_Read_Plan
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Plan_Array
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Common.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_plan_service_read_plan_json")));
   begin
      return Decode_Plan_Array (Response_Json);
   end Invoke_Read_Plan;

   function Invoke_Update_Plan
     (Channel : String;
      Request : Pyramid.Data_Model.Autonomy.Types.Plan)
      return Pyramid.Data_Model.Common.Types.Ack
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Autonomy.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_plan_service_update_plan_json")));
   begin
      return Pyramid.Data_Model.Common.Types_Codec.From_Json (Response_Json, null);
   end Invoke_Update_Plan;

   function Invoke_Delete_Plan
     (Channel : String;
      Request : Pyramid.Data_Model.Base.Types.Identifier)
      return Pyramid.Data_Model.Common.Types.Ack
   is
      Response_Json : constant String :=
        Call_Json (Channel, Encode_Identifier (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_plan_service_delete_plan_json")));
   begin
      return Pyramid.Data_Model.Common.Types_Codec.From_Json (Response_Json, null);
   end Invoke_Delete_Plan;

   function Invoke_Read_Run
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Execution_Run_Array
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Common.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_execution_run_service_read_run_json")));
   begin
      return Decode_Execution_Run_Array (Response_Json);
   end Invoke_Read_Run;

   function Invoke_Read_Placement
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Requirement_Placement_Array
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Common.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_requirement_placement_service_read_placement_json")));
   begin
      return Decode_Requirement_Placement_Array (Response_Json);
   end Invoke_Read_Placement;

end Pyramid.Components.Autonomy_backend.Services.Provided.GRPC_Transport;
