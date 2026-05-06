--  Auto-generated gRPC transport body -- do not edit

with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with Ada.Unchecked_Conversion;
with Interfaces.C.Strings;
with GNATCOLL.JSON; use GNATCOLL.JSON;
with System;
with Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Tactical.Types;
with Pyramid.Data_Model.Common.Types_Codec;
with Pyramid.Data_Model.Tactical.Types_Codec;

package body Pyramid.Components.Tactical_objects.Services.Provided.GRPC_Transport is

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

   function Decode_Object_Match_Array (Response_Json : String) return Object_Match_Array is
      J : constant JSON_Value := Read (Response_Json);
      Arr : constant JSON_Array := Get (J);
      Len : constant Natural := Length (Arr);
      Result : Object_Match_Array (1 .. Len);
   begin
      for I in Result'Range loop
         Result (I) := Pyramid.Data_Model.Tactical.Types_Codec.From_Json (Write (Get (Arr, I)), null);
      end loop;
      return Result;
   end Decode_Object_Match_Array;

   function Decode_Object_Interest_Requirement_Array (Response_Json : String) return Object_Interest_Requirement_Array is
      J : constant JSON_Value := Read (Response_Json);
      Arr : constant JSON_Array := Get (J);
      Len : constant Natural := Length (Arr);
      Result : Object_Interest_Requirement_Array (1 .. Len);
   begin
      for I in Result'Range loop
         Result (I) := Pyramid.Data_Model.Tactical.Types_Codec.From_Json (Write (Get (Arr, I)), null);
      end loop;
      return Result;
   end Decode_Object_Interest_Requirement_Array;

   function Decode_Object_Detail_Array (Response_Json : String) return Object_Detail_Array is
      J : constant JSON_Value := Read (Response_Json);
      Arr : constant JSON_Array := Get (J);
      Len : constant Natural := Length (Arr);
      Result : Object_Detail_Array (1 .. Len);
   begin
      for I in Result'Range loop
         Result (I) := Pyramid.Data_Model.Tactical.Types_Codec.From_Json (Write (Get (Arr, I)), null);
      end loop;
      return Result;
   end Decode_Object_Detail_Array;

   function Invoke_Read_Match
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Object_Match_Array
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Common.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_matching_objects_service_read_match_json")));
   begin
      return Decode_Object_Match_Array (Response_Json);
   end Invoke_Read_Match;

   function Invoke_Create_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Tactical.Types.Object_Interest_Requirement)
      return Pyramid.Data_Model.Base.Types.Identifier
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Tactical.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_object_of_interest_service_create_requirement_json")));
   begin
      return Ada.Strings.Unbounded.To_Unbounded_String (Normalise_Json_String (Response_Json));
   end Invoke_Create_Requirement;

   function Invoke_Read_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Object_Interest_Requirement_Array
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Common.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_object_of_interest_service_read_requirement_json")));
   begin
      return Decode_Object_Interest_Requirement_Array (Response_Json);
   end Invoke_Read_Requirement;

   function Invoke_Update_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Tactical.Types.Object_Interest_Requirement)
      return Pyramid.Data_Model.Common.Types.Ack
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Tactical.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_object_of_interest_service_update_requirement_json")));
   begin
      return Pyramid.Data_Model.Common.Types_Codec.From_Json (Response_Json, null);
   end Invoke_Update_Requirement;

   function Invoke_Delete_Requirement
     (Channel : String;
      Request : Pyramid.Data_Model.Base.Types.Identifier)
      return Pyramid.Data_Model.Common.Types.Ack
   is
      Response_Json : constant String :=
        Call_Json (Channel, Encode_Identifier (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_object_of_interest_service_delete_requirement_json")));
   begin
      return Pyramid.Data_Model.Common.Types_Codec.From_Json (Response_Json, null);
   end Invoke_Delete_Requirement;

   function Invoke_Read_Detail
     (Channel : String;
      Request : Pyramid.Data_Model.Common.Types.Query)
      return Object_Detail_Array
   is
      Response_Json : constant String :=
        Call_Json (Channel, Pyramid.Data_Model.Common.Types_Codec.To_Json (Request),
                   To_Invoke_Json (Symbol_Address ("grpc_provided_specific_object_detail_service_read_detail_json")));
   begin
      return Decode_Object_Detail_Array (Response_Json);
   end Invoke_Read_Detail;

end Pyramid.Components.Tactical_objects.Services.Provided.GRPC_Transport;
