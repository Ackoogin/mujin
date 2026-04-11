with Ada.Command_Line;
with Ada.Text_IO;
with Ada.Unchecked_Conversion;
with Interfaces.C;
with Interfaces.C.Strings;
with System;

procedure Ada_Grpc_Cpp_Interop_E2E is
   use type Interfaces.C.int;
   use type Interfaces.C.Strings.chars_ptr;
   use type System.Address;

   function Load_Library_A (Name : Interfaces.C.Strings.chars_ptr)
     return System.Address
     with Import, Convention => C, External_Name => "LoadLibraryA";

   function Get_Proc_Address
     (Module : System.Address; Name : Interfaces.C.Strings.chars_ptr)
      return System.Address
     with Import, Convention => C, External_Name => "GetProcAddress";

   function Free_Library (Module : System.Address) return Interfaces.C.int
     with Import, Convention => C, External_Name => "FreeLibrary";

   type Start_Server_Access is access procedure
     (Address : Interfaces.C.Strings.chars_ptr);
   pragma Convention (C, Start_Server_Access);

   type Stop_Server_Access is access procedure;
   pragma Convention (C, Stop_Server_Access);

   type Invoke_Create_Requirement_Json_Access is access function
     (Channel : Interfaces.C.Strings.chars_ptr;
      Request : Interfaces.C.Strings.chars_ptr)
      return Interfaces.C.Strings.chars_ptr;
   pragma Convention (C, Invoke_Create_Requirement_Json_Access);

   type Free_String_Access is access procedure
     (Value : Interfaces.C.Strings.chars_ptr);
   pragma Convention (C, Free_String_Access);

   function To_Invoke_Create_Requirement_Json is new Ada.Unchecked_Conversion
     (System.Address, Invoke_Create_Requirement_Json_Access);
   function To_Free_String is new Ada.Unchecked_Conversion
     (System.Address, Free_String_Access);

   Dll_Path : Interfaces.C.Strings.chars_ptr :=
     Interfaces.C.Strings.New_String ("");
   Address_C : Interfaces.C.Strings.chars_ptr :=
     Interfaces.C.Strings.New_String ("127.0.0.1:50101");
   Request_Json_C : Interfaces.C.Strings.chars_ptr :=
     Interfaces.C.Strings.New_String
       ("{""base"":{""id"":""ada-grpc-interest"",""source"":""ada-grpc-source""," &
        """update_time"":12.5}," &
        """status"":{""id"":""ada-grpc-achievement"",""source"":""ada-grpc-planner""," &
        """update_time"":13.0,""status"":""PROGRESS_IN_PROGRESS"",""quality"":0.75," &
        """achieveability"":""FEASIBILITY_FEASIBLE""}," &
        """source"":""OBJECT_SOURCE_RADAR""," &
        """policy"":""DATA_POLICY_OBTAIN""," &
        """dimension"":[""BATTLE_DIMENSION_SEA_SURFACE"",""BATTLE_DIMENSION_AIR""]," &
        """poly_area"":{""points"":[{""latitude"":0.872664625997,""longitude"":-0.01745329252}," &
        "{""latitude"":0.872664625997,""longitude"":0.01745329252}," &
        "{""latitude"":0.907571211037,""longitude"":0.01745329252}," &
        "{""latitude"":0.907571211037,""longitude"":-0.01745329252}]}}");
   Expected_Response : constant String := "grpc-interest-ada-grpc-interest-4-2";

   procedure Log (Msg : String) is
   begin
      Ada.Text_IO.Put_Line
        (Ada.Text_IO.Standard_Error, "[ada_grpc_cpp_interop] " & Msg);
   end Log;

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

   procedure Parse_Args is
      use Ada.Command_Line;
      I : Positive := 1;
   begin
      while I <= Argument_Count loop
         if Argument (I) = "--dll" and then I + 1 <= Argument_Count then
            Interfaces.C.Strings.Free (Dll_Path);
            Dll_Path := Interfaces.C.Strings.New_String (Argument (I + 1));
            I := I + 2;
         elsif Argument (I) = "--address" and then I + 1 <= Argument_Count then
            Interfaces.C.Strings.Free (Address_C);
            Address_C := Interfaces.C.Strings.New_String (Argument (I + 1));
            I := I + 2;
         else
            I := I + 1;
         end if;
      end loop;
   end Parse_Args;

begin
   Parse_Args;
   declare
      Dll_Value : constant String :=
        (if Dll_Path = Interfaces.C.Strings.Null_Ptr
         then ""
         else Interfaces.C.Strings.Value (Dll_Path));
   begin
      if Dll_Path = Interfaces.C.Strings.Null_Ptr
        or else Dll_Value = ""
      then
         Log ("FAIL: missing --dll");
         Ada.Command_Line.Set_Exit_Status (1);
         return;
      end if;
   end;

   declare
      Module : constant System.Address := Load_Library_A (Dll_Path);
      Invoke_Create_Requirement_Json : Invoke_Create_Requirement_Json_Access;
      Free_String : Free_String_Access;
      Invoke_Name : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String
          ("grpc_provided_object_of_interest_service_create_requirement_json");
      Free_Name : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String
          ("pyramid_services_tactical_objects_grpc_free_string");
   begin
      if Module = System.Null_Address then
         Log ("FAIL: could not load shim DLL");
         Ada.Command_Line.Set_Exit_Status (1);
         return;
      end if;

      Invoke_Create_Requirement_Json :=
        To_Invoke_Create_Requirement_Json (Get_Proc_Address (Module, Invoke_Name));
      Free_String := To_Free_String (Get_Proc_Address (Module, Free_Name));

      Interfaces.C.Strings.Free (Invoke_Name);
      Interfaces.C.Strings.Free (Free_Name);

      if Invoke_Create_Requirement_Json = null then
         Log ("FAIL: invoke export was not found");
         Ada.Command_Line.Set_Exit_Status (1);
         return;
      end if;

      if Free_String = null then
         Log ("FAIL: free export was not found");
         Ada.Command_Line.Set_Exit_Status (1);
         return;
      end if;

      delay 0.05;
      declare
         Response_Ptr : constant Interfaces.C.Strings.chars_ptr :=
           Invoke_Create_Requirement_Json
             (Channel => Address_C,
              Request => Request_Json_C);
         Response_Text : constant String :=
           (if Response_Ptr = Interfaces.C.Strings.Null_Ptr
            then ""
            else Interfaces.C.Strings.Value (Response_Ptr));
         Normalised_Response : constant String :=
           Normalise_Json_String (Response_Text);
      begin
         Log ("Received response: " & Response_Text);
         if Normalised_Response = Expected_Response then
            Log ("PASS: Ada invoked the C++ gRPC transport successfully");
            Ada.Command_Line.Set_Exit_Status (0);
         else
            Log ("FAIL: unexpected response");
            Ada.Command_Line.Set_Exit_Status (1);
         end if;

         if Response_Ptr /= Interfaces.C.Strings.Null_Ptr then
            Free_String (Response_Ptr);
         end if;
      end;

      Interfaces.C.Strings.Free (Dll_Path);
      Interfaces.C.Strings.Free (Address_C);
      Interfaces.C.Strings.Free (Request_Json_C);
      pragma Unreferenced (Module);
   end;
end Ada_Grpc_Cpp_Interop_E2E;
