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

   type Module_Handle is new System.Address;

   function Load_Library_A (Name : Interfaces.C.Strings.chars_ptr)
     return Module_Handle
     with Import, Convention => C, External_Name => "LoadLibraryA";

   function Get_Proc_Address
     (Module : Module_Handle; Name : Interfaces.C.Strings.chars_ptr)
      return System.Address
     with Import, Convention => C, External_Name => "GetProcAddress";

   function Free_Library (Module : Module_Handle) return Interfaces.C.int
     with Import, Convention => C, External_Name => "FreeLibrary";

   type Start_Server_Access is access procedure
     (Address : Interfaces.C.Strings.chars_ptr);
   pragma Convention (C, Start_Server_Access);

   type Stop_Server_Access is access procedure;
   pragma Convention (C, Stop_Server_Access);

   type Invoke_Create_Requirement_Access is access procedure
     (Channel  : System.Address;
      Request  : System.Address;
      Response : System.Address);
   pragma Convention (C, Invoke_Create_Requirement_Access);

   function To_Start_Server is new Ada.Unchecked_Conversion
     (System.Address, Start_Server_Access);
   function To_Stop_Server is new Ada.Unchecked_Conversion
     (System.Address, Stop_Server_Access);
   function To_Invoke_Create_Requirement is new Ada.Unchecked_Conversion
     (System.Address, Invoke_Create_Requirement_Access);
   function To_Address is new Ada.Unchecked_Conversion
     (Interfaces.C.Strings.chars_ptr, System.Address);

   Response_Buffer : aliased Interfaces.C.char_array (0 .. 255) :=
     (others => Interfaces.C.nul);

   Dll_Path : Interfaces.C.Strings.chars_ptr :=
     Interfaces.C.Strings.New_String ("");
   Address_C : Interfaces.C.Strings.chars_ptr :=
     Interfaces.C.Strings.New_String ("127.0.0.1:50101");
   Policy_C : Interfaces.C.Strings.chars_ptr :=
     Interfaces.C.Strings.New_String ("DATA_POLICY_OBTAIN");

   procedure Log (Msg : String) is
   begin
      Ada.Text_IO.Put_Line
        (Ada.Text_IO.Standard_Error, "[ada_grpc_cpp_interop] " & Msg);
   end Log;

   function Buffer_To_String return String is
   begin
      return Interfaces.C.To_Ada (Response_Buffer);
   end Buffer_To_String;

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
      Module : constant Module_Handle := Load_Library_A (Dll_Path);
      Freed : Interfaces.C.int;
      Start_Server : Start_Server_Access;
      Stop_Server : Stop_Server_Access;
      Invoke_Create_Requirement : Invoke_Create_Requirement_Access;
      Start_Name : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("pyramid_grpc_server_start");
      Stop_Name : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String ("pyramid_grpc_server_stop");
      Invoke_Name : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String
          ("grpc_object_of_interest_service_create_requirement");
   begin
      if Module = Module_Handle (System.Null_Address) then
         Log ("FAIL: could not load shim DLL");
         Ada.Command_Line.Set_Exit_Status (1);
         return;
      end if;

      Start_Server := To_Start_Server (Get_Proc_Address (Module, Start_Name));
      Stop_Server := To_Stop_Server (Get_Proc_Address (Module, Stop_Name));
      Invoke_Create_Requirement :=
        To_Invoke_Create_Requirement (Get_Proc_Address (Module, Invoke_Name));

      Interfaces.C.Strings.Free (Start_Name);
      Interfaces.C.Strings.Free (Stop_Name);
      Interfaces.C.Strings.Free (Invoke_Name);

      if Start_Server = null
        or else Stop_Server = null
        or else Invoke_Create_Requirement = null
      then
         Log ("FAIL: required gRPC exports were not found");
         Ada.Command_Line.Set_Exit_Status (1);
         return;
      end if;

      Log ("Starting C++ gRPC server via shim");
      Start_Server (Address_C);

      delay 0.05;
      Response_Buffer := (others => Interfaces.C.nul);
      Invoke_Create_Requirement
        (Channel  => To_Address (Address_C),
         Request  => To_Address (Policy_C),
         Response => Response_Buffer'Address);

      declare
         Response_Text : constant String := Buffer_To_String;
      begin
         Log ("Received response: " & Response_Text);
         if Response_Text = "ada-grpc-interest-42" then
            Log ("PASS: Ada invoked the C++ gRPC transport successfully");
            Ada.Command_Line.Set_Exit_Status (0);
         else
            Log ("FAIL: unexpected response");
            Ada.Command_Line.Set_Exit_Status (1);
         end if;
      end;

      Stop_Server.all;
      Interfaces.C.Strings.Free (Dll_Path);
      Interfaces.C.Strings.Free (Address_C);
      Interfaces.C.Strings.Free (Policy_C);
      Freed := Free_Library (Module);
      pragma Unreferenced (Freed);
   end;
end Ada_Grpc_Cpp_Interop_E2E;
