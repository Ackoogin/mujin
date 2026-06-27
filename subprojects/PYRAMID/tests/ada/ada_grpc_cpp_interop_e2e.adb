with Ada.Command_Line;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Interfaces;
with Interfaces.C.Strings;
with Pcl_Bindings;
with Pcl_Plugins;
with Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Tactical.Types;
with Pyramid.Services.Tactical_Objects.Provided;
with System;
with System.Storage_Elements;

procedure Ada_Grpc_Cpp_Interop_E2E is
   use Ada.Strings.Unbounded;
   use type Pcl_Bindings.Pcl_Status;
   use type Pcl_Bindings.Pcl_Executor_Access;
   use type Pcl_Bindings.Pcl_Transport_Const_Access;

   package Base_Types renames Pyramid.Data_Model.Base.Types;
   package Common_Types renames Pyramid.Data_Model.Common.Types;
   package Tactical_Types renames Pyramid.Data_Model.Tactical.Types;
   package Provided renames Pyramid.Services.Tactical_Objects.Provided;

   Transport_Plugin_Path : Unbounded_String := Null_Unbounded_String;
   Codec_Plugin_Path : Unbounded_String := Null_Unbounded_String;
   Address : Unbounded_String := To_Unbounded_String ("127.0.0.1:50101");
   Expected_Response : constant String := "grpc-interest-ada-grpc-interest-4-2";
   Response_Id : Base_Types.Identifier := Null_Unbounded_String;
   Content_Type : constant String := "application/protobuf";

   procedure Log (Msg : String) is
   begin
      Ada.Text_IO.Put_Line
        (Ada.Text_IO.Standard_Error, "[ada_grpc_cpp_interop] " & Msg);
   end Log;

   function Make_Request return Tactical_Types.Object_Interest_Requirement is
      Request : Tactical_Types.Object_Interest_Requirement;
   begin
      Request.Base.Id := To_Unbounded_String ("ada-grpc-interest");
      Request.Base.Source := To_Unbounded_String ("ada-grpc-source");
      Request.Base.Update_Time := 12.5;

      Request.Status.Id := To_Unbounded_String ("ada-grpc-achievement");
      Request.Status.Source := To_Unbounded_String ("ada-grpc-planner");
      Request.Status.Update_Time := 13.0;
      Request.Status.Status := Common_Types.Progress_InProgress;
      Request.Status.Quality := 0.75;
      Request.Status.Achieveability := Common_Types.Feasibility_Feasible;

      Request.Source := Tactical_Types.Source_Radar;
      Request.Policy := Common_Types.Policy_Obtain;
      Request.Dimension :=
        new Tactical_Types.Dimension_Array'
          (1 => Common_Types.Dimension_SeaSurface,
           2 => Common_Types.Dimension_Air);
      Request.Has_Val_Poly_Area := True;
      Request.Val_Poly_Area.Points :=
        new Common_Types.Points_Array'
          (1 => (Latitude => 0.872664625997, Longitude => -0.01745329252),
           2 => (Latitude => 0.872664625997, Longitude => 0.01745329252),
           3 => (Latitude => 0.907571211037, Longitude => 0.01745329252),
           4 => (Latitude => 0.907571211037, Longitude => -0.01745329252));
      return Request;
   end Make_Request;

   procedure On_Create_Requirement_Response
     (Resp      : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address);
   pragma Convention (C, On_Create_Requirement_Response);

   procedure On_Create_Requirement_Response
     (Resp      : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address)
   is
      pragma Unreferenced (User_Data);
   begin
      Response_Id :=
        Provided.Decode_Object_Of_Interest_Create_Requirement_Response
          (Resp);
   end On_Create_Requirement_Response;

   procedure Parse_Args is
      use Ada.Command_Line;
      I : Positive := 1;
   begin
      while I <= Argument_Count loop
         if Argument (I) = "--transport-plugin" and then I + 1 <= Argument_Count then
            Transport_Plugin_Path := To_Unbounded_String (Argument (I + 1));
            I := I + 2;
         elsif Argument (I) = "--codec-plugin" and then I + 1 <= Argument_Count then
            Codec_Plugin_Path := To_Unbounded_String (Argument (I + 1));
            I := I + 2;
         elsif Argument (I) = "--address" and then I + 1 <= Argument_Count then
            Address := To_Unbounded_String (Argument (I + 1));
            I := I + 2;
         else
            I := I + 1;
         end if;
      end loop;
   end Parse_Args;

begin
   Parse_Args;

   if Length (Transport_Plugin_Path) = 0 then
      Log ("FAIL: missing --transport-plugin");
      Ada.Command_Line.Set_Exit_Status (1);
      return;
   end if;
   if Length (Codec_Plugin_Path) = 0 then
      Log ("FAIL: missing --codec-plugin");
      Ada.Command_Line.Set_Exit_Status (1);
      return;
   end if;

   declare
      Codec_Path_C : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.New_String (To_String (Codec_Plugin_Path));
      Codec_Handle : aliased System.Address := System.Null_Address;
      Codec_Status : Pcl_Bindings.Pcl_Status;
   begin
      Codec_Status := Pcl_Plugins.Pcl_Plugin_Load_Codec
        (Codec_Path_C, Interfaces.C.Strings.Null_Ptr,
         Pcl_Plugins.Pcl_Codec_Registry_Default, Codec_Handle'Access);
      Interfaces.C.Strings.Free (Codec_Path_C);
      if Codec_Status /= Pcl_Bindings.PCL_OK then
         Log ("FAIL: could not load protobuf codec plugin");
         Ada.Command_Line.Set_Exit_Status (1);
         return;
      end if;
   end;

   declare
      Exec : Pcl_Bindings.Pcl_Executor_Access := Pcl_Bindings.Create_Executor;
      Transport_Handle : aliased System.Address := System.Null_Address;
      Transport_Vtable : aliased Pcl_Bindings.Pcl_Transport_Const_Access := null;
      Exec_Int : Interfaces.Integer_64;
      Config : Unbounded_String;
      Transport_Path_C : Interfaces.C.Strings.chars_ptr;
      Config_C : Interfaces.C.Strings.chars_ptr;
      Status : Pcl_Bindings.Pcl_Status;
   begin
      if Exec = null then
         Log ("FAIL: could not create executor");
         Ada.Command_Line.Set_Exit_Status (1);
         return;
      end if;

      Exec_Int := Interfaces.Integer_64
        (System.Storage_Elements.To_Integer (Exec.all'Address));
      Config := To_Unbounded_String
        ("{""executor"":" & Interfaces.Integer_64'Image (Exec_Int) &
         ",""role"":""consumed"",""address"":""" &
         To_String (Address) & """}");

      Transport_Path_C :=
        Interfaces.C.Strings.New_String (To_String (Transport_Plugin_Path));
      Config_C := Interfaces.C.Strings.New_String (To_String (Config));
      Status := Pcl_Plugins.Pcl_Plugin_Load_Transport
        (Transport_Path_C, Config_C, Transport_Handle'Access,
         Transport_Vtable'Access);
      Interfaces.C.Strings.Free (Transport_Path_C);
      Interfaces.C.Strings.Free (Config_C);

      if Status /= Pcl_Bindings.PCL_OK or else Transport_Vtable = null then
         Log ("FAIL: could not load gRPC transport plugin");
         Pcl_Bindings.Destroy_Executor (Exec);
         Ada.Command_Line.Set_Exit_Status (1);
         return;
      end if;

      Status := Pcl_Bindings.Set_Transport (Exec, Transport_Vtable);
      if Status /= Pcl_Bindings.PCL_OK then
         Log ("FAIL: could not set gRPC transport");
         Status := Pcl_Plugins.Pcl_Plugin_Unload_Transport
           (Transport_Handle, Transport_Vtable);
         Pcl_Bindings.Destroy_Executor (Exec);
         Ada.Command_Line.Set_Exit_Status (1);
         return;
      end if;

      Provided.Invoke_Object_Of_Interest_Create_Requirement
        (Executor     => Exec,
         Request      => Make_Request,
         Callback     => On_Create_Requirement_Response'Unrestricted_Access,
         Content_Type => Content_Type);

      Pcl_Bindings.Destroy_Executor (Exec);
      Status := Pcl_Plugins.Pcl_Plugin_Unload_Transport
        (Transport_Handle, Transport_Vtable);
   end;

   declare
      Response_Text : constant String := To_String (Response_Id);
   begin
      Log ("Received response: " & Response_Text);
      if Response_Text = Expected_Response then
         Log ("PASS: Ada invoked the generated service facade successfully");
         Ada.Command_Line.Set_Exit_Status (0);
      else
         Log ("FAIL: unexpected response");
         Ada.Command_Line.Set_Exit_Status (1);
      end if;
   end;
end Ada_Grpc_Cpp_Interop_E2E;
