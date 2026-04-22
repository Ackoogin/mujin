with Ada.Command_Line;
with Ada.Strings.Unbounded;
with Ada.Text_IO;
with Pcl_Bindings;
with Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Tactical.Types;
with Pyramid.Services.Tactical_Objects.Provided;
with System;

procedure Ada_Grpc_Cpp_Interop_E2E is
   use Ada.Strings.Unbounded;

   package Base_Types renames Pyramid.Data_Model.Base.Types;
   package Common_Types renames Pyramid.Data_Model.Common.Types;
   package Tactical_Types renames Pyramid.Data_Model.Tactical.Types;
   package Provided renames Pyramid.Services.Tactical_Objects.Provided;

   Dll_Path : Unbounded_String := Null_Unbounded_String;
   Address : Unbounded_String := To_Unbounded_String ("127.0.0.1:50101");
   Expected_Response : constant String := "grpc-interest-ada-grpc-interest-4-2";
   Response_Id : Base_Types.Identifier := Null_Unbounded_String;

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
      Response_Id := Provided.Decode_Create_Requirement_Response (Resp);
   end On_Create_Requirement_Response;

   procedure Parse_Args is
      use Ada.Command_Line;
      I : Positive := 1;
   begin
      while I <= Argument_Count loop
         if Argument (I) = "--dll" and then I + 1 <= Argument_Count then
            Dll_Path := To_Unbounded_String (Argument (I + 1));
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

   if Length (Dll_Path) = 0 then
      Log ("FAIL: missing --dll");
      Ada.Command_Line.Set_Exit_Status (1);
      return;
   end if;

   Provided.Configure_Grpc_Library (To_String (Dll_Path));
   Provided.Configure_Grpc_Channel (To_String (Address));

   delay 0.05;
   declare
   begin
      Provided.Invoke_Create_Requirement
        (Executor     => null,
         Request      => Make_Request,
         Callback     => On_Create_Requirement_Response'Unrestricted_Access,
         Content_Type => Provided.Grpc_Content_Type);
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
