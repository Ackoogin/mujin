--  tobj_interest_client.adb
--
--  Component body: subscribes to entity_matches, invokes create_requirement.
--  Uses generated service bindings for all PCL operations.

with Ada.Text_IO;
with GNATCOLL.JSON;
with Interfaces.C;
with Pyramid.Services.Tactical_Objects.Provided;
with System;

package body Tobj_Interest_Client is

   package Provided renames Pyramid.Services.Tactical_Objects.Provided;

   use type Interfaces.C.unsigned;
   use type System.Address;

   procedure Log (Msg : String) is
   begin
      Ada.Text_IO.Put_Line (Ada.Text_IO.Standard_Error,
                            "[interest_client] " & Msg);
      Ada.Text_IO.Flush (Ada.Text_IO.Standard_Error);
   end Log;

   -- -- On_Configure ----------------------------------------------------------

   function On_Configure
     (Container : Pcl_Bindings.Pcl_Container_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (User_Data);
   begin
      Provided.Subscribe_Entity_Matches
        (Container => Container,
         Callback  => On_Entity_Matches'Unrestricted_Access);
      return Pcl_Bindings.PCL_OK;
   end On_Configure;

   -- -- On_Entity_Matches -----------------------------------------------------

   procedure On_Entity_Matches
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Msg       : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address)
   is
      pragma Unreferenced (Container, User_Data);
      use GNATCOLL.JSON;
   begin
      if Msg.Data = System.Null_Address or else Msg.Size = 0 then
         return;
      end if;

      declare
         Body_Str     : constant String :=
           Provided.Msg_To_String (Msg.Data, Msg.Size);
         Parse_Result : Read_Result;
         Val          : JSON_Value;
         Arr          : JSON_Array;
      begin
         Log ("standard.entity_matches: " & Body_Str);
         Parse_Result := Read (Body_Str);
         if not Parse_Result.Success then
            Log ("WARNING: could not parse entity_matches JSON");
            return;
         end if;
         Val := Parse_Result.Value;
         if Val.Kind /= JSON_Array_Type then
            return;
         end if;
         Arr := Get (Val);
         for I in 1 .. Length (Arr) loop
            declare
               Ent      : constant JSON_Value := Get (Arr, I);
               Identity : constant String :=
                 (if Has_Field (Ent, "identity")
                  then Get (Ent, "identity") else "");
               Obj_Id   : constant String :=
                 (if Has_Field (Ent, "object_id")
                  then Get (Ent, "object_id") else "");
            begin
               Log ("  entity[" & Natural'Image (I) & "] id=" & Obj_Id &
                    " identity=" & Identity);
               Matches_Received := Matches_Received + 1;
               if Identity = "STANDARD_IDENTITY_HOSTILE" then
                  Found_Hostile_Entity := True;
               end if;
            end;
         end loop;
      end;
   end On_Entity_Matches;

   -- -- On_Create_Requirement_Response ----------------------------------------

   procedure On_Create_Requirement_Response
     (Resp      : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address)
   is
      pragma Unreferenced (User_Data);
      use GNATCOLL.JSON;
   begin
      if Resp /= null and then
         Resp.Data /= System.Null_Address and then
         Resp.Size > 0
      then
         declare
            Body_Str     : constant String :=
              Provided.Msg_To_String (Resp.Data, Resp.Size);
            Parse_Result : Read_Result;
         begin
            Log ("create_requirement response: " & Body_Str);
            Parse_Result := Read (Body_Str);
            if Parse_Result.Success and then
               Has_Field (Parse_Result.Value, "interest_id")
            then
               Interest_Id_Received := True;
            end if;
         end;
      end if;
      Svc_Response_Ready := True;
   end On_Create_Requirement_Response;

   -- -- Send_Create_Requirement -----------------------------------------------

   procedure Send_Create_Requirement
     (Transport   : Pcl_Bindings.Pcl_Socket_Transport_Access;
      Policy      : String;
      Identity    : String;
      Dimension   : String := "";
      Min_Lat_Rad : Long_Float := 0.0;
      Max_Lat_Rad : Long_Float := 0.0;
      Min_Lon_Rad : Long_Float := 0.0;
      Max_Lon_Rad : Long_Float := 0.0)
   is
      Req_Str : constant String :=
        Provided.Build_Standard_Requirement_Json
          (Policy      => Policy,
           Identity    => Identity,
           Dimension   => Dimension,
           Min_Lat_Rad => Min_Lat_Rad,
           Max_Lat_Rad => Max_Lat_Rad,
           Min_Lon_Rad => Min_Lon_Rad,
           Max_Lon_Rad => Max_Lon_Rad);
   begin
      Log ("create_requirement request: " & Req_Str);
      Provided.Invoke_Create_Requirement
        (Transport => Transport,
         Request   => Req_Str,
         Callback  => On_Create_Requirement_Response'Unrestricted_Access);
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
