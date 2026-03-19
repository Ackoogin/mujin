--  tobj_evidence_provider.adb
--
--  Component body: subscribes to evidence_requirements, publishes observations.
--  Uses generated service bindings for all PCL operations.

with Ada.Text_IO;
with Interfaces.C;
with Pyramid.Services.Tactical_Objects.Provided;
with Pyramid.Services.Tactical_Objects.Consumed;
with System;

package body Tobj_Evidence_Provider is

   package Provided renames Pyramid.Services.Tactical_Objects.Provided;
   package Consumed renames Pyramid.Services.Tactical_Objects.Consumed;

   use type Interfaces.C.unsigned;
   use type Pcl_Bindings.Pcl_Executor_Access;
   use type System.Address;

   Pi         : constant Long_Float := 3.14159265358979323846;
   Deg_To_Rad : constant Long_Float := Pi / 180.0;

   procedure Log (Msg : String) is
   begin
      Ada.Text_IO.Put_Line (Ada.Text_IO.Standard_Error,
                            "[evidence_provider] " & Msg);
      Ada.Text_IO.Flush (Ada.Text_IO.Standard_Error);
   end Log;

   -- -- On_Configure ----------------------------------------------------------

   function On_Configure
     (Container : Pcl_Bindings.Pcl_Container_Access;
      User_Data : System.Address) return Pcl_Bindings.Pcl_Status
   is
      pragma Unreferenced (User_Data);
   begin
      Provided.Subscribe_Evidence_Requirements
        (Container => Container,
         Callback  => On_Evidence_Requirement'Unrestricted_Access);
      return Pcl_Bindings.PCL_OK;
   end On_Configure;

   -- -- On_Evidence_Requirement -----------------------------------------------

   procedure On_Evidence_Requirement
     (Container : Pcl_Bindings.Pcl_Container_Access;
      Msg       : access constant Pcl_Bindings.Pcl_Msg;
      User_Data : System.Address)
   is
      pragma Unreferenced (Container, User_Data);
   begin
      if Msg.Data = System.Null_Address or else Msg.Size = 0 then
         return;
      end if;

      Evidence_Req_Received := True;
      Log ("evidence requirement: " &
           Provided.Msg_To_String (Msg.Data, Msg.Size));

      --  Publish a standard observation.
      --  Position: 51.0N 0.0E in radians; HOSTILE; SEA_SURFACE dimension.
      if Exec_Handle /= null and then not Observation_Sent then
         declare
            Obs_Json : constant String :=
              Provided.Build_Standard_Evidence_Json
                (Identity    => "STANDARD_IDENTITY_HOSTILE",
                 Dimension   => "BATTLE_DIMENSION_SEA_SURFACE",
                 Lat_Rad     => 51.0 * Deg_To_Rad,
                 Lon_Rad     => 0.0 * Deg_To_Rad,
                 Confidence  => 0.9,
                 Observed_At => 0.5);
         begin
            Log ("Publishing standard observation to " &
                 Consumed.Topic_Object_Evidence);
            Consumed.Publish_Object_Evidence (Exec_Handle, Obs_Json);
            Observation_Sent := True;
            Log ("Standard observation published");
         end;
      end if;
   end On_Evidence_Requirement;

   -- -- Reset -----------------------------------------------------------------

   procedure Reset is
   begin
      Evidence_Req_Received := False;
      Observation_Sent      := False;
   end Reset;

end Tobj_Evidence_Provider;
