--  tobj_evidence_provider.adb
--
--  Component body: subscribes to evidence_requirements, publishes observations.
--  Uses generated service bindings and proto-native tactical codecs.

with Ada.Text_IO;
with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Interfaces.C;
with Pyramid.Services.Tactical_Objects.Provided;
with Pyramid.Services.Tactical_Objects.Consumed;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Common.Types_Codec;
with Pyramid.Data_Model.Tactical.Types;  use Pyramid.Data_Model.Tactical.Types;
with System;

package body Tobj_Evidence_Provider is

   package Provided renames Pyramid.Services.Tactical_Objects.Provided;
   package Consumed renames Pyramid.Services.Tactical_Objects.Consumed;
   package Common   renames Pyramid.Data_Model.Common.Types_Codec;

   use type Interfaces.C.unsigned;
   use type Pcl_Bindings.Pcl_Executor_Access;
   use type System.Address;

   Pi         : constant Long_Float := 3.14159265358979323846;
   Deg_To_Rad : constant Long_Float := Pi / 180.0;
   pragma Unreferenced (Deg_To_Rad);

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
         Callback  => On_Evidence_Requirement'Unrestricted_Access,
         Content_Type => To_String (Content_Type));
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

      declare
         Req : constant Object_Evidence_Requirement :=
           Provided.Decode_Evidence_Requirements (Msg);
         Dimension_Str : constant String :=
           (if Req.Dimension = null or else Req.Dimension'Length = 0
            then "DIMENSION_UNSPECIFIED"
            else Common.To_String (Req.Dimension (Req.Dimension'First)));
      begin
         Log ("evidence requirement:"
              & " policy=" & Common.To_String (Req.Policy)
              & " dimension=" & Dimension_Str
              & " id=" & To_String (Req.Base.Id));
      end;

      --  Publish a typed canonical Object_Detail observation.
      --  Position: 51.0N 0.0E in radians; HOSTILE; SEA_SURFACE dimension.
      if Exec_Handle /= null and then not Observation_Sent then
         declare
            Obs : Object_Detail;
         begin
            Obs.Id            := To_Unbounded_String ("obj-1");
            Obs.Identity      := Identity_Hostile;
            Obs.Dimension     := Dimension_SeaSurface;
            Obs.Position.Latitude  := 51.0 * (3.14159265358979323846 / 180.0);
            Obs.Position.Longitude := 0.0;
            Obs.Quality       := 0.9;
            Obs.Creation_Time := 0.5;

            Log ("Publishing standard observation to " &
                 Consumed.Topic_Object_Evidence);
            Consumed.Publish_Object_Evidence
              (Exec_Handle, Obs, To_String (Content_Type));
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
