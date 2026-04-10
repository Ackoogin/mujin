--  tobj_evidence_provider.adb
--
--  Component body: subscribes to evidence_requirements, publishes observations.
--  Uses generated service bindings and Json_Codec for all serialisation.

with Ada.Text_IO;
with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Interfaces.C;
with Interfaces.C.Strings;
with Pyramid.Services.Tactical_Objects.Provided;
with Pyramid.Services.Tactical_Objects.Consumed;
with Pyramid.Services.Tactical_Objects.Flatbuffers_Codec;
with Pyramid.Services.Tactical_Objects.Json_Codec;
with Pyramid.Services.Tactical_Objects.Wire_Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with System;

package body Tobj_Evidence_Provider is

   package Provided renames Pyramid.Services.Tactical_Objects.Provided;
   package Consumed renames Pyramid.Services.Tactical_Objects.Consumed;
   package Flat     renames Pyramid.Services.Tactical_Objects.Flatbuffers_Codec;
   package Codec    renames Pyramid.Services.Tactical_Objects.Json_Codec;
   package Wire     renames Pyramid.Services.Tactical_Objects.Wire_Types;

   use type Interfaces.C.unsigned;
   use type Interfaces.C.Strings.chars_ptr;
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

   function Decode_Evidence_Requirement
     (Msg : access constant Pcl_Bindings.Pcl_Msg)
      return Wire.Evidence_Requirement
   is
      Payload : constant String := Provided.Msg_To_String (Msg.Data, Msg.Size);
   begin
      if Msg.Type_Name /= Interfaces.C.Strings.Null_Ptr
        and then Interfaces.C.Strings.Value (Msg.Type_Name) = Flat.Content_Type
      then
         return Flat.From_Binary_Evidence_Requirement (Payload, null);
      end if;
      return Codec.From_Json (Payload);
   end Decode_Evidence_Requirement;

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
         Req : constant Wire.Evidence_Requirement := Decode_Evidence_Requirement (Msg);
      begin
         Log ("evidence requirement:"
              & " policy=" & Codec.Data_Policy_To_String (Req.Policy)
              & " dimension=" & Codec.Battle_Dimension_To_String (Req.Dimension)
              & " min_lat=" & Long_Float'Image (Req.Min_Lat_Rad)
              & " max_lat=" & Long_Float'Image (Req.Max_Lat_Rad));
      end;

      --  Publish a typed observation using the canonical Json_Codec.
      --  Position: 51.0N 0.0E in radians; HOSTILE; SEA_SURFACE dimension.
      if Exec_Handle /= null and then not Observation_Sent then
         declare
            Obs : Wire.Object_Evidence;
         begin
            Obs.Identity      := Identity_Hostile;
            Obs.Dimension     := Dimension_SeaSurface;
            Obs.Latitude_Rad  := 51.0 * (3.14159265358979323846 / 180.0);
            Obs.Longitude_Rad := 0.0;
            Obs.Confidence    := 0.9;
            Obs.Observed_At   := 0.5;

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
