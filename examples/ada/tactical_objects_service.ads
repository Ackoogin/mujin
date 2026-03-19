--  tactical_objects_service.ads
--
--  Unified service interface for the Tactical Objects component.
--
--  Standard service channel enums, wire-name constants, topic constants,
--  and JSON builders are imported from the generated packages:
--    Pyramid.Services.Tactical_Objects.Provided
--    Pyramid.Services.Tactical_Objects.Consumed
--
--  This package re-exports the generated constants for backward compatibility
--  and adds codec-level helpers (ordinal conversions, frame-to-object, etc.)
--  that are not generated from proto definitions.

with Tactical_Objects_Types;  use Tactical_Objects_Types;
with Streaming_Codec;
with Pyramid.Services.Tactical_Objects.Provided;
with Pyramid.Services.Tactical_Objects.Consumed;
with System;

package Tactical_Objects_Service is

   -- =========================================================================
   --  Re-exported generated types and constants (from Provided / Consumed)
   -- =========================================================================

   --  Service channel types (re-exported from generated packages)
   subtype Provided_Channel is
     Pyramid.Services.Tactical_Objects.Provided.Service_Channel;
   subtype Consumed_Channel is
     Pyramid.Services.Tactical_Objects.Consumed.Service_Channel;

   --  Standard bridge service / topic names (from generated Provided package)
   Standard_Create_Requirement_Service : constant String :=
     Pyramid.Services.Tactical_Objects.Provided.Svc_Create_Requirement;
   Standard_Entity_Matches_Topic : constant String :=
     Pyramid.Services.Tactical_Objects.Provided.Topic_Entity_Matches;
   Standard_Evidence_Reqs_Topic  : constant String :=
     Pyramid.Services.Tactical_Objects.Provided.Topic_Evidence_Requirements;

   --  Standard consumed topic name (from generated Consumed package)
   Standard_Object_Evidence_Topic : constant String :=
     Pyramid.Services.Tactical_Objects.Consumed.Topic_Object_Evidence;

   -- =========================================================================
   --  Legacy service constants (direct TacticalObjectsComponent interface)
   -- =========================================================================

   Read_Service_Name   : constant String := "subscribe_interest";
   Create_Service_Name : constant String := "create_object";
   Update_Service_Name : constant String := "update_object";
   Delete_Service_Name : constant String := "delete_object";

   -- =========================================================================
   --  JSON builders (delegated to generated package)
   -- =========================================================================

   function Build_Standard_Requirement_Json
     (Policy      : String;
      Identity    : String;
      Dimension   : String := "";
      Min_Lat_Rad : Long_Float := 0.0;
      Max_Lat_Rad : Long_Float := 0.0;
      Min_Lon_Rad : Long_Float := 0.0;
      Max_Lon_Rad : Long_Float := 0.0) return String
     renames Pyramid.Services.Tactical_Objects.Provided.Build_Standard_Requirement_Json;

   function Build_Standard_Evidence_Json
     (Identity    : String;
      Dimension   : String;
      Lat_Rad     : Long_Float;
      Lon_Rad     : Long_Float;
      Confidence  : Long_Float;
      Observed_At : Long_Float := 0.5) return String
     renames Pyramid.Services.Tactical_Objects.Provided.Build_Standard_Evidence_Json;

   -- =========================================================================
   --  Legacy query JSON builders (hand-maintained, not proto-derived)
   -- =========================================================================

   function Build_Read_Request_Json
     (Query : Tactical_Object_Query) return String;

   function Build_Active_Find_Request_Json
     (Query : Tactical_Object_Query) return String;

   -- =========================================================================
   --  Codec helpers (StreamingCodec wire-format ordinal conversions)
   -- =========================================================================

   function Frame_To_Tactical_Object
     (Frame : Streaming_Codec.Entity_Update_Frame) return Tactical_Object;

   function Tactical_Object_Image
     (Obj : Tactical_Object) return String;

   function Ordinal_To_Affiliation
     (V : Streaming_Codec.Byte) return Affiliation;

   function Ordinal_To_Object_Type
     (V : Streaming_Codec.Byte) return Object_Type;

   function Ordinal_To_Lifecycle_Status
     (V : Streaming_Codec.Byte) return Lifecycle_Status;

end Tactical_Objects_Service;
