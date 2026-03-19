--  tactical_objects_service.ads
--
--  Legacy codec helpers for the TacticalObjects binary wire format.
--
--  Standard service constants, JSON builders, and PCL bindings are now
--  in the generated packages:
--    Pyramid.Services.Tactical_Objects.Provided
--    Pyramid.Services.Tactical_Objects.Consumed
--
--  This package retains only:
--    - Legacy service name constants (direct TacticalObjectsComponent)
--    - Binary codec helpers (ordinal conversions, Frame_To_Tactical_Object)
--    - Legacy query JSON builders (Build_Read_Request_Json, etc.)

with Tactical_Objects_Types;  use Tactical_Objects_Types;
with Streaming_Codec;

package Tactical_Objects_Service is

   -- =========================================================================
   --  Legacy service constants (direct TacticalObjectsComponent interface)
   -- =========================================================================

   Read_Service_Name   : constant String := "subscribe_interest";
   Create_Service_Name : constant String := "create_object";
   Update_Service_Name : constant String := "update_object";
   Delete_Service_Name : constant String := "delete_object";

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
