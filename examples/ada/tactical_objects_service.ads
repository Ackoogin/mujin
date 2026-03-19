--  tactical_objects_service.ads
--
--  Hand-crafted EntityActions service interface for the Tactical Objects
--  component.  Corresponds to the proto-generated pattern that
--  ada_service_generator.py produces from:
--    proto/pyramid/components/tactical_objects.proto
--
--  The canonical generated spec (Handle_*/Dispatch only) lives in:
--    examples/ada/generated/pyramid-services-tactical_objects-provided.ads
--
--  This file retains the transport-level helpers (Build_Read_Request_Json,
--  Frame_To_Tactical_Object, etc.) alongside the generated interface because
--  the demo client (ada_tobj_client.adb) calls them directly.  Once the
--  client is migrated to call Handle_Read_Tactical_Object, those helpers
--  move to the .adb body.

with Tactical_Objects_Types;  use Tactical_Objects_Types;
with Streaming_Codec;
with System;

package Tactical_Objects_Service is

   --  -- EntityActions operation kind -----------------------------------------

   type Operation_Kind is
     (Op_Create,
      Op_Read,
      Op_Update,
      Op_Delete);

   --  -- Service channel (one per proto rpc) ----------------------------------

   type Service_Channel is
     (Ch_Create_Tactical_Object,
      Ch_Read_Tactical_Object,
      Ch_Update_Tactical_Object,
      Ch_Delete_Tactical_Object,
      Ch_Create_Zone,
      Ch_Read_Zone,
      Ch_Update_Zone,
      Ch_Delete_Zone,
      Ch_Create_Observation);

   --  Array types for streaming Read responses
   type Tactical_Object_Array is array (Positive range <>) of Tactical_Object;
   type Zone_Array            is array (Positive range <>) of Zone_Entity;

   --  -- EntityActions handlers ------------------------------------------------
   --  Implement these in the package body.

   --  TacticalObjectService
   procedure Handle_Create_Tactical_Object
     (Request  : in  Tactical_Object;
      Response : out Tactical_Objects_Types.Identifier);

   procedure Handle_Read_Tactical_Object
     (Request  : in  Tactical_Object_Query;
      Response : out Tactical_Object_Array);

   procedure Handle_Update_Tactical_Object
     (Request  : in  Tactical_Object;
      Response : out Tactical_Objects_Types.Ack);

   procedure Handle_Delete_Tactical_Object
     (Request  : in  Tactical_Objects_Types.Identifier;
      Response : out Tactical_Objects_Types.Ack);

   --  ZoneService
   procedure Handle_Create_Zone
     (Request  : in  Zone_Entity;
      Response : out Tactical_Objects_Types.Identifier);

   procedure Handle_Read_Zone
     (Request  : in  Zone_Query;
      Response : out Zone_Array);

   procedure Handle_Update_Zone
     (Request  : in  Zone_Entity;
      Response : out Tactical_Objects_Types.Ack);

   procedure Handle_Delete_Zone
     (Request  : in  Tactical_Objects_Types.Identifier;
      Response : out Tactical_Objects_Types.Ack);

   --  ObservationIngressService
   procedure Handle_Create_Observation
     (Request  : in  Observation;
      Response : out Tactical_Objects_Types.Identifier);

   --  -- Transport integration point ------------------------------------------
   --  Route an incoming (channel, raw buffer) call to the correct typed
   --  handler.  The transport layer calls this; it never calls Handle_*
   --  procedures directly.

   procedure Dispatch
     (Channel      : in  Service_Channel;
      Request_Buf  : in  System.Address;
      Request_Size : in  Natural;
      Response_Buf : out System.Address;
      Response_Size: out Natural);

   --  -- Transport-level helpers (used by ada_tobj_client) --------------------
   --  These will move to the .adb body once the client calls Handle_Read_*
   --  directly instead of driving PCL socket invocations manually.

   Read_Service_Name   : constant String := "subscribe_interest";
   Create_Service_Name : constant String := "create_object";
   Update_Service_Name : constant String := "update_object";
   Delete_Service_Name : constant String := "delete_object";

   function Build_Read_Request_Json
     (Query : Tactical_Object_Query) return String;

   function Build_Active_Find_Request_Json
     (Query : Tactical_Object_Query) return String;

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
