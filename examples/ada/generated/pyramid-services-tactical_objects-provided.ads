--  Auto-generated service binding specification
--  Generated from: tactical_objects.proto by ada_service_generator.py
--  Package: Pyramid.Services.Tactical_Objects.Provided
--
--  Architecture: component logic > service binding (this) > PCL
--
--  This package provides:
--    1. Wire-name constants and topic constants
--    2. EntityActions handler stubs (Handle_*)
--    3. PCL binding procedures (Subscribe_*, Invoke_*, Publish_*)
--    4. Msg_To_String utility for PCL message payloads
--
--  JSON serialisation/deserialisation is provided by the companion
--  Pyramid.Services.Tactical_Objects.Json_Codec package.

with Pyramid_Data_Model_Base_Types;  use Pyramid_Data_Model_Base_Types;
with Pyramid_Data_Model_Common_Types;  use Pyramid_Data_Model_Common_Types;
with Pyramid_Data_Model_Tactical_Types;  use Pyramid_Data_Model_Tactical_Types;
with Pcl_Bindings;
with Interfaces.C;
with System;

package Pyramid.Services.Tactical_Objects.Provided is

   type Operation_Kind is
     (Op_Create,
      Op_Read,
      Op_Update,
      Op_Delete);

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

   type Tactical_Object_Array is array (Positive range <>) of Tactical_Object;
   type Zone_Array is array (Positive range <>) of Zone;

   --  -- Service wire-name constants (generated from proto) --------

   Svc_Create_Tactical_Object : constant String :=
     "tactical_object_service.create_tactical_object";
   Svc_Read_Tactical_Object : constant String :=
     "tactical_object_service.read_tactical_object";
   Svc_Update_Tactical_Object : constant String :=
     "tactical_object_service.update_tactical_object";
   Svc_Delete_Tactical_Object : constant String :=
     "tactical_object_service.delete_tactical_object";
   Svc_Create_Zone : constant String :=
     "zone_service.create_zone";
   Svc_Read_Zone : constant String :=
     "zone_service.read_zone";
   Svc_Update_Zone : constant String :=
     "zone_service.update_zone";
   Svc_Delete_Zone : constant String :=
     "zone_service.delete_zone";
   Svc_Create_Observation : constant String :=
     "observation_ingress_service.create_observation";

   --  -- Standard topic name constants --------------------------

   Topic_Object_Evidence : constant String :=
     "standard.object_evidence";

   --  -- PCL message utility ------------------------------------

   function Msg_To_String
     (Data : System.Address;
      Size : Interfaces.C.unsigned) return String;

   --  -- EntityActions handlers ------------------------------------
   --  Implement these procedures in the package body.

   --  TacticalObjectService
   procedure Handle_Create_Tactical_Object
     (Request  : in  Tactical_Object;
      Response : out Identifier);
   function Handle_Read_Tactical_Object
     (Request : Tactical_Object_Query) return Tactical_Object_Array;
   procedure Handle_Update_Tactical_Object
     (Request  : in  Tactical_Object;
      Response : out Ack);
   procedure Handle_Delete_Tactical_Object
     (Request  : in  Identifier;
      Response : out Ack);
   --  ZoneService
   procedure Handle_Create_Zone
     (Request  : in  Zone;
      Response : out Identifier);
   function Handle_Read_Zone
     (Request : Query) return Zone_Array;
   procedure Handle_Update_Zone
     (Request  : in  Zone;
      Response : out Ack);
   procedure Handle_Delete_Zone
     (Request  : in  Identifier;
      Response : out Ack);
   --  ObservationIngressService
   procedure Handle_Create_Observation
     (Request  : in  Observation;
      Response : out Identifier);

   --  -- PCL binding procedures ------------------------------------
   --  Subscribe/Invoke/Publish wrappers for PCL transport layer.
   --  Serialisation is handled internally (codec baked at generation time).

   procedure Publish_Object_Evidence
     (Exec    : Pcl_Bindings.Pcl_Executor_Access;
      Payload : String);

   --  -- Transport integration point ------------------------------

   procedure Dispatch
     (Channel      : in  Service_Channel;
      Request_Buf  : in  System.Address;
      Request_Size : in  Natural;
      Response_Buf : out System.Address;
      Response_Size: out Natural);

end Pyramid.Services.Tactical_Objects.Provided;
