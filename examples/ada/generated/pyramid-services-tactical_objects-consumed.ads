--  Auto-generated service binding specification
--  Generated from: components by ada_service_generator.py
--  Package: Pyramid.Services.Tactical_Objects.Consumed
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

package Pyramid.Services.Tactical_Objects.Consumed is

   type Operation_Kind is
     (Op_Create,
      Op_Read,
      Op_Update,
      Op_Delete);

   type Service_Channel is
     (Ch_Read_Detail,
      Ch_Create_Requirement,
      Ch_Read_Requirement,
      Ch_Update_Requirement,
      Ch_Delete_Requirement,
      Ch_Read_Capability);

   type Identifier_Array is array (Positive range <>) of Identifier;
   type Object_Detail_Array is array (Positive range <>) of Object_Detail;
   type Object_Evidence_Requirement_Array is array (Positive range <>) of Object_Evidence_Requirement;

   --  -- Service wire-name constants (generated from proto) --------

   Svc_Read_Detail : constant String :=
     "object_evidence.read_detail";
   Svc_Create_Requirement : constant String :=
     "object_solution_evidence.create_requirement";
   Svc_Read_Requirement : constant String :=
     "object_solution_evidence.read_requirement";
   Svc_Update_Requirement : constant String :=
     "object_solution_evidence.update_requirement";
   Svc_Delete_Requirement : constant String :=
     "object_solution_evidence.delete_requirement";
   Svc_Read_Capability : constant String :=
     "object_source_capability.read_capability";

   --  -- Standard topic name constants --------------------------

   Topic_Object_Evidence : constant String :=
     "standard.object_evidence";

   --  -- PCL message utility ------------------------------------

   function Msg_To_String
     (Data : System.Address;
      Size : Interfaces.C.unsigned) return String;

   --  -- EntityActions handlers ------------------------------------
   --  Implement these procedures in the package body.

   --  Object_Evidence_Service
   procedure Handle_Read_Detail
     (Request  : in  Query;
      Response : out Object_Detail_Array);
   --  Object_Solution_Evidence_Service
   procedure Handle_Create_Requirement
     (Request  : in  Object_Evidence_Requirement;
      Response : out Identifier);
   procedure Handle_Read_Requirement
     (Request  : in  Query;
      Response : out Object_Evidence_Requirement_Array);
   procedure Handle_Update_Requirement
     (Request  : in  Object_Evidence_Requirement;
      Response : out Ack);
   procedure Handle_Delete_Requirement
     (Request  : in  Identifier;
      Response : out Ack);
   --  Object_Source_Capability_Service
   procedure Handle_Read_Capability
     (Request  : in  Query;
      Response : out Identifier_Array);

   --  -- PCL binding procedures ------------------------------------
   --  Subscribe/Invoke/Publish wrappers for PCL transport layer.
   --  Use Json_Codec to serialise/deserialise message payloads.

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

end Pyramid.Services.Tactical_Objects.Consumed;
