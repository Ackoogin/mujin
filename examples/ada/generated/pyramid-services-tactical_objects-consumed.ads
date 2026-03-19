--  Auto-generated EntityActions service specification
--  Generated from: services by ada_service_generator.py
--  Package: Pyramid.Services.Tactical_Objects.Consumed
--
--  Each Handle_<Op>_<Entity> procedure corresponds to one EntityActions
--  CRUD operation.  The Dispatch procedure is the single integration
--  point for any transport (PCL, socket, shared memory, etc.).

with Tactical_Objects_Types;  use Tactical_Objects_Types;
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

   --  -- Transport integration point ------------------------------

   procedure Dispatch
     (Channel      : in  Service_Channel;
      Request_Buf  : in  System.Address;
      Request_Size : in  Natural;
      Response_Buf : out System.Address;
      Response_Size: out Natural);

end Pyramid.Services.Tactical_Objects.Consumed;
