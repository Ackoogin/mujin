--  Auto-generated EntityActions service specification
--  Generated from: services by ada_service_generator.py
--  Package: Pyramid.Services.Tactical_Objects.Provided
--
--  Each Handle_<Op>_<Entity> procedure corresponds to one EntityActions
--  CRUD operation.  The Dispatch procedure is the single integration
--  point for any transport (PCL, socket, shared memory, etc.).

with Tactical_Objects_Types;  use Tactical_Objects_Types;
with System;

package Pyramid.Services.Tactical_Objects.Provided is

   type Operation_Kind is
     (Op_Create,
      Op_Read,
      Op_Update,
      Op_Delete);

   type Service_Channel is
     (Ch_Read_Match,
      Ch_Create_Requirement,
      Ch_Read_Requirement,
      Ch_Update_Requirement,
      Ch_Delete_Requirement,
      Ch_Read_Detail);

   type Object_Detail_Array is array (Positive range <>) of Object_Detail;
   type Object_Interest_Requirement_Array is array (Positive range <>) of Object_Interest_Requirement;
   type Object_Match_Array is array (Positive range <>) of Object_Match;

   --  -- Service wire-name constants (generated from proto) --------

   Svc_Read_Match : constant String :=
     "matching_objects.read_match";
   Svc_Create_Requirement : constant String :=
     "object_of_interest.create_requirement";
   Svc_Read_Requirement : constant String :=
     "object_of_interest.read_requirement";
   Svc_Update_Requirement : constant String :=
     "object_of_interest.update_requirement";
   Svc_Delete_Requirement : constant String :=
     "object_of_interest.delete_requirement";
   Svc_Read_Detail : constant String :=
     "specific_object_detail.read_detail";

   --  -- Standard topic name constants --------------------------

   Topic_Entity_Matches : constant String :=
     "standard.entity_matches";
   Topic_Evidence_Requirements : constant String :=
     "standard.evidence_requirements";

   --  -- EntityActions handlers ------------------------------------
   --  Implement these procedures in the package body.

   --  Matching_Objects_Service
   procedure Handle_Read_Match
     (Request  : in  Query;
      Response : out Object_Match_Array);
   --  Object_Of_Interest_Service
   procedure Handle_Create_Requirement
     (Request  : in  Object_Interest_Requirement;
      Response : out Identifier);
   procedure Handle_Read_Requirement
     (Request  : in  Query;
      Response : out Object_Interest_Requirement_Array);
   procedure Handle_Update_Requirement
     (Request  : in  Object_Interest_Requirement;
      Response : out Ack);
   procedure Handle_Delete_Requirement
     (Request  : in  Identifier;
      Response : out Ack);
   --  Specific_Object_Detail_Service
   procedure Handle_Read_Detail
     (Request  : in  Query;
      Response : out Object_Detail_Array);

   --  -- JSON builder functions (GNATCOLL.JSON) -----------------

   function Build_Standard_Requirement_Json
     (Policy      : String;
      Identity    : String;
      Dimension   : String := "";
      Min_Lat_Rad : Long_Float := 0.0;
      Max_Lat_Rad : Long_Float := 0.0;
      Min_Lon_Rad : Long_Float := 0.0;
      Max_Lon_Rad : Long_Float := 0.0) return String;

   function Build_Standard_Evidence_Json
     (Identity    : String;
      Dimension   : String;
      Lat_Rad     : Long_Float;
      Lon_Rad     : Long_Float;
      Confidence  : Long_Float;
      Observed_At : Long_Float := 0.5) return String;

   --  -- Transport integration point ------------------------------

   procedure Dispatch
     (Channel      : in  Service_Channel;
      Request_Buf  : in  System.Address;
      Request_Size : in  Natural;
      Response_Buf : out System.Address;
      Response_Size: out Natural);

end Pyramid.Services.Tactical_Objects.Provided;
