--  Auto-generated EntityActions service specification
--  Generated from: services by ada_service_generator.py
--  Package: Pyramid.Services.Tactical_Objects.Consumed
--
--  Each Handle_<Op>_<Entity> procedure corresponds to one EntityActions
--  CRUD operation.  The Dispatch procedure is the single integration
--  point for any transport (PCL, socket, shared memory, etc.).
--  DO NOT add Pyramid.Middleware.Send/Receive calls here.

with Pyramid.Model;  --  Identifier, Query, Ack
use  Pyramid.Model;

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

   type Capability_Array is array (Positive range <>) of Capability;
   type Detail_Array is array (Positive range <>) of Detail;
   type Requirement_Array is array (Positive range <>) of Requirement;

   --  -- EntityActions handlers -------------------------------------
   --  Implement these procedures in the package body.

   --  Object_Evidence_Service
   procedure Handle_Read_Detail
     (Request  : in  DetailQuery;
      Response : out Detail_Array);
   --  Object_Solution_Evidence_Service
   procedure Handle_Create_Requirement
     (Request  : in  Requirement;
      Response : out Identifier);
   procedure Handle_Read_Requirement
     (Request  : in  RequirementQuery;
      Response : out Requirement_Array);
   procedure Handle_Update_Requirement
     (Request  : in  Requirement;
      Response : out Ack);
   procedure Handle_Delete_Requirement
     (Request  : in  Identifier;
      Response : out Ack);
   --  Object_Source_Capability_Service
   procedure Handle_Read_Capability
     (Request  : in  CapabilityQuery;
      Response : out Capability_Array);

   --  -- Transport integration point ---------------------------------
   --  Route an incoming (channel, raw buffer) call to the correct
   --  typed handler.  The transport layer calls this; it never calls
   --  Handle_* procedures directly.

   procedure Dispatch
     (Channel      : in  Service_Channel;
      Request_Buf  : in  System.Address;
      Request_Size : in  Natural;
      Response_Buf : out System.Address;
      Response_Size: out Natural);

end Pyramid.Services.Tactical_Objects.Consumed;
