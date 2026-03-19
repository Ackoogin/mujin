--  Auto-generated EntityActions service specification
--  Generated from: tactical_objects.proto by ada_service_generator.py
--  Package: Pyramid.Services.Tactical_Objects.Provided
--
--  Each Handle_<Op>_<Entity> procedure corresponds to one EntityActions
--  CRUD operation.  The Dispatch procedure is the single integration
--  point for any transport (PCL, socket, shared memory, etc.).
--  DO NOT add Pyramid.Middleware.Send/Receive calls here.

with Pyramid.Model;  --  Identifier, Query, Ack
use  Pyramid.Model;

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

   type TacticalObject_Array is array (Positive range <>) of TacticalObject;
   type Zone_Array is array (Positive range <>) of Zone;

   --  -- EntityActions handlers -------------------------------------
   --  Implement these procedures in the package body.

   --  TacticalObjectService
   procedure Handle_Create_Tactical_Object
     (Request  : in  TacticalObject;
      Response : out Identifier);
   procedure Handle_Read_Tactical_Object
     (Request  : in  TacticalObjectQuery;
      Response : out TacticalObject_Array);
   procedure Handle_Update_Tactical_Object
     (Request  : in  TacticalObject;
      Response : out Ack);
   procedure Handle_Delete_Tactical_Object
     (Request  : in  Identifier;
      Response : out Ack);
   --  ZoneService
   procedure Handle_Create_Zone
     (Request  : in  Zone;
      Response : out Identifier);
   procedure Handle_Read_Zone
     (Request  : in  ZoneQuery;
      Response : out Zone_Array);
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

end Pyramid.Services.Tactical_Objects.Provided;
