--  Auto-generated EntityActions service body
--  Package body: Pyramid.Services.Tactical_Objects.Provided
--  TODO: replace null stubs with real implementations.

with System;

package body Pyramid.Services.Tactical_Objects.Provided is

   --  -- TacticalObjectService -------------------------------------
   procedure Handle_Create_Tactical_Object
     (Request  : in  TacticalObject;
      Response : out Identifier)
   is
   begin
      null;  --  TODO: implement
   end Handle_Create_Tactical_Object;

   procedure Handle_Read_Tactical_Object
     (Request  : in  TacticalObjectQuery;
      Response : out TacticalObject_Array)
   is
   begin
      null;  --  TODO: implement
   end Handle_Read_Tactical_Object;

   procedure Handle_Update_Tactical_Object
     (Request  : in  TacticalObject;
      Response : out Ack)
   is
   begin
      null;  --  TODO: implement
   end Handle_Update_Tactical_Object;

   procedure Handle_Delete_Tactical_Object
     (Request  : in  Identifier;
      Response : out Ack)
   is
   begin
      null;  --  TODO: implement
   end Handle_Delete_Tactical_Object;

   --  -- ZoneService -------------------------------------
   procedure Handle_Create_Zone
     (Request  : in  Zone;
      Response : out Identifier)
   is
   begin
      null;  --  TODO: implement
   end Handle_Create_Zone;

   procedure Handle_Read_Zone
     (Request  : in  ZoneQuery;
      Response : out Zone_Array)
   is
   begin
      null;  --  TODO: implement
   end Handle_Read_Zone;

   procedure Handle_Update_Zone
     (Request  : in  Zone;
      Response : out Ack)
   is
   begin
      null;  --  TODO: implement
   end Handle_Update_Zone;

   procedure Handle_Delete_Zone
     (Request  : in  Identifier;
      Response : out Ack)
   is
   begin
      null;  --  TODO: implement
   end Handle_Delete_Zone;

   --  -- ObservationIngressService -------------------------------------
   procedure Handle_Create_Observation
     (Request  : in  Observation;
      Response : out Identifier)
   is
   begin
      null;  --  TODO: implement
   end Handle_Create_Observation;

   procedure Dispatch
     (Channel      : in  Service_Channel;
      Request_Buf  : in  System.Address;
      Request_Size : in  Natural;
      Response_Buf : out System.Address;
      Response_Size: out Natural)
   is
   begin
      Response_Buf  := System.Null_Address;
      Response_Size := 0;
      case Channel is
         when Ch_Create_Tactical_Object =>
            null;  --  TODO: deserialise Request_Buf, call Handle_Create_Tactical_Object
         when Ch_Read_Tactical_Object =>
            null;  --  TODO: deserialise Request_Buf, call Handle_Read_Tactical_Object
         when Ch_Update_Tactical_Object =>
            null;  --  TODO: deserialise Request_Buf, call Handle_Update_Tactical_Object
         when Ch_Delete_Tactical_Object =>
            null;  --  TODO: deserialise Request_Buf, call Handle_Delete_Tactical_Object
         when Ch_Create_Zone =>
            null;  --  TODO: deserialise Request_Buf, call Handle_Create_Zone
         when Ch_Read_Zone =>
            null;  --  TODO: deserialise Request_Buf, call Handle_Read_Zone
         when Ch_Update_Zone =>
            null;  --  TODO: deserialise Request_Buf, call Handle_Update_Zone
         when Ch_Delete_Zone =>
            null;  --  TODO: deserialise Request_Buf, call Handle_Delete_Zone
         when Ch_Create_Observation =>
            null;  --  TODO: deserialise Request_Buf, call Handle_Create_Observation
      end case;
   end Dispatch;

end Pyramid.Services.Tactical_Objects.Provided;
