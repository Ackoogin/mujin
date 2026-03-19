--  Auto-generated EntityActions service body
--  Package body: Pyramid.Services.Tactical_Objects.Consumed
--  TODO: replace null stubs with real implementations.

with System;

package body Pyramid.Services.Tactical_Objects.Consumed is

   --  -- Object_Evidence_Service -------------------------------------
   procedure Handle_Read_Detail
     (Request  : in  DetailQuery;
      Response : out Detail_Array)
   is
   begin
      null;  --  TODO: implement
   end Handle_Read_Detail;

   --  -- Object_Solution_Evidence_Service -------------------------------------
   procedure Handle_Create_Requirement
     (Request  : in  Requirement;
      Response : out Identifier)
   is
   begin
      null;  --  TODO: implement
   end Handle_Create_Requirement;

   procedure Handle_Read_Requirement
     (Request  : in  RequirementQuery;
      Response : out Requirement_Array)
   is
   begin
      null;  --  TODO: implement
   end Handle_Read_Requirement;

   procedure Handle_Update_Requirement
     (Request  : in  Requirement;
      Response : out Ack)
   is
   begin
      null;  --  TODO: implement
   end Handle_Update_Requirement;

   procedure Handle_Delete_Requirement
     (Request  : in  Identifier;
      Response : out Ack)
   is
   begin
      null;  --  TODO: implement
   end Handle_Delete_Requirement;

   --  -- Object_Source_Capability_Service -------------------------------------
   procedure Handle_Read_Capability
     (Request  : in  CapabilityQuery;
      Response : out Capability_Array)
   is
   begin
      null;  --  TODO: implement
   end Handle_Read_Capability;

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
         when Ch_Read_Detail =>
            null;  --  TODO: deserialise Request_Buf, call Handle_Read_Detail
         when Ch_Create_Requirement =>
            null;  --  TODO: deserialise Request_Buf, call Handle_Create_Requirement
         when Ch_Read_Requirement =>
            null;  --  TODO: deserialise Request_Buf, call Handle_Read_Requirement
         when Ch_Update_Requirement =>
            null;  --  TODO: deserialise Request_Buf, call Handle_Update_Requirement
         when Ch_Delete_Requirement =>
            null;  --  TODO: deserialise Request_Buf, call Handle_Delete_Requirement
         when Ch_Read_Capability =>
            null;  --  TODO: deserialise Request_Buf, call Handle_Read_Capability
      end case;
   end Dispatch;

end Pyramid.Services.Tactical_Objects.Consumed;
