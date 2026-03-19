--  Auto-generated EntityActions service body
--  Package body: Pyramid.Services.Tactical_Objects.Consumed

with System;

package body Pyramid.Services.Tactical_Objects.Consumed is

   --  -- Object_Evidence_Service ------------------------------------
   procedure Handle_Read_Detail
     (Request  : in  Query;
      Response : out Object_Detail_Array)
   is
      pragma Unreferenced (Request);
      Empty : Object_Detail_Array (1 .. 0);
   begin
      Response := Empty;
   end Handle_Read_Detail;

   --  -- Object_Solution_Evidence_Service ------------------------------------
   procedure Handle_Create_Requirement
     (Request  : in  Object_Evidence_Requirement;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Identifier;
   end Handle_Create_Requirement;

   procedure Handle_Read_Requirement
     (Request  : in  Query;
      Response : out Object_Evidence_Requirement_Array)
   is
      pragma Unreferenced (Request);
      Empty : Object_Evidence_Requirement_Array (1 .. 0);
   begin
      Response := Empty;
   end Handle_Read_Requirement;

   procedure Handle_Update_Requirement
     (Request  : in  Object_Evidence_Requirement;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := Ack_Ok;
   end Handle_Update_Requirement;

   procedure Handle_Delete_Requirement
     (Request  : in  Identifier;
      Response : out Ack)
   is
      pragma Unreferenced (Request);
   begin
      Response := Ack_Ok;
   end Handle_Delete_Requirement;

   --  -- Object_Source_Capability_Service ------------------------------------
   procedure Handle_Read_Capability
     (Request  : in  Query;
      Response : out Identifier_Array)
   is
      pragma Unreferenced (Request);
      Empty : Identifier_Array (1 .. 0);
   begin
      Response := Empty;
   end Handle_Read_Capability;

   procedure Dispatch
     (Channel      : in  Service_Channel;
      Request_Buf  : in  System.Address;
      Request_Size : in  Natural;
      Response_Buf : out System.Address;
      Response_Size: out Natural)
   is
      pragma Unreferenced (Request_Buf, Request_Size);
   begin
      Response_Buf  := System.Null_Address;
      Response_Size := 0;
      case Channel is
         when Ch_Read_Detail =>
            null;  --  TODO: deserialise, call Handle_Read_Detail
         when Ch_Create_Requirement =>
            null;  --  TODO: deserialise, call Handle_Create_Requirement
         when Ch_Read_Requirement =>
            null;  --  TODO: deserialise, call Handle_Read_Requirement
         when Ch_Update_Requirement =>
            null;  --  TODO: deserialise, call Handle_Update_Requirement
         when Ch_Delete_Requirement =>
            null;  --  TODO: deserialise, call Handle_Delete_Requirement
         when Ch_Read_Capability =>
            null;  --  TODO: deserialise, call Handle_Read_Capability
      end case;
   end Dispatch;

end Pyramid.Services.Tactical_Objects.Consumed;
