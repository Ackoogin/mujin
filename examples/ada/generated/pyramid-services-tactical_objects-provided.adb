--  Auto-generated EntityActions service body
--  Package body: Pyramid.Services.Tactical_Objects.Provided

with GNATCOLL.JSON;  use GNATCOLL.JSON;
with System;

package body Pyramid.Services.Tactical_Objects.Provided is

   --  -- Matching_Objects_Service ------------------------------------
   procedure Handle_Read_Match
     (Request  : in  Query;
      Response : out Object_Match_Array)
   is
      pragma Unreferenced (Request);
      Empty : Object_Match_Array (1 .. 0);
   begin
      Response := Empty;
   end Handle_Read_Match;

   --  -- Object_Of_Interest_Service ------------------------------------
   procedure Handle_Create_Requirement
     (Request  : in  Object_Interest_Requirement;
      Response : out Identifier)
   is
      pragma Unreferenced (Request);
   begin
      Response := Null_Identifier;
   end Handle_Create_Requirement;

   procedure Handle_Read_Requirement
     (Request  : in  Query;
      Response : out Object_Interest_Requirement_Array)
   is
      pragma Unreferenced (Request);
      Empty : Object_Interest_Requirement_Array (1 .. 0);
   begin
      Response := Empty;
   end Handle_Read_Requirement;

   procedure Handle_Update_Requirement
     (Request  : in  Object_Interest_Requirement;
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

   --  -- Specific_Object_Detail_Service ------------------------------------
   procedure Handle_Read_Detail
     (Request  : in  Query;
      Response : out Object_Detail_Array)
   is
      pragma Unreferenced (Request);
      Empty : Object_Detail_Array (1 .. 0);
   begin
      Response := Empty;
   end Handle_Read_Detail;

   --  -- JSON builder: Build_Standard_Requirement_Json ----------

   function Build_Standard_Requirement_Json
     (Policy      : String;
      Identity    : String;
      Dimension   : String := "";
      Min_Lat_Rad : Long_Float := 0.0;
      Max_Lat_Rad : Long_Float := 0.0;
      Min_Lon_Rad : Long_Float := 0.0;
      Max_Lon_Rad : Long_Float := 0.0) return String
   is
      Obj : JSON_Value := Create_Object;
   begin
      Set_Field (Obj, "policy",   Policy);
      Set_Field (Obj, "identity", Identity);
      if Dimension /= "" then
         Set_Field (Obj, "dimension", Dimension);
      end if;
      Set_Field_Long_Float (Obj, "min_lat_rad", Min_Lat_Rad);
      Set_Field_Long_Float (Obj, "max_lat_rad", Max_Lat_Rad);
      Set_Field_Long_Float (Obj, "min_lon_rad", Min_Lon_Rad);
      Set_Field_Long_Float (Obj, "max_lon_rad", Max_Lon_Rad);
      return Write (Obj);
   end Build_Standard_Requirement_Json;

   --  -- JSON builder: Build_Standard_Evidence_Json -------------

   function Build_Standard_Evidence_Json
     (Identity    : String;
      Dimension   : String;
      Lat_Rad     : Long_Float;
      Lon_Rad     : Long_Float;
      Confidence  : Long_Float;
      Observed_At : Long_Float := 0.5) return String
   is
      Obj : JSON_Value := Create_Object;
   begin
      Set_Field (Obj, "identity",      Identity);
      Set_Field (Obj, "dimension",     Dimension);
      Set_Field_Long_Float (Obj, "latitude_rad",  Lat_Rad);
      Set_Field_Long_Float (Obj, "longitude_rad", Lon_Rad);
      Set_Field_Long_Float (Obj, "confidence",    Confidence);
      Set_Field_Long_Float (Obj, "observed_at",   Observed_At);
      return Write (Obj);
   end Build_Standard_Evidence_Json;

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
         when Ch_Read_Match =>
            null;  --  TODO: deserialise, call Handle_Read_Match
         when Ch_Create_Requirement =>
            null;  --  TODO: deserialise, call Handle_Create_Requirement
         when Ch_Read_Requirement =>
            null;  --  TODO: deserialise, call Handle_Read_Requirement
         when Ch_Update_Requirement =>
            null;  --  TODO: deserialise, call Handle_Update_Requirement
         when Ch_Delete_Requirement =>
            null;  --  TODO: deserialise, call Handle_Delete_Requirement
         when Ch_Read_Detail =>
            null;  --  TODO: deserialise, call Handle_Read_Detail
      end case;
   end Dispatch;

end Pyramid.Services.Tactical_Objects.Provided;
