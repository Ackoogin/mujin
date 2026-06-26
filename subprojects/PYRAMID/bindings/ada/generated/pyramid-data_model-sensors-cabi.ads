--  Auto-generated Ada C-ABI mirror specification
--  Generated from: pyramid.data_model.sensors.proto by generate_bindings.py (ada cabi)
--  Package: Pyramid.Data_Model.Sensors.Cabi

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Interfaces.C;
with Interfaces.C.Strings;
with System;
with Pyramid.Data_Model.Sensors.Types;  use Pyramid.Data_Model.Sensors.Types;
with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Radar.Types;  use Pyramid.Data_Model.Radar.Types;
with Pyramid.Data_Model.Sensorproducts.Types;  use Pyramid.Data_Model.Sensorproducts.Types;
with Pyramid.Data_Model.Base.Cabi;  use Pyramid.Data_Model.Base.Cabi;
with Pyramid.Data_Model.Common.Cabi;  use Pyramid.Data_Model.Common.Cabi;
with Pyramid.Data_Model.Radar.Cabi;  use Pyramid.Data_Model.Radar.Cabi;
with Pyramid.Data_Model.Sensorproducts.Cabi;  use Pyramid.Data_Model.Sensorproducts.Cabi;

package Pyramid.Data_Model.Sensors.Cabi is

   pragma Elaborate_Body;

   type Pyramid_Str_T is record
      Ptr : Interfaces.C.Strings.chars_ptr :=
        Interfaces.C.Strings.Null_Ptr;
      Len : Interfaces.C.unsigned := 0;
   end record;
   pragma Convention (C, Pyramid_Str_T);

   type Pyramid_Slice_T is record
      Ptr : System.Address := System.Null_Address;
      Len : Interfaces.C.unsigned := 0;
   end record;
   pragma Convention (C, Pyramid_Slice_T);

   type Pyramid_Interpretation_Requirement_C is record
      Base : Pyramid_Entity_C;
      Status : Pyramid_Achievement_C;
      Policy : Interfaces.C.int;
      Type_Field : Interfaces.C.int;
      Has_Poly_Area : Interfaces.C.unsigned_char := 0;
      Poly_Area : Pyramid_Poly_Area_C;
      Has_Circle_Area : Interfaces.C.unsigned_char := 0;
      Circle_Area : Pyramid_Circle_Area_C;
      Has_Point : Interfaces.C.unsigned_char := 0;
      Point : Pyramid_Point_C;
   end record;
   pragma Convention (C, Pyramid_Interpretation_Requirement_C);

   type Pyramid_Manual_Track_Requirement_C is record
      Base : Pyramid_Entity_C;
      Status : Pyramid_Achievement_C;
      Position : Pyramid_Geodetic_Position_C;
   end record;
   pragma Convention (C, Pyramid_Manual_Track_Requirement_C);

   type Pyramid_Ati_Requirement_C is record
      Base : Pyramid_Entity_C;
      Status : Pyramid_Achievement_C;
      Auto_Zone : Pyramid_Poly_Area_C;
   end record;
   pragma Convention (C, Pyramid_Ati_Requirement_C);

   type Pyramid_Track_Provision_Requirement_C is record
      Base : Pyramid_Entity_C;
      Status : Pyramid_Achievement_C;
      Has_Poly_Area : Interfaces.C.unsigned_char := 0;
      Poly_Area : Pyramid_Poly_Area_C;
      Has_Circle_Area : Interfaces.C.unsigned_char := 0;
      Circle_Area : Pyramid_Circle_Area_C;
      Has_Point : Interfaces.C.unsigned_char := 0;
      Point : Pyramid_Point_C;
   end record;
   pragma Convention (C, Pyramid_Track_Provision_Requirement_C);

   type Pyramid_Object_Evidence_Provision_Requirement_C is record
      Base : Pyramid_Entity_C;
      Status : Pyramid_Achievement_C;
      Has_Poly_Area : Interfaces.C.unsigned_char := 0;
      Poly_Area : Pyramid_Poly_Area_C;
      Has_Circle_Area : Interfaces.C.unsigned_char := 0;
      Circle_Area : Pyramid_Circle_Area_C;
      Has_Point : Interfaces.C.unsigned_char := 0;
      Point : Pyramid_Point_C;
   end record;
   pragma Convention (C, Pyramid_Object_Evidence_Provision_Requirement_C);

   type Pyramid_Object_Aquisition_Requirement_C is record
      Base : Pyramid_Entity_C;
      Status : Pyramid_Achievement_C;
      Has_Poly_Area : Interfaces.C.unsigned_char := 0;
      Poly_Area : Pyramid_Poly_Area_C;
      Has_Circle_Area : Interfaces.C.unsigned_char := 0;
      Circle_Area : Pyramid_Circle_Area_C;
      Has_Point : Interfaces.C.unsigned_char := 0;
      Point : Pyramid_Point_C;
   end record;
   pragma Convention (C, Pyramid_Object_Aquisition_Requirement_C);

   type Pyramid_Sensor_Object_C is record
      Has_Update_Time : Interfaces.C.unsigned_char := 0;
      Update_Time : Interfaces.C.double;
      Id : Pyramid_Str_T;
      Source : Pyramid_Str_T;
      Position : Pyramid_Geodetic_Position_C;
      Creation_Time : Interfaces.C.double;
      Has_Quality : Interfaces.C.unsigned_char := 0;
      Quality : Interfaces.C.double;
      Has_Course : Interfaces.C.unsigned_char := 0;
      Course : Interfaces.C.double;
      Has_Speed : Interfaces.C.unsigned_char := 0;
      Speed : Interfaces.C.double;
      Has_Length : Interfaces.C.unsigned_char := 0;
      Length : Interfaces.C.double;
   end record;
   pragma Convention (C, Pyramid_Sensor_Object_C);

   type Pyramid_Radar_Mode_Change_Requirement_C is record
      Base : Pyramid_Entity_C;
      Status : Pyramid_Achievement_C;
      Mode : Interfaces.C.int;
   end record;
   pragma Convention (C, Pyramid_Radar_Mode_Change_Requirement_C);

   procedure To_C
     (In_Value  : Interpretation_Requirement;
      Out_Value : out Pyramid_Interpretation_Requirement_C);
   procedure From_C
     (In_Value  : Pyramid_Interpretation_Requirement_C;
      Out_Value : out Interpretation_Requirement);
   procedure Free_Interpretation_Requirement (Value : access Pyramid_Interpretation_Requirement_C);
   pragma Import (C, Free_Interpretation_Requirement, "pyramid_InterpretationRequirement_c_free");

   procedure To_C
     (In_Value  : Manual_Track_Requirement;
      Out_Value : out Pyramid_Manual_Track_Requirement_C);
   procedure From_C
     (In_Value  : Pyramid_Manual_Track_Requirement_C;
      Out_Value : out Manual_Track_Requirement);
   procedure Free_Manual_Track_Requirement (Value : access Pyramid_Manual_Track_Requirement_C);
   pragma Import (C, Free_Manual_Track_Requirement, "pyramid_ManualTrackRequirement_c_free");

   procedure To_C
     (In_Value  : Ati_Requirement;
      Out_Value : out Pyramid_Ati_Requirement_C);
   procedure From_C
     (In_Value  : Pyramid_Ati_Requirement_C;
      Out_Value : out Ati_Requirement);
   procedure Free_Ati_Requirement (Value : access Pyramid_Ati_Requirement_C);
   pragma Import (C, Free_Ati_Requirement, "pyramid_ATIRequirement_c_free");

   procedure To_C
     (In_Value  : Track_Provision_Requirement;
      Out_Value : out Pyramid_Track_Provision_Requirement_C);
   procedure From_C
     (In_Value  : Pyramid_Track_Provision_Requirement_C;
      Out_Value : out Track_Provision_Requirement);
   procedure Free_Track_Provision_Requirement (Value : access Pyramid_Track_Provision_Requirement_C);
   pragma Import (C, Free_Track_Provision_Requirement, "pyramid_TrackProvisionRequirement_c_free");

   procedure To_C
     (In_Value  : Object_Evidence_Provision_Requirement;
      Out_Value : out Pyramid_Object_Evidence_Provision_Requirement_C);
   procedure From_C
     (In_Value  : Pyramid_Object_Evidence_Provision_Requirement_C;
      Out_Value : out Object_Evidence_Provision_Requirement);
   procedure Free_Object_Evidence_Provision_Requirement (Value : access Pyramid_Object_Evidence_Provision_Requirement_C);
   pragma Import (C, Free_Object_Evidence_Provision_Requirement, "pyramid_ObjectEvidenceProvisionRequirement_c_free");

   procedure To_C
     (In_Value  : Object_Aquisition_Requirement;
      Out_Value : out Pyramid_Object_Aquisition_Requirement_C);
   procedure From_C
     (In_Value  : Pyramid_Object_Aquisition_Requirement_C;
      Out_Value : out Object_Aquisition_Requirement);
   procedure Free_Object_Aquisition_Requirement (Value : access Pyramid_Object_Aquisition_Requirement_C);
   pragma Import (C, Free_Object_Aquisition_Requirement, "pyramid_ObjectAquisitionRequirement_c_free");

   procedure To_C
     (In_Value  : Sensor_Object;
      Out_Value : out Pyramid_Sensor_Object_C);
   procedure From_C
     (In_Value  : Pyramid_Sensor_Object_C;
      Out_Value : out Sensor_Object);
   procedure Free_Sensor_Object (Value : access Pyramid_Sensor_Object_C);
   pragma Import (C, Free_Sensor_Object, "pyramid_SensorObject_c_free");

   procedure To_C
     (In_Value  : Radar_Mode_Change_Requirement;
      Out_Value : out Pyramid_Radar_Mode_Change_Requirement_C);
   procedure From_C
     (In_Value  : Pyramid_Radar_Mode_Change_Requirement_C;
      Out_Value : out Radar_Mode_Change_Requirement);
   procedure Free_Radar_Mode_Change_Requirement (Value : access Pyramid_Radar_Mode_Change_Requirement_C);
   pragma Import (C, Free_Radar_Mode_Change_Requirement, "pyramid_RadarModeChangeRequirement_c_free");

end Pyramid.Data_Model.Sensors.Cabi;
