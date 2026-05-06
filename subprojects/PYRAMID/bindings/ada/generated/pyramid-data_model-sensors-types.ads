--  Auto-generated types specification
--  Generated from: pyramid.data_model.sensors.proto by generate_bindings.py (types)
--  Package: Pyramid.Data_Model.Sensors.Types

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Radar.Types;  use Pyramid.Data_Model.Radar.Types;
with Pyramid.Data_Model.Sensorproducts.Types;  use Pyramid.Data_Model.Sensorproducts.Types;

package Pyramid.Data_Model.Sensors.Types is


   type Interpretation_Policy is
     (Policy_Unspecified,
      Policy_IgnoreObjects,
      Policy_IncludeObjects);

   type Interpretation_Type is
     (Type_Unspecified,
      Type_LocateSeaSurfaceObjects);

   type Interpretation_Requirement is record
      Base : Entity;
      Status : Achievement;
      Policy : Interpretation_Policy := Policy_Unspecified;
      Type_Field : Interpretation_Type := Type_Unspecified;
      --  oneof location
      Has_Val_Poly_Area : Boolean := False;
      Val_Poly_Area : Poly_Area;
      Has_Val_Circle_Area : Boolean := False;
      Val_Circle_Area : Circle_Area;
      Has_Val_Point : Boolean := False;
      Val_Point : Point;
   end record;

   type Manual_Track_Requirement is record
      Base : Entity;
      Status : Achievement;
      Position : Geodetic_Position;
   end record;

   type Ati_Requirement is record
      Base : Entity;
      Status : Achievement;
      Auto_Zone : Poly_Area;
   end record;

   type Track_Provision_Requirement is record
      Base : Entity;
      Status : Achievement;
      --  oneof location
      Has_Val_Poly_Area : Boolean := False;
      Val_Poly_Area : Poly_Area;
      Has_Val_Circle_Area : Boolean := False;
      Val_Circle_Area : Circle_Area;
      Has_Val_Point : Boolean := False;
      Val_Point : Point;
   end record;

   type Object_Evidence_Provision_Requirement is record
      Base : Entity;
      Status : Achievement;
      --  oneof location
      Has_Val_Poly_Area : Boolean := False;
      Val_Poly_Area : Poly_Area;
      Has_Val_Circle_Area : Boolean := False;
      Val_Circle_Area : Circle_Area;
      Has_Val_Point : Boolean := False;
      Val_Point : Point;
   end record;

   type Object_Aquisition_Requirement is record
      Base : Entity;
      Status : Achievement;
      --  oneof location
      Has_Val_Poly_Area : Boolean := False;
      Val_Poly_Area : Poly_Area;
      Has_Val_Circle_Area : Boolean := False;
      Val_Circle_Area : Circle_Area;
      Has_Val_Point : Boolean := False;
      Val_Point : Point;
   end record;

   type Sensor_Object is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Position : Geodetic_Position;
      Creation_Time : Long_Float := 0.0;
      Quality : Long_Float := 0.0;
      Course : Long_Float := 0.0;
      Speed : Long_Float := 0.0;
      Length : Long_Float := 0.0;
   end record;

   type Radar_Mode_Change_Requirement is record
      Base : Entity;
      Status : Achievement;
      Mode : Radar_Operational_Mode := Mode_Unspecified;
   end record;

end Pyramid.Data_Model.Sensors.Types;
