--  Auto-generated types specification
--  Generated from: pyramid.data_model.tactical.proto by generate_bindings.py (types)
--  Package: Pyramid.Data_Model.Tactical.Types

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;

package Pyramid.Data_Model.Tactical.Types is


   type Object_Source is
     (Source_Unspecified,
      Source_Radar,
      Source_Local);

   type Source_Array is array (Positive range <>) of Object_Source;
   type Source_Array_Acc is access all Source_Array;
   type Dimension_Array is array (Positive range <>) of Battle_Dimension;
   type Dimension_Array_Acc is access all Dimension_Array;

   type Object_Detail is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Entity_Source : Unbounded_String := Null_Unbounded_String;
      Source : Source_Array_Acc := null;
      Position : Geodetic_Position;
      Creation_Time : Long_Float := 0.0;
      Quality : Long_Float := 0.0;
      Course : Long_Float := 0.0;
      Speed : Long_Float := 0.0;
      Length : Long_Float := 0.0;
      Identity : Standard_Identity := Identity_Unspecified;
      Dimension : Battle_Dimension := Dimension_Unspecified;
   end record;

   type Object_Evidence_Requirement is record
      Base : Entity;
      Status : Achievement;
      Policy : Data_Policy := Policy_Unspecified;
      Dimension : Dimension_Array_Acc := null;
      --  oneof location
      Has_Val_Poly_Area : Boolean := False;
      Val_Poly_Area : Poly_Area;
      Has_Val_Circle_Area : Boolean := False;
      Val_Circle_Area : Circle_Area;
      Has_Val_Point : Boolean := False;
      Val_Point : Point;
   end record;

   type Object_Interest_Requirement is record
      Base : Entity;
      Status : Achievement;
      Source : Object_Source := Source_Unspecified;
      Policy : Data_Policy := Policy_Unspecified;
      Dimension : Dimension_Array_Acc := null;
      --  oneof location
      Has_Val_Poly_Area : Boolean := False;
      Val_Poly_Area : Poly_Area;
      Has_Val_Circle_Area : Boolean := False;
      Val_Circle_Area : Circle_Area;
      Has_Val_Point : Boolean := False;
      Val_Point : Point;
   end record;

   type Object_Match is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Matching_Object_Id : Unbounded_String := Null_Unbounded_String;
   end record;

end Pyramid.Data_Model.Tactical.Types;
