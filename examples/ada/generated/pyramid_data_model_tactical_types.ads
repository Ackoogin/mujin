--  Auto-generated types specification
--  Generated from: tactical.proto by ada_service_generator.py --types
--  Package: Pyramid_Data_Model_Tactical_Types

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid_Data_Model_Base_Types;  use Pyramid_Data_Model_Base_Types;
with Pyramid_Data_Model_Common_Types;  use Pyramid_Data_Model_Common_Types;

package Pyramid_Data_Model_Tactical_Types is


   type Object_Source is
     (Source_Unspecified,
      Source_Radar,
      Source_Local);

   type Source_Array is array (Positive range <>) of Object_Source;
   type Dimension_Array is array (Positive range <>) of Battle_Dimension;

   type Object_Detail is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Entity_Source : Unbounded_String := Null_Unbounded_String;
      Source : Source_Array;
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
      Dimension : Dimension_Array;
      --  oneof location
      Has_Poly_Area : Boolean := False;
      Poly_Area : Poly_Area;
      Has_Circle_Area : Boolean := False;
      Circle_Area : Circle_Area;
      Has_Point : Boolean := False;
      Point : Point;
   end record;

   type Object_Interest_Requirement is record
      Base : Entity;
      Status : Achievement;
      Source : Object_Source := Source_Unspecified;
      Policy : Data_Policy := Policy_Unspecified;
      Dimension : Dimension_Array;
      --  oneof location
      Has_Poly_Area : Boolean := False;
      Poly_Area : Poly_Area;
      Has_Circle_Area : Boolean := False;
      Circle_Area : Circle_Area;
      Has_Point : Boolean := False;
      Point : Point;
   end record;

   type Object_Match is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Matching_Object_Id : Unbounded_String := Null_Unbounded_String;
   end record;

end Pyramid_Data_Model_Tactical_Types;
