--  Auto-generated types specification
--  Generated from: proto/pyramid/data_model by ada_service_generator.py --types
--  Package: Tactical_Objects_Types

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;

package Tactical_Objects_Types is

   subtype Angle is Long_Float;
   subtype Length is Long_Float;
   subtype Timestamp is Long_Float;
   subtype Identifier is Unbounded_String;
   subtype Speed is Long_Float;
   subtype Percentage is Long_Float;

   type Feasibility is
     (Feasibility_Unspecified,
      Feasibility_Feasible,
      Feasibility_NotFeasible,
      Feasibility_PartiallyFeasible,
      Feasibility_Pending);

   type Progress is
     (Progress_Unspecified,
      Progress_NotStarted,
      Progress_InProgress,
      Progress_Completed,
      Progress_Cancelled,
      Progress_Failed);

   type Standard_Identity is
     (Identity_Unspecified,
      Identity_Unknown,
      Identity_Friendly,
      Identity_Hostile,
      Identity_Suspect,
      Identity_Neutral,
      Identity_Pending,
      Identity_Joker,
      Identity_Faker,
      Identity_AssumedFriendly);

   type Battle_Dimension is
     (Dimension_Unspecified,
      Dimension_Ground,
      Dimension_Subsurface,
      Dimension_SeaSurface,
      Dimension_Air,
      Dimension_Unknown);

   type Data_Policy is
     (Policy_Unspecified,
      Policy_Query,
      Policy_Obtain);

   type Object_Source is
     (Source_Unspecified,
      Source_Radar,
      Source_Local);

   type Id_Array is array (Positive range <>) of Unbounded_String;
   type Source_Array is array (Positive range <>) of Object_Source;
   type Dimension_Array is array (Positive range <>) of Battle_Dimension;

   type Geodetic_Position is record
      Latitude : Long_Float := 0.0;
      Longitude : Long_Float := 0.0;
   end record;

   type Points_Array is array (Positive range <>) of Geodetic_Position;

   type Poly_Area is record
      Points : Points_Array;
   end record;

   type Entity is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
   end record;

   type Achievement is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Status : Progress := Progress_Unspecified;
      Quality : Long_Float := 0.0;
      Achieveability : Feasibility := Feasibility_Unspecified;
   end record;

   type Requirement is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Status : Achievement;
   end record;

   type Contraint is record
      Name : Unbounded_String := Null_Unbounded_String;
      Value : Integer := 0;
   end record;

   type Contraint_Array is array (Positive range <>) of Contraint;

   type Capability is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Availability : Boolean := False;
      Name : Unbounded_String := Null_Unbounded_String;
      Contraint : Contraint_Array;
   end record;

   type Circle_Area is record
      Position : Geodetic_Position;
      Radius : Long_Float := 0.0;
   end record;

   type Point is record
      Position : Geodetic_Position;
   end record;

   type Ack is record
      Success : Boolean := False;
   end record;

   type Query is record
      Id : Id_Array;
      Has_One_Shot : Boolean := False;
      One_Shot : Boolean := False;
   end record;

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

end Tactical_Objects_Types;
