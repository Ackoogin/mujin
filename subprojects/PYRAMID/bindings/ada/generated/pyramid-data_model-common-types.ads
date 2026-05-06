--  Auto-generated types specification
--  Generated from: pyramid.data_model.common.proto by generate_bindings.py (types)
--  Package: Pyramid.Data_Model.Common.Types

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;

package Pyramid.Data_Model.Common.Types is


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

   type Id_Array is array (Positive range <>) of Unbounded_String;
   type Id_Array_Acc is access all Id_Array;

   type Geodetic_Position is record
      Latitude : Long_Float := 0.0;
      Longitude : Long_Float := 0.0;
   end record;

   type Points_Array is array (Positive range <>) of Geodetic_Position;
   type Points_Array_Acc is access all Points_Array;

   type Poly_Area is record
      Points : Points_Array_Acc := null;
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
   type Contraint_Array_Acc is access all Contraint_Array;

   type Capability is record
      Update_Time : Long_Float := 0.0;
      Id : Unbounded_String := Null_Unbounded_String;
      Source : Unbounded_String := Null_Unbounded_String;
      Availability : Boolean := False;
      Name : Unbounded_String := Null_Unbounded_String;
      Contraint : Contraint_Array_Acc := null;
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
      Id : Id_Array_Acc := null;
      Has_One_Shot : Boolean := False;
      One_Shot : Boolean := False;
   end record;

end Pyramid.Data_Model.Common.Types;
