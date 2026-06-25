--  Auto-generated Ada C-ABI mirror specification
--  Generated from: pyramid.data_model.tactical.proto by generate_bindings.py (ada cabi)
--  Package: Pyramid.Data_Model.Tactical.Cabi

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Interfaces.C;
with Interfaces.C.Strings;
with System;
with Pyramid.Data_Model.Tactical.Types;  use Pyramid.Data_Model.Tactical.Types;
with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Base.Cabi;  use Pyramid.Data_Model.Base.Cabi;
with Pyramid.Data_Model.Common.Cabi;  use Pyramid.Data_Model.Common.Cabi;

package Pyramid.Data_Model.Tactical.Cabi is

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

   type Pyramid_Object_Detail_C is record
      Has_Update_Time : Interfaces.C.unsigned_char := 0;
      Update_Time : Interfaces.C.double;
      Id : Pyramid_Str_T;
      Entity_Source : Pyramid_Str_T;
      Source : Pyramid_Slice_T;
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
      Identity : Interfaces.C.int;
      Dimension : Interfaces.C.int;
   end record;
   pragma Convention (C, Pyramid_Object_Detail_C);

   type Pyramid_Object_Evidence_Requirement_C is record
      Base : Pyramid_Entity_C;
      Status : Pyramid_Achievement_C;
      Policy : Interfaces.C.int;
      Dimension : Pyramid_Slice_T;
      Has_Poly_Area : Interfaces.C.unsigned_char := 0;
      Poly_Area : Pyramid_Poly_Area_C;
      Has_Circle_Area : Interfaces.C.unsigned_char := 0;
      Circle_Area : Pyramid_Circle_Area_C;
      Has_Point : Interfaces.C.unsigned_char := 0;
      Point : Pyramid_Point_C;
   end record;
   pragma Convention (C, Pyramid_Object_Evidence_Requirement_C);

   type Pyramid_Object_Interest_Requirement_C is record
      Base : Pyramid_Entity_C;
      Status : Pyramid_Achievement_C;
      Has_Source : Interfaces.C.unsigned_char := 0;
      Source : Interfaces.C.int;
      Policy : Interfaces.C.int;
      Dimension : Pyramid_Slice_T;
      Has_Poly_Area : Interfaces.C.unsigned_char := 0;
      Poly_Area : Pyramid_Poly_Area_C;
      Has_Circle_Area : Interfaces.C.unsigned_char := 0;
      Circle_Area : Pyramid_Circle_Area_C;
      Has_Point : Interfaces.C.unsigned_char := 0;
      Point : Pyramid_Point_C;
   end record;
   pragma Convention (C, Pyramid_Object_Interest_Requirement_C);

   type Pyramid_Object_Match_C is record
      Has_Update_Time : Interfaces.C.unsigned_char := 0;
      Update_Time : Interfaces.C.double;
      Id : Pyramid_Str_T;
      Source : Pyramid_Str_T;
      Matching_Object_Id : Pyramid_Str_T;
   end record;
   pragma Convention (C, Pyramid_Object_Match_C);

   procedure To_C
     (In_Value  : Object_Detail;
      Out_Value : out Pyramid_Object_Detail_C);
   procedure From_C
     (In_Value  : Pyramid_Object_Detail_C;
      Out_Value : out Object_Detail);
   procedure Free_Object_Detail (Value : access Pyramid_Object_Detail_C);
   pragma Import (C, Free_Object_Detail, "pyramid_ObjectDetail_c_free");

   procedure To_C
     (In_Value  : Object_Evidence_Requirement;
      Out_Value : out Pyramid_Object_Evidence_Requirement_C);
   procedure From_C
     (In_Value  : Pyramid_Object_Evidence_Requirement_C;
      Out_Value : out Object_Evidence_Requirement);
   procedure Free_Object_Evidence_Requirement (Value : access Pyramid_Object_Evidence_Requirement_C);
   pragma Import (C, Free_Object_Evidence_Requirement, "pyramid_ObjectEvidenceRequirement_c_free");

   procedure To_C
     (In_Value  : Object_Interest_Requirement;
      Out_Value : out Pyramid_Object_Interest_Requirement_C);
   procedure From_C
     (In_Value  : Pyramid_Object_Interest_Requirement_C;
      Out_Value : out Object_Interest_Requirement);
   procedure Free_Object_Interest_Requirement (Value : access Pyramid_Object_Interest_Requirement_C);
   pragma Import (C, Free_Object_Interest_Requirement, "pyramid_ObjectInterestRequirement_c_free");

   procedure To_C
     (In_Value  : Object_Match;
      Out_Value : out Pyramid_Object_Match_C);
   procedure From_C
     (In_Value  : Pyramid_Object_Match_C;
      Out_Value : out Object_Match);
   procedure Free_Object_Match (Value : access Pyramid_Object_Match_C);
   pragma Import (C, Free_Object_Match, "pyramid_ObjectMatch_c_free");

end Pyramid.Data_Model.Tactical.Cabi;
