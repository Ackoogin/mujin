--  Auto-generated Ada C-ABI mirror specification
--  Generated from: pyramid.data_model.common.proto by generate_bindings.py (ada cabi)
--  Package: Pyramid.Data_Model.Common.Cabi

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Interfaces.C;
with Interfaces.C.Strings;
with System;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Base.Types;  use Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Base.Cabi;  use Pyramid.Data_Model.Base.Cabi;

package Pyramid.Data_Model.Common.Cabi is

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

   type Pyramid_Geodetic_Position_C is record
      Latitude : Interfaces.C.double;
      Longitude : Interfaces.C.double;
   end record;
   pragma Convention (C, Pyramid_Geodetic_Position_C);

   type Pyramid_Poly_Area_C is record
      Points : Pyramid_Slice_T;
   end record;
   pragma Convention (C, Pyramid_Poly_Area_C);

   type Pyramid_Entity_C is record
      Has_Update_Time : Interfaces.C.unsigned_char := 0;
      Update_Time : Interfaces.C.double;
      Id : Pyramid_Str_T;
      Source : Pyramid_Str_T;
   end record;
   pragma Convention (C, Pyramid_Entity_C);

   type Pyramid_Achievement_C is record
      Has_Update_Time : Interfaces.C.unsigned_char := 0;
      Update_Time : Interfaces.C.double;
      Id : Pyramid_Str_T;
      Source : Pyramid_Str_T;
      Status : Interfaces.C.int;
      Has_Quality : Interfaces.C.unsigned_char := 0;
      Quality : Interfaces.C.double;
      Achieveability : Interfaces.C.int;
   end record;
   pragma Convention (C, Pyramid_Achievement_C);

   type Pyramid_Requirement_C is record
      Has_Update_Time : Interfaces.C.unsigned_char := 0;
      Update_Time : Interfaces.C.double;
      Id : Pyramid_Str_T;
      Source : Pyramid_Str_T;
      Status : Pyramid_Achievement_C;
   end record;
   pragma Convention (C, Pyramid_Requirement_C);

   type Pyramid_Contraint_C is record
      Name : Pyramid_Str_T;
      Value : Interfaces.C.int;
   end record;
   pragma Convention (C, Pyramid_Contraint_C);

   type Pyramid_Capability_C is record
      Has_Update_Time : Interfaces.C.unsigned_char := 0;
      Update_Time : Interfaces.C.double;
      Id : Pyramid_Str_T;
      Source : Pyramid_Str_T;
      Availability : Interfaces.C.unsigned_char;
      Name : Pyramid_Str_T;
      Contraint : Pyramid_Slice_T;
   end record;
   pragma Convention (C, Pyramid_Capability_C);

   type Pyramid_Circle_Area_C is record
      Position : Pyramid_Geodetic_Position_C;
      Radius : Interfaces.C.double;
   end record;
   pragma Convention (C, Pyramid_Circle_Area_C);

   type Pyramid_Point_C is record
      Position : Pyramid_Geodetic_Position_C;
   end record;
   pragma Convention (C, Pyramid_Point_C);

   type Pyramid_Ack_C is record
      Success : Interfaces.C.unsigned_char;
   end record;
   pragma Convention (C, Pyramid_Ack_C);

   type Pyramid_Query_C is record
      Id : Pyramid_Slice_T;
      Has_One_Shot : Interfaces.C.unsigned_char := 0;
      One_Shot : Interfaces.C.unsigned_char;
   end record;
   pragma Convention (C, Pyramid_Query_C);

   procedure To_C
     (In_Value  : Geodetic_Position;
      Out_Value : out Pyramid_Geodetic_Position_C);
   procedure From_C
     (In_Value  : Pyramid_Geodetic_Position_C;
      Out_Value : out Geodetic_Position);
   procedure Free_Geodetic_Position (Value : access Pyramid_Geodetic_Position_C);
   pragma Import (C, Free_Geodetic_Position, "pyramid_GeodeticPosition_c_free");

   procedure To_C
     (In_Value  : Poly_Area;
      Out_Value : out Pyramid_Poly_Area_C);
   procedure From_C
     (In_Value  : Pyramid_Poly_Area_C;
      Out_Value : out Poly_Area);
   procedure Free_Poly_Area (Value : access Pyramid_Poly_Area_C);
   pragma Import (C, Free_Poly_Area, "pyramid_PolyArea_c_free");

   procedure To_C
     (In_Value  : Achievement;
      Out_Value : out Pyramid_Achievement_C);
   procedure From_C
     (In_Value  : Pyramid_Achievement_C;
      Out_Value : out Achievement);
   procedure Free_Achievement (Value : access Pyramid_Achievement_C);
   pragma Import (C, Free_Achievement, "pyramid_Achievement_c_free");

   procedure To_C
     (In_Value  : Requirement;
      Out_Value : out Pyramid_Requirement_C);
   procedure From_C
     (In_Value  : Pyramid_Requirement_C;
      Out_Value : out Requirement);
   procedure Free_Requirement (Value : access Pyramid_Requirement_C);
   pragma Import (C, Free_Requirement, "pyramid_Requirement_c_free");

   procedure To_C
     (In_Value  : Capability;
      Out_Value : out Pyramid_Capability_C);
   procedure From_C
     (In_Value  : Pyramid_Capability_C;
      Out_Value : out Capability);
   procedure Free_Capability (Value : access Pyramid_Capability_C);
   pragma Import (C, Free_Capability, "pyramid_Capability_c_free");

   procedure To_C
     (In_Value  : Entity;
      Out_Value : out Pyramid_Entity_C);
   procedure From_C
     (In_Value  : Pyramid_Entity_C;
      Out_Value : out Entity);
   procedure Free_Entity (Value : access Pyramid_Entity_C);
   pragma Import (C, Free_Entity, "pyramid_Entity_c_free");

   procedure To_C
     (In_Value  : Circle_Area;
      Out_Value : out Pyramid_Circle_Area_C);
   procedure From_C
     (In_Value  : Pyramid_Circle_Area_C;
      Out_Value : out Circle_Area);
   procedure Free_Circle_Area (Value : access Pyramid_Circle_Area_C);
   pragma Import (C, Free_Circle_Area, "pyramid_CircleArea_c_free");

   procedure To_C
     (In_Value  : Point;
      Out_Value : out Pyramid_Point_C);
   procedure From_C
     (In_Value  : Pyramid_Point_C;
      Out_Value : out Point);
   procedure Free_Point (Value : access Pyramid_Point_C);
   pragma Import (C, Free_Point, "pyramid_Point_c_free");

   procedure To_C
     (In_Value  : Contraint;
      Out_Value : out Pyramid_Contraint_C);
   procedure From_C
     (In_Value  : Pyramid_Contraint_C;
      Out_Value : out Contraint);
   procedure Free_Contraint (Value : access Pyramid_Contraint_C);
   pragma Import (C, Free_Contraint, "pyramid_Contraint_c_free");

   procedure To_C
     (In_Value  : Ack;
      Out_Value : out Pyramid_Ack_C);
   procedure From_C
     (In_Value  : Pyramid_Ack_C;
      Out_Value : out Ack);
   procedure Free_Ack (Value : access Pyramid_Ack_C);
   pragma Import (C, Free_Ack, "pyramid_Ack_c_free");

   procedure To_C
     (In_Value  : Query;
      Out_Value : out Pyramid_Query_C);
   procedure From_C
     (In_Value  : Pyramid_Query_C;
      Out_Value : out Query);
   procedure Free_Query (Value : access Pyramid_Query_C);
   pragma Import (C, Free_Query, "pyramid_Query_c_free");

end Pyramid.Data_Model.Common.Cabi;
