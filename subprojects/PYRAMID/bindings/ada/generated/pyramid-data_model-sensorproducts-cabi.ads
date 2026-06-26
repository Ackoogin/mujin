--  Auto-generated Ada C-ABI mirror specification
--  Generated from: pyramid.data_model.sensorproducts.proto by generate_bindings.py (ada cabi)
--  Package: Pyramid.Data_Model.Sensorproducts.Cabi

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Interfaces.C;
with Interfaces.C.Strings;
with System;
with Pyramid.Data_Model.Sensorproducts.Types;  use Pyramid.Data_Model.Sensorproducts.Types;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Common.Cabi;  use Pyramid.Data_Model.Common.Cabi;

package Pyramid.Data_Model.Sensorproducts.Cabi is

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

   type Pyramid_Radar_Display_Product_Requirement_C is record
      Base : Pyramid_Entity_C;
      Status : Pyramid_Achievement_C;
   end record;
   pragma Convention (C, Pyramid_Radar_Display_Product_Requirement_C);

   type Pyramid_Radar_Product_Requirement_C is record
      Base : Pyramid_Entity_C;
      Status : Pyramid_Achievement_C;
   end record;
   pragma Convention (C, Pyramid_Radar_Product_Requirement_C);

   procedure To_C
     (In_Value  : Radar_Display_Product_Requirement;
      Out_Value : out Pyramid_Radar_Display_Product_Requirement_C);
   procedure From_C
     (In_Value  : Pyramid_Radar_Display_Product_Requirement_C;
      Out_Value : out Radar_Display_Product_Requirement);
   procedure Free_Radar_Display_Product_Requirement (Value : access Pyramid_Radar_Display_Product_Requirement_C);
   pragma Import (C, Free_Radar_Display_Product_Requirement, "pyramid_RadarDisplayProductRequirement_c_free");

   procedure To_C
     (In_Value  : Radar_Product_Requirement;
      Out_Value : out Pyramid_Radar_Product_Requirement_C);
   procedure From_C
     (In_Value  : Pyramid_Radar_Product_Requirement_C;
      Out_Value : out Radar_Product_Requirement);
   procedure Free_Radar_Product_Requirement (Value : access Pyramid_Radar_Product_Requirement_C);
   pragma Import (C, Free_Radar_Product_Requirement, "pyramid_RadarProductRequirement_c_free");

end Pyramid.Data_Model.Sensorproducts.Cabi;
