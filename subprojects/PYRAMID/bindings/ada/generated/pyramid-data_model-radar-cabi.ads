--  Auto-generated Ada C-ABI mirror specification
--  Generated from: pyramid.data_model.radar.proto by generate_bindings.py (ada cabi)
--  Package: Pyramid.Data_Model.Radar.Cabi

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Interfaces.C;
with Interfaces.C.Strings;
with System;
with Pyramid.Data_Model.Radar.Types;  use Pyramid.Data_Model.Radar.Types;

package Pyramid.Data_Model.Radar.Cabi is

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

end Pyramid.Data_Model.Radar.Cabi;
