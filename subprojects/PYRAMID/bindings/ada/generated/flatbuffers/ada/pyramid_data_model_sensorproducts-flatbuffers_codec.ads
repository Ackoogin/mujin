--  Auto-generated FlatBuffers codec spec -- do not edit
--  Backend: flatbuffers | Package: Pyramid.Data_model.Sensorproducts.Flatbuffers_Codec
--
--  This package provides thin bindings to the C++ FlatBuffers codec.
--  Actual ser/de is performed via C interop (Import pragma).

with Interfaces.C; use Interfaces.C;
with System;

package Pyramid.Data_model.Sensorproducts.Flatbuffers_Codec is

   Content_Type : constant String := "application/flatbuffers";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "RadarDisplayProductRequirement_to_flatbuffer";

   function From_Binary_Radar_Display_Product_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "RadarDisplayProductRequirement_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "RadarProductRequirement_to_flatbuffer";

   function From_Binary_Radar_Product_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "RadarProductRequirement_from_flatbuffer";

end Pyramid.Data_model.Sensorproducts.Flatbuffers_Codec;
