--  Auto-generated Protobuf codec spec -- do not edit
--  Backend: protobuf | Package: Pyramid.Data_model.Sensorproducts.Protobuf_Codec
--
--  Thin Ada binding to protobuf C++ codec via C interop.
--  Protobuf has no native Ada support; serialisation is
--  delegated to the C++ implementation via pragma Import.

with Interfaces.C; use Interfaces.C;
with Interfaces.C.Strings;
with System;

package Pyramid.Data_model.Sensorproducts.Protobuf_Codec is

   Content_Type : constant String := "application/protobuf";

   --  RadarDisplayProductRequirement: protobuf SerializeToString / ParseFromArray
   function To_Binary_Radar_Display_Product_Requirement (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "RadarDisplayProductRequirement_to_protobuf";

   function From_Binary_Radar_Display_Product_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "RadarDisplayProductRequirement_from_protobuf";

   --  RadarProductRequirement: protobuf SerializeToString / ParseFromArray
   function To_Binary_Radar_Product_Requirement (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "RadarProductRequirement_to_protobuf";

   function From_Binary_Radar_Product_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "RadarProductRequirement_from_protobuf";

end Pyramid.Data_model.Sensorproducts.Protobuf_Codec;
