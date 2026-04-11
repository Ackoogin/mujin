--  Auto-generated Protobuf codec spec — do not edit
--  Backend: protobuf | Package: Pyramid.Data_model.Common.Protobuf_Codec
--
--  Thin Ada binding to protobuf C++ codec via C interop.
--  Protobuf has no native Ada support; serialisation is
--  delegated to the C++ implementation via pragma Import.

with Interfaces.C; use Interfaces.C;
with Interfaces.C.Strings;
with System;

package Pyramid.Data_model.Common.Protobuf_Codec is

   Content_Type : constant String := "application/protobuf";

   --  GeodeticPosition: protobuf SerializeToString / ParseFromArray
   function To_Binary_Geodetic_Position (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "GeodeticPosition_to_protobuf";

   function From_Binary_Geodetic_Position
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "GeodeticPosition_from_protobuf";

   --  PolyArea: protobuf SerializeToString / ParseFromArray
   function To_Binary_Poly_Area (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "PolyArea_to_protobuf";

   function From_Binary_Poly_Area
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "PolyArea_from_protobuf";

   --  Achievement: protobuf SerializeToString / ParseFromArray
   function To_Binary_Achievement (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Achievement_to_protobuf";

   function From_Binary_Achievement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Achievement_from_protobuf";

   --  Requirement: protobuf SerializeToString / ParseFromArray
   function To_Binary_Requirement (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Requirement_to_protobuf";

   function From_Binary_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Requirement_from_protobuf";

   --  Capability: protobuf SerializeToString / ParseFromArray
   function To_Binary_Capability (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Capability_to_protobuf";

   function From_Binary_Capability
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Capability_from_protobuf";

   --  Entity: protobuf SerializeToString / ParseFromArray
   function To_Binary_Entity (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Entity_to_protobuf";

   function From_Binary_Entity
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Entity_from_protobuf";

   --  CircleArea: protobuf SerializeToString / ParseFromArray
   function To_Binary_Circle_Area (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "CircleArea_to_protobuf";

   function From_Binary_Circle_Area
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "CircleArea_from_protobuf";

   --  Point: protobuf SerializeToString / ParseFromArray
   function To_Binary_Point (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Point_to_protobuf";

   function From_Binary_Point
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Point_from_protobuf";

   --  Contraint: protobuf SerializeToString / ParseFromArray
   function To_Binary_Contraint (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Contraint_to_protobuf";

   function From_Binary_Contraint
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Contraint_from_protobuf";

   --  Ack: protobuf SerializeToString / ParseFromArray
   function To_Binary_Ack (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Ack_to_protobuf";

   function From_Binary_Ack
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Ack_from_protobuf";

   --  Query: protobuf SerializeToString / ParseFromArray
   function To_Binary_Query (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Query_to_protobuf";

   function From_Binary_Query
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Query_from_protobuf";

end Pyramid.Data_model.Common.Protobuf_Codec;
