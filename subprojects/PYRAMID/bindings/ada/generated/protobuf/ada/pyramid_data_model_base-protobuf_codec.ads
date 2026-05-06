--  Auto-generated Protobuf codec spec -- do not edit
--  Backend: protobuf | Package: Pyramid.Data_model.Base.Protobuf_Codec
--
--  Thin Ada binding to protobuf C++ codec via C interop.
--  Protobuf has no native Ada support; serialisation is
--  delegated to the C++ implementation via pragma Import.

with Interfaces.C; use Interfaces.C;
with Interfaces.C.Strings;
with System;

package Pyramid.Data_model.Base.Protobuf_Codec is

   Content_Type : constant String := "application/protobuf";

   --  Angle: protobuf SerializeToString / ParseFromArray
   function To_Binary_Angle (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Angle_to_protobuf";

   function From_Binary_Angle
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Angle_from_protobuf";

   --  Length: protobuf SerializeToString / ParseFromArray
   function To_Binary_Length (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Length_to_protobuf";

   function From_Binary_Length
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Length_from_protobuf";

   --  Timestamp: protobuf SerializeToString / ParseFromArray
   function To_Binary_Timestamp (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Timestamp_to_protobuf";

   function From_Binary_Timestamp
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Timestamp_from_protobuf";

   --  Identifier: protobuf SerializeToString / ParseFromArray
   function To_Binary_Identifier (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Identifier_to_protobuf";

   function From_Binary_Identifier
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Identifier_from_protobuf";

   --  Speed: protobuf SerializeToString / ParseFromArray
   function To_Binary_Speed (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Speed_to_protobuf";

   function From_Binary_Speed
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Speed_from_protobuf";

   --  Percentage: protobuf SerializeToString / ParseFromArray
   function To_Binary_Percentage (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Percentage_to_protobuf";

   function From_Binary_Percentage
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Percentage_from_protobuf";

end Pyramid.Data_model.Base.Protobuf_Codec;
