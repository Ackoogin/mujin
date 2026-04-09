--  Auto-generated FlatBuffers codec spec — do not edit
--  Backend: flatbuffers | Package: Pyramid.Data_model.Base.Flatbuffers_Codec
--
--  This package provides thin bindings to the C++ FlatBuffers codec.
--  Actual ser/de is performed via C interop (Import pragma).

with Interfaces.C; use Interfaces.C;
with System;

package Pyramid.Data_model.Base.Flatbuffers_Codec is

   Content_Type : constant String := "application/flatbuffers";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Angle_to_flatbuffer";

   function From_Binary_Angle
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Angle_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Length_to_flatbuffer";

   function From_Binary_Length
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Length_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Timestamp_to_flatbuffer";

   function From_Binary_Timestamp
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Timestamp_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Identifier_to_flatbuffer";

   function From_Binary_Identifier
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Identifier_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Speed_to_flatbuffer";

   function From_Binary_Speed
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Speed_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Percentage_to_flatbuffer";

   function From_Binary_Percentage
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Percentage_from_flatbuffer";

end Pyramid.Data_model.Base.Flatbuffers_Codec;
