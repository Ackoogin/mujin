--  Auto-generated FlatBuffers codec spec — do not edit
--  Backend: flatbuffers | Package: Pyramid.Components.Tactical_objects.Flatbuffers_Codec
--
--  This package provides thin bindings to the C++ FlatBuffers codec.
--  Actual ser/de is performed via C interop (Import pragma).

with Interfaces.C; use Interfaces.C;
with System;

package Pyramid.Components.Tactical_objects.Flatbuffers_Codec is

   Content_Type : constant String := "application/flatbuffers";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Position_to_flatbuffer";

   function From_Binary_Position
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Position_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Velocity_to_flatbuffer";

   function From_Binary_Velocity
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Velocity_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "BoundingBox_to_flatbuffer";

   function From_Binary_Bounding_Box
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "BoundingBox_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "SourceRef_to_flatbuffer";

   function From_Binary_Source_Ref
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "SourceRef_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "MilClassProfile_to_flatbuffer";

   function From_Binary_Mil_Class_Profile
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "MilClassProfile_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ZoneGeometry_to_flatbuffer";

   function From_Binary_Zone_Geometry
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ZoneGeometry_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "TacticalObject_to_flatbuffer";

   function From_Binary_Tactical_Object
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "TacticalObject_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "TacticalObjectQuery_to_flatbuffer";

   function From_Binary_Tactical_Object_Query
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "TacticalObjectQuery_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Zone_to_flatbuffer";

   function From_Binary_Zone
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Zone_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Observation_to_flatbuffer";

   function From_Binary_Observation
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Observation_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "EntityUpdateFrame_to_flatbuffer";

   function From_Binary_Entity_Update_Frame
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "EntityUpdateFrame_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "InterestCriteria_to_flatbuffer";

   function From_Binary_Interest_Criteria
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "InterestCriteria_from_flatbuffer";

end Pyramid.Components.Tactical_objects.Flatbuffers_Codec;
