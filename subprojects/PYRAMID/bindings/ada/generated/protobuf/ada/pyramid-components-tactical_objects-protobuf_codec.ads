--  Auto-generated Protobuf codec spec — do not edit
--  Backend: protobuf | Package: Pyramid.Components.Tactical_objects.Protobuf_Codec
--
--  Thin Ada binding to protobuf C++ codec via C interop.
--  Protobuf has no native Ada support; serialisation is
--  delegated to the C++ implementation via pragma Import.

with Interfaces.C; use Interfaces.C;
with Interfaces.C.Strings;
with System;

package Pyramid.Components.Tactical_objects.Protobuf_Codec is

   Content_Type : constant String := "application/protobuf";

   --  Position: protobuf SerializeToString / ParseFromArray
   function To_Binary_Position (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Position_to_protobuf";

   function From_Binary_Position
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Position_from_protobuf";

   --  Velocity: protobuf SerializeToString / ParseFromArray
   function To_Binary_Velocity (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Velocity_to_protobuf";

   function From_Binary_Velocity
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Velocity_from_protobuf";

   --  BoundingBox: protobuf SerializeToString / ParseFromArray
   function To_Binary_Bounding_Box (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "BoundingBox_to_protobuf";

   function From_Binary_Bounding_Box
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "BoundingBox_from_protobuf";

   --  SourceRef: protobuf SerializeToString / ParseFromArray
   function To_Binary_Source_Ref (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "SourceRef_to_protobuf";

   function From_Binary_Source_Ref
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "SourceRef_from_protobuf";

   --  MilClassProfile: protobuf SerializeToString / ParseFromArray
   function To_Binary_Mil_Class_Profile (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "MilClassProfile_to_protobuf";

   function From_Binary_Mil_Class_Profile
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "MilClassProfile_from_protobuf";

   --  ZoneGeometry: protobuf SerializeToString / ParseFromArray
   function To_Binary_Zone_Geometry (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ZoneGeometry_to_protobuf";

   function From_Binary_Zone_Geometry
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ZoneGeometry_from_protobuf";

   --  TacticalObject: protobuf SerializeToString / ParseFromArray
   function To_Binary_Tactical_Object (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "TacticalObject_to_protobuf";

   function From_Binary_Tactical_Object
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "TacticalObject_from_protobuf";

   --  TacticalObjectQuery: protobuf SerializeToString / ParseFromArray
   function To_Binary_Tactical_Object_Query (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "TacticalObjectQuery_to_protobuf";

   function From_Binary_Tactical_Object_Query
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "TacticalObjectQuery_from_protobuf";

   --  Zone: protobuf SerializeToString / ParseFromArray
   function To_Binary_Zone (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Zone_to_protobuf";

   function From_Binary_Zone
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Zone_from_protobuf";

   --  Observation: protobuf SerializeToString / ParseFromArray
   function To_Binary_Observation (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "Observation_to_protobuf";

   function From_Binary_Observation
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "Observation_from_protobuf";

   --  EntityUpdateFrame: protobuf SerializeToString / ParseFromArray
   function To_Binary_Entity_Update_Frame (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "EntityUpdateFrame_to_protobuf";

   function From_Binary_Entity_Update_Frame
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "EntityUpdateFrame_from_protobuf";

   --  InterestCriteria: protobuf SerializeToString / ParseFromArray
   function To_Binary_Interest_Criteria (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "InterestCriteria_to_protobuf";

   function From_Binary_Interest_Criteria
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "InterestCriteria_from_protobuf";

end Pyramid.Components.Tactical_objects.Protobuf_Codec;
