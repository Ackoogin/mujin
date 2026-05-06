--  Auto-generated FlatBuffers codec spec -- do not edit
--  Backend: flatbuffers | Package: Pyramid.Data_model.Sensors.Flatbuffers_Codec
--
--  This package provides thin bindings to the C++ FlatBuffers codec.
--  Actual ser/de is performed via C interop (Import pragma).

with Interfaces.C; use Interfaces.C;
with System;

package Pyramid.Data_model.Sensors.Flatbuffers_Codec is

   Content_Type : constant String := "application/flatbuffers";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "InterpretationRequirement_to_flatbuffer";

   function From_Binary_Interpretation_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "InterpretationRequirement_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ManualTrackRequirement_to_flatbuffer";

   function From_Binary_Manual_Track_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ManualTrackRequirement_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ATIRequirement_to_flatbuffer";

   function From_Binary_ATI_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ATIRequirement_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "TrackProvisionRequirement_to_flatbuffer";

   function From_Binary_Track_Provision_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "TrackProvisionRequirement_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ObjectEvidenceProvisionRequirement_to_flatbuffer";

   function From_Binary_Object_Evidence_Provision_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ObjectEvidenceProvisionRequirement_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ObjectAquisitionRequirement_to_flatbuffer";

   function From_Binary_Object_Aquisition_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ObjectAquisitionRequirement_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "SensorObject_to_flatbuffer";

   function From_Binary_Sensor_Object
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "SensorObject_from_flatbuffer";

   function To_Binary (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "RadarModeChangeRequirement_to_flatbuffer";

   function From_Binary_Radar_Mode_Change_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "RadarModeChangeRequirement_from_flatbuffer";

end Pyramid.Data_model.Sensors.Flatbuffers_Codec;
