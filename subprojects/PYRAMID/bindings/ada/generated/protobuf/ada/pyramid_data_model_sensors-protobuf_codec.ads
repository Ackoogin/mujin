--  Auto-generated Protobuf codec spec -- do not edit
--  Backend: protobuf | Package: Pyramid.Data_model.Sensors.Protobuf_Codec
--
--  Thin Ada binding to protobuf C++ codec via C interop.
--  Protobuf has no native Ada support; serialisation is
--  delegated to the C++ implementation via pragma Import.

with Interfaces.C; use Interfaces.C;
with Interfaces.C.Strings;
with System;

package Pyramid.Data_model.Sensors.Protobuf_Codec is

   Content_Type : constant String := "application/protobuf";

   --  InterpretationRequirement: protobuf SerializeToString / ParseFromArray
   function To_Binary_Interpretation_Requirement (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "InterpretationRequirement_to_protobuf";

   function From_Binary_Interpretation_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "InterpretationRequirement_from_protobuf";

   --  ManualTrackRequirement: protobuf SerializeToString / ParseFromArray
   function To_Binary_Manual_Track_Requirement (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ManualTrackRequirement_to_protobuf";

   function From_Binary_Manual_Track_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ManualTrackRequirement_from_protobuf";

   --  ATIRequirement: protobuf SerializeToString / ParseFromArray
   function To_Binary_ATI_Requirement (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ATIRequirement_to_protobuf";

   function From_Binary_ATI_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ATIRequirement_from_protobuf";

   --  TrackProvisionRequirement: protobuf SerializeToString / ParseFromArray
   function To_Binary_Track_Provision_Requirement (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "TrackProvisionRequirement_to_protobuf";

   function From_Binary_Track_Provision_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "TrackProvisionRequirement_from_protobuf";

   --  ObjectEvidenceProvisionRequirement: protobuf SerializeToString / ParseFromArray
   function To_Binary_Object_Evidence_Provision_Requirement (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ObjectEvidenceProvisionRequirement_to_protobuf";

   function From_Binary_Object_Evidence_Provision_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ObjectEvidenceProvisionRequirement_from_protobuf";

   --  ObjectAquisitionRequirement: protobuf SerializeToString / ParseFromArray
   function To_Binary_Object_Aquisition_Requirement (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ObjectAquisitionRequirement_to_protobuf";

   function From_Binary_Object_Aquisition_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ObjectAquisitionRequirement_from_protobuf";

   --  SensorObject: protobuf SerializeToString / ParseFromArray
   function To_Binary_Sensor_Object (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "SensorObject_to_protobuf";

   function From_Binary_Sensor_Object
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "SensorObject_from_protobuf";

   --  RadarModeChangeRequirement: protobuf SerializeToString / ParseFromArray
   function To_Binary_Radar_Mode_Change_Requirement (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "RadarModeChangeRequirement_to_protobuf";

   function From_Binary_Radar_Mode_Change_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "RadarModeChangeRequirement_from_protobuf";

end Pyramid.Data_model.Sensors.Protobuf_Codec;
