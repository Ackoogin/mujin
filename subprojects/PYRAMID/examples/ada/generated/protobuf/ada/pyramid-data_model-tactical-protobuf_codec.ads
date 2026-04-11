--  Auto-generated Protobuf codec spec — do not edit
--  Backend: protobuf | Package: Pyramid.Data_model.Tactical.Protobuf_Codec
--
--  Thin Ada binding to protobuf C++ codec via C interop.
--  Protobuf has no native Ada support; serialisation is
--  delegated to the C++ implementation via pragma Import.

with Interfaces.C; use Interfaces.C;
with Interfaces.C.Strings;
with System;

package Pyramid.Data_model.Tactical.Protobuf_Codec is

   Content_Type : constant String := "application/protobuf";

   --  ObjectDetail: protobuf SerializeToString / ParseFromArray
   function To_Binary_Object_Detail (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ObjectDetail_to_protobuf";

   function From_Binary_Object_Detail
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ObjectDetail_from_protobuf";

   --  ObjectEvidenceRequirement: protobuf SerializeToString / ParseFromArray
   function To_Binary_Object_Evidence_Requirement (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ObjectEvidenceRequirement_to_protobuf";

   function From_Binary_Object_Evidence_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ObjectEvidenceRequirement_from_protobuf";

   --  ObjectInterestRequirement: protobuf SerializeToString / ParseFromArray
   function To_Binary_Object_Interest_Requirement (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ObjectInterestRequirement_to_protobuf";

   function From_Binary_Object_Interest_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ObjectInterestRequirement_from_protobuf";

   --  ObjectMatch: protobuf SerializeToString / ParseFromArray
   function To_Binary_Object_Match (Msg : System.Address)
     return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "ObjectMatch_to_protobuf";

   function From_Binary_Object_Match
     (Data : System.Address; Size : Interfaces.C.size_t)
     return System.Address
     with Import, Convention => C,
          External_Name => "ObjectMatch_from_protobuf";

end Pyramid.Data_model.Tactical.Protobuf_Codec;
