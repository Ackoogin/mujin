--  Auto-generated service FlatBuffers codec

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Ada.Unchecked_Conversion;
with GNATCOLL.JSON;  use GNATCOLL.JSON;
with Interfaces.C;
with Interfaces.C.Strings;
with System;
with Pyramid.Data_Model.Common.Types_Codec;
with Pyramid.Data_Model.Tactical.Types_Codec;
with Pyramid.Services.Tactical_Objects.Json_Codec;

package body Pyramid.Services.Tactical_Objects.Flatbuffers_Codec is
   use type Interfaces.C.size_t;
   use type Interfaces.C.Strings.chars_ptr;
   use type System.Address;

   function To_Address is new
     Ada.Unchecked_Conversion (Interfaces.C.Strings.chars_ptr, System.Address);

   function Copy_From_Buffer
     (Data : System.Address; Size : Interfaces.C.size_t) return String
   is
      type Char_Array is array (1 .. Natural (Size)) of Character;
      pragma Pack (Char_Array);
   begin
      if Data = System.Null_Address or else Size = 0 then
         return "";
      end if;

      declare
         Chars : Char_Array;
         for Chars'Address use Data;
         pragma Import (Ada, Chars);
      begin
         return String (Chars);
      end;
   end Copy_From_Buffer;

   procedure Free_Buffer (Data : System.Address)
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_free_buffer";

   function Imported_To_Binary_Geodetic_Position
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_GeodeticPosition_to_flatbuffer_json";

   function Imported_From_Binary_Geodetic_Position
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_GeodeticPosition_from_flatbuffer_json";

   function Imported_To_Binary_Poly_Area
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_PolyArea_to_flatbuffer_json";

   function Imported_From_Binary_Poly_Area
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_PolyArea_from_flatbuffer_json";

   function Imported_To_Binary_Achievement
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_Achievement_to_flatbuffer_json";

   function Imported_From_Binary_Achievement
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_Achievement_from_flatbuffer_json";

   function Imported_To_Binary_Entity
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_Entity_to_flatbuffer_json";

   function Imported_From_Binary_Entity
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_Entity_from_flatbuffer_json";

   function Imported_To_Binary_Circle_Area
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_CircleArea_to_flatbuffer_json";

   function Imported_From_Binary_Circle_Area
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_CircleArea_from_flatbuffer_json";

   function Imported_To_Binary_Point
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_Point_to_flatbuffer_json";

   function Imported_From_Binary_Point
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_Point_from_flatbuffer_json";

   function Imported_To_Binary_Contraint
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_Contraint_to_flatbuffer_json";

   function Imported_From_Binary_Contraint
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_Contraint_from_flatbuffer_json";

   function Imported_To_Binary_Ack
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_Ack_to_flatbuffer_json";

   function Imported_From_Binary_Ack
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_Ack_from_flatbuffer_json";

   function Imported_To_Binary_Query
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_Query_to_flatbuffer_json";

   function Imported_From_Binary_Query
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_Query_from_flatbuffer_json";

   function Imported_To_Binary_Object_Detail
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectDetail_to_flatbuffer_json";

   function Imported_From_Binary_Object_Detail
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectDetail_from_flatbuffer_json";

   function Imported_To_Binary_Object_Evidence_Requirement
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectEvidenceRequirement_to_flatbuffer_json";

   function Imported_From_Binary_Object_Evidence_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectEvidenceRequirement_from_flatbuffer_json";

   function Imported_To_Binary_Object_Interest_Requirement
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectInterestRequirement_to_flatbuffer_json";

   function Imported_From_Binary_Object_Interest_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectInterestRequirement_from_flatbuffer_json";

   function Imported_To_Binary_Object_Match
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectMatch_to_flatbuffer_json";

   function Imported_From_Binary_Object_Match
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectMatch_from_flatbuffer_json";

   function Imported_To_Binary_Capability
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_Capability_to_flatbuffer_json";

   function Imported_From_Binary_Capability
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_Capability_from_flatbuffer_json";

   function Imported_To_Binary_Identifier
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_Identifier_to_flatbuffer_json";

   function Imported_From_Binary_Identifier
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_Identifier_from_flatbuffer_json";

   function Imported_To_Binary_Object_Detail_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectDetailArray_to_flatbuffer_json";

   function Imported_From_Binary_Object_Detail_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectDetailArray_from_flatbuffer_json";

   function Imported_To_Binary_Object_Evidence_Requirement_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectEvidenceRequirementArray_to_flatbuffer_json";

   function Imported_From_Binary_Object_Evidence_Requirement_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectEvidenceRequirementArray_from_flatbuffer_json";

   function Imported_To_Binary_Capability_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_CapabilityArray_to_flatbuffer_json";

   function Imported_From_Binary_Capability_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_CapabilityArray_from_flatbuffer_json";

   function Imported_To_Binary_Object_Match_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectMatchArray_to_flatbuffer_json";

   function Imported_From_Binary_Object_Match_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectMatchArray_from_flatbuffer_json";

   function Imported_To_Binary_Object_Interest_Requirement_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectInterestRequirementArray_to_flatbuffer_json";

   function Imported_From_Binary_Object_Interest_Requirement_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectInterestRequirementArray_from_flatbuffer_json";

   function Imported_To_Binary_Entity_Match
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_EntityMatch_to_flatbuffer_json";

   function Imported_From_Binary_Entity_Match
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_EntityMatch_from_flatbuffer_json";

   function Imported_To_Binary_Entity_Match_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_EntityMatchArray_to_flatbuffer_json";

   function Imported_From_Binary_Entity_Match_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_EntityMatchArray_from_flatbuffer_json";

   function Imported_To_Binary_Object_Evidence
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectEvidence_to_flatbuffer_json";

   function Imported_From_Binary_Object_Evidence
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_ObjectEvidence_from_flatbuffer_json";

   function Imported_To_Binary_Evidence_Requirement
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_EvidenceRequirement_to_flatbuffer_json";

   function Imported_From_Binary_Evidence_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_tactical_objects_EvidenceRequirement_from_flatbuffer_json";

   function To_Binary_Geodetic_Position (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Geodetic_Position (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Geodetic_Position";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Geodetic_Position;

   function From_Binary_Geodetic_Position (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Geodetic_Position
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Geodetic_Position";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Geodetic_Position;

   function To_Binary_Poly_Area (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Poly_Area (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Poly_Area";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Poly_Area;

   function From_Binary_Poly_Area (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Poly_Area
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Poly_Area";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Poly_Area;

   function To_Binary_Achievement (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Achievement (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Achievement";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Achievement;

   function From_Binary_Achievement (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Achievement
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Achievement";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Achievement;

   function To_Binary_Entity (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Entity (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Entity";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Entity;

   function From_Binary_Entity (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Entity
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Entity";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Entity;

   function To_Binary_Circle_Area (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Circle_Area (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Circle_Area";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Circle_Area;

   function From_Binary_Circle_Area (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Circle_Area
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Circle_Area";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Circle_Area;

   function To_Binary_Point (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Point (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Point";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Point;

   function From_Binary_Point (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Point
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Point";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Point;

   function To_Binary_Contraint (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Contraint (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Contraint";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Contraint;

   function From_Binary_Contraint (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Contraint
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Contraint";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Contraint;

   function To_Binary_Ack (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Ack (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Ack";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Ack;

   function From_Binary_Ack (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Ack
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Ack";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Ack;

   function To_Binary_Query (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Query (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Query";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Query;

   function From_Binary_Query (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Query
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Query";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Query;

   function To_Binary_Object_Detail (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Object_Detail (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Object_Detail";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Object_Detail;

   function From_Binary_Object_Detail (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Object_Detail
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Object_Detail";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Object_Detail;

   function To_Binary_Object_Evidence_Requirement (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Object_Evidence_Requirement (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Object_Evidence_Requirement";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Object_Evidence_Requirement;

   function From_Binary_Object_Evidence_Requirement (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Object_Evidence_Requirement
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Object_Evidence_Requirement";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Object_Evidence_Requirement;

   function To_Binary_Object_Interest_Requirement (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Object_Interest_Requirement (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Object_Interest_Requirement";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Object_Interest_Requirement;

   function From_Binary_Object_Interest_Requirement (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Object_Interest_Requirement
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Object_Interest_Requirement";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Object_Interest_Requirement;

   function To_Binary_Object_Match (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Object_Match (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Object_Match";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Object_Match;

   function From_Binary_Object_Match (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Object_Match
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Object_Match";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Object_Match;

   function To_Binary_Capability (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Capability (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Capability";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Capability;

   function From_Binary_Capability (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Capability
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Capability";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Capability;

   function To_Binary_Identifier (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Identifier (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Identifier";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Identifier;

   function From_Binary_Identifier (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Identifier
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Identifier";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Identifier;

   function To_Binary_Object_Detail_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Object_Detail_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Object_Detail_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Object_Detail_Array;

   function From_Binary_Object_Detail_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Object_Detail_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Object_Detail_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Object_Detail_Array;

   function To_Binary_Object_Evidence_Requirement_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Object_Evidence_Requirement_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Object_Evidence_Requirement_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Object_Evidence_Requirement_Array;

   function From_Binary_Object_Evidence_Requirement_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Object_Evidence_Requirement_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Object_Evidence_Requirement_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Object_Evidence_Requirement_Array;

   function To_Binary_Capability_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Capability_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Capability_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Capability_Array;

   function From_Binary_Capability_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Capability_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Capability_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Capability_Array;

   function To_Binary_Object_Match_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Object_Match_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Object_Match_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Object_Match_Array;

   function From_Binary_Object_Match_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Object_Match_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Object_Match_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Object_Match_Array;

   function To_Binary_Object_Interest_Requirement_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Object_Interest_Requirement_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Object_Interest_Requirement_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Object_Interest_Requirement_Array;

   function From_Binary_Object_Interest_Requirement_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Object_Interest_Requirement_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Object_Interest_Requirement_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Object_Interest_Requirement_Array;

   function To_Binary_Entity_Match (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Entity_Match (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Entity_Match";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Entity_Match;

   function From_Binary_Entity_Match (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Entity_Match
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Entity_Match";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Entity_Match;

   function To_Binary_Entity_Match_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Entity_Match_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Entity_Match_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Entity_Match_Array;

   function From_Binary_Entity_Match_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Entity_Match_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Entity_Match_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Entity_Match_Array;

   function To_Binary_Object_Evidence (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Object_Evidence (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Object_Evidence";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Object_Evidence;

   function From_Binary_Object_Evidence (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Object_Evidence
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Object_Evidence";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Object_Evidence;

   function To_Binary_Evidence_Requirement (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Evidence_Requirement (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Evidence_Requirement";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Evidence_Requirement;

   function From_Binary_Evidence_Requirement (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Evidence_Requirement
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Evidence_Requirement";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Evidence_Requirement;

   function To_Binary_Geodetic_Position (Msg : Pyramid.Data_Model.Common.Types.Geodetic_Position) return String is
   begin
      return To_Binary_Geodetic_Position (Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg));
   end To_Binary_Geodetic_Position;

   function From_Binary_Geodetic_Position
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Geodetic_Position) return Pyramid.Data_Model.Common.Types.Geodetic_Position
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Geodetic_Position (Payload);
      Result : Pyramid.Data_Model.Common.Types.Geodetic_Position;
   begin
      Result := Pyramid.Data_Model.Common.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Geodetic_Position;

   function To_Binary_Poly_Area (Msg : Pyramid.Data_Model.Common.Types.Poly_Area) return String is
   begin
      return To_Binary_Poly_Area (Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg));
   end To_Binary_Poly_Area;

   function From_Binary_Poly_Area
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Poly_Area) return Pyramid.Data_Model.Common.Types.Poly_Area
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Poly_Area (Payload);
      Result : Pyramid.Data_Model.Common.Types.Poly_Area;
   begin
      Result := Pyramid.Data_Model.Common.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Poly_Area;

   function To_Binary_Achievement (Msg : Pyramid.Data_Model.Common.Types.Achievement) return String is
   begin
      return To_Binary_Achievement (Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg));
   end To_Binary_Achievement;

   function From_Binary_Achievement
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Achievement) return Pyramid.Data_Model.Common.Types.Achievement
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Achievement (Payload);
      Result : Pyramid.Data_Model.Common.Types.Achievement;
   begin
      Result := Pyramid.Data_Model.Common.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Achievement;

   function To_Binary_Entity (Msg : Pyramid.Data_Model.Common.Types.Entity) return String is
   begin
      return To_Binary_Entity (Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg));
   end To_Binary_Entity;

   function From_Binary_Entity
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Entity) return Pyramid.Data_Model.Common.Types.Entity
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Entity (Payload);
      Result : Pyramid.Data_Model.Common.Types.Entity;
   begin
      Result := Pyramid.Data_Model.Common.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Entity;

   function To_Binary_Circle_Area (Msg : Pyramid.Data_Model.Common.Types.Circle_Area) return String is
   begin
      return To_Binary_Circle_Area (Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg));
   end To_Binary_Circle_Area;

   function From_Binary_Circle_Area
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Circle_Area) return Pyramid.Data_Model.Common.Types.Circle_Area
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Circle_Area (Payload);
      Result : Pyramid.Data_Model.Common.Types.Circle_Area;
   begin
      Result := Pyramid.Data_Model.Common.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Circle_Area;

   function To_Binary_Point (Msg : Pyramid.Data_Model.Common.Types.Point) return String is
   begin
      return To_Binary_Point (Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg));
   end To_Binary_Point;

   function From_Binary_Point
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Point) return Pyramid.Data_Model.Common.Types.Point
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Point (Payload);
      Result : Pyramid.Data_Model.Common.Types.Point;
   begin
      Result := Pyramid.Data_Model.Common.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Point;

   function To_Binary_Contraint (Msg : Pyramid.Data_Model.Common.Types.Contraint) return String is
   begin
      return To_Binary_Contraint (Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg));
   end To_Binary_Contraint;

   function From_Binary_Contraint
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Contraint) return Pyramid.Data_Model.Common.Types.Contraint
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Contraint (Payload);
      Result : Pyramid.Data_Model.Common.Types.Contraint;
   begin
      Result := Pyramid.Data_Model.Common.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Contraint;

   function To_Binary_Ack (Msg : Pyramid.Data_Model.Common.Types.Ack) return String is
   begin
      return To_Binary_Ack (Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg));
   end To_Binary_Ack;

   function From_Binary_Ack
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Ack) return Pyramid.Data_Model.Common.Types.Ack
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Ack (Payload);
      Result : Pyramid.Data_Model.Common.Types.Ack;
   begin
      Result := Pyramid.Data_Model.Common.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Ack;

   function To_Binary_Query (Msg : Pyramid.Data_Model.Common.Types.Query) return String is
   begin
      return To_Binary_Query (Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg));
   end To_Binary_Query;

   function From_Binary_Query
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Query) return Pyramid.Data_Model.Common.Types.Query
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Query (Payload);
      Result : Pyramid.Data_Model.Common.Types.Query;
   begin
      Result := Pyramid.Data_Model.Common.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Query;

   function To_Binary_Object_Detail (Msg : Pyramid.Data_Model.Tactical.Types.Object_Detail) return String is
   begin
      return To_Binary_Object_Detail (Pyramid.Data_Model.Tactical.Types_Codec.To_Json (Msg));
   end To_Binary_Object_Detail;

   function From_Binary_Object_Detail
     (Payload : String; Tag : access Pyramid.Data_Model.Tactical.Types.Object_Detail) return Pyramid.Data_Model.Tactical.Types.Object_Detail
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Object_Detail (Payload);
      Result : Pyramid.Data_Model.Tactical.Types.Object_Detail;
   begin
      Result := Pyramid.Data_Model.Tactical.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Object_Detail;

   function To_Binary_Object_Evidence_Requirement (Msg : Pyramid.Data_Model.Tactical.Types.Object_Evidence_Requirement) return String is
   begin
      return To_Binary_Object_Evidence_Requirement (Pyramid.Data_Model.Tactical.Types_Codec.To_Json (Msg));
   end To_Binary_Object_Evidence_Requirement;

   function From_Binary_Object_Evidence_Requirement
     (Payload : String; Tag : access Pyramid.Data_Model.Tactical.Types.Object_Evidence_Requirement) return Pyramid.Data_Model.Tactical.Types.Object_Evidence_Requirement
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Object_Evidence_Requirement (Payload);
      Result : Pyramid.Data_Model.Tactical.Types.Object_Evidence_Requirement;
   begin
      Result := Pyramid.Data_Model.Tactical.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Object_Evidence_Requirement;

   function To_Binary_Object_Interest_Requirement (Msg : Pyramid.Data_Model.Tactical.Types.Object_Interest_Requirement) return String is
   begin
      return To_Binary_Object_Interest_Requirement (Pyramid.Data_Model.Tactical.Types_Codec.To_Json (Msg));
   end To_Binary_Object_Interest_Requirement;

   function From_Binary_Object_Interest_Requirement
     (Payload : String; Tag : access Pyramid.Data_Model.Tactical.Types.Object_Interest_Requirement) return Pyramid.Data_Model.Tactical.Types.Object_Interest_Requirement
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Object_Interest_Requirement (Payload);
      Result : Pyramid.Data_Model.Tactical.Types.Object_Interest_Requirement;
   begin
      Result := Pyramid.Data_Model.Tactical.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Object_Interest_Requirement;

   function To_Binary_Object_Match (Msg : Pyramid.Data_Model.Tactical.Types.Object_Match) return String is
   begin
      return To_Binary_Object_Match (Pyramid.Data_Model.Tactical.Types_Codec.To_Json (Msg));
   end To_Binary_Object_Match;

   function From_Binary_Object_Match
     (Payload : String; Tag : access Pyramid.Data_Model.Tactical.Types.Object_Match) return Pyramid.Data_Model.Tactical.Types.Object_Match
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Object_Match (Payload);
      Result : Pyramid.Data_Model.Tactical.Types.Object_Match;
   begin
      Result := Pyramid.Data_Model.Tactical.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Object_Match;

   function To_Binary_Capability (Msg : Pyramid.Data_Model.Common.Types.Capability) return String is
   begin
      return To_Binary_Capability (Pyramid.Data_Model.Common.Types_Codec.To_Json (Msg));
   end To_Binary_Capability;

   function From_Binary_Capability
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Capability) return Pyramid.Data_Model.Common.Types.Capability
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Capability (Payload);
      Result : Pyramid.Data_Model.Common.Types.Capability;
   begin
      Result := Pyramid.Data_Model.Common.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Capability;

   function To_Binary_Identifier (Msg : Pyramid.Data_Model.Base.Types.Identifier) return String is
   begin
      return To_Binary_Identifier (Ada.Strings.Unbounded.To_String (Msg));
   end To_Binary_Identifier;

   function From_Binary_Identifier
     (Payload : String; Tag : access Pyramid.Data_Model.Base.Types.Identifier) return Pyramid.Data_Model.Base.Types.Identifier
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Identifier (Payload);
      Result : Pyramid.Data_Model.Base.Types.Identifier;
   begin
      declare
         Value : constant JSON_Value := Read (Json);
         Text  : constant String := String'(UTF8_String'(Get (Value)));
      begin
         Result := To_Unbounded_String (Text);
      end;
      return Result;
   end From_Binary_Identifier;

   function To_Binary_Entity_Match (Msg : Pyramid.Services.Tactical_Objects.Wire_Types.Entity_Match) return String is
   begin
      return To_Binary_Entity_Match (Pyramid.Services.Tactical_Objects.Json_Codec.To_Json (Msg));
   end To_Binary_Entity_Match;

   function From_Binary_Entity_Match
     (Payload : String; Tag : access Pyramid.Services.Tactical_Objects.Wire_Types.Entity_Match) return Pyramid.Services.Tactical_Objects.Wire_Types.Entity_Match
   is
      pragma Unreferenced (Tag);
      Json : constant String := From_Binary_Entity_Match (Payload);
   begin
      return Pyramid.Services.Tactical_Objects.Json_Codec.From_Json (Json);
   end From_Binary_Entity_Match;

   function To_Binary_Object_Evidence (Msg : Pyramid.Services.Tactical_Objects.Wire_Types.Object_Evidence) return String is
   begin
      return To_Binary_Object_Evidence (Pyramid.Services.Tactical_Objects.Json_Codec.To_Json (Msg));
   end To_Binary_Object_Evidence;

   function From_Binary_Object_Evidence
     (Payload : String; Tag : access Pyramid.Services.Tactical_Objects.Wire_Types.Object_Evidence) return Pyramid.Services.Tactical_Objects.Wire_Types.Object_Evidence
   is
      pragma Unreferenced (Tag);
      Json : constant String := From_Binary_Object_Evidence (Payload);
   begin
      return Pyramid.Services.Tactical_Objects.Json_Codec.From_Json (Json);
   end From_Binary_Object_Evidence;

   function To_Binary_Evidence_Requirement (Msg : Pyramid.Services.Tactical_Objects.Wire_Types.Evidence_Requirement) return String is
   begin
      return To_Binary_Evidence_Requirement (Pyramid.Services.Tactical_Objects.Json_Codec.To_Json (Msg));
   end To_Binary_Evidence_Requirement;

   function From_Binary_Evidence_Requirement
     (Payload : String; Tag : access Pyramid.Services.Tactical_Objects.Wire_Types.Evidence_Requirement) return Pyramid.Services.Tactical_Objects.Wire_Types.Evidence_Requirement
   is
      pragma Unreferenced (Tag);
      Json : constant String := From_Binary_Evidence_Requirement (Payload);
   begin
      return Pyramid.Services.Tactical_Objects.Json_Codec.From_Json (Json);
   end From_Binary_Evidence_Requirement;

end Pyramid.Services.Tactical_Objects.Flatbuffers_Codec;
