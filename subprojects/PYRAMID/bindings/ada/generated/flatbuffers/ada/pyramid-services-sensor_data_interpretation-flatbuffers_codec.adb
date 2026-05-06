--  Auto-generated service FlatBuffers codec

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Ada.Unchecked_Conversion;
with GNATCOLL.JSON;  use GNATCOLL.JSON;
with Interfaces.C;
with Interfaces.C.Strings;
with System;
with Pyramid.Data_Model.Common.Types_Codec;
with Pyramid.Data_Model.Sensors.Types_Codec;

package body Pyramid.Services.Sensor_Data_Interpretation.Flatbuffers_Codec is
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
          External_Name => "pyramid_services_sensor_data_interpretation_free_buffer";

   function Imported_To_Binary_Geodetic_Position
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_GeodeticPosition_to_flatbuffer_json";

   function Imported_From_Binary_Geodetic_Position
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_GeodeticPosition_from_flatbuffer_json";

   function Imported_To_Binary_Poly_Area
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_PolyArea_to_flatbuffer_json";

   function Imported_From_Binary_Poly_Area
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_PolyArea_from_flatbuffer_json";

   function Imported_To_Binary_Achievement
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_Achievement_to_flatbuffer_json";

   function Imported_From_Binary_Achievement
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_Achievement_from_flatbuffer_json";

   function Imported_To_Binary_Entity
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_Entity_to_flatbuffer_json";

   function Imported_From_Binary_Entity
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_Entity_from_flatbuffer_json";

   function Imported_To_Binary_Circle_Area
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_CircleArea_to_flatbuffer_json";

   function Imported_From_Binary_Circle_Area
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_CircleArea_from_flatbuffer_json";

   function Imported_To_Binary_Point
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_Point_to_flatbuffer_json";

   function Imported_From_Binary_Point
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_Point_from_flatbuffer_json";

   function Imported_To_Binary_Contraint
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_Contraint_to_flatbuffer_json";

   function Imported_From_Binary_Contraint
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_Contraint_from_flatbuffer_json";

   function Imported_To_Binary_Ack
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_Ack_to_flatbuffer_json";

   function Imported_From_Binary_Ack
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_Ack_from_flatbuffer_json";

   function Imported_To_Binary_Query
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_Query_to_flatbuffer_json";

   function Imported_From_Binary_Query
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_Query_from_flatbuffer_json";

   function Imported_To_Binary_Interpretation_Requirement
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_InterpretationRequirement_to_flatbuffer_json";

   function Imported_From_Binary_Interpretation_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_InterpretationRequirement_from_flatbuffer_json";

   function Imported_To_Binary_Object_Evidence_Provision_Requirement
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_ObjectEvidenceProvisionRequirement_to_flatbuffer_json";

   function Imported_From_Binary_Object_Evidence_Provision_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_ObjectEvidenceProvisionRequirement_from_flatbuffer_json";

   function Imported_To_Binary_Object_Aquisition_Requirement
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_ObjectAquisitionRequirement_to_flatbuffer_json";

   function Imported_From_Binary_Object_Aquisition_Requirement
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_ObjectAquisitionRequirement_from_flatbuffer_json";

   function Imported_To_Binary_Capability
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_Capability_to_flatbuffer_json";

   function Imported_From_Binary_Capability
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_Capability_from_flatbuffer_json";

   function Imported_To_Binary_Identifier
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_Identifier_to_flatbuffer_json";

   function Imported_From_Binary_Identifier
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_Identifier_from_flatbuffer_json";

   function Imported_To_Binary_Object_Evidence_Provision_Requirement_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_ObjectEvidenceProvisionRequirementArray_to_flatbuffer_json";

   function Imported_From_Binary_Object_Evidence_Provision_Requirement_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_ObjectEvidenceProvisionRequirementArray_from_flatbuffer_json";

   function Imported_To_Binary_Object_Aquisition_Requirement_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_ObjectAquisitionRequirementArray_to_flatbuffer_json";

   function Imported_From_Binary_Object_Aquisition_Requirement_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_ObjectAquisitionRequirementArray_from_flatbuffer_json";

   function Imported_To_Binary_Capability_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_CapabilityArray_to_flatbuffer_json";

   function Imported_From_Binary_Capability_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_CapabilityArray_from_flatbuffer_json";

   function Imported_To_Binary_Interpretation_Requirement_Array
     (Json     : Interfaces.C.Strings.chars_ptr;
      Size_Out : access Interfaces.C.size_t) return System.Address
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_InterpretationRequirementArray_to_flatbuffer_json";

   function Imported_From_Binary_Interpretation_Requirement_Array
     (Data : System.Address; Size : Interfaces.C.size_t)
      return Interfaces.C.Strings.chars_ptr
     with Import, Convention => C,
          External_Name => "pyramid_services_sensor_data_interpretation_InterpretationRequirementArray_from_flatbuffer_json";

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

   function To_Binary_Interpretation_Requirement (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Interpretation_Requirement (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Interpretation_Requirement";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Interpretation_Requirement;

   function From_Binary_Interpretation_Requirement (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Interpretation_Requirement
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Interpretation_Requirement";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Interpretation_Requirement;

   function To_Binary_Object_Evidence_Provision_Requirement (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Object_Evidence_Provision_Requirement (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Object_Evidence_Provision_Requirement";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Object_Evidence_Provision_Requirement;

   function From_Binary_Object_Evidence_Provision_Requirement (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Object_Evidence_Provision_Requirement
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Object_Evidence_Provision_Requirement";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Object_Evidence_Provision_Requirement;

   function To_Binary_Object_Aquisition_Requirement (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Object_Aquisition_Requirement (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Object_Aquisition_Requirement";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Object_Aquisition_Requirement;

   function From_Binary_Object_Aquisition_Requirement (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Object_Aquisition_Requirement
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Object_Aquisition_Requirement";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Object_Aquisition_Requirement;

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

   function To_Binary_Object_Evidence_Provision_Requirement_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Object_Evidence_Provision_Requirement_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Object_Evidence_Provision_Requirement_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Object_Evidence_Provision_Requirement_Array;

   function From_Binary_Object_Evidence_Provision_Requirement_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Object_Evidence_Provision_Requirement_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Object_Evidence_Provision_Requirement_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Object_Evidence_Provision_Requirement_Array;

   function To_Binary_Object_Aquisition_Requirement_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Object_Aquisition_Requirement_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Object_Aquisition_Requirement_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Object_Aquisition_Requirement_Array;

   function From_Binary_Object_Aquisition_Requirement_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Object_Aquisition_Requirement_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Object_Aquisition_Requirement_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Object_Aquisition_Requirement_Array;

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

   function To_Binary_Interpretation_Requirement_Array (Json : String) return String is
      Json_C   : Interfaces.C.Strings.chars_ptr := Interfaces.C.Strings.New_String (Json);
      Size_Out : aliased Interfaces.C.size_t := 0;
      Data     : System.Address := Imported_To_Binary_Interpretation_Requirement_Array (Json_C, Size_Out'Access);
   begin
      Interfaces.C.Strings.Free (Json_C);
      if Data = System.Null_Address then
         raise Constraint_Error with "FlatBuffers encode failed for Interpretation_Requirement_Array";
      end if;

      declare
         Result : constant String := Copy_From_Buffer (Data, Size_Out);
      begin
         Free_Buffer (Data);
         return Result;
      end;
   end To_Binary_Interpretation_Requirement_Array;

   function From_Binary_Interpretation_Requirement_Array (Payload : String) return String is
      Payload_Bytes : aliased constant String := Payload;
      Json_C : Interfaces.C.Strings.chars_ptr := Imported_From_Binary_Interpretation_Requirement_Array
        ((if Payload_Bytes'Length = 0
          then System.Null_Address
          else Payload_Bytes (Payload_Bytes'First)'Address),
         Interfaces.C.size_t (Payload_Bytes'Length));
   begin
      if Json_C = Interfaces.C.Strings.Null_Ptr then
         raise Constraint_Error with "FlatBuffers decode failed for Interpretation_Requirement_Array";
      end if;

      declare
         Result : constant String := Interfaces.C.Strings.Value (Json_C);
      begin
         Free_Buffer (To_Address (Json_C));
         return Result;
      end;
   end From_Binary_Interpretation_Requirement_Array;

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

   function To_Binary_Interpretation_Requirement (Msg : Pyramid.Data_Model.Sensors.Types.Interpretation_Requirement) return String is
   begin
      return To_Binary_Interpretation_Requirement (Pyramid.Data_Model.Sensors.Types_Codec.To_Json (Msg));
   end To_Binary_Interpretation_Requirement;

   function From_Binary_Interpretation_Requirement
     (Payload : String; Tag : access Pyramid.Data_Model.Sensors.Types.Interpretation_Requirement) return Pyramid.Data_Model.Sensors.Types.Interpretation_Requirement
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Interpretation_Requirement (Payload);
      Result : Pyramid.Data_Model.Sensors.Types.Interpretation_Requirement;
   begin
      Result := Pyramid.Data_Model.Sensors.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Interpretation_Requirement;

   function To_Binary_Object_Evidence_Provision_Requirement (Msg : Pyramid.Data_Model.Sensors.Types.Object_Evidence_Provision_Requirement) return String is
   begin
      return To_Binary_Object_Evidence_Provision_Requirement (Pyramid.Data_Model.Sensors.Types_Codec.To_Json (Msg));
   end To_Binary_Object_Evidence_Provision_Requirement;

   function From_Binary_Object_Evidence_Provision_Requirement
     (Payload : String; Tag : access Pyramid.Data_Model.Sensors.Types.Object_Evidence_Provision_Requirement) return Pyramid.Data_Model.Sensors.Types.Object_Evidence_Provision_Requirement
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Object_Evidence_Provision_Requirement (Payload);
      Result : Pyramid.Data_Model.Sensors.Types.Object_Evidence_Provision_Requirement;
   begin
      Result := Pyramid.Data_Model.Sensors.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Object_Evidence_Provision_Requirement;

   function To_Binary_Object_Aquisition_Requirement (Msg : Pyramid.Data_Model.Sensors.Types.Object_Aquisition_Requirement) return String is
   begin
      return To_Binary_Object_Aquisition_Requirement (Pyramid.Data_Model.Sensors.Types_Codec.To_Json (Msg));
   end To_Binary_Object_Aquisition_Requirement;

   function From_Binary_Object_Aquisition_Requirement
     (Payload : String; Tag : access Pyramid.Data_Model.Sensors.Types.Object_Aquisition_Requirement) return Pyramid.Data_Model.Sensors.Types.Object_Aquisition_Requirement
   is
      pragma Unreferenced (Tag);
      Json   : constant String := From_Binary_Object_Aquisition_Requirement (Payload);
      Result : Pyramid.Data_Model.Sensors.Types.Object_Aquisition_Requirement;
   begin
      Result := Pyramid.Data_Model.Sensors.Types_Codec.From_Json (Json, null);
      return Result;
   end From_Binary_Object_Aquisition_Requirement;

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
      return To_Binary_Identifier (String'(Write (Create (UTF8_String'(Ada.Strings.Unbounded.To_String (Msg))))));
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

end Pyramid.Services.Sensor_Data_Interpretation.Flatbuffers_Codec;
