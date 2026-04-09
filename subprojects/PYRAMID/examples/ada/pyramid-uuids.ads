with Ada.Strings.Unbounded;
with Interfaces;
with Pyramid_Data_Model_Base_Types;

package Pyramid.UUIDs is

   Invalid_UUID : exception;

   subtype Identifier is Pyramid_Data_Model_Base_Types.Identifier;

   subtype Byte is Interfaces.Unsigned_8;
   type Byte_Array is array (Natural range <>) of Byte;

   type UUID is private;

   Null_UUID : constant UUID;

   function Is_Null (Value : UUID) return Boolean;

   function To_String (Value : UUID) return String;
   function To_Identifier (Value : UUID) return Identifier;

   function To_Bytes (Value : UUID) return Byte_Array;
   function From_Bytes (Value : Byte_Array) return UUID;

   function Parse (Value : String) return UUID;
   function Parse (Value : Identifier) return UUID;

   function Try_Parse
     (Value  : String;
      Result : out UUID) return Boolean;

   function Try_Parse
     (Value  : Identifier;
      Result : out UUID) return Boolean;

   function Is_Valid (Value : String) return Boolean;
   function Is_Valid (Value : Identifier) return Boolean;

   function Generate_V4 return UUID;
   function Generate_V4_Identifier return Identifier;

   function Generate_V5
     (Namespace_UUID : UUID;
      Name           : String) return UUID;

   function Generate_V5
     (Namespace_ID : Identifier;
      Name         : String) return Identifier;

private
   type UUID is record
      Bytes : Byte_Array (0 .. 15) := (others => 0);
   end record;

   Null_UUID : constant UUID := (Bytes => (others => 0));
end Pyramid.UUIDs;
