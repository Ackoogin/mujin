--  Auto-generated JSON codec — do not edit
--  Backend: json | Package: Pyramid.Data_model.Base.Json_Codec

with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with GNATCOLL.JSON;

package Pyramid.Data_model.Base.Json_Codec is

   type Angle is record
      Radians : Long_Float;
   end record;

   type Length is record
      Meters : Long_Float;
   end record;

   type Timestamp is record
      Value : Timestamp;
   end record;

   type Identifier is record
      Value : Unbounded_String;
   end record;

   type Speed is record
      Meters_Per_Second : Long_Float;
   end record;

   type Percentage is record
      Value : Long_Float;
   end record;

   function To_Json (Msg : Angle) return String;
   function From_Json (S : String) return Angle;

   function To_Json (Msg : Length) return String;
   function From_Json (S : String) return Length;

   function To_Json (Msg : Timestamp) return String;
   function From_Json (S : String) return Timestamp;

   function To_Json (Msg : Identifier) return String;
   function From_Json (S : String) return Identifier;

   function To_Json (Msg : Speed) return String;
   function From_Json (S : String) return Speed;

   function To_Json (Msg : Percentage) return String;
   function From_Json (S : String) return Percentage;

end Pyramid.Data_model.Base.Json_Codec;
