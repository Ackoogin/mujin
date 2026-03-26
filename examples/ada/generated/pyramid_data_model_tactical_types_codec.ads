--  Auto-generated data model JSON codec specification
--  Generated from: tactical.proto by ada_service_generator.py --codec
--  Package: Pyramid_Data_Model_Tactical_Types_Codec

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid_Data_Model_Tactical_Types;  use Pyramid_Data_Model_Tactical_Types;

package Pyramid_Data_Model_Tactical_Types_Codec is

   function To_String (V : Object_Source) return String;
   function Object_Source_From_String (S : String) return Object_Source;

   function To_Json (Msg : Object_Detail) return String;
   function From_Json (S : String; Tag : access Object_Detail) return Object_Detail;
   function To_Json (Msg : Object_Evidence_Requirement) return String;
   function From_Json (S : String; Tag : access Object_Evidence_Requirement) return Object_Evidence_Requirement;
   function To_Json (Msg : Object_Interest_Requirement) return String;
   function From_Json (S : String; Tag : access Object_Interest_Requirement) return Object_Interest_Requirement;
   function To_Json (Msg : Object_Match) return String;
   function From_Json (S : String; Tag : access Object_Match) return Object_Match;

end Pyramid_Data_Model_Tactical_Types_Codec;
