--  Auto-generated JSON codec — do not edit
--  Backend: json | Package: Pyramid.Data_model.Tactical.Json_Codec

with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with GNATCOLL.JSON;

package Pyramid.Data_model.Tactical.Json_Codec is

   type Object_Source is
     (Unspecified,
      Radar,
      Local);

   function To_String (V : Object_Source) return String;
   function From_String (S : String) return Object_Source;

   type Object_Detail is record
      Base : Entity;
      Source : Object_Source;
      Position : Geodetic_Position;
      Creation_Time : Timestamp;
      Quality : Percentage;
      Course : Angle;
      Speed : Speed;
      Length : Length;
      Identity : Standard_Identity;
      Dimension : Battle_Dimension;
   end record;

   type Object_Evidence_Requirement is record
      Base : Requirement;
      Policy : Data_Policy;
      Dimension : Battle_Dimension;
      Poly_Area : Poly_Area;
      Circle_Area : Circle_Area;
      Point : Point;
   end record;

   type Object_Interest_Requirement is record
      Base : Requirement;
      Source : Object_Source;
      Policy : Data_Policy;
      Dimension : Battle_Dimension;
      Poly_Area : Poly_Area;
      Circle_Area : Circle_Area;
      Point : Point;
   end record;

   type Object_Match is record
      Base : Entity;
      Matching_Object_Id : Identifier;
   end record;

   function To_Json (Msg : Object_Detail) return String;
   function From_Json (S : String) return Object_Detail;

   function To_Json (Msg : Object_Evidence_Requirement) return String;
   function From_Json (S : String) return Object_Evidence_Requirement;

   function To_Json (Msg : Object_Interest_Requirement) return String;
   function From_Json (S : String) return Object_Interest_Requirement;

   function To_Json (Msg : Object_Match) return String;
   function From_Json (S : String) return Object_Match;

end Pyramid.Data_model.Tactical.Json_Codec;
