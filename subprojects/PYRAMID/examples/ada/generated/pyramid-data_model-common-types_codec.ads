--  Auto-generated data model JSON codec specification
--  Generated from: common.proto by generate_bindings.py (codec)
--  Package: Pyramid.Data_Model.Common.Types_Codec

with Ada.Strings.Unbounded;  use Ada.Strings.Unbounded;
with Pyramid.Data_Model.Common.Types;  use Pyramid.Data_Model.Common.Types;

package Pyramid.Data_Model.Common.Types_Codec is

   function To_String (V : Feasibility) return String;
   function Feasibility_From_String (S : String) return Feasibility;
   function To_String (V : Progress) return String;
   function Progress_From_String (S : String) return Progress;
   function To_String (V : Standard_Identity) return String;
   function Standard_Identity_From_String (S : String) return Standard_Identity;
   function To_String (V : Battle_Dimension) return String;
   function Battle_Dimension_From_String (S : String) return Battle_Dimension;
   function To_String (V : Data_Policy) return String;
   function Data_Policy_From_String (S : String) return Data_Policy;

   function To_Json (Msg : Geodetic_Position) return String;
   function From_Json (S : String; Tag : access Geodetic_Position) return Geodetic_Position;
   function To_Json (Msg : Poly_Area) return String;
   function From_Json (S : String; Tag : access Poly_Area) return Poly_Area;
   function To_Json (Msg : Achievement) return String;
   function From_Json (S : String; Tag : access Achievement) return Achievement;
   function To_Json (Msg : Requirement) return String;
   function From_Json (S : String; Tag : access Requirement) return Requirement;
   function To_Json (Msg : Capability) return String;
   function From_Json (S : String; Tag : access Capability) return Capability;
   function To_Json (Msg : Entity) return String;
   function From_Json (S : String; Tag : access Entity) return Entity;
   function To_Json (Msg : Circle_Area) return String;
   function From_Json (S : String; Tag : access Circle_Area) return Circle_Area;
   function To_Json (Msg : Point) return String;
   function From_Json (S : String; Tag : access Point) return Point;
   function To_Json (Msg : Contraint) return String;
   function From_Json (S : String; Tag : access Contraint) return Contraint;
   function To_Json (Msg : Ack) return String;
   function From_Json (S : String; Tag : access Ack) return Ack;
   function To_Json (Msg : Query) return String;
   function From_Json (S : String; Tag : access Query) return Query;

end Pyramid.Data_Model.Common.Types_Codec;
