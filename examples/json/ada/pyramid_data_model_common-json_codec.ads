--  Auto-generated JSON codec — do not edit
--  Backend: json | Package: Pyramid.Data_model.Common.Json_Codec

with Ada.Strings.Unbounded; use Ada.Strings.Unbounded;
with GNATCOLL.JSON;

package Pyramid.Data_model.Common.Json_Codec is

   type Feasibility is
     (Unspecified,
      Feasible,
      Not_Feasible,
      Partially_Feasible,
      Pending);

   function To_String (V : Feasibility) return String;
   function From_String (S : String) return Feasibility;

   type Progress is
     (Unspecified,
      Not_Started,
      In_Progress,
      Completed,
      Cancelled,
      Failed);

   function To_String (V : Progress) return String;
   function From_String (S : String) return Progress;

   type Standard_Identity is
     (Unspecified,
      Unknown,
      Friendly,
      Hostile,
      Suspect,
      Neutral,
      Pending,
      Joker,
      Faker,
      Assumed_Friendly);

   function To_String (V : Standard_Identity) return String;
   function From_String (S : String) return Standard_Identity;

   type Battle_Dimension is
     (Unspecified,
      Ground,
      Subsurface,
      Sea_Surface,
      Air,
      Unknown);

   function To_String (V : Battle_Dimension) return String;
   function From_String (S : String) return Battle_Dimension;

   type Data_Policy is
     (Unspecified,
      Query,
      Obtain);

   function To_String (V : Data_Policy) return String;
   function From_String (S : String) return Data_Policy;

   type Geodetic_Position is record
      Latitude : Angle;
      Longitude : Angle;
   end record;

   type Poly_Area is record
      Points : Geodetic_Position;
   end record;

   type Achievement is record
      Base : Entity;
      Status : Progress;
      Quality : Percentage;
      Achieveability : Feasibility;
   end record;

   type Requirement is record
      Base : Entity;
      Status : Achievement;
   end record;

   type Capability is record
      Base : Entity;
      Availability : Boolean;
      Name : Unbounded_String;
      Contraint : Contraint;
   end record;

   type Entity is record
      Update_Time : Timestamp;
      Id : Identifier;
      Source : Identifier;
   end record;

   type Circle_Area is record
      Position : Geodetic_Position;
      Radius : Length;
   end record;

   type Point is record
      Position : Geodetic_Position;
   end record;

   type Contraint is record
      Name : Unbounded_String;
      Value : Integer;
   end record;

   type Ack is record
      Success : Boolean;
   end record;

   type Query is record
      Id : Identifier;
      One_Shot : Boolean;
   end record;

   function To_Json (Msg : Geodetic_Position) return String;
   function From_Json (S : String) return Geodetic_Position;

   function To_Json (Msg : Poly_Area) return String;
   function From_Json (S : String) return Poly_Area;

   function To_Json (Msg : Achievement) return String;
   function From_Json (S : String) return Achievement;

   function To_Json (Msg : Requirement) return String;
   function From_Json (S : String) return Requirement;

   function To_Json (Msg : Capability) return String;
   function From_Json (S : String) return Capability;

   function To_Json (Msg : Entity) return String;
   function From_Json (S : String) return Entity;

   function To_Json (Msg : Circle_Area) return String;
   function From_Json (S : String) return Circle_Area;

   function To_Json (Msg : Point) return String;
   function From_Json (S : String) return Point;

   function To_Json (Msg : Contraint) return String;
   function From_Json (S : String) return Contraint;

   function To_Json (Msg : Ack) return String;
   function From_Json (S : String) return Ack;

   function To_Json (Msg : Query) return String;
   function From_Json (S : String) return Query;

end Pyramid.Data_model.Common.Json_Codec;
