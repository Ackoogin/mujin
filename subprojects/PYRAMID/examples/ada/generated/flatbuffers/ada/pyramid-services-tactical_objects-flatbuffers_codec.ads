--  Auto-generated service FlatBuffers codec
--  Backend: flatbuffers
--  Generated from proto service closure for pyramid.components.tactical_objects.services

with Pyramid.Data_Model.Base.Types;
with Pyramid.Data_Model.Common.Types;
with Pyramid.Data_Model.Tactical.Types;

package Pyramid.Services.Tactical_Objects.Flatbuffers_Codec is
   Content_Type : constant String := "application/flatbuffers";

   function To_Binary_Geodetic_Position (Json : String) return String;
   function To_Binary_Geodetic_Position (Msg : Pyramid.Data_Model.Common.Types.Geodetic_Position) return String;
   function From_Binary_Geodetic_Position (Payload : String) return String;

   function From_Binary_Geodetic_Position
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Geodetic_Position) return Pyramid.Data_Model.Common.Types.Geodetic_Position;

   function To_Binary_Poly_Area (Json : String) return String;
   function To_Binary_Poly_Area (Msg : Pyramid.Data_Model.Common.Types.Poly_Area) return String;
   function From_Binary_Poly_Area (Payload : String) return String;

   function From_Binary_Poly_Area
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Poly_Area) return Pyramid.Data_Model.Common.Types.Poly_Area;

   function To_Binary_Achievement (Json : String) return String;
   function To_Binary_Achievement (Msg : Pyramid.Data_Model.Common.Types.Achievement) return String;
   function From_Binary_Achievement (Payload : String) return String;

   function From_Binary_Achievement
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Achievement) return Pyramid.Data_Model.Common.Types.Achievement;

   function To_Binary_Entity (Json : String) return String;
   function To_Binary_Entity (Msg : Pyramid.Data_Model.Common.Types.Entity) return String;
   function From_Binary_Entity (Payload : String) return String;

   function From_Binary_Entity
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Entity) return Pyramid.Data_Model.Common.Types.Entity;

   function To_Binary_Circle_Area (Json : String) return String;
   function To_Binary_Circle_Area (Msg : Pyramid.Data_Model.Common.Types.Circle_Area) return String;
   function From_Binary_Circle_Area (Payload : String) return String;

   function From_Binary_Circle_Area
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Circle_Area) return Pyramid.Data_Model.Common.Types.Circle_Area;

   function To_Binary_Point (Json : String) return String;
   function To_Binary_Point (Msg : Pyramid.Data_Model.Common.Types.Point) return String;
   function From_Binary_Point (Payload : String) return String;

   function From_Binary_Point
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Point) return Pyramid.Data_Model.Common.Types.Point;

   function To_Binary_Contraint (Json : String) return String;
   function To_Binary_Contraint (Msg : Pyramid.Data_Model.Common.Types.Contraint) return String;
   function From_Binary_Contraint (Payload : String) return String;

   function From_Binary_Contraint
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Contraint) return Pyramid.Data_Model.Common.Types.Contraint;

   function To_Binary_Ack (Json : String) return String;
   function To_Binary_Ack (Msg : Pyramid.Data_Model.Common.Types.Ack) return String;
   function From_Binary_Ack (Payload : String) return String;

   function From_Binary_Ack
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Ack) return Pyramid.Data_Model.Common.Types.Ack;

   function To_Binary_Query (Json : String) return String;
   function To_Binary_Query (Msg : Pyramid.Data_Model.Common.Types.Query) return String;
   function From_Binary_Query (Payload : String) return String;

   function From_Binary_Query
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Query) return Pyramid.Data_Model.Common.Types.Query;

   function To_Binary_Object_Detail (Json : String) return String;
   function To_Binary_Object_Detail (Msg : Pyramid.Data_Model.Tactical.Types.Object_Detail) return String;
   function From_Binary_Object_Detail (Payload : String) return String;

   function From_Binary_Object_Detail
     (Payload : String; Tag : access Pyramid.Data_Model.Tactical.Types.Object_Detail) return Pyramid.Data_Model.Tactical.Types.Object_Detail;

   function To_Binary_Object_Evidence_Requirement (Json : String) return String;
   function To_Binary_Object_Evidence_Requirement (Msg : Pyramid.Data_Model.Tactical.Types.Object_Evidence_Requirement) return String;
   function From_Binary_Object_Evidence_Requirement (Payload : String) return String;

   function From_Binary_Object_Evidence_Requirement
     (Payload : String; Tag : access Pyramid.Data_Model.Tactical.Types.Object_Evidence_Requirement) return Pyramid.Data_Model.Tactical.Types.Object_Evidence_Requirement;

   function To_Binary_Object_Interest_Requirement (Json : String) return String;
   function To_Binary_Object_Interest_Requirement (Msg : Pyramid.Data_Model.Tactical.Types.Object_Interest_Requirement) return String;
   function From_Binary_Object_Interest_Requirement (Payload : String) return String;

   function From_Binary_Object_Interest_Requirement
     (Payload : String; Tag : access Pyramid.Data_Model.Tactical.Types.Object_Interest_Requirement) return Pyramid.Data_Model.Tactical.Types.Object_Interest_Requirement;

   function To_Binary_Object_Match (Json : String) return String;
   function To_Binary_Object_Match (Msg : Pyramid.Data_Model.Tactical.Types.Object_Match) return String;
   function From_Binary_Object_Match (Payload : String) return String;

   function From_Binary_Object_Match
     (Payload : String; Tag : access Pyramid.Data_Model.Tactical.Types.Object_Match) return Pyramid.Data_Model.Tactical.Types.Object_Match;

   function To_Binary_Capability (Json : String) return String;
   function To_Binary_Capability (Msg : Pyramid.Data_Model.Common.Types.Capability) return String;
   function From_Binary_Capability (Payload : String) return String;

   function From_Binary_Capability
     (Payload : String; Tag : access Pyramid.Data_Model.Common.Types.Capability) return Pyramid.Data_Model.Common.Types.Capability;

   function To_Binary_Identifier (Json : String) return String;
   function To_Binary_Identifier (Msg : Pyramid.Data_Model.Base.Types.Identifier) return String;
   function From_Binary_Identifier (Payload : String) return String;

   function From_Binary_Identifier
     (Payload : String; Tag : access Pyramid.Data_Model.Base.Types.Identifier) return Pyramid.Data_Model.Base.Types.Identifier;

   function To_Binary_Object_Detail_Array (Json : String) return String;
   function From_Binary_Object_Detail_Array (Payload : String) return String;

   function To_Binary_Object_Evidence_Requirement_Array (Json : String) return String;
   function From_Binary_Object_Evidence_Requirement_Array (Payload : String) return String;

   function To_Binary_Capability_Array (Json : String) return String;
   function From_Binary_Capability_Array (Payload : String) return String;

   function To_Binary_Object_Match_Array (Json : String) return String;
   function From_Binary_Object_Match_Array (Payload : String) return String;

   function To_Binary_Object_Interest_Requirement_Array (Json : String) return String;
   function From_Binary_Object_Interest_Requirement_Array (Payload : String) return String;

end Pyramid.Services.Tactical_Objects.Flatbuffers_Codec;
